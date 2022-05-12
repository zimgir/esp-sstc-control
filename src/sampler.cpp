#include <Arduino.h>

#include <math.h>

// unfortunately stdatomic.h does not work when using arduino framework 
// so have to use the c++ version :(
// it does work in esp-idf framework though
#include <atomic> 

#include <fft.h>

#include "config.h"
#include "utils.h"
#include "gpio.h"
#include "adc.h"
#include "timer.h"
#include "sampler.h"

static uint32_t DRAM_ATTR __sample_index;
static uint16_t DMA_ATTR __sample_buffer[SAMPLER_FFT_SIZE * 2]; // double buffer scheme

static float DMA_ATTR __fft_input_buffer[SAMPLER_FFT_SIZE];  // real FFT
static float DMA_ATTR __fft_output_buffer[SAMPLER_FFT_SIZE]; // real FFT

static fft_config_t DRAM_ATTR *__fft_config;
static fft_callback_t __fft_callback;

static TaskHandle_t DRAM_ATTR __sampler_task;

static volatile std::atomic<uint32_t> DRAM_ATTR __enable_flag;
static volatile std::atomic<uint32_t> DRAM_ATTR __proc_flag;

#if (SAMPLER_FFT_SIZE == 256)
static DMA_ATTR float __fft_hanning_buffer[SAMPLER_FFT_SIZE / 2] = // half buffer because of window symmetry

    {0.00000, 0.00015, 0.00061, 0.00137, 0.00243, 0.00379, 0.00545, 0.00742,
     0.00968, 0.01224, 0.01510, 0.01825, 0.02170, 0.02543, 0.02946, 0.03376,
     0.03836, 0.04323, 0.04838, 0.05380, 0.05949, 0.06546, 0.07168, 0.07817,
     0.08491, 0.09190, 0.09914, 0.10663, 0.11435, 0.12231, 0.13050, 0.13891,
     0.14754, 0.15638, 0.16543, 0.17469, 0.18414, 0.19379, 0.20362, 0.21363,
     0.22382, 0.23417, 0.24468, 0.25535, 0.26617, 0.27713, 0.28823, 0.29945,
     0.31079, 0.32225, 0.33382, 0.34549, 0.35725, 0.36910, 0.38103, 0.39303,
     0.40510, 0.41722, 0.42940, 0.44161, 0.45387, 0.46615, 0.47845, 0.49076,
     0.50308, 0.51540, 0.52771, 0.54000, 0.55226, 0.56450, 0.57670, 0.58885,
     0.60094, 0.61298, 0.62494, 0.63683, 0.64864, 0.66036, 0.67197, 0.68349,
     0.69489, 0.70618, 0.71734, 0.72837, 0.73926, 0.75000, 0.76059, 0.77103,
     0.78130, 0.79140, 0.80132, 0.81106, 0.82061, 0.82996, 0.83912, 0.84807,
     0.85681, 0.86533, 0.87363, 0.88170, 0.88954, 0.89714, 0.90451, 0.91163,
     0.91849, 0.92511, 0.93146, 0.93756, 0.94339, 0.94895, 0.95423, 0.95924,
     0.96398, 0.96843, 0.97259, 0.97647, 0.98006, 0.98336, 0.98636, 0.98907,
     0.99149, 0.99360, 0.99542, 0.99693, 0.99814, 0.99905, 0.99966, 0.99996};
#endif

static bool IRAM_ATTR ISR_timer_sample(void *arg)
{
    uint16_t sample;
    BaseType_t task_awake = pdFALSE;

#ifdef SAMPLER_CHECK_TIMER
    static int state = 0;
    if (state)
        GPIO_SET0(GPIO_DEBUG_OUT);
    else
        GPIO_CLR0(GPIO_DEBUG_OUT);
    state = ~state;
#endif

    sample = adc1_fast_sample(ADC_CH_SAMPLER);

    // it seems ISR can't handle float buffer and crashes if it is float
    __sample_buffer[__sample_index] = sample;
    __sample_index = (__sample_index + 1) % (sizeof(__sample_buffer) / sizeof(__sample_buffer[0]));

    // double buffer switch event
    if (__sample_index == SAMPLER_FFT_SIZE || __sample_index == 0)
    {
        __proc_flag.store(1);

        vTaskNotifyGiveFromISR(__sampler_task, &task_awake);    
    }

    return task_awake == pdTRUE;
}

static void __fft_process(fft_analysis_t *analysis)
{
    int i;
    int idx;
    uint16_t sample;
    int max_mag_idx;
    float mag_squared;
    float delta;

    // get double buffer start index - take the buffer that is ready for fft and not currently sampled
    idx = __sample_index >= SAMPLER_FFT_SIZE ? 0 : SAMPLER_FFT_SIZE;

    // copy samples as float and calculate min max and average samples
    analysis->min_smpl = __sample_buffer[idx];
    analysis->max_smpl = __sample_buffer[idx];
    analysis->avg_smpl = 0;

    for (i = 0; i < SAMPLER_FFT_SIZE; ++i)
    {
        sample = __sample_buffer[idx + i];

        __fft_input_buffer[i] = (float)sample;

        if (sample < analysis->min_smpl)
            analysis->min_smpl = sample;

        if (sample > analysis->max_smpl)
            analysis->max_smpl = sample;

        analysis->avg_smpl += __fft_input_buffer[i];
    }

    analysis->avg_smpl /= SAMPLER_FFT_SIZE;

    // remove DC and apply window function in 2 opposing half iterations

    for (i = 0; i < (SAMPLER_FFT_SIZE / 2); ++i)
    {
        __fft_input_buffer[i] = (__fft_input_buffer[i] - analysis->avg_smpl) * __fft_hanning_buffer[i];
    }

    for (i = 0; i < (SAMPLER_FFT_SIZE / 2); ++i)
    {
        idx = SAMPLER_FFT_SIZE - 1 - i;
        __fft_input_buffer[idx] = (__fft_input_buffer[idx] - analysis->avg_smpl) * __fft_hanning_buffer[i];
    }

    // run fft

    fft_execute(__fft_config);

    //  Output : [ X[0], X[NFFT/2], Re(X[1]), Im(X[1]), ..., Re(X[NFFT/2-1]), Im(X[NFFT/2-1]) ]

    // start from 2 to skip DC values at 0 and NFFT/2

    // convert real vals to magnitude and calculate energy

    analysis->energy = 0;

    mag_squared = sq(__fft_output_buffer[0]);
    analysis->energy += mag_squared;

    for (i = 2; i < SAMPLER_FFT_SIZE; i += 2)
    {
        mag_squared = sq(__fft_output_buffer[i]) + sq(__fft_output_buffer[i + 1]);
        analysis->energy += mag_squared;
        __fft_output_buffer[i] = sqrt(mag_squared);
    }

    // normalize energy value
    analysis->energy = analysis->energy / SAMPLER_FFT_SIZE;

    // find max (magnitude, frequency) peak between edge values

    analysis->min_mag = __fft_output_buffer[2];
    analysis->max_mag = __fft_output_buffer[2];
    analysis->avg_mag = 0;
    max_mag_idx = 0;

    for (i = 2; i < (SAMPLER_FFT_SIZE - 2); i += 2)
    {
        if (__fft_output_buffer[i] > analysis->max_mag)
        {
            analysis->max_mag = __fft_output_buffer[i];
            max_mag_idx = i / 2;
        }

        if (__fft_output_buffer[i] < analysis->min_mag)
        {
            analysis->min_mag = __fft_output_buffer[i];
        }

        analysis->avg_mag += __fft_output_buffer[i];
    }

    analysis->avg_mag /= (SAMPLER_FFT_SIZE - 2);

    idx = max_mag_idx * 2;

    analysis->max_bin_freq = max_mag_idx * SAMPLER_SAMPLE_FREQ / SAMPLER_FFT_SIZE;

    // frequecy peak estimation math magic
    delta = 0.5 * ((__fft_output_buffer[idx - 2] - __fft_output_buffer[idx + 2]) /
                   (__fft_output_buffer[idx - 2] - (2.0 * __fft_output_buffer[idx]) + __fft_output_buffer[idx + 2]));

    analysis->max_est_freq = ((max_mag_idx + delta) * SAMPLER_SAMPLE_FREQ) / SAMPLER_FFT_SIZE;
}

static void __sampler_task_init(void)
{
    timer_init_t timer_init_data = {
        .config = {
            .alarm_en = TIMER_ALARM_EN,
            .counter_en = TIMER_PAUSE,
            .intr_type = TIMER_INTR_LEVEL,
            .counter_dir = TIMER_COUNT_UP,
            .auto_reload = TIMER_AUTORELOAD_EN,
            .divider = TIMER_DIVIDER(SAMPLER_TIMER_FREQ),
        },
        .count = 0,
        .alarm = TIMER_ALARM_COUNT(SAMPLER_TIMER_FREQ, SAMPLER_SAMPLE_FREQ),
        .isr_func = ISR_timer_sample,
        .isr_arg = NULL,
    };

    timer_init_simple(TIMER_GRP_SAMPLER, TIMER_IDX_SAMPLER, &timer_init_data);

    adc1_init_channel(ADC_CH_SAMPLER, ADC_ATTEN_SAMPLER);

    __fft_config = fft_init(SAMPLER_FFT_SIZE, FFT_REAL, FFT_FORWARD, __fft_input_buffer, __fft_output_buffer);

    BUG(__fft_config == NULL);
}

static void TASK_sampler_main(void *param)
{
    uint32_t sampler_count;
    fft_analysis_t analysis;

    __sampler_task_init();

    while (1)
    {
        // sleep until notified for double buffer switch or for 1 sec
        PROFILE_OP(sampler_count = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1000)));

        if (sampler_count == 0)
        { // wait timeout no actual notifications
            continue; 
        }

        if (__proc_flag.load())
        {
            PROFILE_OP(__fft_process(&analysis));

            PROFILE_OP(__fft_callback(&analysis));

            __proc_flag.store(0);
        }

        if (__enable_flag.load())
        {
            // if timer is not running start it here
            if (!timer_is_counting(TIMER_GRP_SAMPLER, TIMER_IDX_SAMPLER))
                ESP_ERROR_CHECK(timer_start(TIMER_GRP_SAMPLER, TIMER_IDX_SAMPLER));
        }
        else
        {
            ESP_ERROR_CHECK(timer_pause(TIMER_GRP_SAMPLER, TIMER_IDX_SAMPLER));
            __sample_index = 0;
        }
    }
}

void sampler_init(fft_callback_t callback)
{
    BaseType_t ret;

    BUG(callback == NULL);

    __fft_callback = callback;

    __proc_flag.store(0);
    __enable_flag.store(0);
    
    ret = xTaskCreatePinnedToCore(TASK_sampler_main, "SAMPLER", SAMPLER_TASK_STACK_SIZE, NULL, SAMPLER_TASK_PRIORITY, &__sampler_task, SAMPLER_TASK_CORE);

    BUG(ret != pdPASS);
}

void sampler_start(void)
{
    __enable_flag.store(1);
    xTaskNotifyGive(__sampler_task);
}

void sampler_stop(void)
{
    __enable_flag.store(0);
    xTaskNotifyGive(__sampler_task);
}
