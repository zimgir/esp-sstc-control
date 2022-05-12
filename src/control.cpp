#include <Arduino.h>

#include <atomic>

#include "MovingAverage.h"

#include "config.h"
#include "utils.h"
#include "ledc.h"
#include "adc.h"
#include "timer.h"
#include "sampler.h"
#include "control.h"

static TaskHandle_t DRAM_ATTR __control_task;
static volatile std::atomic<uint32_t> DRAM_ATTR __pwrfrq;
static volatile std::atomic<uint32_t> DRAM_ATTR __out_flag;
static volatile std::atomic<control_mode_t> DRAM_ATTR __set_mode;
static volatile std::atomic<control_mode_t> DRAM_ATTR __cur_mode;

static MovingAverage<float, CONTROL_MOVING_AVERAGE_SIZE> DRAM_ATTR __avg_energy;
static MovingAverage<float, CONTROL_MOVING_AVERAGE_SIZE> DRAM_ATTR __avg_standout;

static uint32_t __step_count;
static uint32_t __step_freq[CONTROL_CONFIDENCE_STEP_COUNT] = {0};

static uint32_t __get_power_knob_val()
{
    // TODO
    return 0;
}

static bool IRAM_ATTR ISR_timer_control(void *arg)
{
    uint16_t sample;
    BaseType_t task_awake = pdFALSE;

#ifdef CONTROL_CHECK_TIMER
    static int state = 0;
    if (state)
        GPIO_SET0(GPIO_DEBUG_OUT);
    else
        GPIO_CLR0(GPIO_DEBUG_OUT);
    state = ~state;
#endif

    // signal output event
    __out_flag.store(1);

    // wake up task for PWM update
    vTaskNotifyGiveFromISR(__control_task, &task_awake);

    return task_awake == pdTRUE;
}

// executed by the ther core in sampler task context
// time sensitive - must finish before next fft buffer is filled by sampler
static void __fft_callback(struct fft_analysis_t *analysis)
{
    int i;
    float standout;
    float energy_step;
    float standout_step;

    uint32_t cur_pwrfrq;
    uint32_t new_pwrfrq;
    standout = analysis->max_mag - analysis->avg_mag;

    // handle initial case
    if (unlikely(__avg_energy.num() == 0))
    {
        BUG(__avg_standout.num() != 0);
        __avg_energy.add(analysis->energy);
        __avg_standout.add(standout);
    }

    // __avg_energy.value() != 0 (unexpected bug othewise)
    energy_step = analysis->energy / __avg_energy.value();

    // __avg_standout.value() != 0 (unexpected bug othewise)
    standout_step = standout / __avg_standout.value();

    cur_pwrfrq = __pwrfrq.load();
    new_pwrfrq = cur_pwrfrq;

    if (cur_pwrfrq != 0)
    { // ON state
        if (energy_step <= CONTROL_OFF_STEP_ENERGY && standout_step <= CONTROL_OFF_STEP_STANDOUT)
        { // OFF condition immediately without confidence steps

            __avg_energy.reset();
            __avg_standout.reset();

            new_pwrfrq = 0;
        }
        else
        { // update frequency with confidence steps

            if (__step_freq_persistent((uint32_t)analysis->max_est_freq))
            {
                if (__step_count == CONTROL_CONFIDENCE_STEP_COUNT)
                {
                    new_pwrfrq = PWRFRQ_VAL(__get_power_knob_val(), (uint32_t)analysis->max_est_freq);

                    __step_count = 0;
                } 
                else
                {
                    __step_freq[__step_count++] = (uint32_t)analysis->max_est_freq;
                }
            }
            else
            {
                __step_count = 0;
            }
        }

        __avg_energy.add(analysis->energy);
        __avg_standout.add(standout);
    }
    else
    { // OFF state
        if (energy_step >= CONTROL_ON_STEP_ENERGY && standout_step >= CONTROL_ON_STEP_STANDOUT)
        { // ON condition with confidence steps

            if (__step_freq_persistent((uint32_t)analysis->max_est_freq))
            {
                if (__step_count == CONTROL_CONFIDENCE_STEP_COUNT)
                {
                    __avg_energy.reset();
                    __avg_standout.reset();

                    new_pwrfrq = PWRFRQ_VAL(__get_power_knob_val(), (uint32_t)analysis->max_est_freq);

                    __step_count = 0;
                } 
                else
                {
                    __step_freq[__step_count++] = (uint32_t)analysis->max_est_freq;
                }
            }
            else
            {
                __step_count = 0;
            }
        }

        __avg_energy.add(analysis->energy);
        __avg_standout.add(standout);
    }

    if (new_pwrfrq != cur_pwrfrq)
    {
        LOG("\n==========================================================\n");
        LOG("max_mag: %f\n", analysis->max_mag);
        LOG("energy: %f\n", analysis->energy);
        LOG("max_mag: %f\n", analysis->max_mag);
        LOG("avg_mag: %f\n", analysis->avg_mag);
        LOG("standout: %f\n", standout);
        LOG("energy_step: %f\n", energy_step);
        LOG("standout_step: %f\n", standout_step);
        LOG("avg energy: %f\n", __avg_energy.value());
        LOG("avg standout: %f\n", __avg_standout.value());
        LOG("new_frq: %d\n", PWRFRQ_FRQ(new_pwrfrq));
        LOG("\n==========================================================\n");
    }

    __pwrfrq.store(new_pwrfrq);

    xTaskNotifyGive(__control_task);
}

static void __control_output(control_mode_t mode)
{
    switch (mode)
    {
    case CTRL_MODE_KNOBS:
    {
        // sample knobs and update PWM
        break;
    }
    case CTRL_MODE_AUDIO:
    {
        // update PWM according to sampled values from sampler
        break;
    }
    default:
    {
        // this case should not happen but stop output just in case it does
        ESP_ERROR_CHECK(timer_pause(TIMER_GRP_CONTROL, TIMER_IDX_CONTROL));
        sampler_stop();
        break;
    }
    }
}

control_mode_t control_get_mode(void)
{
    return __cur_mode.load();
}

void control_set_mode(control_mode_t mode)
{
    __set_mode.store(mode);
    xTaskNotifyGive(__control_task);
}

static void __control_task_init(void)
{
    timer_init_t timer_init_data = {
        .config = {
            .alarm_en = TIMER_ALARM_EN,
            .counter_en = TIMER_PAUSE,
            .intr_type = TIMER_INTR_LEVEL,
            .counter_dir = TIMER_COUNT_UP,
            .auto_reload = TIMER_AUTORELOAD_EN,
            .divider = TIMER_DIVIDER(CONTROL_TIMER_FREQ),
        },
        .count = 0,
        .alarm = TIMER_ALARM_COUNT(CONTROL_TIMER_FREQ, CONTROL_KNOB_UPDATE_FREQ),
        .isr_func = ISR_timer_control,
        .isr_arg = NULL,
    };

    timer_init_simple(TIMER_GRP_CONTROL, TIMER_IDX_CONTROL, &timer_init_data);

    adc1_init_channel(ADC_CH_KNOB1, ADC_ATTEN_KNOB);
    adc1_init_channel(ADC_CH_KNOB2, ADC_ATTEN_KNOB);

    sampler_init(__fft_callback);
}

static void __control_stop(control_mode_t mode)
{

    switch (mode)
    {
    case CTRL_MODE_KNOBS:
    {
        ESP_ERROR_CHECK(timer_pause(TIMER_GRP_CONTROL, TIMER_IDX_CONTROL));
        break;
    }
    case CTRL_MODE_AUDIO:
    {
        sampler_stop();
        break;
    }
    default:
    {
        break;
    }
    }
}

static control_mode_t __control_start(control_mode_t mode)
{
    switch (mode)
    {
    case CTRL_MODE_KNOBS:
    {
        ESP_ERROR_CHECK(timer_start(TIMER_GRP_CONTROL, TIMER_IDX_CONTROL));
        return mode;
    }
    case CTRL_MODE_AUDIO:
    {
        sampler_start();
        return mode;
    }
    default:
    {
        return CTRL_MODE_STOP;
    }
    }
}

static void TASK_control_main(void *param)
{
    uint32_t control_count;
    control_mode_t old_mode, new_mode;

    __control_task_init();

    while (1)
    {
        // sleep until notified or for 1 sec
        PROFILE_OP(control_count = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1000)));

        if (control_count == 0)
        { // wait timeout no actual notifications
            continue;
        }

        old_mode = __cur_mode.load();

        if (__out_flag.load())
        {
            __control_output(old_mode);
            __out_flag.store(0);
        }

        new_mode = __set_mode.load();

        if (new_mode != old_mode)
        {
            __control_stop(old_mode);
            new_mode = __control_start(new_mode);
            LOG("Start control mode: %d\n", new_mode);
            __cur_mode.store(new_mode);
            __set_mode.store(new_mode);
        }
    }
}

void control_init(void)
{
    BaseType_t ret;

    __pwrfrq.store(0);
    __out_flag.store(0);
    __set_mode.store(CTRL_MODE_STOP);
    __cur_mode.store(CTRL_MODE_STOP);

    __step_count = 0;

    ret = xTaskCreatePinnedToCore(TASK_control_main, "CONTROL", CONTROL_TASK_STACK_SIZE, NULL, CONTROL_TASK_PRIORITY, &__control_task, CONTROL_TASK_CORE);

    BUG(ret != pdPASS);
}
