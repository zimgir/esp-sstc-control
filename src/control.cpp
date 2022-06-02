#include <Arduino.h>

#include <atomic>

#include "MovingAverage.h"

#include "config.h"
#include "utils.h"
#include "pwm.h"
#include "adc.h"
#include "timer.h"
#include "sampler.h"
#include "control.h"

static TaskHandle_t DRAM_ATTR __control_task;
static volatile std::atomic<uint32_t> DRAM_ATTR __pwrfrq;
static volatile std::atomic<uint32_t> DRAM_ATTR __out_flag;
static volatile std::atomic<control_mode_t> DRAM_ATTR __set_mode;
static volatile std::atomic<control_mode_t> DRAM_ATTR __cur_mode;

static MovingAverage<float, CONTROL_MOVING_AVERAGE_SIZE> DRAM_ATTR __avg_off_energy;
static MovingAverage<float, CONTROL_MOVING_AVERAGE_SIZE> DRAM_ATTR __avg_off_standout;

static MovingAverage<float, CONTROL_MOVING_AVERAGE_SIZE> DRAM_ATTR __avg_on_energy;
static MovingAverage<float, CONTROL_MOVING_AVERAGE_SIZE> DRAM_ATTR __avg_on_standout;

static MovingAverage<uint32_t, CONTROL_MOVING_AVERAGE_SIZE> DRAM_ATTR __avg_power;
static MovingAverage<uint32_t, CONTROL_MOVING_AVERAGE_SIZE> DRAM_ATTR __avg_frequency;


static float DRAM_ATTR __prev_freq;
static uint32_t DRAM_ATTR __step_count;

static HWTimer DRAM_ATTR __timer;
static LEDCPWM DRAM_ATTR __pwm;

static uint32_t __get_power_knob_val()
{
    uint32_t sample;
    sample = adc1_fast_sample(ADC_CH_KNOB1);
#ifdef CONTROL_DEBUG_KNOBS
    LOG("KNOB1: %d ", sample);
#endif
    // normalize to ADC_MAX_VAL acording to physical knob properties
    sample = ADC_MAX_VAL * sample / CONTROL_KNOB1_MAX_VAL;
    return sample;
}

static uint32_t __get_freqency_knob_val()
{
    uint32_t sample;
    sample = adc1_fast_sample(ADC_CH_KNOB2);
#ifdef CONTROL_DEBUG_KNOBS
    LOG("KNOB2: %d ", sample);
#endif
    // normalize to ADC_MAX_VAL acording to physical knob properties
    sample = ADC_MAX_VAL * sample / CONTROL_KNOB2_MAX_VAL;
    return sample;
}

static bool IRAM_ATTR ISR_timer_control(void *arg)
{
    uint32_t power;
    uint32_t frequency;
    BaseType_t task_awake = pdFALSE;

#ifdef CONTROL_CHECK_TIMER
    static int state = 0;
    if (state)
        GPIO_SET0(GPIO_DEBUG_OUT);
    else
        GPIO_CLR0(GPIO_DEBUG_OUT);
    state = ~state;
#endif

    power = __get_power_knob_val();
    frequency = CONTROL_PWM_MAX_FREQ * __get_freqency_knob_val() / ADC_MAX_VAL;

    // smooth out noise with moving average to reduce jitter
    __avg_power.add(power);
    __avg_frequency.add(frequency);

    __pwrfrq.store(PWRFRQ_VAL(__avg_power.value(), __avg_frequency.value()));

    // signal output event
    __out_flag.store(1);

    // wake up task for PWM update
    vTaskNotifyGiveFromISR(__control_task, &task_awake);

    return task_awake == pdTRUE;
}

static int __is_persistent_freq(float new_freq)
{
    int i;

    if (new_freq < (__prev_freq - CONTROL_PERSISTENT_FREQ_LIMIT) ||
        new_freq > (__prev_freq + CONTROL_PERSISTENT_FREQ_LIMIT))
        return 0;

    return 1;
}

// executed by the ther core in sampler task context
// time sensitive - must finish before next fft buffer is filled by sampler
static void __fft_callback(struct fft_analysis_t *analysis)
{
    float standout;
    float energy_step;
    float standout_step;
    float step_score;

    int freq_peresist;

    float on_threshold;
    float off_threshold;

    uint32_t cur_pwrfrq;
    uint32_t new_pwrfrq;

    standout = analysis->max_mag - analysis->avg_mag;

    cur_pwrfrq = __pwrfrq.load();
    new_pwrfrq = cur_pwrfrq;

    if (cur_pwrfrq != 0)
    { // ON state

        if (unlikely(__avg_on_energy.num() == 0))
        {
            BUG(__avg_on_standout.num() != 0);
            __avg_on_energy.add(analysis->energy);
            __avg_on_standout.add(standout);
        }

        // __avg_energy.value() != 0 (unexpected bug othewise)
        energy_step = analysis->energy / __avg_on_energy.value();

        // __avg_standout.value() != 0 (unexpected bug othewise)
        standout_step = standout / __avg_on_standout.value();

        freq_peresist = __is_persistent_freq(analysis->max_est_freq);

        step_score = CONTROL_WEIGHT_ENERGY * energy_step + CONTROL_WEIGHT_STANDOUT * standout_step;

        step_score = freq_peresist ? step_score * CONTROL_MULTILIER_PERSIST : step_score;

        off_threshold = CONTROL_SCORE_OFF_TH_BASE +
                        CONTROL_SCORE_OFF_TH_VAR * ((float)__get_freqency_knob_val() / ADC_MAX_VAL);

        if (step_score < off_threshold)
        { // OFF condition immediately without confidence steps

            new_pwrfrq = 0;
            __avg_off_energy.add(analysis->energy);
            __avg_off_standout.add(standout);
        }
        else
        { // update frequency with confidence steps

            if (freq_peresist)
            {
                if (__step_count >= CONTROL_CONFIDENCE_STEP_COUNT)
                {
                    new_pwrfrq = PWRFRQ_VAL(__get_power_knob_val(), (uint32_t)analysis->max_est_freq);
                }
                else
                {
                    __step_count++;
                }
            }
            else
            {
                // reset and increment
                __step_count = 1;
            }

            __avg_on_energy.add(analysis->energy);
            __avg_on_standout.add(standout);
        }
    }
    else
    { // OFF state

        // handle initial case
        if (unlikely(__avg_off_energy.num() == 0))
        {
            BUG(__avg_off_standout.num() != 0);
            __avg_off_energy.add(analysis->energy);
            __avg_off_standout.add(standout);
        }

        // __avg_energy.value() != 0 (unexpected bug othewise)
        energy_step = analysis->energy / __avg_off_energy.value();

        // __avg_standout.value() != 0 (unexpected bug othewise)
        standout_step = standout / __avg_off_standout.value();

        freq_peresist = __is_persistent_freq((uint32_t)analysis->max_est_freq);

        step_score = CONTROL_WEIGHT_ENERGY * energy_step + CONTROL_WEIGHT_STANDOUT * standout_step;

        step_score = freq_peresist ? step_score * CONTROL_MULTILIER_PERSIST : step_score;

        on_threshold = CONTROL_SCORE_ON_TH_BASE -
                       CONTROL_SCORE_ON_TH_VAR * ((float)__get_freqency_knob_val() / ADC_MAX_VAL);

        if (step_score >= on_threshold)
        { // ON condition with confidence steps

            if (freq_peresist)
            {
                if (__step_count >= CONTROL_CONFIDENCE_STEP_COUNT)
                {
                    new_pwrfrq = PWRFRQ_VAL(__get_power_knob_val(), (uint32_t)analysis->max_est_freq);
                }
                else
                {
                    __step_count++;
                }
            }
            else
            {
                // reset and increment
                __step_count = 1;
            }

            __avg_on_energy.add(analysis->energy);
            __avg_on_standout.add(standout);
        }
        else
        {
            __avg_off_energy.add(analysis->energy);
            __avg_off_standout.add(standout);
        }
    }

#ifdef CONTROL_DEBUG_AUDIO_CTRL
    if (new_pwrfrq != cur_pwrfrq)
    {
        LOG("\n==========================================================\n");
        LOG("energy: %f\n", analysis->energy);
        LOG("max_mag: %f\n", analysis->max_mag);
        LOG("avg_mag: %f\n", analysis->avg_mag);
        LOG("standout: %f\n", standout);
        LOG("energy_step: %f\n", energy_step);
        LOG("standout_step: %f\n", standout_step);
        LOG("step_score: %f\n", step_score);
        LOG("avg_off_energy: %f\n", __avg_off_energy.value());
        LOG("avg_off_standout: %f\n", __avg_off_standout.value());
        LOG("avg_on_energy: %f\n", __avg_on_energy.value());
        LOG("avg_on_standout: %f\n", __avg_on_standout.value());
        LOG("new_frq: %d\n", PWRFRQ_FRQ(new_pwrfrq));
        LOG("\n==========================================================\n");
    }
#endif
    __prev_freq = analysis->max_est_freq;
    __pwrfrq.store(new_pwrfrq);
    __out_flag.store(1);

    xTaskNotifyGive(__control_task);
}

static uint32_t __control_pwm_duty(uint32_t power, uint32_t freq)
{
    uint32_t duty;
    uint32_t max_duty;

    // max duty from max wdith limit
    // MAX_DUTY * (MAX_WIDTH_US / MAX_PERIOD_US)
    // CONTROL_PWM_MAX_DUTY * CONTROL_PWM_MAX_WIDTH_US / (1000000 / freq)

    max_duty = freq * (CONTROL_PWM_MAX_DUTY * CONTROL_PWM_MAX_WIDTH_US / 1000000);

    if (max_duty > CONTROL_PWM_DUTY_LIMIT)
        max_duty = CONTROL_PWM_DUTY_LIMIT;

    duty = max_duty * power / ADC_MAX_VAL;

    return duty;
}

static void __control_stop()
{
    __pwm.stop();
    __timer.stop();
    sampler_stop();
}

static void __control_output(control_mode_t mode)
{
    uint32_t pwrfrq;
    uint32_t power;
    uint32_t freq;
    uint32_t duty;

    pwrfrq = __pwrfrq.load();
    power = PWRFRQ_PWR(pwrfrq);
    freq = PWRFRQ_FRQ(pwrfrq);

    switch (mode)
    {
    case CTRL_MODE_KNOBS:
    case CTRL_MODE_AUDIO_FOLLOW:
    {
        break;
    }
    case CTRL_MODE_AUDIO_AUTOTUNE:
    {
        // update autotune frequency
        break;
    }
    case CTRL_MODE_AUDIO_POWER_CHORDS:
    {
        // update power chord frequency
        break;
    }
    default:
    {
        // this case should not happen but stop all output just in case it does
        __control_stop();
        return;
    }
    }

    if(freq < CONTROL_PWM_MIN_FREQ)
        freq = CONTROL_PWM_MIN_FREQ;

    if (freq > CONTROL_PWM_MAX_FREQ)
        freq = CONTROL_PWM_MAX_FREQ;

    duty = __control_pwm_duty(power, freq);

    __pwm.output(duty, freq);
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
        .group = TIMER_GRP_CONTROL,
        .index = TIMER_IDX_CONTROL,
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

    pwm_init_t pwm_init_data = {
        .timer = {
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_16_BIT,
            .timer_num = PWM_TIMER_CONTROL,
            .freq_hz = CONTROL_PWM_MAX_FREQ, // set max freq to check conflict with resolution
            .clk_cfg = LEDC_USE_APB_CLK      // 80MHz
        },
        .channel = {.gpio_num = GPIO_PWM_OUT, .speed_mode = LEDC_HIGH_SPEED_MODE, .channel = PWM_CHANNEL_CONTROL, .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER_0,
                    .duty = 0, // initial OFF state
                    .hpoint = 0,
                    .flags = {0}}};

    __timer.init(timer_init_data);

    __pwm.init(pwm_init_data);

    adc1_init_channel(ADC_CH_KNOB1, ADC_ATTEN_KNOB);
    adc1_init_channel(ADC_CH_KNOB2, ADC_ATTEN_KNOB);

    sampler_init(__fft_callback);
}

void __control_start(control_mode_t mode)
{
    switch (mode)
    {
    case CTRL_MODE_KNOBS:
    {
        __timer.start();
        break;
    }
    case CTRL_MODE_AUDIO_FOLLOW:
    {
        sampler_start();
        break;
    }
    default:
    {
        __control_stop();
        break;
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
            __control_stop();
            __control_start(new_mode);
            __cur_mode.store(new_mode);
            __set_mode.store(new_mode);
#ifdef CONTROL_DEBUG_MODE
            LOG("mode: old: %d new: %d\n", old_mode, new_mode);
#endif
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
    __prev_freq = 0;

    ret = xTaskCreatePinnedToCore(TASK_control_main, "CONTROL", CONTROL_TASK_STACK_SIZE, NULL, CONTROL_TASK_PRIORITY, &__control_task, CONTROL_TASK_CORE);

    BUG(ret != pdPASS);
}
