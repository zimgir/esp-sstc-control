#include <Arduino.h>

#include <atomic>

#include "config.h"
#include "utils.h"
#include "ledc.h"
#include "adc.h"
#include "timer.h"
#include "sampler.h"
#include "control.h"

static TaskHandle_t DRAM_ATTR __control_task;
static volatile std::atomic<uint32_t> __out_flag;
static volatile std::atomic<control_mode_t> __set_mode;
static volatile std::atomic<control_mode_t> __cur_mode;


static void __fft_callback(struct fft_analysis_t *analysis)
{
    LOG("\n==========================================================\n");
    LOG("min_smpl: %d\n", analysis->min_smpl);
    LOG("max_smpl: %d\n", analysis->max_smpl);
    LOG("avg_smpl: %f\n", analysis->avg_smpl);
    LOG("energy: %f\n", analysis->energy);
    LOG("max_bin_freq: %f\n", analysis->max_bin_freq);
    LOG("max_est_freq: %f\n", analysis->max_est_freq);
    LOG("\n==========================================================\n");
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
        .alarm = TIMER_ALARM_COUNT(CONTROL_TIMER_FREQ, CONTROL_AUDIO_UPDATE_FREQ),
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
    ESP_ERROR_CHECK(timer_pause(TIMER_GRP_CONTROL, TIMER_IDX_CONTROL));

    switch (mode)
    {
    case CTRL_MODE_KNOBS:
    {
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
        ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GRP_CONTROL, TIMER_IDX_CONTROL, TIMER_ALARM_COUNT(CONTROL_TIMER_FREQ, CONTROL_KNOB_UPDATE_FREQ)));
        ESP_ERROR_CHECK(timer_start(TIMER_GRP_CONTROL, TIMER_IDX_CONTROL));
        return mode;
    }
    case CTRL_MODE_AUDIO:
    {
        ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GRP_CONTROL, TIMER_IDX_CONTROL, TIMER_ALARM_COUNT(CONTROL_TIMER_FREQ, CONTROL_AUDIO_UPDATE_FREQ)));
        ESP_ERROR_CHECK(timer_start(TIMER_GRP_CONTROL, TIMER_IDX_CONTROL));
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
    uint32_t notify_count;
    control_mode_t old_mode, new_mode;

    __control_task_init();

    while (1)
    {
        // sleep until notified or for 1 sec
        PROFILE_OP(notify_count = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1000)));

        if (notify_count == 0)
            continue; // wait timeout no actual notifications

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
            LOG("started mode: %d\n", new_mode);
            __cur_mode.store(new_mode);
            __set_mode.store(new_mode);
        }
    }
}

void control_init(void)
{
    BaseType_t ret;

    __out_flag.store(0);
    __set_mode.store(CTRL_MODE_STOP);
    __cur_mode.store(CTRL_MODE_STOP);

    ret = xTaskCreatePinnedToCore(TASK_control_main, "CONTROL", CONTROL_TASK_STACK_SIZE, NULL, CONTROL_TASK_PRIORITY, &__control_task, CONTROL_TASK_CORE);

    BUG(ret != pdPASS);
}
