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
static volatile std::atomic<uint32_t> DRAM_ATTR __volfrq;
static volatile std::atomic<uint32_t> DRAM_ATTR __out_flag;
static volatile std::atomic<control_mode_t> DRAM_ATTR __set_mode;
static volatile std::atomic<control_mode_t> DRAM_ATTR __cur_mode;

static MovingAverage<float, CONTROL_MOVING_AVERAGE_SIZE> DRAM_ATTR __avg_off_energy;
static MovingAverage<float, CONTROL_MOVING_AVERAGE_SIZE> DRAM_ATTR __avg_off_standout;

static MovingAverage<float, CONTROL_MOVING_AVERAGE_SIZE> DRAM_ATTR __avg_on_energy;
static MovingAverage<float, CONTROL_MOVING_AVERAGE_SIZE> DRAM_ATTR __avg_on_standout;

static MovingAverage<uint32_t, CONTROL_MOVING_AVERAGE_SIZE> DRAM_ATTR __avg_volume;
static MovingAverage<uint32_t, CONTROL_MOVING_AVERAGE_SIZE> DRAM_ATTR __avg_frequency;


static float DRAM_ATTR __prev_freq;
static uint32_t DRAM_ATTR __step_count;

static HWTimer DRAM_ATTR __oneshot;
static HWTimer DRAM_ATTR __timer;
static LEDCPWM DRAM_ATTR __pwm;

static volatile uint32_t DRAM_ATTR __prog_idx;

typedef struct {
    uint16_t freq;
    uint16_t dur_ms;
} program_note_t;

const program_note_t __program[] = {
   {     0,   428, } , // note 0
   {   329,   321, } , // note 1
   {     0,   107, } , // note 2
   {   246,   214, } , // note 3
   {   261,   214, } , // note 4
   {   293,   214, } , // note 5
   {   329,   107, } , // note 6
   {   293,   107, } , // note 7
   {   261,   214, } , // note 8
   {   246,   214, } , // note 9
   {   220,   107, } , // note 10
   {     0,   107, } , // note 11
   {   220,   214, } , // note 12
   {   246,   214, } , // note 13
   {   261,   214, } , // note 14
   {   329,   428, } , // note 15
   {   293,   214, } , // note 16
   {   261,   214, } , // note 17
   {   246,   214, } , // note 18
   {     0,   214, } , // note 19
   {   246,   214, } , // note 20
   {   261,   214, } , // note 21
   {   293,   428, } , // note 22
   {   329,   428, } , // note 23
   {   261,   428, } , // note 24
   {   220,   214, } , // note 25
   {     0,   214, } , // note 26
   {   220,   428, } , // note 27
   {     0,   642, } , // note 28
   {   349,   428, } , // note 29
   {   391,   214, } , // note 30
   {   440,   107, } , // note 31
   {     0,   107, } , // note 32
   {   440,   214, } , // note 33
   {   391,   214, } , // note 34
   {   349,   214, } , // note 35
   {   440,   214, } , // note 36
   {   329,   107, } , // note 37
   {     0,   107, } , // note 38
   {   329,   214, } , // note 39
   {   349,   214, } , // note 40
   {   329,   428, } , // note 41
   {   293,   214, } , // note 42
   {   261,   214, } , // note 43
   {   246,   214, } , // note 44
   {   220,   214, } , // note 45
   {   246,   214, } , // note 46
   {   261,   214, } , // note 47
   {   293,   428, } , // note 48
   {   329,   428, } , // note 49
   {   261,   428, } , // note 50
   {   220,   214, } , // note 51
   {     0,   214, } , // note 52
   {   220,   428, } , // note 53
};

static uint32_t __get_volume_knob_val()
{
    uint32_t sample;
    sample = adc1_fast_sample(ADC_CH_KNOB_VOL);
#ifdef CONTROL_DEBUG_KNOB_VOL
    LOG("KNOB VOL: %d\n", sample);
#endif
    // normalize to ADC_MAX_VAL acording to physical knob properties
    sample = ADC_MAX_VAL * sample / CONTROL_KNOB_VOL_MAX_VAL;
    return sample;
}

static uint32_t __get_freqency_knob_val()
{
    uint32_t sample;
    sample = adc1_fast_sample(ADC_CH_KNOB_FRQ);
#ifdef CONTROL_DEBUG_KNOB_FRQ
    LOG("KNOB FRQ: %d\n", sample);
#endif
    // normalize to ADC_MAX_VAL acording to physical knob properties
    sample = ADC_MAX_VAL * sample / CONTROL_KNOB_FRQ_MAX_VAL;
    return sample;
}

static void __program_start() 
{
    // restart program
    __prog_idx = 0;

#ifdef CONTROL_DEBUG_PROGRAM
    LOG("PROG N: %d F: %d T: %d\n", 
    __prog_idx,
    __program[__prog_idx].freq,
    __program[__prog_idx].dur_ms);
#endif

    // signal first program note output event
    __volfrq.store(VOLFRQ_VAL(__get_volume_knob_val(), __program[__prog_idx].freq));
    __out_flag.store(1);

    // oneshot timer frequency is 1 MHz so 1 tick is 1 us - convert ms to us
    __oneshot.reload(__program[__prog_idx].dur_ms * 1000);
    __oneshot.start();
}

static bool IRAM_ATTR ISR_oneshot_control(void *arg)
{
    BaseType_t task_awake = pdFALSE;

    __prog_idx = (__prog_idx + 1) % ARRAY_SIZE(__program);

#ifdef CONTROL_DEBUG_PROGRAM
    LOG("PROG N: %d F: %d T: %d\n",
    __prog_idx, 
    __program[__prog_idx].freq,
    __program[__prog_idx].dur_ms);
#endif

    // signal next program note output event
    __volfrq.store(VOLFRQ_VAL(__get_volume_knob_val(), __program[__prog_idx].freq));
    __out_flag.store(1);

    // wake up task for PWM update
    vTaskNotifyGiveFromISR(__control_task, &task_awake);

    return task_awake == pdTRUE;
}

static bool IRAM_ATTR ISR_timer_control(void *arg)
{
    uint32_t volume;
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

    volume = __get_volume_knob_val();
    frequency = CONTROL_PWM_MAX_FREQ * __get_freqency_knob_val() / ADC_MAX_VAL;

    // smooth out noise with moving average to reduce jitter
    __avg_volume.add(volume);
    __avg_frequency.add(frequency);

    __volfrq.store(VOLFRQ_VAL(__avg_volume.value(), __avg_frequency.value()));

    // signal output event
    __out_flag.store(1);

    // wake up task for PWM update
    vTaskNotifyGiveFromISR(__control_task, &task_awake);

    return task_awake == pdTRUE;
}

static int __is_persistent_freq(float new_freq)
{
    int i;

    if (new_freq < (__prev_freq - CONTROL_PERSISTENT_FREQ_TOLERANCE) ||
        new_freq > (__prev_freq + CONTROL_PERSISTENT_FREQ_TOLERANCE))
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

    uint32_t cur_volfrq;
    uint32_t new_volfrq;

    standout = analysis->max_mag - analysis->avg_mag;

    cur_volfrq = __volfrq.load();
    new_volfrq = cur_volfrq;

    if (cur_volfrq != 0)
    { // ON state

        // handle initial case
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
        { // switch OFF

            new_volfrq = 0;
            __avg_off_energy.add(analysis->energy);
            __avg_off_standout.add(standout);
        }
        else
        { // update ON frequency

            new_volfrq = VOLFRQ_VAL(__get_volume_knob_val(), (uint32_t)analysis->max_est_freq);

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
        { // switch ON and update frequency

            new_volfrq = VOLFRQ_VAL(__get_volume_knob_val(), (uint32_t)analysis->max_est_freq);

            __avg_on_energy.add(analysis->energy);
            __avg_on_standout.add(standout);
        }
        else
        { // stay OFF but update moving averages
            __avg_off_energy.add(analysis->energy);
            __avg_off_standout.add(standout);
        }
    }

#ifdef CONTROL_DEBUG_AUDIO_CTRL
    if (new_volfrq != cur_volfrq)
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
        LOG("new_frq: %d\n", VOLFRQ_FRQ(new_volfrq));
        LOG("\n==========================================================\n");
    }
#endif
    __prev_freq = analysis->max_est_freq;
    __volfrq.store(new_volfrq);
    __out_flag.store(1);

    xTaskNotifyGive(__control_task);
}

static uint32_t __control_pwm_duty(uint32_t volume, uint32_t freq)
{
    uint32_t duty;
    uint32_t max_duty;

    // max duty from max wdith limit
    // MAX_DUTY * (MAX_WIDTH_US / MAX_PERIOD_US)
    // CONTROL_PWM_MAX_DUTY * CONTROL_PWM_MAX_WIDTH_US / (1000000 / freq)

    max_duty = freq * (CONTROL_PWM_MAX_DUTY * CONTROL_PWM_MAX_WIDTH_US / 1000000);

    if (max_duty > CONTROL_PWM_DUTY_LIMIT)
        max_duty = CONTROL_PWM_DUTY_LIMIT;

    duty = max_duty * volume / ADC_MAX_VAL;

    return duty;
}

static void __control_stop()
{
    __pwm.stop();
    __timer.stop();
    __oneshot.stop();
    sampler_stop();
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
    case CTRL_MODE_PROGRAM:
    {
        __program_start();
        break;
    }
    default:
    {
        __control_stop();
        break;
    }
    }
}

static void __control_output(control_mode_t mode)
{
    uint32_t volfrq;
    uint32_t volume;
    uint32_t freq;
    uint32_t duty;

    volfrq = __volfrq.load();
    volume = VOLFRQ_VOL(volfrq);
    freq = VOLFRQ_FRQ(volfrq);

    switch (mode)
    {
    case CTRL_MODE_KNOBS:
    case CTRL_MODE_AUDIO_FOLLOW:
    {
        break;
    }
    case CTRL_MODE_PROGRAM:
    {
        // reload program timer
        __oneshot.reload(__program[__prog_idx].dur_ms * 1000);
        __oneshot.start();
        break;
    }
    default:
    {
        // this case should not happen but stop all output just in case it does
        __control_stop();
        return;
    }
    }
      
    if(freq == 0)
        volume = 0;
    else if(freq < CONTROL_PWM_MIN_FREQ)
        freq = CONTROL_PWM_MIN_FREQ;
    else if (freq > CONTROL_PWM_MAX_FREQ)
        freq = CONTROL_PWM_MAX_FREQ;

    duty = __control_pwm_duty(volume, freq);

    __pwm.output(duty, freq);
}

static void __control_task_init(void)
{
    timer_init_t oneshot_init_data = {
        .group = TIMER_GRP_ONESHOT,
        .index = TIMER_IDX_ONESHOT,
        .config = {
            .alarm_en = TIMER_ALARM_EN,
            .counter_en = TIMER_PAUSE,
            .intr_type = TIMER_INTR_LEVEL,
            .counter_dir = TIMER_COUNT_UP,
            .auto_reload = TIMER_AUTORELOAD_DIS,
            .divider = TIMER_DIVIDER(CONTROL_TIMER_FREQ),
        },
        .count = 0,
        .alarm = TIMER_ALARM_COUNT(CONTROL_TIMER_FREQ, CONTROL_KNOB_UPDATE_FREQ),
        .isr_func = ISR_oneshot_control,
        .isr_arg = NULL,
    };

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


    __oneshot.init(oneshot_init_data);
    __timer.init(timer_init_data);
    __pwm.init(pwm_init_data);

    adc1_init_channel(ADC_CH_KNOB_VOL, ADC_ATTEN_KNOB);
    adc1_init_channel(ADC_CH_KNOB_FRQ, ADC_ATTEN_KNOB);

    sampler_init(__fft_callback);
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

control_mode_t control_get_mode(void)
{
    return __cur_mode.load();
}

void control_set_mode(control_mode_t mode)
{
    __set_mode.store(mode);
    xTaskNotifyGive(__control_task);
}

void control_init(void)
{
    BaseType_t ret;

    __volfrq.store(0);
    __out_flag.store(0);
    __set_mode.store(CTRL_MODE_STOP);
    __cur_mode.store(CTRL_MODE_STOP);

    __step_count = 0;
    __prev_freq = 0;

    __prog_idx = 0;

    ret = xTaskCreatePinnedToCore(TASK_control_main, "CONTROL", CONTROL_TASK_STACK_SIZE, NULL, CONTROL_TASK_PRIORITY, &__control_task, CONTROL_TASK_CORE);

    BUG(ret != pdPASS);
}
