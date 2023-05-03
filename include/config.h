#ifndef __CONFIG_H__
#define __CONFIG_H__

/*****************  debug options *******************/

//#define PROFILING_ENABLE
//#define SAMPLER_CHECK_TIMER
//#define CONTROL_CHECK_TIMER
//#define CONTROL_DEBUG_AUDIO_CTRL
//#define CONTROL_DEBUG_MODE
//#define CONTROL_DEBUG_KNOB_VOL
//#define CONTROL_DEBUG_KNOB_FRQ
//#define CONTROL_DEBUG_PWM
//#define CONTROL_DEBUG_PROGRAM

/******************  clock frequency ******************/

#define SERIAL_FREQ 115200UL

/******************  GPIO allocation  *****************/

#define GPIO_ONBOARD_LED      2                  // GPIO[2]

#define GPIO_DEBUG_OUT        13                 // GPIO[13]

#define GPIO_PORT0            16                 // GPIO[16]
#define GPIO_PORT1            17                 // GPIO[17]
#define GPIO_PORT2            18                 // GPIO[18]

#define PORT_MODE_MASK        0x70000            // GPIO[16:18]
#define PORT_MODE_SHIFT       16

#define GPIO_PWM_OUT          23                 // GPIO[23]

#define ADC_CH_KNOB_VOL          ADC1_CHANNEL_6     // GPIO[34]
#define ADC_CH_KNOB_FRQ          ADC1_CHANNEL_7     // GPIO[35]
#define ADC_CH_SAMPLER        ADC1_CHANNEL_0     // GPIO[36] (SP)

/******************  timer allocation  *****************/

#define TIMER_GRP_SAMPLER    TIMER_GROUP_0
#define TIMER_IDX_SAMPLER    TIMER_0

#define TIMER_GRP_CONTROL    TIMER_GROUP_1
#define TIMER_IDX_CONTROL    TIMER_0

#define TIMER_GRP_ONESHOT    TIMER_GROUP_1
#define TIMER_IDX_ONESHOT    TIMER_1


/******************  ADC configuration  *****************/

#define ADC_RESOLUTION        ADC_WIDTH_BIT_12
#define ADC_MAX_VAL           4095 // ((1 << ADC_RESOLUTION) - 1)

#define ADC_ATTEN_SAMPLER     ADC_ATTEN_DB_11
#define ADC_ATTEN_KNOB        ADC_ATTEN_DB_11


/******************  PWM configuration  *****************/

#define PWM_TIMER_CONTROL     LEDC_TIMER_0
#define PWM_CHANNEL_CONTROL   LEDC_CHANNEL_0

/************  controller configuration  ************/

/************  controller resources ************/
#define CONTROL_TASK_CORE 0
#define CONTROL_TASK_PRIORITY 1
#define CONTROL_TASK_STACK_SIZE 4096

#define CONTROL_TIMER_FREQ 1000000UL

/************  controller external knobs tuning  ************/
// knobs do not reach max ADC value due to biasing and resistor differences
// use these defines to calculate accurate range
#define CONTROL_KNOB_VOL_MAX_VAL 2860
#define CONTROL_KNOB_FRQ_MAX_VAL 2780

/************  controller refresh rate  ************/
#define CONTROL_KNOB_UPDATE_FREQ 20
#define CONTROL_AUDIO_UPDATE_FREQ 16

/************  controller fft analysis algorithm tuning  ************/
#define CONTROL_MOVING_AVERAGE_SIZE 8

#define CONTROL_PERSISTENT_FREQ_TOLERANCE 4

#define CONTROL_WEIGHT_ENERGY 0.2
#define CONTROL_WEIGHT_STANDOUT 0.8
#define CONTROL_MULTILIER_PERSIST 1.1

// <less-sensitive> - <more-sensitive>
// 0.2 - 0.8
#define CONTROL_SCORE_OFF_TH_BASE 0.2
#define CONTROL_SCORE_OFF_TH_VAR 0.6

// 3.5 - 1.5
#define CONTROL_SCORE_ON_TH_BASE 3.5
#define CONTROL_SCORE_ON_TH_VAR 2.0

/************  controller pwm output settings  ************/
#define CONTROL_PWM_RESOLUTION LEDC_TIMER_16_BIT // 16 bit available when frequency < 1kHz
#define CONTROL_PWM_MAX_DUTY 65535 // ((1 << PWM_RESOLUTION) - 1)

#define CONTROL_PWM_MIN_FREQ 40 // PWM is unstable if it goes below 40 HZ
#define CONTROL_PWM_MAX_FREQ 500

/************  controller safety limits  ************/
#define CONTROL_PWM_MAX_WIDTH_US 1000
#define CONTROL_PWM_MAX_DUTY_PCNT 20
#define CONTROL_PWM_DUTY_LIMIT (CONTROL_PWM_MAX_DUTY * CONTROL_PWM_MAX_DUTY_PCNT / 100)

/************  audio sampler configuration  ************/

#define SAMPLER_TASK_CORE 1
#define SAMPLER_TASK_PRIORITY 1
#define SAMPLER_TASK_STACK_SIZE 4096

#define SAMPLER_FFT_SIZE 256UL
#define SAMPLER_TIMER_FREQ 1000000UL
// should be >= 2kHz
#define SAMPLER_SAMPLE_FREQ (CONTROL_AUDIO_UPDATE_FREQ * SAMPLER_FFT_SIZE) 

/************  mode sampler configuration  ************/

#define MODE_SAMPLE_DELAY 200

#endif