#ifndef __CONFIG_H__
#define __CONFIG_H__

/*****************  debug options *******************/

//#define PROFILING_ENABLE
//#define SAMPLER_CHECK_TIMER
//#define CONTROL_CHECK_TIMER

/******************  clock frequency ******************/

#define SERIAL_FREQ 115200UL

/******************  GPIO allocation  *****************/

#define GPIO_ONBOARD_LED      2                  // GPIO[2]

#define GPIO_DEBUG_OUT        13                 // GPIO[13]

#define PORT_MODE_MASK        0xF0000            // GPIO[16:19]
#define PORT_MODE_SHIFT       16

#define GPIO_PWM_OUT          23                 // GPIO[23]

#define ADC_CH_KNOB1          ADC1_CHANNEL_6     // GPIO[34]
#define ADC_CH_KNOB2          ADC1_CHANNEL_7     // GPIO[35]
#define ADC_CH_SAMPLER        ADC1_CHANNEL_0     // GPIO[36]

/******************  timer allocation  *****************/

#define TIMER_GRP_SAMPLER    TIMER_GROUP_0
#define TIMER_IDX_SAMPLER    TIMER_0

#define TIMER_GRP_CONTROL    TIMER_GROUP_1
#define TIMER_IDX_CONTROL    TIMER_0


/******************  ADC configuration  *****************/

#define ADC_RESOLUTION        ADC_WIDTH_BIT_12
#define ADC_MAX_VAL           4095

#define ADC_ATTEN_SAMPLER     ADC_ATTEN_DB_11
#define ADC_ATTEN_KNOB        ADC_ATTEN_DB_11

/************  controller configuration  ************/

#define CONTROL_TASK_CORE 0
#define CONTROL_TASK_PRIORITY 1
#define CONTROL_TASK_STACK_SIZE 4096

#define CONTROL_TIMER_FREQ 1000000UL

#define CONTROL_MAX_OUTPUT_FREQ 500

#define CONTROL_KNOB_UPDATE_FREQ 20
#define CONTROL_AUDIO_UPDATE_FREQ 8

#define CONTROL_MOVING_AVERAGE_SIZE 16

#define CONTROL_PERSISTENT_FREQ_LIMIT 4

#define CONTROL_CONFIDENCE_STEP_COUNT 1

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

/************  audio sampler configuration  ************/

#define SAMPLER_TASK_CORE 1
#define SAMPLER_TASK_PRIORITY 1
#define SAMPLER_TASK_STACK_SIZE 4096

#define SAMPLER_FFT_SIZE 256UL
#define SAMPLER_TIMER_FREQ 1000000UL
// should be around 2kHz
#define SAMPLER_SAMPLE_FREQ (CONTROL_AUDIO_UPDATE_FREQ * SAMPLER_FFT_SIZE) 

/************  mode sampler configuration  ************/

#define MODE_SAMPLE_DELAY 200

#endif