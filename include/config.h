#ifndef __CONFIG_H__
#define __CONFIG_H__

/*****************  feature options *******************/

#define PROFILING_ENABLE
//#define SAMPLER_CHECK_TIMER

/******************  clock frequency ******************/

#define SERIAL_FREQ 115200UL

/******************  GPIO allocation  *****************/

#define GPIO_ONBOARD_LED      2                 // GPIO[2]

#define GPIO_DEBUG_OUT        13                // GPIO[13]

#define PORT_MODE_MASK        0xF0000           // GPIO[16:19]
#define PORT_MODE_SHIFT       16

#define GPIO_PWM_OUT          23                // GPIO[23]

#define ADC_CH_KNOB1          ADC1_CHANNEL_6     // GPIO[34]
#define ADC_CH_KNOB2          ADC1_CHANNEL_7     // GPIO[35]
#define ADC_CH_SAMPLER        ADC1_CHANNEL_0     // GPIO[36]

/******************  timer allocation  *****************/

#define TIMER_GRP_SAMPLER    TIMER_GROUP_0
#define TIMER_IDX_SAMPLER    TIMER_0


/******************  ADC configuration  *****************/

#define ADC_RESOLUTION        ADC_WIDTH_BIT_12

#define ADC_ATTEN_SAMPLER     ADC_ATTEN_DB_11

/************  controller configuration  ************/

#define CONTROL_TIMER_FREQ 1000000UL
#define CONTROL_SAMPLE_FREQ 4
#define CONTROL_TASK_CORE 0
#define CONTROL_TASK_STACK_SIZE 8192

/************  audio sampler configuration  ************/

#define SAMPLER_TIMER_FREQ 1000000UL
#define SAMPLER_SAMPLE_FREQ 2000UL
#define SAMPLER_FFT_SIZE 256UL
#define SAMPLER_TASK_CORE 0
#define SAMPLER_TASK_STACK_SIZE 8192


/************  mode sampler configuration  ************/

#define MODE_SAMPLE_DELAY 200

#endif