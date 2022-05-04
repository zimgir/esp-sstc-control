#ifndef __ADC_H__
#define __ADC_H__

#include <stdint.h>
#include <driver/adc.h>


void adc1_init();
void adc1_init_channel(adc1_channel_t channel, adc_atten_t atten);

uint16_t adc1_fast_sample(adc1_channel_t channel);

#endif