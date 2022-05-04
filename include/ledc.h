#ifndef __LEDC_H__
#define __LEDC_H__

#include <driver/ledc.h>

typedef struct {
    ledc_timer_config_t timer;
    ledc_channel_config_t channel;
} ledc_init_t;


void ledc_init_simple(const ledc_init_t* init);


#endif