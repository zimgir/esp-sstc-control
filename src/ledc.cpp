#include <Arduino.h>

#include "config.h"
#include "utils.h"
#include "ledc.h"

void ledc_init_simple(const ledc_init_t* init)
{
    ESP_ERROR_CHECK(ledc_timer_config(&init->timer));
    ESP_ERROR_CHECK(ledc_channel_config(&init->channel));
}