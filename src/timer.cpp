#include <Arduino.h>

#include "config.h"
#include "utils.h"
#include "timer.h"

void timer_init_simple(timer_group_t grp, timer_idx_t idx, const timer_init_t *init)
{
    ESP_ERROR_CHECK(timer_init(grp, idx, &init->config));
    ESP_ERROR_CHECK(timer_set_counter_value(grp, idx, init->count));
    ESP_ERROR_CHECK(timer_set_alarm_value(grp, idx, init->alarm));

    if (init->isr_func)
    {
        ESP_ERROR_CHECK(timer_enable_intr(grp, idx));
        ESP_ERROR_CHECK(timer_isr_callback_add(grp, idx, init->isr_func, init->isr_arg, ESP_INTR_FLAG_IRAM));
    }
}

int timer_is_counting(timer_group_t grp, timer_idx_t idx)
{
    timer_config_t config;
    ESP_ERROR_CHECK(timer_get_config(grp, idx, &config));
    return config.counter_en == TIMER_START;
}