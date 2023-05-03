#include <Arduino.h>

#include "config.h"
#include "utils.h"
#include "timer.h"


void HWTimer::init(const timer_init_t &init)
{
    ESP_ERROR_CHECK(timer_init(init.group, init.index, &init.config));
    ESP_ERROR_CHECK(timer_set_counter_value(init.group, init.index, init.count));
    ESP_ERROR_CHECK(timer_set_alarm_value(init.group, init.index, init.alarm));

    if (init.isr_func)
    {
        ESP_ERROR_CHECK(timer_enable_intr(init.group, init.index));
        ESP_ERROR_CHECK(timer_isr_callback_add(init.group, init.index, init.isr_func, init.isr_arg, ESP_INTR_FLAG_IRAM));
    }

    __group_id = init.group;
    __timer_id = init.index;
}
void HWTimer::start()
{
    ESP_ERROR_CHECK(timer_start(__group_id, __timer_id));
}

void HWTimer::stop()
{
    ESP_ERROR_CHECK(timer_pause(__group_id, __timer_id));
}

void HWTimer::reload(uint64_t ticks)
{
    ESP_ERROR_CHECK(timer_set_counter_value(__group_id, __timer_id, 0));
    ESP_ERROR_CHECK(timer_set_alarm_value(__group_id, __timer_id, ticks));
    ESP_ERROR_CHECK(timer_set_alarm(__group_id, __timer_id, TIMER_ALARM_EN));
}

bool HWTimer::counting()
{
    timer_config_t config;
    ESP_ERROR_CHECK(timer_get_config(__group_id, __timer_id, &config));
    return config.counter_en == TIMER_START;
}
