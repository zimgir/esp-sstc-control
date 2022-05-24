#include <Arduino.h>

#include "config.h"
#include "utils.h"
#include "pwm.h"

void LEDCPWM::init(const pwm_init_t &init)
{
    ESP_ERROR_CHECK(ledc_timer_config(&init.timer));
    ESP_ERROR_CHECK(ledc_channel_config(&init.channel));

    __speed_mode = init.channel.speed_mode;
    __timer_id = init.timer.timer_num;
    __channel_id = init.channel.channel;
}

void LEDCPWM::output(uint32_t duty, uint32_t freq)
{
    if (freq == 0 || duty == 0)
    {
        __set_duty = 0;
        __set_freq = 0;
        stop();
        return;
    }

    if (__is_paused)
    {
        ESP_ERROR_CHECK(ledc_timer_resume(__speed_mode, __timer_id));
        __is_paused = false;
    }

    if (freq >= __set_freq)
    {
        // if new frequency is higher then update matching duty first to stay within safe duty limits in higher frequency
        // otherwise old duty might go out of safe limits in higher frequency
        ESP_ERROR_CHECK(ledc_set_duty(__speed_mode, __channel_id, duty));
        ESP_ERROR_CHECK(ledc_update_duty(__speed_mode, __channel_id));
        ESP_ERROR_CHECK(ledc_set_freq(__speed_mode, __timer_id, freq));
    }
    else
    {
        // if new frequency is lower then update frequency first to stay within safe duty limits in lower frequency
        // otherwise new duty might go out of safe limits in higher frequency
        ESP_ERROR_CHECK(ledc_set_freq(__speed_mode, __timer_id, freq));
        ESP_ERROR_CHECK(ledc_set_duty(__speed_mode, __channel_id, duty));
        ESP_ERROR_CHECK(ledc_update_duty(__speed_mode, __channel_id));
    }

    __set_duty = duty;
    __set_freq = freq;

#ifdef CONTROL_DEBUG_PWM
    LOG("D: %lu F: %lu\n", __set_duty, __set_freq);
#endif
}

void LEDCPWM::stop()
{
    if (__is_paused)
        return;

    ESP_ERROR_CHECK(ledc_stop(__speed_mode, __channel_id, 0));
    ESP_ERROR_CHECK(ledc_timer_pause(__speed_mode, __timer_id));

    __is_paused = true;
}

void LEDCPWM::start()
{
    output(__set_duty, __set_freq);
}