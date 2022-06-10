#include <Arduino.h>

#include "config.h"
#include "utils.h"
#include "pwm.h"

esp_err_t ledc_set_freq_no_reset(ledc_mode_t speed_mode, ledc_timer_t timer_num, uint32_t freq_hz);

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
    if (duty == 0)
    {
        __set_duty = 0;
        stop();
        return;
    }

    if (__is_paused)
    {
        ESP_ERROR_CHECK(ledc_timer_resume(__speed_mode, __timer_id));
        __is_paused = false;
    }

    // conditionally update only if values updated to avoid same value update jitter

    if (freq >= __set_freq)
    {
        // if new frequency is higher then update matching duty first to stay within safe duty limits in higher frequency
        // otherwise old duty might go out of safe limits in higher frequency

        if (duty != __set_duty)
        {
            ESP_ERROR_CHECK(ledc_set_duty(__speed_mode, __channel_id, duty));
            ESP_ERROR_CHECK(ledc_update_duty(__speed_mode, __channel_id));
        }

        if (freq != __set_freq)
        {
            ESP_ERROR_CHECK(ledc_set_freq_no_reset(__speed_mode, __timer_id, freq));
        }
    }
    else
    {
        // if new frequency is lower then update frequency first to stay within safe duty limits in lower frequency
        // otherwise new duty might go out of safe limits in higher frequency

        if (freq != __set_freq)
        {
            ESP_ERROR_CHECK(ledc_set_freq_no_reset(__speed_mode, __timer_id, freq));
        }

        if (duty != __set_duty)
        {
            ESP_ERROR_CHECK(ledc_set_duty(__speed_mode, __channel_id, duty));
            ESP_ERROR_CHECK(ledc_update_duty(__speed_mode, __channel_id));
        }
    }

    __set_duty = duty;
    __set_freq = freq;

#ifdef CONTROL_DEBUG_PWM
    LOG("PWM D: %lu F: %lu\n", __set_duty, __set_freq);
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