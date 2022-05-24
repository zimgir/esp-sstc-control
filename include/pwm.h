#ifndef __PWM_H__
#define __PWM_H__

#include <driver/ledc.h>

typedef struct
{
    ledc_timer_config_t timer;
    ledc_channel_config_t channel;
} pwm_init_t;

class LEDCPWM
{

public:
    LEDCPWM() : __is_paused(false),
                __set_duty(0),
                __set_freq(0),
                __speed_mode(LEDC_SPEED_MODE_MAX),
                __timer_id(LEDC_TIMER_MAX),
                __channel_id(LEDC_CHANNEL_MAX)
    {}

    ~LEDCPWM() {}

    void init(const pwm_init_t &init);
    void output(uint32_t duty, uint32_t freq);
    void stop();
    void start();

private:
    bool __is_paused;
    uint32_t __set_duty;
    uint32_t __set_freq;
    ledc_mode_t __speed_mode;
    ledc_timer_t __timer_id;
    ledc_channel_t __channel_id;
};

#endif