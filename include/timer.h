#ifndef __TIMER_H__
#define __TIMER_H__

#include <driver/timer.h>

#define TIMER_DIVIDER(timer_freq) (APB_CLK_FREQ / timer_freq)
#define TIMER_ALARM_COUNT(timer_freq, irq_freq) (timer_freq / irq_freq)

typedef struct
{
    timer_group_t group;
    timer_idx_t index;
    timer_config_t config;
    uint64_t count;
    uint64_t alarm;
    timer_isr_t isr_func;
    void *isr_arg;
} timer_init_t;

class HWTimer
{

public:
    HWTimer() : __group_id(TIMER_GROUP_MAX),
                __timer_id(TIMER_MAX)
    {}

    ~HWTimer() {}

    void init(const timer_init_t &init);
    void start();
    void stop();
    bool counting();

private:
    timer_group_t __group_id;
    timer_idx_t __timer_id;
};

#endif