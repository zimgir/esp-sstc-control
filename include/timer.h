#ifndef __TIMER_H__
#define __TIMER_H__

#include <driver/timer.h>

#define TIMER_DIVIDER(timer_freq) (APB_CLK_FREQ / timer_freq)
#define TIMER_ALARM_COUNT(timer_freq, irq_freq) (timer_freq / irq_freq)

typedef struct {
    timer_config_t config;
    uint64_t count;
    uint64_t alarm;
    timer_isr_t isr_func;
    void * isr_arg;
} timer_init_t;


void timer_init_simple(timer_group_t grp, timer_idx_t idx, const timer_init_t* init);

int timer_is_counting(timer_group_t grp, timer_idx_t idx);


#endif