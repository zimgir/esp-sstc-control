#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <stdint.h>

#define PWRFRQ_VAL(pwr, freq) ((((pwr) & 0xFFFF) << 16) | ((freq) & 0xFFFF))
#define PWRFRQ_PWR(val) ((val) >> 16)
#define PWRFRQ_FRQ(val) ((val) & 0xFFFF)

typedef enum {
    CTRL_MODE_STOP,
    CTRL_MODE_KNOBS,
    CTRL_MODE_AUDIO_FOLLOW,
    CTRL_MODE_AUDIO_AUTOTUNE,
    CTRL_MODE_AUDIO_POWER_CHORDS,
    CTRL_MODE_MAX
} control_mode_t;


void control_init(void);

void control_set_mode(control_mode_t mode);

control_mode_t control_get_mode(void);

#endif