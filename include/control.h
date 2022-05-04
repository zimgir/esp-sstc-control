#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <stdint.h>

typedef enum {
    CTRL_MODE_STOP,
    CTRL_MODE_KNOBS,
    CTRL_MODE_AUDIO
} control_mode_t;

void control_init(void);

void control_set_mode(control_mode_t mode);

control_mode_t control_get_mode(void);

#endif