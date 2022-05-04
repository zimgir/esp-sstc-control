#include <Arduino.h>

#include "config.h"
#include "utils.h"
#include "gpio.h"
#include "control.h"

static uint32_t mode;

void setup()
{
  log_init();
  gpio_init();
  control_init();
  control_set_mode(CTRL_MODE_AUDIO);
}

void loop()
{
  PORT_GET0(mode, PORT_MODE_MASK, PORT_MODE_SHIFT);

  LOG("sampled mode: %d\n");

#if 0
  if (mode != control_get_mode())
    control_set_mode((control_mode_t)mode);
#endif

  delay(MODE_SAMPLE_DELAY);
}