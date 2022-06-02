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
}

void loop()
{
  PORT_GET0(mode, PORT_MODE_MASK, PORT_MODE_SHIFT);

  if (mode != control_get_mode())
  {
    control_set_mode((control_mode_t)mode);
  }
    
  delay(MODE_SAMPLE_DELAY);
}