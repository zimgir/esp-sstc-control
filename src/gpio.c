#include <Arduino.h>

#include "config.h"
#include "utils.h"
#include "gpio.h"

void gpio_init(void)
{
  //pinMode(GPIO_ONBOARD_LED, OUTPUT); // on board led is already used by the system
  //pinMode(GPIO_DEBUG_OUT, OUTPUT);
  pinMode(GPIO_PORT0, INPUT);
  pinMode(GPIO_PORT1, INPUT);
  pinMode(GPIO_PORT2, INPUT);
}