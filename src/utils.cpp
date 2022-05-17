#include <Arduino.h>


#include "config.h"
#include "utils.h"

void log_init(void)
{
    Serial.begin(SERIAL_FREQ);
}