#ifndef __GPIO_H__
#define __GPIO_H__

#include <driver/gpio.h>

#include "utils.h"

// GPIO0~31
#define GPIO_SET0(num) do {*RAW_REG(GPIO_OUT_W1TS_REG) |= (NUM2BIT(num) & 0xFFFFFFFF);} while(0)
#define GPIO_CLR0(num) do {*RAW_REG(GPIO_OUT_W1TC_REG) |= (NUM2BIT(num) & 0xFFFFFFFF);} while(0)

#define PORT_GET0(var, mask, shift) do {var = (((*RAW_REG(GPIO_IN_REG)) & mask) >> shift);} while(0)

// GPIO32~39
#define GPIO_SET1(num) do {*RAW_REG(GPIO_OUT1_W1TS_REG) |= (NUM2BIT(num) & 0xFF);} while(0)
#define GPIO_CLR1(num) do {*RAW_REG(GPIO_OUT1_W1TC_REG) |= (NUM2BIT(num) & 0xFF);} while(0)

#define PORT_GET1(var, mask, shift) do {var = (((*RAW_REG(GPIO_IN1_REG)) & mask) >> shift);} while(0)

void gpio_init(void);

#endif