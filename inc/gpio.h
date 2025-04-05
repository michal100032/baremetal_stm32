#pragma once

#include <stdint.h>
#include <stm32l4xx.h>

#define GPIO_OUTPUT 1UL
#define GPIO_HIGH 1UL
#define GPIO_LOW 0L

int gpio_pin_configure(GPIO_TypeDef* gpio_port, uint32_t pin, uint32_t mode);
void gpio_pin_write(GPIO_TypeDef* gpio_port, uint32_t pin, uint32_t value);