#include <stdint.h>
#include <stdbool.h>
#include <stm32l4xx.h>

#include "gpio.h"

int gpio_pin_configure(GPIO_TypeDef* gpio_port, uint32_t pin, uint32_t mode) {
	if(mode == 1) { // 01 - output
		gpio_port->MODER &= ~(2UL << (2 * pin)); // bit 1: 0
		gpio_port->MODER |= (1UL << (2 * pin));  // bit 0: 1
	
		// output type: push pull (0)
		gpio_port->OTYPER &= ~(1UL << pin);
		// speed: very low (0)
		gpio_port->OSPEEDR &= ~(3UL << (2 * pin));
	
		// pull-up / pull-down: no pullup no pulldown (00)
		gpio_port->PUPDR &= ~(3UL << (2 * pin));	
		
		return 0;
	} 
	return -1; // Not implemented
}

void gpio_pin_write(GPIO_TypeDef* gpio_port, uint32_t pin, uint32_t value) {
	if(value) { // high
		gpio_port->BSRR |= (1UL << (16UL + pin));
	} else {	// low
		gpio_port->BSRR |= (1UL << pin);
	}
}
