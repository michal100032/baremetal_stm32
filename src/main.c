#include <stdint.h>
#include <stdbool.h>
#include <stm32l4xx.h>

#include "gpio.h"
#include "rcc.h"

static void delay(uint32_t delay_time) {
	for(volatile uint32_t i = 0; i < delay_time; i++);
}

int main(void) {

	// rcc_setup();
	rcc_gpioa_enable();
	gpio_pin_configure(GPIOA, 5, GPIO_OUTPUT);	

	while(true) {
		gpio_pin_write(GPIOA, 5, GPIO_HIGH);

		delay(100000);

		gpio_pin_write(GPIOA, 5, GPIO_LOW);

		delay(100000);
	}
}