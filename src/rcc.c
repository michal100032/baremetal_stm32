#include <stdint.h>
#include <stdbool.h>
#include <stm32l4xx.h>

#include "rcc.h"


void rcc_setup(void) {
	RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0); // Wait until HSI ready

    // 2. Configure Flash latency (1WS for 40MHz)
    FLASH->ACR |= FLASH_ACR_LATENCY_1WS;

    // 3. Set AHB, APB1, APB2 prescalers to no division
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);

    // 4. Configure PLL
    RCC->PLLCFGR = 0; // Reset PLLCFGR
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI; // HSI as PLL source
    RCC->PLLCFGR |= (1 << RCC_PLLCFGR_PLLM_Pos); // PLLM = 2 (1 << 4)
    RCC->PLLCFGR |= (20 << RCC_PLLCFGR_PLLN_Pos); // PLLN = 20
    RCC->PLLCFGR |= (0 << RCC_PLLCFGR_PLLR_Pos); // PLLR = 2 -> then divide by 4 (00: /2, 01: /4, 10: /6, 11: /8)
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN; // Enable PLLR output

    // 5. Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0); // Wait for PLL ready

    // 6. Switch system clock to PLL
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Wait for PLL used as system clock

    // System now runs at 40 MHz
}

void rcc_gpioa_enable(void) {
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
}