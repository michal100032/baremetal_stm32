.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

.global Reset_Handler
.global main

.equ PERIPH_BASE, 0x40000000
.equ AHB1PERIPH_BASE, (PERIPH_BASE + 0x00020000)
.equ RCC_BASE, (AHB1PERIPH_BASE + 0x1000UL)
.equ RCC_AHB2ENR, (RCC_BASE + 0x4C)
.equ RCC_AHB2ENR_GPIOAEN, 1

.equ AHB2PERIPH_BASE, (PERIPH_BASE + 0x08000000UL)
.equ GPIOA_BASE, (AHB2PERIPH_BASE + 0x0000UL)

.equ GPIOA_MODER, (GPIOA_BASE + 0x0)

.equ GPIO_MODER_MODE5_0, 0x00000400
.equ GPIO_MODER_MODE5_1, 0x00000800

.equ GPIOA_OTYPER, (GPIOA_BASE + 0x4)
.equ GPIO_OTYPER_OT5, 0x00000020

.equ GPIOA_OSPEEDR, (GPIOA_BASE + 0x8)
.equ GPIO_OSPEEDR_OSPEED5, 0x00000C00

.equ GPIOA_PUPDR, (GPIOA_BASE + 0x0C)
.equ GPIO_PUPDR_PUPD5, 0x00000C00

.equ GPIOA_BSRR, (GPIOA_BASE + 0x18)
.equ GPIO_BSRR_BR5, 0x00200000
.equ GPIO_BSRR_BS5, 0x00000020

.section .text


// wychodzi na to że ważne, ustawia bit 0?
.type Reset_Handler, %function
Reset_Handler:
  /* Set the stack pointer */
  ldr   r0, =_estack
  mov   sp, r0


  ldr r0, =_edata
  ldr r1, =_sdata

  ldr r2, =_etext
  subs r0, r1

  // r0 - loop counter (remaining words to copy)
  // r1 - target address in SRAM
  // r2 - source address in FLASH
  beq copy_data_loop_skip
  copy_data_loop:
    
    ldr r3, [r2], #4
    str r3, [r1], #4

    subs r0, #4
    bne copy_data_loop
copy_data_loop_skip:
  
  ldr r0, = _sbss
  ldr r1, = _ebss
  
  eor r3, r3

  ldr r2, =_etext
  cmp r0, r1
  beq zero_bss_loop_skip
  zero_bss_loop:
    str r3, [r0], #4
    cmp r0, r1
    bne zero_bss_loop
zero_bss_loop_skip:
  b main

  .size Reset_Handler, .-Reset_Handler

blink_for_eternity:
  // load the pointer to the memory
  ldr r1, =RCC_AHB2ENR
  // load data from the pointer
  ldr r0, [r1]
  orr r0, RCC_AHB2ENR_GPIOAEN
  str r0, [r1]

  @ // gpio output (01)
	@ GPIOA->MODER &= ~GPIO_MODER_MODE5_1; // bit 1: 0
	@ GPIOA->MODER |= GPIO_MODER_MODE5_0;  // bit 0: 1

  ldr r1, =GPIOA_MODER
  ldr r0, [r1]

  bic r0, r0, GPIO_MODER_MODE5_1
  orr r0, GPIO_MODER_MODE5_0
  str r0, [r1]


	@ // output type: push pull (0)
	@ GPIOA->OTYPER &= ~GPIO_OTYPER_OT5;
  ldr r1, =GPIOA_OTYPER
  ldr r0, [r1]

  bic r0, r0, GPIO_OTYPER_OT5
  str r0, [r1]
	@ // speed: very low (0)
	@ GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED5;
  ldr r1, =GPIOA_OSPEEDR
  ldr r0, [r1]

  bic r0, r0, GPIO_OSPEEDR_OSPEED5
  str r0, [r1]
	@ // pull-up / pull-down: no pullup no pulldown (00)
	@ GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD5;
  ldr r1, =GPIOA_PUPDR
  ldr r0, [r1]

  bic r0, r0, GPIO_PUPDR_PUPD5
  str r0, [r1]

  ldr r0, =GPIOA_BSRR

  eor r1, r1
  eor r2, r2

  movt r1, (GPIO_BSRR_BR5 >> 16)
  movw r2, GPIO_BSRR_BS5
  main_loop:
    str r1, [r0]
    bl delay
    str r2, [r0]
    bl delay
    b main_loop

    

delay:
  ldr r8, =1000000
  delay_loop:
    subs r8, #1
    bne delay_loop
  bx lr

Default_Handler:
Infinite_Loop:
  b Infinite_Loop
  .size Default_Handler, .-Default_Handler


.section .isr_vector

.vector_table:

.word _estack
.word Reset_Handler
.word NMI_Handler
.word HardFault_Handler
.word	MemManage_Handler
.word	BusFault_Handler
.word	UsageFault_Handler
.word	0
.word	0
.word	0
.word	0
.word	SVC_Handler
.word	DebugMon_Handler
.word	0
.word	PendSV_Handler
.word	SysTick_Handler
.word	WWDG_IRQHandler              			/* Window Watchdog interrupt                                           */
.word	PVD_PVM_IRQHandler           			/* PVD through EXTI line detection                                     */
.word	TAMP_STAMP_IRQHandler        			/* Tamper and TimeStamp interrupts                                     */
.word	RTC_WKUP_IRQHandler          			/* RTC Tamper or TimeStamp /CSS on LSE through EXTI line 19 interrupts */
.word	FLASH_IRQHandler             			/* Flash global interrupt                                              */
.word	RCC_IRQHandler               			/* RCC global interrupt                                                */
.word	EXTI0_IRQHandler             			/* EXTI Line 0 interrupt                                               */
.word	EXTI1_IRQHandler             			/* EXTI Line 1 interrupt                                               */
.word	EXTI2_IRQHandler             			/* EXTI Line 2 interrupt                                               */
.word	EXTI3_IRQHandler             			/* EXTI Line 3 interrupt                                               */
.word	EXTI4_IRQHandler             			/* EXTI Line 4 interrupt                                               */
.word	DMA1_CH1_IRQHandler          			/* DMA1 Channel1 global interrupt                                      */
.word	DMA1_CH2_IRQHandler          			/* DMA1 Channel2 global interrupt                                      */
.word	DMA1_CH3_IRQHandler          			/* DMA1 Channel3 interrupt                                             */
.word	DMA1_CH4_IRQHandler          			/* DMA1 Channel4 interrupt                                             */
.word	DMA1_CH5_IRQHandler          			/* DMA1 Channel5 interrupt                                             */
.word	DMA1_CH6_IRQHandler          			/* DMA1 Channel6 interrupt                                             */
.word	DMA1_CH7_IRQHandler          			/* DMA1 Channel 7 interrupt                                            */
.word	ADC1_2_IRQHandler            			/* ADC1 and ADC2 global interrupt                                      */
.word	CAN1_TX_IRQHandler           			/* CAN1 TX interrupts                                                  */
.word	CAN1_RX0_IRQHandler          			/* CAN1 RX0 interrupts                                                 */
.word	CAN1_RX1_IRQHandler          			/* CAN1 RX1 interrupts                                                 */
.word	CAN1_SCE_IRQHandler          			/* CAN1 SCE interrupt                                                  */
.word	EXTI9_5_IRQHandler           			/* EXTI Line5 to Line9 interrupts                                      */
.word	TIM1_BRK_TIM15_IRQHandler    			/* TIM1 Break/TIM15 global interrupts                                  */
.word	TIM1_UP_TIM16_IRQHandler     			/* TIM1 Update/TIM16 global interrupts                                 */
.word	TIM1_TRG_COM_TIM17_IRQHandler			/* TIM1 Trigger and Commutation interrupts and TIM17 global interrupt  */
.word	TIM1_CC_IRQHandler           			/* TIM1 Capture Compare interrupt                                      */
.word	TIM2_IRQHandler              			/* TIM2 global interrupt                                               */
.word	TIM3_IRQHandler              			/* TIM3 global interrupt                                               */
.word	TIM4_IRQHandler              			/* TIM4 global interrupt                                               */
.word	I2C1_EV_IRQHandler           			/* I2C1 event interrupt                                                */
.word	I2C1_ER_IRQHandler           			/* I2C1 error interrupt                                                */
.word	I2C2_EV_IRQHandler           			/* I2C2 event interrupt                                                */
.word	I2C2_ER_IRQHandler           			/* I2C2 error interrupt                                                */
.word	SPI1_IRQHandler              			/* SPI1 global interrupt                                               */
.word	SPI2_IRQHandler              			/* SPI2 global interrupt                                               */
.word	USART1_IRQHandler            			/* USART1 global interrupt                                             */
.word	USART2_IRQHandler            			/* USART2 global interrupt                                             */
.word	USART3_IRQHandler            			/* USART3 global interrupt                                             */
.word	EXTI15_10_IRQHandler         			/* EXTI Lines 10 to 15 interrupts                                      */
.word	RTC_ALARM_IRQHandler         			/* RTC alarms through EXTI line 18 interrupts                          */
.word	DFSDM1_FLT3_IRQHandler       			/* DFSDM1_FLT3 global interrupt                                        */
.word	TIM8_BRK_IRQHandler          			/* TIM8 Break Interrupt                                                */
.word	TIM8_UP_IRQHandler           			/* TIM8 Update Interrupt                                               */
.word	TIM8_TRG_COM_IRQHandler      			/* TIM8 Trigger and Commutation Interrupt                              */
.word	TIM8_CC_IRQHandler           			/* TIM8 Capture Compare Interrupt                                      */
.word	ADC3_IRQHandler              			/* ADC3 global interrupt                                               */
.word	FMC_IRQHandler               			/* FMC global Interrupt                                                */
.word	SDMMC1_IRQHandler            			/* SDMMC global Interrupt                                              */
.word	TIM5_IRQHandler              			/* TIM5 global Interrupt                                               */
.word	SPI3_IRQHandler              			/* SPI3 global Interrupt                                               */
.word	UART4_IRQHandler             			/* UART4 global Interrupt                                              */
.word	UART5_IRQHandler             			/* UART5 global Interrupt                                              */
.word	TIM6_DACUNDER_IRQHandler     			/* TIM6 global and DAC1 and 2 underrun error interrupts                */
.word	TIM7_IRQHandler              			/* TIM7 global interrupt                                               */
.word	DMA2_CH1_IRQHandler          			/* DMA2 Channel 1 global Interrupt                                     */
.word	DMA2_CH2_IRQHandler          			/* DMA2 Channel 2 global Interrupt                                     */
.word	DMA2_CH3_IRQHandler          			/* DMA2 Channel 3 global Interrupt                                     */
.word	DMA2_CH4_IRQHandler          			/* DMA2 Channel 4 global Interrupt                                     */
.word	DMA2_CH5_IRQHandler          			/* DMA2 Channel 5 global Interrupt                                     */
.word	DFSDM1_FLT0_IRQHandler       			/* DFSDM1_FLT0 global interrupt                                        */
.word	DFSDM1_FLT1_IRQHandler       			/* DFSDM1_FLT1 global interrupt                                        */
.word	DFSDM1_FLT2_IRQHandler       			/* DFSDM1_FLT2 global interrupt                                        */
.word	COMP_IRQHandler              			/* COMP1 and COMP2 interrupts                                          */
.word	LPTIM1_IRQHandler            			/* LP TIM1 interrupt                                                   */
.word	LPTIM2_IRQHandler            			/* LP TIM2 interrupt                                                   */
.word	OTG_FS_IRQHandler            			/* USB OTG FS global Interrupt                                         */
.word	DMA2_CH6_IRQHandler          			/* DMA2 Channel 6 global Interrupt                                     */
.word	DMA2_CH7_IRQHandler          			/* DMA2 Channel 7 global Interrupt                                     */
.word	LPUART1_IRQHandler           			/* LPUART1 global interrupt                                            */
.word	QUADSPI_IRQHandler           			/* Quad SPI global interrupt                                           */
.word	I2C3_EV_IRQHandler           			/* I2C3 event interrupt                                                */
.word	I2C3_ER_IRQHandler           			/* I2C3 error interrupt                                                */
.word	SAI1_IRQHandler              			/* SAI1 global interrupt                                               */
.word	SAI2_IRQHandler              			/* SAI2 global interrupt                                               */
.word	SWPMI1_IRQHandler            			/* SWPMI1 global interrupt                                             */
.word	TSC_IRQHandler               			/* TSC global interrupt                                                */
.word	LCD_IRQHandler               			/* LCD global interrupt                                                */
.word	0                            			/* Reserved                                                            */
.word	RNG_IRQHandler               			/* RNG and HASH global interrupt                                       */
.word	FPU_IRQHandler               			/* Floating point interrupt                                            */
.word	CRS_IRQHandler               			/* HASH and CRS interrupt                                              */

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/

.weak	NMI_Handler
.thumb_set NMI_Handler,Default_Handler

.weak	HardFault_Handler
.thumb_set HardFault_Handler,Default_Handler

.weak	MemManage_Handler
.thumb_set MemManage_Handler,Default_Handler

.weak	BusFault_Handler
.thumb_set BusFault_Handler,Default_Handler

.weak	UsageFault_Handler
.thumb_set UsageFault_Handler,Default_Handler

.weak	SVC_Handler
.thumb_set SVC_Handler,Default_Handler

.weak	DebugMon_Handler
.thumb_set DebugMon_Handler,Default_Handler

.weak	PendSV_Handler
.thumb_set PendSV_Handler,Default_Handler

.weak	SysTick_Handler
.thumb_set SysTick_Handler,Default_Handler

.weak	WWDG_IRQHandler
.thumb_set WWDG_IRQHandler,Default_Handler

.weak	PVD_PVM_IRQHandler
.thumb_set PVD_PVM_IRQHandler,Default_Handler

.weak	TAMP_STAMP_IRQHandler
.thumb_set TAMP_STAMP_IRQHandler,Default_Handler

.weak	RTC_WKUP_IRQHandler
.thumb_set RTC_WKUP_IRQHandler,Default_Handler

.weak	FLASH_IRQHandler
.thumb_set FLASH_IRQHandler,Default_Handler

.weak	RCC_IRQHandler
.thumb_set RCC_IRQHandler,Default_Handler

.weak	EXTI0_IRQHandler
.thumb_set EXTI0_IRQHandler,Default_Handler

.weak	EXTI1_IRQHandler
.thumb_set EXTI1_IRQHandler,Default_Handler

.weak	EXTI2_IRQHandler
.thumb_set EXTI2_IRQHandler,Default_Handler

.weak	EXTI3_IRQHandler
.thumb_set EXTI3_IRQHandler,Default_Handler

.weak	EXTI4_IRQHandler
.thumb_set EXTI4_IRQHandler,Default_Handler

.weak	DMA1_CH1_IRQHandler
.thumb_set DMA1_CH1_IRQHandler,Default_Handler

.weak	DMA1_CH2_IRQHandler
.thumb_set DMA1_CH2_IRQHandler,Default_Handler

.weak	DMA1_CH3_IRQHandler
.thumb_set DMA1_CH3_IRQHandler,Default_Handler

.weak	DMA1_CH4_IRQHandler
.thumb_set DMA1_CH4_IRQHandler,Default_Handler

.weak	DMA1_CH5_IRQHandler
.thumb_set DMA1_CH5_IRQHandler,Default_Handler

.weak	DMA1_CH6_IRQHandler
.thumb_set DMA1_CH6_IRQHandler,Default_Handler

.weak	DMA1_CH7_IRQHandler
.thumb_set DMA1_CH7_IRQHandler,Default_Handler

.weak	ADC1_2_IRQHandler
.thumb_set ADC1_2_IRQHandler,Default_Handler

.weak	CAN1_TX_IRQHandler
.thumb_set CAN1_TX_IRQHandler,Default_Handler

.weak	CAN1_RX0_IRQHandler
.thumb_set CAN1_RX0_IRQHandler,Default_Handler

.weak	CAN1_RX1_IRQHandler
.thumb_set CAN1_RX1_IRQHandler,Default_Handler

.weak	CAN1_SCE_IRQHandler
.thumb_set CAN1_SCE_IRQHandler,Default_Handler

.weak	EXTI9_5_IRQHandler
.thumb_set EXTI9_5_IRQHandler,Default_Handler

.weak	TIM1_BRK_TIM15_IRQHandler
.thumb_set TIM1_BRK_TIM15_IRQHandler,Default_Handler

.weak	TIM1_UP_TIM16_IRQHandler
.thumb_set TIM1_UP_TIM16_IRQHandler,Default_Handler

.weak	TIM1_TRG_COM_TIM17_IRQHandler
.thumb_set TIM1_TRG_COM_TIM17_IRQHandler,Default_Handler

.weak	TIM1_CC_IRQHandler
.thumb_set TIM1_CC_IRQHandler,Default_Handler

.weak	TIM2_IRQHandler
.thumb_set TIM2_IRQHandler,Default_Handler

.weak	TIM3_IRQHandler
.thumb_set TIM3_IRQHandler,Default_Handler

.weak	TIM4_IRQHandler
.thumb_set TIM4_IRQHandler,Default_Handler

.weak	I2C1_EV_IRQHandler
.thumb_set I2C1_EV_IRQHandler,Default_Handler

.weak	I2C1_ER_IRQHandler
.thumb_set I2C1_ER_IRQHandler,Default_Handler

.weak	I2C2_EV_IRQHandler
.thumb_set I2C2_EV_IRQHandler,Default_Handler

.weak	I2C2_ER_IRQHandler
.thumb_set I2C2_ER_IRQHandler,Default_Handler

.weak	SPI1_IRQHandler
.thumb_set SPI1_IRQHandler,Default_Handler

.weak	SPI2_IRQHandler
.thumb_set SPI2_IRQHandler,Default_Handler

.weak	USART1_IRQHandler
.thumb_set USART1_IRQHandler,Default_Handler

.weak	USART2_IRQHandler
.thumb_set USART2_IRQHandler,Default_Handler

.weak	USART3_IRQHandler
.thumb_set USART3_IRQHandler,Default_Handler

.weak	EXTI15_10_IRQHandler
.thumb_set EXTI15_10_IRQHandler,Default_Handler

.weak	RTC_ALARM_IRQHandler
.thumb_set RTC_ALARM_IRQHandler,Default_Handler

.weak	DFSDM1_FLT3_IRQHandler
.thumb_set DFSDM1_FLT3_IRQHandler,Default_Handler

.weak	TIM8_BRK_IRQHandler
.thumb_set TIM8_BRK_IRQHandler,Default_Handler

.weak	TIM8_UP_IRQHandler
.thumb_set TIM8_UP_IRQHandler,Default_Handler

.weak	TIM8_TRG_COM_IRQHandler
.thumb_set TIM8_TRG_COM_IRQHandler,Default_Handler

.weak	TIM8_CC_IRQHandler
.thumb_set TIM8_CC_IRQHandler,Default_Handler

.weak	ADC3_IRQHandler
.thumb_set ADC3_IRQHandler,Default_Handler

.weak	FMC_IRQHandler
.thumb_set FMC_IRQHandler,Default_Handler

.weak	SDMMC1_IRQHandler
.thumb_set SDMMC1_IRQHandler,Default_Handler

.weak	TIM5_IRQHandler
.thumb_set TIM5_IRQHandler,Default_Handler

.weak	SPI3_IRQHandler
.thumb_set SPI3_IRQHandler,Default_Handler

.weak	UART4_IRQHandler
.thumb_set UART4_IRQHandler,Default_Handler

.weak	UART5_IRQHandler
.thumb_set UART5_IRQHandler,Default_Handler

.weak	TIM6_DACUNDER_IRQHandler
.thumb_set TIM6_DACUNDER_IRQHandler,Default_Handler

.weak	TIM7_IRQHandler
.thumb_set TIM7_IRQHandler,Default_Handler

.weak	DMA2_CH1_IRQHandler
.thumb_set DMA2_CH1_IRQHandler,Default_Handler

.weak	DMA2_CH2_IRQHandler
.thumb_set DMA2_CH2_IRQHandler,Default_Handler

.weak	DMA2_CH3_IRQHandler
.thumb_set DMA2_CH3_IRQHandler,Default_Handler

.weak	DMA2_CH4_IRQHandler
.thumb_set DMA2_CH4_IRQHandler,Default_Handler

.weak	DMA2_CH5_IRQHandler
.thumb_set DMA2_CH5_IRQHandler,Default_Handler

.weak	DFSDM1_FLT0_IRQHandler
.thumb_set DFSDM1_FLT0_IRQHandler,Default_Handler

.weak	DFSDM1_FLT1_IRQHandler
.thumb_set DFSDM1_FLT1_IRQHandler,Default_Handler

.weak	DFSDM1_FLT2_IRQHandler
.thumb_set DFSDM1_FLT2_IRQHandler,Default_Handler

.weak	COMP_IRQHandler
.thumb_set COMP_IRQHandler,Default_Handler

.weak	LPTIM1_IRQHandler
.thumb_set LPTIM1_IRQHandler,Default_Handler

.weak	LPTIM2_IRQHandler
.thumb_set LPTIM2_IRQHandler,Default_Handler

.weak	OTG_FS_IRQHandler
.thumb_set OTG_FS_IRQHandler,Default_Handler

.weak	DMA2_CH6_IRQHandler
.thumb_set DMA2_CH6_IRQHandler,Default_Handler

.weak	DMA2_CH7_IRQHandler
.thumb_set DMA2_CH7_IRQHandler,Default_Handler

.weak	LPUART1_IRQHandler
.thumb_set LPUART1_IRQHandler,Default_Handler

.weak	QUADSPI_IRQHandler
.thumb_set QUADSPI_IRQHandler,Default_Handler

.weak	I2C3_EV_IRQHandler
.thumb_set I2C3_EV_IRQHandler,Default_Handler

.weak	I2C3_ER_IRQHandler
.thumb_set I2C3_ER_IRQHandler,Default_Handler

.weak	SAI1_IRQHandler
.thumb_set SAI1_IRQHandler,Default_Handler

.weak	SAI2_IRQHandler
.thumb_set SAI2_IRQHandler,Default_Handler

.weak	SWPMI1_IRQHandler
.thumb_set SWPMI1_IRQHandler,Default_Handler

.weak	TSC_IRQHandler
.thumb_set TSC_IRQHandler,Default_Handler

.weak	LCD_IRQHandler
.thumb_set LCD_IRQHandler,Default_Handler

.weak	RNG_IRQHandler
.thumb_set RNG_IRQHandler,Default_Handler

.weak	FPU_IRQHandler
.thumb_set FPU_IRQHandler,Default_Handler

.weak	CRS_IRQHandler
.thumb_set CRS_IRQHandler,Default_Handler

.weak	SystemInit

/************************ (C) COPYRIGHT STMicroelectonics *****END OF FILE****/
