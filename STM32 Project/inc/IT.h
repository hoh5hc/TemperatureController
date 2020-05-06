#ifndef _IT_H
#define _IT_H

#include "STM32F4xx.h"

void SysTick_Handler(void);
void TIM6_DAC_IRQHandler(void);
void EXTI0_IRQHandler(void);

#endif
