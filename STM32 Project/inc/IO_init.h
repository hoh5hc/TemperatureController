#ifndef _IO_init_H
#define _IO_init_H

#include "STM32F4xx.h"

void Temp_SPI_init(void);
void PWM_Init(void);
void Sample_Timer_init(void);
void user_bt_init(void);

//////////////////////////////////////////////////
///////// Defines const here///////////////////
//////////////////////////////////////////////////


//Timer6_Sample_Time 
#define			Sample_Prescalered_clock		5000
#define			Sample_Period								10000

//PWM DEFINE
#define    	PWM_PORT_CLK					RCC_AHB1Periph_GPIOA
#define  	  PWMx_CLK 							RCC_APB2Periph_TIM1
#define			PWM_PORT							GPIOA				
#define			PWM_PIN								GPIO_Pin_8	
#define			PWM_PinSource					GPIO_PinSource8	
#define			PWMx_AF								GPIO_AF_TIM1
#define			PWMx									TIM1
#define			PWM_Prescalered_clock			5000
#define			PWM_Period								10000



#endif
