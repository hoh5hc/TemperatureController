#include "IO_init.h"
	
GPIO_InitTypeDef GPIO_InitStructure;
NVIC_InitTypeDef   NVIC_InitStructure;
EXTI_InitTypeDef   EXTI_InitStructure;
USART_InitTypeDef USART_InitStructure;
TIM_ICInitTypeDef  TIM_ICInitStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
uint16_t PrescalerValue = 0;
	/*
//SPI_InitTypeDef SPI_InitTypeDefStruct;
//GPIO_InitTypeDef GPIO_InitStructure;

//void SPI_init(void)
//{
//	
//RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
// 

// 
// 
//SPI_InitTypeDefStruct.SPI_Direction = SPI_Direction_1Line_Rx;
//SPI_InitTypeDefStruct.SPI_Mode = SPI_Mode_Master;
//SPI_InitTypeDefStruct.SPI_DataSize = SPI_DataSize_16b;
//SPI_InitTypeDefStruct.SPI_CPOL = SPI_CPOL_High;
//SPI_InitTypeDefStruct.SPI_CPHA = SPI_CPHA_2Edge;
//SPI_InitTypeDefStruct.SPI_NSS = SPI_NSS_Soft;
//SPI_InitTypeDefStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
//SPI_InitTypeDefStruct.SPI_FirstBit = SPI_FirstBit_MSB;
// 
//SPI_Init(SPI2, &SPI_InitTypeDefStruct);
// 
// 
//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);
// 
//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//GPIO_Init(GPIOB, &GPIO_InitStructure);
// 
//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//GPIO_Init(GPIOB, &GPIO_InitStructure);
// 
//GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
//GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
// 
//GPIO_SetBits(GPIOB, GPIO_Pin_6);
// 
// 
//SPI_Cmd(SPI2, ENABLE);
// 
//}
*/
void Temp_SPI_init()
{
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_13 ;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
GPIO_Init(GPIOB, &GPIO_InitStructure);
}
//===========================++===============================//

void PWM_Init()
{
	 /* TIM1 clock enable */
  RCC_APB2PeriphClockCmd(PWMx_CLK, ENABLE);

  /* GPIOA clock enable */
  RCC_AHB1PeriphClockCmd(PWM_PORT_CLK, ENABLE);
  
  
  GPIO_InitStructure.GPIO_Pin = PWM_PIN ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(PWM_PORT, &GPIO_InitStructure); 

 
  GPIO_PinAFConfig(PWM_PORT, PWM_PinSource, PWMx_AF);


  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock /2) /PWM_Prescalered_clock) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = PWM_Period;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(PWMx, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(PWMx, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(PWMx, TIM_OCPreload_Enable);


  TIM_ARRPreloadConfig(PWMx, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(PWMx, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}


//===============================++=========================//

void Sample_Timer_init()
{
	PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / Sample_Prescalered_clock) - 1;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel =TIM6_DAC_IRQn ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = Sample_Period;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	
	TIM_PrescalerConfig(TIM6,PrescalerValue, TIM_PSCReloadMode_Immediate);
	TIM_ITConfig(TIM6,TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM6, ENABLE);
}

//===============================++=========================//
void user_bt_init()
{
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0 ;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//config exti for line 14,15 for PE14,PE15
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

	/* Configure EXTI Line14,15 */
  EXTI_InitStructure.EXTI_Line =  EXTI_Line14;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI15_10 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}



