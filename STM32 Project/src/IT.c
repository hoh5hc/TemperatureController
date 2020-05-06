#include "IT.h"
#include "delay.h"
 
extern uint8_t Timer6_IT;
uint8_t Tuning;
void SysTick_Handler()
{
	TimingDelay_Decrement();
}
void TIM6_DAC_IRQHandler()
{
	Timer6_IT=1;
	TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
	
}
void EXTI0_IRQHandler()
{

	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {
		Tuning=1;
	}
		EXTI_ClearITPendingBit(EXTI_Line0);
}
