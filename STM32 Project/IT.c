#include "IT.h"
#include "delay.h"


void SysTick_Handler()
{
	TimingDelay_Decrement();
}