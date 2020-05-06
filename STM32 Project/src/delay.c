#include "delay.h"

static __IO uint32_t TimingDelay;

/*--------------------------------------------------------RUTIN DELAY-----------------------------------------------------------------------------*/
void delay_ms(__IO uint32_t nTime)
{ 
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

void delay_us(__IO uint32_t xx)
{  xx*= delay_const;	
   while(xx--);
}
