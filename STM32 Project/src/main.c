#include "delay.h"
#include "IO_init.h"
#include "IT.h"
#include "Function.h"

int main()
	{
	
	SystemCoreClockUpdate();                      
  if (SysTick_Config(SystemCoreClock / 1000)) 
		{ 
    while (1);                                  
		}
	//==============================================//

	
		Temp_SPI_init();
		PWM_Init();
		Sample_Timer_init();

		user_bt_init();
  //==============================================//



  //==============================================//		
	while(1)
	{

	SampleTime_1s();

	

	
	
	}
	
	
}
