#include "Function.h"
#include "delay.h"
//============================ Temp Sense Variables ============================//
	uint32_t RawTemp;
	double AvrAmbTemp;
	double AvrThermoTemp;
	double AmbTemp;
	double ThermoTemp;
	char *Message;
	int count;
	uint8_t err=0;
	
//============================ PID Controller Variables ============================//

double Err,PreErr;
double DesTemp=120;
double P,I,D,PID;
float Kp=73,Ki=0.1,Kd=0;
float SampleTime=3;
uint32_t PWMValue;
//============================ Fuzzy Controller Variables ============================//
double U;
int16_t PWMValueF;



//============================ PID Tuner Variable ============================//

int16_t Duty_Tuning=10000;

double  Possible_MaxT[10];
double Possible_MinT[10];
double MaxT[3];
double MinT[3];

int16_t CountPosMax=0,CountMax=-1,CountPosMin=0,CountMin=-1,Cycle=1;

uint32_t Up_period, Low_period,Pu;

double Ku;


//======================Write Flash Variable===========================//
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;



#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_4   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_SECTOR_5   /* End @ of user Flash area */


#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */

uint32_t StartSector = 0, EndSector = 0, SectorCounter = 0;

  

//================================ Others Variables ==============================//
	uint8_t Timer6_IT;
extern	uint8_t Tuning;
	uint8_t FS=1;



////////////////////////////////////////////////////////////////////////
////////////// Begin Function Program /////////////////////////////////
//////////////////////////////////////////////////////////////////////
void GetTemp()
{
	int i=0;
	uint32_t tmp=0;
	 RawTemp=0;
	 AmbTemp=0;
	 ThermoTemp=0;
	GPIO_ResetBits(GPIOB, GPIO_Pin_6);
	for (i=31; i>=0; i--)
            {
                    GPIO_ResetBits(GPIOB, GPIO_Pin_13);
										
									
                    if (GPIO_ReadInputDataBit( GPIOB, GPIO_Pin_14))   
                              RawTemp |= (1 << i);
                   GPIO_SetBits(GPIOB, GPIO_Pin_13);
									
										
            }
		GPIO_SetBits(GPIOB, GPIO_Pin_6);
		if ((RawTemp & 0x07)==0)
		{
			count++;
			err=0;
			Message="Working";
		//==================//
		tmp=(RawTemp >> 4)& 0x00fff;
		if ((tmp>>11)==0)
		{
			AmbTemp=((double)(tmp))*0.0625;
		}
		else
		{
			tmp=((~tmp)+1)&0x7ff;
			AmbTemp=-(((double)(tmp))*0.0625);
		}
		//=================//
		tmp=(RawTemp >> 18)& 0x03fff;
			if ((tmp>>13)==0)
		{
			ThermoTemp=((double)(tmp))*0.25;
		}
		else
		{
			tmp=((~tmp)+1)&0x1fff;
			ThermoTemp=-(((double)(tmp))*0.25);
		}
		}
	else
	{
		err=1;
		switch(RawTemp & 0x07)
		{
			case 1:
				Message = " Open Circuit ";
			break;
			case 2:
				Message = " Short to GND ";
			break;
			case 3:
				Message = " Short to VCC ";
			break;
			default:
				break;
		}
	}
}

void AverTemp(void)
{
	
	int j;
	double tmpAvrAmbTemp=0;
	double tmpAvrThermoTemp=0;
	
	for(j=0;j<30;j++)
	{
		GetTemp();
		if (err==0)
		{
		tmpAvrAmbTemp+=AmbTemp;
		tmpAvrThermoTemp+=ThermoTemp;	
	}
	}
	if (count>20)
	{
	AvrAmbTemp=tmpAvrAmbTemp/count;
	AvrThermoTemp=tmpAvrThermoTemp/count;	
	count=0;
	}
}
//============================++===============================//
void SampleTime_1s()
{
if (Timer6_IT==1)
{
	if (FS==1)
	{
	Kp=(float)(Read_Flash( FLASH_USER_START_ADDR))/1000000;
	Ki=(float)(Read_Flash(FLASH_USER_START_ADDR+4))/1000000;
		if (Kp <=0 & Ki<=0)
		{
	Tuning=1;
		}
	}
		AverTemp();
	if (Tuning==0)
	{
		Err=DesTemp - AvrThermoTemp;
	
	PID_Fuzzy_SelfTuning();
	PID_controller();


	PD_Fuzzy();
	PreErr=Err;
		
		if (AvrThermoTemp >= 0*DesTemp)
		{
			TIM_SetCompare1(TIM1,PWMValue);
		}
		else 
		{
			TIM_SetCompare1(TIM1,PWMValueF);
		}
	}
	else
	{
		if (AvrThermoTemp<=0.8*DesTemp | Pu>0)
		{
	PID_Tuner();
		}
	}
	FS=0;
}

Timer6_IT=0;

}

//=============================++===============================//
void PID_controller()
{
	P=Kp*Err;
	I+=Ki*(Err)*SampleTime;
	D=(Kd*(Err-PreErr))/SampleTime;
	PID = P+I+D;

	if (PID>10000)
		PWMValue=10000;
	else if (PID<0)
		PWMValue=0;
	else 
		PWMValue=(int)(PID);
		
	
	
	

}
//============================**===========================//
void PID_Tuner()
{

	double Trigger;
	int i;
		if( AvrThermoTemp >0.6*DesTemp)
	{
		Duty_Tuning=3000;
	}
	Trigger= AvrThermoTemp-0.9*DesTemp;
	//=============== condition ==============//
		if (CountMax==2 & CountMin==2)
	{
		Ku=(double)((Duty_Tuning*2))/(3.141592654*(MaxT[1]-MinT[2]));
		Kp=0.4*Ku;
		Ki=0.48*Ku/Pu;	
		Write_Flash(FLASH_USER_START_ADDR,Kp*1000000,Ki*1000000);
		TIM_SetCompare1(TIM1,0);
		Tuning=0;
	}
	else
	{
	Pu=Up_period + Low_period;
	//=========== upper cycle=============//
	if (Trigger>=0.5)
	{
			PWMValue=0;
		if (Cycle==0)
		{
			CountMax++;
			Up_period=0;
			for (i=0;i<=9;i++)
			{
				Possible_MaxT[i]=0;
			}
			CountPosMax=0;
		}
		Up_period++;
		
		if (CountPosMax==10)
		{
			// finding max temp in 10 sample //
		MaxT[CountMax]=Possible_MaxT[0];
		for (i=1;i<=9;i++)
		{
		if (MaxT[CountMax]<Possible_MaxT[i])
		{
			MaxT[CountMax]=Possible_MaxT[i];
		}
		}
		//========================================///
			for (i=0;i<=9;i++)
			{
				Possible_MaxT[i]=0;
			}
			Possible_MaxT[0]=MaxT[CountMax];
			CountPosMax=1;
		}
		if (CountPosMax<=9)
		{
				
			Possible_MaxT[CountPosMax]=AvrThermoTemp;		
			CountPosMax++;	
		}
	
		Cycle=1;
	}
	
		
	//=========== lower cycle=============//
	else if (Trigger<=-0.5)
	{
		PWMValue=Duty_Tuning;
		
			if (Cycle==1)
			{
			CountMin++;
			Low_period=0;
			for (i=0;i<=9;i++)
			{
				Possible_MinT[i]=0;
			}
			CountPosMin=0;
			}
		Low_period++;
		if (CountPosMin==10)
		{
			// finding min temp in 10 sample //
		MinT[CountMin]=Possible_MinT[0];
		for (i=1;i<=9;i++)
		{
		if (MinT[CountMin]>Possible_MinT[i])
		{
			MinT[CountMin]=Possible_MinT[i];
		}
		}
		//========================================///
			for (i=0;i<=9;i++)
			{
				Possible_MinT[i]=0;
			}
			Possible_MinT[0]=MinT[CountMin];
			CountPosMin=1;
		}
		if (CountPosMin<=9)
		{
					
			Possible_MinT[CountPosMin]=AvrThermoTemp;		
			CountPosMin++;
		}
	
		Cycle=0;
	}
	TIM_SetCompare1(TIM1,PWMValue);
		
	}

}
//=========================================================//
float hinhthang(double x,float L,float C1,float C2,float R)
{
	float y;
if (x<L) 
{
y=0;
}
else if (x<C1)
{
y =(x-L)/(C1-L);
}
else if (x<C2)
{
y=1;
}
else if (x<R)
{
y=(R-x)/(R-C2);
}
else y=0;
return(y);

}
//=========================================================//
void PID_Fuzzy_SelfTuning()
{
	int i,j;
double e_scale,de_scale;	
double E[7],DE[5];
double P0,P1,P2,P3,P4,P5,P6,P7,P8,P9;
double beta[35];


e_scale=Err/(DesTemp-AvrAmbTemp);
de_scale=(Err - PreErr)/SampleTime;
// Declare input e fuzzy //
E[0]=hinhthang(e_scale,-10,-9,-0.5,-0.1);
E[1]=hinhthang(e_scale,-0.5,-0.1,-0.1,0);
E[2]=hinhthang(e_scale,-0.1,0,0,0.1);
E[3]=hinhthang(e_scale,0,0.1,0.1,0.2);
E[4]=hinhthang(e_scale,0.1,0.2,0.2,0.5);
E[5]=hinhthang(e_scale,0.2,0.5,0.5,0.7);
E[6]=hinhthang(e_scale,0.5,0.7,9,10.0);
	
	
	
// Declare input de fuzzy //
DE[0]=hinhthang(de_scale,-10.0,-9,-0.7,-0.3);
DE[1]=hinhthang(de_scale,-0.7,-0.3,-0.3,0);
DE[2]=hinhthang(de_scale,-0.3,0,0,0.3);
DE[3]=hinhthang(de_scale,0,0.3,0.3,0.7);
DE[4]=hinhthang(de_scale,0.3,0.7,9,10.0);

// Declare output Kp & Ki fuzzy //
	
P0=0.02;
P1=0.04;
P2=0.08;
P3=0.16;
P4=0.25;
P5=0.4;
P6=0.5;
P7=0.6;
P8=0.8;
P9=1;

// Calculate Beta with MAX PROD //

for (i=0;i<=6;i++)
{
	for (j=0;j<=4;j++)
	{
	beta[i*5+j]=E[i]*DE[j];
	}
	j=0;
}

Kp=1+300*(
beta[0]*P5+ beta[1]*P4+ beta[2]*P3+ beta[3]*P4+ beta[4]*P3+
beta[5]*P6+ beta[6]*P5+ beta[7]*P4+ beta[8]*P1+ beta[9]*P0+
beta[10]*P5+ beta[11]*P5+ beta[12]*P6+ beta[13]*P7+ beta[14]*P6+ 
beta[15]*P3+ beta[16]*P4+ beta[17]*P5+ beta[18]*P9+ beta[19]*P9+
beta[20]*P2+ beta[21]*P3+ beta[22]*P4+ beta[23]*P9+ beta[24]*P9+
beta[25]*P5+ beta[26]*P6+ beta[27]*P7+ beta[28]*P8+ beta[29]*P9+
beta[30]*P7+ beta[31]*P8+ beta[32]*P9+ beta[33]*P9+ beta[34]*P9);


Ki=(
beta[0]*P2+ beta[1]*P1+ beta[2]*P0+ beta[3]*P0+ beta[4]*P0+
beta[5]*P3+ beta[6]*P2+ beta[7]*P1+ beta[8]*P0+ beta[9]*P0+
beta[10]*P5+ beta[11]*P5+ beta[12]*P4+ beta[13]*P3+ beta[14]*P2+ 
beta[15]*P5+ beta[16]*P6+ beta[17]*P7+ beta[18]*P9+ beta[19]*P8+
beta[20]*P2+ beta[21]*P1+ beta[22]*P0+ beta[23]*P9+ beta[24]*P8+
beta[25]*P2+ beta[26]*P1+ beta[27]*P0+ beta[28]*P7+ beta[29]*P8+
beta[30]*P2+ beta[31]*P1+ beta[32]*P0+ beta[33]*P8+ beta[34]*P9);

Kd=10000*(
beta[0]*P9+ beta[1]*P8+ beta[2]*P0+ beta[3]*P0+ beta[4]*P0+
beta[5]*P7+ beta[6]*P6+ beta[7]*P5+ beta[8]*P1+ beta[9]*P2+
beta[10]*P8+ beta[11]*P7+ beta[12]*P6+ beta[13]*P3+ beta[14]*P4+ 
beta[15]*P3+ beta[16]*P2+ beta[17]*P1+ beta[18]*P7+ beta[19]*P8+
beta[20]*P2+ beta[21]*P1+ beta[22]*P0+ beta[23]*P5+ beta[24]*P6+
beta[25]*P2+ beta[26]*P1+ beta[27]*P0+ beta[28]*P3+ beta[29]*P4+
beta[30]*P0+ beta[31]*P0+ beta[32]*P0+ beta[33]*P1+ beta[34]*P2);

	
	
	
	
}
//=================================================//
void PD_Fuzzy()
{
double e_scale,de_scale;	
double muy_NB_e,muy_NS_e,muy_ZE_e,muy_PS_e,muy_PB_e,muy_NB_de,muy_NS_de,muy_ZE_de,muy_PS_de,muy_PB_de;
double P0,P1,P2,P3,P4;
double beta[25];


e_scale=Err/(DesTemp-AvrAmbTemp);
de_scale=(Err - PreErr)/SampleTime;
// Declare input e fuzzy //
muy_NB_e=hinhthang(e_scale,-10.0,-9,-0.8,-0.5);
muy_NS_e=hinhthang(e_scale,-0.8,-0.5,-0.5,0);
muy_ZE_e=hinhthang(e_scale,-0.5,0,0,0.5);
muy_PS_e=hinhthang(e_scale,0,0.5,0.5,0.8);
muy_PB_e=hinhthang(e_scale,0.5,0.8,9,10.0);


// Declare input de fuzzy //
muy_NB_de=hinhthang(de_scale,-10.0,-9,-0.5,-0.25);
muy_NS_de=hinhthang(de_scale,-0.5,-0.25,-0.25,0);
muy_ZE_de=hinhthang(de_scale,-0.25,0,0,0.25);
muy_PS_de=hinhthang(de_scale,0,0.25,0.25,0.5);
muy_PB_de=hinhthang(de_scale,0.25,0.5,9,10.0);

// Declare output U fuzzy //
	
P0=0;
P1=0.1;
P2=0.25;
P3=0.6;
P4=1;



// Calculate Beta with MAX PROD //

beta[0]=muy_NB_e*muy_NB_de;
beta[1]=muy_NB_e*muy_NS_de;
beta[2]=muy_NB_e*muy_ZE_de;
beta[3]=muy_NB_e*muy_PS_de;
beta[4]=muy_NB_e*muy_PB_de;

beta[5]=muy_NS_e*muy_NB_de;
beta[6]=muy_NS_e*muy_NS_de;
beta[7]=muy_NS_e*muy_ZE_de;
beta[8]=muy_NS_e*muy_PS_de;
beta[9]=muy_NS_e*muy_PB_de;

beta[10]=muy_ZE_e*muy_NB_de;
beta[11]=muy_ZE_e*muy_NS_de;
beta[12]=muy_ZE_e*muy_ZE_de;
beta[13]=muy_ZE_e*muy_PS_de;
beta[14]=muy_ZE_e*muy_PB_de;

beta[15]=muy_PS_e*muy_NB_de;
beta[16]=muy_PS_e*muy_NS_de;
beta[17]=muy_PS_e*muy_ZE_de;
beta[18]=muy_PS_e*muy_PS_de;
beta[19]=muy_PS_e*muy_PB_de;

beta[20]=muy_PB_e*muy_NB_de;
beta[21]=muy_PB_e*muy_NS_de;
beta[22]=muy_PB_e*muy_ZE_de;
beta[23]=muy_PB_e*muy_PS_de;
beta[24]=muy_PB_e*muy_PB_de;




U=10000*(
beta[0]*P0+ beta[1]*P0+ beta[2]*P0+ beta[3]*P0+ beta[4]*P0+
beta[5]*P0+ beta[6]*P0+ beta[7]*P0+ beta[8]*P0+ beta[9]*P1+
beta[10]*P0+ beta[11]*P0+ beta[12]*P0+ beta[13]*P1+ beta[14]*P2+
beta[15]*P1+ beta[16]*P2+ beta[17]*P3+ beta[18]*P4+ beta[19]*P4+
beta[20]*P3+ beta[21]*P4+ beta[22]*P4+ beta[23]*P4+ beta[24]*P4
);
	if (U>10000)
		PWMValueF=10000;
	else if (U<0)
		PWMValueF=0;
	else 
		PWMValueF=(int)(U);
		


}
//=================================================//
float FindMax(float x[])
{
		int i;
	float Max=x[0];
	
		for (i=0;i<=4;i++)
		{
		if (Max<x[i])
		{
			Max=x[i];
		}
		}
	return(Max);
	
}



//=================================================//
uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;
  
  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_Sector_0;  
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_Sector_1;  
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_Sector_2;  
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_Sector_3;  
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_Sector_4;  
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_Sector_5;  
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_Sector_6;  
  }
  else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_Sector_7;  
  }
  else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_Sector_8;  
  }
  else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_Sector_9;  
  }
  else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_Sector_10;  
  }
  else/*(Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11))*/
  {
    sector = FLASH_Sector_11;  
  }

  return sector;
}
//========================================================//
void Write_Flash(uint32_t Addr,uint32_t Data1,uint32_t Data2)
{

  FLASH_Unlock();  
  
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR); 

  StartSector = GetSector(FLASH_USER_START_ADDR);
  EndSector = GetSector(FLASH_USER_END_ADDR);

  for (SectorCounter = StartSector; SectorCounter < EndSector; SectorCounter += 8)
  {
    
    if (FLASH_EraseSector(SectorCounter, VoltageRange_3) != FLASH_COMPLETE)
    { 
      
      while (1)
      {
      }
    }
  }

  Addr = FLASH_USER_START_ADDR;


    if (FLASH_ProgramWord(Addr, Data1) == FLASH_COMPLETE)
    {
      Addr = Addr + 4;
    }
    else
    {  
      while (1)
      {}
    }
		  if (FLASH_ProgramWord(Addr, Data2) == FLASH_COMPLETE)
    {
      Addr = Addr + 4;
    }
    else
    {  
      while (1)
      {}
    }
  
  FLASH_Lock(); 
}

//==========================================================//
float Read_Flash(uint32_t Addr)
{
	int data;
    data= *(__IO uint32_t*)Addr;
return data;  
}
