#ifndef _Function_H
#define _Function_H

#include "STM32F4xx.h"
void GetTemp(void);
void AverTemp(void);
void SampleTime_1s(void);
void PID_controller(void);
void PID_Tuner(void);
uint32_t GetSector(uint32_t Address);
void Write_Flash(uint32_t Addr,uint32_t Data1,uint32_t Data2);
float Read_Flash(uint32_t Addr);
void PID_Fuzzy_SelfTuning(void);
float hinhthang(double x,float L,float C1,float C2,float R);
float FindMax(float x[]);
void PD_Fuzzy(void);
#endif
