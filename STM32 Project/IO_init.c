#include "IO_init.h"


void SPI_init(void)
{
	
RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
 
SPI_InitTypeDef SPI_InitTypeDefStruct;
 
 
SPI_InitTypeDefStruct.SPI_Direction = SPI_Direction_1Line_Rx;
SPI_InitTypeDefStruct.SPI_Mode = SPI_Mode_Master;
SPI_InitTypeDefStruct.SPI_DataSize = SPI_DataSize_16b;
SPI_InitTypeDefStruct.SPI_CPOL = SPI_CPOL_High;
SPI_InitTypeDefStruct.SPI_CPHA = SPI_CPHA_2Edge;
SPI_InitTypeDefStruct.SPI_NSS = SPI_NSS_Soft;
SPI_InitTypeDefStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
SPI_InitTypeDefStruct.SPI_FirstBit = SPI_FirstBit_MSB;
 
SPI_Init(SPI2, &SPI_InitTypeDefStruct);
 
 
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);
 
GPIO_InitTypeDef GPIO_InitTypeDefStruct;
 
GPIO_InitTypeDefStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
GPIO_InitTypeDefStruct.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitTypeDefStruct.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitTypeDefStruct.GPIO_OType = GPIO_OType_PP;
GPIO_InitTypeDefStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
GPIO_Init(GPIOB, &GPIO_InitTypeDefStruct);
 
GPIO_InitTypeDefStruct.GPIO_Pin = GPIO_Pin_6;
GPIO_InitTypeDefStruct.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitTypeDefStruct.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitTypeDefStruct.GPIO_PuPd = GPIO_PuPd_UP;
GPIO_InitTypeDefStruct.GPIO_OType = GPIO_OType_PP;
GPIO_Init(GPIOB, &GPIO_InitTypeDefStruct);
 
GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
 
GPIO_SetBits(GPIOB, GPIO_Pin_6);
 
 
SPI_Cmd(SPI2, ENABLE);
 
}










