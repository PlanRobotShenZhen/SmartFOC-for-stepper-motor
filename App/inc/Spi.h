#ifndef __SPI_H__
#define __SPI_H__

#include "n32g45x.h"
#include "n32g45x_conf.h"

#define SPI_Pin_CSN			GPIO_PIN_15
#define SPI_Pin_SCK			GPIO_PIN_3
#define SPI_Pin_MISO		GPIO_PIN_4
#define SPI_Pin_MOSI		GPIO_PIN_5

#define CSn1_H   GPIO_SetBits(GPIOA, SPI_Pin_CSN)//PA15 SPI NSSÒý½Å
#define CSn1_L   GPIO_ResetBits(GPIOA, SPI_Pin_CSN)

void Delay_Us(unsigned int i);
void delay_Ms(uint16_t nms);
//u32 MT6835_Angle_Read(void);
u16 SPI_ReadWriteByte(u16 TxData);
u16 MT6835_Read_Reg(u16 regaddr);
void MT6835_Init(void);
//double SPIGetDegreeMT6835(void);

#endif
