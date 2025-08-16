#include <stdio.h>
#include <stdint.h>
#include "mb.h"
#include "uart_interface.h"

void Delay_ms(uint32_t nCount)
{
    uint32_t tcnt;
    while (nCount--)
    {
        tcnt = 144000 / 5;
        while (tcnt--){;}
    }
}

int main(void)
{
	RCC_ConfigPclk2(RCC_HCLK_DIV2);	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Modbus_DMA_Init();
	Modbus_USART_Init(115200);
	
	
	UCHAR mySlaveAddress = 0x01;//< 从机地址
	MBModify modify;
	eMBInit(MB_RTU, mySlaveAddress, 3, 115200, MB_PAR_NONE);
	eMBEnable();
	uint16_t* pdu = getPDUData();
	while (1)
	{
		Delay_ms(1);
		if (modbus_recv_flag == 1)
		{
			modbus_recv_flag = 0;
			pxMBFrameCBByteReceived();
			Update_Modbus_RO_Data();
		}
		
		eMBPoll(&modify);
		if (modbus_send_flag == 2)
		{//< 回复帧
				modbus_send_flag = 0;
				Modbus_DMA_ReEnable(MODBUS_DMA_TX_Channel,modbus_send_len);
				Modbus_DMA_ReEnable(MODBUS_DMA_RX_Channel,MODBUS_RX_MAXBUFF);
				Modbus_Respond(&modify);
		}
	}
}




