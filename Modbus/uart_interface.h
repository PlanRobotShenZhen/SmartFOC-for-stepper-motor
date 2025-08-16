#ifndef __UART_INTERFACE_H
#define __UART_INTERFACE_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "n32g45x.h"
#include "mb.h"
//modbus串口配置
#define MODBUS_USART           		USART3
#define MODBUS_USART_GPIO       	GPIOB
#define MODBUS_USART_CLK        	RCC_APB1_PERIPH_USART3
#define MODBUS_USART_GPIO_CLK   	RCC_APB2_PERIPH_GPIOB
#define MODBUS_USART_RxPin      	GPIO_PIN_11
#define MODBUS_USART_TxPin      	GPIO_PIN_10
#define MODBUS_USART_APBxClkCmd 	RCC_EnableAPB1PeriphClk
#define MODBUS_USART_IRQn  				USART3_IRQn
#define MODBUS_USART_IRQHandler  	USART3_IRQHandler

//modbus DMA配置
#define MODBUS_DMA					     	DMA1
#define MODBUS_DMA_RX_Channel     DMA1_CH3
#define MODBUS_DMA_TX_Channel     DMA1_CH2
#define MODBUS_DMA_TX_REMAP 			DMA1_REMAP_USART3_TX
#define MODBUS_DMA_RX_REMAP 			DMA1_REMAP_USART3_RX
#define MODBUS_DMA_CLK	 					RCC_AHB_PERIPH_DMA1

//DMA发送完成中断配置
#define MODBUS_DMA_TX_IRQn 				DMA1_Channel2_IRQn
#define MODBUS_DMA_TX_INT_TXC 		DMA1_INT_TXC2
#define MODBUS_DMA_TX_IRQHandler  DMA1_Channel2_IRQHandler

//485控制引脚配置
#define RS485_DE_GPIO       			GPIOB
#define RS485_DE_Pin      				GPIO_PIN_0
#define RS485_DE_GPIO_CLK   			RCC_APB2_PERIPH_GPIOB

//485发送与接收使能
#define SET_RS485_RX_ENABLE 			GPIO_ResetBits(RS485_DE_GPIO, RS485_DE_Pin)
#define SET_RS485_TX_ENABLE 			GPIO_SetBits(RS485_DE_GPIO, RS485_DE_Pin)
//读取485使能引脚
#define GET_RS485_DE_STATE        GPIO_ReadOutputDataBit(RS485_DE_GPIO,RS485_DE_Pin)

#define UART_TX_BUFFER_SIZE 64
#define UART_RX_BUFFER_SIZE 64

#define MODBUS_RX_MAXBUFF   256 		// 接收数据最大缓冲区
#define MODBUS_TX_MAXBUFF   256 		// 发送数据最大缓冲区

// PID存储起始序号
#define UdsPID_STARTINDEX   351
#define UqsPID_STARTINDEX   371
#define TorPID_STARTINDEX   391
#define VelPID_STARTINDEX   411
#define HodPID_STARTINDEX   431
#define PosPID_STARTINDEX   451



extern uint8_t modbus_recv_data[MODBUS_RX_MAXBUFF];			// 接收数据缓冲区
extern uint8_t modbus_send_data[MODBUS_TX_MAXBUFF];			// 发送数据缓冲区
extern uint32_t modbus_recv_flag;       								// 接收完成标志位
extern uint32_t modbus_recv_len;        								// 接收的数据长度
extern uint32_t modbus_send_flag;       								// 发送完成标志位
extern uint32_t modbus_send_len;        								// 发送的数据长度


void Modbus_USART_Init(unsigned int baud);
void Modbus_DMA_Init(void);
void Modbus_DMA_ReEnable(DMA_ChannelType* DMA_Channel,uint16_t len);
void Modbus_Respond(MBModify* modify);
void Update_Modbus_RO_Data(void);
void Modbus_Task(MBModify* modify_);
#ifdef __cplusplus
}
#endif

#endif /*__UART_INTERFACE_H */



