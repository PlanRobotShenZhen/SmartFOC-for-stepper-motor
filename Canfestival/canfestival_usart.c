/**
  ******************************************************************************
  * @file    USART/USART_Printf/main.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    24-July-2014
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "canfestival_usart.h"

/** @addtogroup STM32F0xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup USART_Printf
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
#define DEBUG_PORT USART1

void USART_Config(void);

//#ifdef __GNUC__
//  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
//     set to 'Yes') calls __io_putchar() */
//  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif /* __GNUC__ */
  
/* Private functions ---------------------------------------------------------*/
/**
  * @brief Configure the USART Device
  * @param  None
  * @retval None
  */
void usart_config(uint8_t port,  uint32_t BaudRate)
{ 
//    USART_InitTypeDef USART_InitStructure;
//    GPIO_InitTypeDef GPIO_InitStructure;
//    USART_TypeDef *InitPort = USART1; //默认是debug口
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//  
//    if (port == 1) {
//        RCC_APB2PeriphClockCmd (RCC_APB2Periph_USART1, ENABLE);
//        //GPIO MAP
//        //GPIO_PinAFConfig (GPIOA, GPIO_PinSource9, GPIO_AF_1);
//        //GPIO_PinAFConfig (GPIOA, GPIO_PinSource10, GPIO_AF_1);
//        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
//        InitPort = USART1;
//    } else if (port == 2) {
//        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
//        //GPIO MAP
//        //GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
//        //GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
//        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
//        InitPort = USART2;
//    }
//    USART_DeInit(InitPort);
//  /* Configure pins as AF pushpull */
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);

//  /* USARTx configured as follow:
//  - BaudRate = 115200 baud  
//  - Word Length = 8 Bits
//  - Stop Bit = 1 Stop Bit
//  - Parity = No Parity
//  - Hardware flow control disabled (RTS and CTS signals)
//  - Receive and transmit enabled
//  */
//  USART_InitStructure.USART_BaudRate = BaudRate;
//  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//  USART_InitStructure.USART_StopBits = USART_StopBits_1;
//  USART_InitStructure.USART_Parity = USART_Parity_No;
//  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//  
//  USART_Init(InitPort, &USART_InitStructure);
//  //USART_ITConfig(InitPort, USART_IT_RXNE, ENABLE);
//  USART_Cmd(InitPort,ENABLE);
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
//PUTCHAR_PROTOTYPE
//{
//  /* Place your implementation of fputc here */
//  /* e.g. write a character to the USART */
//  USART_SendData(DEBUG_PORT, (uint8_t) ch);

//  /* Loop until transmit data register is empty */
//  while (USART_GetFlagStatus(DEBUG_PORT, USART_FLAG_TXE) == RESET)
//  {}

//  return ch;
//}

void uart1SendChar(u8 ch)
{
//  while((USART1->SR & USART_FLAG_TC) == 0);
//  USART1->DR = (ch & (uint16_t)0x01FF);
}

//用于485总线
void uart2SendChar(u8 ch)
{
//  while((USART2->SR & USART_FLAG_TC) == 0);
//  USART2->DR = (ch & (uint16_t)0x01FF);
}

void uart2SendChars(u8 *str, u16 strlen)
{
//  u16 k= 0 ;
//  do {
//      uart2SendChar(*(str + k));
//      k++;
//  } while (k < strlen);
}

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
