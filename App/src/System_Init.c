/*****************************************************************************
 * Copyright (c) 2019, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file system_init.c
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#include "SystemDefine.h"
#include "ExternGlobals.h"
#include "uart_interface.h"
#include "drv8323.h"
#include "ExternGlobals.h"
#include "Function.h"


void System_Init(void);
uint16_t  ADC_ConvertedValue[ADC_CHANNEL_Number];
uint16_t* address_adc_value[2];

void EnternExMode(void)							//启动电源拓展模式，用于低功耗
{
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR, ENABLE);
    PWR->CTRL3 |= 0x00000001;       //启动电源拓展模式，用于低功耗等
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR, DISABLE);

}
// ========================================================================
// 函数名称：System_Init
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：系统初始化
// ========================================================================


extern void MT6835_Init(void);
extern void SPI_Timer_Init(void);

void System_Init()				//系统初始化
{

    EnternExMode(); 			//启动电源拓展模式，用于低功耗
    SystemClk_Init();     //系统时钟和外设时钟初始化
	
					/* This instruction raises the execution priority to 0. This prevents all
           exceptions with configurable priority from activating, other than through
           the HardFault fault escalation mechanism. */
	
    __disable_irq();			//用于全局禁止中断
    system_tick_init();   //滴答定时器初始化
    Gpio_Init();          //外设IO初始化

    //DRV8323_GPIO_Init(); //DRV IO初始化
    Pwm_Init();          	//TIM1 PWM初始化
    Adc_Init();          	//ADC初始化

    MT6835_Init();       	//编码器SPI初始化
    SPI_Timer_Init();			//SPI_Timer初始化
	  
    Modbus_DMA_Init();   //串口modbus DMA初始化
    Modbus_USART_Init(115200); //modbus IO和串口初始化
		
	        /* This instruction will allow all exceptions with configurable priority to
           be activated. */
					 
    __enable_irq();				//用于开启 全局中断
}


// ========================================================================
// 函数名称：SysTick_Handler
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：滴答时钟中断，处理低频任务
// ========================================================================

#include "Timer1.h"

#define M_ENABLE 1

extern TIMER timer;
void SysTick_Handler(void)				//系统滴答计时器来控制 下面的功能
{
    Timeing(&timer);							//计时间
    SpeedCalculate();							//速度计算					
    MotorControl(); // 1K   HZ    电机控制（开环控制 零点辨识等等）

}

// ========================================================================
// 函数名称：ADC1_2_IRQHandler
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：ADC1-2采集数据完成中断
// ========================================================================
short CurrentValue[3] = {0};
void ADC1_2_IRQHandler(void)
{
    if(ADC_GetIntStatus(ADC2, ADC_INT_JENDC) == SET)
    {
        ADC_ClearFlag(ADC2, ADC_FLAG_JENDC);
        if(TIM1->CTRL1 & 0x0010)   //上溢出，对应下管高电平
        {         
            ADC_ConvertedValue[0] = ADC_GetInjectedConversionDat(ADC2, ADC_INJ_CH_4); // IW
					
					  ADC_ConvertedValue[1] = ADC_GetInjectedConversionDat(ADC2, ADC_INJ_CH_2); // IU
					  ADC_ConvertedValue[2] = ADC_GetInjectedConversionDat(ADC2, ADC_INJ_CH_1); // IV
					
					  CurrentValue[1] = (short)2048 - (short)((ADC_ConvertedValue[1]));
			      CurrentValue[2] = (short)2048 - (short)((ADC_ConvertedValue[2]));

            SystemError.ImeasA = CurrentValue[2];
            SystemError.ImeasB = CurrentValue[1];

            CurrentLoopISR();
        }
    }
    else if(ADC_GetIntStatus(ADC2, ADC_INT_AWD) == SET)
    {
        ADC_ClearFlag(ADC2, ADC_FLAG_AWDG);
    }
}


// ========================================================================
// 函数名称：TIM1_BRK_UP_TRG_COM_IRQHandler
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：过流刹车中断函数
// ========================================================================
void TIM1_BRK_UP_TRG_COM_IRQHandler()
{
    if(TIM_GetIntStatus(TIM1, TIM_INT_BREAK) != RESET)
    {
        TIM_ClrIntPendingBit(TIM1, TIM_INT_BREAK);
    }

}
