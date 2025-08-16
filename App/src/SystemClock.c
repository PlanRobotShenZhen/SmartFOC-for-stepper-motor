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
 * @file systemclock.c
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#include "ExternGlobals.h"

void SystemClk_Init(void);  		// 系统时钟初始化
void Gpio_Init(void);       		// GPIO初始化
void system_tick_init(void);		//滴答定时器初始化

// ========================================================================
// 函数名称：SystemClk_Init
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：系统时钟初始化
// ========================================================================
void SystemClk_Init()
{
    ErrorStatus HSEStartUpStatus;

    //RCC system reset(for debug purpose)
    RCC_DeInit();

    //Enable HSE
    RCC_ConfigHse(RCC_HSE_ENABLE);
    //Wait till HSE is ready
    HSEStartUpStatus = RCC_WaitHseStable();

    if(HSEStartUpStatus == SUCCESS)
    {
        //HCLK = SYSCLK
        RCC_ConfigHclk(RCC_SYSCLK_DIV1);
        //PCLK2 = HCLK
        RCC_ConfigPclk2(RCC_HCLK_DIV1);
        //PCLK1 = HCLK/2
        RCC_ConfigPclk1(RCC_HCLK_DIV2);

        //icache and prefetch configure
        FLASH_iCacheCmd(FLASH_iCache_EN);
        FLASH_PrefetchBufSet(FLASH_PrefetchBuf_DIS);

        if(MAIN_FREQUENCY == 72000000)
        {
            //Flash wait state,2
            FLASH_SetLatency(FLASH_LATENCY_2);
            //PLLCLK = 8MHz * 9 = 72 MHz
            RCC_ConfigPll(RCC_PLL_SRC_HSE_DIV1, RCC_PLL_MUL_9);
        }
        else if(MAIN_FREQUENCY == 96000000)
        {
            //Flash wait state 3
            FLASH_SetLatency(FLASH_LATENCY_3);
            //PLLCLK = 8MHz * 12 = 96 MHz
            RCC_ConfigPll(RCC_PLL_SRC_HSE_DIV1, RCC_PLL_MUL_12);
        }
        else
        {
            //Flash wait state,4
            FLASH_SetLatency(FLASH_LATENCY_4);
            //PLLCLK = 8MHz * 18 = 144 MHz
            RCC_ConfigPll(RCC_PLL_SRC_HSE_DIV1, RCC_PLL_MUL_18);
        }

        //Enable PLL
        RCC_EnablePll(ENABLE);

        //Wait till PLL is ready
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRD) == RESET)
        {
        }

        //Select PLL as system clock source
        RCC_ConfigSysclk(RCC_SYSCLK_SRC_PLLCLK);

        //Wait till PLL is used as system clock source
        while(RCC_GetSysclkSrc() != 0x08)
        {
        }
    }

    //GPIO clocks enable
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO | RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_GPIOC | RCC_APB2_PERIPH_GPIOD |
                            RCC_APB2_PERIPH_GPIOE | RCC_APB2_PERIPH_GPIOF | RCC_APB2_PERIPH_GPIOG, ENABLE);
    GPIO_ConfigPinRemap(GPIO_RMP_SW_JTAG_SW_ENABLE, ENABLE);

    //Enable ADC1, ADC2, ADC3 and ADC4 clocks
//   RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC1 | RCC_AHB_PERIPH_ADC2 | RCC_AHB_PERIPH_ADC3 | RCC_AHB_PERIPH_ADC4, ENABLE);

    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC1 | RCC_AHB_PERIPH_ADC2, ENABLE);
    //RCC_ADCHCLKDiv
    RCC_ConfigAdcHclk(RCC_ADCHCLK_DIV8);

    //Enable COMP OPA clocks
    //  RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_OPAMP | RCC_APB1_PERIPH_COMP | RCC_APB1_PERIPH_COMP_FILT, ENABLE);

    //TIM1,TIM8
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_TIM1, ENABLE);
    //RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_TIM8, ENABLE);
    //UART  clocks enable
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_UART4, ENABLE);

    /* Enable peripheral clocks --------------------------------------------------*/
    /* GPIOA, GPIOB and SPI1 clock enable */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_SPI1, ENABLE);

    /* SPI3 Periph clock enable */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_SPI3, ENABLE);
}

void Gpio_Init(void)
{
    GPIO_InitType GPIO_InitStructure;							

    //GPIO for LED
    GPIO_InitStructure.Pin = GPIO_PIN_13 | GPIO_PIN_14| GPIO_PIN_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);
    LED1_OFF;   //上电默认暗
    LED2_OFF;

		GPIO_InitStructure.Pin =  GPIO_PIN_12;                // ALL_L
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure); 
    GPIO_WriteBit(GPIOA, GPIO_PIN_12, Bit_RESET);
		

    //FOC with TIM1 Channel 1, 1N, 2, 2N, 3, 3N and 4 Output
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10| GPIO_PIN_11;  //wjj
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
		
    GPIO_InitStructure.Pin =  GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
	

    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.Pin = GPIO_PIN_5;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
		
    GPIO_InitStructure.Pin = GPIO_PIN_4;             //Vdc
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
		
    GPIO_InitStructure.Pin = GPIO_PIN_7;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

    //SPI==>MT6835
    GPIO_InitStructure.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_15;       //SPI2_NSS
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
    GPIO_SetBits(GPIOA, GPIO_PIN_15);
		
    GPIO_InitStructure.Pin = GPIO_PIN_12;     //CAL_EN 磁感应编码的计算使能开关
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB, GPIO_PIN_12);		
}
// ========================================================================
// 函数名称：system_tick_init
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：用于执行低频任务
// ========================================================================
void system_tick_init(void)
{
    SysTick_Config(MAIN_FREQUENCY / 10000);				
    SysTick->CTRL |= 0x00000004U;									
    NVIC_SetPriority(SysTick_IRQn, 2);						
}


void SPI_Timer_Init(void)
{
    uint16_t Prescaler = 35;  // 144/(35+1)=4M
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM4, ENABLE);
    TIM_TimeBaseInitType TIM_TimeBaseStructure;
    NVIC_InitType NVIC_InitStructure;
    TIM_InitTimBaseStruct(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.Period    = 125;  //125/4M=1/32k
    TIM_TimeBaseStructure.Prescaler = Prescaler;
    TIM_TimeBaseStructure.ClkDiv    = 0;
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;
    TIM_InitTimeBase(TIM4, &TIM_TimeBaseStructure);
    TIM_ConfigPrescaler(TIM4, Prescaler, TIM_PSC_RELOAD_MODE_IMMEDIATE); //立即重装载
    TIM4->CNT = 0;
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_ConfigInt(TIM4, TIM_INT_UPDATE, ENABLE);
    TIM_Enable(TIM4, ENABLE);
}


#include "Spi.h"
void TIM4_IRQHandler(void)
{
    if(TIM_GetIntStatus(TIM4, TIM_INT_UPDATE) != RESET)
    {
        TIM_ClrIntPendingBit(TIM4, TIM_INT_UPDATE);
        if((SpiReadState == 0) && (SpiReadState2 == 0)&&(UartMode.Mode != 6))
        {
            CSn1_L;       //使能SPI传输，在中断里失能
            Delay_Us(1);  //延时约600-700ns
            SpiReadState2 = 2;
            MT6835_Read_Reg(0xA003);
        }
    }
}

//用于测试耗时
void Timer_Init(void){
	  uint16_t Prescaler = 8;  // 144/(8+1)=16M
	  RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM2, ENABLE); 
    TIM_TimeBaseInitType TIM_TimeBaseStructure;
	  NVIC_InitType NVIC_InitStructure;
    TIM_InitTimBaseStruct(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.Period    = 65535;
    TIM_TimeBaseStructure.Prescaler = Prescaler;
    TIM_TimeBaseStructure.ClkDiv    = 0;
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;
    TIM_InitTimeBase(TIM2, &TIM_TimeBaseStructure);
    TIM_ConfigPrescaler(TIM2, Prescaler, TIM_PSC_RELOAD_MODE_IMMEDIATE); //立即重装载  	
	  TIM2->CNT = 0;
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	  TIM_ConfigInt(TIM2, TIM_INT_UPDATE, ENABLE);
    TIM_Enable(TIM2, ENABLE);
}
volatile uint32_t Timer_Loop = 0;

void TIM2_IRQHandler(void)
{
    if (TIM_GetIntStatus(TIM2, TIM_INT_UPDATE) != RESET)
    {
        TIM_ClrIntPendingBit(TIM2, TIM_INT_UPDATE);
        Timer_Loop++;
    }
}
