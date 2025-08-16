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
 * @file pwm.c
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
//#include "SystemDefine.h"
//#include "DeviceConfig.h"
//#include "Function.h"

#include "ExternGlobals.h"
void Pwm_Init(void);						// Pwm初始化
void Pwm_II_Init(void);					
void PwmDutySet_I(uint8_t num,int16_t duty1,int16_t duty2,int16_t duty3);// 设置占空比
void AllPwmShut(uint8_t num);  				// 六管全关
void AllPwmOpen(uint8_t num);  				// 六管全开
void SetPwmFreq(uint16_t Freq); 			// 设置pwm频率
void Brake(uint8_t num);							// 刹车

// ========================================================================
// 函数名称：Pwm_Init
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：PWM初始化
// ========================================================================
void Pwm_Init(void)
{
	TIM_TimeBaseInitType TIM1_TimeBaseStructure;
	OCInitType TIM1_OCInitStructure;
	TIM_BDTRInitType TIM1_BDTRInitStructure;
		
	uint16_t TimerPeriod = 0;

	TimerPeriod = (MAIN_FREQUENCY / (PWM_FREQUENCY1*2)) - 1;  //4499
	
	//Time Base configuration
	TIM_DeInit(TIM1);
	TIM_InitTimBaseStruct(&TIM1_TimeBaseStructure);
	TIM1_TimeBaseStructure.Prescaler = 0;
	TIM1_TimeBaseStructure.CntMode = TIM_CNT_MODE_CENTER_ALIGN1;
	TIM1_TimeBaseStructure.Period = TimerPeriod;
	TIM1_TimeBaseStructure.ClkDiv = TIM_CLK_DIV1;
	TIM1_TimeBaseStructure.RepetCnt = 0;
	
	TIM_InitTimeBase(TIM1, &TIM1_TimeBaseStructure);
	
	//Channel 1, 2,3 in PWM mode
	TIM_InitOcStruct(&TIM1_OCInitStructure);
	TIM1_OCInitStructure.OcMode = TIM_OCMODE_PWM1;
	TIM1_OCInitStructure.OutputState = TIM_OUTPUT_STATE_DISABLE; 
	TIM1_OCInitStructure.OutputNState = TIM_OUTPUT_NSTATE_DISABLE;                  
	TIM1_OCInitStructure.Pulse = (TimerPeriod>>1);
	TIM1_OCInitStructure.OcPolarity = TIM_OC_POLARITY_HIGH;
	TIM1_OCInitStructure.OcNPolarity = TIM_OCN_POLARITY_HIGH; 
	TIM1_OCInitStructure.OcIdleState = TIM_OC_IDLE_STATE_RESET;
	TIM1_OCInitStructure.OcNIdleState = TIM_OC_IDLE_STATE_RESET;  
	
	
  #if ABAB == 1
		TIM_InitOc1(TIM1, &TIM1_OCInitStructure); 
		TIM_InitOc2(TIM1, &TIM1_OCInitStructure);
		TIM1_OCInitStructure.OcPolarity = TIM_OC_POLARITY_LOW;  
		TIM_InitOc3(TIM1, &TIM1_OCInitStructure);
		TIM_InitOc4(TIM1, &TIM1_OCInitStructure);
	#else  //AABB
	  TIM_InitOc1(TIM1, &TIM1_OCInitStructure); 
		TIM_InitOc3(TIM1, &TIM1_OCInitStructure);
		TIM1_OCInitStructure.OcPolarity = TIM_OC_POLARITY_LOW;  
		TIM_InitOc2(TIM1, &TIM1_OCInitStructure);
		TIM_InitOc4(TIM1, &TIM1_OCInitStructure);
	#endif
	TIM_SelectOutputTrig(TIM1, TIM_TRGO_SRC_UPDATE);   //选择发出的触发信号
	//Enables the TIM1 Preload on CC1,CC2,CC3,CC4 Register
	TIM_ConfigOc1Preload(TIM1, TIM_OC_PRE_LOAD_ENABLE);
	TIM_ConfigOc2Preload(TIM1, TIM_OC_PRE_LOAD_ENABLE);
	TIM_ConfigOc3Preload(TIM1, TIM_OC_PRE_LOAD_ENABLE);
	TIM_ConfigOc4Preload(TIM1, TIM_OC_PRE_LOAD_ENABLE);

	//Automatic Output enable, Break, dead time and lock configuration
	TIM1_BDTRInitStructure.OssrState = TIM_OSSR_STATE_ENABLE;
	TIM1_BDTRInitStructure.OssiState = TIM_OSSI_STATE_ENABLE;
	TIM1_BDTRInitStructure.LockLevel = TIM_LOCK_LEVEL_OFF; 
	TIM1_BDTRInitStructure.DeadTime = 0;//0x72;
	TIM1_BDTRInitStructure.Break = TIM_BREAK_IN_ENABLE;
	TIM1_BDTRInitStructure.BreakPolarity = TIM_BREAK_POLARITY_HIGH;
	TIM1_BDTRInitStructure.AutomaticOutput = TIM_AUTO_OUTPUT_DISABLE;
  TIM1_BDTRInitStructure.IomBreakEn = false;
	TIM_ConfigBkdt(TIM1, &TIM1_BDTRInitStructure);

	//TIM1 counter enable
	TIM_Enable(TIM1, ENABLE);
	TIM_EnableCtrlPwmOutputs(TIM1,ENABLE);
}

void PwmShut(void)
{
    uint16_t tmp;							
    tmp = TIM1->CCEN;
		tmp &= (uint16_t)(~((uint16_t)TIM_CCEN_CC1EN));
		tmp &= (uint16_t)(~((uint16_t)TIM_CCEN_CC2EN));
		tmp &= (uint16_t)(~((uint16_t)TIM_CCEN_CC3EN));
	  tmp &= (uint16_t)(~((uint16_t)TIM_CCEN_CC4EN));
    TIM1->CCEN = tmp;
	  GPIO_ResetBits(GPIOA, GPIO_PIN_12);
	  SystemError.PwmOn = 0;
}

void PwmOpen(void)
{
    uint16_t tmp;
    tmp = TIM1->CCEN;
		tmp |= (uint16_t)(((uint16_t)TIM_CCEN_CC1EN));
		tmp |= (uint16_t)(((uint16_t)TIM_CCEN_CC2EN)); 
		tmp |= (uint16_t)(((uint16_t)TIM_CCEN_CC3EN)); 
	  tmp |= (uint16_t)(((uint16_t)TIM_CCEN_CC4EN));
    TIM1->CCEN = tmp;
	  GPIO_SetBits(GPIOA, GPIO_PIN_12);
	  SystemError.PwmOn = 1;
}

