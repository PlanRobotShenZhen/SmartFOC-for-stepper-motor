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
 * @file adc.c
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
//#include "SystemDefine.h"
#include "ExternGlobals.h"
void Adc_Init(void);				// ADC≥ı ºªØ

void Adc_Init(void)
{
	ADC_InitType ADC_InitStructure;
	NVIC_InitType NVIC_InitStructure;
	ADC_DeInit(ADC1);
	ADC_DeInit(ADC2);
	//ADC1 and ADC2 configuration
	ADC_InitStruct(&ADC_InitStructure);
	ADC_InitStructure.WorkMode = ADC_WORKMODE_INJ_SIMULT;
	ADC_InitStructure.MultiChEn = ENABLE;
	ADC_InitStructure.ContinueConvEn = DISABLE;
	ADC_InitStructure.ExtTrigSelect = ADC_EXT_TRIG_INJ_CONV_T1_TRGO;
	ADC_InitStructure.DatAlign = ADC_DAT_ALIGN_R;
	ADC_InitStructure.ChsNumber = 0;
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Init(ADC2, &ADC_InitStructure);

  ADC_ConfigInjectedSequencerLength(ADC2,3);
  ADC_ConfigInjectedChannel(ADC2, ADC_CH_4, 1, ADC_SAMP_TIME_28CYCLES5);//Iv   // ADC_BAT 
	//ADC2_IN1 TO IV        ADC2_IN2 TO IU
  ADC_ConfigInjectedChannel(ADC2, ADC_CH_2, 2, ADC_SAMP_TIME_28CYCLES5);//Iu	 //IU
  ADC_ConfigInjectedChannel(ADC2, ADC_CH_1, 3, ADC_SAMP_TIME_28CYCLES5);//Iu	 //IV 
	
	ADC_EnableExternalTrigInjectedConv(ADC2,ENABLE);
    
	//NVIC Initial
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//Enable the ADC Interrupt 
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

  //Enable ADC
	ADC_Enable(ADC2, ENABLE);
    while(ADC_GetFlagStatusNew(ADC2,ADC_FLAG_RDY) == RESET);

	//Calibration
	ADC_StartCalibration(ADC2);
    while (ADC_GetCalibrationStatus(ADC2));

	ADC_ConfigInt(ADC2, ADC_INT_JENDC | ADC_INT_AWD, ENABLE);
}
