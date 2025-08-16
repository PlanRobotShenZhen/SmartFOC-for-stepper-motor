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

void EnternExMode(void)							//������Դ��չģʽ�����ڵ͹���
{
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR, ENABLE);
    PWR->CTRL3 |= 0x00000001;       //������Դ��չģʽ�����ڵ͹��ĵ�
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR, DISABLE);

}
// ========================================================================
// �������ƣ�System_Init
// �����������
// �����������
// ��    �룺��
// ��    ������
// ����������ϵͳ��ʼ��
// ========================================================================


extern void MT6835_Init(void);
extern void SPI_Timer_Init(void);

void System_Init()				//ϵͳ��ʼ��
{

    EnternExMode(); 			//������Դ��չģʽ�����ڵ͹���
    SystemClk_Init();     //ϵͳʱ�Ӻ�����ʱ�ӳ�ʼ��
	
					/* This instruction raises the execution priority to 0. This prevents all
           exceptions with configurable priority from activating, other than through
           the HardFault fault escalation mechanism. */
	
    __disable_irq();			//����ȫ�ֽ�ֹ�ж�
    system_tick_init();   //�δ�ʱ����ʼ��
    Gpio_Init();          //����IO��ʼ��

    //DRV8323_GPIO_Init(); //DRV IO��ʼ��
    Pwm_Init();          	//TIM1 PWM��ʼ��
    Adc_Init();          	//ADC��ʼ��

    MT6835_Init();       	//������SPI��ʼ��
    SPI_Timer_Init();			//SPI_Timer��ʼ��
	  
    Modbus_DMA_Init();   //����modbus DMA��ʼ��
    Modbus_USART_Init(115200); //modbus IO�ʹ��ڳ�ʼ��
		
	        /* This instruction will allow all exceptions with configurable priority to
           be activated. */
					 
    __enable_irq();				//���ڿ��� ȫ���ж�
}


// ========================================================================
// �������ƣ�SysTick_Handler
// �����������
// �����������
// ��    �룺��
// ��    ������
// �����������δ�ʱ���жϣ������Ƶ����
// ========================================================================

#include "Timer1.h"

#define M_ENABLE 1

extern TIMER timer;
void SysTick_Handler(void)				//ϵͳ�δ��ʱ�������� ����Ĺ���
{
    Timeing(&timer);							//��ʱ��
    SpeedCalculate();							//�ٶȼ���					
    MotorControl(); // 1K   HZ    ������ƣ��������� ����ʶ�ȵȣ�

}

// ========================================================================
// �������ƣ�ADC1_2_IRQHandler
// �����������
// �����������
// ��    �룺��
// ��    ������
// ����������ADC1-2�ɼ���������ж�
// ========================================================================
short CurrentValue[3] = {0};
void ADC1_2_IRQHandler(void)
{
    if(ADC_GetIntStatus(ADC2, ADC_INT_JENDC) == SET)
    {
        ADC_ClearFlag(ADC2, ADC_FLAG_JENDC);
        if(TIM1->CTRL1 & 0x0010)   //���������Ӧ�¹ܸߵ�ƽ
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
// �������ƣ�TIM1_BRK_UP_TRG_COM_IRQHandler
// �����������
// �����������
// ��    �룺��
// ��    ������
// ��������������ɲ���жϺ���
// ========================================================================
void TIM1_BRK_UP_TRG_COM_IRQHandler()
{
    if(TIM_GetIntStatus(TIM1, TIM_INT_BREAK) != RESET)
    {
        TIM_ClrIntPendingBit(TIM1, TIM_INT_BREAK);
    }

}
