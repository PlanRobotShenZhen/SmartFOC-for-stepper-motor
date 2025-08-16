/* 
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * Copyright (c) 2006 Christian Walter <wolti@sil.at>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: mb.c,v 1.28 2010/06/06 13:54:40 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"
//#include "usart.h"
#include "uart_interface.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbconfig.h"
#include "mbframe.h"
#include "mbproto.h"
#include "mbfunc.h"

#include "mbport.h"
#if MB_RTU_ENABLED == 1
#include "mbrtu.h"
#endif
#if MB_ASCII_ENABLED == 1
#include "mbascii.h"
#endif
#if MB_TCP_ENABLED == 1
#include "mbtcp.h"
#endif

#ifndef MB_PORT_HAS_CLOSE
#define MB_PORT_HAS_CLOSE 0
#endif
/* ----------------------- Static variables ---------------------------------*/

static UCHAR    ucMBAddress;
/* ˽�б��� ------------------------------------------------------------------*/
static enum
{
    STATE_ENABLED,
    STATE_DISABLED,
    STATE_NOT_INITIALIZED
} eMBState = STATE_NOT_INITIALIZED;
PDUData_TypeDef PduData;
REG_VALUE R_value;
//static  wjj
	uint16_t mbdata[MB_RTU_DATA_MAX_SIZE];
extern uint16_t error_code;

/* Functions pointer which are initialized in eMBInit( ). Depending on the
 * mode (RTU or ASCII) the are set to the correct implementations.
 */

//static peMBFrameSend peMBFrameSendCur;
static pvMBFrameStart pvMBFrameStartCur;
static pvMBFrameStop pvMBFrameStopCur;
static peMBFrameReceive peMBFrameReceiveCur;
static pvMBFrameClose pvMBFrameCloseCur;

/* Callback functions required by the porting layer. They are called when
 * an external event has happend which includes a timeout or the reception
 * or transmission of a character.
 */
BOOL( *pxMBFrameCBByteReceived ) ( void );
BOOL( *pxMBFrameCBTransmitterEmpty ) ( void );
BOOL( *pxMBPortCBTimerExpired ) ( void );

BOOL( *pxMBFrameCBReceiveFSMCur ) ( void );
BOOL( *pxMBFrameCBTransmitFSMCur ) ( void );

eMBErrorCode eMBEnable(void);
/* An array of Modbus functions handlers which associates Modbus function
 * codes with implementing functions.
 */
static xMBFunctionHandler xFuncHandlers[MB_FUNC_HANDLERS_MAX] = {
#if MB_FUNC_OTHER_REP_SLAVEID_ENABLED > 0
    {MB_FUNC_OTHER_REPORT_SLAVEID, eMBFuncReportSlaveID},
#endif
#if MB_FUNC_READ_INPUT_ENABLED > 0
    {MB_FUNC_READ_INPUT_REGISTER, eMBFuncReadInputRegister},
#endif
#if MB_FUNC_READ_HOLDING_ENABLED > 0
    {MB_FUNC_READ_HOLDING_REGISTER, eMBFuncReadHoldingRegister},
#endif
#if MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0
    {MB_FUNC_WRITE_MULTIPLE_REGISTERS, eMBFuncWriteMultipleHoldingRegister},
#endif
#if MB_FUNC_WRITE_HOLDING_ENABLED > 0
    {MB_FUNC_WRITE_REGISTER, eMBFuncWriteHoldingRegister},
#endif
#if MB_FUNC_READWRITE_HOLDING_ENABLED > 0
    {MB_FUNC_READWRITE_MULTIPLE_REGISTERS, eMBFuncReadWriteMultipleHoldingRegister},
#endif
#if MB_FUNC_READ_COILS_ENABLED > 0
    {MB_FUNC_READ_COILS, eMBFuncReadCoils},
#endif
#if MB_FUNC_WRITE_COIL_ENABLED > 0
    {MB_FUNC_WRITE_SINGLE_COIL, eMBFuncWriteCoil},
#endif
#if MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0
    {MB_FUNC_WRITE_MULTIPLE_COILS, eMBFuncWriteMultipleCoils},
#endif
#if MB_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0
    {MB_FUNC_READ_DISCRETE_INPUTS, eMBFuncReadDiscreteInputs},
#endif
};
/**
  * ��������: ����ڴ�
  * �������: buf:�ڴ�ռ��׵�ַ,Code:������
  * �� �� ֵ: ��
  * ˵    ��: ���ܲ�ͬ�Ĺ�����������ݲ�һ���ڴ�ռ�,
  */
void FillBuf(uint16_t* buf, uint8_t Code)
{
    uint16_t i = 0;
    switch (Code)
    {
    case FUN_CODE_03H:
    case FUN_CODE_06H:
    case FUN_CODE_10H:
        for (i = 0; i < MB_RTU_DATA_MAX_SIZE; i++)
            buf[i] = 0;
        break;
    }
}
/**
  * ��������: �жϲ������������Ƿ����Э�鷶Χ
  * �������: _RegNum:�Ĵ�������,_FunCode:������,_ByteNum:�ֽ�����
  * �� �� ֵ: �쳣��:03��NONE
  * ˵    ��: �Կɲ��������ڴ�ռ�Ĺ�������Ҫ��֤�����ĵ�ַ�Ƿ���Ϸ�Χ
  */
uint8_t MB_JudgeNum(uint16_t _RegNum, uint8_t _FunCode, uint16_t _ByteNum)
{
    uint8_t Excode = EX_CODE_NONE;
    uint16_t _CoilNum = _RegNum; // ��Ȧ(��ɢ��)������
    switch (_FunCode)
    {
    case FUN_CODE_01H:
    case FUN_CODE_02H:
        if ((_CoilNum < 0x0001) || (_CoilNum > 0x07D0))
            Excode = EX_CODE_03H;// �쳣��03H;
        break;
    case FUN_CODE_03H:
    case FUN_CODE_04H:
        if ((_RegNum < 0x0001) || (_RegNum > 0x007D))
            Excode = EX_CODE_03H;// �쳣��03H;      
        break;
    case FUN_CODE_10H:
        if ((_RegNum < 0x0001) || (_RegNum > 0x007B))
            Excode = EX_CODE_03H;// �쳣��03H
        if (_ByteNum != (_RegNum << 1))
            Excode = EX_CODE_03H;// �쳣��03H
        break;
    }
    return Excode;
}

/**
  * ��������: �жϵ�ַ�Ƿ����Э�鷶Χ
  * �������: _Addr:��ʼ��ַ,_RegNum:�Ĵ�������,_FunCode:������
  * �� �� ֵ: �쳣��:02H��NONE
  * ˵    ��: ��ַ��Χ��0x0000~0xFFFF,�ɲ����Ŀռ䷶Χ���ܳ����������
  */
uint8_t MB_JudgeAddr(uint16_t _Addr, uint16_t _RegNum)
{
    uint8_t Excode = EX_CODE_NONE;
    /* ��ַ+�Ĵ����������ܳ���0xFFFF */
    if (((uint32_t)_RegNum + (uint32_t)_Addr) > (uint32_t)0xFFFF)
    {
        Excode = EX_CODE_02H;// �쳣�� 02H
    }
    return Excode;
}

/**
  * ��������: �Խ��յ������ݽ��з�����ִ��
  * �������: ��
  * �� �� ֵ: �쳣���0x00
  * ˵    ��: �жϹ�����,��֤��ַ�Ƿ���ȷ.��ֵ�����Ƿ����,����û����ͷ�����Ӧ�ź�
  */
uint8_t MB_Analyze_Execute(void)
{
    uint16_t ExCode = EX_CODE_NONE;
    /* У�鹦���� */
    if (IS_NOT_FUNCODE(PduData.Code)) // ��֧�ֵĹ�����
    {
        /* Modbus�쳣��Ӧ */
        ExCode = EX_CODE_01H;            // �쳣��01H
        return ExCode;
    }
    /* ���ݹ�����ֱ����ж� */
    switch (PduData.Code)
    {
        /* ������Ϊ01H�������02��������һ����,��ʵҲûʲô��һ��
         * ֻ�ǲ�����ַ���ܲ�һ��,��һ���Ͼ�����ʵ��,������main����
         * ���뵥�����ڴ�ʹ�ò�ͬ�Ĺ�����,��ʵ��Ӧ���б����������ʹ��
         * ��ͬ���ڴ�ռ�
         */
         /* ---- 01H  02H ��ȡ��ɢ������(Coil Input)---------------------- */
    case FUN_CODE_01H:
    case FUN_CODE_02H:
        /* �ж���Ȧ�����Ƿ���ȷ */
        ExCode = MB_JudgeNum(PduData.Num, PduData.Code, 1);
        if (ExCode != EX_CODE_NONE)
            return ExCode;

        /* �жϵ�ַ�Ƿ���ȷ*/
        ExCode = MB_JudgeAddr(PduData.Addr, PduData.Num);
        if (ExCode != EX_CODE_NONE)
            return ExCode;
        break;
        /* ---- 03H  04H ��ȡ����/����Ĵ���---------------------- */
    case FUN_CODE_03H:
    case FUN_CODE_04H:
        /* �жϼĴ��������Ƿ���ȷ */
        ExCode = MB_JudgeNum(PduData.Num, PduData.Code, PduData.byteNums);
        if (ExCode != EX_CODE_NONE)
            return ExCode;

        /* �жϵ�ַ�Ƿ���ȷ*/
        ExCode = MB_JudgeAddr(PduData.Addr, PduData.Num);
        if (ExCode != EX_CODE_NONE)
            return ExCode;
        break;
        /* ---- 05H д�뵥����ɢ��---------------------- */
    case FUN_CODE_05H:
        break;
        /* ---- 06H д�������ּĴ��� ---------------------- */
    case FUN_CODE_06H:
        break;
        /* ---- 10H д������ּĴ��� ---------------------- */
    case FUN_CODE_10H:
        /* �жϼĴ��������Ƿ���ȷ */
        ExCode = MB_JudgeNum(PduData.Num, PduData.Code, PduData.byteNums);
        if (ExCode != EX_CODE_NONE)
            return ExCode;
        /* �жϵ�ַ�Ƿ���ȷ*/
        ExCode = MB_JudgeAddr(PduData.Addr, PduData.Num);
        if (ExCode != EX_CODE_NONE)
            return ExCode;
        break;
    }
    /* ����֡û���쳣 */
    return ExCode; //   EX_CODE_NONE
}


static uint16_t MB_RSP_AAH(uint16_t _TxCount, uint16_t* _AddrOffset, uint16_t _RegNum)
{
    /* ���ر��ּĴ����ڵ����� */
    for (uint8_t i = 0; i < _RegNum; i++)
    {
        modbus_send_data[_TxCount++] = ((*_AddrOffset) >> 8);
        modbus_send_data[_TxCount++] = *_AddrOffset++;
    }

    return _TxCount;

}
/**
  * ��������: ��ȡ��Ȧ״̬����/д��
  * �������: _TxCount :���ͼ�����,_AddrOffset��ַƫ����,_CoilNum:��Ȧ����
  * �� �� ֵ: Tx_Buf������Ԫ������
  * ˵    ��: ��ȡ��ɢ���,�������Tx_Buf
  */
static uint16_t MB_RSP_01H(uint16_t _TxCount, uint16_t _AddrOffset, uint16_t _CoilNum)
{
    /*
        ��������:
            01 �ӻ���ַ
            01 ������
            00 �Ĵ�����ʼ��ַ���ֽ�
            02 �Ĵ�����ʼ��ַ���ֽ�
            00 �Ĵ����������ֽ�
            08 �Ĵ����������ֽ�
            9C CRCУ����ֽ�
            0C CRCУ����ֽ�

        �ӻ�Ӧ��: 	1����ON��0����OFF��ʹ��LED��״̬�����棩�������ص���Ȧ����Ϊ8�ı�����������������ֽ�δβʹ��0����. BIT0��Ӧ��1��
            01 �ӻ���ַ
            01 ������
            01 �����ֽ���
            02 ����1(��Ȧ0002H-��Ȧ0011H)
            D0 CRCУ����ֽ�
            49 CRCУ����ֽ�

        ����1:
        ���ͣ�	01 01 00 02 00 08   9C 0C	  --- ��ѯD02��ʼ��8���̵���״̬
        ���أ�	01 01 01 01         90 48   --- ��ѯ��8��״̬Ϊ��0000 0001 �ڶ���LEDΪ��

        ����2:
        ���ͣ�	01 01 00 01 00 10   6C 06	  --- ��ѯD01��ʼ��16���̵���״̬
        ���أ�	01 01 02 FF FF      B8 4C   --- ��ѯ�������ֽ�����Ϊ0xFFFF

    */
    //  uint16_t i = 0;
    //	uint16_t m;	
    //	uint8_t status[10];	
    //	
    //  /* ���㷵���ֽ�����_CoilNum��������λΪ��λ�� */
    //  m = (_CoilNum+7)/8;
    //  /* �����ֽ�����������*/
    //	Tx_Buf[_TxCount++] = m; 
    //	if ((_AddrOffset >= COIL_D01) && (_CoilNum > 0))
    //  {
    //		/* ����ȡ����Ȧ״̬�������� */
    //		for (i = 0; i < m; i++)
    //		{
    //			status[i] = 0;
    //		}		
    //		/* ��ȡ��Ӧ��Ȧ״̬��������д��status[] */
    //		for (i = 0; i < _CoilNum; i++)
    //		{
    //			/* ��LED��״̬��д��״̬�Ĵ�����ÿһλ */
    //			if (Get_LEDx_State(i + 1 + _AddrOffset - COIL_D01))		
    //			{  
    //				status[i / 8] |= (1 << (i % 8));
    //			}			
    //		}    		
    //	}
    //	/* ��䷢������ */
    //	for (i = 0; i < m; i++)
    //	{
    //		Tx_Buf[_TxCount++] = status[i];	/* �̵���״̬ */
    //	}		

        /*----------------------------�ָ���----------------------------------*/
    modbus_send_data[_TxCount++] = 2;
    /* ��䷵������ */
    if (_AddrOffset == COIL_D01)
    {
        modbus_send_data[_TxCount++] = R_value.D01 >> 8;
        modbus_send_data[_TxCount++] = R_value.D01;
    }
    else if (_AddrOffset == COIL_D02)
    {
        modbus_send_data[_TxCount++] = R_value.D02 >> 8;
        modbus_send_data[_TxCount++] = R_value.D02;
    }
    else if (_AddrOffset == COIL_D03)
    {
        modbus_send_data[_TxCount++] = R_value.D03 >> 8;
        modbus_send_data[_TxCount++] = R_value.D03;
    }
    else if (_AddrOffset == COIL_D04)
    {
        modbus_send_data[_TxCount++] = R_value.D04 >> 8;
        modbus_send_data[_TxCount++] = R_value.D04;
    }
    else
    {
        modbus_send_data[_TxCount++] = 0;
        modbus_send_data[_TxCount++] = 0;
    }

    return _TxCount;
}

/**
  * ��������: ��ȡ��ɢ���루ֻ����
  * �������: _TxCount :���ͼ�����,_AddrOffset��ַƫ����,_CoilNum:��Ȧ����
  * �� �� ֵ: Tx_Buf������Ԫ������
  * ˵    ��: ��ȡ��ɢ���,�������Tx_Buf
  */
static uint16_t MB_RSP_02H(uint16_t _TxCount, uint16_t _AddrOffset, uint16_t _CoilNum)
{
    /*
        ��������:
            01 �ӻ���ַ
            02 ������
            00 �Ĵ�����ʼ��ַ���ֽ�
            01 �Ĵ�����ʼ��ַ���ֽ�
            00 �Ĵ����������ֽ�
            08 �Ĵ����������ֽ�
            28 CRCУ����ֽ�
            0C CRCУ����ֽ�

        �ӻ�Ӧ��: 	1����ON��0����OFF��ʹ��LED��״̬�����棩�������ص���Ȧ����Ϊ8�ı�����������������ֽ�δβʹ��0����. BIT0��Ӧ��1��
            01 �ӻ���ַ
            02 ������
            01 �����ֽ���
            02 ����1(��Ȧ0002H-��Ȧ0011H)
            D0 CRCУ����ֽ�
            49 CRCУ����ֽ�

        ����:
        ���ͣ�	01 01 00 02 00 08   9C 0C	  --- ��ѯD02��ʼ��8���̵���״̬
        ���أ�	01 01 01 02         D0 49   --- ��ѯ��8��״̬Ϊ��0000 0010 �ڶ���LEDΪ��
    */
    uint16_t i = 0;
    uint16_t m;
    uint8_t status[10];

    /* ���㷵���ֽ�����_CoilNum��������λΪ��λ�� */
    m = (_CoilNum + 7) / 8;
    /* �����ֽ�����������*/
    modbus_send_data[_TxCount++] = m;

    if ((_AddrOffset >= COIL_D01) && (_CoilNum > 0))
    {
        /* ����ȡ����Ȧ״̬�������� */
        for (i = 0; i < m; i++)
        {
            status[i] = 0;
        }
        /* ��ȡ��Ӧ��Ȧ״̬��������д��status[] */
        //for (i = 0; i < _CoilNum; i++)
        //{
        //    /* ��LED��״̬��д��״̬�Ĵ�����ÿһλ */
        //    if (Get_LEDx_State(i + 1 + _AddrOffset - COIL_D01))
        //    {
        //        status[i / 8] |= (1 << (i % 8));
        //    }
        //}
    }
    /* ��䷢������ */
    for (i = 0; i < m; i++)
    {
        /* �̵���״̬ */
        modbus_send_data[_TxCount++] = status[i];
    }
    return _TxCount;
}

/**
  * ��������: ��ȡ���ּĴ�������/д��
  * �������: _TxCount :���ͼ�����,_AddrOffset��ַƫ����,_RegNum:�Ĵ�������
  * �� �� ֵ: Tx_Buf������Ԫ������
  * ˵    ��: ��ȡ���ּĴ���������,�������Tx_Buf
  */
static uint8_t MB_RSP_03H(uint16_t _TxCount, uint16_t* _AddrOffset, uint16_t _RegNum)
{
    /*
        �ӻ���ַΪ01H�����ּĴ�������ʼ��ַΪ0010H��������ַΪ0011H���ôβ�ѯ�ܹ�����2�����ּĴ�����
        ��������:
            01 �ӻ���ַ
            03 ������
            00 �Ĵ�����ַ���ֽ�
            10 �Ĵ�����ַ���ֽ�
            00 �Ĵ����������ֽ�
            02 �Ĵ����������ֽ�
            C5 CRC���ֽ�
            CE CRC���ֽ�

        �ӻ�Ӧ��: 	���ּĴ����ĳ���Ϊ2���ֽڡ����ڵ������ּĴ������ԣ��Ĵ������ֽ������ȱ����䣬
                    ���ֽ����ݺ󱻴��䡣���ּĴ���֮�䣬�͵�ַ�Ĵ����ȱ����䣬�ߵ�ַ�Ĵ����󱻴��䡣
            01 �ӻ���ַ
            03 ������
            04 �ֽ���
            12 ����1���ֽ�(0010H)
            34 ����1���ֽ�(0010H)
            02 ����2���ֽ�(0011H)
            03 ����2���ֽ�(0100H)
            FF CRC���ֽ�
            F4 CRC���ֽ�

        ��һ�����ּĴ�������:
            ���ͣ�	01 03 00 10 00 01            85 CF ---- �� 0010Hһ���Ĵ�������
            ���أ�	01 03 02 12 34               B5 33 ---- ����10H������д������ݣ�10H���������ܣ�
    */

    /* ��䷵�ؼĴ������� */
    modbus_send_data[_TxCount++] = _RegNum * 2;
    /* ���ر��ּĴ����ڵ����� */
    for (uint8_t i = 0; i < _RegNum; i++)
    {
        modbus_send_data[_TxCount++] = ((*_AddrOffset) >> 8);
        modbus_send_data[_TxCount++] = *_AddrOffset++;
    }

    return _TxCount;
}

/**
  * ��������: ��ȡ����Ĵ�����������03Hָ�����ƣ���������Ҳ���ƣ���ֻ����
  * �������: _TxCount :���ͼ�����,_AddrOffset��ַƫ����,_RegNum:�Ĵ�������
  * �� �� ֵ: Tx_Buf������Ԫ������
  * ˵    ��: ��ȡ���ּĴ���������,�������Tx_Buf
  */
static uint8_t MB_RSP_04H(uint16_t _TxCount, uint16_t _AddrOffset, uint16_t _RegNum)
{
    /*
        ��������:
            01 �ӻ���ַ
            04 ������
            00 �Ĵ�����ʼ��ַ���ֽ�
            20 �Ĵ�����ʼ��ַ���ֽ�
            00 �Ĵ����������ֽ�
            02 �Ĵ����������ֽ�
            70 CRC���ֽ�
            01 CRC���ֽ�

        �ӻ�Ӧ��:  ����Ĵ�������Ϊ2���ֽڡ����ڵ�������Ĵ������ԣ��Ĵ������ֽ������ȱ����䣬
                ���ֽ����ݺ󱻴��䡣����Ĵ���֮�䣬�͵�ַ�Ĵ����ȱ����䣬�ߵ�ַ�Ĵ����󱻴��䡣
            01 �ӻ���ַ
            04 ������
            02 �ֽ���
            02 ����1���ֽ�(0020H)
            03 ����1���ֽ�(0020H)
            00 ����2���ֽ�(0021H)
            00 ����2���ֽ�(0021H)
            82 CRC���ֽ�
            3C CRC���ֽ�

        ����:
            ���ͣ�	01 04 00 20 00 02      70 01  --- �� 0020H IN1 ��ʼ��2���ֽ���������
            ���أ�	01 04 02 02 03 00 00   82 3C  --- ���أ�02 03 00 00 4�����ݣ�����KEY1������ı� R_value.IN1��ֵ��
    */
    uint8_t i;
    uint16_t reg_value[64];
    /* ��䷵�ؼĴ������� */
    modbus_send_data[_TxCount++] = _RegNum;
    /* ��ȡ���ּĴ������� */
    for (i = 0; i < _RegNum; i++)
    {
        switch (_AddrOffset)
        {
            /* ���Բ��� */
        case REG_IN1:
            reg_value[i] = R_value.IN1;
            break;

        default:
            reg_value[i] = 0;
            break;
        }
        _AddrOffset++;
    }

    /* ��䷵������ */
    for (i = 0; i < _RegNum; i++)
    {

        modbus_send_data[_TxCount++] = reg_value[i] >> 8;
        modbus_send_data[_TxCount++] = reg_value[i] & 0xFF;
    }
    return _TxCount;
}

/**
  * ��������: д������Ȧ����/д��
  * �������: _TxCount :���ͼ�����,_AddrOffset��ַƫ����,_RegDATA:д������
  * �� �� ֵ: Tx_Buf������Ԫ������
  * ˵    ��: ���Tx_Buf
  */
static uint8_t MB_RSP_05H(uint16_t _TxCount, uint16_t _AddrOffset, uint16_t _RegDATA)
{
    /*
        ��������: д������Ȧ�Ĵ������򵥵�01~03�Ĵ�����ַ��ӦLED1~LED3,�����ݴ����D01��D02��D03��D04��4����Ա��
        05Hָ�����õ�����Ȧ��״̬
            01 �ӻ���ַ
            05 ������
            00 �Ĵ�����ַ���ֽ�
            01 �Ĵ�����ַ���ֽ�
            FF ����1���ֽ�
            FF ����1���ֽ�
            9D CRCУ����ֽ�
            BA CRCУ����ֽ�

        �ӻ�Ӧ��:
            01 �ӻ���ַ
            05 ������
            00 �Ĵ�����ַ���ֽ�
            01 �Ĵ�����ַ���ֽ�
            FF �Ĵ���1���ֽ�
            FF �Ĵ���1���ֽ�
            9D CRCУ����ֽ�
            BA CRCУ����ֽ�

        ����:
        ���ͣ�	01 05 00 04 FF FF   8D BB   -- ��������0xFFFF��0x04��ַ��Ȧ��
        ���أ�	01 05 00 04 FF FF   8D BB   -- ����ԭʼ����
    */

    /* ����ֵַ */
    modbus_send_data[_TxCount++] = _AddrOffset >> 8;
    modbus_send_data[_TxCount++] = _AddrOffset;

    if (_AddrOffset == COIL_D01)
    {
        R_value.D01 = _RegDATA;
        //LED1_ON;
    }
    else if (_AddrOffset == COIL_D02)
    {
        R_value.D02 = _RegDATA;
        //LED2_ON;
    }
    else if (_AddrOffset == COIL_D03)
    {
        R_value.D03 = _RegDATA;
        //LED3_ON;
    }
    else if (_AddrOffset == COIL_D04)
    {
        R_value.D04 = _RegDATA;
        //LED1_TOGGLE;
        //LED2_TOGGLE;
        //LED3_TOGGLE;
    }
    else
    {
        R_value.D01 = 0;
        R_value.D02 = 0;
        R_value.D03 = 0;
        R_value.D04 = 0;
    }

    modbus_send_data[_TxCount++] = _RegDATA >> 8;
    modbus_send_data[_TxCount++] = _RegDATA;
    return _TxCount;
}

int isWritableAddr(uint16_t _AddrOffset)
{
    if (_AddrOffset >= 150&& _AddrOffset<=250)
    {
        return 0;
    }
    return 1;
}

/**
  * ��������: д�������ּĴ�������/д��
  * �������: _TxCount :���ͼ�����,_AddrOffset:��ַƫ����,_RegNum: д�����ݣ�_AddrAbs�����ּĴ�����ַ
  * �� �� ֵ: Tx_Buf������Ԫ������
  * ˵    ��: ���Tx_Buf
  */
static uint8_t MB_RSP_06H(uint16_t _TxCount, uint16_t _AddrOffset, uint16_t _RegNum, uint16_t* _AddrAbs)
{
    /*
        д���ּĴ�����ע��06ָ��ֻ�ܲ����������ּĴ�����10Hָ��������õ����������ּĴ���
        ��������:
            01 �ӻ���ַ
            06 ������
            00 �Ĵ�����ַ���ֽ�
            10 �Ĵ�����ַ���ֽ�
            67 ����1���ֽ�
            4A ����1���ֽ�
            23 CRCУ����ֽ�
            C8 CRCУ����ֽ�

        �ӻ���Ӧ:
            01 �ӻ���ַ
            06 ������
            00 �Ĵ�����ַ���ֽ�
            10 �Ĵ�����ַ���ֽ�
            67 ����1���ֽ�
            4A ����1���ֽ�
            23 CRCУ����ֽ�
            C8 CRCУ����ֽ�

        ����:
            ���ͣ�	01 06 00 10 67 4A  23 C8    ---- ��0010��ַ�Ĵ�������Ϊ67 4A
            ���أ�	01 06 00 10 67 4A  23 C8    ---- ����ͬ������
*/
/* ����ֵַ */
    modbus_send_data[_TxCount++] = _AddrOffset >> 8;
    modbus_send_data[_TxCount++] = _AddrOffset;

    /* ������д�뱣�ּĴ����� */
    if (isWritableAddr(_AddrOffset))
    {//<  ��д��ַ
        *_AddrAbs = _RegNum;
    }
    if (_AddrOffset == 6 && _RegNum == 0xa5)
    {
        //Soft_Reset();
    }
    else if (_AddrOffset == 3 && _RegNum == 0)
    {
        //error_code = 0;
    }
    /* ��䷵������ */
    modbus_send_data[_TxCount++] = PduData.Num >> 8;
    modbus_send_data[_TxCount++] = PduData.Num;

    return _TxCount;
}

/**
  * ��������: д������ּĴ�������/д��
  * �������: _TxCount :���ͼ�����,_AddrOffset��ַƫ����,_RegNum:�ֽ�������_Datebuf:����
  * �� �� ֵ: Tx_Buf������Ԫ������
  * ˵    ��: ���Tx_Buf
  */
static uint8_t MB_RSP_10H(uint16_t _TxCount, uint16_t  _AddrOffset, uint16_t _RegNum, uint16_t* _AddrAbs, uint8_t* _Datebuf)
{
    /*
        ��������:
            01 �ӻ���ַ
            10 ������
            00 �Ĵ�����ʼ��ַ���ֽ�
            10 �Ĵ�����ʼ��ַ���ֽ�
            00 �Ĵ����������ֽ�
            02 �Ĵ����������ֽ�
            04 �ֽ���
            12 ����1���ֽ�
            34 ����1���ֽ�
            02 ����2���ֽ�
            03 ����2���ֽ�
            F7 CRCУ����ֽ�
            74 CRCУ����ֽ�

        �ӻ���Ӧ:
            01 �ӻ���ַ
            10 ������
            00 �Ĵ�����ַ���ֽ�
            10 �Ĵ�����ַ���ֽ�
            00 �Ĵ����������ֽ�
            02 �Ĵ����������ֽ�
            40 CRCУ����ֽ�
            0D CRCУ����ֽ�

        ����:
            ���ͣ�	01 10 00 10 00 02 04 12 34 02 03 F7 74    ----  ��0010H~0011Hд��12 34 02 03 �ĸ��ֽ�����
            ���أ�	01 10 00 10 00 02 40 0D                   ----  ��������

    */
    uint16_t i = 0;
    uint16_t Value = 0;
    /* ����ֵַ */
    modbus_send_data[_TxCount++] = _AddrOffset >> 8;
    modbus_send_data[_TxCount++] = _AddrOffset;

    /* д�������ּĴ��� */
    for (i = 0; i < _RegNum; i++)
    {
        Value = (uint16_t)((*_Datebuf << 8) | (*(_Datebuf + 1)));
        *_AddrAbs++ = Value;
        _Datebuf += 2;
    }

    modbus_send_data[_TxCount++] = _RegNum >> 8;
    modbus_send_data[_TxCount++] = _RegNum;
    return _TxCount;
}

/**
  * ��������: �쳣��Ӧ
  * �������: _FunCode :�����쳣�Ĺ�����,_ExCode:�쳣��
  * �� �� ֵ: ��
  * ˵    ��: ��ͨ������֡�����쳣ʱ,�����쳣��Ӧ
  */
void MB_Exception_RSP(uint8_t _FunCode, uint8_t _ExCode)
{
    uint16_t crc;
    modbus_send_len = 0;
    modbus_send_data[modbus_send_len++] = ucMBAddress;		    /* ��վ��ַ */
    modbus_send_data[modbus_send_len++] = _FunCode | 0x80;		  /* ������ + 0x80*/
    modbus_send_data[modbus_send_len++] = _ExCode;	          /* �쳣��*/

    crc = usMBCRC16((UCHAR*)&modbus_send_data, modbus_send_len);
    modbus_send_data[modbus_send_len++] = crc;	          /* crc ���ֽ� */
    modbus_send_data[modbus_send_len++] = crc >> 8;		      /* crc ���ֽ� */
    //UART_Tx((uint8_t*)Tx_Buf, TxCount);
    modbus_send_flag = 2;
}
/**
  * ��������: ������Ӧ
  * �������: _FunCode :������
  * �� �� ֵ: ��
  * ˵    ��: ��ͨ������֡û���쳣ʱ���ҳɹ�ִ��֮��,������Ӧ����֡
  */
void MB_RSP(uint8_t _FunCode)
{
    modbus_send_len = 0;
    uint16_t crc = 0;	
    modbus_send_data[modbus_send_len++] = ucMBAddress;		 /* ��վ��ַ */
    modbus_send_data[modbus_send_len++] = _FunCode;        /* ������   */
    switch (_FunCode)
    {
    case FUN_CODE_AAH:
        PduData.Addr = 218;
        PduData.PtrHoldingOffset = PduData.PtrHoldingbase + PduData.Addr; // ���ּĴ�������ʼ��ַ
        modbus_send_len = MB_RSP_AAH(modbus_send_len, (uint16_t*)PduData.PtrHoldingOffset,7);
        break;
    case FUN_CODE_01H:
        /* ��ȡ��Ȧ״̬ */
        modbus_send_len = MB_RSP_01H(modbus_send_len, PduData.Addr, PduData.Num);
        break;
    case FUN_CODE_02H:
        /* ��ȡ��ɢ���� */
        modbus_send_len = MB_RSP_02H(modbus_send_len, PduData.Addr, PduData.Num);
        break;
    case FUN_CODE_03H:
        /* ��ȡ���ּĴ��� */
        modbus_send_len = MB_RSP_03H(modbus_send_len, (uint16_t*)PduData.PtrHoldingOffset, PduData.Num);
        break;
    case FUN_CODE_04H:
        /* ��ȡ����Ĵ��� */
        modbus_send_len = MB_RSP_04H(modbus_send_len, PduData.Addr, PduData.Num);
        break;
    case FUN_CODE_05H:
        /* д������Ȧ */
        modbus_send_len = MB_RSP_05H(modbus_send_len, PduData.Addr, PduData.Num);
        break;
    case FUN_CODE_06H:
        /* д�������ּĴ��� */
        modbus_send_len = MB_RSP_06H(modbus_send_len, PduData.Addr, PduData.Num, (uint16_t*)PduData.PtrHoldingOffset);
        break;
    case FUN_CODE_10H:
        /* д������ּĴ��� */
        modbus_send_len = MB_RSP_10H(modbus_send_len, PduData.Addr, PduData.Num, (uint16_t*)PduData.PtrHoldingOffset, (uint8_t*)PduData.ValueReg);
        break;
    }
    crc = usMBCRC16((UCHAR*)&modbus_send_data, modbus_send_len);
    modbus_send_data[modbus_send_len++] = crc;	          /* crc ���ֽ� */
    modbus_send_data[modbus_send_len++] = crc >> 8;		      /* crc ���ֽ� */
    modbus_send_flag = 2;
}

/* ----------------------- Start implementation -----------------------------*/
eMBErrorCode eMBInit( eMBMode eMode, UCHAR ucSlaveAddress, UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity )
{
    eMBErrorCode    eStatus = MB_ENOERR;

    /* �����ڴ�ռ���Ϊ���ּĴ�����ַ,��Ӧ������03H��06H��10H */
    //PduData.PtrHoldingbase = (uint16_t*)malloc(sizeof(uint16_t) * MB_RTU_DATA_MAX_SIZE);
    PduData.PtrHoldingbase = mbdata;

    /* check preconditions */
    if( ( ucSlaveAddress == MB_ADDRESS_BROADCAST ) ||
        ( ucSlaveAddress < MB_ADDRESS_MIN ) || ( ucSlaveAddress > MB_ADDRESS_MAX ) )
    {
        eStatus = MB_EINVAL;
    }
    else
    {
        ucMBAddress = ucSlaveAddress;

        switch ( eMode )
        {
#if MB_RTU_ENABLED > 0
        case MB_RTU:
            pvMBFrameStartCur = eMBRTUStart;
            pvMBFrameStopCur = eMBRTUStop;
            //peMBFrameSendCur = eMBRTUSend;
            peMBFrameReceiveCur = eMBRTUReceive;
            pvMBFrameCloseCur = MB_PORT_HAS_CLOSE ? vMBPortClose : NULL;
            pxMBFrameCBByteReceived = xMBRTUReceiveFSM;
            pxMBFrameCBTransmitterEmpty = xMBRTUTransmitFSM;
            pxMBPortCBTimerExpired = xMBRTUTimerT35Expired;

            break;
#endif
#if MB_ASCII_ENABLED > 0
        case MB_ASCII:
            pvMBFrameStartCur = eMBASCIIStart;
            pvMBFrameStopCur = eMBASCIIStop;
            peMBFrameSendCur = eMBASCIISend;
            peMBFrameReceiveCur = eMBASCIIReceive;
            pvMBFrameCloseCur = MB_PORT_HAS_CLOSE ? vMBPortClose : NULL;
            pxMBFrameCBByteReceived = xMBASCIIReceiveFSM;
            pxMBFrameCBTransmitterEmpty = xMBASCIITransmitFSM;
            pxMBPortCBTimerExpired = xMBASCIITimerT1SExpired;

            eStatus = eMBASCIIInit( ucMBAddress, ucPort, ulBaudRate, eParity );
            break;
#endif
        default:
            eStatus = MB_EINVAL;
        }

        if( eStatus == MB_ENOERR )
        {
            if( !xMBPortEventInit(  ) )
            {
                /* port dependent event module initalization failed. */
                eStatus = MB_EPORTERR;
            }
            else
            {
                eMBState = STATE_DISABLED;
            }
        }
    }
    return eStatus;
}

#if MB_TCP_ENABLED > 0
eMBErrorCode
eMBTCPInit( USHORT ucTCPPort )
{
    eMBErrorCode    eStatus = MB_ENOERR;

    if( ( eStatus = eMBTCPDoInit( ucTCPPort ) ) != MB_ENOERR )
    {
        eMBState = STATE_DISABLED;
    }
    else if( !xMBPortEventInit(  ) )
    {
        /* Port dependent event module initalization failed. */
        eStatus = MB_EPORTERR;
    }
    else
    {
        pvMBFrameStartCur = eMBTCPStart;
        pvMBFrameStopCur = eMBTCPStop;
        peMBFrameReceiveCur = eMBTCPReceive;
        peMBFrameSendCur = eMBTCPSend;
        pvMBFrameCloseCur = MB_PORT_HAS_CLOSE ? vMBTCPPortClose : NULL;
        ucMBAddress = MB_TCP_PSEUDO_ADDRESS;
        eMBState = STATE_DISABLED;
    }
    return eStatus;
}
#endif

eMBErrorCode eMBRegisterCB( UCHAR ucFunctionCode, pxMBFunctionHandler pxHandler )
{
    int             i;
    eMBErrorCode    eStatus;

    if( ( 0 < ucFunctionCode ) && ( ucFunctionCode <= 127 ) )
    {
        ENTER_CRITICAL_SECTION(  );
        if( pxHandler != NULL )
        {
            for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ )
            {
                if( ( xFuncHandlers[i].pxHandler == NULL ) ||
                    ( xFuncHandlers[i].pxHandler == pxHandler ) )
                {
                    xFuncHandlers[i].ucFunctionCode = ucFunctionCode;
                    xFuncHandlers[i].pxHandler = pxHandler;
                    break;
                }
            }
            eStatus = ( i != MB_FUNC_HANDLERS_MAX ) ? MB_ENOERR : MB_ENORES;
        }
        else
        {
            for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ )
            {
                if( xFuncHandlers[i].ucFunctionCode == ucFunctionCode )
                {
                    xFuncHandlers[i].ucFunctionCode = 0;
                    xFuncHandlers[i].pxHandler = NULL;
                    break;
                }
            }
            /* Remove can't fail. */
            eStatus = MB_ENOERR;
        }
        EXIT_CRITICAL_SECTION(  );
    }
    else
    {
        eStatus = MB_EINVAL;
    }
    return eStatus;
}


eMBErrorCode
eMBClose( void )
{
    eMBErrorCode    eStatus = MB_ENOERR;

    if( eMBState == STATE_DISABLED )
    {
        if( pvMBFrameCloseCur != NULL )
        {
            pvMBFrameCloseCur(  );
        }
    }
    else
    {
        eStatus = MB_EILLSTATE;
    }
    return eStatus;
}

eMBErrorCode eMBEnable( void )
{
    eMBErrorCode    eStatus = MB_ENOERR;

    if( eMBState == STATE_DISABLED )
    {
        /* Activate the protocol stack. */
        pvMBFrameStartCur(  );
        eMBState = STATE_ENABLED;
    }
    else
    {
        eStatus = MB_EILLSTATE;
    }
    return eStatus;
}

eMBErrorCode
eMBDisable( void )
{
    eMBErrorCode    eStatus;

    if( eMBState == STATE_ENABLED )
    {
        pvMBFrameStopCur(  );
        eMBState = STATE_DISABLED;
        eStatus = MB_ENOERR;
    }
    else if( eMBState == STATE_DISABLED )
    {
        eStatus = MB_ENOERR;
    }
    else
    {
        eStatus = MB_EILLSTATE;
    }
    return eStatus;
}

eMBErrorCode eMBPoll(MBModify* modify)
{
    static UCHAR   *ucMBFrame;
    static UCHAR    ucRcvAddress;
    static USHORT   usLength;

    eMBErrorCode    eStatus = MB_ENOERR;
    eMBEventType    eEvent;
    modify->is_modify = 0;
    /* Check if the protocol stack is ready. */
    if( eMBState != STATE_ENABLED )
    {
        return MB_EILLSTATE;
    }

    /* Check if there is a event available. If not return control to caller.
     * Otherwise we will handle the event. */
    if( xMBPortEventGet( &eEvent ) == TRUE )
    {
        switch ( eEvent )
        {
        case EV_READY:
            break;

        case EV_FRAME_RECEIVED:
            eStatus = peMBFrameReceiveCur( &ucRcvAddress, &ucMBFrame, &usLength );
            if( eStatus == MB_ENOERR )
            {
                /* Check if the frame is for us. If not ignore the frame. */
                if( ( ucRcvAddress == ucMBAddress ) || ( ucRcvAddress == MB_ADDRESS_BROADCAST ) )
                {//< ���ݽ���
                    /* ��������֡��ִ�� */
                    uint16_t Ex_code;
                    PduData.Code = ucMBFrame[1];                   // ������
                    PduData.Addr = ((ucMBFrame[2] << 8) | ucMBFrame[3]);// �Ĵ�����ʼ��ַ
                    PduData.Num = ((ucMBFrame[4] << 8) | ucMBFrame[5]);// ����(Coil,Input,Holding Reg,Input Reg)
                    PduData.byteNums = ucMBFrame[6];                                     // ����ֽ���
                    PduData.ValueReg = (uint8_t*)&ucMBFrame[7];                          // �Ĵ���ֵ��ʼ��ַ
                    PduData.PtrHoldingOffset = PduData.PtrHoldingbase + PduData.Addr; // ���ּĴ�������ʼ��ַ
                    Ex_code = MB_Analyze_Execute();
                    if (Ex_code != EX_CODE_NONE)
                    {/* �����쳣 */
                        MB_Exception_RSP(PduData.Code, Ex_code+5);
                    }
                    else
                    {
                        MB_RSP(PduData.Code);
												if((PduData.Code == FUN_CODE_06H)|| (PduData.Code == FUN_CODE_10H))
												{
													modify->is_modify = 1;
                            modify->modify_addr = PduData.Addr;
												}
												else
												{
													modify->is_modify = 0;
												}
                    }
                }
                else
                {
                    Modbus_DMA_ReEnable(MODBUS_DMA_RX_Channel,MODBUS_RX_MAXBUFF);
										//HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_recv_data, USART1_RX_MAXBUFF); //����1����DMA����
                }
            }
            else
            {
								Modbus_DMA_ReEnable(MODBUS_DMA_RX_Channel,MODBUS_RX_MAXBUFF);
                //HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_recv_data, USART1_RX_MAXBUFF); //����1����DMA����
            }
            break;
        case EV_EXECUTE:
            break;
        case EV_FRAME_SENT:
            break;
        }
    }
    return MB_ENOERR;
}

uint16_t* getPDUData(void)
{
    return mbdata;
}
