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
 * File: $Id: mbframe.h,v 1.9 2006/12/07 22:10:34 wolti Exp $
 */

#ifndef _MB_FRAME_H
#define _MB_FRAME_H

#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif

#include "n32g45x.h"
/*!
 * Constants which defines the format of a modbus frame. The example is
 * shown for a Modbus RTU/ASCII frame. Note that the Modbus PDU is not
 * dependent on the underlying transport.
 *
 * <code>
 * <------------------------ MODBUS SERIAL LINE PDU (1) ------------------->
 *              <----------- MODBUS PDU (1') ---------------->
 *  +-----------+---------------+----------------------------+-------------+
 *  | Address   | Function Code | Data                       | CRC/LRC     |
 *  +-----------+---------------+----------------------------+-------------+
 *  |           |               |                                   |
 * (2)        (3/2')           (3')                                (4)
 *
 * (1)  ... MB_SER_PDU_SIZE_MAX = 256
 * (2)  ... MB_SER_PDU_ADDR_OFF = 0
 * (3)  ... MB_SER_PDU_PDU_OFF  = 1
 * (4)  ... MB_SER_PDU_SIZE_CRC = 2
 *
 * (1') ... MB_PDU_SIZE_MAX     = 253
 * (2') ... MB_PDU_FUNC_OFF     = 0
 * (3') ... MB_PDU_DATA_OFF     = 1
 * </code>
 */

/* ----------------------- Defines ------------------------------------------*/
#define MB_PDU_SIZE_MAX     253 /*!< Maximum size of a PDU. */
#define MB_PDU_SIZE_MIN     1   /*!< Function Code */
#define MB_PDU_FUNC_OFF     0   /*!< Offset of function code in PDU. */
#define MB_PDU_DATA_OFF     1   /*!< Offset for response data in PDU. */
/* ���Ͷ��� ------------------------------------------------------------------*/
typedef struct {
    __IO uint8_t  Code;  	        // ������
    __IO uint8_t byteNums; 	        // �ֽ���
    __IO uint16_t Addr;            // �����ڴ����ʼ��ַ
    __IO uint16_t Num; 	            // �Ĵ���������Ȧ������
    __IO uint16_t _CRC;       	      // CRCУ����
    __IO uint8_t* ValueReg; 	      // 10H�����������
    __IO uint16_t* PtrHoldingbase;  // HoldingReg�ڴ��׵�ַ
    __IO uint16_t* PtrHoldingOffset;// HoldingReg�ڴ��׵�ַ
}PDUData_TypeDef;
typedef struct
{
    uint16_t IN1;

    /* 01H 05H ��д����ǿ����Ȧ */
    uint16_t D01;
    uint16_t D02;
    uint16_t D03;
    uint16_t D04;

}REG_VALUE;
/* �궨�� --------------------------------------------------------------------*/
#define MB_SLAVEADDR            0x0001
#define MB_ALLSLAVEADDR         0x00FF

#define FUN_CODE_01H            0x01  // ������01H 
#define FUN_CODE_02H            0x02  // ������02H
#define FUN_CODE_03H            0x03  // ������03H
#define FUN_CODE_04H            0x04  // ������04H
#define FUN_CODE_05H            0x05  // ������05H
#define FUN_CODE_06H            0x06  // ������06H
#define FUN_CODE_10H            0x10  // ������10H
#define FUN_CODE_AAH            0xAA  // ������AAH
/* ��������֧�ֵĹ�����,��Ҫ����¹����뻹��Ҫ��.c�ļ�������� */
#define IS_NOT_FUNCODE(code)  (!((code == FUN_CODE_01H)||\
                                 (code == FUN_CODE_02H)||\
                                 (code == FUN_CODE_03H)||\
                                 (code == FUN_CODE_04H)||\
                                 (code == FUN_CODE_05H)||\
                                 (code == FUN_CODE_06H)||\
                                 (code == FUN_CODE_AAH)||\
                                 (code == FUN_CODE_10H)))

#define EX_CODE_NONE           0x00  // �쳣�� ���쳣
#define EX_CODE_01H            0x01  // �쳣��
#define EX_CODE_02H            0x02  // �쳣��
#define EX_CODE_03H            0x03  // �쳣��
#define EX_CODE_04H            0x04  // �쳣��

#define COIL_D01		0x01
#define COIL_D02		0x02
#define COIL_D03		0x03
#define COIL_D04		0x04

#define REG_IN1		  0x0020
/* 03H �����ּĴ��� */
/* 06H д���ּĴ��� */
/* 10H д�������Ĵ��� */
#define HOLD_REG_01		0x0010
#define HOLD_REG_02		0x0011
#define HOLD_REG_03		0x0012

/* ��չ���� ------------------------------------------------------------------*/
extern PDUData_TypeDef PduData;




/* ----------------------- Prototypes  0-------------------------------------*/
typedef void    ( *pvMBFrameStart ) ( void );

typedef void    ( *pvMBFrameStop ) ( void );

typedef eMBErrorCode( *peMBFrameReceive ) ( UCHAR * pucRcvAddress,
                                            UCHAR ** pucFrame,
                                            USHORT * pusLength );

typedef eMBErrorCode( *peMBFrameSend ) ( UCHAR slaveAddress,
                                         const UCHAR * pucFrame,
                                         USHORT usLength );

typedef void( *pvMBFrameClose ) ( void );

#ifdef __cplusplus
PR_END_EXTERN_C
#endif
#endif
