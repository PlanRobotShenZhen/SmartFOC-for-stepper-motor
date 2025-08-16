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
 * File: $Id: mbrtu.c,v 1.18 2007/09/12 10:15:56 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbrtu.h"
#include "mbframe.h"

#include "mbcrc.h"
#include "mbport.h"
#include "uart_interface.h"
/* ----------------------- Defines ------------------------------------------*/
#define MB_SER_PDU_SIZE_MIN     4       /*!< Minimum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_MAX     256     /*!< Maximum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_CRC     2       /*!< Size of CRC field in PDU. */
#define MB_SER_PDU_ADDR_OFF     0       /*!< Offset of slave address in Ser-PDU. */
#define MB_SER_PDU_PDU_OFF      1       /*!< Offset of Modbus-PDU in Ser-PDU. */

/* ----------------------- Type definitions ---------------------------------*/
typedef enum
{
    STATE_RX_INIT,              /*!< Receiver is in initial state. */
    STATE_RX_IDLE,              /*!< Receiver is in idle state. */
    STATE_RX_RCV,               /*!< Frame is beeing received. */
    STATE_RX_ERROR              /*!< If the frame is invalid. */
} eMBRcvState;

typedef enum
{
    STATE_TX_IDLE,              /*!< Transmitter is in idle state. */
    STATE_TX_XMIT               /*!< Transmitter is in transfer state. */
} eMBSndState;

/* ----------------------- Static variables ---------------------------------*/
static volatile eMBSndState eSndState;
static volatile eMBRcvState eRcvState;

UCHAR* ucRTUBuf;

static volatile UCHAR *pucSndBufferCur;
static volatile USHORT usSndBufferCount;

/* ----------------------- Start implementation -----------------------------*/

void
eMBRTUStart( void )
{
    ENTER_CRITICAL_SECTION(  );
    /* Initially the receiver is in the state STATE_RX_INIT. we start
     * the timer and if no character is received within t3.5 we change
     * to STATE_RX_IDLE. This makes sure that we delay startup of the
     * modbus protocol stack until the bus is free.
     */
    eRcvState = STATE_RX_INIT;

    EXIT_CRITICAL_SECTION(  );
}

void
eMBRTUStop( void )
{
    ENTER_CRITICAL_SECTION(  );
    vMBPortSerialEnable( FALSE, FALSE );
    EXIT_CRITICAL_SECTION(  );
}

/**************************************************
* �������ܣ�	eMBRTUReceive
* ��    ����  pucRcvAddress��ָ��UCHAR���͵�ָ�룬���ڴ�����յ��ĵ�ַ��Ϣ
							pucFrame��ָ��ָ���ָ�룬�洢���յ������ݣ�������֡�ݸ����÷�
							pusLength��ָ��USHORT���͵�ָ�룬����֡����
* ˵    ����  ���յ�������֡ʱ��У��֡�����Լ�CRC��������ַ�����ݡ����Ⱥ󴫲�
* �� �� ֵ��  ����eMBErrorCode��ö��
**************************************************/
eMBErrorCode
eMBRTUReceive( UCHAR * pucRcvAddress, UCHAR ** pucFrame, USHORT * pusLength )
{
    //BOOL            xFrameReceived = FALSE;
    eMBErrorCode    eStatus = MB_ENOERR;
    assert_param(modbus_recv_len < MB_SER_PDU_SIZE_MAX );

    /* Length and CRC check
			У�����ݳ����Լ�CRCֵ*/
    USHORT crc = (ucRTUBuf[modbus_recv_len - 1]<<8) | ucRTUBuf[modbus_recv_len -2];

    if (modbus_recv_len < MB_SER_PDU_SIZE_MIN)
    {
        eStatus = MB_LESS_ERR;
    }
    else if (modbus_recv_len >= MB_SER_PDU_SIZE_MAX)
    {
        eStatus = MB_EXCEED_ERR;
    }
    else if (usMBCRC16((UCHAR*)ucRTUBuf, modbus_recv_len - 2) != crc)
    {
        eStatus = MB_CRC_ERR;
    }
		//У��ɹ�
    if(eStatus == MB_ENOERR)
    {
			/*�����ucRTUBuf�ں���һ��������uart1_recv_data����*/
        /* Save the address field. All frames are passed to the upper layed
         * and the decision if a frame is used is done there.
			�����ַ�ֶΣ�����ĵ�һ���ֽڣ���Ϊ���ж�֡�Ƿ��Ƿ����Լ�*/
        *pucRcvAddress = ucRTUBuf[MB_SER_PDU_ADDR_OFF];

        /* Total length of Modbus-PDU is Modbus-Serial-Line-PDU minus
         * size of address field and CRC checksum.Modbus PDU�ܳ��ȼ��㣬��ȥ����ʼ�ֽ��Լ�У���ֽڣ����ڽ���Modbus�����������
         */
        *pusLength = ( USHORT )(modbus_recv_len - MB_SER_PDU_PDU_OFF - MB_SER_PDU_SIZE_CRC );

        /* Return the start of the Modbus PDU to the caller. pucFrame��ΪRTU֡��ָ�룬���ڳ������֡����*/
        *pucFrame = ucRTUBuf;
       // xFrameReceived = TRUE;
    }
    return eStatus;
}


/*******************************************************************************
* ��������		     : RTU����״̬������ֻҪ����ú������ͻ�������ʱ���жϡ�
* ��    ��         : ��
* ˵    ��         :UART1_TXͨ��DMA1ͨ��4��uart1_send_data�����ݴ���������ΪmodbusЭ���ʽ��֡��
*******************************************************************************/ 
BOOL
xMBRTUReceiveFSM( void )
{
    BOOL            xTaskNeedSwitch = FALSE;
    switch ( eRcvState )
    {
    case STATE_RX_INIT:
    case STATE_RX_IDLE:
    case STATE_RX_RCV://���ڽ�������֡
        ucRTUBuf = modbus_recv_data;
        xMBPortEventPost(EV_FRAME_RECEIVED);
        break;
        /* In the error state we wait until all characters in the
         * damaged frame are transmitted.
         */
    case STATE_RX_ERROR:
        eRcvState = STATE_RX_RCV;
        //vMBPortTimersEnable(  );
        break;
    default:break;
    }
    return xTaskNeedSwitch;
}

BOOL
xMBRTUTransmitFSM( void )
{
    BOOL            xNeedPoll = FALSE;

    assert_param( eRcvState == STATE_RX_IDLE );

    switch ( eSndState )
    {
        /* We should not get a transmitter event if the transmitter is in
         * idle state.  */
    case STATE_TX_IDLE:
        /* enable receiver/disable transmitter. */
        vMBPortSerialEnable( TRUE, FALSE );
        break;

    case STATE_TX_XMIT:
        /* check if we are finished. */
        if( usSndBufferCount != 0 )
        {
            xMBPortSerialPutByte( ( CHAR )*pucSndBufferCur );
            pucSndBufferCur++;  /* next byte in sendbuffer. */
            usSndBufferCount--;
        }
        else
        {
            xNeedPoll = xMBPortEventPost( EV_FRAME_SENT );
            /* Disable transmitter. This prevents another transmit buffer
             * empty interrupt. */
            vMBPortSerialEnable( TRUE, FALSE );
            eSndState = STATE_TX_IDLE;
        }
        break;
    }

    return xNeedPoll;
}

BOOL
xMBRTUTimerT35Expired( void )
{
    BOOL            xNeedPoll = FALSE;

    switch ( eRcvState )
    {
        /* Timer t35 expired. Startup phase is finished. */
    case STATE_RX_INIT:
        xNeedPoll = xMBPortEventPost( EV_READY );
        break;

        /* A frame was received and t35 expired. Notify the listener that
         * a new frame was received. */
    case STATE_RX_RCV:
        xNeedPoll = xMBPortEventPost( EV_FRAME_RECEIVED );
        break;

        /* An error occured while receiving the frame. */
    case STATE_RX_ERROR:
        break;

        /* Function called in an illegal state. */
    default:
        assert_param( ( eRcvState == STATE_RX_INIT ) ||
                ( eRcvState == STATE_RX_RCV ) || ( eRcvState == STATE_RX_ERROR ) );
    }

    eRcvState = STATE_RX_IDLE;

    return xNeedPoll;
}
