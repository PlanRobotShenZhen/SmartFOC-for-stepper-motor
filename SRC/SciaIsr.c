#include "DSP2803x_Device.h"     // DSP280x Headerfile Include File
#include "DSP2803x_Examples.h"   // DSP280x Examples Include File
#include "DSP2803x_EPwm.h"          // Include header for the PWMGEN object
#include "DeviceConfig.h"
#include "ExternGlobals.h"
extern void scia_xmitH(int a);
extern void scia_msgH(char* msgH);
void CreatCRC(void);
void ReverseCRC(void);
int SciaRcBuf = 0;
interrupt void SciaRxIsr(void)
{
    int RxTmp;
    RxTmp = SciaRegs.SCIRXBUF.all; // Read data
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP9;
    // if((RxTmp==0xff)&&(RxCount<5))
    if(RxTmp == 0xff)
    {
        RxCount = 0;
        SynchFlag = 1;
    }
    if(SynchFlag)
    {
        RxCommand[RxCount] = RxTmp;
        RxCount++;
        if(RxCount > 4) // 0,1,2,3,4,5
        {
            //ReverseCRC();
            RxFinish = 1;
            SynchFlag = 0;
            RxCount = 0;
        }
    }
    SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1; // Clear Overflow flag
    SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1; // Clear Interrupt flag
}


interrupt void SciaTxIsr(void)
{

    SciaRegs.SCITXBUF = TxCommand[TxCount];
    TxCount++;
    //if(TxCount>8)
    if(TxCount > 7)
    {
        TxFlag = 0;
        TxCount = 0;
        SciaRegs.SCICTL1.bit.TXENA = 0;
    }
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP9;
    SciaRegs.SCIFFTX.bit.TXFFINTCLR = 1; // Clear Interrupt flag
}



void scia_xmitH(int a)
{
    while(SciaRegs.SCIFFTX.bit.TXFFST != 0) {}
    SciaRegs.SCITXBUF = a;

}

void scia_msgH(char* msgH)
{
    int i;
    i = 0;
    while(msgH[i] != '\0')
    {
        scia_xmitH(msgH[i]);
        i++;
    }
}






void CreatCRC(void)
{
    /* int i=0;
    int GenPoly=0x07;
    int CRC=0;
    for(i=1;i<8;i++)
    {
    	GenPoly = TxCommand[i]^GenPoly;
        GenPoly &= 0xff;
    }
    CRC=GenPoly;
     TxCommand[8]=CRC;
    */
}



void ReverseCRC(void)
{

    int i = 0;
    int GenPoly = 0x07;
    int CRC = 0;
    for(i = 1; i < 6; i++)
    {
        GenPoly = RxCommand[i] ^ GenPoly;
        GenPoly &= 0xff;
    }
    CRC = GenPoly;

    if(CRC == 0)
    {

        RxFinish = 1;
    }
}


