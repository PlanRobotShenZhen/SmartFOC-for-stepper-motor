
#include "DSP2803x_Device.h"     // Headerfile Include File
#include "ExternGlobals.h"
interrupt void SPI_isr(void)
{
    SpiRxData = SpiaRegs.SPIRXBUF;
    RotorMacAngle = (((SpiRxData >> 4) & 0xfff) - 200 + RotorMacAngleOffSet) & 0xfff;
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP6;
}
