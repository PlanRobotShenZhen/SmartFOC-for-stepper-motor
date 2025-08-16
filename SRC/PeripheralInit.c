
#include "DSP2803x_Device.h"     // DSP280x Headerfile Include File
#include "f280xpwm.h"
void InitECapture(void);
void InitCpuTimers(void);   // For this example, only initialize the Cpu Timers
void SpiInit(void);
void F280X_PWM_InitA(void);
void F280X_PWM_InitB(void);
void scia_fifo_init(void);
void UserIoInit(void);
void ADInit(void);

void ADInit(void)
{
    EALLOW;
    //E_CURRENT
    AdcRegs.ADCSOC0CTL.bit.CHSEL = 7;
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 5;
    AdcRegs.ADCSOC0CTL.bit.ACQPS = 25;
    //B_VOLTAGE
    AdcRegs.ADCSOC1CTL.bit.CHSEL = 6;
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL = 5;
    AdcRegs.ADCSOC1CTL.bit.ACQPS = 25;
    //W_S
    AdcRegs.ADCSOC2CTL.bit.CHSEL = 3;
    AdcRegs.ADCSOC2CTL.bit.TRIGSEL = 5;
    AdcRegs.ADCSOC2CTL.bit.ACQPS = 25;
    //V_S
    AdcRegs.ADCSOC3CTL.bit.CHSEL = 4;
    AdcRegs.ADCSOC3CTL.bit.TRIGSEL = 5;
    AdcRegs.ADCSOC3CTL.bit.ACQPS = 25;
    //FootStoolSpeed
    AdcRegs.ADCSOC4CTL.bit.CHSEL = 2;
    AdcRegs.ADCSOC4CTL.bit.TRIGSEL = 5;
    AdcRegs.ADCSOC4CTL.bit.ACQPS = 25;
    //Oil
    AdcRegs.ADCSOC5CTL.bit.CHSEL = 1;
    AdcRegs.ADCSOC5CTL.bit.TRIGSEL = 5;
    AdcRegs.ADCSOC5CTL.bit.ACQPS = 25;

    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcRegs.ADCSAMPLEMODE.bit.SIMULEN0 = 0;
    AdcRegs.ADCSAMPLEMODE.bit.SIMULEN2 = 0;
    AdcRegs.ADCSAMPLEMODE.bit.SIMULEN4 = 0;
    AdcRegs.INTSEL3N4.bit.INT3SEL = 1;
    AdcRegs.INTSEL3N4.bit.INT3E = 1;
    AdcRegs.INTSEL3N4.bit.INT3CONT = 1;
    EDIS;
}

void I2CA_Init(void)
{
    // Initialize I2C
    I2caRegs.I2CSAR = 0x0050;    // Slave address - EEPROM control code
    I2caRegs.I2CPSC.all = 64;      // Prescaler - need 7-12 Mhz on module clk
    I2caRegs.I2CCLKL = 5;      // NOTE: must be non zero
    I2caRegs.I2CCLKH = 5;      // NOTE: must be non zero
    I2caRegs.I2CIER.all = 0x26;    // Enable SCD & ARDY & NACKinterrupts
    I2caRegs.I2CMDR.all = 0x20;  // Take I2C out of reset
}
void SpiInit()
{
    SpiaRegs.SPICCR.all = 0x004F; //下降沿输出数据，16位的字长
    SpiaRegs.SPICTL.all = 0x0006; //无延时，设置为主机，使能发送
    SpiaRegs.SPIBRR = 0xc; //波特率为 60m/ 60=1m
    SpiaRegs.SPICCR.bit.SPISWRESET = 1; //spi准备发送和接收下一个字
    SpiaRegs.SPIPRI.bit.FREE = 1;                // Set so breakpoints don't disturb xmission
    SpiaRegs.SPICTL.bit.SPIINTENA = 1;
}
//===========================================================================
// No more.
//===========================================================================

void scia_fifo_init()
{
    SciaRegs.SCICCR.all = 0x0007;  // 1 stop bit,  No loopback                                  // async mode, idle-line protocol
    SciaRegs.SCICTL1.all = 0x0003; // enable TX, RX, internal SCICLK,                                   // Disable RX ERR, SLEEP, TXWAKE
    SciaRegs.SCICTL2.bit.TXINTENA = 1;
    SciaRegs.SCICTL2.bit.RXBKINTENA = 1;
    SciaRegs.SCIHBAUD    = 0x0000; // 9600 baud @LSPCLK = 20MHz.
    SciaRegs.SCILBAUD    = 0x00c2;

    SciaRegs.SCIFFTX.all = 0xC020;
    SciaRegs.SCIFFRX.all = 0x0028;
    SciaRegs.SCIFFRX.bit.RXFFIL = 1;
    SciaRegs.SCIFFCT.all = 0x00;
    SciaRegs.SCICTL1.all = 0x0023;    // Relinquish SCI from Reset
    SciaRegs.SCIFFTX.bit.TXFIFOXRESET = 1;
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
}



void F280X_PWM_InitB(void)
{
    EALLOW;
    EPwm6Regs.ETSEL.bit.INTEN = 1;   // Enable EPWM1INT generation
    EPwm6Regs.ETSEL.bit.INTSEL = 1;  // Enable interrupt CNT_zero event
    EPwm6Regs.ETPS.bit.INTPRD = 1;   // Generate interrupt on the 1st event

    EPwm6Regs.TBCTL.bit.CTRMODE = 0; // Count up
    EPwm6Regs.TBPRD = 3000;       // Set timer period
    EPwm6Regs.TBCTL.bit.PHSEN = 0;             // Disable phase loading
    EPwm6Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
    EPwm6Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = 2;
    EPwm6Regs.TBCTL.bit.CLKDIV = 4;
    // Setup shadow register load on PERIOD
    EPwm6Regs.CMPCTL.bit.SHDWAMODE = 0;
    EPwm6Regs.CMPCTL.bit.SHDWBMODE = 0;
    EPwm6Regs.CMPCTL.bit.LOADAMODE = 1;
    EPwm6Regs.CMPCTL.bit.LOADBMODE = 1;

    // Set Compare values
    EPwm6Regs.CMPA.half.CMPA = 3000;    // Set compare A value
    EPwm6Regs.CMPB = 3000;                // Set Compare B value
    // Set actions
    EPwm6Regs.AQCTLA.bit.ZRO = 2;       // Set PWM2A on Zero- low
    EPwm6Regs.AQCTLA.bit.CAU = 1;       // Set PWM2A on Compare Values- high
    EPwm6Regs.AQCTLB.bit.ZRO = 2;       // Set PWM2B on Zero- low
    EPwm6Regs.AQCTLB.bit.CBU = 1;       // Set PWM2B on Compare Values- high
    /*set deadband*/
    EPwm6Regs.DBCTL.bit.OUT_MODE = 0;   //disable deadband

    EPwm6Regs.ETSEL.bit.INTSEL = 2;     // period interrupt
    EPwm6Regs.ETSEL.bit.INTEN = 1;      // enable interrupt
    EPwm6Regs.ETPS.bit.INTPRD = 1;      // Generate INT on first event




    EPwm5Regs.TBCTL.bit.CTRMODE = 0; // Count up
    EPwm5Regs.TBPRD = 1000;       // Set timer period
    EPwm5Regs.TBCTL.bit.PHSEN = 0;             // Disable phase loading
    EPwm5Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
    EPwm5Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = 2;
    EPwm5Regs.TBCTL.bit.CLKDIV = 4;

    // Setup shadow register load on PERIOD
    EPwm5Regs.CMPCTL.bit.SHDWAMODE = 0;
    EPwm5Regs.CMPCTL.bit.SHDWBMODE = 0;
    EPwm5Regs.CMPCTL.bit.LOADAMODE = 1;
    EPwm5Regs.CMPCTL.bit.LOADBMODE = 1;

    // Set Compare values
    EPwm5Regs.CMPA.half.CMPA = 1000;    // Set compare A value
    EPwm5Regs.CMPB = 0;                // Set Compare B value


    // Set actions
    EPwm5Regs.AQCTLA.bit.ZRO = 2;       // Set PWM2A on Zero- low
    EPwm5Regs.AQCTLA.bit.CAU = 1;       // Set PWM2A on Compare Values- high
    EPwm5Regs.AQCTLB.bit.ZRO = 2;       // Set PWM2B on Zero- low
    EPwm5Regs.AQCTLB.bit.CBU = 1;       // Set PWM2B on Compare Values- high













    EDIS;
}

void F280X_PWM_InitA(void)
{
    EALLOW;
    EPwm1Regs.TBCTL.bit.SYNCOSEL = 0;          // Pass through
    EPwm2Regs.TBCTL.bit.SYNCOSEL = 0;          // Pass through
    EPwm3Regs.TBCTL.bit.SYNCOSEL = 0;          // Pass through

    EPwm1Regs.TBCTL.bit.PHSEN = 1;
    EPwm2Regs.TBCTL.bit.PHSEN = 1;
    EPwm3Regs.TBCTL.bit.PHSEN = 1;

    EPwm1Regs.TBPRD = 1875;
    EPwm2Regs.TBPRD = 1875;
    EPwm3Regs.TBPRD = 1875;
    EPwm1Regs.TBPHS.half.TBPHS = 0;
    EPwm2Regs.TBPHS.half.TBPHS = 0;
    EPwm3Regs.TBPHS.half.TBPHS = 0;


    EPwm1Regs.TBCTL.all = PWM_INIT_STATE;
    EPwm2Regs.TBCTL.all = PWM_INIT_STATE;
    EPwm3Regs.TBCTL.all = PWM_INIT_STATE;

    // Init Compare Control Register for EPWM1-EPWM3
    EPwm1Regs.CMPCTL.all = CMPCTL_INIT_STATE;
    EPwm2Regs.CMPCTL.all = CMPCTL_INIT_STATE;
    EPwm3Regs.CMPCTL.all = CMPCTL_INIT_STATE;


    EPwm1Regs.AQCTLA.all = AQCTLA_INIT_STATE;
    EPwm2Regs.AQCTLA.all = AQCTLA_INIT_STATE;
    EPwm3Regs.AQCTLA.all = AQCTLA_INIT_STATE;


    EPwm1Regs.AQCTLA.bit.CAU = 0x01;
    EPwm1Regs.AQCTLA.bit.CAD = 0x02;

    EPwm2Regs.AQCTLA.bit.CAU = 0x01;
    EPwm2Regs.AQCTLA.bit.CAD = 0x02;

    EPwm3Regs.AQCTLA.bit.CAU = 0x01;
    EPwm3Regs.AQCTLA.bit.CAD = 0x02;

    EPwm1Regs.DBCTL.all = DBCTL_INIT_STATE;
    EPwm2Regs.DBCTL.all = DBCTL_INIT_STATE;
    EPwm3Regs.DBCTL.all = DBCTL_INIT_STATE;


    EPwm1Regs.DBCTL.bit.POLSEL = 0x01;
    EPwm2Regs.DBCTL.bit.POLSEL = 0x01;
    EPwm3Regs.DBCTL.bit.POLSEL = 0x01;

    // Init Dead-Band Generator Falling/Rising Edge Delay Register for EPWM1-EPWM3
    /*
    EPwm1Regs.DBFED = DBCNT_INIT_STATE;
    EPwm1Regs.DBRED = DBCNT_INIT_STATE;
    EPwm2Regs.DBFED = DBCNT_INIT_STATE;
    EPwm2Regs.DBRED = DBCNT_INIT_STATE;
    EPwm3Regs.DBFED = DBCNT_INIT_STATE;
    EPwm3Regs.DBRED = DBCNT_INIT_STATE;
    */

    EPwm1Regs.DBFED = 170;
    EPwm1Regs.DBRED = 170;
    EPwm2Regs.DBFED = 170;
    EPwm2Regs.DBRED = 170;
    EPwm3Regs.DBFED = 170;
    EPwm3Regs.DBRED = 170;

    // Init PWM Chopper Control Register for EPWM1-EPWM3
    EPwm1Regs.PCCTL.all = PCCTL_INIT_STATE;
    EPwm2Regs.PCCTL.all = PCCTL_INIT_STATE;
    EPwm3Regs.PCCTL.all = PCCTL_INIT_STATE;

    // Init Trip Zone Select Register
    EPwm1Regs.TZSEL.all = TZSEL_INIT_STATE;
    EPwm2Regs.TZSEL.all = TZSEL_INIT_STATE;
    EPwm3Regs.TZSEL.all = TZSEL_INIT_STATE;

    //Init Trip Zone Control Register
    EPwm1Regs.TZCTL.all = TZCTL_INIT_STATE;
    EPwm2Regs.TZCTL.all = TZCTL_INIT_STATE;
    EPwm3Regs.TZCTL.all = TZCTL_INIT_STATE;

    EPwm1Regs.TZSEL.bit.OSHT1 = 1;
    EPwm2Regs.TZSEL.bit.OSHT1 = 1;
    EPwm3Regs.TZSEL.bit.OSHT1 = 1;

    EPwm1Regs.TZCTL.bit.TZA = 1;
    EPwm1Regs.TZCTL.bit.TZB = 1;

    EPwm2Regs.TZCTL.bit.TZA = 1;
    EPwm2Regs.TZCTL.bit.TZB = 1;

    EPwm3Regs.TZCTL.bit.TZA = 1;
    EPwm3Regs.TZCTL.bit.TZB = 1;

    EPwm1Regs.TZEINT.bit.OST = 1;
    EPwm2Regs.TZEINT.bit.OST = 1;
    EPwm3Regs.TZEINT.bit.OST = 1;

    EPwm1Regs.TZFLG.bit.OST = 1;
    EPwm2Regs.TZFLG.bit.OST = 1;
    EPwm3Regs.TZFLG.bit.OST = 1;

    EPwm1Regs.CMPA.half.CMPA = 0;
    EPwm2Regs.CMPA.half.CMPA = 0;
    EPwm3Regs.CMPA.half.CMPA = 0;

    EPwm1Regs.ETSEL.bit.INTEN = 1;   // Enable EPWM1INT generation
    EPwm1Regs.ETSEL.bit.INTSEL = 1;  // Enable interrupt CNT_zero event
    EPwm1Regs.ETPS.bit.INTPRD = 1;   // Generate interrupt on the 1st event
    EPwm1Regs.ETCLR.bit.INT = 1;     // Enable more interrupts
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;        // Enable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = 4;
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;

    EDIS;
}



void UserIoInit(void)
{
    EALLOW;
    //ELEG1  out
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;
    //ELEG2  out
    GpioCtrlRegs.GPBPUD.bit.GPIO40 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO40 = 1;
    //ELEG3  out
    GpioCtrlRegs.GPBPUD.bit.GPIO34 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
    //ELEG4  out
    GpioCtrlRegs.GPAPUD.bit.GPIO13 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO13 = 1;
    //ELEG5  out
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;

    //E_Lock
    GpioCtrlRegs.GPBPUD.bit.GPIO42 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO42 = 1;

    //RGON out
    GpioCtrlRegs.GPBPUD.bit.GPIO39 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO39 = 1;

    //RLYON out
    GpioCtrlRegs.GPAPUD.bit.GPIO30 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO30 = 1;

    //DischargeResDetect  int
    GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO25 = 0;

    //PowerDirection        int
    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO8 = 0;

    //SouDongDaoFengSwitch_In  in
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO20 = 0;

    //AnQuanSwitch_In  in
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO14 = 0;


    //FootStoolCutWire_In  in
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO22 = 0;

    //FootStoolTaiYaJiao_In
    GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO24 = 0;

    /*
    //JiTouSwitch_In    int
    GpioCtrlRegs.GPBPUD.bit.GPIO39=0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO39=0;
    GpioCtrlRegs.GPBDIR.bit.GPIO39=0;

    //FootStoolLiShi_In  in
    GpioCtrlRegs.GPBPUD.bit.GPIO40=0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO40=0;
    GpioCtrlRegs.GPBDIR.bit.GPIO40=0;
    */
    EDIS;
}
