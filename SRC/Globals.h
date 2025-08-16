#include "DeviceConfig.h"
#include "pidids.h"
#include "pida.h"
#include "pidc.h"
#include "IQmathLib.h"
#include "Timer.h"
#include "Timer1.h"
#include "svpwm_dsp.h"
#include "SystemVar.h"
#include "UartMode.h"
#include "SystemError.h"
#include "MotorControl.h"
#include "CanOpenMode.h"
#include "pidcspeed.h"
#include "pidIqs.h"
#include "uart_interface.h"
#include "SpeedPlan2.h"
#include "pidtorque.h"
#include "Pidposition.h"
#include "Pidholding.h"
MBModify modify;  //modbus修改判断结构体

short SoftwareVersion = 1007;
short HardwareVersion = 1007;
short MotorVersion    = 1007;

PIDIds       UdsPid = PIDIds_DEFAULTS;
PIDTorque pidptv = PIDTorque_DEFAULTS;            //电流环 -速度模式用
PIDSpeed  pidpv  = PIDSPEDD_DEFAULTS;

PIDIqs UqsPid = PIDIqs_DEFAULTS;

PIDC pidpt = PIDC_DEFAULTS;             //电流环

Pidholding pidholding = PIDCh_DEFAULTS;

PIDpos pidc_position = PIDpos_DEFAULTS;

SVPVM svpwm = SVPVM_DEFAULTS;
TIMER timer = TIMER_INITSTATE;

SYSTEMVAR SystemVar;
UARTMODE UartMode ;
CANOPENMODE CanOpenMode = CANOPENMODE_DEFAULT;
SYSTEMERROR SystemError;
MOTORCONTROL MotorControler;
short MotorVar[M_MOTORVARNUM_MAX] =
{
    4000, 85, 40, 35, 40, 3800, 3800, 1000, 3400, 4000,
    140, 0, 5, 265, 3000, 170, 3000, 245, 100, 300,
    40, 1500, 3000, 210, 5400, 28, 7000, 9000, 0, 0,

    30, 5500, 55, 6500, 15, 35, 30, 1000, 1200, 800,
    60, 30, 65, 20, 0,
    0, 4500, 65, 6500, 15, 35, 30, 1000, 1200, 800,
    90, 100, 100, 85, 0,
    0, 4500, 55, 6500, 15, 35, 30, 1000, 1200, 800,
    60, 0, 65, 85, 0,
    0, 4500, 55, 6500, 15, 35, 30, 1000, 1200, 800,
    60, 30, 65, 85, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0
};

const short MotorVarInitTable[M_MOTORVARNUM_MAX] =
{
    4000, 85, 40, 35, 40, 3800, 3800, 1000, 3400, 4000,
    140, 0, 5, 265, 3000, 170, 3000, 245, 100, 300,
    40, 1500, 3000, 210, 5400, 28, 7000, 9000, 0, 0,

    30, 5500, 55, 6500, 15, 35, 30, 1000, 1200, 800,
    60, 30, 65, 20, 0,
    0, 4500, 65, 6500, 15, 35, 30, 1000, 1200, 800,
    90, 100, 100, 85, 0,
    0, 4500, 55, 6500, 15, 35, 30, 1000, 1200, 800,
    60, 0, 65, 85, 0,
    0, 4500, 55, 6500, 15, 35, 30, 1000, 1200, 800,
    60, 30, 65, 85, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0
};


const short MotorVarMax[M_MOTORVARNUM_MAX] =
{
    5000, 150, 85, 85, 63, 4095, 4095, 4095, 4095, 4095,
    200, 200, 20, 300, 6000, 130, 6000, 300, 600, 600,
    100, 3000, 6000, 300, 9000, 100, 2000, 9000, 9000, 9000,
    30, 9000, 100, 9000, 90, 90, 90, 9000, 9000, 9000,
    150, 150, 120, 150, 9000,
    30, 9000, 100, 9000, 90, 90, 90, 9000, 9000, 9000,
    150, 150, 120, 150, 9000,
    30, 9000, 100, 9000, 90, 90, 90, 9000, 9000, 9000,
    150, 150, 120, 150, 9000,
    30, 9000, 100, 9000, 90, 90, 90, 9000, 9000, 9000,
    150, 150, 120, 150, 9000,
    9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999
};

const short MotorVarMin[M_MOTORVARNUM_MAX] =
{
    0, 10, 10, 10, 10, 0, 0, 0, 0, 0,
    20, 0, 1, 160, 1000, 80, 1000, 10, 10, 10,
    1, 100, 10, 100, 1000, 0, 1000, 1000, 0, 0,
    0, 1000, 0, 1000, 0, 0, 0, 1000, 1000, 1000,
    0, 0, 20, 10, 0,
    0, 1000, 0, 1000, 0, 0, 0, 1000, 1000, 1000,
    0, 0, 20, 10, 0,
    0, 1000, 0, 1000, 0, 0, 0, 1000, 1000, 1000,
    0, 0, 20, 10, 0,
    0, 1000, 0, 1000, 0, 0, 0, 1000, 1000, 1000,
    0, 0, 20, 10, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

short SysVar[M_SYSVARNUM_MAX] =
{
    50, 8, 200, 4000, 3500, 1800, 1800, 1, 1, 1800, //0--9
    175, 350, 350, 800, 3000, 2500, 50, 0, 0, 0,
    0, 0, 2, 500, 10, 2000, 2000, 200, 2, 3,
    2650, 100, 5, 2, 1, 200, 250, 1, 4000, 3000,
    7, 5, 7, 5, 7, 5, 0, 2000, 2000, 1,
    1, 0, 100, 70, 0, 0, 0, 250, 2, 3, //50-59
    20, 50, 50, 38, 0, 0, 2, 675, 675, 1000,
    10, 10, 0, 0, 0, 0, 0, 10, 29, 30, //70-79
    50, 200, 2000, 300, 300, 1850, 600, 125, 512, 600,
    650, 0, 55, 45, 3, 100, 100, 960, 888, 2222
};

short SysVarUser[M_SYSVARNUM_MAX] =
{
    50, 8, 200, 4000, 3500, 1800, 1800, 1, 1, 1800, //0--9
    175, 350, 350, 800, 3000, 2500, 50, 0, 0, 0,
    0, 0, 2, 500, 10, 2000, 2000, 200, 2, 3,
    2650, 100, 5, 2, 1, 200, 250, 1, 4000, 3000,
    7, 5, 7, 5, 7, 5, 0, 2000, 2000, 1,
    1, 0, 100, 70, 0, 0, 0, 250, 2, 3, //50-59
    20, 50, 50, 38, 0, 0, 2, 675, 675, 1000,
    10, 10, 0, 0, 0, 0, 0, 10, 29, 30, //70-79
    50, 200, 2000, 300, 300, 1850, 600, 125, 512, 600,
    650, 0, 55, 45, 3, 100, 100, 960, 888, 2222
};

const short SysVarInitTable[M_SYSVARNUM_MAX] =
{
    50, 8, 200, 4000, 3500, 1800, 1800, 1, 1, 1800, //0--9
    175, 350, 350, 800, 3000, 2500, 50, 0, 0, 0,
    0, 0, 2, 500, 10, 2000, 2000, 200, 2, 3,
    2650, 100, 5, 2, 1, 200, 250, 1, 4000, 3000,
    7, 5, 7, 5, 7, 5, 0, 2000, 2000, 1,
    1, 0, 100, 70, 0, 0, 0, 250, 2, 3, //50-59
    20, 50, 50, 38, 0, 0, 2, 675, 675, 1000,
    10, 10, 0, 0, 0, 0, 0, 10, 29, 30, //70-79
    50, 200, 2000, 300, 300, 1850, 600, 125, 512, 600,
    650, 0, 55, 45, 3, 100, 100, 960, 888, 2222
};

const short SysVarMax[M_SYSVARNUM_MAX] =
{
    100, 8, 500, 5000, 4000, 3000, 3000, 1, 1, 3000, //0--9
    200, 1000, 1000, 1900, 4095, 3000, 1000, 1, 1, 1,
    9999, 2, 15, 3000, 100, 9999, 9999, 500, 50, 50,
    3700, 500, 15, 15, 1, 500, 500, 1, 5000, 4000,
    16, 16, 16, 16, 16, 16, 1, 9999, 9999, 1,
    4, 1, 9999, 9999, 1, 1, 1000, 1000, 50, 50,   //50-59
    120, 800, 800, 120, 900, 1, 3, 1000, 1000, 9999,
    100, 100, 1, 1, 3, 9999, 9999, 100, 100, 100, //70-79
    1000, 1000, 2650, 500, 500, 2000, 1230, 250, 600, 700,
    800, 3, 9999, 9999, 16, 500, 500, 1900, 9999, 9999
};

const short SysVarMin[M_SYSVARNUM_MAX] =
{
    1, 1, 150, 150, 150, 200, 200, 0, 0, 200,     //0--9
    100, 0, 0, 600, 1500, 200, 20, 0, 0, 0,
    0, 0, 0, 200, 1, 1, 0, 20, 1, 1,
    1300, 0, 1, 1, 0, 200, 150, 0, 200, 200,
    0, 0, 0, 0, 0, 0, 0, 100, 100, 0,            //40-49
    0, 0, 30, 20, 0, 0, 0, 20, 1, 1,
    1, 20, 20, 0, 0, 0, 0, 200, 200, 100,
    1, 1, 0, 0, 0, 0, 0, 0, 0, 3,
    1, 1, 1, 1, 1, 1, 3, 50, 400, 500,
    600, 0, 0, 0, 0, 0, 0, 0, 0, 0
};



short ProcessVar[M_PROCESSVARNUM_MAX] =
{
    1, 1, 0, 1, 1, 1, 1, 1, 1, 1,
    2, 2, 3, 3, 3, 3, 3, 3, 1, 1, //  Free
    2, 2, 3, 3, 3, 3, 15, 15, 15, 15, // Fixed1
    2, 2, 3, 3, 3, 3, 15, 15, 15, 15, //Fixed4
    2, 2, 3, 3, 3, 3, 15, 15, 15, 15, //Fixed7
    2, 2, 3, 3, 3, 3, 15, 15, 15, 15, //Fixed8
    1, 1, 4, 4, 5, 1, 1, 4, 4, 5, //Fixedw
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0
};

const short ProcessVarInitTable[M_PROCESSVARNUM_MAX] =
{
    1, 1, 0, 1, 1, 1, 1, 1, 1, 1,
    2, 2, 3, 3, 3, 3, 3, 3, 1, 1, //  Free
    2, 2, 3, 3, 3, 3, 15, 15, 15, 15, // Fixed1
    2, 2, 3, 3, 3, 3, 15, 15, 15, 15, //Fixed4
    2, 2, 3, 3, 3, 3, 15, 15, 15, 15, //Fixed7
    2, 2, 3, 3, 3, 3, 15, 15, 15, 15, //Fixed8
    1, 1, 4, 4, 5, 1, 1, 4, 4, 5, //Fixedw
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0
};

short MonitorVar[100];


long dbufferU[(FIR_ORDER + 2) / 2];
long dbufferV[(FIR_ORDER + 2) / 2];
long dbufferHall[(FIR_ORDER + 2) / 2];
long dbufferEle[(FIR_ORDER + 2) / 2];
long dbufferVolt[(FIR_ORDER + 2) / 2];

short SysErrSave[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
short SysErrCountSave[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/****参数保存*********/
#define FLASH_ADDRESS0 0x0803B800
#define FLASH_ADDRESS1 0x0803C800
#define FLASH_ADDRESS2 0x0803D800
int VarDateFlash0[512];
int VarDateFlash1[512];
int VarDateFlash2[512];


//short PostionPlan_accel_max ;
//short PostionPlan_a_accel ;
//short PostionPlan_a_decel ;
//short PostionPlan_decel_max ;
//short PostionPlan_vel_init ;
//short PostionPlan_vel_tar;

float PostionPlan_accel_max ;
float PostionPlan_a_accel ;
float PostionPlan_a_decel ;
float PostionPlan_decel_max ;
short PostionPlan_vel_init ;
short PostionPlan_vel_tar;


/****参数保存*********/


/****PID参数*********/
short Uds_OutMax = _IQ15(0.5);
short Uds_OutMin = _IQ15(-0.5);
int Uds_UiMax = _IQ30(0.5);
int Uds_UiMin = _IQ30(-0.5);
short Uds_Kp = 30;
short Uds_Ki = 30;

short Uqs_OutMax = _IQ15(0.8);
short Uqs_OutMin = _IQ15(-0.8);
int Uqs_UiMax = _IQ30(0.8);
int Uqs_UiMin = _IQ30(-0.8);
short Uqs_Kp = 300;
short Uqs_Ki = 6;

short pidpt_OutMax = _IQ15(0.5);
short pidpt_OutMin = _IQ15(-0.5);
int pidpt_UiMax = _IQ30(0.5);
int pidpt_UiMin = _IQ30(-0.5);
short pidpt_Kp = 200;
short pidpt_Ki = 30;


short pidpv_OutMax = _IQ15(0.8);
short pidpv_OutMin = _IQ15(-0.8);
int pidpv_UiMax = _IQ30(0.8);
int pidpv_UiMin = _IQ30(-0.8);
int pidpv_Kp = 300;
int pidpv_Ki = 30;


short   pidc_position_OutMax = _IQ15(0.5);
short   pidc_position_OutMin = _IQ15(-0.5);
int   pidc_position_UiMax = _IQ30(0.5);
int   pidc_position_UiMin = _IQ30(-0.5);
short    pidc_position_Kp = 30;
short    pidc_position_Ki = 3;
short    pidc_position_Kd = 0;

short   pidholding_OutMax = _IQ15(0.2);
short   pidholding_OutMin = _IQ15(-0.2);
int   pidholding_UiMax = _IQ30(0.2);
int   pidholding_UiMin = _IQ30(-0.2);
short   pidholding_Kp = 30;
short   pidholding_Ki = 3;


short  LedStateR = 0;
short  LedStateG = 0;

short SpiReadState = 0;
short SpiReadState2  = 0;

/****PID参数*********/
