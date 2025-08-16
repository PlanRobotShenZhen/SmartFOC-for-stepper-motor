#ifndef __EXTERNGLOBALS_H__
#define __EXTERNGLOBALS_H__
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
extern short SoftwareVersion ;
extern short HardwareVersion ;
extern short MotorVersion;
extern PIDIds UdsPid;
extern PIDIqs UqsPid;
extern PIDC pidpt;           //电流环

extern PIDTorque pidptv;          //电流环 -速度模式用

extern PIDSpeed pidpv;
extern Pidholding pidholding;
extern PIDpos pidc_position;
extern SVPVM svpwm;
extern TIMER timer;
extern SYSTEMVAR SystemVar;
extern UARTMODE UartMode;
extern SYSTEMERROR SystemError;
extern MOTORCONTROL MotorControler;
extern short MotorVar[M_MOTORVARNUM_MAX];
extern  const short MotorVarInitTable[M_MOTORVARNUM_MAX];
extern  const short MotorVarMax[M_MOTORVARNUM_MAX];
extern  const short MotorVarMin[M_MOTORVARNUM_MAX];
extern short SysVar[M_SYSVARNUM_MAX];
extern  short SysVarUser[M_SYSVARNUM_MAX];
extern  const short SysVarInitTable[M_SYSVARNUM_MAX];
extern  const short SysVarMax[M_SYSVARNUM_MAX];
extern  const short SysVarMin[M_SYSVARNUM_MAX];
extern  short ProcessVar[M_PROCESSVARNUM_MAX];
extern  const short ProcessVarInitTable[M_PROCESSVARNUM_MAX];
extern short MonitorVar[100];

extern long dbufferU[(FIR_ORDER + 2) / 2];
extern long dbufferV[(FIR_ORDER + 2) / 2];
extern long dbufferHall[(FIR_ORDER + 2) / 2];
extern long dbufferEle[(FIR_ORDER + 2) / 2];
extern long dbufferVolt[(FIR_ORDER + 2) / 2];

extern  short SysErrSave[16];
extern short SysErrCountSave[15];

/****参数保存*********/
#define FLASH_ADDRESS0 0x0803B800
#define FLASH_ADDRESS1 0x0803C800
#define FLASH_ADDRESS2 0x0803D800
extern int VarDateFlash0[512];
extern int VarDateFlash1[512];
extern int VarDateFlash2[512];


extern float PostionPlan_accel_max;
extern float PostionPlan_a_accel;
extern float PostionPlan_a_decel;
extern float PostionPlan_decel_max;
extern short PostionPlan_vel_init;
extern short PostionPlan_vel_tar;

/****PID参数*********/
extern short Uds_OutMax;
extern short Uds_OutMin;
extern int Uds_UiMax;
extern int Uds_UiMin;
extern short Uds_Kp;
extern short Uds_Ki;

extern short Uqs_OutMax;
extern short Uqs_OutMin;
extern int Uqs_UiMax;
extern int Uqs_UiMin;
extern short Uqs_Kp;
extern short Uqs_Ki;

extern short pidpt_OutMax;
extern short pidpt_OutMin;
extern int pidpt_UiMax;
extern int pidpt_UiMin;
extern short pidpt_Kp;
extern short pidpt_Ki;


extern short pidpv_OutMax;
extern short pidpv_OutMin;
extern int pidpv_UiMax;
extern int pidpv_UiMin;
extern int pidpv_Kp;
extern int pidpv_Ki;

extern short   pidc_position_OutMax;
extern short   pidc_position_OutMin;
extern int     pidc_position_UiMax;
extern int     pidc_position_UiMin;
extern short   pidc_position_Kp;
extern short   pidc_position_Ki;
extern short   pidc_position_Err;

extern short   pidholding_OutMax;
extern short   pidholding_OutMin;
extern int     pidholding_UiMax;
extern int     pidholding_UiMin;
extern short   pidholding_Kp;
extern short   pidholding_Ki;
extern short   pidholding_Kd;

/****PID参数*********/
extern short  LedStateR;
extern short  LedStateG;

extern short SpiReadState;
extern short SpiReadState2;
#endif
