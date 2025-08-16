#ifndef __FUNCTION_H__
#define __FUNCTION_H__
#include <stdint.h>
#include "svpwm_dsp.h"
#include "Timer1.h"


extern short Ids_filter(short input);
extern void Set_DRV8323(void);
extern void CiA402Mode_Runing(void);
extern void UartMode_Runing(void);
//extern double SPIangleMT6835; //µ±Ç°½Ç¶È
extern void delay_Ms(uint16_t nms);
//extern double SPIGetDegreeMT6835(void);
extern void ReadVar(void);
extern void LostCoder(void);
extern void LostPhase(void);
extern void TorsionAnalyse(void);
extern void SpeedAnalyse(void);
extern void SysErrManage(void);
extern void SaveAllRsetVar(void);
extern void SaveAllVar(void);
extern void PowerManage(void);

extern void clarke(SVPVM *v);
extern void ipark(SVPVM *v);
extern void svgendq(SVPVM *v);
extern void PWM(SVPVM *v);

extern void PwmShut(void);
extern void PwmOpen(void);

extern void CurrentLoopISR(void);
extern void Timeing(TIMER *v);
extern void SpeedCalculate(void);
extern void MotorControl(void);

extern void CANopen_Init(void);
extern void CANopen_Task(void);
extern void Led(void);
#endif
