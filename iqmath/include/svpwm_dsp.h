#ifndef __SVPWM_H__
#define __SVPWM_H__
#include "IQmathLib.h" 
typedef struct 
{              
		short SvpwmControlState;																						 //switch case number  电机运动场景
	  short  DcCoeff;																											 //Dc 多项式系数
		short  As;  		// Input: phase-a stator variable										 -- A相定子变量
		short  Bs;			// Input: phase-b stator variable 									 -- B相定子变量
		short  Alpha;		// Output: stationary d-axis stator variable 				 --不变的D轴定子变量
		short  Beta;		// Output: stationary q-axis stator variable				 --不变的q轴定子变量
		short  Angle;		// Input: rotating angle (pu)  											 --旋转角度 pu
	  short  Angle2;	// Input: rotating angle (pu)  
		short  CurrentRef;																									//电流参考值
		short  CurrentFdb;																									//电流反馈值
		int UQ_MAX;
		int UD_MAX;	
		short  Sin;																													//正弦
		short  Cos;																													//余弦
	
		short  IDs;			// Output: rotating d-axis stator variable 					//输出：定子D轴电流变量
		short  IQs;			// Output: rotating q-axis stator variable 					//输出：定子Q轴电流变量
	
		short  UDs;			 
		int  UQs;		
	
		short  UQsRef;																											//	
		short  UQsstep;				
		//
		short  Ualpha; 			// Input: reference alpha-axis phase voltage 			输入：α轴的参考电压
		short  Ubeta;				// Input: reference beta-axis phase voltage 			输入：β轴的参考电压
		
		short  Ta;					// Output: reference phase-a switching function		
		short  Tb;					// Output: reference phase-b switching function 
		short  Tc;					// Output: reference phase-c switching function

		short  Va;																													//A相占空比&A相互补占空比																								
		short  Vb;																													//B相占空比&B相互补占空比														
		short  Vc;    																											//C相占空比&C相互补占空比

		short  PeriodMax;     // Parameter: PWM Half-Period in CPU clock cycles (Q0)
		short  MfuncPeriod;    // Input: Period scaler (Q15) 
		short  MfuncC1;        // Input: EPWM1 A&B Duty cycle ratio (Q15)
		short  MfuncC2;        // Input: EPWM2 A&B Duty cycle ratio (Q15) 
		short  MfuncC3;        // Input: EPWM3 A&B Duty cycle ratio (Q15)
} SVPVM;	  
				 				 
			           


/*-----------------------------------------------------------------------------
Default initalizer for the CLARKE object.
-----------------------------------------------------------------------------*/                     
#define SVPVM_DEFAULTS { _IQ15(0.99), \
		0, \
		0, \
		0, \
	  0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0}
#endif // __SVPWM_H__
