#ifndef __SVPWM_H__
#define __SVPWM_H__
#include "IQmathLib.h" 
typedef struct 
{              
		short SvpwmControlState;																						 //switch case number  ����˶�����
	  short  DcCoeff;																											 //Dc ����ʽϵ��
		short  As;  		// Input: phase-a stator variable										 -- A�ඨ�ӱ���
		short  Bs;			// Input: phase-b stator variable 									 -- B�ඨ�ӱ���
		short  Alpha;		// Output: stationary d-axis stator variable 				 --�����D�ᶨ�ӱ���
		short  Beta;		// Output: stationary q-axis stator variable				 --�����q�ᶨ�ӱ���
		short  Angle;		// Input: rotating angle (pu)  											 --��ת�Ƕ� pu
	  short  Angle2;	// Input: rotating angle (pu)  
		short  CurrentRef;																									//�����ο�ֵ
		short  CurrentFdb;																									//��������ֵ
		int UQ_MAX;
		int UD_MAX;	
		short  Sin;																													//����
		short  Cos;																													//����
	
		short  IDs;			// Output: rotating d-axis stator variable 					//���������D���������
		short  IQs;			// Output: rotating q-axis stator variable 					//���������Q���������
	
		short  UDs;			 
		int  UQs;		
	
		short  UQsRef;																											//	
		short  UQsstep;				
		//
		short  Ualpha; 			// Input: reference alpha-axis phase voltage 			���룺����Ĳο���ѹ
		short  Ubeta;				// Input: reference beta-axis phase voltage 			���룺����Ĳο���ѹ
		
		short  Ta;					// Output: reference phase-a switching function		
		short  Tb;					// Output: reference phase-b switching function 
		short  Tc;					// Output: reference phase-c switching function

		short  Va;																													//A��ռ�ձ�&A�໥��ռ�ձ�																								
		short  Vb;																													//B��ռ�ձ�&B�໥��ռ�ձ�														
		short  Vc;    																											//C��ռ�ձ�&C�໥��ռ�ձ�

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
