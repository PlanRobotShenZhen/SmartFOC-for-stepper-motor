#ifndef __PIDT_H__
#define __PIDT_H__

typedef struct
{
    int  Ref;   		// Input: Reference input
    int  Fdb;   		// Input: Feedback input
    int  Err;			// Variable: Error
    int  Kp;			// Parameter: Proportional gain
    int  Ki;			// Parameter: Integral gain
    int  Up;			// Variable: Proportional output
    int  Ui;			// Variable: Integral output
    int  OutPreSat;	// Variable: Pre-saturated output
    short   OutMax;		// Parameter: Maximum output
    short   OutMin;		// Parameter: Minimum output
    int  UiMax;		// Parameter: Integral Maximum output
    int  UiMin;		// Parameter: Integral Minimum output
    short   Out;   		// Output: PID output
    void (*calc)();	  	// Pointer to calculation function
} PIDTorque;

typedef PIDTorque* PIDTorque_handle;

/*-----------------------------------------------------------------------------
Default initalizer for the PIDREG3 object.
-----------------------------------------------------------------------------*/
#define PIDTorque_DEFAULTS { 0, \
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
		(void (*)(unsigned int))pidtorque_calc }

/*------------------------------------------------------------------------------
Prototypes for the functions in PIDREG3.C
------------------------------------------------------------------------------*/
void pidtorque_calc(PIDTorque_handle);
#endif



