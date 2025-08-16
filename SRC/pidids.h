#ifndef __PIDIds_H__
#define __PIDIds_H__

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
} PIDIds;

typedef PIDIds* PIDIds_handle;

/*-----------------------------------------------------------------------------
Default initalizer for the PIDREG3 object.
-----------------------------------------------------------------------------*/
#define PIDIds_DEFAULTS { 0, \
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
		(void (*)(unsigned int))pidids_calc }

/*------------------------------------------------------------------------------
Prototypes for the functions in PIDREG3.C
------------------------------------------------------------------------------*/
void pidids_calc(PIDIds_handle);
#endif
