#ifndef __PIDIqs_H__
#define __PIDIqs_H__

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
    short SpeedFdbp;
    int  UQs;
    int   Out;   		// Output: PID output
    short  CurrentRef;
    void (*calc)();	  	// Pointer to calculation function
} PIDIqs;

typedef PIDIqs* PIDIqs_handle;

/*-----------------------------------------------------------------------------
Default initalizer for the PIDREG3 object.
-----------------------------------------------------------------------------*/
#define PIDIqs_DEFAULTS { 0, \
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
		(void (*)(unsigned int))pidIqs_calc }

/*------------------------------------------------------------------------------
Prototypes for the functions in PIDREG3.C
------------------------------------------------------------------------------*/
void pidIqs_calc(PIDIqs_handle);
#endif



