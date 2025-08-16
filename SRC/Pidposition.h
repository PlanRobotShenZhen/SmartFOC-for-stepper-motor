#ifndef __PIDpos_H__
#define __PIDpos_H__

typedef struct
{
    int  Ref;   		// Input: Reference input
    int  Fdb;   		// Input: Feedback input
    int  Speedref;   		// Input: Reference input
    int  Speedfdb;   		// Input: Reference input
    int  Err;			 // Variable: Error
	  int  Err_pass; //last error
    int  Kp;			// Parameter: Proportional gain
    int  Ki;			// Parameter: Integral gain
		int  Kd;
    int  Up;			// Variable: Proportional output
    int  Ui;			// Variable: Integral output
		int  Ud;
    int  OutPreSat;	// Variable: Pre-saturated output
    short OutMax;		// Parameter: Maximum output
    short OutMin;		// Parameter: Minimum output
    int  UiMax;		// Parameter: Integral Maximum output
    int  UiMin;		// Parameter: Integral Minimum output
    short   Out;   		// Output: PID output
    short Iqs;
    long long SumIqs;
    void (*calc)();	  	// Pointer to calculation function
} PIDpos;

typedef PIDpos* PIDpos_handle;

/*-----------------------------------------------------------------------------
Default initalizer for the PIDREG3 object.
-----------------------------------------------------------------------------*/
#define PIDpos_DEFAULTS { \
	  0,  /* Ref */ \
    0,  /* Fdb */ \
    0,  /* Speedref */ \
    0,  /* Speedfdb */ \
    0,  /* Err */ \
    0,  /* Err_pass */ \
    0,  /* Kp */ \
    0,  /* Ki */ \
    0,  /* Kd */ \
    0,  /* Up */ \
    0,  /* Ui */ \
    0,  /* Ud */ \
    0,  /* OutPreSat */ \
    0,  /* OutMax */ \
    0,  /* OutMin */ \
    0,  /* UiMax */ \
    0,  /* UiMin */ \
    0,  /* Out */ \
    0,  /* Iqs */ \
    0,  /* SumIqs */ \
    (void (*)(struct PIDpos*))pidposition_calc}

/*------------------------------------------------------------------------------
Prototypes for the functions in PIDREG3.C
------------------------------------------------------------------------------*/
void pidposition_calc(PIDpos_handle);
#endif



