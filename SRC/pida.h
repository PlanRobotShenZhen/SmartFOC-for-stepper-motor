
#ifndef __PIDA_H__
#define __PIDA_H__
typedef struct
{
    long  Ref;   		// Input: Reference input
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
    short   KfdbAdd;
    int  KfdbSub;
    int  AddTor;
    short   SubTor;
    short   OutQs;   		// Output: PID output
    short   OutDs;   		// Output: PID output
    short  EzhAdd;
    short  EzhSub;
    short  OutAre;
    short  OutAdd;
    short KfdbAddDs;
    short KfdbSubDs;
    short KfdbSatDs;
    short AddTorDs;
    short SubTorDs;
    short SatTorDs;
    short InvarianceAddTime;
    void (*calc)();	  	// Pointer to calculation function
} PIDA;

typedef PIDA* PIDA_handle;

/*-----------------------------------------------------------------------------
Default initalizer for the PIDREG3 object.
-----------------------------------------------------------------------------*/
#define PIDA_DEFAULTS { 0, \
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
		0, \
		(void (*)(unsigned int))pida_calc }

/*------------------------------------------------------------------------------
Prototypes for the functions in PIDREG3.C
------------------------------------------------------------------------------*/
void pida_calc(PIDA_handle);


#endif
