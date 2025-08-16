#ifndef __MOTORCONTROL_H__
#define __MOTORCONTROL_H__

typedef struct
{
    short State;
    short SpeedRef;
    short SpeedFdb;
    short SpeedAcc;
    short SpeedDcc;
    short TorqueRef;
    short TorqueFdb;
    int PositionRef;
    int PositionFdb;
    short Error;
    int MotorActivePostion;
    int RotorCount;
    short MaxTorque;
    short RatedTorque;
    short MaxSpeed;

    short SpeedFdbpFilter1;
    short SpeedFdbpFilter2;
    short SpeedFdbpFilter3;

    short SpeedFdbp;
    short MotorPoles;


    unsigned int spiread_anlge;
    short AngleFromMT6835;
    short AngleFromMT6835Offset;
    short AngleFromMT6835Offset1;

    short MT6835_Cla_State;


} MOTORCONTROL;

typedef MOTORCONTROL* MOTORCONTROL_handle;

#endif
