#ifndef __UARTMODE_H__
#define __UARTMODE_H__

typedef struct
{
    short Mode;
    short ModePass;//模式切换后，需要重置所以参数
    short ModePass2;//模式切换后，需要重置所以参数
    short CMD;
    short Speed;
    short VF_Ref;
    short State;
    short Enable;
    short DisEnable;
    short Start;
    short Stop;
    short Torque;
    short TorqueK;
    int TargetPosition;

} UARTMODE;

typedef UARTMODE* UARTMODE_handle;

#endif
