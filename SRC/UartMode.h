#ifndef __UARTMODE_H__
#define __UARTMODE_H__

typedef struct
{
    short Mode;
    short ModePass;//ģʽ�л�����Ҫ�������Բ���
    short ModePass2;//ģʽ�л�����Ҫ�������Բ���
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
