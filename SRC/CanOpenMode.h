#ifndef __CANOPENMODE_H__
#define __CANOPENMODE_H__

typedef struct
{
    unsigned short CiA402_RunMode;        //0x6060
    unsigned short CiA402_CMD;            //0x6040
    unsigned short CiA402_State;          //0x6041
    unsigned short CiA402_QuickStopState; //0x605A
    int CiA402_TargetPosition;            //0x607A
    int CiA402_SpeedRef;                  //0x60FF

    unsigned short CanOpen_Last_ClearError; //��һ�ο�����0x6040��bit7
    unsigned short CanOpen_Last_RunMode;    //��һ������ģʽ
    unsigned short CanOpen_Running;         //�������б�־
    short CanOpen_Mode1State;               //λ��ģʽ״̬
    short CanOpen_Mode3State;               //�ٶ�ģʽ״̬
    short CanOpen_Mode6State;               //ת��ģʽ״̬

    _Bool  CiA402_RcadyToSwitchOn;
    _Bool  CiA402_SwitchedOn;
    _Bool  CiA402_OperationEnabled;
    _Bool  CiA402_Fault;
    _Bool  CiA402_VoltageEnabled;
    _Bool  CiA402_QuickStop;
    _Bool  CiA402_SwitchOnDisabled;
    _Bool  CiA402_Warning;
    _Bool  CiA402_ManufacturerSpecific;
    _Bool  CiA402_Remote;
    _Bool  CiA402_TargetReached;
    _Bool  CiA402_IntermalLimmitActive;
    _Bool  CiA402_OperationModeSpecific0;
    _Bool  CiA402_OperationModeSpecific1;
    _Bool  CiA402_ManufacturerSpecific0;
    _Bool  CiA402_ManufacturerSpecific1;
} CANOPENMODE;
extern CANOPENMODE CanOpenMode;

//��ʼ����״̬����0x0250
#define CANOPENMODE_DEFAULT { 0, \
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
				1, \
				0, \
				1, \
				0, \
				0, \
				1, \
				0, \
				0, \
				0, \
				0, \
				0, \
				0   }


#endif
