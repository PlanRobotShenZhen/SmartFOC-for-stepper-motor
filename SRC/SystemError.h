#ifndef __SYSTEMERROR_H__
#define __SYSTEMERROR_H__

typedef struct
{
    short ImeasA;
    short ImeasB;
    short ImeasC;
    short ImeasAOffset;
    short ImeasBOffset;
    short ImeasCOffset;
    short ImeasOffsetFlag;

    short RotorMacAnglePastp;

    short SpeedFdbpPost;
    short MotorRunFlag;
    short OverLoadTimer; //ת�ع���
    short OverSpeedTimer;//ת�ٳ���
    short CoderTimer;
    int DcCoeff;
    int PositiveDcCoeff;

    short IqsMax;
    short IdsMax;
    short IqsMaxOverTime;
    short DcRef;
    short DcRefK;
    short DcRefB;

    short AdDcVolt;
    volatile short SysErr;
    short PwmOn;
    short LowSpeedTimer;
    short DcVolt;
    short DcRefMax;
    short DcMaxErrRef;
    short DcRefMin;
    short DcMinErrRef;
    short DcArrestMax;
    short DcArrestDelayMax;
    short DcArrest2DelayMax;
    short ImeMax;

    short LossACTimeMax;
    short NoSysErrPowerOff;
//**************************************//
    short ImeasCheckDelay;
    short SysErrRuningAsk;
    short SysErrRuning;
    /******2013.02.28********************/
    short SysResetVar;
    short SysErrSaveLock;
    short SaveAllRsetFlag;
    short SaveAllParaFlag;
    short RuningMode;  /****����ģʽ*********/
    short RuningModeFdb;  /****����ģʽ*********/
    short SysClearFlag;
    short OverSpeed;
    short Temp;
    short KpGainCoeff; //�л�PID��Kp�ı��ϵ��
    short KiGainCoeff; //�л�PID��Ki�ı��ϵ��
    short ThresholdCoeff; //�ż��л�ϵ�����������ֵ128���Եõ��ٷֱ�
    short TorqueCoeff;       //���ڸı��ٶȵ����Ť��ֵ������ֵ������λΪ1/4�
    short TorquePIDLimitCoeff;  //����PID�׶��޷������Ť��ֵ����λΪ1/4�
    short TorqueDECLimitCoeff;  //����ͬ�����ʱ�޷������Ť��ֵ����λΪ1/4�
} SYSTEMERROR;

typedef SYSTEMERROR* SYSTEMERROR_handle;

#endif
