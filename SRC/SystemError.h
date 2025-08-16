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
    short OverLoadTimer; //转矩过载
    short OverSpeedTimer;//转速超速
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
    short RuningMode;  /****运行模式*********/
    short RuningModeFdb;  /****运行模式*********/
    short SysClearFlag;
    short OverSpeed;
    short Temp;
    short KpGainCoeff; //切换PID后Kp改变的系数
    short KiGainCoeff; //切换PID后Ki改变的系数
    short ThresholdCoeff; //门槛切换系数，除以最大值128可以得到百分比
    short TorqueCoeff;       //用于改变速度的输出扭矩值（绝对值），单位为1/4额定
    short TorquePIDLimitCoeff;  //用于PID阶段限幅的输出扭矩值，单位为1/4额定
    short TorqueDECLimitCoeff;  //用于同向减速时限幅的输出扭矩值，单位为1/4额定
} SYSTEMERROR;

typedef SYSTEMERROR* SYSTEMERROR_handle;

#endif
