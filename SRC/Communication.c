#include "DSP2803x_Device.h"     // Headerfile Include File
#include "DSP2803x_Examples.h"   // Examples Include File
#include "ExternGlobals.h"
void AnalyseRxCommand(void);
void AnalyseTxCommand(void);
extern void ProcessVarInit2(void);

void AnalyseTxCommand(void)
{
    if(SciaRegs.SCICTL1.bit.TXENA == 1)
    {
        TxCommandRungFlag = 1;
    }
    else
    {
        if(SysErr == M_SYSERR_POWEROFF)
        {
            TxCommand[1] = M_DEBUG_FUN;
            TxCommand[2] = 1;
            TxCommand[3] = SysErr;
            TxCommand[4] = SysErrRuningAsk;
            CreatCRC();
            SciaRegs.SCICTL1.bit.TXENA = 1;
        }
        else if(AnalyseRxCommandFinish)
        {
            AnalyseRxCommandFinish = 0;
            CreatCRC();
            SciaRegs.SCICTL1.bit.TXENA = 1;
        }

        else if(TxModelDataFlag == 2)
        {
            TxCommand[1] = M_MODEL_DATA_2;
            TxCommand[2] =  SenSor + LedCut + LedCut2 + LedHaltAngle + LedTieg;
            TxCommand[3] = ((ProcessVar[(SewModel * 10)]) << 4)  + (ProcessVar[(SewModel * 10) + 1]); //SMB + EMB
            TxCommand[4] = (((ProcessVar[(SewModel * 10) + 2]) - 1) << 4)  + ((ProcessVar[(SewModel * 10) + 3]) - 1); //A<<4 + B
            TxCommand[5] = (((ProcessVar[(SewModel * 10) + 4]) - 1) << 4) + ((ProcessVar[(SewModel * 10) + 5]) - 1); //C<<4 +D
            CreatCRC();
            TxModelDataFlag--;
            SciaRegs.SCICTL1.bit.TXENA = 1;
        }
        else if(TxModelDataFlag == 1)
        {

            TxCommand[1] = M_MODEL_DATA_1;
            TxCommand[2] = ProcessVar[(SewModel * 10) + 6] - 1; //E
            TxCommand[3] = ProcessVar[(SewModel * 10) + 7] - 1; //F
            TxCommand[4] = ProcessVar[(SewModel * 10) + 8] - 1; //G
            TxCommand[5] = ProcessVar[(SewModel * 10) + 9] - 1; //H
            CreatCRC();
            TxModelDataFlag--;
            SciaRegs.SCICTL1.bit.TXENA = 1;
        }
        else if(SysResetVar == 2)
        {
            TxCommand[1] = M_DEBUG_FUN;
            TxCommand[2] = 3;
            TxCommand[3] = M_SYSERR_SYSVARRESET;
            TxCommand[4] = SysErrRuningAsk;
            CreatCRC();
            SciaRegs.SCICTL1.bit.TXENA = 1;
        }
        else if(SysErrRuningAsk)
        {

            TxCommand[1] = M_DEBUG_FUN;
            TxCommand[2] = 1;
            TxCommand[3] = SysErr;
            TxCommand[4] = SysErrRuningAsk;
            CreatCRC();
            SciaRegs.SCICTL1.bit.TXENA = 1;
        }
        else if(ProcessRuningAsk)
        {
            TxCommand[1] = M_DEBUG_FUN;
            TxCommand[2] = 2;
            TxCommand[3] = SewProcessState;
            TxCommand[4] = ProcessRuningAsk;
            CreatCRC();
            SciaRegs.SCICTL1.bit.TXENA = 1;
        }
    }
}


void AnalyseRxCommand(void)
{
    if(RxCommand[1] == M_DEBUG_FUN)
    {
        TxCommand[1] = M_DEBUG_FUN;
        if(RxCommand[2] == 1) //SysErr;
        {
            TxCommand[2] = 1;
            TxCommand[3] = SysErr;
            SysErrRuningAsk = RxCommand[4];
            TxCommand[4] = SysErrRuningAsk;
        }
        if(RxCommand[2] == 2)
        {
            TxCommand[2] = 2;
            TxCommand[3] = SewProcessState;
            ProcessRuningAsk = RxCommand[4];
            TxCommand[4] = ProcessRuningAsk;
        }
    }

    else if(RxCommand[1] == M_DEBUG_COMKEY)
    {
        TxCommand[1] = M_DEBUG_COMKEY;
        if(RxCommand[2] == 1) //软起动开关
        {
            TxCommand[2] = 1;
            if(RxCommand[4] == 1)
            {
                if(SlowStartFalg)
                {
                    SlowStartFalg = 0;
                    ProcessVar[2] = SlowStartFalg;
                    TxCommand[3] = 0;
                }
                else
                {
                    SlowStartFalg = 1;
                    ProcessVar[2] = SlowStartFalg;
                    TxCommand[3] = 1;
                }
            }
            else
            {
                TxCommand[3] = SlowStartFalg;
            }
        }
        else if(RxCommand[2] == 2) //招压
        {
            TxCommand[2] = 2;
            if(RxCommand[4] == 1)
            {
                if(TaiYaJiaoEleOn)
                {
                    TaiYaJiaoEleOn = 0;
                    SysVar[19] = 0;
                    SysVarUser[19] = 0;
                    TxCommand[3] = 0;
                }
                else
                {
                    TaiYaJiaoEleOn = 1;
                    SysVar[19] = 1;
                    SysVarUser[19] = 1;
                    TxCommand[3] = 1;
                }
                SysVarUserNum = 19;
                SysVarNum = 19;
                SvaeSFlag = 1;
                SvaeSUserFlag = 1;
            }
            else
            {
                TxCommand[3] = TaiYaJiaoEleOn;
            }
        }
        else if(RxCommand[2] == 3) //底线针数
        {
            TxCommand[2] = 3;
            TxCommand[3] = DangQianDixiZhongShu >> 7;
            TxCommand[4] = DangQianDixiZhongShu & 0x7f;
        }
        else if(RxCommand[2] == 5) //起针夹线器
        {
            TxCommand[2] = 5;
            if(RxCommand[4] == 1)
            {
                if(JiaXianQiEleOn)
                {
                    JiaXianQiEleOn = 0;
                    TxCommand[3] = 0;
                }
                else
                {
                    JiaXianQiEleOn = 1;
                    TxCommand[3] = 1;
                }
            }
            else
            {
                TxCommand[3] = JiaXianQiEleOn;
            }
            ProcessVar[3] = JiaXianQiEleOn;
        }
    }
    else if(RxCommand[1] == M_SERVER_TO_MASTER)
    {
        if(RxCommand[2] == 1)
        {

            TxCommand[2] =  1;
            TxCommand[3] = YouLiang >> 7;
        }
        else
        {
            TxCommand[2] =  0;
        }
        TxCommand[1] = M_SERVER_TO_MASTER;
        TxCommand[4] =  0;
        TxCommand[5] = 0;
    }
    else if(RxCommand[1] == M_DEBUG_SYN)
    {
        TxCommand[1]  = M_DEBUG_SYN;
        TxCommand[2]  = (MachineType << 4) + SewModel;
        TxCommand[3]  = SoftwareVersion;
        TxCommand[4]  = CraftsPassWord >> 7;
        TxCommand[5]  = CraftsPassWord & 0x7f;
    }

    else if(RxCommand[1] == M_DEBUG_RESET)
    {
        ResetVarAllFlag = RxCommand[3];
        TxCommand[1] = M_DEBUG_RESET;
        TxCommand[2] = 1;
        TxCommand[3] = ResetVarAllFlag;
        SysResetVar = 1;
    }
    else if(RxCommand[1] == M_DEBUG_IDLE)
    {
        TxCommand[1] =  M_DEBUG_IDLE;
        if(RxCommand[2] == 1) //修改SewModel;
        {
            ProcessVar[0] = RxCommand[3];
            SewModel = RxCommand[3];
            ProcessVarInit2();
            TxCommand[1] =  M_DEBUG_IDLE;
            TxCommand[2] = 1;
            TxCommand[3] =  SewModel;
            TxModelDataFlag = 2;
        }
        if(RxCommand[2] == 2) //ADD123
        {
            ProcessVar[(SewModel * 10) + RxCommand[3]] = RxCommand[4];
            TxCommand[1] =  M_DEBUG_IDLE;
            TxCommand[2] =  2;
            TxCommand[3] =  RxCommand[3];
            TxCommand[4] =  ProcessVar[(SewModel * 10) + RxCommand[3]];
            ProcessVarInit2();
        }
        if(RxCommand[2] == 3) //CUT
        {
            ProcessVar[1] = RxCommand[3];
            LedCut = RxCommand[3] & 0x40;
            LedHaltAngle = RxCommand[3] & 0x10;
            LedTieg =  RxCommand[3] & 0x08;
            TxCommand[1] =  M_DEBUG_IDLE;
            TxCommand[2] =  3;
            TxCommand[3] =  SenSor + LedCut + LedCut2 + LedHaltAngle + LedTieg;
        }
        if(RxCommand[2] == 4) //ADD
        {
            NeedleAddFalg = RxCommand[3];
            TxCommand[1] =  M_DEBUG_IDLE;
            TxCommand[2] =  4;
        }
        if(RxCommand[2] == 5) //LedLiangDu
        {
            TxCommand[1] =  M_DEBUG_IDLE;
            TxCommand[2] =  5;
            if(LedLiangDuDeLay == 0)
            {
                LedLiangDuDeLay = 3;
                LedLiangDu = LedLiangDu + 200;
                ProcessVar[4] = LedLiangDu;
            }
            if(LedLiangDu > 1000)
            {
                LedLiangDu = 0;
                ProcessVar[4] = 0;
            }
            TxCommand[3] =  0;
            TxCommand[4] =  0;
            TxCommand[5] =  0;
        }
    }
    else if(RxCommand[1] == M_DEBUG_PASSWORD)
    {
    }
    else if(RxCommand[1] == M_DEBUG_P) //2222
    {
        if(RxCommand[2] == 1)
        {
            SysVarUserNum = RxCommand[3];
            TxCommand[1] =  M_DEBUG_P;
            TxCommand[2] = SysVarUser[SysVarUserNum] >> 7;
            TxCommand[3] = SysVarUser[SysVarUserNum] & 0x7f;
        }
        if(RxCommand[2] == 2)
        {
            SysVarUser[SysVarUserNum] = (RxCommand[3] * 128) + RxCommand[4];
            if((SysVar[SysVarNum]) > SysVarMax[SysVarNum])
            {
                SysVarUser[SysVarNum] = SysVarMax[SysVarNum];
            }
            else if((SysVar[SysVarNum]) < SysVarMin[SysVarNum])
            {
                SysVarUser[SysVarNum] = SysVarMin[SysVarNum];
            }
            TxCommand[1] =  M_DEBUG_P;
            TxCommand[2] = SysVarUser[SysVarUserNum] >> 7;
            TxCommand[3] = SysVarUser[SysVarUserNum] & 0x7f;
        }
        if(RxCommand[2] == 3)
        {
            SysVarUserNum = RxCommand[3];
            TxCommand[1] =  M_DEBUG_P;
            TxCommand[2] = SysVarUser[SysVarUserNum] >> 7;
            TxCommand[3] = SysVarUser[SysVarUserNum] & 0x7f;
            SysVarUserInit();
            if(SysVarUser[98] == 8888)
            {
                SysVarUser[98] = 888;
                ResetSysVarUserFlag = 1;
                SysResetVar = 1;
            }
            else
            {
                SvaeSUserFlag = 1;
            }
        }
        TxCommand[4] =  SysVarMin[SysVarUserNum] >> 7;
        TxCommand[5] =  SysVarMin[SysVarUserNum] & 0x7f;
        TxCommand[6] =  SysVarMax[SysVarUserNum] >> 7;
        TxCommand[7] =  SysVarMax[SysVarUserNum] & 0x7f;
    }
    else if(RxCommand[1] == M_DEBUG_F) //3333
    {
        if(RxCommand[2] == 1)
        {
            MotorVarNum = RxCommand[3];
            TxCommand[1] =  M_DEBUG_F;
            TxCommand[2] = MotorVar[MotorVarNum] >> 7;
            TxCommand[3] = MotorVar[MotorVarNum] & 0x7f;
        }
        if(RxCommand[2] == 2)
        {
            MotorVar[MotorVarNum] = (RxCommand[3] * 128) + RxCommand[4];

            if((MotorVar[MotorVarNum]) > MotorVarMax[MotorVarNum])
            {
                MotorVar[MotorVarNum] = MotorVarMax[MotorVarNum];
            }
            else if((MotorVar[MotorVarNum]) < MotorVarMin[MotorVarNum])
            {
                MotorVar[MotorVarNum] = MotorVarMin[MotorVarNum];
            }

            TxCommand[1] =  M_DEBUG_F;
            TxCommand[2] = MotorVar[MotorVarNum] >> 7;
            TxCommand[3] = MotorVar[MotorVarNum] & 0x7f;
        }
        if(RxCommand[2] == 3)
        {
            MotorVarNum = RxCommand[3];
            TxCommand[1] =  M_DEBUG_F;
            TxCommand[2] = MotorVar[MotorVarNum] >> 7;
            TxCommand[3] = MotorVar[MotorVarNum] & 0x7f;
            SvaeMFlag = 1;
            MotorVarInit();
        }
        TxCommand[4] =  MotorVarMin[MotorVarNum] >> 7;
        TxCommand[5] =  MotorVarMin[MotorVarNum] & 0x7f;
        TxCommand[6] =  MotorVarMax[MotorVarNum] >> 7;
        TxCommand[7] =  MotorVarMax[MotorVarNum] & 0x7f;
    }
    else if(RxCommand[1] == M_DEBUG_MONITOR)
    {
    }
    else if(RxCommand[1] == M_DEBUG_SIX) //1111
    {
        if(RxCommand[2] == 1)
        {
            SysVarNum = RxCommand[3];
            SysVarUserNum = SysVarNum ;
            TxCommand[1] =  M_DEBUG_SIX;
            TxCommand[2] =  SysVar[SysVarNum] >> 7;
            TxCommand[3] =  SysVar[SysVarNum] & 0x7f;
        }
        if(RxCommand[2] == 2)
        {
            SysVar[SysVarNum] = (RxCommand[3] * 128) + RxCommand[4];
            if((SysVar[SysVarNum]) > SysVarMax[SysVarNum])
            {
                SysVar[SysVarNum] = SysVarMax[SysVarNum];
            }
            else if((SysVar[SysVarNum]) < SysVarMin[SysVarNum])
            {
                SysVar[SysVarNum] = SysVarMin[SysVarNum];
            }
            SysVarUser[SysVarUserNum] = SysVar[SysVarNum];
            TxCommand[1] =  M_DEBUG_SIX;
            TxCommand[2] =  SysVar[SysVarNum] >> 7;
            TxCommand[3] =  SysVar[SysVarNum] & 0x7f;
        }
        if(RxCommand[2] == 3)
        {
            SysVarNum = RxCommand[3];
            SysVarUserNum = SysVarNum ;
            TxCommand[1] =  M_DEBUG_SIX;
            TxCommand[2] = SysVar[SysVarNum] >> 7;
            TxCommand[3] = SysVar[SysVarNum] & 0x7f;
            SysVarInit();
            SysVarUserInit();
            if(SysVar[98] == 8888)
            {
                SysVar[98] = 888; //2013.02.18
                ResetVarAllFlag = 1;
                SysResetVar = 1;
            }
            else
            {
                SvaeSFlag = 1;
                SvaeSUserFlag = 1;
            }
        }
        TxCommand[4] =  SysVarMin[SysVarNum] >> 7;
        TxCommand[5] =  SysVarMin[SysVarNum] & 0x7f;
        TxCommand[6] =  SysVarMax[SysVarNum] >> 7;
        TxCommand[7] =  SysVarMax[SysVarNum] & 0x7f;
    }
    else if(RxCommand[1] == M_DEBUG_VIE)
    {
        MonitorVar[0] = SoftwareVersion;
        MonitorVar[1] = ImeasA + ImeasAOffset;
        MonitorVar[2] = ImeasB + ImeasBOffset;
        MonitorVar[3] = DcVolt;
        MonitorVar[4] = AdDcVolt;
        MonitorVar[5] = EleCurrent;
        MonitorVar[6] = ImeasAOffset;
        MonitorVar[7] = ImeasBOffset;
        MonitorVar[8] = Hmeas;
        MonitorVar[9] = RotorMacAngle;
        MonitorVar[10] = (AdSpdRef * 29) >> 1;
        MonitorVar[11] = (SpeedFdb * 29) >> 1;
        MonitorVar[12] = Hmeas;
        MonitorVar[13] = SewProcessState;
        MonitorVar[14] = MotorState;
        MonitorVar[15] = svpwm.IDs;
        MonitorVar[16] = svpwm.IQs;
        MonitorVar[17] = _IQtoIQ12(svpwm.UQs);
        MonitorVar[18] = _IQtoIQ12(svpwm.UDs);
        MonitorVar[19] = (SpiRxData & 0xc) >> 2;
        MonitorVar[20] = ProcessVar[90];
        MonitorVar[21] = ProcessVar[91];
        MonitorVar[22] = ProcessVar[92];
        MonitorVar[23] = ProcessVar[93];
        MonitorVar[24] = ProcessVar[94];
        MonitorVar[25] = ProcessVar[95];
        MonitorVar[26] = ProcessVar[96];
        MonitorVar[27] = ProcessVar[97];
        MonitorVar[28] = ProcessVar[98];
        MonitorVar[29] = ProcessVar[99];
        MonitorVar[30] = ProcessVar[100];
        MonitorVar[31] = ProcessVar[101];
        MonitorVar[32] = ProcessVar[102];
        MonitorVar[33] = ProcessVar[103];
        MonitorVar[34] = ProcessVar[104];
        MonitorVar[35] = ProcessVar[105];
        MonitorVar[36] = ProcessVar[106];
        MonitorVar[37] = ProcessVar[107];
        MonitorVar[38] = ProcessVar[108];
        MonitorVar[39] = ProcessVar[109];
        MonitorVar[40] = ProcessVar[110];
        MonitorVar[41] = ProcessVar[111];
        MonitorVar[40] = ProcessVar[112];
        MonitorVar[41] = ProcessVar[113];
        MonitorVar[42] = ProcessVar[114];
        MonitorVar[43] = ProcessVar[115];
        MonitorVar[44] = ProcessVar[116];
        MonitorVar[45] = ProcessVar[117];
        MonitorVar[46] = ProcessVar[118];
        MonitorVar[47] = ProcessVar[119];
        MonitorVar[48] = ProcessVar[120];
        MonitorVar[49] = ProcessVar[121];
        MonitorVar[50] = ProcessVar[122];

        if(RxCommand[2] == 1)
        {
            MonitorDeBugVar = RxCommand[3];
            TxCommand[1] =  M_DEBUG_VIE;
            TxCommand[2] = MonitorVar[MonitorDeBugVar] >> 7;
            TxCommand[3] = MonitorVar[MonitorDeBugVar] & 0x7f;
        }
    }
    else if(RxCommand[1] == M_DEBUG_USERVIE)
    {
        ProcessVar[80] = JianXianJiShu;
        ProcessVar[81] = DangQianDixiZhongShu;
        ProcessVar[85] = MotorRunTime1h;
        ProcessVar[86] = SystemPowerOnTime1h;

        for(SysErrSaveTmp = 0; SysErrSaveTmp < 16; SysErrSaveTmp++)
        {
            ProcessVar[90 + SysErrSaveTmp] = SysErrSave[SysErrSaveTmp];
        }
        for(SysErrSaveTmp = 0; SysErrSaveTmp < 15; SysErrSaveTmp++)
        {
            ProcessVar[106 + SysErrSaveTmp] = SysErrCountSave[SysErrSaveTmp];
        }
        if(RxCommand[2] == 1)
        {
            ProcessDeBugVar = RxCommand[3];
            if(ProcessDeBugVar > 40)
            {
                ProcessDeBugVar = 40;
            }
            TxCommand[1] =  M_DEBUG_USERVIE;
            TxCommand[2] = ProcessVar[ProcessDeBugVar + 80] >> 7;
            TxCommand[3] = ProcessVar[ProcessDeBugVar + 80] & 0x7f;
        }
    }
    else if(RxCommand[1] == M_DEBUG_SEVEN)
    {
    }
    else if(RxCommand[1] == M_MACHINE_TYPE)
    {

    }
    else if(RxCommand[1] == M_SOFTWARE_VERSION)
    {
    }
    else if(RxCommand[1] == M_MACHINE_TEN)
    {
    }
    else if(RxCommand[1] == M_DEBUG_SWM)
    {
    }
    else if(RxCommand[1] == M_DEBUG_SBM)
    {
    }
    else if(RxCommand[1] == M_DEBUG_EBM)
    {
    }
    else if(RxCommand[1] == M_PKEY_SPEED)
    {
        TxCommand[1] =  M_PKEY_SPEED; //发送指令数组
        if(RxCommand[2] == 2)
        {
            TxCommand[2] = 2;
            SysVarUser[3] = (RxCommand[3] * 128) + RxCommand[4];
            if(SysVarUser[3] > 5000)
            {
                SysVarUser[3] = 5000;
            }
            else if(SysVarUser[3] < 150)
            {
                SysVarUser[3] = 150;
            }
            SysVar[3] = SysVarUser[3] ;
            SysVarUserNum = 3;
            SysVarNum = 3;
            SvaeSFlag = 1;
            SvaeSUserFlag = 1;
        }
        else
        {
            TxCommand[2] = 1;
        }
        TxCommand[3] = SysVarUser[3] >> 7;
        TxCommand[4] = SysVarUser[3] & 0x7f;
        TxCommand[5] = 5000 >> 7;
        TxCommand[6] = 5000 & 0x7f;
        SpeedRefMax = ((SysVarUser[3]) * 2) / 29;
        if(SpeedRefMax >= 335)
        {
            SpeedRefMax = 335;
        }
    }
    else if(RxCommand[1] == M_DEBUG_SYSERR)
    {
    }
}


