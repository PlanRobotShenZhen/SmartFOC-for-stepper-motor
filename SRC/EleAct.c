#include "DSP2803x_Device.h"     // Headerfile Include File
#include "DSP2803x_Examples.h"   // Examples Include File
#include "ExternGlobals.h"
#include "DeviceConfig.h"
void EleAct2(void);
int SaoxianTimer = 0;
int SaoxianState = 0;
int SouDaoBuZhenFlag = 0;
int SouDaoBuZhenCount = 0;
int SouDongDaoFengSwitchOFFCount = 0;
int SouDaoBuZhenHlaf = 0;
int SouDongDaoFengNeedleAddFalg = 0;
int SouDongDaoFengRuning = 0;
int PWMTimer = 0;
int PWMTimerMax = 100;
int DelayEleS = 0;
int DelayEleE = 0;
int AnQuanSwitchDelay = 0;
int SaoxianFlagGetZero = 0;
int JiaXianTimerP = 0;
void EleAct2(void)
{
    EPwm5Regs.CMPB = LedLiangDu;

    if(SewModel == 6)
    {
        DelayEleS = DelayEleSW;
        DelayEleE = DelayEleEW;
    }
    else if(SewProcessState <= 4)
    {
        DelayEleS = DelayEleS1 - DelayEleS1Offset;
        DelayEleE = DelayEleS2 - DelayEleS2Offset;
    }
    else
    {
        DelayEleS = DelayEleE1 - DelayEleE1Offset;
        DelayEleE = DelayEleE2 - DelayEleE2Offset;
    }
    if((SouDongDaoFengSwitch_OFF) && (SouDongDaoFengRuning))
    {
        SouDongDaoFengRuning = 0;
        DAOFENG_OFF;
        EPwm6Regs.CMPA.half.CMPA = 5000;
        DaoFengTmr = 0;
        DFEEndDelay = 0;
        DaoFengELEState = 0;
    }

    if(SouDongDaoFengSwitch_ON)
    {
        if((SewModel == 1) && (MotorState == M_MOTOR_IDLE) && (MotorRunFlag == 0) && (DakFengKey))
        {
            SouDaoBuZhenFlag = 1;
        }
        SouDongDaoFengSwitchOFFCount = 0;
    }
    else if((AnQuanSwitch_ON) && (AnQuanSwitchModel == 1))
    {
        if((SewModel == 1) && (MotorState == M_MOTOR_IDLE) && (MotorRunFlag == 0) && (DakFengKey))
        {
            SouDaoBuZhenFlag = 1;
        }
        SouDongDaoFengSwitchOFFCount = 0;
    }
    else
    {
        if(SouDaoBuZhenFlag)
        {
            SouDongDaoFengSwitchOFFCount++;
        }
    }
    /**********************手动补针****************************/
    if(SouDaoBuZhenFlag)
    {
        SouDaoBuZhenCount++;
        if(SouDaoBuZhenCount > 32000)
        {
            SouDaoBuZhenCount = 32000;
        }
        if(SouDaoBuZhenCount > 200)
        {
            SouDaoBuZhenHlaf = 0;
            if(MotorState == M_MOTOR_IDLE)
            {
                MotorStopHorL = !MotorStopHorL;
                MotorState = M_MOTOR_RUNING;
                SpeedRef = NeedelAddSpeedRef;
                SouDongDaoFengNeedleAddFalg = 1;
            }
        }
        else if(SouDaoBuZhenCount > 10)
        {
            SouDaoBuZhenHlaf = 1;
        }
        if(SouDongDaoFengSwitchOFFCount >= 50)
        {
            SpeedRef = 0;
            if(SouDaoBuZhenHlaf == 1)
            {
                MotorStopHorL = !MotorStopHorL;
                MotorState = M_MOTOR_ARREST;
                SouDaoBuZhenHlaf = 0;
            }
            if(MotorState == M_MOTOR_RUNING)
            {
                MotorStopHorL = !MotorStopHorL;
                MotorState = M_MOTOR_ARREST;
            }
            if(MotorState == M_MOTOR_IDLE)
            {
                SouDaoBuZhenFlag = 0;
                SouDongDaoFengSwitchOFFCount = 0;
                SouDongDaoFengNeedleAddFalg = 0;
            }
            SouDaoBuZhenCount = 0;
        }
    }//END 手动补针

    /**********************手动补针****************************/


    /********************开始倒缝电磁铁***************************/
    if(DaoFengELEState == 0)
    {
        if((SouDongDaoFengSwitch_ON) && (SewModel == 1) && (SouDaoBuZhenFlag == 0) && (DakFengKey))
        {
            if(MotorRunFlag)
            {
                DaoFengELEState = 1;
                SouDongDaoFengRuning = 1;
            }
        }
        else if((DakFengKey == 0) && (SouDongDaoFengSwitch_ON) && (TaiYaJiaoELEState == 0)) // 抬压脚的时候不允许倒缝
        {
            DaoFengELEState = 1;
            SouDongDaoFengRuning = 1;
        }
        DAOFENG_OFF;//倒缝电磁铁要关掉。
    }

    if(DaoFengELEState == 1)
    {
        DFEStartDelay++;
        if(DFEStartDelay >= (DelayEleS))
        {
            DaoFengELEState = 2;
            DFEStartDelay = 0;
        }
    }
    else if(DaoFengELEState == 2)
    {
        DaoFengTmr++;
        if(DaoFengTmr >= DaoFengEleFullVolt)
        {
            DaoFengTmr = DaoFengEleFullVolt;
            DaoFengELEState = 3;
            DFEStartDelay = 0; //2013.05.11
        }
        //ELEG4_ON;
        DAOFENG_ON;
        EPwm6Regs.CMPA.half.CMPA = 0;
    }
    else if(DaoFengELEState == 3)
    {
        EPwm6Regs.CMPA.half.CMPA = (1000 - (DaoFengEleRun * 10) - 300) * 3; //2013.02.15
        //ELEG4_ON;
        DAOFENG_ON;
    }
    else if(DaoFengELEState == 4)
    {
        DFEEndDelay++;
        if(DFEEndDelay >= (DelayEleE))
        {
            DaoFengELEState = 5;
        }
    }
    else if(DaoFengELEState == 5)
    {
        DAOFENG_OFF;
        EPwm6Regs.CMPA.half.CMPA = 5000;
        DaoFengTmr = 0;
        DFEEndDelay = 0;
        DaoFengELEState = 0;
        DFEStartDelay = 0;;
    }
    /********************结束倒缝电磁铁***************************/



    /********************开始抬压脚电磁铁*************************/
    if(TaiYaJiaoELEState == 0)
    {
        TAIYAJIAO_OFF;
        if(TaiYaJiaoStart)
        {
            TaiYaJiaoFlag = 1;
        }
        else
        {
            TaiYaJiaoFlag = 0;
        }
        if((TaiYaJiaoFlag) && (DaoFengELEState == 0)) //抬压脚，倒缝纫互锁
        {
            if((TaiYaJiaoEleOn) && (MotorState == M_MOTOR_IDLE)) //P61电机停下来后启动延时
            {
                TaiYaJiaoELEState = 1;
            }
        }
    }
    else if(TaiYaJiaoELEState == 1)
    {
        TYJStartDelay++;
        if(TYJStartDelay >= TaiYaJiaoDelayTimeS) //延时10ms 起动
        {
            TaiYaJiaoELEState = 2;
            TYJStartDelay = 0;
        }
    }
    else if(TaiYaJiaoELEState == 2) //电磁铁全电压起动  P57
    {
        TaiYaJiaoTmr++;
        if(TaiYaJiaoTmr >= TaiYaJiaoStartTime)
        {
            TaiYaJiaoTmr = 0;
            TaiYaJiaoELEState = 3;
            TaiYaJiaoProtectCount = 0;
            PWMTimer = 0;
        }
        TAIYAJIAO_ON;
        EPwm6Regs.CMPA.half.CMPA = 0;
    }
    else if(TaiYaJiaoELEState == 3)
    {
        EPwm6Regs.CMPA.half.CMPA = (1000 - (TaiYaJiaoRunTime * 10) - 300) * 3; //P58  //PWM 运行
        TAIYAJIAO_ON;
        PWMTimer++;
        if(TaiYaJiaoProtectCount >= TaiYaJiaoProtectTime) //抬压脚动作保护 P60
        {
            TaiYaJiaoELEState = 40;
            FangYaJiaoTimer = 0;
        }
        if((!TaiYaJiaoStart) && (PWMTimer > PWMTimerMax))
        {
            TaiYaJiaoELEState = 40;
            FangYaJiaoTimer = 0;
        }
    }
    else if(TaiYaJiaoELEState == 40) //开始放压脚
    {
        TAIYAJIAO_OFF;
        EPwm6Regs.CMPA.half.CMPA = 5000;
        FangYaJiaoTimer++;
        if(FangYaJiaoTimer >= FangYaJiaoTimerMax)
        {
            TaiYaJiaoELEState = 41;
            ErChiTaiYaJiaoTimer = 0;
        }
    }
    else if(TaiYaJiaoELEState == 41)
    {
        TAIYAJIAO_ON;
        EPwm6Regs.CMPA.half.CMPA = 0;
        ErChiTaiYaJiaoTimer++;
        if(ErChiTaiYaJiaoTimer >= ErChiTaiYaJiaoTimerMax)
        {
            TaiYaJiaoELEState = 4;
        }
    }
    else if(TaiYaJiaoELEState == 4)          //           延时10ms 放压脚 P62
    {
        TAIYAJIAO_OFF;
        EPwm6Regs.CMPA.half.CMPA = 5000;
        TYJEndDelay++;
        if(TYJEndDelay >= TaiYaJiaoDelayTimeE)
        {
            TaiYaJiaoELEState = 5;
            TaiYaJiaoDelayTimeETimer = 0;
        }
    }
    else if(TaiYaJiaoELEState == 5) //压脚放下来后多长时间才能启动电机
    {
        TaiYaJiaoTmr = 0;
        TYJEndDelay = 0;
        if(!TaiYaJiaoStart)
        {
            TaiYaJiaoELEState = 0;
            TaiYaJiaoFlag = 0;
        }
    }
    /********************开始抬压脚电磁铁*************************/

    /********************开始剪线电磁铁***************************/
    if(MotorState == M_MOTOR_CUTING)
    {
        if(RotorMacAngle < (CutStartMinAngle))
        {
            CutFlag = 1;
        }
    }
    if(CutFlag)
    {
        CutTmr++;
        if(MotorState == M_MOTOR_HALT)
        {
            CutFlag = 0;
        }
    }
    else
    {
        CutTmr = 0;
    }

    if((CutTmr > 1) && (CutTmr < (OutCutTime * 5)))
    {
        //JIANXIAN_ON;
        JIANXIAN_ON;
        SongXianFlag = 1;
        asm(" RPT #10 || NOP");
        SONGXINA_ON;
    }
    else if(CutTmr > (OutCutTime * 5))
    {
        CutTmr = (OutCutTime * 5) + 1;
        JIANXIAN_0FF;
        SysErr = M_SYSERR_CLECUT;
    }
    else
    {
        JIANXIAN_0FF;
        /*
        if(MotorState==M_MOTOR_CUTING)
        {
            if(SaoxianFlagGetZero==1)
            {
               SaoxianFlag=1;
               SaoxianFlagGetZero=0;
            }
        }
        else if(MotorState==M_MOTOR_IDLE)
        {
            SaoxianFlagGetZero=1;
        }
        */
        /*******扫线电磁铁与松线器之间的选择*********/
        if(SongXianFlag)
        {
            SongXianDelay++;
            if(SongXianDelay > SongXianMax)
            {
                SongXianFlag = 0;
                SongXianDelay = 0;
                SONGXINA_OFF;
            }
        }
    }
    /********************结束剪线电磁铁***************************/


    /********************开始扫线电磁铁***************************/
    if((SaoxianFlag) && (SaoxianState == 0))
    {
        if((RotorMacAngle < HaltAngle + SaoxianAngleOffset)) //防此卡刀时扫线，造成大电流
        {
            SaoxianState = 1;
        }
    }
    else if(SaoxianState == 1)
    {
        SaoxianTimer++;
        if(SaoxianTimer >= ((OutShaoXianTime >> 1) + 15))
        {
            SAOXIAN_OFF;
            SaoxianState = 2;
        }
        else if(SaoxianTimer >= 15) //延迟15ms
        {
            SAOXIAN_ON;
        }
    }
    else if(SaoxianState == 2)
    {
        SaoxianTimer++;
        if(SaoxianTimer >= 300)
        {
            SaoxianTimer = 0;
            SaoxianState = 0;
            SaoxianFlag = 0;
        }
    }
    /********************结束扫线电磁铁***************************/


    /********************开始夹线电磁铁***************************/
    if(JIAXIANQI_RUN)
    {
        JiaXianTimerP++;
        if(JiaXianTimerP > 350)
        {
            JiaXianTimerP = 350;
            SysErr = M_SYSERR_JIAXIANQI;
            JIAXIANQI_OFF;
            ELOCK_ON;
        }
    }
    else
    {
        JiaXianTimerP = 0;
    }

    /********************结束夹线电磁铁***************************/

    /********************开始安全开关功能选择********************/
    if(AnQuanSwitch_ON)
    {
        if(AnQuanSwitchModel == 3) //安全开关
        {
            AnQuanSwitchDelay++;
            if(AnQuanSwitchDelay > 10)
            {
                AnQuanSwitchDelay = 0;
                SysErr = M_SYSERR_SAFSWITCH;
            }
        }
    }
    else
    {
        AnQuanSwitchDelay = 0;
    }
    /********************结束开关功能选择**********************/
}
