#include "DSP2803x_Device.h"     // Headerfile Include File
#include "DSP2803x_Examples.h"   // Examples Include File
#include "ExternGlobals.h"
extern int Sewing_AB(int ReinforceNum, int NeedleA, int NeedleB);
extern int Sewing_CD(int ReinforceNum, int NeedleC, int NeedleD);
void FreeModel(void)
{
    switch(SewProcessState)
    {
        case 1 :
        {
            SewingIdle(FreeStartReinforce);
        }
        break;
        case 2:
        {
            if(SewingABOk == 1)
            {
                if((AdSpdRef == 0) && (MotorState == M_MOTOR_IDLE))
                {
                    AdMotorIdle = 1;
                }
                else if((AdSpdRef >= SpeedRefMin) && (AdMotorIdle))
                {
                    SewProcessState = 4;
                    RotorCount = 0;
                    RotorCountTmp = RotorCount;
                    AdMotorIdle = 0;
                }
                else if((!StartReinforcePause) && (AdSpdRef >= SpeedRefMin) && ((RotorCount - RotorCountTmp) >= StartReinforcePauseOffset))
                {
                    MotorState = M_MOTOR_RUNING;
                    SewProcessState = 4;
                    RotorCount = 0;
                    RotorCountTmp = RotorCount;
                    AdMotorIdle = 0;
                    StartReinforceSnub = FreeB;
                    if(StartReinforceSnub > 2)
                    {
                        StartReinforceSnub = 2;
                    }
                }
                else
                {
                    if((RotorCount - RotorCountTmp) >= StartReinforcePauseOffset)
                    {
                        if(MotorState == M_MOTOR_RUNING)
                        {
                            MotorStopFlag = 1;
                            MotorStopHorL = (LedHaltAngle);
                        }
                    }
                }

            }
            else if(Sewing_AB(FreeStartReinforce, FreeA, FreeB) == 0)
            {
                SewingABOk = 1; //返回后不要立及停车，而是再等一两针再等车。
                RotorCount = 0;
                RotorCountTmp = RotorCount;
                if(FreeA > 1)
                {
                    StartReinforcePauseOffset = 2;
                }
                else
                {
                    StartReinforcePauseOffset = 0;
                }
            }
        }
        break;





        case 4 : //ziyou
        {
            TorsionConTrol = 2;
            AdCutFlag = 50;
            if((AdSpdRef >= SpeedRefMin))
            {
                if(MotorState == M_MOTOR_IDLE)
                {
                    MotorState = M_MOTOR_RUNING;
                }
                if(MotorState == M_MOTOR_RUNING)
                {
                    if((SlowStartFalg) && (FreeStartReinforce == 0) && ((RotorCount - RotorCountTmp) <= SlowStartCount))
                    {
                        if(AdSpdRef > SlowStartSpeed)
                        {
                            SpeedRef = SlowStartSpeed;
                        }
                        else
                        {
                            SpeedRef = AdSpdRef;
                        }
                    }
                    else if((RotorCount - RotorCountTmp) < StartReinforceSnub)
                    {
                        if((AdSpdRef < StartReinforceSpeed) && (AdSpdRef >= SpeedRefMin))
                        {
                            SpeedRef = AdSpdRef;
                        }
                        else
                        {
                            SpeedRef = StartReinforceSpeed;
                        }

                    }
                    else if((RotorCount - RotorCountTmp) < 1)
                    {
                        if(FirstNeedleSpeed > AdSpdRef)
                        {
                            SpeedRef = AdSpdRef;
                        }
                        else
                        {
                            SpeedRef = FirstNeedleSpeed;
                        }
                    }
                    else
                    {
                        SpeedRef = AdSpdRef;
                    }
                }
            }
            else//剪线还是停针
            {
                MotorStopFlag = 1;
                SpeedRef = 0;
                if((AdSpdRef == 2) && (LedCut) && (FreeEndReinforce == 0))
                {
                    if((MotorState == M_MOTOR_RUNING) || (MotorState == M_MOTOR_ARREST))
                    {
                        DaoFengFlag = 0;
                        StopingCutFalg = 2;
                        SewProcessState = 7;
                        MotorStopHorL = 0;
                    }
                }
                else
                {
                    if(MotorState == M_MOTOR_RUNING)
                    {
                        MotorStopHorL = (LedHaltAngle);

                    }
                }
            }
            if((AdSpdRef == 2) && (MotorState == M_MOTOR_IDLE))
            {
                if(TaiYaJiaoFlag == 0)
                {
                    MotorGetZeroAgain = 0;
                    SewProcessState = 5;
                    RotorCount = 0;
                    RotorCountTmp = RotorCount;
                }
            }
            if((AdSpdRef == 3) && (MotorState == M_MOTOR_IDLE))
            {
                TaiYaJiaoStart = 1;
            }
            else
            {
                TaiYaJiaoStart = 0;
            }
        }
        break;

        case 5 :
        {
            if(FreeEndReinforce == 0)
            {
                SewProcessState = 7;
                DaoFengFlag = 0;
                if(LedCut)
                {
                    if(MotorState == M_MOTOR_RUNING)
                    {
                        MotorStopFlag = 1;
                        StopingCutFalg = 2;
                    }
                    else if(MotorState == M_MOTOR_IDLE)
                    {
                        MotorState = M_MOTOR_PROCUTING;
                        StopingCutFalg = 1;
                    }
                    SpeedRef = SpdCuting;
                }
                else  if(MotorState == M_MOTOR_RUNING)
                {
                    MotorStopFlag = 1;
                    SpeedRef = 0;
                }
            }
            else if(Sewing_CD(FreeEndReinforce, FreeC, FreeD) == 0)
            {
                SewProcessState = 7;
                if(LedCut)
                {
                    DaoFengFlag = 0;
                    StopingCutFalg = 1;
                    SpeedRef = SpdCuting;
                }
                else if(MotorState == M_MOTOR_RUNING)
                {
                    SpeedRef = 0;
                    MotorStopFlag = 1;
                }
            }
        }
        break;

        case 7:
        {
            if(MotorState == M_MOTOR_IDLE)
            {
                SewProcessState = 8;
            }
        }
        break;
        case 500 :
        {
            ExigenceCutDone(7);
        }
        break;

        case 8 :
        {
            AdSpdDelay();
        }
        break;
    }//end switch

    ExigenceCut(7);
}//end all

