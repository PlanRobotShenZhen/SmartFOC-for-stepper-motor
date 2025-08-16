#include "DSP2803x_Device.h"     // Headerfile Include File
#include "DSP2803x_Examples.h"   // Examples Include File
#include "ExternGlobals.h"
int Sewing_AB(int ReinforceNum, int NeedleA, int NeedleB);
int Sewing_CD(int ReinforceNum, int NeedleC, int NeedleD);
int Fixed1(int FixedLen, int FixedTirg, int RetouchLen, int LedFalg);
int Fixed(int FixedLen, int FixedTirg, int NeedleC, int NeedleD);
void SewingWait(int NextSatae);
void AdSpdDelay(void);
void AdSpdDelay(void);
void SewingIdle(int StartReinforce);
void ExigenceCut(int OutState);
void BeforeEndReinforce(int EndReinforce, int OutState);
void ExigenceCutDone(int OutState);
int SpeedRefTmp = 0;
int FinishTaiYaJiaoLock = 0;
int DaoFengFlagDelay = 0;
int Sewing_CDHighSpeedInFlag = 0;
int SewingABState = 0;
int AlreadySewingAB = 0;
int SewingCDState = 0;
int WaitEleCount = 0;
int FirstFinishFlag = 0;
int FixedLen2 = 0;
int EndReinforceSpeedChange = 0;
int LedHaltAngleTemp = 0;
int Sewing_AB(int ReinforceNum, int NeedleA, int NeedleB)
{

    int ReturnData = 0;
    AlreadySewingAB = 1;

    TorsionConTrol = 1;
    if(SewingABState == 0)
    {
        SewingABState = 3;
        FirstFinishFlag = 0;
        ReturnData = 1;
    }
    else if(SewingABState == 3)
    {
        ReturnData = 1;
        RotorCount = 0;
        if(ReinforceNum == 1)
        {
            if(!ELERung)
            {
                DaoFengELEState = 2;
            }
            ELERung = 1;
            DaoFengFlag = 1;
            DaoFengFlagDelay++;
            if(DaoFengFlagDelay >= 20)
            {
                MotorState = M_MOTOR_RUNING;
                if(NeedleB < 2)
                {
                    SewingABState = 22;
                    SpeedRef = StartReinforceSpeedOneToOne;
                    DelayEleE1Offset = DelayEleE1;
                    DelayEleE2Offset = DelayEleE2;
                    DelayEleS1Offset = DelayEleS1;
                    DelayEleS2Offset = DelayEleS2;
                    RotorCountTmp = RotorCount;
                }
                else
                {
                    SewingABState = 2;
                    SpeedRef = StartReinforceSpeed;
                    DelayEleE1Offset = 0;
                    DelayEleE2Offset = 0;
                    DelayEleS1Offset = 0;
                    DelayEleS2Offset = 0;
                    RotorCountTmp = RotorCount - 2;
                }
            }
        }
        else
        {
            MotorState = M_MOTOR_RUNING;
            if((NeedleA < 2) || (NeedleB < 2))
            {
                SewingABState = 11;
                SpeedRef = StartReinforceSpeedOneToOne;
                DelayEleE1Offset = DelayEleE1;
                DelayEleE2Offset = DelayEleE2;
                DelayEleS1Offset = DelayEleS1;
                DelayEleS2Offset = DelayEleS2;
                RotorCountTmp = RotorCount;
            }
            else
            {
                SewingABState = 1;
                SpeedRef = StartReinforceSpeed;
                DelayEleE1Offset = 0;
                DelayEleE2Offset = 0;
                DelayEleS1Offset = 0;
                DelayEleS2Offset = 0;
                RotorCountTmp = RotorCount - 2;
            }
        }
    }
    else if(SewingABState == 11)
    {
        ReturnData = 1;
        if(((RotorCount - RotorCountTmp)) >= NeedleA)
        {
            DaoFengFlag = 1;
            RotorCount = 0;
            RotorCountTmp = RotorCount;
            SewingABState = 22;

        }
    }
    else if(SewingABState == 22)
    {
        ReturnData = 1;
        if(((RotorCount - RotorCountTmp)) >= NeedleB)
        {
            DaoFengFlag = 0;
            RotorCount = 0;
            RotorCountTmp = RotorCount;
            if((ReinforceNum == 3) && (FirstFinishFlag == 0))
            {
                FirstFinishFlag = 1;
                SewingABState = 11;
                ReturnData = 1;
            }
            else
            {
                SewingABState = 0;
                ReturnData = 0;
                FirstFinishFlag = 0;
            }
        }
    }
    else  if(SewingABState == 1)
    {
        DaoFengFlagDelay = 0;
        if(((RotorCount - RotorCountTmp)) >= NeedleA)
        {
            if((NeedleA < 3) && (FirstFinishFlag == 0))
            {
                DelayEleS1Offset = DelayEleS1OffsetInit;
            }
            else
            {
                DelayEleS1Offset = 0;
            }
            DaoFengFlag = 1;
            RotorCount = 0;
            RotorCountTmp = RotorCount;
            SewingABState = 2;
            ReturnData = 1;
        }
        else
        {
            DaoFengFlag = 0;
            ReturnData = 1;
        }
    }
    else if(SewingABState == 2)
    {
        if(((RotorCount - RotorCountTmp)) >= NeedleB)
        {
            DaoFengFlag = 0;
            if((ReinforceNum == 3) && (FirstFinishFlag == 0))
            {
                FirstFinishFlag = 1;
                RotorCount = 0;
                RotorCountTmp = RotorCount;
                SewingABState = 1;
                ReturnData = 1;
            }
            else
            {
                FirstFinishFlag = 0;
                DaoFengFlag = 0;
                ReturnData = 0;
                SewingABState = 0;
            }
        }
        else
        {
            DaoFengFlag = 1;
            ReturnData = 1;
        }
    } //end if(Sewing_AB_state==2)
    return ReturnData;
}

/****************Sewing_CD***********************************/

int Sewing_CD(int ReinforceNum, int NeedleC, int NeedleD)
{
    int ReturnData = 0;

    TorsionConTrol = 1;
    if(SewingCDState == 0)
    {
        ReturnData = 1;
        FirstFinishFlag = 0;
        WaitEleCount++;
        if(MotorState == M_MOTOR_IDLE)
        {
            if((RotorMacAngle < 2635) && (RotorMacAngle > 1200)) //¸Õ¹ý¼ÆÕëÏß  //Òª×¢ÒâÏÂÍ£³µ
            {
                //MotorState=M_MOTOR_ARREST;
                MotorState = M_MOTOR_INIT;

                //LedHaltAngleTemp=1;

            }
            else
            {
                if(!ELERung)
                {
                    DaoFengELEState = 2;
                    WaitEleCount = 0;
                    ELERung = 1;
                }

                DaoFengFlag = 1;
            }
        }
        else if(MotorState == M_MOTOR_RUNING)
        {
            DelayEleE1Offset = 0;
            DelayEleE2Offset = 0;
            DelayEleS1Offset = 0;
            DelayEleS2Offset = 0;
            Sewing_CDHighSpeedInFlag = 1;
            /********************¼ÆÕò¿ÕÇø***********************/
            if(DaoFengFlag == 0)
            {
                DaoFengFlag = 1;
                if((NeedleC < 2) || (NeedleD < 2))
                {
                    if(ReinforceNum == 1)
                    {
                        SewingCDState = 3;
                    }
                    else
                    {
                        SewingCDState = 11;
                    }
                    SpeedRef = EndReinforceSpeedOneToOne;
                    DelayEleE1Offset = DelayEleE1;
                    DelayEleE2Offset = DelayEleE2;
                    DelayEleS1Offset = DelayEleS1;
                    DelayEleS2Offset = DelayEleS2;

                }
                else
                {
                    if(ReinforceNum == 1)
                    {
                        SewingCDState = 5;
                    }
                    else
                    {
                        SewingCDState = 1;
                    }
                    DelayEleE1Offset = 0;
                    DelayEleE2Offset = 0;
                    DelayEleS1Offset = 0;
                    DelayEleS2Offset = 0;
                    SpeedRef = EndReinforceSpeed;
                }
                RotorCount = 0;
                RotorCountTmp = RotorCount;

            }

            if((NeedleC < 2) || (NeedleD < 2))
            {
                SpeedRef = EndReinforceSpeedOneToOne;

            }
            else
            {
                SpeedRef = EndReinforceSpeed;

            }

        }
        if((WaitEleCount >= (WaitEleCountMax)) && (MotorState == M_MOTOR_IDLE))
        {
            Sewing_CDHighSpeedInFlag = 0;
            MotorState = M_MOTOR_RUNING;
            RotorCount = 0;
            ELERung = 1;
            DaoFengFlag = 1;
            WaitEleCount = 0;

            if((RotorMacAngle < 1200) || (RotorMacAngle > 3800)) //¸Õ¹ý¼ÆÕëÏß  //Òª×¢ÒâÏÂÍ£³µ
            {
                LedHaltAngleTemp = 0;
            }
            else
            {
                LedHaltAngleTemp = 1;

            }




            if((NeedleC < 2) || (NeedleD < 2))
            {
                SpeedRef = EndReinforceSpeedOneToOne;
                if(ReinforceNum == 1)
                {
                    SewingCDState = 3;
                }
                else
                {
                    SewingCDState = 11;
                }

                DelayEleE1Offset = DelayEleE1;
                DelayEleE2Offset = DelayEleE2;
                DelayEleS1Offset = DelayEleS1;
                DelayEleS2Offset = DelayEleS2;
                RotorCountTmp = RotorCount;
            }
            else
            {
                SpeedRef = EndReinforceSpeed;
                if(ReinforceNum == 1)
                {
                    SewingCDState = 3;
                    RotorCountTmp = RotorCount;
                }
                else
                {
                    SewingCDState = 1;
                    RotorCountTmp = RotorCount - 2;
                }
                DelayEleE1Offset = 0;
                DelayEleE2Offset = 0;
                DelayEleS1Offset = 0;
                DelayEleS2Offset = 0;
            }
        }
    } // end SewingCDState==0

    else if(SewingCDState == 11)
    {
        ReturnData = 1;
        if(((RotorCount - RotorCountTmp)) >= NeedleC)
        {
            DaoFengFlag = 0;
            SewingCDState = 22;
            RotorCount = 0;
            RotorCountTmp = RotorCount;
            if(ReinforceNum == 1)
            {
                ReturnData = 0;
                SewingCDState = 0;
                MotorStopFlag = 1;
                MotorStopHorL = 0;
            }
        }
    }
    else if(SewingCDState == 22)
    {
        ReturnData = 1;
        if(((RotorCount - RotorCountTmp)) >= NeedleD)
        {
            if((ReinforceNum == 3) && (FirstFinishFlag == 0))
            {
                FirstFinishFlag = 1;
                DaoFengFlag = 1;
                RotorCount = 0;
                RotorCountTmp = RotorCount;
                SewingCDState = 11;
            }
            else
            {
                SewingCDState = 0;
                ReturnData = 0;
                MotorStopFlag = 1;
                MotorStopHorL = 0;
                FirstFinishFlag = 0;
            }
        }
    }

    else if(SewingCDState == 1)
    {
        ReturnData = 1;
        DaoFengFlag = 1;
        if(((RotorCount - RotorCountTmp)) >= NeedleC)
        {
            DaoFengFlag = 0;
            SewingCDState = 2;
            RotorCount = 0;

            //if((RotorMacAngle<2635)&&(RotorMacAngle>1200))

            if((LedHaltAngleTemp) && (Sewing_CDHighSpeedInFlag == 0))
                //if((RotorMacAngle>2635)&&(RotorMacAngle<DaoFengStartAngle)&&(Sewing_CDHighSpeedInFlag==0))
            {
                if((NeedleC < 3) && (FirstFinishFlag == 0))
                {
                    DelayEleE2Offset = 30;
                }
                else
                {
                    DelayEleE2Offset = 0;
                }
            }
            if((ReinforceNum == 2) || FirstFinishFlag)
            {
                RotorCountTmp = RotorCount + 2;
            }
            else
            {
                RotorCountTmp = RotorCount;
            }
        }
    }
    else if(SewingCDState == 2)
    {
        ReturnData = 1;
        DaoFengFlag = 0;
        if(((RotorCount - RotorCountTmp)) >= NeedleD)
        {
            if((ReinforceNum == 3) && (FirstFinishFlag == 0))
            {
                FirstFinishFlag = 1;
                DaoFengFlag = 1;
                RotorCount = 0;
                RotorCountTmp = RotorCount;
                SewingCDState = 1;
            }
            else
            {
                SewingCDState = 0;
                ReturnData = 0;
                MotorStopFlag = 1;
                MotorStopHorL = 0;
                FirstFinishFlag = 0;
            }
        }
    }
    else if(SewingCDState == 3)
    {
        if(((RotorCount - RotorCountTmp)) >= NeedleC)
        {
            ReturnData = 0;
            SewingCDState = 0;
            MotorStopFlag = 1;
            MotorStopHorL = 0;
        }
        else
        {
            ReturnData = 1;
        }
    }
    else if(SewingCDState == 5)
    {
        if(((RotorCount - RotorCountTmp)) >= (NeedleC + 2))
        {
            ReturnData = 0;
            SewingCDState = 0;
            MotorStopFlag = 1;
            MotorStopHorL = 0;
        }
        else
        {
            ReturnData = 1;
        }
    }
    return ReturnData;
}


/******************************************************/


/*************************************************/
int Fixed(int FixedLen, int FixedTirg, int NeedleC, int NeedleD)
{
    int ReturnData = 0;
    TorsionConTrol = 2;

    if((NeedleC < 2) || (NeedleD < 2))
    {
        EndReinforceSpeedChange = EndReinforceSpeedOneToOne;
        FixedLen2 = FixedLen;
    }
    else
    {
        EndReinforceSpeedChange = EndReinforceSpeed;
        FixedLen2 = FixedLen - 2;
    }

    if(((RotorCount - RotorCountTmp)) >= (FixedLen2))
    {
        FixedTirgStartFalg = 0;
        ReturnData = 0;
    }
    else
    {
        /****************************************´¥·¢²Ù×÷**********************************************/
        if(FixedTirg)
        {
            /*****************ÈíÆð¶¯***********/
            if((SlowStartFalg) && ((RotorCount - RotorCountTmp) <= SlowStartCount) && (!AlreadySewingAB))
            {
                TorsionConTrol = 2;
                if(AdSpdRef > SlowStartSpeed)
                {
                    SpeedRef = SlowStartSpeed;
                }
                else
                {
                    SpeedRef = AdSpdRef;
                }
            }
            /************////Ç°¹Ìºó·ÀÖ¹ÃÍ¼ÓËÙ***********/
            else if(((RotorCount - RotorCountTmp)) < StartReinforceSnub)
            {
                SpeedRef = StartReinforceSpeed;
                TorsionConTrol = 2; //2013.04.10
            }
            /************µÚÒ»ÕëËÙÂÊ*************/
            else if(((RotorCount - RotorCountTmp) < 1) && (FixedLen >= 4))
            {

                TorsionConTrol = 2;
                if(FirstNeedleSpeed > AdSpdRef)
                {
                    SpeedRef = AdSpdRef;
                }
                else
                {
                    SpeedRef = FirstNeedleSpeed;
                }
            }
            /*************Õý³£ÔËÐÐ****************/
            else
            {
                TorsionConTrol = 2; //2013.04.10
                SpeedRef = FixedSpeed;
            }

            if(FixedLen < 4) //×î¸ßËÙÏÞ¶¨¡£
            {
                if(FixedSpeed > EndReinforceSpeed)
                {
                    SpeedRef = EndReinforceSpeed;
                }
                else
                {
                    SpeedRef = FixedSpeed;
                }

            }
            if((NeedleC < 2) || (NeedleD < 2))
            {
                /******************½øÈë5Õë¼õËÙÇø***********************/
                if(((RotorCount - RotorCountTmp)) >= (FixedLen - 5))
                {
                    TorsionConTrol = 4;
                    SpeedRefTmp = FixedSpeed;
                    if(SpeedRefTmp > ((FixedLen - (RotorCount - RotorCountTmp)) * 45))
                    {
                        SpeedRefTmp = ((FixedLen - (RotorCount - RotorCountTmp)) * 45);
                    }
                    SpeedRef = SpeedRefTmp;
                }
            }
            else
            {
                if((RotorCount - RotorCountTmp) >= (FixedLen - SubAddBuffer))
                {
                    TorsionConTrol = 3;
                    if(FixedSpeed > EndReinforceSpeedChange)
                    {
                        SpeedRef = EndReinforceSpeedChange;
                    }
                    else
                    {
                        SpeedRef = FixedSpeed;
                    }
                }
            }
            if((AdSpdRef > (SpeedRefMin)) && (FixedTirgStartFalg == 0))
            {
                FixedTirgStartFalg = 1;

                MotorState = M_MOTOR_RUNING;
            }
            else
            {
                if((MotorState == M_MOTOR_RUNING) && (FixedTirgStartFalg == 0))
                {
                    MotorStopFlag = 1;
                    MotorStopHorL = (LedHaltAngle);
                }
            }
        }  // END if(FixedTirg)

        /*************************************ÊÖ¶¯²Ù×÷*********************************************/
        else
        {
            if((AdSpdRef >= SpeedRefMin))
            {

                if(MotorState == M_MOTOR_IDLE)
                {
                    MotorState = M_MOTOR_RUNING;
                }

                else if(MotorState == M_MOTOR_RUNING)
                {
                    /*****************ÈíÆð¶¯***********/
                    if((SlowStartFalg) && ((RotorCount - RotorCountTmp) <= SlowStartCount) && (!AlreadySewingAB))
                    {
                        TorsionConTrol = 2;
                        if(AdSpdRef > SlowStartSpeed)
                        {
                            SpeedRef = SlowStartSpeed;
                        }
                        else
                        {
                            SpeedRef = AdSpdRef;
                        }
                    }
                    /*****************Ç°¹Ìºó·ÀÖ¹ÃÍ¼ÓËÙ***********/
                    else if((RotorCount - RotorCountTmp) < StartReinforceSnub)
                    {
                        SpeedRef = StartReinforceSpeed;
                        TorsionConTrol = 2;
                    }
                    /******************µÚÒ»ÕëËÙÂÊ*****************/
                    else if(((RotorCount - RotorCountTmp) < 1) && (FixedLen >= 4))
                    {

                        TorsionConTrol = 2;
                        if(FirstNeedleSpeed > AdSpdRef)
                        {
                            SpeedRef = AdSpdRef;
                        }
                        else
                        {
                            SpeedRef = FirstNeedleSpeed;
                        }
                    }
                    /****************Õý³£ÔËÐÐ********************/
                    else
                    {
                        TorsionConTrol = 2;
                        SpeedRef = AdSpdRef;
                    }
                    if(FixedLen < 4) //×î¸ßËÙÏÞ¶¨¡£
                    {
                        if(AdSpdRef > EndReinforceSpeed)
                        {
                            SpeedRef = EndReinforceSpeed;
                        }
                        else
                        {
                            SpeedRef = AdSpdRef;
                        }

                    }
                    if((NeedleC < 2) || (NeedleD < 2))
                    {
                        /******************½øÈë5Õë¼õËÙÇø***********************/
                        if(((RotorCount - RotorCountTmp)) >= (FixedLen - 5))
                        {
                            TorsionConTrol = 4;
                            SpeedRefTmp = AdSpdRef;
                            if(SpeedRefTmp > ((FixedLen - (RotorCount - RotorCountTmp)) * 45))
                            {
                                SpeedRefTmp = ((FixedLen - (RotorCount - RotorCountTmp)) * 45);
                            }
                            SpeedRef = SpeedRefTmp;
                        }
                    }
                    else
                    {

                        if((RotorCount - RotorCountTmp) >= (FixedLen - SubAddBuffer))
                        {
                            TorsionConTrol = 3;
                            if(AdSpdRef > EndReinforceSpeedChange)
                            {
                                SpeedRef = EndReinforceSpeedChange;
                            }
                            else
                            {
                                SpeedRef = AdSpdRef;
                            }
                        }
                    }
                }
                /******************/
            }

            /*****************Í£Ö¹********************************************************************/
            else
            {

                if(MotorState == M_MOTOR_RUNING)
                {
                    MotorStopFlag = 1;
                    MotorStopHorL = (LedHaltAngle);
                }
            }
            /*****************Ì§Ñ¹½Å***********************************************/
            if((AdSpdRef == 3) && (MotorState == M_MOTOR_IDLE))
            {
                TaiYaJiaoStart = 1;
            }
            else
            {
                TaiYaJiaoStart = 0;
            }
        }//Íê³ÉÊÖ¶¯²Ù×÷
        ReturnData = 1;
    }
    return ReturnData;
}




int Fixed1(int FixedLen, int FixedTirg, int RetouchLen, int LedFlag) //ÎÞºó¹Ì·ìÈÒ
{
    int ReturnData = 0;
    int FixedLenTmp = 0;
    TorsionConTrol = 2;
    if((LedHaltAngle) && (!LedFlag)) //ÏÂÍ£Õë¼ôÏßµã²»Í¬
    {
        FixedLenTmp = FixedLen - 1;
    }
    else
    {
        FixedLenTmp = FixedLen;
    }

    if(((RotorCount - RotorCountTmp)) >= (FixedLenTmp))
    {
        FixedTirgStartFalg = 0;
        ReturnData = 0;
        AlreadySewingAB = 0;
        StartReinforceSnub = 0;
    }
    else
    {
        /****************************************´¥·¢²Ù×÷**********************************************/
        if(FixedTirg)
        {
            /*****************ÈíÆð¶¯**********/
            if((SlowStartFalg) && ((RotorCount - RotorCountTmp) <= SlowStartCount) && (!AlreadySewingAB))
            {
                if(AdSpdRef > SlowStartSpeed)
                {
                    SpeedRef = SlowStartSpeed;
                }
                else
                {
                    SpeedRef = AdSpdRef;
                }
                TorsionConTrol = 2;
            }
            /************////Ç°¹Ìºó·ÀÖ¹ÃÍ¼ÓËÙ***********/
            else if(((RotorCount - RotorCountTmp)) < StartReinforceSnub)
            {
                SpeedRef = StartReinforceSpeed;
                TorsionConTrol = 2;
            }
            /**********µÚÒ»ÕëËÙÂÊ*************/
            else if((RotorCount - RotorCountTmp) < 1)
            {
                TorsionConTrol = 2;
                if(FirstNeedleSpeed > AdSpdRef)
                {
                    SpeedRef = AdSpdRef;
                }
                else
                {
                    SpeedRef = FirstNeedleSpeed;
                }
            } /**********½øÈë3Õë¼õËÙÇø***********************/
            else if(((RotorCount - RotorCountTmp)) >= (FixedLen - 5))
            {
                SpeedRefTmp = FixedSpeed;
                TorsionConTrol = 4;
                if(SpeedRefTmp > ((FixedLen - (RotorCount - RotorCountTmp)) * 45) + 45)
                {
                    SpeedRefTmp = ((FixedLen - (RotorCount - RotorCountTmp)) * 45) + 45;
                }
                SpeedRef = SpeedRefTmp;
            }
            /************Õý³£ÔËÐÐ**************************/
            else
            {
                SpeedRef = FixedSpeed;
                TorsionConTrol = 2;
            }

            if((AdSpdRef > (SpeedRefMin)) && (FixedTirgStartFalg == 0))
            {
                FixedTirgStartFalg = 1;

                MotorState = M_MOTOR_RUNING;
            }
            else
            {
                if((MotorState == M_MOTOR_RUNING) && (FixedTirgStartFalg == 0))
                {
                    MotorStopFlag = 1;
                    MotorStopHorL = (LedHaltAngle);
                }
            }
        }   // END if(FixedTirg)

        /****************************************ÊÖ¶¯²Ù×÷**********************************************/

        else
        {
            if((AdSpdRef >= SpeedRefMin))
            {

                if(MotorState == M_MOTOR_IDLE)
                {
                    MotorState = M_MOTOR_RUNING;
                }
                else if(MotorState == M_MOTOR_RUNING)
                {
                    /*****************ÈíÆð?**********/
                    if((SlowStartFalg) && ((RotorCount - RotorCountTmp) <= SlowStartCount) && (!AlreadySewingAB))
                    {
                        if(AdSpdRef > SlowStartSpeed)
                        {
                            SpeedRef = SlowStartSpeed;
                        }
                        else
                        {
                            SpeedRef = AdSpdRef;
                        }
                        TorsionConTrol = 2;
                    }/*******************Ç°¹Ìºó·ÀÖ¹ÃÍ¼ÓËÙ********************/
                    else if((RotorCount - RotorCountTmp) < StartReinforceSnub)
                    {
                        SpeedRef = StartReinforceSpeed;
                        TorsionConTrol = 2;
                    }
                    /********************µÚÒ»ÕëËÙÂÊ*************************/
                    else if((RotorCount - RotorCountTmp) < 1)
                    {
                        TorsionConTrol = 2;
                        if(FirstNeedleSpeed > AdSpdRef)
                        {
                            SpeedRef = AdSpdRef;
                        }
                        else
                        {
                            SpeedRef = FirstNeedleSpeed;
                        }
                    }
                    /******************½øÈë5Õë¼õËÙÇø***********************/
                    else if(((RotorCount - RotorCountTmp)) >= (FixedLen - 5))
                    {
                        TorsionConTrol = 4;
                        SpeedRefTmp = AdSpdRef;
                        if(SpeedRefTmp > ((FixedLen - (RotorCount - RotorCountTmp)) * 45) + 45)
                        {
                            SpeedRefTmp = ((FixedLen - (RotorCount - RotorCountTmp)) * 45) + 45;
                        }
                        SpeedRef = SpeedRefTmp;
                    }
                    /****************Õý³£ÔËÐÐ***************************/
                    else
                    {
                        SpeedRef = AdSpdRef;
                        TorsionConTrol = 2;
                    }
                }
            }

            /*****************Í£Ö¹********************************************************************/
            else
            {
                if(MotorState == M_MOTOR_RUNING)
                {
                    MotorStopFlag = 1;
                    MotorStopHorL = (LedHaltAngle);

                }

            }
            /*****************Ì§Ñ¹½Å***********************************************/

            if((AdSpdRef == 3) && (MotorState == M_MOTOR_IDLE))
            {
                TaiYaJiaoStart = 1;
            }
            else
            {
                TaiYaJiaoStart = 0;
            }
        }//Íê³ÉÊÖ¶¯²Ù×÷
        ReturnData = 1;
    }
    return ReturnData;
}




void SewingWait(int NextSatae)
{
    if((AdSpdRef == 0) && (MotorState == M_MOTOR_IDLE))
    {
        AdMotorIdle = 1;
    }
    else if((AdSpdRef >= SpeedRefMin) && (AdMotorIdle) && (TaiYaJiaoFlag == 0))
    {
        SewProcessState = NextSatae;
        AdMotorIdle = 0;
        RotorCount = 0;
        RotorCountTmp = RotorCount;
    }
    else
    {
        if(MotorState == M_MOTOR_RUNING)
        {
            MotorStopFlag = 1;
            SpeedRef = 0;
            MotorStopHorL = (LedHaltAngle);
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

void AdSpdDelay(void)
{
    static  int AdSpdDelayCount = 0;
    AdSpdDelayCount++;
    if(MotorState == M_MOTOR_IDLE) //Òª¿¼ ==2  ==3
    {
        if(AdSpdDelayCount >= AdSpdDelayTime)
        {
            AdSpdDelayCount = 0;
            AdMotorIdle = 0;
            AlreadySewingAB = 0;
            DaoFengELEState = 0;
            DaoFengFlag = 0;
            ELERung = 0;
            FixedTirgStartFalg = 0;
            StopingCutFalg = 0;
            SewingABOk = 0;
            StartReinforceSnub = 0;
            SewProcessState = 1;
            TaiYaJiaoStart = 0;

            Sewing_CDHighSpeedInFlag = 0;
            SewingABState = 0;
            AlreadySewingAB = 0;
            SewingCDState = 0;
            WaitEleCount = 0;
            FirstFinishFlag = 0;
            FixedLen2 = 0;
            EndReinforceSpeedChange = 0;

        }
        if(TaiYaJiaoEleOn)
        {
            TaiYaJiaoStart = 1;
            FinishTaiYaJiaoLock = 1;
        }
    }
    else
    {
        AdSpdDelayCount = 0;
    }
}


void SewingIdle(int StartReinforce)
{
    static int MotorInitAgain = 0;
    static int AdSpdRefGetZero = 0;

    Sewing_CDHighSpeedInFlag = 0;
    SewingABState = 0;
    AlreadySewingAB = 0;
    SewingCDState = 0;
    WaitEleCount = 0;
    FirstFinishFlag = 0;
    FixedLen2 = 0;
    EndReinforceSpeedChange = 0;

    if(AdSpdRef == 0)
    {
        if(FinishTaiYaJiaoFlag == 1)
        {
            TaiYaJiaoStart = 0;
        }
        MotorInitAgain = 0;
        if(AdSpdRefGetZero == 0)
        {
            AdSpdRefGetZero = 1;
        }
        if(FinishTaiYaJiaoLock == 0)
        {
            TaiYaJiaoStart = 0;
        }
    }
    if(AdSpdRef >= SpeedRefMin) //µç»úÆô¶¯Ê±£¬Ñ¹½Å±ØÐè·ÅÏÂÀ´£»
    {
        if(AdSpdRefGetZero == 1)
        {
            TaiYaJiaoStart = 0;
        }
    }
    if(AdSpdRef == 2)
    {
        FinishTaiYaJiaoLock = 0;
        if((MotorState == M_MOTOR_IDLE) && (MotorInitAgain == 0))
        {
            MotorInitAgain = 1;
            MotorStopHorL = 0; //Ó¦¸ÃÔÚÉÏÍ£ÕëÎ»
            if((RotorMacAngle > TiaoXianGanAngleMin) && (RotorMacAngle < TiaoXianGanAngleMax)) //¿¼ÂÇÉÏÍ£ÕëÎ»£¬»¹ÊÇÏÂÍ£ÕëÎ»
            {
            }
            else
            {
                MotorState = M_MOTOR_INIT;        //µç»ú¿ªÊ¼¶¯ÁË£»
            }
        }
        if(MotorState == M_MOTOR_IDLE)
        {
            if((RotorMacAngle > TiaoXianGanAngleMin) && (RotorMacAngle < TiaoXianGanAngleMax)) //¿¼ÂÇÉÏÍ£ÕëÎ»£¬»¹ÊÇÏÂÍ£ÕëÎ»
            {
                TaiYaJiaoStart = 1;
            }
        }
    }
    else if(AdSpdRef == 3) //Ì§Ñ¹½Å¹¤ÄÜ,»¹Òª×¢ÒâÔÚÉÏÍ£ÕëÎ»µÄÊ±ºò
    {
        FinishTaiYaJiaoLock = 0;
        if(MotorState == M_MOTOR_IDLE)
        {
            if((RotorMacAngle > TiaoXianGanAngleMin) && (RotorMacAngle < TiaoXianGanAngleMax)) //¿¼ÂÇÉÏÍ£ÕëÎ»£¬»¹ÊÇÏÂÍ£ÕëÎ»
            {
                TaiYaJiaoStart = 1;
            }
        }

        if((MotorState == M_MOTOR_IDLE) && (MotorInitAgain == 0))
        {
            MotorInitAgain = 1;
            MotorStopHorL = 0; //Ó¦¸ÃÔÚÉÏÍ£ÕëÎ»
            if((RotorMacAngle > TiaoXianGanAngleMin) && (RotorMacAngle < TiaoXianGanAngleMax)) //¿¼ÂÇÉÏÍ£ÕëÎ»£¬»¹ÊÇÏÂÍ£Õë»
            {

            }
            else
            {
                MotorState = M_MOTOR_INIT;        //µç»ú¿ªÊ¼¶¯ÁË£»
            }
        }
    }
    if((AdSpdRef >= SpeedRefMin) && (MotorState == M_MOTOR_IDLE) && (TaiYaJiaoFlag == 0) && (AdSpdRefGetZero == 1))
    {
        AdCutFlag = 0;
        AdSpdRefGetZero = 0;
        SewingABOk = 0;
        SewingABState = 0;
        RotorCount = 0;
        RotorCountTmp = RotorCount;
        MotorStopHorL = (LedHaltAngle);
        if(JiaXianQiEleOn)
        {
            JaiXianQiFlag = 1;
        }

        if(StartReinforce != 0)
        {
            SewProcessState = 2;
            MotorState = M_MOTOR_RUNING;
        }
        else
        {
            SewProcessState = 4;
        }
    }
}

void ExigenceCutDone(int OutState)
{

    TorsionConTrol = 2;
    MotorStopHorL = 0;

    if((MotorState == M_MOTOR_RUNING) || (MotorState == M_MOTOR_ARREST))
    {
        SpeedRef = 0;
        MotorStopFlag = 1;
        if(LedCut)
        {
            StopingCutFalg = 2;
        }
        SewProcessState = OutState;
    }
    else if(MotorState == M_MOTOR_IDLE)
    {
        SewProcessState = OutState;
        if(LedCut)
        {
            MotorState = M_MOTOR_PROCUTING;
        }
    }
}


void ExigenceCut(int OutState)
{
    if((SewProcessState > 1) && (TaiYaJiaoFlag == 0))
    {
        if(((AdCutFlag < 50) && (AdSpdRef == 2)) || ((AnQuanSwitch_ON) && (AnQuanSwitchModel == 2)))
        {
            if((SewProcessState != 500) && (SewProcessState != OutState))
            {
                SewProcessState = 500; //½øÈëÇ¿ÐÐÖÆ¶¯£»
                AdCutFlag = 50;
            }
        }
    }
}


void   BeforeEndReinforce(int EndReinforce, int OutState)
{
    static int AdToIdle = 0;
    if((AdSpdRef == 0) && (MotorState == M_MOTOR_IDLE))
    {
        AdToIdle = 1;
    }
    if((AdSpdRef >= SpeedRefMin) && (MotorState == M_MOTOR_IDLE) && (AdToIdle == 1))
    {
        SewProcessState = OutState;
        RotorCount = 0;
        RotorCountTmp = RotorCount - 1;
        AdToIdle = 0;
    }


    if(MotorState == M_MOTOR_RUNING)
    {
        SpeedRef = 0;
        MotorStopFlag = 1;
        MotorStopHorL = (LedHaltAngle);
    }
}
