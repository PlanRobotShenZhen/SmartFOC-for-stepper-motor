
#include "ExternGlobals.h"
void SysVarUserInit(void);
void ProcessVarInit2(void);
void UserInit(void)
{
//    UqsPid.OutMax = _IQ(0.95);
//    UqsPid.OutMin = _IQ(0);
//    UqsPid.UiMax = _IQ30(0.95);
//    UqsPid.UiMin = _IQ30(0);
//    UqsPid.Kp = _IQ(1);
//    UqsPid.Ki = _IQ(2);
//    UqsPid.Err=0;

//    UdsPid.OutMax = _IQ(0.95);
//    UdsPid.OutMin = _IQ(0);
//    UdsPid.UiMax = _IQ30(0.95);
//    UdsPid.UiMin = _IQ30(0);
//    UdsPid.Kp = _IQ(0.5);
//    UdsPid.Ki = _IQ(0.5);
//    UdsPid.Err=0;

//    pida1.OutMax = _IQ(0.7);
//    pida1.OutMin = _IQ(0);
//    pida1.UiMax = _IQ30(0.7);
//    pida1.UiMin = _IQ30(-0.7);
//    pida1.Kp = _IQ(1);
//    pida1.Ki = _IQ(1);
//    pida1.Err=0;

//	//pida1.EzhAdd=PidEzhAdd;
//    pida1.KfdbAdd=90;
//	  pida1.AddTor=5000;//µ÷Õû²Î¿¼µçÑ¹ºó¸Ä5000
//
//  	//pida1.EzhSub=PidEzhSub;
//  	pida1.KfdbSub=55;
//  	pida1.SubTor=6500;
//
//  	pida1.KfdbAddDs=22;
//  	pida1.KfdbSubDs=10;
//  	pida1.KfdbSatDs=25;
//
//  	pida1.AddTorDs=1200;
//  	pida1.SubTorDs=1200;
//  	pida1.SatTorDs=1200;

//    pidc1.OutMax = _IQ(0.5);
//    pidc1.OutMin = _IQ(-0.5);
//    pidc1.UiMax = _IQ30(0.5);
//    pidc1.UiMin = _IQ30(-0.5);
//    pidc1.Kp = _IQ(50);
//    pidc1.Ki = _IQ(5);
//    pidc1.Err=0;

//    pidc2.OutMax = _IQ(0.5);
//    pidc2.OutMin = _IQ(-0.5);
//    pidc2.UiMax = _IQ30(0.5);
//    pidc2.UiMin = _IQ30(-0.5);
//    pidc2.Kp = _IQ(400);
//    pidc2.Ki = _IQ(1.5);
//    pidc2.Err=0;


//    AdU.dbuffer_ptr=(long*)dbufferU;
//    AdU.coeff_ptr=(long*)coeffU;
//	AdU.cbindex=0;
//    AdU.order=FIR_ORDER;
//    AdU.init(&AdU);

//    AdV.dbuffer_ptr=(long*)dbufferV;
//    AdV.coeff_ptr=(long*)coeffV;
//    AdV.order=FIR_ORDER;
//    AdV.init(&AdV);
//
//    AdHall.dbuffer_ptr=(long*)dbufferHall;
//    AdHall.coeff_ptr=(long*)coeffHall;
//    AdHall.order=FIR_ORDER;
//    AdHall.init(&AdHall);



//    AdEle.dbuffer_ptr=(long*)dbufferEle;
//    AdEle.coeff_ptr=(long*)coeffEle;
//    AdEle.order=FIR_ORDER;
//    AdEle.init(&AdEle);

//	  GenVolt.dbuffer_ptr=(long*)dbufferVolt;
//    GenVolt.coeff_ptr=(long*)coeffVolt;
//    GenVolt.order=FIR_ORDER;
//    GenVolt.init(&GenVolt);

//	  svpwm.DcCoeff=_IQ(0.99);
//	  svpwm.PeriodMax=1875;
//    svpwm.MfuncPeriod =0x7fff;
//
//    CwCcFlag=0;
//
//    EPwm1Regs.TBPRD = 1875;
//    EPwm2Regs.TBPRD = 1875;
//    EPwm3Regs.TBPRD = 1875;
}

void MotorVarInit(void)
{

//    SemiAngle  = MotorVar[1];
//    PidEzhAdd1 = MotorVar[2];
//    PidEzhSub1 = MotorVar[3];
//    PidEzhAdd2 = MotorVar[4];
//    PidEzhSub2 = MotorVar[5];
//    IdleWaitMax =MotorVar[6];
//	RotorMacAngleOffSet=MotorVar[7];



//    DaoFengStartAngle = MotorVar[8];
//    DaoFengStoptAngle = MotorVar[9];
//   //TiaoXianGanAngleMin = HaltAngle-SysVarUser[83];
//   //TiaoXianGanAngleMax = HaltAngle+SysVarUser[84];
//
//	TestSpeed=MotorVar[10]/15;
//	MotorInitSpeed = MotorVar[11]/15;
//	InvarianceAddTime = MotorVar[12];

//	pida1.InvarianceAddTime = 20;
//	ReinforceTorsionOffset=MotorVar[14]*3;
//    ArrestOutSpeed1=MotorVar[16];
//	ArrestOutSpeed2=MotorVar[18]-30;
//    StopStartAngleL=MotorVar[17];
//    StopStartAngle2=MotorVar[19];

//    IqsMax=MotorVar[20];
//    IdsMax=MotorVar[21];
//    IqsMaxOverTime=MotorVar[22];
//    DcRefMax=MotorVar[23];
//    DcMaxErrRef=MotorVar[24];
//    DcRefMin=MotorVar[25];
//    DcMinErrRef=MotorVar[26];
//    DcArrestMax=MotorVar[27];
//    DcArrestDelayMax=MotorVar[28];
//    DcArrest2DelayMax=MotorVar[29];
//    LossACTimeMax=MotorVar[30];
//    ImeMax=MotorVar[31];
//    DcRef=MotorVar[33];
//	DcRefK=MotorVar[34];
//	DcRefB=MotorVar[35];

//    ProHaltUqs=MotorVar[37];
//	HaltProtectTimerMax=MotorVar[38]*300;
//	SubAddBuffer = MotorVar[39];
//    LowSpeedTimer=MotorVar[40];
}

void SysVarInit(void)
{

//   AdSpdRefK=SysVar[0];
//   SpeedRefMin=SysVar[2];
//   FreeSpeedMax=SysVar[3];
//   FoldSpeedRef=SysVar[4];
//   StartReinforceSpeefRef=SysVar[5];
//   EleDKS=(((1000L*32767L)/(((long)StartReinforceSpeefRef)/(60L)))/8);
//   EndReinforceSpeefRef=SysVar[6];
//   EleDKE=(((1000L*32767L)/(((long)EndReinforceSpeefRef)/(60L)))/8);
//   StartReinforcePause=!(SysVar[7]);
//   EndReinforcePause=!(SysVar[8]);
//   WSpeedRef=SysVar[9];
//   EleDKW=(((1000L*32767L)/(((long)WSpeedRef)/(60L)))/8);//2013.04.09
//   AngleHtoL=(int)((((long)(SysVar[10]))*(372463L))>>15);
//
//   CutHaltAngeOffset = SysVar[11];
//   HaltAngleOffset = SysVar[12];
//   HaltAngle = SysVar[13]+15;

//   TiaoXianGanAngleMin = HaltAngle-300;
//   TiaoXianGanAngleMax = HaltAngle+300;

//   CutRunAngle = HaltAngle+SysVar[14]+400;
//   ReinforceSpeefMax=SysVar[15];
//   ShaoXianTime=SysVar[16];
//   DakFengKey=SysVar[17];
//   NeedleSpeedPre=SysVar[18];
//   TaiYaJiaoEleOn=SysVar[19];
//   CountType=SysVar[21];
//   SlowStartCount=SysVar[22];
//   SlowStartSpeed=SysVar[23]/15;
//   DixianJiShu=SysVar[24];
//   DixianzongShu=SysVar[25];
//   DangQianDixiZhongShu=SysVar[26];
//   DaoFengEleFullVolt=SysVar[27];
//   DaoFengEleRun=SysVar[28];

//   CutStartMinAngle = SysVar[30]+500;
//   SaoxianAngleOffset=SysVar[31];
//   CutInMaxSpeed = SysVar[32];
//   StopingSpeed  = SysVar[33];

//   PoweOnAngleFlag=SysVar[34];
//   NeedelAddSpeedRef=(SysVar[35]*2)/29;
//   CutSpeedRef=SysVar[36];
//   AutoTaiYaJiao=SysVar[37];
//   AllSpeedRefMax=(SysVar[38]*2)/29;
//   FirstNeedleSpeed=SysVar[39]/15;
//   StartReinforceCompensate1=SysVar[40];
//   StartReinforceCompensate2=SysVar[41];
//   EndReinforceCompensate1=SysVar[42];
//   EndReinforceCompensate2=SysVar[43];
//   WCompensate1=SysVar[44];
//   WCompensate2=SysVar[45];
//
//   DelayEleS1=(int)((((long)(16-StartReinforceCompensate1))*(EleDKS))>>15);
//   DelayEleS2=(int)((((long)(16-StartReinforceCompensate2))*(EleDKS))>>15);
//   DelayEleE1=(int)((((long)(16-EndReinforceCompensate1))*(EleDKE))>>15);
//   DelayEleE2=(int)((((long)(16-EndReinforceCompensate2))*(EleDKE))>>15);
//   DelayEleSW=(int)((((long)(16-WCompensate1))*(EleDKW))>>15);
//   DelayEleEW=(int)((((long)(16-WCompensate2))*(EleDKW))>>15);
//
//
//   if(SysVarNum==46)
//   {
//       TestModelFlag=SysVar[46];
//   }
//   AgingStopTime=SysVar[47];
//   AgingRunTime=SysVar[48];
//   MotorCwCcw=SysVar[49];
//   SpeedGain=110-(SysVar[50]*10);
//   if(SpeedGain<60)
//   {
//      SpeedGain=60;
//   }
//   if(SpeedGain>110)
//   {
//       SpeedGain=110;
//   }

//   CuterModel=SysVar[51];
//   OutCutTime=SysVar[52];
//   OutShaoXianTime=SysVar[53];
//   SafetySwitchType=SysVar[54];
//   FinishTaiYaJiaoFlag=SysVar[55];
//   MachineRunTime=  SysVar[56];
//   TaiYaJiaoStartTime=SysVar[57];
//   TaiYaJiaoRunTime=SysVar[58];
//   TaiYaJiaoStopTime=SysVar[59];
//   TaiYaJiaoProtectTime=SysVar[60];
//   TaiYaJiaoDelayTimeS=SysVar[61];
//   TaiYaJiaoDelayTimeE=SysVar[62];
//   FangYaJiaoTimerMax=SysVar[63];
//   ErChiTaiYaJiaoTimerMax=SysVar[64];
//   ClothSensorFunction =SysVar[65];
//   ClothSensorType=SysVar[66];

//   StartReinforceSpeedOneToOne=SysVar[67]/15;// 2013.05.28
//   EndReinforceSpeedOneToOne=SysVar[68]/15;// 2013.05.28

//   ClothDelayStart=SysVar[69];
//   ClothStartNeedleNum=SysVar[70];
//   ClothEndNeedleNum=SysVar[71];
//   ZhuangShuiFlag = SysVar[72];
//   ClothSensorExist =SysVar[73];
//   TR_Pneumatic=SysVar[74];
//   CutCount=SysVar[76];
//   if(SysVar[76]==1)
//   {
//      SysVar[76]=0;
//      JianXianJiShu=0;
//	  ProcessVar[80]=0;
//
//   }

//   DelayEleS1OffsetInit=0-SysVar[77];// 2013.05.28
//   DelayEleE2OffsetInit=SysVar[78];// 2013.05.28

//   WaitEleCountMax=SysVar[79];
//   AdSpdDelayTime=SysVar[80];
//   AutoTaiYaJiaoDelay=SysVar[81];
//   FoldWSpeed=WSpeedRef/15;
//   FixedSpeed=FoldSpeedRef/15;

//  /*******************20120209****************/
//   StartReinforceSpeed=StartReinforceSpeefRef/15;
//   EndReinforceSpeed=EndReinforceSpeefRef/15;
//   SpeedRefMax=(FreeSpeedMax*2)/29;
//   if(SpeedRefMax>=335)
//   {
//      SpeedRefMax=335;
//   }
//   TestRunTime=AgingRunTime;
//   TestStopTime=AgingStopTime;
//   SpeedRefMin=SpeedRefMin/15;  //makshure
//   SpdCuting=(CutSpeedRef)/15;
//   /********************************************/

//



//     AdSpdH=SysVar[87];
//	 AdSpdZ=SysVar[88];
//	 AdSpdL=SysVar[89];
//	 AdSpdS=SysVar[90];
//	 if(AdSpdH<AdSpdL)
//     {
//	     AdSpdCw=1;
//         AdSpdHZ=SysVar[88]-SysVar[87];
//	     AdSpdL=-AdSpdL;
//	     AdSpdZ=-AdSpdZ;
//	     AdSpdH=-AdSpdH;
//	     AdSpdS=-AdSpdS;
//     }
//     else
//     {
//         AdSpdCw=0;
//		 AdSpdHZ=SysVar[87]-SysVar[89];
//		 AdSpdH=AdSpdH;
//		 AdSpdZ=AdSpdZ;
//         AdSpdL=(AdSpdL);
//         AdSpdS=AdSpdS;
//     }
//	 AdSpdSlope=(int)((((long)AdSpdHZ)*((long)AdSpdRefK))/100);
//
//	JiaXainStartMinAngle=SysVar[94];
//    JiaXainEndMinAngle=SysVar[95];
//	//SongXianMax=SysVarUser[95];
//	CCWAngle=SysVarUser[96];
//    DefaultPassWord=SysVarUser[98];
//    CraftsPassWord=SysVarUser[99];




//     pidc1.Kp = 13000000L;
//     pidc1.Ki = ((long)(SpdCuting))*(2000L);
//	 pidbUi   = (170000000L- (((long)SpdCuting)*2000000L));
}

void SysVarUserInit(void)
{
//   AdSpdRefK=SysVarUser[0];
//   SpeedRefMin=SysVarUser[2];
//   FreeSpeedMax=SysVarUser[3];
//   FoldSpeedRef=SysVarUser[4];
//   StartReinforceSpeefRef=SysVarUser[5];
//   EleDKS=(((1000L*32767L)/(((long)StartReinforceSpeefRef)/(60L)))/8);
//   EndReinforceSpeefRef=SysVarUser[6];
//   EleDKE=(((1000L*32767L)/(((long)EndReinforceSpeefRef)/(60L)))/8);
//   StartReinforcePause=!(SysVarUser[7]);
//   EndReinforcePause=!(SysVarUser[8]);
//   WSpeedRef=SysVarUser[9];
//   EleDKW=(((1000L*32767L)/(((long)WSpeedRef)/(60L)))/8);//2013.04.09
//   AngleHtoL=(int)((((long)(SysVarUser[10]))*(372463L))>>15);
//   CutHaltAngeOffset=SysVarUser[11];
//   HaltAngleOffset = SysVarUser[12];
//   HaltAngle=SysVarUser[13]+15;

//   TiaoXianGanAngleMin = HaltAngle-300;
//   TiaoXianGanAngleMax = HaltAngle+300;

//   CutRunAngle = HaltAngle+SysVarUser[14]+400;
//   ReinforceSpeefMax=SysVarUser[15];
//   ShaoXianTime=SysVarUser[16];
//   DakFengKey=SysVarUser[17];
//   NeedleSpeedPre=SysVarUser[18];
//   TaiYaJiaoEleOn=SysVarUser[19];
//   CountType=SysVarUser[21];
//   SlowStartCount=SysVarUser[22];
//   SlowStartSpeed=SysVarUser[23]/15;
//   DixianJiShu=SysVarUser[24];
//   DixianzongShu=SysVarUser[25];
//   DangQianDixiZhongShu=SysVarUser[26];
//   DaoFengEleFullVolt=SysVarUser[27];
//   DaoFengEleRun=SysVarUser[28];


//   CutStartMinAngle = SysVarUser[30]+500;
//   SaoxianAngleOffset=SysVarUser[31];
//   CutInMaxSpeed = SysVarUser[32];
//   StopingSpeed  = SysVarUser[33];

//   PoweOnAngleFlag=SysVarUser[34];
//   NeedelAddSpeedRef=(SysVarUser[35]*2)/29;
//   CutSpeedRef=SysVarUser[36];
//   AutoTaiYaJiao=SysVarUser[37];
//   AllSpeedRefMax=(SysVarUser[38]*2)/29;
//   FirstNeedleSpeed=SysVarUser[39]/15;
//   StartReinforceCompensate1=SysVarUser[40];
//   StartReinforceCompensate2=SysVarUser[41];
//   EndReinforceCompensate1=SysVarUser[42];
//   EndReinforceCompensate2=SysVarUser[43];
//   WCompensate1=SysVarUser[44];
//   WCompensate2=SysVarUser[45];
//
//   DelayEleS1=(int)((((long)(16-StartReinforceCompensate1))*(EleDKS))>>15);
//   DelayEleS2=(int)((((long)(16-StartReinforceCompensate2))*(EleDKS))>>15);
//   DelayEleE1=(int)((((long)(16-EndReinforceCompensate1))*(EleDKE))>>15);
//   DelayEleE2=(int)((((long)(16-EndReinforceCompensate2))*(EleDKE))>>15);
//   DelayEleSW=(int)((((long)(16-WCompensate1))*(EleDKW))>>15);
//   DelayEleEW=(int)((((long)(16-WCompensate2))*(EleDKW))>>15);

//   if(SysVarUserNum==46)
//   {
//       TestModelFlag=SysVarUser[46];
//   }
//   AgingStopTime=SysVarUser[47];
//   AgingRunTime=SysVarUser[48];
//   MotorCwCcw=SysVarUser[49];
//   SpeedGain=110-(SysVarUser[50]*10);
//   if(SpeedGain<60)
//   {
//       SpeedGain=60;
//   }
//   if(SpeedGain>110)
//   {
//       SpeedGain=110;
//   }
//   CuterModel=SysVarUser[51];
//   OutCutTime=SysVarUser[52];
//   OutShaoXianTime=SysVarUser[53];
//   SafetySwitchType=SysVarUser[54];
//   FinishTaiYaJiaoFlag=SysVarUser[55];
//   MachineRunTime=  SysVarUser[56];
//   TaiYaJiaoStartTime=SysVarUser[57];
//   TaiYaJiaoRunTime=SysVarUser[58];
//   TaiYaJiaoStopTime=SysVarUser[59];
//   TaiYaJiaoProtectTime=SysVarUser[60];
//   TaiYaJiaoDelayTimeS=SysVarUser[61];
//   TaiYaJiaoDelayTimeE=SysVarUser[62];
//   FangYaJiaoTimerMax=SysVarUser[63];
//   ErChiTaiYaJiaoTimerMax=SysVarUser[64];
//   ClothSensorFunction =SysVarUser[65];
//   ClothSensorType=SysVarUser[66];
//
//   StartReinforceSpeedOneToOne=SysVarUser[67]/15;// 2013.05.28
//   EndReinforceSpeedOneToOne=SysVarUser[68]/15;// 2013.05.28

//   ClothDelayStart=SysVarUser[69];
//   ClothStartNeedleNum=SysVarUser[70];
//   ClothEndNeedleNum=SysVarUser[71];
//   ZhuangShuiFlag=SysVarUser[72];
//   ClothSensorExist =SysVarUser[73];
//   TR_Pneumatic=SysVarUser[74];
//   CutCount=SysVarUser[76];
//     if(SysVarUser[76]==1)
//   {
//      SysVarUser[76]=0;
//      JianXianJiShu=0;
//	  ProcessVar[80]=0;
//   }
//   DelayEleS1OffsetInit=0-SysVarUser[77];// 2013.05.28
//   DelayEleE2OffsetInit=SysVarUser[78];// 2013.05.28

//   WaitEleCountMax=SysVarUser[79];
//   AdSpdDelayTime=SysVarUser[80];
//   AutoTaiYaJiaoDelay=SysVarUser[81];

//   FoldWSpeed=WSpeedRef/15;
//   FixedSpeed=FoldSpeedRef/15;

//  /*******************20120209****************/
//   StartReinforceSpeed=StartReinforceSpeefRef/15;
//   EndReinforceSpeed=EndReinforceSpeefRef/15;
//   SpeedRefMax=(FreeSpeedMax*2)/29;
//   if(SpeedRefMax>=335)
//   {
//      SpeedRefMax=335;
//   }
//   TestRunTime=AgingRunTime;
//   TestStopTime=AgingStopTime;
//   SpeedRefMin=SpeedRefMin/15;  //makshure
//   SpdCuting=(CutSpeedRef)/15;
//   /********************************************/


//
//    AdSpdH=SysVarUser[87];
//	  AdSpdZ=SysVarUser[88];
//	  AdSpdL=SysVarUser[89];
//	  AdSpdS=SysVarUser[90];
//	  if(AdSpdH<AdSpdL)
//    {
//	     AdSpdCw=1;
//       AdSpdHZ=SysVarUser[88]-SysVarUser[87];
//	     AdSpdL=-AdSpdL;
//	     AdSpdZ=-AdSpdZ;
//	     AdSpdH=-AdSpdH;
//	     AdSpdS=-AdSpdS;
//     }
//     else
//     {
//       AdSpdCw=0;
//		   AdSpdHZ=SysVarUser[87]-SysVarUser[89];
//		   AdSpdH=AdSpdH;
//		   AdSpdZ=AdSpdZ;
//       AdSpdL=(AdSpdL);
//       AdSpdS=AdSpdS;
//     }
//	   AdSpdSlope=(int)((((long)AdSpdHZ)*((long)AdSpdRefK))/100);



//	JiaXainStartMinAngle=SysVarUser[94];
//    JiaXainEndMinAngle=SysVarUser[95];
//	//SongXianMax=SysVarUser[95];
//	CCWAngle=SysVarUser[96];
//    DefaultPassWord=SysVarUser[98];
//    CraftsPassWord=SysVarUser[99];





//     pidc1.Kp = 13000000L;
//     pidc1.Ki = ((long)(SpdCuting))*(2000L);
//	 pidbUi   = (170000000L- (((long)SpdCuting)*2000000L));
}


void ProcessVarInit(void)
{
//	SewModel=ProcessVar[0];
//    LedTaiYaJiao=ProcessVar[1]&0x80;
//	LedCut=ProcessVar[1]&0x40;
//	LedSoftSpeed=ProcessVar[1]&0x20;
//	LedHaltAngle=ProcessVar[1]&0x10;
//	LedTieg=ProcessVar[1]&0x08;


//	SlowStartFalg=ProcessVar[2];
//	JiaXianQiEleOn=ProcessVar[3];
//	LedLiangDu=ProcessVar[4];
///************×ÔÓÉ·ì*********************/
//    FreeStartReinforce=ProcessVar[10];
//	FreeEndReinforce=ProcessVar[11];
//	FreeA=ProcessVar[12];
//	FreeB=ProcessVar[13];
//	FreeC=ProcessVar[14];
//	FreeD=ProcessVar[15];

///************¶¨³¤·ì********************/
//	Fixed1StartReinforce=ProcessVar[20];
//	Fixed1EndReinforce=ProcessVar[21];
//	Fixed1A=ProcessVar[22];
//	Fixed1B=ProcessVar[23];
//	Fixed1C=ProcessVar[24];
//	Fixed1D=ProcessVar[25];
//	Fixed1E=ProcessVar[26];
//

///************ËÄ¶Ï·ì*********************/
//	Fixed4StartReinforce=ProcessVar[30];
//	Fixed4EndReinforce=ProcessVar[31];
//	Fixed4A=ProcessVar[32];
//	Fixed4B=ProcessVar[33];
//	Fixed4C=ProcessVar[34];
//	Fixed4D=ProcessVar[35];
//	Fixed4E=ProcessVar[36];
//	Fixed4F=ProcessVar[37];
//	Fixed4G=ProcessVar[38];
//	Fixed4H=ProcessVar[39];

///************Æß¶Ï·ì*********************/
//	Fixed7StartReinforce=ProcessVar[40];
//	Fixed7EndReinforce=ProcessVar[41];
//	Fixed7A=ProcessVar[42];
//	Fixed7B=ProcessVar[43];
//	Fixed7C=ProcessVar[44];
//	Fixed7D=ProcessVar[45];
//	Fixed7E=ProcessVar[46];
//	Fixed7F=ProcessVar[47];
//	Fixed7G=ProcessVar[48];
//	Fixed7H=ProcessVar[49];

///************°Ë¶Ï·ì*********************/
//	Fixed8StartReinforce=ProcessVar[50];
//	Fixed8EndReinforce=ProcessVar[51];
//	Fixed8A=ProcessVar[52];
//	Fixed8B=ProcessVar[53];
//	Fixed8C=ProcessVar[54];
//	Fixed8D=ProcessVar[55];
//	Fixed8E=ProcessVar[56];
//	Fixed8F=ProcessVar[57];
//	Fixed8G=ProcessVar[58];
//	Fixed8H=ProcessVar[59];

///************ÕÛµþ·ì*********************/
//	FoldWA=ProcessVar[62];
//	FoldWB=ProcessVar[63];
//	FoldWH=ProcessVar[64];

//  JianXianJiShu=ProcessVar[80];
//  DangQianDixiZhongShu=ProcessVar[81];
//  MotorRunTime1h=ProcessVar[85];
//  SystemPowerOnTime1h=ProcessVar[86];

//  for(SysErrSaveTmp=0;SysErrSaveTmp<16;SysErrSaveTmp++)
//	{
//	    SysErrSave[SysErrSaveTmp] =  ProcessVar[90+SysErrSaveTmp];
//  }
//	for(SysErrSaveTmp=0;SysErrSaveTmp<15;SysErrSaveTmp++)
//	{
//	    SysErrCountSave[SysErrSaveTmp]=  ProcessVar[106+SysErrSaveTmp];
//	}
//	TestStartReinforce=ProcessVar[(SewModel*10)];
//	TestEndReinforce=ProcessVar[(SewModel*10)+1];
//    TestA=ProcessVar[(SewModel*10)+2];
//    TestB=ProcessVar[(SewModel*10)+3];
//    TestC=ProcessVar[(SewModel*10)+4];
//    TestD=ProcessVar[(SewModel*10)+5];
}




void ProcessVarInit2(void)
{
//	SewModel=ProcessVar[0];
//    LedTaiYaJiao=ProcessVar[1]&0x80;
//	LedCut=ProcessVar[1]&0x40;
//	LedSoftSpeed=ProcessVar[1]&0x20;
//	LedHaltAngle=ProcessVar[1]&0x10;
//	LedTieg=ProcessVar[1]&0x08;
//	SlowStartFalg=ProcessVar[2];
//	JiaXianQiEleOn=ProcessVar[3];
//    LedLiangDu=ProcessVar[4];

///************×ÔÓÉ·ì*********************/
//    FreeStartReinforce=ProcessVar[10];
//	FreeEndReinforce=ProcessVar[11];
//	FreeA=ProcessVar[12];
//	FreeB=ProcessVar[13];
//	FreeC=ProcessVar[14];
//	FreeD=ProcessVar[15];

///************¶¨³¤·ì********************/
//	Fixed1StartReinforce=ProcessVar[20];
//	Fixed1EndReinforce=ProcessVar[21];
//	Fixed1A=ProcessVar[22];
//	Fixed1B=ProcessVar[23];
//	Fixed1C=ProcessVar[24];
//	Fixed1D=ProcessVar[25];
//	Fixed1E=ProcessVar[26];
//

///************ËÄ¶Ï·ì*********************/
//	Fixed4StartReinforce=ProcessVar[30];
//	Fixed4EndReinforce=ProcessVar[31];
//	Fixed4A=ProcessVar[32];
//	Fixed4B=ProcessVar[33];
//	Fixed4C=ProcessVar[34];
//	Fixed4D=ProcessVar[35];
//	Fixed4E=ProcessVar[36];
//	Fixed4F=ProcessVar[37];
//	Fixed4G=ProcessVar[38];
//	Fixed4H=ProcessVar[39];

///************Æß¶Ï·ì*********************/
//	Fixed7StartReinforce=ProcessVar[40];
//	Fixed7EndReinforce=ProcessVar[41];
//	Fixed7A=ProcessVar[42];
//	Fixed7B=ProcessVar[43];
//	Fixed7C=ProcessVar[44];
//	Fixed7D=ProcessVar[45];
//	Fixed7E=ProcessVar[46];
//	Fixed7F=ProcessVar[47];
//	Fixed7G=ProcessVar[48];
//	Fixed7H=ProcessVar[49];

///************°Ë¶Ï·ì*********************/
//	Fixed8StartReinforce=ProcessVar[50];
//	Fixed8EndReinforce=ProcessVar[51];
//	Fixed8A=ProcessVar[52];
//	Fixed8B=ProcessVar[53];
//	Fixed8C=ProcessVar[54];
//	Fixed8D=ProcessVar[55];
//	Fixed8E=ProcessVar[56];
//	Fixed8F=ProcessVar[57];
//	Fixed8G=ProcessVar[58];
//	Fixed8H=ProcessVar[59];

///************ÕÛµþ·ì*********************/
//	FoldWA=ProcessVar[62];
//	FoldWB=ProcessVar[63];
//	FoldWH=ProcessVar[64];


//   JianXianJiShu=ProcessVar[80];
//   DangQianDixiZhongShu=ProcessVar[81];
//   MotorRunTime1h=ProcessVar[85];
//   SystemPowerOnTime1h=ProcessVar[86];

//   TestStartReinforce=ProcessVar[(SewModel*10)];
//   TestEndReinforce=ProcessVar[(SewModel*10)+1];
//   TestA=ProcessVar[(SewModel*10)+2];
//   TestB=ProcessVar[(SewModel*10)+3];
//   TestC=ProcessVar[(SewModel*10)+4];
//   TestD=ProcessVar[(SewModel*10)+5];
}
