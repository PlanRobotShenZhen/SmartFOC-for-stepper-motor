#include "ExternGlobals.h"
#include "Spi.h"
#include "Ds402_Slave.h"
#include "CanOpenMode.h"
#include "Function.h"
#include "Kalman.h"
extern Set_SP_Para PostionPlanCiA402Mode_1;
extern int PostionPlanStepCount;
extern int PostionPlanStepMax;
extern int PostionPlanStep;
extern bool postion_direction_flag;
int current_ref_test=1;
int UQsRef_test=5000;

short deadtime=15;
short  Moving_Average_Window_Filter_4096(short input, short CHANNEL_ID,short width);
short  Moving_Average_Window_Filter(short input, short CHANNEL_ID,short width);
Current_value value;

void SpeedCalculate(void);   // 1K    HZ
void MotorControl(void);     // 1K    HZ
void SvpwmControl(void) ;    // 16K   HZ
void CurrentLoopISR(void);   // 16K   HZ
void PID_IdIq(void);         // 16K   HZ
void park(SVPVM *v);


short tmp20 = 0;             //待定参数
short speedref_test=0;
int As_mean;
int Bs_mean;
int TEMP_A;
int TEMP_B;
int TEMP_Uq_filter32;
int TEMP_Uq_filter8;

/************************ SVPWM控制状态宏定义 ************************/
#define SVPWM_STATE_DISABLE           0   // 关闭状态（初始化、清除输出）
#define SVPWM_STATE_VF_MODE           1   // V/F控制模式
#define SVPWM_STATE_FIND_ZERO         2   // 寻找零点
#define SVPWM_STATE_BASIC_TEST        3   // 基础功能测试模式
#define SVPWM_STATE_POS_LOCK          4   // 使能后位置锁定模式
#define SVPWM_STATE_CURRENT_MODE      5   // 电流控制模式
#define SVPWM_STATE_SPEED_MODE        6   // 速度控制模式（对应上位机速度控制）
#define SVPWM_STATE_SPEED_MODE2       61  // 速度控制模式2（未使用）
#define SVPWM_STATE_POS_MODE          7   // 位置闭环控制模式
#define SVPWM_STATE_POS_LOCK_TEST     10  // 位置锁定测试模式（仅测试用）

/************************ 电机控制状态宏定义 ************************/
#define MOTOR_STATE_STOP              0   // 电机停止状态（初始化参数）
#define MOTOR_STATE_VF_RUN            1   // V/F模式运行
#define MOTOR_STATE_FIND_ZERO         2   // 寻找零点 
#define MOTOR_STATE_BASIC_TEST        3   // 基础功能测试
#define MOTOR_STATE_LOCK_AFTER_ENABLE 4   // 使能后的位置锁定
#define MOTOR_STATE_TORQUE_CONTROL    5   // 转矩控制模式
#define MOTOR_STATE_SPEED_CONTROL     6   // 速度闭环控制
#define MOTOR_STATE_SPEED_CONTROL2    61  // 速度闭环控制2（备用）
#define MOTOR_STATE_POS_HOLD          7   // 位置保持模式
#define MOTOR_STATE_POS_MOVE          71  // 位置规划运动模式（执行位置轨迹）

#define Bias_current_mode 3

#define FLITER_MODE 0            //1为卡尔曼滤波，0为滑动均值

/************************前馈控制 ************************/
 static int prev_speed_ref = 0; // 上一周期的速度命令
 #define FF_GAIN 10 // 前馈增益

#if FLITER_MODE == 0
void CurrentLoopISR(void)   //CurrentLoopISR 16K HZ     电流环的ISR
{
	
	#if Bias_current_mode==1
    if (SystemError.ImeasOffsetFlag < 10000) {  //在初始时大量采集各相位的电流偏置作为后续采集相电流补偿
        SystemError.ImeasAOffset =  Moving_Average_Window_Filter_4096(SystemError.ImeasA,0,4095);//ADC获取的A相电流的均值 得到系统误差中的A相位电流补偿
        SystemError.ImeasBOffset =  Moving_Average_Window_Filter_4096(SystemError.ImeasB,1,4095);//ADC获取的B相电流的均值 得到系统误差中的B相位电流补偿
		    SystemError.ImeasOffsetFlag++;
        SystemError.SysErr = 0;
			
    } else 
		{ //各相测量电流减去补偿值应为真实电流值
        TEMP_A = Moving_Average_Window_Filter(SystemError.ImeasA,0,8) - SystemError.ImeasAOffset;			//As 这个参数是测量电流 - 测量电流的补偿  
        TEMP_B = Moving_Average_Window_Filter(SystemError.ImeasB,1,8) - SystemError.ImeasBOffset;			//Bs 这个参数是测量电流 - 测量电流的补偿  
		    As_mean = Moving_Average_Window_Filter_4096(TEMP_A,2,4095);
		    Bs_mean = Moving_Average_Window_Filter_4096(TEMP_B,3,4095);
		    svpwm.As=TEMP_A-As_mean;
		    svpwm.Bs=TEMP_B-Bs_mean;
		}
	#endif
	#if	Bias_current_mode==2
	    if (SystemError.ImeasOffsetFlag < 10000) 
		{  //在初始时大量采集各相位的电流偏置作为后续采集相电流补偿
        SystemError.ImeasAOffset =  Moving_Average_Window_Filter_4096(SystemError.ImeasA,0,4095);//ADC获取的A相电流的均值 得到系统误差中的A相位电流补偿
        SystemError.ImeasBOffset =  Moving_Average_Window_Filter_4096(SystemError.ImeasB,1,4095);//ADC获取的B相电流的均值 得到系统误差中的B相位电流补偿
		    SystemError.ImeasOffsetFlag++;
        SystemError.SysErr = 0;
		} 
		else 
		{ //各相测量电流减去补偿值应为真实电流值
		    svpwm.As =SystemError.ImeasA- SystemError.ImeasAOffset-30;			//As 这个参数是测量电流 - 测量电流的补偿  
		    svpwm.Bs =SystemError.ImeasB - SystemError.ImeasBOffset+40;
		}

		
	#endif
	#if Bias_current_mode==3
		  SystemError.ImeasAOffset = Moving_Average_Window_Filter_4096(SystemError.ImeasA,0,4095);
		  SystemError.ImeasBOffset = Moving_Average_Window_Filter_4096(SystemError.ImeasB,1,4095);
	
	  	svpwm.As = Moving_Average_Window_Filter(SystemError.ImeasA,0,8)- SystemError.ImeasAOffset;			//As 这个参数是测量电流 - 测量电流的补偿  
    	svpwm.Bs = Moving_Average_Window_Filter(SystemError.ImeasB,1,8) - SystemError.ImeasBOffset;
	#endif
	//计算电角度 = 机械角度×极对数
		
    svpwm.Angle2 = (((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset1) & 0x7fff))) * ((
                   int)MotorControler.MotorPoles)) & 0x7fff;           //与上0x7fff，即使数据保持在15位范围内
		//park
	  park(&svpwm);
    PID_IdIq();     //函数这里是计算编码器获得的角度信息补偿
    SvpwmControl(); // 16KHZ  根据不同的工作模式设定对应的Uq和Uq变化步长设置

    //斜坡逼近，由不同模式对应的步长变化，防止跳变
    if (svpwm.UQs < svpwm.UQsRef) {
        svpwm.UQs = svpwm.UQs + svpwm.UQsstep;
    }

    if (svpwm.UQs > svpwm.UQsRef) {
        svpwm.UQs = svpwm.UQs - svpwm.UQsstep;
    }

    //限幅
    if (svpwm.UQs > svpwm.UQ_MAX) {
        svpwm.UQs = svpwm.UQ_MAX;
    }

    if (svpwm.UQs < -svpwm.UQ_MAX) {
        svpwm.UQs = -svpwm.UQ_MAX;
    }

    if ((SystemError.SysErr) && (SystemError.SysErr != M_SYSERR_CODER)) { //出了系统故障，强制关闭输出
        svpwm.UQs = 0;
        svpwm.UDs = 0;
    }

    ipark(&svpwm);           //反Park变换，由d-q分量转换到α-β坐标
    PWM(&svpwm);             //由Va、Vb计算占空比


	  TIM1->CCDAT1 = svpwm.Va;  // A相占空比
    TIM1->CCDAT2 = svpwm.Va;  // A相互补占空比
    TIM1->CCDAT3 = svpwm.Vb;  // B相占空比
    TIM1->CCDAT4 = svpwm.Vb;  // B相互补占空比
}

/*-----------------------------------------------*/
void PID_IdIq(void)					 //D轴Q轴电流的PID
{
	  int speed_change = pidpv.Ref - prev_speed_ref; // 速度变化量
    prev_speed_ref = pidpv.Ref; // 更新反馈值
	
    //D轴电流环
    UdsPid.Ref = 0;
	  UdsPid.Fdb = svpwm.IDs;
    UdsPid.calc(&UdsPid);
		
    //Q轴电流环
    UqsPid.Fdb =svpwm.IQs;//q15
		
    //速度环与电流环的前馈控制
	  float pidOutput = pidpv.Out * 500 / 8000;
    int feedforward = speed_change * FF_GAIN; //前馈补偿值
	  UqsPid.Ref = pidOutput + feedforward;

    UqsPid.CurrentRef = svpwm.CurrentRef;
		UqsPid.SpeedFdbp  = MotorControler.SpeedFdbp;
    UqsPid.UQs = svpwm.UQs;
    UqsPid.calc(&UqsPid);//用于电流限幅
    MotorControler.AngleFromMT6835Offset = MotorControler.AngleFromMT6835Offset1 + tmp20; // + UdsPid.Out;
		
}
/*-----------------------------------------------*/


void SvpwmControl(void)   // 16K HZ					SVPWM控制
{
    static short RotorElectricalMacAngle = 0;
    int   RotorMacAngleTemp = 0;

    switch (svpwm.SvpwmControlState) {
    case SVPWM_STATE_DISABLE ://下使能
        PwmShut();  //清零输出
        svpwm.DcCoeff = SystemError.DcCoeff;     //32400; //_IQ(0.99);
        svpwm.PeriodMax = 4499;
        svpwm.MfuncPeriod = 0x7fff;
        svpwm.UDs = 0;
        svpwm.UQsRef = 0;
        svpwm.UQs = 0;
		    svpwm.UQ_MAX =_IQ15(0.9);//8192//16384//29491
        RotorElectricalMacAngle = 0;
        pidpv.Ui = 0;
        pidholding.Ui = 0;
        UdsPid.Ui = 0;         //重置各个PID参数
        UqsPid.Ui = 0;
        UdsPid.OutMax = _IQ15(0.3);
        UdsPid.OutMin = _IQ15(-0.3);
        UdsPid.UiMax = _IQ30(0.2);
        UdsPid.UiMin = _IQ30(-0.2);
		
        UdsPid.Kp = 200;
        UdsPid.Ki = 30;
        UdsPid.Err = 0;
		
        UqsPid.Kp = 2000;
        UqsPid.Ki =5;
        UqsPid.Err = 0;
				
        UqsPid.OutMax = _IQ15(0.9);
        UqsPid.OutMin = _IQ15(-0.9);
        UqsPid.UiMax = pidpv_UiMax;
        UqsPid.UiMin = pidpv_UiMin;
        svpwm.UQsstep = 20;
        //LED_Red_Flash();//测试灯********************************
        break;

    case SVPWM_STATE_VF_MODE :    //  V/F模式
        svpwm.DcCoeff = SystemError.DcCoeff;//电压修正系数设置
        PwmOpen();
       // svpwm.UQsRef = SystemVar.VF_Voltage;   //使用固定的开环参数运行电机
		    svpwm.UQsRef = 4000;
	
        svpwm.Angle = SystemVar.VF_ElectricalAngle;
        break;

    case SVPWM_STATE_FIND_ZERO :    //  寻找零点
        svpwm.DcCoeff = SystemError.DcCoeff;
        PwmOpen();
        RotorElectricalMacAngle = RotorElectricalMacAngle + 10;//每周期增加10单位
        RotorElectricalMacAngle = RotorElectricalMacAngle & 0x7fff;//保证角度在设置范围内循环
        svpwm.Angle = RotorElectricalMacAngle;
        break;

    case SVPWM_STATE_BASIC_TEST :    //  基本测试
        svpwm.DcCoeff = 4096;//SystemError.DcCoeff;
        PwmOpen();
	    	//将编码器读取的角度转换为电角度
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
        svpwm.UQsRef = SystemVar.test_uqs;//设置测试电压Uq目标值
        svpwm.Angle = RotorElectricalMacAngle;//设置编码器电角度
        break;

    case SVPWM_STATE_POS_LOCK ://  上使能  位置锁定
		if( MotorControler.State == 4){
        svpwm.DcCoeff = SystemError.DcCoeff;
        PwmOpen();
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
       
				svpwm.Angle = RotorElectricalMacAngle;
        pidholding.Fdb = MotorControler.MotorActivePostion;  //反馈设置为电机活动位置
        pidholding.Speedref = 0;             //速度参考值设置为0，即保持某一位置不动
        pidholding.Speedfdb = MotorControler.SpeedFdbp;
        pidholding.Iqsfdb = svpwm.IQs;
		
        pidholding.calc(&pidholding);
		
        svpwm.UQsRef = pidholding.Out;
        UqsPid.Ui = 0;
        UqsPid.Out = 0;			//锁定后，会有一部分电流
	   } else
	  	{
			 PwmShut();   //清零输出
			 pidholding.Ui = 0;   //积分清零
	    }
        break;

    case SVPWM_STATE_CURRENT_MODE :   // 电流模式
        svpwm.DcCoeff = SystemError.DcCoeff;
        PwmOpen();
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
        svpwm.Angle = RotorElectricalMacAngle;
        svpwm.UQs = pidpt.Out;
        break;

    case SVPWM_STATE_SPEED_MODE :   // 速度模式  --对应上位机的速度控制模式
        svpwm.DcCoeff = SystemError.DcCoeff;
        PwmOpen();
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
        svpwm.Angle = RotorElectricalMacAngle;
				//svpwm.UQsRef = (Ids_filter4(UqsPid.Out)*svpwm.UQ_MAX)/UqsPid.OutMax;		//设置斜坡逼近的Uq目标值为速度PID的输出和UqPID的输出	
				TEMP_Uq_filter32 = Moving_Average_Window_Filter(UqsPid.Out,2,31);
				TEMP_Uq_filter8 = Moving_Average_Window_Filter(UqsPid.Out,3,7);
				
				if(pidpv.Ref<130&&pidpv.Ref>80)
				{
					svpwm.UQsRef=TEMP_Uq_filter8;
				}
				if(pidpv.Ref<80)
				{
//					svpwm.UQsRef=UqsPid.Out;
					svpwm.UQsRef=TEMP_Uq_filter32;
				}
				if(pidpv.Ref>160)
				{
					svpwm.UQsRef=UqsPid.Out;
				}
				else
				{
					svpwm.UQsRef=TEMP_Uq_filter8;
				}
//				svpwm.UQsRef = pidpv.Out;
//				svpwm.UQsRef=UQsRef_test;
        break;

    case SVPWM_STATE_SPEED_MODE2 :   // 速度模式2								--	没有被使用到
        svpwm.DcCoeff = SystemError.DcCoeff;//32400;
        PwmOpen();
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
        svpwm.Angle = RotorElectricalMacAngle;
        pidptv.Ref = (pidpv.Out) >> 5; //将速度环PID的输出缩放映射成电流环-速度模式PID的预期值并做限幅
        if (pidptv.Ref > 600) {
            pidptv.Ref = 600;
        }

        if (pidptv.Ref < -600) {
            pidptv.Ref = -600;
        }

        pidptv.Fdb = svpwm.IQs;
        pidptv.calc(&pidptv);
        svpwm.UQs = pidptv.Out;  //测试模式区别在于对应的PID环输出直接作用于SVPWM，而不是通过斜坡逼近
        break;

    case SVPWM_STATE_POS_MODE:   //位置环模式      -----------上位机 位置控制模式
		    svpwm.DcCoeff = SystemError.DcCoeff;
        PwmOpen();
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
        svpwm.Angle = RotorElectricalMacAngle;
				TEMP_Uq_filter32 = Moving_Average_Window_Filter(UqsPid.Out,2,31);
				TEMP_Uq_filter8 = Moving_Average_Window_Filter(UqsPid.Out,3,7);
				if(pidpv.Ref<130&&pidpv.Ref>80)
				{
					svpwm.UQsRef=TEMP_Uq_filter8;
				}
				if(pidpv.Ref>160)
				{
					svpwm.UQsRef=UqsPid.Out;
				}
				if(pidpv.Ref>160)
				{
					svpwm.UQsRef=UqsPid.Out;
				}
				else
				{
					svpwm.UQsRef=TEMP_Uq_filter8;
				}
        break;

    case SVPWM_STATE_POS_LOCK_TEST ://  上使能  位置锁定  测试用
        svpwm.DcCoeff = SystemError.DcCoeff;
        PwmOpen();
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
        svpwm.Angle = RotorElectricalMacAngle;
        pidholding.Ref = 0;
        pidholding.Fdb = MotorControler.MotorActivePostion;
        pidholding.calc(&pidholding);
        svpwm.UQs = pidholding.Out;
        break;

    default: {
    }
    break;
    }
}
#endif
#if FLITER_MODE == 1
void CurrentLoopISR(void)   //CurrentLoopISR 16K HZ     电流环的ISR
{
	
	#if Bias_current_mode==1
    if (SystemError.ImeasOffsetFlag < 10000) {  //在初始时大量采集各相位的电流偏置作为后续采集相电流补偿
        SystemError.ImeasAOffset =  Moving_Average_Window_Filter_4096(SystemError.ImeasA,0,4095);//ADC获取的A相电流的均值 得到系统误差中的A相位电流补偿
        SystemError.ImeasBOffset =  Moving_Average_Window_Filter_4096(SystemError.ImeasB,1,4095);//ADC获取的B相电流的均值 得到系统误差中的B相位电流补偿
		    SystemError.ImeasOffsetFlag++;
        SystemError.SysErr = 0;
			
    } else 
		{ //各相测量电流减去补偿值应为真实电流值
        TEMP_A = Kalman_Filter(SystemError.ImeasA,0) - SystemError.ImeasAOffset;			//As 这个参数是测量电流 - 测量电流的补偿  
        TEMP_B = Kalman_Filter(SystemError.ImeasB,1) - SystemError.ImeasBOffset;			//Bs 这个参数是测量电流 - 测量电流的补偿  
		As_mean = Kalman_Filter(TEMP_A,2);
		Bs_mean = Kalman_Filter(TEMP_B,3);
		svpwm.As=TEMP_A-As_mean;
		svpwm.Bs=TEMP_B-Bs_mean;
		}
	#endif
	#if	Bias_current_mode==2
	    if (SystemError.ImeasOffsetFlag < 10000) 
		{  //在初始时大量采集各相位的电流偏置作为后续采集相电流补偿
        SystemError.ImeasAOffset =  Kalman_Filter(SystemError.ImeasA,0);//ADC获取的A相电流的均值 得到系统误差中的A相位电流补偿
        SystemError.ImeasBOffset =  Kalman_Filter(SystemError.ImeasB,1);//ADC获取的B相电流的均值 得到系统误差中的B相位电流补偿
		SystemError.ImeasOffsetFlag++;
        SystemError.SysErr = 0;
		} 
		else 
		{ //各相测量电流减去补偿值应为真实电流值
		svpwm.As =SystemError.ImeasA- SystemError.ImeasAOffset-30;			//As 这个参数是测量电流 - 测量电流的补偿  
		svpwm.Bs =SystemError.ImeasB - SystemError.ImeasBOffset+40;
		}

		
	#endif
	#if Bias_current_mode==3
		SystemError.ImeasAOffset = Kalman_Filter(SystemError.ImeasA,0);
		SystemError.ImeasBOffset = Kalman_Filter(SystemError.ImeasB,1);
	
	  	svpwm.As = Kalman_Filter(SystemError.ImeasA,0)- SystemError.ImeasAOffset;			//As 这个参数是测量电流 - 测量电流的补偿  
    	svpwm.Bs = Kalman_Filter(SystemError.ImeasB,1) - SystemError.ImeasBOffset;
	#endif
	//计算电角度 = 机械角度×极对数
		
    svpwm.Angle2 = (((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset1) & 0x7fff))) * ((
                   int)MotorControler.MotorPoles)) & 0x7fff;           //与上0x7fff，即使数据保持在15位范围内
		//park
	park(&svpwm);
    PID_IdIq();     //函数这里是计算编码器获得的角度信息补偿
    SvpwmControl(); // 16KHZ  根据不同的工作模式设定对应的Uq和Uq变化步长设置

    //斜坡逼近，由不同模式对应的步长变化，防止跳变
    if (svpwm.UQs < svpwm.UQsRef) {
        svpwm.UQs = svpwm.UQs + svpwm.UQsstep;
    }

    if (svpwm.UQs > svpwm.UQsRef) {
        svpwm.UQs = svpwm.UQs - svpwm.UQsstep;
    }

    //限幅
    if (svpwm.UQs > svpwm.UQ_MAX) {
        svpwm.UQs = svpwm.UQ_MAX;
    }

    if (svpwm.UQs < -svpwm.UQ_MAX) {
        svpwm.UQs = -svpwm.UQ_MAX;
    }

    if ((SystemError.SysErr) && (SystemError.SysErr != M_SYSERR_CODER)) { //出了系统故障，强制关闭输出
        svpwm.UQs = 0;
        svpwm.UDs = 0;
    }

    ipark(&svpwm);           //反Park变换，由d-q分量转换到α-β坐标
    PWM(&svpwm);             //由Va、Vb计算占空比


	  TIM1->CCDAT1= svpwm.Va;  // A相占空比
    TIM1->CCDAT2= svpwm.Va;  // A相互补占空比
    TIM1->CCDAT3 = svpwm.Vb; // B相占空比
    TIM1->CCDAT4 = svpwm.Vb; // B相互补占空比
}

/*-----------------------------------------------*/
void PID_IdIq(void)					 //D轴Q轴电流的PID
{
	  int speed_change = pidpv.Ref - prev_speed_ref; // 速度变化量
    prev_speed_ref = pidpv.Ref; // 更新
    //D轴电流环
    UdsPid.Ref = 0;
	  UdsPid.Fdb = svpwm.IDs;
    UdsPid.calc(&UdsPid);
		
    //Q轴电流环
    UqsPid.Fdb =svpwm.IQs;     //q15
		
    //速度环与电流环的前馈控制
	  float pidOutput = pidpv.Out * 500 / 8000;
    int feedforward = speed_change * FF_GAIN; //前馈补偿值
    UqsPid.Ref = pidOutput + feedforward;

    UqsPid.UQs = svpwm.UQs;
    UqsPid.calc(&UqsPid);//用于电流限幅
    MotorControler.AngleFromMT6835Offset = MotorControler.AngleFromMT6835Offset1 + tmp20; // + UdsPid.Out;
		
}
/*-----------------------------------------------*/

void SvpwmControl(void)   // 16K HZ					SVPWM控制
{
    static short RotorElectricalMacAngle = 0;
    int   RotorMacAngleTemp = 0;

    switch (svpwm.SvpwmControlState) {
    case SVPWM_STATE_DISABLE ://下使能
        PwmShut();  //清零输出
        svpwm.DcCoeff = SystemError.DcCoeff;//32400; //_IQ(0.99);
        svpwm.PeriodMax = 4499;
        svpwm.MfuncPeriod = 0x7fff;
        svpwm.UDs = 0;
        svpwm.UQsRef = 0;
        svpwm.UQs = 0;
	    	svpwm.UQ_MAX =_IQ15(0.8);//8192//16384//29491
        RotorElectricalMacAngle = 0;
        pidpv.Ui = 0;
        pidholding.Ui = 0;
        UdsPid.Ui = 0;         //重置各个PID参数
        UqsPid.Ui = 0;
        UdsPid.OutMax = _IQ15(0.3);
        UdsPid.OutMin = _IQ15(-0.3);
        UdsPid.UiMax = _IQ30(0.2);
        UdsPid.UiMin = _IQ30(-0.2);
		
        UdsPid.Kp = 200;
        UdsPid.Ki = 30;
        UdsPid.Err = 0;
		
        UqsPid.Kp = 2000;
        UqsPid.Ki =5;
        UqsPid.Err = 0;
				
        UqsPid.OutMax = _IQ15(0.8);
        UqsPid.OutMin = _IQ15(-0.8);
        UqsPid.UiMax = pidpv_UiMax;
        UqsPid.UiMin = pidpv_UiMin;
        svpwm.UQsstep = 20;
        //LED_Red_Flash();//测试灯********************************
        break;

    case SVPWM_STATE_VF_MODE :    //  V/F模式
        svpwm.DcCoeff = SystemError.DcCoeff;//电压修正系数设置
        PwmOpen();
       // svpwm.UQsRef = SystemVar.VF_Voltage;   //使用固定的开环参数运行电机
		svpwm.UQsRef = 4000;
	
        svpwm.Angle = SystemVar.VF_ElectricalAngle;
        break;

    case SVPWM_STATE_FIND_ZERO :    //  寻找零点
        svpwm.DcCoeff = SystemError.DcCoeff;
        PwmOpen();
        RotorElectricalMacAngle = RotorElectricalMacAngle + 10;//每周期增加10单位
        RotorElectricalMacAngle = RotorElectricalMacAngle & 0x7fff;//保证角度在设置范围内循环
        svpwm.Angle = RotorElectricalMacAngle;
        break;

    case SVPWM_STATE_BASIC_TEST :    //  基本测试
        svpwm.DcCoeff = 4096;//SystemError.DcCoeff;
        PwmOpen();
		//将编码器读取的角度转换为电角度
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
        svpwm.UQsRef = SystemVar.test_uqs;//设置测试电压Uq目标值
        svpwm.Angle = RotorElectricalMacAngle;//设置编码器电角度
        break;

    case SVPWM_STATE_POS_LOCK ://  上使能  位置锁定
		if( MotorControler.State == 4){
        svpwm.DcCoeff = SystemError.DcCoeff;
        PwmOpen();
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
        svpwm.Angle = RotorElectricalMacAngle;
        pidholding.Fdb = MotorControler.MotorActivePostion;  //反馈设置为电机活动位置
        pidholding.Speedref = 0;             //速度参考值设置为0，即保持某一位置不动
        pidholding.Speedfdb = MotorControler.SpeedFdbp;
        pidholding.Iqsfdb = svpwm.IQs;
		
        pidholding.calc(&pidholding);
		
        svpwm.UQsRef = pidholding.Out;
        UqsPid.Ui = 0;
        UqsPid.Out = 0;			//锁定后，会有一部分电流
	   } else
	  	{
			 PwmShut();   //清零输出
			 pidholding.Ui = 0;   //积分清零
	    }
        break;

    case SVPWM_STATE_CURRENT_MODE :   // 电流模式
        svpwm.DcCoeff = SystemError.DcCoeff;
        PwmOpen();
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
        svpwm.Angle = RotorElectricalMacAngle;
        svpwm.UQs = pidpt.Out;
        break;

    case SVPWM_STATE_SPEED_MODE :   // 速度模式  --对应上位机的速度控制模式
        svpwm.DcCoeff = SystemError.DcCoeff;
        PwmOpen();
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
        svpwm.Angle = RotorElectricalMacAngle;
			//svpwm.UQsRef = (Ids_filter4(UqsPid.Out)*svpwm.UQ_MAX)/UqsPid.OutMax;		//设置斜坡逼近的Uq目标值为速度PID的输出和UqPID的输出	
				TEMP_Uq_filter32 = Kalman_Filter(UqsPid.Out,2);
				TEMP_Uq_filter8 = Kalman_Filter(UqsPid.Out,3);
				if(pidpv.Ref<130&&pidpv.Ref>80)
				{
					svpwm.UQsRef=TEMP_Uq_filter8;
				}
				if(pidpv.Ref>160)
				{
					svpwm.UQsRef=UqsPid.Out;
				}
				if(pidpv.Ref>160)
				{
					svpwm.UQsRef=UqsPid.Out;
				}
				else
				{
					svpwm.UQsRef=TEMP_Uq_filter8;
				}

        break;

    case SVPWM_STATE_SPEED_MODE2 :   // 速度模式2								--	没有被使用到
        svpwm.DcCoeff = SystemError.DcCoeff;//32400;
        PwmOpen();
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
        svpwm.Angle = RotorElectricalMacAngle;
        pidptv.Ref = (pidpv.Out) >> 5; //将速度环PID的输出缩放映射成电流环-速度模式PID的预期值并做限幅
        if (pidptv.Ref > 600) {
            pidptv.Ref = 600;
        }

        if (pidptv.Ref < -600) {
            pidptv.Ref = -600;
        }

        pidptv.Fdb = svpwm.IQs;
        pidptv.calc(&pidptv);
        svpwm.UQs = pidptv.Out;  //测试模式区别在于对应的PID环输出直接作用于SVPWM，而不是通过斜坡逼近
        break;

    case SVPWM_STATE_POS_MODE:   //位置环模式      -----------上位机 位置控制模式
		    svpwm.DcCoeff = SystemError.DcCoeff;
        PwmOpen();
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
        svpwm.Angle = RotorElectricalMacAngle;
				//svpwm.UQsRef = (Ids_filter4(UqsPid.Out)*svpwm.UQ_MAX)/UqsPid.OutMax;		//设置斜坡逼近的Uq目标值为速度PID的输出和UqPID的输出	
				TEMP_Uq_filter32 = Kalman_Filter(UqsPid.Out,2);
				TEMP_Uq_filter8 = Kalman_Filter(UqsPid.Out,3);
				if(pidpv.Ref<130&&pidpv.Ref>80)
				{
					svpwm.UQsRef=TEMP_Uq_filter8;
				}
				if(pidpv.Ref>160)
				{
					svpwm.UQsRef=UqsPid.Out;
				}
				if(pidpv.Ref>160)
				{
					svpwm.UQsRef=UqsPid.Out;
				}
				else
				{
					svpwm.UQsRef=TEMP_Uq_filter8;
				}
        break;

    case SVPWM_STATE_POS_LOCK_TEST ://  上使能  位置锁定  测试用
        svpwm.DcCoeff = SystemError.DcCoeff;
        PwmOpen();
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
        svpwm.Angle = RotorElectricalMacAngle;
        pidholding.Ref = 0;
        pidholding.Fdb = MotorControler.MotorActivePostion;
        pidholding.calc(&pidholding);
        svpwm.UQs = pidholding.Out;
        break;

    default: {
    }
    break;
    }
}
#endif

//根据电机运行状态执行对应的控制策略
void MotorControl(void)  //1KHZ
{
    static short ExampleCount = 0;
    static short VF_Dir = 0;              //开环模式
    static int RotorMacAngleTemp1 = 0;    //零点认别
    static int RotorMacAngleTemp2 = 0;    //零点认别
    static short RotorMacAngleTemp3 = 0;  //零点认别
    ExampleCount++;

    if (ExampleCount >= 10) {
        ExampleCount = 0;

        switch (MotorControler.State) {
        case MOTOR_STATE_STOP : {
            svpwm.SvpwmControlState = SVPWM_STATE_DISABLE;
            MotorControler.Error = 0;
            /*****VF部分清零**************/
            SystemVar.VF_ElectricalAngleStep = 0;
            SystemVar.VF_ElectricalAngle = 0;
            VF_Dir = 0;
            SystemVar.VF_Voltage = 0;
            RotorMacAngleTemp3 = 0;
            RotorMacAngleTemp2 = 0;
            svpwm.CurrentRef = 0;  //清除转矩限制
        }
        break;

        case MOTOR_STATE_VF_RUN : {       //开环运行
            svpwm.SvpwmControlState = SVPWM_STATE_VF_MODE;
            SystemVar.VF_ElectricalAngleStepCount++;

            if (SystemVar.VF_ElectricalAngleStepCount >= 100) {
                SystemVar.VF_ElectricalAngleStepCount = 0;
				//限定更新周期
				//开环电角度限幅&判定转动方向
                if ((SystemVar.VF_ElectricalAngleStep < SystemVar.VF_ElectricalAngleStepMax)
                        && (SystemVar.VF_ElectricalAngleStepMin == 0)) {
                    SystemVar.VF_ElectricalAngleStep = SystemVar.VF_ElectricalAngleStep + 1;
                    VF_Dir = 1;
                }

                if ((SystemVar.VF_ElectricalAngleStep > SystemVar.VF_ElectricalAngleStepMin)
                        && (SystemVar.VF_ElectricalAngleStepMax == 0)) {
                    SystemVar.VF_ElectricalAngleStep = SystemVar.VF_ElectricalAngleStep - 1;
                    VF_Dir = -1;
                }

                if ((SystemVar.VF_ElectricalAngleStepMax != 0) && (SystemVar.VF_ElectricalAngleStepMin != 0)) {
                    SystemVar.VF_ElectricalAngleStep = 0;
                    SystemVar.VF_ElectricalAngle = 0;
                    VF_Dir = 0;
                    SystemVar.VF_Voltage = 0;
                }

                if ((SystemVar.VF_ElectricalAngleStepMax == 0) && (SystemVar.VF_ElectricalAngleStepMin == 0)) {
                    SystemVar.VF_ElectricalAngleStep = 0;
                    SystemVar.VF_ElectricalAngle = 0;
                    VF_Dir = 0;
                    SystemVar.VF_Voltage = 0;
                }

                //根据实际效果，增加限幅工作。
            }

			      //更新电角度
            SystemVar.VF_ElectricalAngle = (SystemVar.VF_ElectricalAngle + ((short)((((float)SystemVar.VF_ElectricalAngleStep)) *
                                            27.31f))) & 0x7FFF;
            //计算输出电压
			      SystemVar.VF_Voltage = (SystemVar.VF_ElectricalAngleStep >> 1) * (SystemVar.VF_Coefficient * 10) + ((
                                       SystemVar.VF_Coefficient_B - 500) * VF_Dir);
        }
        break;

        case MOTOR_STATE_FIND_ZERO : { //找零点
			//计算编码器当前读取的电角度值
            RotorMacAngleTemp1 = ((int)(((MotorControler.AngleFromMT6835 + RotorMacAngleTemp2) & 0x7fff) *
                                        MotorControler.MotorPoles)) & 0x7fff;
            //开环运行电角度清零，输出电压设置为定值，进入开环运行模式来找零点
			      SystemVar.VF_ElectricalAngle = 0;
            SystemVar.VF_Voltage = 1500;
            svpwm.SvpwmControlState = SVPWM_STATE_VF_MODE;
            RotorMacAngleTemp3++;

            if (RotorMacAngleTemp3 >= 1000) {
                RotorMacAngleTemp3 = 1000;
                RotorMacAngleTemp2 = RotorMacAngleTemp2 + 1;
                RotorMacAngleTemp2 = RotorMacAngleTemp2 & 0x7fff;

				//寻找特定窗口值认定为零点
                if ((RotorMacAngleTemp1 > 8188) && (RotorMacAngleTemp1 < 8196)) {
                    MotorControler.AngleFromMT6835Offset1 = RotorMacAngleTemp2 + 25;
                    MotorControler.State = 0;
                }
            }
        }
        break;

        case MOTOR_STATE_BASIC_TEST : {              //基本测试
            svpwm.SvpwmControlState = SVPWM_STATE_BASIC_TEST;
        }
        break;

        case MOTOR_STATE_LOCK_AFTER_ENABLE : {             //使能后锁定位置
            svpwm.SvpwmControlState = SVPWM_STATE_POS_LOCK;
            pidholding.Ref = MotorControler.PositionRef;
            pidpv.OutMax = pidpv_OutMax;
            pidpv.OutMin = pidpv_OutMin;
            pidpv.UiMax = pidpv_UiMax;
            pidpv.UiMin = pidpv_UiMin;
            pidpv.Kp = 300;
            pidpv.Ki = 30;

        }
        break;

        case MOTOR_STATE_TORQUE_CONTROL: {  //力矩控制模式
            svpwm.SvpwmControlState = SVPWM_STATE_CURRENT_MODE;
            svpwm.CurrentRef = MotorControler.TorqueRef;
            pidpt.Fdb = svpwm.IQs;
            pidpt.calc(&pidpt);
        }
        break;

        case MOTOR_STATE_SPEED_CONTROL: {                     //速度环
            svpwm.SvpwmControlState = SVPWM_STATE_SPEED_MODE;
            if (pidpv.Ref < MotorControler.SpeedRef) {
                pidpv.Ref = pidpv.Ref + MotorControler.SpeedAcc;

                if (pidpv.Ref >= MotorControler.SpeedRef) {
                    pidpv.Ref = MotorControler.SpeedRef;
                }
            } else if (pidpv.Ref > MotorControler.SpeedRef) {
                pidpv.Ref = pidpv.Ref - MotorControler.SpeedDcc;

                if (pidpv.Ref <= MotorControler.SpeedRef) {
                    pidpv.Ref = MotorControler.SpeedRef;
                }
            }
			
			      pidpv.Fdb=MotorControler.SpeedFdbp;
            pidpv.calc(&pidpv);
        }
        break;

        case MOTOR_STATE_SPEED_CONTROL2: {                     //速度环，用于伺服
            pidptv.Kp = 50;
            pidptv.Ki = 50;
            pidptv.UiMax = pidpv_UiMax;
            pidptv.UiMin = pidpv_UiMin;
            pidptv.OutMax = pidpv_OutMax;
            pidptv.OutMin = pidpv_OutMin;
            svpwm.SvpwmControlState = SVPWM_STATE_SPEED_MODE2;
            svpwm.CurrentRef = MotorControler.TorqueRef;      //转矩限制

            if (pidpv.Ref < MotorControler.SpeedRef) {
                pidpv.Ref = pidpv.Ref + MotorControler.SpeedAcc;

                if (pidpv.Ref >= MotorControler.SpeedRef) {
                    pidpv.Ref = MotorControler.SpeedRef;
                }
            } else if (pidpv.Ref > MotorControler.SpeedRef) {
                pidpv.Ref = pidpv.Ref - MotorControler.SpeedDcc;

                if (pidpv.Ref <= MotorControler.SpeedRef) {
                    pidpv.Ref = MotorControler.SpeedRef;
                }
            }

            pidpv.Fdb = MotorControler.SpeedFdbp;
            pidpv.calc(&pidpv);
        }
        break;

        case MOTOR_STATE_POS_HOLD :
            svpwm.SvpwmControlState = SVPWM_STATE_POS_LOCK;
            pidholding.Ref = (int)(value.position);
            break;

        case MOTOR_STATE_POS_MOVE: {
            //pidc_position
            value = SpeedPlant_positionControl(&PostionPlanCiA402Mode_1, PostionPlanStepCount);
            //速度转换时，是由rpm转成cnt/ms，所以S曲线内部的时间单位是ms，所以这里加1而不是加0.001
            PostionPlanStepCount = PostionPlanStepCount + PostionPlanStep;
            svpwm.SvpwmControlState = SVPWM_STATE_POS_MODE;
            svpwm.CurrentRef = MotorControler.TorqueRef; //转矩限制
            pidc_position.Ref = value.position;
            pidc_position.Speedref = (short)(value.vel * 1.831f);
            pidc_position.Speedfdb = MotorControler.SpeedFdbp;
            pidc_position.Fdb = MotorControler.MotorActivePostion;
            pidc_position.Iqs = svpwm.IQs;
            pidc_position.calc(&pidc_position);
			
            //位置环输出赋值速度环Ref
					  pidpv.Kp = 300;
				    pidpv.Ki = 30;
            pidpv.Ref = 500*pidc_position.Out/pidc_position.OutMax;
              
			      pidpv.Fdb=MotorControler.SpeedFdbp;
            pidpv.calc(&pidpv);
		
           if ((PostionPlanStepCount > (PostionPlanStepMax))||((MotorControler.MotorActivePostion>UartMode.TargetPosition)&&(postion_direction_flag==1))||((MotorControler.MotorActivePostion<UartMode.TargetPosition)&&(postion_direction_flag==0))) 
				     {
					     MotorControler.State = MOTOR_STATE_POS_HOLD;
					     svpwm.SvpwmControlState = SVPWM_STATE_POS_LOCK;
					     pidholding.Ui =	0;
					     MotorControler.PositionRef = MotorControler.MotorActivePostion;
				     }
        }
        break;

        default: {
        }
        break;
        }
    }
}

//1毫秒计算一次
//1圈32767
void SpeedCalculate(void)
{
    static short t1 = 0;
    static short t2 = 0;
    static int   sum1 = 0;
    static int   sum2 = 0;
    static int   sum3 = 0;
    static short buffer1[16];
    static short buffer2[32];
    static short buffer3[64];
    static short point_a1 = 0;
    static short flag1 = 0;
    static short point_a2 = 0;
    static short flag2 = 0;
    static short point_a3 = 0;
    static short flag3 = 0;
    static short SpeedLoop = 0;
    int speedfdb_tmp = 0;

	//判断电机正反转跨圈
    if ((MotorControler.AngleFromMT6835 >> 12) == 0) {
        if (t2 == 1) {
            MotorControler.RotorCount++;
            t2 = 0;
        }

        t1 = 1;
    } else if ((MotorControler.AngleFromMT6835 >> 12) == 7) {
        if (t1 == 1) {
            MotorControler.RotorCount--;
            t1 = 0;
        }

        t2 = 1;
    } else if ((MotorControler.AngleFromMT6835 >> 12) == 3) {
        t1 = 0;
        t2 = 0;
    }
	//计算电机当前位置
    MotorControler.MotorActivePostion = (MotorControler.RotorCount * 0x7fff) + ((MotorControler.AngleFromMT6835 + 0) &
                                        0x7fff);
	//每10ms计算一次转速
    SpeedLoop++;
	
    if (SpeedLoop >= 10) {
        SpeedLoop = 0;
        speedfdb_tmp = MotorControler.AngleFromMT6835 - SystemError.RotorMacAnglePastp;
        SystemError.RotorMacAnglePastp = MotorControler.AngleFromMT6835;

		//超过限定阈值处理
        if (speedfdb_tmp > 5000) {
            speedfdb_tmp = SystemError.SpeedFdbpPost;
        }

        if (speedfdb_tmp < -5000) {
            speedfdb_tmp = SystemError.SpeedFdbpPost;
        }

		//保留上次测量速度
        SystemError.SpeedFdbpPost = speedfdb_tmp;
		//速度值换算成目标单位
        MotorControler.SpeedFdbp = (short)((speedfdb_tmp * 18311) / 10000);
        //三重滤波
		/*************************************************************/
        short temp = buffer1[point_a1];
        sum1 = sum1 + MotorControler.SpeedFdbp;
        buffer1[point_a1] = MotorControler.SpeedFdbp;
        point_a1++;

        if (flag1 == 0) {
            MotorControler.SpeedFdbpFilter1 = (short)(sum1 / point_a1);

            if (point_a1 >= 16) {
                flag1 = 1;
                point_a1 = 0;
            }
        } else {
            sum1 = sum1 - temp;
            MotorControler.SpeedFdbpFilter1 = (short)(sum1 / 16);

            if (point_a1 >= 16) {
                point_a1 = 0;
            }
        }

        /*************************************************************/
        temp = buffer2[point_a2];
        sum2 = sum2 + MotorControler.SpeedFdbp;
        buffer2[point_a2] = MotorControler.SpeedFdbp;
        point_a2++;

        if (flag2 == 0) {
            MotorControler.SpeedFdbpFilter2 = (short)(sum2 / point_a2);

            if (point_a2 >= 32) {
                flag2 = 1;
                point_a2 = 0;
            }
        } else {
            sum2 = sum2 - temp;
            MotorControler.SpeedFdbpFilter2 = (short)(sum2 / 32);

            if (point_a2 >= 32) {
                point_a2 = 0;
            }
        }

        /*************************************************************/
        temp = buffer3[point_a3];
        sum3 = sum3 + MotorControler.SpeedFdbp;
        buffer3[point_a3] = MotorControler.SpeedFdbp;
        point_a3++;

        if (flag3 == 0) {
            MotorControler.SpeedFdbpFilter3 = (short)(sum3 / point_a3);

            if (point_a3 >= 64) {
                flag3 = 1;
                point_a3 = 0;
            }
        } else {
            sum3 = sum3 - temp;
            MotorControler.SpeedFdbpFilter3 = (short)(sum3 / 64);

            if (point_a3 >= 64) {
                point_a3 = 0;
            }
        }
    }
}