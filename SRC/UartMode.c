#include "ExternGlobals.h"
#include "SpeedPlan2.h"
#include "pidc.h"
#include "IQmathLib.h"
#include "ExternGlobals.h"
#include "FLASH_WR.h"
#include "UartMode.h"
#include "Spi.h"
#include "Pidholding.h"
extern SpeedPlant SP;//S曲线规划
extern Set_SP_Para PostionPlanCiA402Mode_1;//S曲线规划参数
//SP_Error sp_Error;
enum SP_Error sp_Error;
void UartMode_Runing(void);
extern int PostionPlanStepCount;   // S曲线规划计数
extern int PostionPlanStepMax;     // S曲线规划最大步数

//位置保持中的变量
bool postion_direction_flag=1;

int Istargetposition;  //电机到达目标位置的标志位

extern short clearflg; 

short delaytime = 0;//临时变量延时计数
//UartMode.Mode
#define Stop_Mode         11
#define Position_Mode     1
#define Torque_Mode       2
#define Speed_Mode        3
#define	Openloop_Mode     4
#define Zero_Point_Mode   5
#define Calibration_Mode  6

//UartMode.CMD
#define CMD_Enable 15
#define CMD_Start 31
#define CMD_DisEnable 128
#define CMD_Stop 3
//svpwm.SvpwmControlState
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
//MotorControler.State
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


void UartMode_Runing(void)//UART模式运行主函数
{

    if(UartMode.ModePass != UartMode.Mode) //模式切换后，需要重置所以参数
    {
        UartMode.State = 0;
        UartMode.Enable = 0;
        UartMode.Enable = 0;
        UartMode.Stop = 0;
        UartMode.CMD =  CMD_DisEnable; //强制复位
    }

    UartMode.ModePass = UartMode.Mode;//记录当前模式

    if(UartMode.Enable == 1)         //使能
    {
        UartMode.CMD = CMD_Enable;
        UartMode.Enable = 0;
    }
    else if(UartMode.Start == 1)      //启动
    {
        UartMode.CMD = CMD_Start;
        UartMode.Start = 0;
    }
    else if(UartMode.DisEnable == 1)   //下使能
    {
        UartMode.CMD =  CMD_DisEnable;
        UartMode.Stop = 0;
    }

    if(UartMode.Stop == 1)           //停止
    {
        UartMode.ModePass2 = UartMode.Mode;
        UartMode.Mode = Stop_Mode;
        UartMode.State = 0;
    }

    if(UartMode.CMD == CMD_DisEnable) //清除所有错误
    {
        UartMode.State = 0; //强制清除所有错误
        UartMode.CMD = 0;
    }

    if((SystemError.SysErr) && (SystemError.SysErr != M_SYSERR_CODER))    //出现故障后，
    {
        UartMode.State = 0; //强制清除所有错误
        UartMode.CMD = 0;
    }

      if(UartMode.Mode == Stop_Mode) //停止模式
    {
        SystemError.RuningModeFdb = 211;
    
        switch(UartMode.State)
      {
            case 0 ://下使能，停止
            {
							MotorControler.State = 0; //下使能
							delay_Ms(20);
							
							
							//位置清零  
							MotorControler.MotorActivePostion = 0;    
              MotorControler.PositionRef = 0;           
              MotorControler.RotorCount = 0;  
							
							//进入位置保持
							UartMode.State = 1;
            }
            break;

            case 1://停止后，进入位置保持
            { 
							UartMode.Enable = 1;
							MotorControler.PositionRef = MotorControler.MotorActivePostion;   //更新电机位置
							pidholding.Fdb = MotorControler.MotorActivePostion;  //反馈设置为电机活动位置
							MotorControler.State = MOTOR_STATE_LOCK_AFTER_ENABLE;
							
						 if (UartMode.CMD ==  CMD_DisEnable)
							 { 
                 MotorControler.State = 0;
                 UartMode.Mode = UartMode.ModePass2;      
                 UartMode.State = 0;   							    							 
               }
            }
            break;
        }
    }


    if(UartMode.Mode == Position_Mode) //位置模式
    {
        SystemError.RuningModeFdb = 21;
        switch(UartMode.State)
        {
            case 0 ://初始化
            {
                 //位置PID控制器参数初始化
                pidc_position.OutMax = pidc_position_OutMax;
                pidc_position.OutMin = pidc_position_OutMin;
                pidc_position.UiMax = pidc_position_UiMax;
                pidc_position.UiMin = pidc_position_UiMin;
                pidc_position.Kp = pidc_position_Kp;
                pidc_position.Ki = pidc_position_Ki;							
                pidc_position.Err = 0;
							
							
                //保持PID控制器参数初始化
                pidholding.OutMax = pidholding_OutMax;
                pidholding.OutMin = pidholding_OutMin;
                pidholding.UiMax = pidholding_UiMax;
                pidholding.UiMin = pidholding_UiMin;
                pidholding.Kp = pidholding_Kp;
                pidholding.Ki = pidholding_Ki;
                pidholding.Err = 0;
				
				        pidpv.Kp = pidpv.Kp;
				        pidpv.Ki = pidpv_Ki;
				
                MotorControler.State = 0; //下使能
                pidc_position.Ui = 0;
                pidholding.Ui = 0;
								Istargetposition = 0;  //标志位设为0

                if(UartMode.CMD == CMD_Enable) //  进入上使能状态
                {
                    UartMode.State = 1;
                }
            }
            break;

            case 1 ://上使能准备
            {
                if(UartMode.CMD == CMD_Enable)
                {
                    MotorControler.PositionRef = MotorControler.MotorActivePostion; //以当前位置为基准，进入上使能
                    MotorControler.State = MOTOR_STATE_LOCK_AFTER_ENABLE;//位置保持
                    UartMode.State = 2;
                    MotorControler.TorqueRef = 900;//设置保持力矩
                }
            }
            break;

            case 2 ://位置规划准备
            {
                if(UartMode.CMD == CMD_Start) //准备进行位置规划
                {
									  (UartMode.TargetPosition>MotorControler.MotorActivePostion)?(postion_direction_flag=1):(postion_direction_flag=0);
                     //设置S曲线规划参数
                    PostionPlanCiA402Mode_1.sp = &SP;

                    PostionPlanCiA402Mode_1.accel_max = 0.008;    //postionPlan_accel_max;
                    PostionPlanCiA402Mode_1.a_accel = 0.08;       //PostionPlan_a_accel ;
                    PostionPlanCiA402Mode_1.a_decel = 0.08;       //PostionPlan_a_decel;
                    PostionPlanCiA402Mode_1.decel_max = 0.008;    //PostionPlan_decel_max;

                    //目标速度设置
                    PostionPlan_vel_tar = 100;

                    //起点和终点位置设置
                    PostionPlanCiA402Mode_1.end_position = UartMode.TargetPosition;
                    PostionPlanCiA402Mode_1.start_position = MotorControler.MotorActivePostion;//当前位置
                     //初始速度设置
                    PostionPlanCiA402Mode_1.vel_init = _IQ15(PostionPlan_vel_init);  
                    // 目标速度                    
                    PostionPlanCiA402Mode_1.vel_tar = (short)((((int)PostionPlan_vel_tar) * 10000) / 18311); //内部的位移单位是脉冲，转成脉冲速度
                    sp_Error = Set_SpeedPlant_Para(&PostionPlanCiA402Mode_1);

                   //规划错误检查
                    if(sp_Error != none_err)
                    {
                        UartMode.CMD = CMD_Enable;//返回上使能状态
                    }
                    else//规划成功进入运行状态
                    {
                        PostionPlanStepCount = 1;
                        PostionPlanStepMax = get_total_time(&PostionPlanCiA402Mode_1);
                        MotorControler.State = MOTOR_STATE_POS_MOVE;
                        UartMode.State = 3;
                        UartMode.CMD = CMD_Enable; //规划完成  启动电机
                    }
                }
            }
            break;

             case 3 :  //电机正在运行过程中
            {
							  Istargetposition = 0;   //到达目标位置标志位为1
							
                if(MotorControler.State == MOTOR_STATE_POS_HOLD) //电机到达设定位置
                {
									Istargetposition = 1;   //到达目标位置标志位为1
									UqsPid.Ui = 0;
									MotorControler.State = 0; //下使能
									delay_Ms(20);
									
                  MotorControler.PositionRef = UartMode.TargetPosition;   //更新电机位置
									delay_Ms(20);
									pidholding.Fdb = MotorControler.MotorActivePostion;  //反馈设置为电机活动位置
									//进入位置保持			
									MotorControler.State = 4; 
									UartMode.State = 4;
                }

                UartMode.CMD = CMD_Enable; //规划完成  启动电机
            }
            break;

            case 4:
            {
                 
                if(UartMode.CMD == CMD_Start) //准备进行位置规划
                {
                    UartMode.State = 2; //进行第二次规划
                }
								
            }
            break;

            default:
            {

            }
            break;
        }
    }

    if(UartMode.Mode == Torque_Mode) //转矩控制扭矩控制模式
    {
        SystemError.RuningModeFdb = 22;

        switch(UartMode.State)
        {
            case 0 :
            {
                pidpt.OutMax = pidpv_OutMax;
                pidpt.OutMin = pidpv_OutMin;
                pidpt.UiMax = pidpv_UiMax;
                pidpt.UiMin = pidpv_UiMin;
							
							//转矩控制PI
                pidpt.Kp = pidpt_Kp;
					      pidpt.Ki = pidpt_Ki;
							
//				      pidpt.Kp = 20;
//					    pidpt.Ki = 20;
							
                pidpt.Err = 0;
                MotorControler.State = 0;
                pidpt.Ref = 0;
                pidpt.Ui = 0;

                if(UartMode.CMD == CMD_Enable)////进入上使能状态
                {
                    UartMode.State = 1;
                    UartMode.CMD = 0;
                }
            }
            break;

            case 1 : //上使能状态
            {
							 MotorControler.PositionRef = MotorControler.MotorActivePostion; //以当前位置为基准，进入位置保持
               MotorControler.State = MOTOR_STATE_LOCK_AFTER_ENABLE;//位置保持  
                if(UartMode.CMD == CMD_Start)
                {
                    UartMode.State = 2; //进入转矩控制状态
                    UartMode.CMD = 0;//清除命令
                }

              
                pidpt.Ref = 0;
            }
            break;

            case 2 ://电机正在运行过程中
            {
                MotorControler.State = MOTOR_STATE_TORQUE_CONTROL;

                if(UartMode.Torque > 1500)//转矩限幅正负1500
                {
                    UartMode.Torque = 1500;
                }

                if(UartMode.Torque < -1500)
                {
                    UartMode.Torque = -1500;
                }

                 //设置转矩参考
                pidpt.Ref = UartMode.Torque;
//								UqsPid.Ref = UartMode.Torque;   //直接控制IQ
            }
            break;

            case 4:
            {
            }
            break;

            default:
            {

            }
            break;
        }
    }

    if(UartMode.Mode == Speed_Mode) //速度控制模式
    {
        SystemError.RuningModeFdb = 23; //用于上层获取当前状态

        switch(UartMode.State)
        {
            case 0 :
            {
                pidpv.Err = 0;
                MotorControler.SpeedRef = 0; //速度参考清零
                MotorControler.State = 0;     //下使能状态
                pidpv.Ref = 0;               //速度参考清零
                pidpv.Ui = 0;                //积分项清零
                //检查上使能命令
                if(UartMode.CMD == CMD_Enable)
                {
									  MotorControler.PositionRef=MotorControler.MotorActivePostion;
                    UartMode.State = 1;//进入上使能状态
                    UartMode.CMD = 0;
                }
            }
            break;

            case 1 ://上使能状态
            {
                if(UartMode.CMD == CMD_Start)
                {
									
                    UartMode.State = 2;//进入速度控制状态
                    UartMode.CMD = 0;
					          pidpv.OutMax = pidpv_OutMax;
					          pidpv.OutMin = pidpv_OutMin;
				            pidpv.UiMax = pidpv_UiMax;
					          pidpv.UiMin = pidpv_UiMin;

                }
				       
                MotorControler.State = MOTOR_STATE_LOCK_AFTER_ENABLE;
                MotorControler.SpeedRef = 0;
            }
            break;

            case 2 :  //电机正在运行过程中
            {	
             //速度环PID
					    pidpv.Kp = pidpv_Kp ;
					    pidpv.Ki = pidpv_Ki;

				if(UartMode.Speed!=0)
				{
					MotorControler.State = MOTOR_STATE_SPEED_CONTROL;
				}
				
                if(UartMode.Speed > 3000)//速度限幅正负3000
                {
                    UartMode.Speed = 3000;
                }

                if(UartMode.Speed < -3000)
                {
                    UartMode.Speed = -3000;
                }

                MotorControler.SpeedRef = UartMode.Speed;

               

				 // 设置固定转矩参考值
                MotorControler.TorqueRef = 900;
                // 设置加速度参数
                MotorControler.SpeedAcc= 1000;
                MotorControler.SpeedDcc = 1000;
            }
            break;
        }
    }

    if(UartMode.Mode == Openloop_Mode)//开环运行模式
    {
        SystemError.RuningModeFdb = 24;
        UartMode.VF_Ref = UartMode.Speed;//速度参考值

        switch(UartMode.State)
        {
            case 0 ://启动
            {
                if(UartMode.CMD == CMD_Start) //检查运行启动命令
                {
                      //正转处理
                    if(UartMode.VF_Ref > 0)
                    {
                        SystemVar.VF_ElectricalAngleStepMax = UartMode.VF_Ref; //正转 SystemVar.
                        SystemVar.VF_ElectricalAngleStepMin = 0;
                        UartMode.State = 1;
                        MotorControler.State = MOTOR_STATE_VF_RUN;//开环运行状态
                    }
                    //反转处理
                    else if(UartMode.VF_Ref < 0)
                    {
                        SystemVar.VF_ElectricalAngleStepMax = 0;
                        SystemVar.VF_ElectricalAngleStepMin = UartMode.VF_Ref; //反转
                        UartMode.State = 2;
                        MotorControler.State = MOTOR_STATE_VF_RUN;
                    }
                    //停止处理
                    else
                    {
                        SystemVar.VF_ElectricalAngleStepMax = 0;    //停止
                        SystemVar.VF_ElectricalAngleStepMin = 0;
                        UartMode.CMD = 0;
                        MotorControler.State = MOTOR_STATE_STOP;
                    }
                }
                      //停止命令处理
                if(UartMode.CMD == 0)
                {
                    SystemVar.VF_ElectricalAngleStepMax = 0;    //清零
                    SystemVar.VF_ElectricalAngleStepMin = 0;
                    MotorControler.State = MOTOR_STATE_STOP;
                }
            }
            break;

            case 1://正转运行
            {
                SystemVar.VF_ElectricalAngleStepMax = UartMode.VF_Ref; //正转

            }
            break;

            case 2: //反转运行
            {
                SystemVar.VF_ElectricalAngleStepMin = UartMode.VF_Ref; //反转
            }
            break;

            default:
            {

            }
            break;
        }
    }


    if(UartMode.Mode == Zero_Point_Mode)//零点认别 UartMode.模式
    {
        SystemError.RuningModeFdb = 25;//设置运行模式反馈码

        if(UartMode.Start == 1)
        {
            UartMode.CMD = 1;//设置启动命令
            UartMode.Start = 0;
        }

        switch(UartMode.State)
        {
            case 0 :
            {
                if(UartMode.CMD == CMD_Start)
                {
                    MotorControler.State = MOTOR_STATE_FIND_ZERO;  //零点识别状态
                    MotorControler.AngleFromMT6835Offset1 = 1;     //不能等于零，会报零点缺失错误
                    UartMode.State = 1; 
                }
            }
            break;

            case 1 ://零点识别中
            {
                //检查是否获取到零点值
                if(MotorControler.AngleFromMT6835Offset1 != 1)
                {
                    UartMode.State = 0;
                    UartMode.CMD = 0;
                    // 保存零点值到flash
                    VarDateFlash0[218] = MotorControler.AngleFromMT6835Offset1;
                    MyFLASH_WriteWord(FLASH_ADDRESS0, (uint16_t*)VarDateFlash0, 512);//写值
                }
            }
            break;

            default:
            {

            }
            break;
        }

    }


    if(UartMode.Mode ==  Calibration_Mode)//校准模式
    {
        SystemError.RuningModeFdb = 26;//反馈码

        UartMode.VF_Ref = 30; //以开环方式进行编码器校准

        switch(UartMode.State)
        {
            case 0 :
            {
                if(UartMode.CMD == CMD_Start)
                {
                    //启动正转
                    if(UartMode.VF_Ref > 0)
                    {
                        SystemVar.VF_ElectricalAngleStepMax = UartMode.VF_Ref; //正转 SystemVar.
                        SystemVar.VF_ElectricalAngleStepMin = 0;
                        UartMode.State = 1;
                        MotorControler.State = MOTOR_STATE_VF_RUN;
                    }
                    //启动反转
                    else if(UartMode.VF_Ref < 0)
                    {
                        SystemVar.VF_ElectricalAngleStepMax = 0;
                        SystemVar.VF_ElectricalAngleStepMin = UartMode.VF_Ref; //反转
                        UartMode.State = 2;
                        MotorControler.State = MOTOR_STATE_VF_RUN;
                    }
                    //停止处理
                    else
                    {
                        SystemVar.VF_ElectricalAngleStepMax = 0;    //停止
                        SystemVar.VF_ElectricalAngleStepMin = 0;
                        UartMode.CMD = 0;
                        MotorControler.State = MOTOR_STATE_STOP;
                    }
                }
                    //停止命令处理
                if(UartMode.CMD == 0)
                {
                    SystemVar.VF_ElectricalAngleStepMax = 0;    //清零
                    SystemVar.VF_ElectricalAngleStepMin = 0;
                    MotorControler.State = MOTOR_STATE_STOP;
                }
            }
            break;

            case 1://正转运行
            {
                CSn1_H;                //禁止SPI传输

                SystemVar.VF_ElectricalAngleStepMax = UartMode.VF_Ref; //正转

                delaytime++;

                if(delaytime >= 200)
                {
                    delaytime = 0;
                    UartMode.State = 5;
                }
            }
            break;

            case 2:
            {
                SystemVar.VF_ElectricalAngleStepMin = UartMode.VF_Ref; //反转
            }
            break;

            case 5 : //读取编码器状态
            {
                CSn1_L;       //使能SPI传输，在中断里失能
                Delay_Us(1);  //延时约600-700ns
                SpiReadState = 0x600E;
                MT6835_Read_Reg(0x600E);
                UartMode.State = 6;
            }
            break;

            case 6 : //延时等待
            {
                delaytime++;

                if(delaytime >= 5)
                {
                    delaytime = 0;
                    UartMode.State = 7;
                }
            }
            break;

            case 7 ://第二场读取编码器状态
            {
                CSn1_L;       //使能SPI传输，在中断里失能
                Delay_Us(1);  //延时约600-700ns
                SpiReadState = 0x600E;
                MT6835_Read_Reg(0x600E);
                UartMode.State = 8;
            }
            break;


            case 8 : //延时等待
            {
                delaytime++;
                if(delaytime >= 5)
                {
                    delaytime = 0;
                    UartMode.State = 9;
                }
            }
            break;


            case 9: //信号灯测试
            {
                GPIO_SetBits(GPIOB, GPIO_PIN_12);//设置信号灯引脚
                delaytime++;
                if(delaytime >= 10)
                {
                    delaytime = 0;
                    UartMode.State = 10;
                }
                GPIO_WriteBit(GPIOB, GPIO_PIN_12, Bit_SET);
            }
            break;

            case 10://准备读取校准结果
            {
                delaytime++;
                // 延时结束进入校准状态
                if(delaytime >= 800)
                {
                    delaytime = 0;
                    UartMode.State = 11;
                }
                CSn1_L;       //使能SPI传输，在中断里失能
                Delay_Us(1);  //延时约600-700ns
                SpiReadState = 0x3113;
                MT6835_Read_Reg(0x3113);
            }
            break;

            case 11: //等待校准完成
            {
                delaytime++;
                if(delaytime >= 100)
                {
                    delaytime = 0;
                    UartMode.State = 12;
                }
            }
            break;

            case 12://校准完成
            {
                SystemError.SysErr = 110;//？设置错误码
            }
            break;

            default:
            {

            }
            break;
        }
    }
		if(UartMode.Mode == 7) //相对位置
    {
        SystemError.RuningModeFdb = 21;

        switch(UartMode.State)
        {
            case 0 ://初始化
            {
                 //位置PID控制器参数初始化
                pidc_position.OutMax = pidc_position_OutMax;
                pidc_position.OutMin = pidc_position_OutMin;
                pidc_position.UiMax = pidc_position_UiMax;
                pidc_position.UiMin = pidc_position_UiMin;
                pidc_position.Kp = pidc_position_Kp;
                pidc_position.Ki = pidc_position_Ki;
                pidc_position.Err = 0;
                //保持PID控制器参数初始化
                pidholding.OutMax = pidholding_OutMax;
                pidholding.OutMin = pidholding_OutMin;
                pidholding.UiMax = pidholding_UiMax;
                pidholding.UiMin = pidholding_UiMin;
                pidholding.Kp = pidholding_Kp;
                pidholding.Ki = pidholding_Ki;
                pidholding.Err = 0;
                MotorControler.State = 0; //下使能
                pidc_position.Ui = 0;
                pidholding.Ui = 0;

                UartMode.State = 1;
              
            }
            break;

            case 1 ://上使能准备
            {
                
                    MotorControler.PositionRef = MotorControler.MotorActivePostion; //以汉前位置为基准 让电机上使能
                    MotorControler.State = 4;//位置保持
                    UartMode.State = 2;
                    MotorControler.TorqueRef = 900;//设置保持力矩
            }
            break;

            case 2 ://位置规划准备
            {
                     //设置S曲线规划参数
                    PostionPlanCiA402Mode_1.sp = &SP;

                    PostionPlanCiA402Mode_1.accel_max = 0.002;//ostionPlan_accel_max;
                    PostionPlanCiA402Mode_1.a_accel = 0.02;//PostionPlan_a_accel ;
                    PostionPlanCiA402Mode_1.a_decel =  0.02;//PostionPlan_a_decel;
                    PostionPlanCiA402Mode_1.decel_max =  0.002;//PostionPlan_decel_max;

                    //目标速度设置
                    PostionPlan_vel_tar = 10;

                    //正转90°
                    PostionPlanCiA402Mode_1.end_position = MotorControler.MotorActivePostion + 8192;
                    PostionPlanCiA402Mode_1.start_position = MotorControler.MotorActivePostion;   //当前位置
				
                     //初始速度设置
                    PostionPlanCiA402Mode_1.vel_init = _IQ15(PostionPlan_vel_init);  
                    // 目标速度                    
                    PostionPlanCiA402Mode_1.vel_tar = (short)((((int)PostionPlan_vel_tar) * 10000) / 18311); //内部的位移单位是脉冲，转成脉冲速度
                    sp_Error = Set_SpeedPlant_Para(&PostionPlanCiA402Mode_1);

                   //规划错误检查
                    if(sp_Error != none_err)
                    {
                        UartMode.CMD = 0xF;//返回上使能状态
                    }
                    else//规划成功进入运行状态
                    {
                        PostionPlanStepCount = 1;
                        PostionPlanStepMax = get_total_time(&PostionPlanCiA402Mode_1);
                        MotorControler.State = 71;
                        UartMode.State = 3;
                        UartMode.CMD = 0xF; //规划完成  启动电机
                    }
                
            }
            break;

            case 3 :  //电机正在运行过程中
            {
                if(MotorControler.State == 7) //电机到达设定位置
                {
                    UartMode.State = 4;
                }

                UartMode.CMD = 0xF; //规划完成  启动电机
            }
            break;

            case 4:
            {

                if(UartMode.CMD == 0x1F) //准备进行位置规划
                {
                    UartMode.State = 2; //进行第二次规划
                }

            }
            break;

            default:
            {

            }
            break;
        }
    }
		if(UartMode.Mode == 8) //相对位置
    {
        SystemError.RuningModeFdb = 21;

        switch(UartMode.State)
        {
            case 0 ://初始化
            {
                 //位置PID控制器参数初始化
                pidc_position.OutMax = pidc_position_OutMax;
                pidc_position.OutMin = pidc_position_OutMin;
                pidc_position.UiMax = pidc_position_UiMax;
                pidc_position.UiMin = pidc_position_UiMin;
                pidc_position.Kp = pidc_position_Kp;
                pidc_position.Ki = pidc_position_Ki;
                pidc_position.Err = 0;
                //保持PID控制器参数初始化
                pidholding.OutMax = pidholding_OutMax;
                pidholding.OutMin = pidholding_OutMin;
                pidholding.UiMax = pidholding_UiMax;
                pidholding.UiMin = pidholding_UiMin;
                pidholding.Kp = pidholding_Kp;
                pidholding.Ki = pidholding_Ki;
                pidholding.Err = 0;
                MotorControler.State = 0; //下使能
                pidc_position.Ui = 0;
                pidholding.Ui = 0;

                UartMode.State = 1;
              
            }
            break;

            case 1 ://上使能准备
            {
                
                    MotorControler.PositionRef = MotorControler.MotorActivePostion; //以汉前位置为基准 让电机上使能
                    MotorControler.State = 4;//位置保持
                    UartMode.State = 2;
                    MotorControler.TorqueRef = 900;//设置保持力矩
            }
            break;

            case 2 ://位置规划准备
            {
                     //设置S曲线规划参数
                    PostionPlanCiA402Mode_1.sp = &SP;

                    PostionPlanCiA402Mode_1.accel_max = 0.002; //ostionPlan_accel_max;
                    PostionPlanCiA402Mode_1.a_accel = 0.02;  //PostionPlan_a_accel ;
                    PostionPlanCiA402Mode_1.a_decel = 0.02;  //PostionPlan_a_decel;
                    PostionPlanCiA402Mode_1.decel_max = 0.002; //PostionPlan_decel_max;

                    //目标速度设置
                    PostionPlan_vel_tar = 10;

                    //正转90°
                    PostionPlanCiA402Mode_1.end_position = MotorControler.MotorActivePostion - 8192;
                    PostionPlanCiA402Mode_1.start_position = MotorControler.MotorActivePostion;   //当前位置
				
                     //初始速度设置
                    PostionPlanCiA402Mode_1.vel_init = _IQ15(PostionPlan_vel_init);  
                    // 目标速度                    
                    PostionPlanCiA402Mode_1.vel_tar = (short)((((int)PostionPlan_vel_tar) * 10000) / 18311); //内部的位移单位是脉冲，转成脉冲速度
                    sp_Error = Set_SpeedPlant_Para(&PostionPlanCiA402Mode_1);

                   //规划错误检查
                    if(sp_Error != none_err)
                    {
                        UartMode.CMD = 0xF;//返回上使能状态
                    }
                    else//规划成功进入运行状态
                    {
                        PostionPlanStepCount = 1;
                        PostionPlanStepMax = get_total_time(&PostionPlanCiA402Mode_1);
                        MotorControler.State = 71;
                        UartMode.State = 3;
                        UartMode.CMD = 0xF; //规划完成  启动电机
                    }
                
            }
            break;

            case 3 :  //电机正在运行过程中
            {
                if(MotorControler.State == 7) //电机到达设定位置
                {
                    UartMode.State = 4;
                }

                UartMode.CMD = 0xF; //规划完成  启动电机
            }
            break;

            case 4:
            {

                if(UartMode.CMD == 0x1F) //准备进行位置规划
                {
                    UartMode.State = 2; //进行第二次规划
                }

            }
            break;

            default:
            {

            }
            break;
        }
    }
}

