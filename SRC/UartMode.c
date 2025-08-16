#include "ExternGlobals.h"
#include "SpeedPlan2.h"
#include "pidc.h"
#include "IQmathLib.h"
#include "ExternGlobals.h"
#include "FLASH_WR.h"
#include "UartMode.h"
#include "Spi.h"
#include "Pidholding.h"
extern SpeedPlant SP;//S���߹滮
extern Set_SP_Para PostionPlanCiA402Mode_1;//S���߹滮����
//SP_Error sp_Error;
enum SP_Error sp_Error;
void UartMode_Runing(void);
extern int PostionPlanStepCount;   // S���߹滮����
extern int PostionPlanStepMax;     // S���߹滮�����

//λ�ñ����еı���
bool postion_direction_flag=1;

int Istargetposition;  //�������Ŀ��λ�õı�־λ

extern short clearflg; 

short delaytime = 0;//��ʱ������ʱ����
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
/************************ SVPWM����״̬�궨�� ************************/
#define SVPWM_STATE_DISABLE           0   // �ر�״̬����ʼ������������
#define SVPWM_STATE_VF_MODE           1   // V/F����ģʽ
#define SVPWM_STATE_FIND_ZERO         2   // Ѱ�����
#define SVPWM_STATE_BASIC_TEST        3   // �������ܲ���ģʽ
#define SVPWM_STATE_POS_LOCK          4   // ʹ�ܺ�λ������ģʽ
#define SVPWM_STATE_CURRENT_MODE      5   // ��������ģʽ
#define SVPWM_STATE_SPEED_MODE        6   // �ٶȿ���ģʽ����Ӧ��λ���ٶȿ��ƣ�
#define SVPWM_STATE_SPEED_MODE2       61  // �ٶȿ���ģʽ2��δʹ�ã�
#define SVPWM_STATE_POS_MODE          7   // λ�ñջ�����ģʽ
#define SVPWM_STATE_POS_LOCK_TEST     10  // λ����������ģʽ���������ã�
//MotorControler.State
/************************ �������״̬�궨�� ************************/
#define MOTOR_STATE_STOP              0   // ���ֹͣ״̬����ʼ��������
#define MOTOR_STATE_VF_RUN            1   // V/Fģʽ����
#define MOTOR_STATE_FIND_ZERO         2   // Ѱ����� 
#define MOTOR_STATE_BASIC_TEST        3   // �������ܲ���
#define MOTOR_STATE_LOCK_AFTER_ENABLE 4   // ʹ�ܺ��λ������
#define MOTOR_STATE_TORQUE_CONTROL    5   // ת�ؿ���ģʽ
#define MOTOR_STATE_SPEED_CONTROL     6   // �ٶȱջ�����
#define MOTOR_STATE_SPEED_CONTROL2    61  // �ٶȱջ�����2�����ã�
#define MOTOR_STATE_POS_HOLD          7   // λ�ñ���ģʽ
#define MOTOR_STATE_POS_MOVE          71  // λ�ù滮�˶�ģʽ��ִ��λ�ù켣��


void UartMode_Runing(void)//UARTģʽ����������
{

    if(UartMode.ModePass != UartMode.Mode) //ģʽ�л�����Ҫ�������Բ���
    {
        UartMode.State = 0;
        UartMode.Enable = 0;
        UartMode.Enable = 0;
        UartMode.Stop = 0;
        UartMode.CMD =  CMD_DisEnable; //ǿ�Ƹ�λ
    }

    UartMode.ModePass = UartMode.Mode;//��¼��ǰģʽ

    if(UartMode.Enable == 1)         //ʹ��
    {
        UartMode.CMD = CMD_Enable;
        UartMode.Enable = 0;
    }
    else if(UartMode.Start == 1)      //����
    {
        UartMode.CMD = CMD_Start;
        UartMode.Start = 0;
    }
    else if(UartMode.DisEnable == 1)   //��ʹ��
    {
        UartMode.CMD =  CMD_DisEnable;
        UartMode.Stop = 0;
    }

    if(UartMode.Stop == 1)           //ֹͣ
    {
        UartMode.ModePass2 = UartMode.Mode;
        UartMode.Mode = Stop_Mode;
        UartMode.State = 0;
    }

    if(UartMode.CMD == CMD_DisEnable) //������д���
    {
        UartMode.State = 0; //ǿ��������д���
        UartMode.CMD = 0;
    }

    if((SystemError.SysErr) && (SystemError.SysErr != M_SYSERR_CODER))    //���ֹ��Ϻ�
    {
        UartMode.State = 0; //ǿ��������д���
        UartMode.CMD = 0;
    }

      if(UartMode.Mode == Stop_Mode) //ֹͣģʽ
    {
        SystemError.RuningModeFdb = 211;
    
        switch(UartMode.State)
      {
            case 0 ://��ʹ�ܣ�ֹͣ
            {
							MotorControler.State = 0; //��ʹ��
							delay_Ms(20);
							
							
							//λ������  
							MotorControler.MotorActivePostion = 0;    
              MotorControler.PositionRef = 0;           
              MotorControler.RotorCount = 0;  
							
							//����λ�ñ���
							UartMode.State = 1;
            }
            break;

            case 1://ֹͣ�󣬽���λ�ñ���
            { 
							UartMode.Enable = 1;
							MotorControler.PositionRef = MotorControler.MotorActivePostion;   //���µ��λ��
							pidholding.Fdb = MotorControler.MotorActivePostion;  //��������Ϊ����λ��
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


    if(UartMode.Mode == Position_Mode) //λ��ģʽ
    {
        SystemError.RuningModeFdb = 21;
        switch(UartMode.State)
        {
            case 0 ://��ʼ��
            {
                 //λ��PID������������ʼ��
                pidc_position.OutMax = pidc_position_OutMax;
                pidc_position.OutMin = pidc_position_OutMin;
                pidc_position.UiMax = pidc_position_UiMax;
                pidc_position.UiMin = pidc_position_UiMin;
                pidc_position.Kp = pidc_position_Kp;
                pidc_position.Ki = pidc_position_Ki;							
                pidc_position.Err = 0;
							
							
                //����PID������������ʼ��
                pidholding.OutMax = pidholding_OutMax;
                pidholding.OutMin = pidholding_OutMin;
                pidholding.UiMax = pidholding_UiMax;
                pidholding.UiMin = pidholding_UiMin;
                pidholding.Kp = pidholding_Kp;
                pidholding.Ki = pidholding_Ki;
                pidholding.Err = 0;
				
				        pidpv.Kp = pidpv.Kp;
				        pidpv.Ki = pidpv_Ki;
				
                MotorControler.State = 0; //��ʹ��
                pidc_position.Ui = 0;
                pidholding.Ui = 0;
								Istargetposition = 0;  //��־λ��Ϊ0

                if(UartMode.CMD == CMD_Enable) //  ������ʹ��״̬
                {
                    UartMode.State = 1;
                }
            }
            break;

            case 1 ://��ʹ��׼��
            {
                if(UartMode.CMD == CMD_Enable)
                {
                    MotorControler.PositionRef = MotorControler.MotorActivePostion; //�Ե�ǰλ��Ϊ��׼��������ʹ��
                    MotorControler.State = MOTOR_STATE_LOCK_AFTER_ENABLE;//λ�ñ���
                    UartMode.State = 2;
                    MotorControler.TorqueRef = 900;//���ñ�������
                }
            }
            break;

            case 2 ://λ�ù滮׼��
            {
                if(UartMode.CMD == CMD_Start) //׼������λ�ù滮
                {
									  (UartMode.TargetPosition>MotorControler.MotorActivePostion)?(postion_direction_flag=1):(postion_direction_flag=0);
                     //����S���߹滮����
                    PostionPlanCiA402Mode_1.sp = &SP;

                    PostionPlanCiA402Mode_1.accel_max = 0.008;    //postionPlan_accel_max;
                    PostionPlanCiA402Mode_1.a_accel = 0.08;       //PostionPlan_a_accel ;
                    PostionPlanCiA402Mode_1.a_decel = 0.08;       //PostionPlan_a_decel;
                    PostionPlanCiA402Mode_1.decel_max = 0.008;    //PostionPlan_decel_max;

                    //Ŀ���ٶ�����
                    PostionPlan_vel_tar = 100;

                    //�����յ�λ������
                    PostionPlanCiA402Mode_1.end_position = UartMode.TargetPosition;
                    PostionPlanCiA402Mode_1.start_position = MotorControler.MotorActivePostion;//��ǰλ��
                     //��ʼ�ٶ�����
                    PostionPlanCiA402Mode_1.vel_init = _IQ15(PostionPlan_vel_init);  
                    // Ŀ���ٶ�                    
                    PostionPlanCiA402Mode_1.vel_tar = (short)((((int)PostionPlan_vel_tar) * 10000) / 18311); //�ڲ���λ�Ƶ�λ�����壬ת�������ٶ�
                    sp_Error = Set_SpeedPlant_Para(&PostionPlanCiA402Mode_1);

                   //�滮������
                    if(sp_Error != none_err)
                    {
                        UartMode.CMD = CMD_Enable;//������ʹ��״̬
                    }
                    else//�滮�ɹ���������״̬
                    {
                        PostionPlanStepCount = 1;
                        PostionPlanStepMax = get_total_time(&PostionPlanCiA402Mode_1);
                        MotorControler.State = MOTOR_STATE_POS_MOVE;
                        UartMode.State = 3;
                        UartMode.CMD = CMD_Enable; //�滮���  �������
                    }
                }
            }
            break;

             case 3 :  //����������й�����
            {
							  Istargetposition = 0;   //����Ŀ��λ�ñ�־λΪ1
							
                if(MotorControler.State == MOTOR_STATE_POS_HOLD) //��������趨λ��
                {
									Istargetposition = 1;   //����Ŀ��λ�ñ�־λΪ1
									UqsPid.Ui = 0;
									MotorControler.State = 0; //��ʹ��
									delay_Ms(20);
									
                  MotorControler.PositionRef = UartMode.TargetPosition;   //���µ��λ��
									delay_Ms(20);
									pidholding.Fdb = MotorControler.MotorActivePostion;  //��������Ϊ����λ��
									//����λ�ñ���			
									MotorControler.State = 4; 
									UartMode.State = 4;
                }

                UartMode.CMD = CMD_Enable; //�滮���  �������
            }
            break;

            case 4:
            {
                 
                if(UartMode.CMD == CMD_Start) //׼������λ�ù滮
                {
                    UartMode.State = 2; //���еڶ��ι滮
                }
								
            }
            break;

            default:
            {

            }
            break;
        }
    }

    if(UartMode.Mode == Torque_Mode) //ת�ؿ���Ť�ؿ���ģʽ
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
							
							//ת�ؿ���PI
                pidpt.Kp = pidpt_Kp;
					      pidpt.Ki = pidpt_Ki;
							
//				      pidpt.Kp = 20;
//					    pidpt.Ki = 20;
							
                pidpt.Err = 0;
                MotorControler.State = 0;
                pidpt.Ref = 0;
                pidpt.Ui = 0;

                if(UartMode.CMD == CMD_Enable)////������ʹ��״̬
                {
                    UartMode.State = 1;
                    UartMode.CMD = 0;
                }
            }
            break;

            case 1 : //��ʹ��״̬
            {
							 MotorControler.PositionRef = MotorControler.MotorActivePostion; //�Ե�ǰλ��Ϊ��׼������λ�ñ���
               MotorControler.State = MOTOR_STATE_LOCK_AFTER_ENABLE;//λ�ñ���  
                if(UartMode.CMD == CMD_Start)
                {
                    UartMode.State = 2; //����ת�ؿ���״̬
                    UartMode.CMD = 0;//�������
                }

              
                pidpt.Ref = 0;
            }
            break;

            case 2 ://����������й�����
            {
                MotorControler.State = MOTOR_STATE_TORQUE_CONTROL;

                if(UartMode.Torque > 1500)//ת���޷�����1500
                {
                    UartMode.Torque = 1500;
                }

                if(UartMode.Torque < -1500)
                {
                    UartMode.Torque = -1500;
                }

                 //����ת�زο�
                pidpt.Ref = UartMode.Torque;
//								UqsPid.Ref = UartMode.Torque;   //ֱ�ӿ���IQ
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

    if(UartMode.Mode == Speed_Mode) //�ٶȿ���ģʽ
    {
        SystemError.RuningModeFdb = 23; //�����ϲ��ȡ��ǰ״̬

        switch(UartMode.State)
        {
            case 0 :
            {
                pidpv.Err = 0;
                MotorControler.SpeedRef = 0; //�ٶȲο�����
                MotorControler.State = 0;     //��ʹ��״̬
                pidpv.Ref = 0;               //�ٶȲο�����
                pidpv.Ui = 0;                //����������
                //�����ʹ������
                if(UartMode.CMD == CMD_Enable)
                {
									  MotorControler.PositionRef=MotorControler.MotorActivePostion;
                    UartMode.State = 1;//������ʹ��״̬
                    UartMode.CMD = 0;
                }
            }
            break;

            case 1 ://��ʹ��״̬
            {
                if(UartMode.CMD == CMD_Start)
                {
									
                    UartMode.State = 2;//�����ٶȿ���״̬
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

            case 2 :  //����������й�����
            {	
             //�ٶȻ�PID
					    pidpv.Kp = pidpv_Kp ;
					    pidpv.Ki = pidpv_Ki;

				if(UartMode.Speed!=0)
				{
					MotorControler.State = MOTOR_STATE_SPEED_CONTROL;
				}
				
                if(UartMode.Speed > 3000)//�ٶ��޷�����3000
                {
                    UartMode.Speed = 3000;
                }

                if(UartMode.Speed < -3000)
                {
                    UartMode.Speed = -3000;
                }

                MotorControler.SpeedRef = UartMode.Speed;

               

				 // ���ù̶�ת�زο�ֵ
                MotorControler.TorqueRef = 900;
                // ���ü��ٶȲ���
                MotorControler.SpeedAcc= 1000;
                MotorControler.SpeedDcc = 1000;
            }
            break;
        }
    }

    if(UartMode.Mode == Openloop_Mode)//��������ģʽ
    {
        SystemError.RuningModeFdb = 24;
        UartMode.VF_Ref = UartMode.Speed;//�ٶȲο�ֵ

        switch(UartMode.State)
        {
            case 0 ://����
            {
                if(UartMode.CMD == CMD_Start) //���������������
                {
                      //��ת����
                    if(UartMode.VF_Ref > 0)
                    {
                        SystemVar.VF_ElectricalAngleStepMax = UartMode.VF_Ref; //��ת SystemVar.
                        SystemVar.VF_ElectricalAngleStepMin = 0;
                        UartMode.State = 1;
                        MotorControler.State = MOTOR_STATE_VF_RUN;//��������״̬
                    }
                    //��ת����
                    else if(UartMode.VF_Ref < 0)
                    {
                        SystemVar.VF_ElectricalAngleStepMax = 0;
                        SystemVar.VF_ElectricalAngleStepMin = UartMode.VF_Ref; //��ת
                        UartMode.State = 2;
                        MotorControler.State = MOTOR_STATE_VF_RUN;
                    }
                    //ֹͣ����
                    else
                    {
                        SystemVar.VF_ElectricalAngleStepMax = 0;    //ֹͣ
                        SystemVar.VF_ElectricalAngleStepMin = 0;
                        UartMode.CMD = 0;
                        MotorControler.State = MOTOR_STATE_STOP;
                    }
                }
                      //ֹͣ�����
                if(UartMode.CMD == 0)
                {
                    SystemVar.VF_ElectricalAngleStepMax = 0;    //����
                    SystemVar.VF_ElectricalAngleStepMin = 0;
                    MotorControler.State = MOTOR_STATE_STOP;
                }
            }
            break;

            case 1://��ת����
            {
                SystemVar.VF_ElectricalAngleStepMax = UartMode.VF_Ref; //��ת

            }
            break;

            case 2: //��ת����
            {
                SystemVar.VF_ElectricalAngleStepMin = UartMode.VF_Ref; //��ת
            }
            break;

            default:
            {

            }
            break;
        }
    }


    if(UartMode.Mode == Zero_Point_Mode)//����ϱ� UartMode.ģʽ
    {
        SystemError.RuningModeFdb = 25;//��������ģʽ������

        if(UartMode.Start == 1)
        {
            UartMode.CMD = 1;//������������
            UartMode.Start = 0;
        }

        switch(UartMode.State)
        {
            case 0 :
            {
                if(UartMode.CMD == CMD_Start)
                {
                    MotorControler.State = MOTOR_STATE_FIND_ZERO;  //���ʶ��״̬
                    MotorControler.AngleFromMT6835Offset1 = 1;     //���ܵ����㣬�ᱨ���ȱʧ����
                    UartMode.State = 1; 
                }
            }
            break;

            case 1 ://���ʶ����
            {
                //����Ƿ��ȡ�����ֵ
                if(MotorControler.AngleFromMT6835Offset1 != 1)
                {
                    UartMode.State = 0;
                    UartMode.CMD = 0;
                    // �������ֵ��flash
                    VarDateFlash0[218] = MotorControler.AngleFromMT6835Offset1;
                    MyFLASH_WriteWord(FLASH_ADDRESS0, (uint16_t*)VarDateFlash0, 512);//дֵ
                }
            }
            break;

            default:
            {

            }
            break;
        }

    }


    if(UartMode.Mode ==  Calibration_Mode)//У׼ģʽ
    {
        SystemError.RuningModeFdb = 26;//������

        UartMode.VF_Ref = 30; //�Կ�����ʽ���б�����У׼

        switch(UartMode.State)
        {
            case 0 :
            {
                if(UartMode.CMD == CMD_Start)
                {
                    //������ת
                    if(UartMode.VF_Ref > 0)
                    {
                        SystemVar.VF_ElectricalAngleStepMax = UartMode.VF_Ref; //��ת SystemVar.
                        SystemVar.VF_ElectricalAngleStepMin = 0;
                        UartMode.State = 1;
                        MotorControler.State = MOTOR_STATE_VF_RUN;
                    }
                    //������ת
                    else if(UartMode.VF_Ref < 0)
                    {
                        SystemVar.VF_ElectricalAngleStepMax = 0;
                        SystemVar.VF_ElectricalAngleStepMin = UartMode.VF_Ref; //��ת
                        UartMode.State = 2;
                        MotorControler.State = MOTOR_STATE_VF_RUN;
                    }
                    //ֹͣ����
                    else
                    {
                        SystemVar.VF_ElectricalAngleStepMax = 0;    //ֹͣ
                        SystemVar.VF_ElectricalAngleStepMin = 0;
                        UartMode.CMD = 0;
                        MotorControler.State = MOTOR_STATE_STOP;
                    }
                }
                    //ֹͣ�����
                if(UartMode.CMD == 0)
                {
                    SystemVar.VF_ElectricalAngleStepMax = 0;    //����
                    SystemVar.VF_ElectricalAngleStepMin = 0;
                    MotorControler.State = MOTOR_STATE_STOP;
                }
            }
            break;

            case 1://��ת����
            {
                CSn1_H;                //��ֹSPI����

                SystemVar.VF_ElectricalAngleStepMax = UartMode.VF_Ref; //��ת

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
                SystemVar.VF_ElectricalAngleStepMin = UartMode.VF_Ref; //��ת
            }
            break;

            case 5 : //��ȡ������״̬
            {
                CSn1_L;       //ʹ��SPI���䣬���ж���ʧ��
                Delay_Us(1);  //��ʱԼ600-700ns
                SpiReadState = 0x600E;
                MT6835_Read_Reg(0x600E);
                UartMode.State = 6;
            }
            break;

            case 6 : //��ʱ�ȴ�
            {
                delaytime++;

                if(delaytime >= 5)
                {
                    delaytime = 0;
                    UartMode.State = 7;
                }
            }
            break;

            case 7 ://�ڶ�����ȡ������״̬
            {
                CSn1_L;       //ʹ��SPI���䣬���ж���ʧ��
                Delay_Us(1);  //��ʱԼ600-700ns
                SpiReadState = 0x600E;
                MT6835_Read_Reg(0x600E);
                UartMode.State = 8;
            }
            break;


            case 8 : //��ʱ�ȴ�
            {
                delaytime++;
                if(delaytime >= 5)
                {
                    delaytime = 0;
                    UartMode.State = 9;
                }
            }
            break;


            case 9: //�źŵƲ���
            {
                GPIO_SetBits(GPIOB, GPIO_PIN_12);//�����źŵ�����
                delaytime++;
                if(delaytime >= 10)
                {
                    delaytime = 0;
                    UartMode.State = 10;
                }
                GPIO_WriteBit(GPIOB, GPIO_PIN_12, Bit_SET);
            }
            break;

            case 10://׼����ȡУ׼���
            {
                delaytime++;
                // ��ʱ��������У׼״̬
                if(delaytime >= 800)
                {
                    delaytime = 0;
                    UartMode.State = 11;
                }
                CSn1_L;       //ʹ��SPI���䣬���ж���ʧ��
                Delay_Us(1);  //��ʱԼ600-700ns
                SpiReadState = 0x3113;
                MT6835_Read_Reg(0x3113);
            }
            break;

            case 11: //�ȴ�У׼���
            {
                delaytime++;
                if(delaytime >= 100)
                {
                    delaytime = 0;
                    UartMode.State = 12;
                }
            }
            break;

            case 12://У׼���
            {
                SystemError.SysErr = 110;//�����ô�����
            }
            break;

            default:
            {

            }
            break;
        }
    }
		if(UartMode.Mode == 7) //���λ��
    {
        SystemError.RuningModeFdb = 21;

        switch(UartMode.State)
        {
            case 0 ://��ʼ��
            {
                 //λ��PID������������ʼ��
                pidc_position.OutMax = pidc_position_OutMax;
                pidc_position.OutMin = pidc_position_OutMin;
                pidc_position.UiMax = pidc_position_UiMax;
                pidc_position.UiMin = pidc_position_UiMin;
                pidc_position.Kp = pidc_position_Kp;
                pidc_position.Ki = pidc_position_Ki;
                pidc_position.Err = 0;
                //����PID������������ʼ��
                pidholding.OutMax = pidholding_OutMax;
                pidholding.OutMin = pidholding_OutMin;
                pidholding.UiMax = pidholding_UiMax;
                pidholding.UiMin = pidholding_UiMin;
                pidholding.Kp = pidholding_Kp;
                pidholding.Ki = pidholding_Ki;
                pidholding.Err = 0;
                MotorControler.State = 0; //��ʹ��
                pidc_position.Ui = 0;
                pidholding.Ui = 0;

                UartMode.State = 1;
              
            }
            break;

            case 1 ://��ʹ��׼��
            {
                
                    MotorControler.PositionRef = MotorControler.MotorActivePostion; //�Ժ�ǰλ��Ϊ��׼ �õ����ʹ��
                    MotorControler.State = 4;//λ�ñ���
                    UartMode.State = 2;
                    MotorControler.TorqueRef = 900;//���ñ�������
            }
            break;

            case 2 ://λ�ù滮׼��
            {
                     //����S���߹滮����
                    PostionPlanCiA402Mode_1.sp = &SP;

                    PostionPlanCiA402Mode_1.accel_max = 0.002;//ostionPlan_accel_max;
                    PostionPlanCiA402Mode_1.a_accel = 0.02;//PostionPlan_a_accel ;
                    PostionPlanCiA402Mode_1.a_decel =  0.02;//PostionPlan_a_decel;
                    PostionPlanCiA402Mode_1.decel_max =  0.002;//PostionPlan_decel_max;

                    //Ŀ���ٶ�����
                    PostionPlan_vel_tar = 10;

                    //��ת90��
                    PostionPlanCiA402Mode_1.end_position = MotorControler.MotorActivePostion + 8192;
                    PostionPlanCiA402Mode_1.start_position = MotorControler.MotorActivePostion;   //��ǰλ��
				
                     //��ʼ�ٶ�����
                    PostionPlanCiA402Mode_1.vel_init = _IQ15(PostionPlan_vel_init);  
                    // Ŀ���ٶ�                    
                    PostionPlanCiA402Mode_1.vel_tar = (short)((((int)PostionPlan_vel_tar) * 10000) / 18311); //�ڲ���λ�Ƶ�λ�����壬ת�������ٶ�
                    sp_Error = Set_SpeedPlant_Para(&PostionPlanCiA402Mode_1);

                   //�滮������
                    if(sp_Error != none_err)
                    {
                        UartMode.CMD = 0xF;//������ʹ��״̬
                    }
                    else//�滮�ɹ���������״̬
                    {
                        PostionPlanStepCount = 1;
                        PostionPlanStepMax = get_total_time(&PostionPlanCiA402Mode_1);
                        MotorControler.State = 71;
                        UartMode.State = 3;
                        UartMode.CMD = 0xF; //�滮���  �������
                    }
                
            }
            break;

            case 3 :  //����������й�����
            {
                if(MotorControler.State == 7) //��������趨λ��
                {
                    UartMode.State = 4;
                }

                UartMode.CMD = 0xF; //�滮���  �������
            }
            break;

            case 4:
            {

                if(UartMode.CMD == 0x1F) //׼������λ�ù滮
                {
                    UartMode.State = 2; //���еڶ��ι滮
                }

            }
            break;

            default:
            {

            }
            break;
        }
    }
		if(UartMode.Mode == 8) //���λ��
    {
        SystemError.RuningModeFdb = 21;

        switch(UartMode.State)
        {
            case 0 ://��ʼ��
            {
                 //λ��PID������������ʼ��
                pidc_position.OutMax = pidc_position_OutMax;
                pidc_position.OutMin = pidc_position_OutMin;
                pidc_position.UiMax = pidc_position_UiMax;
                pidc_position.UiMin = pidc_position_UiMin;
                pidc_position.Kp = pidc_position_Kp;
                pidc_position.Ki = pidc_position_Ki;
                pidc_position.Err = 0;
                //����PID������������ʼ��
                pidholding.OutMax = pidholding_OutMax;
                pidholding.OutMin = pidholding_OutMin;
                pidholding.UiMax = pidholding_UiMax;
                pidholding.UiMin = pidholding_UiMin;
                pidholding.Kp = pidholding_Kp;
                pidholding.Ki = pidholding_Ki;
                pidholding.Err = 0;
                MotorControler.State = 0; //��ʹ��
                pidc_position.Ui = 0;
                pidholding.Ui = 0;

                UartMode.State = 1;
              
            }
            break;

            case 1 ://��ʹ��׼��
            {
                
                    MotorControler.PositionRef = MotorControler.MotorActivePostion; //�Ժ�ǰλ��Ϊ��׼ �õ����ʹ��
                    MotorControler.State = 4;//λ�ñ���
                    UartMode.State = 2;
                    MotorControler.TorqueRef = 900;//���ñ�������
            }
            break;

            case 2 ://λ�ù滮׼��
            {
                     //����S���߹滮����
                    PostionPlanCiA402Mode_1.sp = &SP;

                    PostionPlanCiA402Mode_1.accel_max = 0.002; //ostionPlan_accel_max;
                    PostionPlanCiA402Mode_1.a_accel = 0.02;  //PostionPlan_a_accel ;
                    PostionPlanCiA402Mode_1.a_decel = 0.02;  //PostionPlan_a_decel;
                    PostionPlanCiA402Mode_1.decel_max = 0.002; //PostionPlan_decel_max;

                    //Ŀ���ٶ�����
                    PostionPlan_vel_tar = 10;

                    //��ת90��
                    PostionPlanCiA402Mode_1.end_position = MotorControler.MotorActivePostion - 8192;
                    PostionPlanCiA402Mode_1.start_position = MotorControler.MotorActivePostion;   //��ǰλ��
				
                     //��ʼ�ٶ�����
                    PostionPlanCiA402Mode_1.vel_init = _IQ15(PostionPlan_vel_init);  
                    // Ŀ���ٶ�                    
                    PostionPlanCiA402Mode_1.vel_tar = (short)((((int)PostionPlan_vel_tar) * 10000) / 18311); //�ڲ���λ�Ƶ�λ�����壬ת�������ٶ�
                    sp_Error = Set_SpeedPlant_Para(&PostionPlanCiA402Mode_1);

                   //�滮������
                    if(sp_Error != none_err)
                    {
                        UartMode.CMD = 0xF;//������ʹ��״̬
                    }
                    else//�滮�ɹ���������״̬
                    {
                        PostionPlanStepCount = 1;
                        PostionPlanStepMax = get_total_time(&PostionPlanCiA402Mode_1);
                        MotorControler.State = 71;
                        UartMode.State = 3;
                        UartMode.CMD = 0xF; //�滮���  �������
                    }
                
            }
            break;

            case 3 :  //����������й�����
            {
                if(MotorControler.State == 7) //��������趨λ��
                {
                    UartMode.State = 4;
                }

                UartMode.CMD = 0xF; //�滮���  �������
            }
            break;

            case 4:
            {

                if(UartMode.CMD == 0x1F) //׼������λ�ù滮
                {
                    UartMode.State = 2; //���еڶ��ι滮
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

