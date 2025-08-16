
//    No mode change /no mode assignedģʽδ���/ģʽδ�趨
//    Profile position mode����λ�ÿ���ģʽ        pp
//    Velocity mode (�ٶȿ���ģʽ)                 vl
//    Profile position mode����λ�ÿ���ģʽ)       pv
//    Torque profile mod (����ת�ؿ���ģʽ)        tq

//   Homing mode (ԭ��ع����ģʽ)                hm
//   Interpolated position mode (�岹λ�ÿ���ģʽ)      ip
//   Cyclic synchronous position mode (����λ�ÿ���ģʽ)  csp
//   Cyclic synchronous velocity mode (�����ٶȿ���ģʽ)  csv
//   Cyclic synchronous torque mode  (����ת�ؿ���ģʽ)   cst


//   NOT READY TO SWITCH ON ��������оƬ���磬�������ڳ�ʼ�����Լ죬��������δ���ã���״̬Ϊ�ڲ�״̬��
//   SWITCH ON DISABLED ������ʼ����ɣ������������������Ա��޸ģ���״̬û�жԵ�����磬��״̬Ϊ�û��ܹ������������״̬�������ϵ���û��Ӵ�����״̬��
//   READY TO SWITCH ON �����������Ա��޸ģ���������δ���ã��ȴ����� SWITCH ON״̬��
//   SWITCH ON �������ṩ�ߵ�ѹ�����ʷŴ��������������������Ա��޸ģ���������δ���á�
//   OPERATION ENABLE û�м�⵽���ϣ������������ã����Ե���ϵ������������Ա��޸ģ����� BP[N]������ ������״̬ɲ���Ƿ���Զ��ͷš�
//   QUICK STOP ACTIVE �����������Ա��޸ģ���ͣ�������ã������������ã���������ϵ�״̬��
//   FAULT REACTION ACTIVE �����������Ա��޸ģ����������˹��ϣ����Ϸ�Ӧ�������ã���������ͣ�ã���״̬�����ֶ����룬������������ʱ�Զ����롣

//0 Rcady to switchon �ŷ��޹���
//1 Switchedon �ȴ��ŷ�ʹ��
//2 Operation enabled �ŷ�����
//3 Fault ����
//4 Voltage enabled ��ͨ����·
//5 Quick stop ����ͣ��
//6 Switch on disabled �ŷ�׼����
//7 Warning ����
//8 Manufacturer specific �����Զ��壬��
//9 Remote Զ�̿���ѡ��
//10 Target reached Ŀ��λ�û��ٶ��Ƿ񵽴�
//11 Intermal limmit active ����ڲ�λ����λ
//12~13 Operation mode specific ģʽ���
//14 Manufacturer specific
//15 Manufacturer specific

extern unsigned int Profile_acceleration;  //���ٶ�ֵ
extern unsigned int Profile_deceleration;  // ���ٶ�ֵ
extern int Velocity_sensor_actual_value;   //ʵ���ٶȴ�����ֵ
extern int Velocity_demand_value;          //�ٶ�����ֵ
extern int Velocity_actual_value;          //ʵ���ٶ�ֵ
extern short Current_actual_value;         //ʵ�ʵ���ֵ
extern int Position_actual_value;          //ʵ��λ��ֵ

extern int Pos_actual_value_enc;     //������ʵ��λ��ֵ��
extern int Position_actual_value;    //ʵ��λ��ֵ
extern int Target_velocity;          //Ŀ���ٶ�ֵ
extern signed char Modes_of_operation_display;  //��ʾģʽ 
extern signed char Modes_of_operation;          //����ģʽ
extern unsigned short  Controlword ;           //������
extern  short Targetl_torque;        //Ŀ��ת��
extern  short Torque_actual_value;   // ʵ��ת��ֵ


#include "SpeedPlan2.h"
#include "pidc.h"
#include "IQmathLib.h"
#include "ExternGlobals.h"
#include "Ds402_Slave.h"
#include "CanOpenMode.h"
/***********λ��S���߹滮*****************/
SpeedPlant SP;  // S���߹滮��ʵ��
Set_SP_Para PostionPlanCiA402Mode_1; // S���߲���
int PostionPlanStepCount = 1; // ��ǰ�滮����
int PostionPlanStepMax = 0;   // ���滮����
int PostionPlanStep = 1;      // �滮����
//SP_Error error;


enum SP_Error error;// S���߹滮�������
/****************************************/
short tmp600 = 600;    // ��ʱת������ֵ//�о����Գ�ȥ
short shifttest = 1;   // ״̬�ֲ���λ�л���־
short shifttest1 = 1;  // ״̬�ֲ���λ������
void CiA402Mode_Runing(void)
{
    // ���õ�ǰ����ģʽ��6060h��
    CanOpenMode.CiA402_RunMode = Modes_of_operation;

    if(Controlword == 0)
    {
        CanOpenMode.CanOpen_Mode1State =0; //״̬��λ
    }

    if(Controlword == 128)
    {
        SystemError.SysClearFlag = 1;   //״̬Ҫ����
        CanOpenMode.CanOpen_Mode1State = 0;  
        Target_velocity = 0;
        Targetl_torque = 0;
    }


    if(CanOpenMode.CiA402_RunMode == 1)  //�ٶ�      //��������ģʽ������Ӧ����
    {
        SystemError.RuningModeFdb = 11;
        switch(CanOpenMode.CanOpen_Mode1State)//״̬�л�
        {
            case 0 :                   //��ʹ��
            {
                CanOpenMode.CiA402_State = 0x260;
                sendOnePDOevent(&Ds402_Slave_Data, 0);

                if(Controlword == 0x06)//״̬��Ϊ0x06
                {

                    CanOpenMode.CanOpen_Mode1State = 1;//������һ״̬
                }
            }
            break;

            case 1 :                  //��ʹ��
            {
                CanOpenMode.CiA402_State = 0x231;
                sendOnePDOevent(&Ds402_Slave_Data, 0);

                CanOpenMode.CanOpen_Mode1State = 2;//������һ״̬
            }
            break;


            case 2 ://�ȴ���ʹ��
            {
                if(Controlword == 0x07)
                {

                    CanOpenMode.CanOpen_Mode1State = 3;
                    CanOpenMode.CiA402_State = 0x233;
                    sendOnePDOevent(&Ds402_Slave_Data, 0);
                }
            }
            break;

            case 3 ://�ȴ�ʹ��
            {
                if(Controlword == 0x0f)
                {

                    CanOpenMode.CanOpen_Mode1State = 4;
                    CanOpenMode.CiA402_State = 0x237;
                    sendOnePDOevent(&Ds402_Slave_Data, 0);
                }
            }
            break;

            case 4 :                   //��ʹ�ܹ���
            {

                CanOpenMode.CanOpen_Mode1State = 5;
            }
            break;

            case 5 :                  //��ʹ�����
            {
                CanOpenMode.CanOpen_Mode1State = 6;
                CanOpenMode.CiA402_State = 0x637;
                sendOnePDOevent(&Ds402_Slave_Data, 0);
            }
            break;

            case 6:   //ֻ���ٶȱ䣬û��״̬�л�
            {
                if(Controlword == 0x02)
                {
                    CanOpenMode.CanOpen_Mode1State = 7;
                }
            }
            break;

            case 7: //��ʼֹͣ
            {
                CanOpenMode.CiA402_State = 0x617;
                sendOnePDOevent(&Ds402_Slave_Data, 0);
                CanOpenMode.CanOpen_Mode1State = 8;
            }
            break;


            case 8://ֹͣ����
            {
                CanOpenMode.CanOpen_Mode1State = 9;
            }
            break;

            case 9:
            {

                CanOpenMode.CiA402_State = 0x240;
                sendOnePDOevent(&Ds402_Slave_Data, 0);
                CanOpenMode.CanOpen_Mode1State = 10;
            }
            break;

            case 10:
            {
                CanOpenMode.CanOpen_Mode1State = 0;

            }
            break;
        }
    }


    if(CanOpenMode.CiA402_RunMode == 3)  //�ٶ�
    {
        SystemError.RuningModeFdb = 13;//����
        switch(CanOpenMode.CanOpen_Mode1State)
        {
            case 0://��ʼ��
            {
                pidpv.OutMax = pidpv_OutMax;
                pidpv.OutMin = pidpv_OutMin;
                pidpv.UiMax = pidpv_UiMax;
                pidpv.UiMin = pidpv_UiMin;
                pidpv.Kp = 10000;
                pidpv.Ki = pidpv_Ki;

                pidpv.Err = 0;
                pidpv.Ref = 0;
                pidpv.Ui = 0;

                Target_velocity = 0;
                MotorControler.State = 0;

                CanOpenMode.CiA402_State = 0x260;
                sendOnePDOevent(&Ds402_Slave_Data, 0);

                if(Controlword == 0x02)         //ע��״ֵ̬���л�
                {
                    CanOpenMode.CanOpen_Mode1State = 11;
                    CanOpenMode.CiA402_State = 0x1240;
                    sendOnePDOevent(&Ds402_Slave_Data, 0);
                }
                else if(Controlword == 0x06)
                {
                    CanOpenMode.CanOpen_Mode1State = 1;
                }
                
            }
            break;

            case 1 :                  //��ʹ��
            {
                CanOpenMode.CiA402_State = 0x231;
                sendOnePDOevent(&Ds402_Slave_Data, 0);

                CanOpenMode.CanOpen_Mode1State = 2;
            }
            break;


            case 2 ://��ʹ��
            {
                if(Controlword == 0x07)
                {

                    CanOpenMode.CanOpen_Mode1State = 3;
                    CanOpenMode.CiA402_State = 0x233;
                    sendOnePDOevent(&Ds402_Slave_Data, 0);
                }
            }
            break;

            case 3 :

            {

                if(Controlword == 0x0f)
                {

                    CanOpenMode.CanOpen_Mode1State = 4;
                    CanOpenMode.CiA402_State = 0x237;
                    sendOnePDOevent(&Ds402_Slave_Data, 0);
                }
            }
            break;

            case 4 :                  //��ʹ��

            {

                CanOpenMode.CanOpen_Mode1State = 5;
            }
            break;

            case 5 :                  //��ʹ�����
            {

                CanOpenMode.CanOpen_Mode1State = 6;
                CanOpenMode.CiA402_State = 0x637;
                sendOnePDOevent(&Ds402_Slave_Data, 0);
            }
            break;


            case 11 ://����ʹ��
            {
                if(Controlword == 0x06)
                {
                    CanOpenMode.CanOpen_Mode1State = 12;
                    CanOpenMode.CiA402_State = 0x1231;
                    sendOnePDOevent(&Ds402_Slave_Data, 0);
                }
            }
            break;

            case 12 :
            {
                if(Controlword == 0x07)
                {
                    CanOpenMode.CanOpen_Mode1State = 13;
                    CanOpenMode.CiA402_State = 0x1233;
                    sendOnePDOevent(&Ds402_Slave_Data, 0);
                }
            }
            break;

            case 13 :
            {
                if(Controlword == 0x0f)
                {
                    CanOpenMode.CanOpen_Mode1State = 14;
                    CanOpenMode.CiA402_State = 0x1237;
                    sendOnePDOevent(&Ds402_Slave_Data, 0);
                }
            }
            break;

            case 14 :
            {

                CanOpenMode.CanOpen_Mode1State = 15;

            }
            break;

            case 15 :
            {

                CanOpenMode.CanOpen_Mode1State = 6;
                CanOpenMode.CiA402_State = 0x1637;
                sendOnePDOevent(&Ds402_Slave_Data, 0);

            }
            break;

            case 6:   //ֻ���ٶȱ䣬û��״̬�л�
            {
                if(Controlword == 0x02)
                {
                    CanOpenMode.CanOpen_Mode1State = 7;
                }

                MotorControler.State = 6;

                if(Target_velocity > 3000)//�ٶ�����
                {
                    Target_velocity = 3000;
                }

                if(Target_velocity < -3000)
                {
                    Target_velocity = -3000;
                }

                MotorControler.SpeedRef = Target_velocity;
                MotorControler.TorqueRef  =  tmp600;//Targetl_torque;
                MotorControler.SpeedAcc = 1;
                MotorControler.SpeedDcc = 1;

                if(MotorControler.SpeedAcc < 1) MotorControler.SpeedAcc = 1;

                if(MotorControler.SpeedAcc > 10) MotorControler.SpeedAcc = 10;

                if(MotorControler.SpeedDcc < 1) MotorControler.SpeedDcc = 1;

                if(MotorControler.SpeedDcc > 10) MotorControler.SpeedDcc = 10;
            }
            break;

            case 7://ֹͣ
            {
                CanOpenMode.CiA402_State = 0x617;
                sendOnePDOevent(&Ds402_Slave_Data, 0);
                CanOpenMode.CanOpen_Mode1State = 8;
            }
            break;

            case 8:
            {
                CanOpenMode.CanOpen_Mode1State = 9;
            }
            break;

            case 9:
            {
                CanOpenMode.CiA402_State = 0x240;
                sendOnePDOevent(&Ds402_Slave_Data, 0);
                CanOpenMode.CanOpen_Mode1State = 10;
            }
            break;

            case 10:
            {
                CanOpenMode.CanOpen_Mode1State = 0;
            }
            break;
        }
    }

    shifttest1++;

    if(shifttest1 >= 40)
    {
        shifttest = !shifttest;
        shifttest1 = 0;
    }

    if(SystemError.SysErr)
    {
        Statusword = CanOpenMode.CiA402_State + 0x8 + (shifttest << 14); //6041  ���ֹ��ϣ����ӹ��ϱ�־λ
    }
    else
    {
        Statusword = CanOpenMode.CiA402_State + (shifttest << 14); //6041
    }

    Modes_of_operation_display = Modes_of_operation;//6061
    Position_actual_value = MotorControler.MotorActivePostion;//6064
    Velocity_actual_value	=	MotorControler.SpeedFdbp;///;//0x606c; Current_actual_value
    Torque_actual_value = svpwm.IQs; //6077;
}

