/*ת��PID�Ĺ����Ǹ��ݵ���Ĺ���״̬�����١����١����١������л�����̬�������Ʋ�����ʵ�ָ���׼��ת�ؿ���
 �������̣� 1.����������
            2.״̬�жϺ�ģʽѡ��
						3.�����Ʋ���
						4.����ģʽPID����
*/
#include "pidtorque.h"
#include "ExternGlobals.h"
//ȫ�ֱ�������
short integralAdjustFactor = 5;  //����ϵ����������
short static errorThreshold = 0; //�����ʱ����
short Normal_Mode_Flag = 0;      //����ģʽ��־λ
short v1_ValueMax, v2_ValueMax;  //������洢����

extern void FindTable(short speed, short torquecoeff, short* result1, short* result2);
void pidtorque_calc(PIDTorque *v)
{
	  //����PID����
    v->Err = v->Ref - v->Fdb;                //�������
    v->Up = ((int)v->Kp * (int)v->Err) >> 2; //��������Kp*��������λ�൱��/4��

    // �������ο�����Ϊ800
    // ���Ť������ĵ�ѹΪ750*4
    errorThreshold = SystemError.ThresholdCoeff * 3;
    
	  //��ʼ������ģʽ��־λ
    Normal_Mode_Flag = 0;
	
//�жϷ���״̬
    if((v->Ref > 0) && (v->Fdb > 0))  //ͬ���� �Ӽ���
    {
        if(v->Err > errorThreshold)  //����  3000 -500 =2500 >1000
        {   //���ٶ�  �����ķ�ʽ  ���в���ȡ����������
            FindTable(v->Fdb, SystemError.TorquePIDLimitCoeff, &v1_ValueMax, &v2_ValueMax);  
					  //����������������ֵ
            if(v->Out < v2_ValueMax)
            {
                v->Ui = ((int)(v1_ValueMax) << 15); //���û������ʼֵ
                v->Out = v2_ValueMax;               //�������
            }
            else  //�ظ��أ�������ͨ��PID
            {
                Normal_Mode_Flag = 1;
            }
        }
        else if(v->Err < (0 - errorThreshold)) //����  500- 3000 = -2500 <-1000
        {
            FindTable(v->Fdb, 0 - SystemError.TorquePIDLimitCoeff, &v1_ValueMax, &v2_ValueMax);  //���ٶ�  �����ķ�ʽ  ���в��
            //���������������Сֵ
            if(v->Out > v2_ValueMax)
            {  
                v->Ui = ((int)(v1_ValueMax) << 15);
                v->Out = v2_ValueMax;
            }
            else  //�ظ��أ�������ͨ��PID
            {
                Normal_Mode_Flag = 1;
            }
        }
        else   //����
        {
            Normal_Mode_Flag = 1;
        }
    }
    else if((v->Ref < 0) && (v->Fdb < 0))  //ͬ�� �Ӽ���
    {
        if(v->Err < 0 - errorThreshold) //����   -3000 - -500 =-2500 <-1000
        {
            FindTable(v->Fdb, SystemError.TorquePIDLimitCoeff, &v1_ValueMax, &v2_ValueMax);  //���ٶ�  �����ķ�ʽ  ���в��

            if(v->Out > 0 - v2_ValueMax)
            {
                v->Ui = 0 - ((int)(v1_ValueMax) << 15);
                v->Out = 0 - v2_ValueMax;
            }
            else  //�ظ��أ�������ͨ��PID
            {
                Normal_Mode_Flag = 1;
            }
        }
        else if(v->Err > errorThreshold)  //����  -500 - -300  =2500 >1000
        {
            FindTable(v->Fdb, 0 - SystemError.TorquePIDLimitCoeff, &v1_ValueMax, &v2_ValueMax);  //���ٶ�  �����ķ�ʽ  ���в��

            if(v->Out < 0 - v2_ValueMax)
            {
                v->Ui = 0 - ((int)(v1_ValueMax) << 15);
                v->Out = 0 - v2_ValueMax;
            }
            else  //�ظ��أ�������ͨ��PID
            {
                Normal_Mode_Flag = 1;
            }
        }
        else   //����
        {
            Normal_Mode_Flag = 1;
        }
    }
    else if((v->Ref < 0) && (v->Fdb > 0))  //���������
    {
        if(v->Err < (0 - errorThreshold)) //���� -500 - 2000 =-2500  <-1000
        {

            FindTable(v->Fdb, 0 - SystemError.TorquePIDLimitCoeff, &v1_ValueMax, &v2_ValueMax);  //���ٶ�  �����ķ�ʽ  ���в��

            if(v->Out > v2_ValueMax)
            {
                v->Ui = ((int)(v1_ValueMax) << 15);
                v->Out = v2_ValueMax;
            }
            else  //�ظ��أ�������ͨ��PID
            {
                Normal_Mode_Flag = 1;
            }
        }
        else   //����
        {
            Normal_Mode_Flag = 1;
        }
    }
    else if((v->Ref > 0) && (v->Fdb < 0))
    {
        if(v->Err > errorThreshold)  //���� 500 - -2000 =2500  >1000
        {
            FindTable(v->Fdb, 0 - SystemError.TorquePIDLimitCoeff, &v1_ValueMax, &v2_ValueMax);  //���ٶ�  �����ķ�ʽ  ���в��

            if(v->Out < 0 - v2_ValueMax)
            {
                v->Ui = 0 - ((int)(v1_ValueMax) << 15);
                v->Out = 0 - v2_ValueMax;
            }
            else  //�ظ��أ�������ͨ��PID
            {
                Normal_Mode_Flag = 1;
            }
        }
        else   //����
        {
            Normal_Mode_Flag = 1;
        }
    }


    if(Normal_Mode_Flag == 1)

    {   //���Ʊ��������
        if(v->Up > 3000) //��ֹ����ʱ��PD ���������ѹ�����MOS �𻵡�
        {
            v->Up = 3000;
        }
        else if(v->Up < -3000)
        {
            v->Up = -3000 ;
        }
        //���������
        v->Ui += v->Ki * v->Err * integralAdjustFactor;
        //�����޷�
        if(v->Ui < v->UiMin)
        {
            v->Ui = v->UiMin;
        }

        if(v->Ui > v->UiMax)
        {
            v->Ui = v->UiMax;
        }
        //�������������������ӻ��������15λ�൱��/2��15�η���
        v->OutPreSat = v->Up + (v->Ui >> 15);

        if(v->OutPreSat > v->OutMax)
        {
            v->Out =  v->OutMax;
        }
        else if(v->OutPreSat < v->OutMin)
        {
            v->Out =  v->OutMin;
        }
        else
        {
            v->Out = v->OutPreSat;
        }
    }
}
