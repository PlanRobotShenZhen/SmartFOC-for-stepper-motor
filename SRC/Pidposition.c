#include "Pidposition.h"
#include "ExternGlobals.h"

//����ע�Ͳ���Ϊ�ο���������ʱ�����ο� 
		/**
 * PIDλ�ÿ������ṹ��
 * ����λ�ÿ�������ĸ��������״̬����
 */
//typedef struct {
//    int32_t Ref;         // λ�òο�ֵ(�趨ֵ)
//    int32_t Fdb;         // λ�÷���ֵ
//    int32_t Err;         // λ�����
//    int32_t Kp;          // ����ϵ��
//    int32_t Ki;          // ����ϵ��
//    int32_t Up;          // ���������
//    int32_t Ui;          // �������ۼ�ֵ
//    int32_t UiMax;       // ����������
//    int32_t UiMin;       // ����������
//    int32_t OutPreSat;   // ����ǰ���
//    int32_t Out;         // �������
//    int32_t OutMax;      // �������
//    int32_t OutMin;      // �������
//    int32_t Speedref;    // �ٶȲο�ֵ
//} PIDpos;

// ϵͳ��������
//static const int32_t PROPORTIONAL_LIMIT = 2500;     // ����������޷�
//static const int32_t INTEGRAL_MAX = 3000 * 32768;   // ����������
//static const int32_t INTEGRAL_MIN = -3000 * 32768;  // ����������
//static const int32_t DIRECTION_BIAS = 750;           // ����ƫ����
//static const int32_t SPEED_SCALE_FACTOR = 15;        // �ٶ���������
//static const int32_t SPEED_SHIFT = 1;                // �ٶ�����λ����
//static const int32_t INTEGRAL_SHIFT = 15;            // ����������λ����

short v1_ValueMax12, v2_ValueMax12;
short v1_ValueMax13, v2_ValueMax13;
//int atest12345, atest12346, atest12347, atest12348, atest12349, atest1234a;
short  maxt2 = 15; //�������ת�ص�����
short  maxtt2 = -15; //�������ת�ص�����

short  tempspeeda = 0;
short  tempspeedb = 0;
short  tempspeedb2 = 0;

void pidposition_calc(PIDpos *v)		//PID λ�ü���
{
	static short Err_pass;
		// 1. ����λ�����
    v->Err = v->Ref - v->Fdb;
		// 2. ���������޷�����ֹ�����ѹ��MOS��
    v->Up = ((int)v->Kp * ((int)v->Err) >> 4);
	  // 3. ����΢����
    v->Ud = (v->Kd * (v->Err - v->Err_pass))/0.001; // ΢���ӳ���仯��1khz
    v->Err_pass = v->Err;  // ���浱ǰ��������һ�μ���
	
	
	
	if(v->Err<0&&Err_pass>0)
	{
		v->Ui=0;
	}
	if(v->Err>0&&Err_pass<0)
	{
		v->Ui=0;
	}
	
	
	
    if(v->Up > 3500)  //��ֹ����ʱ��PD ���������ѹ�����MOS �𻵡�
    {
        v->Up = 3500 ;
    }
    else if(v->Up < -3500)
    {
        v->Up = -3500;
    }

		// 3. ���û������޷���Χ
    v->UiMax = 5000 * 32768;
    v->UiMin = -5000 * 32768;
		// 4. ����������ֹ���ֱ���
    v->Ui =  v->Ui + (v->Ki * ((v->Err) >> 4));

    if(v->Ui > v->UiMax)
    {
        v->Ui = v->UiMax;
    }

    if(v->Ui < v->UiMin)
    {
        v->Ui = v->UiMin;
    }
		// 5. �����ٶȷ������û���ƫ����
    if(v->Speedref > 0)
        tempspeeda = 750;

    else if(v->Speedref < 0)
        tempspeeda = -750;

    else tempspeeda = 0;

		// 6. �����ٶ�ǰ��������
    tempspeedb = ((v->Speedref * 15) >> 1) + tempspeeda;
    tempspeedb2 = ((MotorControler.SpeedFdbpFilter1 * 15) >> 1) + tempspeeda;
		
		// 7. �ϳ���������������޷�
    v->OutPreSat = v->Up + ((v->Ui) >> 15) + v->Ud + tempspeedb;

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

