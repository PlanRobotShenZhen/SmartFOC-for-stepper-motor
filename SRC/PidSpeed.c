/*�ٶȻ�PID������
�������̣�1.��������޷�
         2.״̬�ж�
					3.������
					4.����ģʽPID����
*/
#include "pidcspeed.h"
#include "ExternGlobals.h"

//�ٶȷ���
short  errLimit = 0; //����޷�����
short  vel_fb;       //�ٶȷ���ֵ

//PID���Ʋ���
static short errorThreshold = 0;  //�����ֵ
short  isNormalMode = 0;   //����ģʽ��־λ
short tableResult1=0, tableResult2=0;//�����

//ת���޷�
short  maxTorque  = 15; //���ת���޷�
short  minTorque = -10; //��Сת���޷�
short  maxTorque1000  = 150; //ǧ�ֱ�ת���޷�

//�����޷�����
int integralMaxPos  = 150 * 32767;  //���������ֵ
int integralMaxNeg  = -150 * 32767; //���������ֵ


short v1_ValueMax3, v2_ValueMax3;
int test12345 = 0, test12346 = 0, test12347 = 0, test12348 = 0, test12349 = 0, test1234a = 0;


short Uqs_Speed_Equation_Table[31][2] =
{
   //��ϵõ���Uqs-�ٶȹ�ʽ����ڶ���{626,1110}��ʾ100rpm��Uqs=626x+1110������x�ĵ�λ��1/4�ת��
   //0�� 3000rpm��ÿһ����100rpm
   {626, 1110}, \
   {626, 1110}, \
   {617, 1876}, \
   {659, 2460}, \
   {667, 3147}, \
   {666, 3934}, \
   {687, 4540}, \
   {715, 5109}, \
   {725, 5846}, \
   {746, 6496}, \
   {803, 7029}, \
   {818, 7641}, \
   {858, 8219}, \
   {829, 9224}, \
   {935, 9713}, \
   {877, 10457}, \
   {1000, 10780}, \
   {972, 11557}, \
   {970, 12440}, \
   {921, 13057}, \
   {930, 13811}, \
   {1001, 14607}, \
   {965, 15253}, \
   {831, 16344}, \
   {1105, 16036}, \
   {966, 17524}, \
   {1017, 17897}, \
   {1287, 17242}, \
   {1175, 18616}, \
   {1296, 19161}, \
   {959, 21605}\
};




void FindTable2(short speed, short torquecoeff, short* result1, short* result2)//��������Ƿ����ٶ� //Ҫ���� �ظ���ʱ�Ĺ���
{
    short speedtem = 0;
    short index = 0;
    short frac = 0;
    short  resulttmp = 0;

    if(speed >= 0) speedtem = speed;
    else speedtem = 0 - speed;

    index = speedtem / 100; // �����ٶ����ڵı���������� (100 rpm Ϊһ������)
    frac = speedtem - index * 100; // ��ȡʮλ�͸�λ��

    if(index < 30)
    {
        resulttmp = (Uqs_Speed_Equation_Table[index][0] + (Uqs_Speed_Equation_Table[index + 1][0] - Uqs_Speed_Equation_Table[index][0]) * frac / 100) * torquecoeff;
        *result1 = Uqs_Speed_Equation_Table[index][1] + (Uqs_Speed_Equation_Table[index + 1][1] - Uqs_Speed_Equation_Table[index][1]) * frac / 100;
        *result2  = *result1 +  resulttmp;       
    }
    else if(index >= 30)
    {
        *result1 = Uqs_Speed_Equation_Table[30][0] * torquecoeff + Uqs_Speed_Equation_Table[30][1];     
    }
}


void FindTable(short speed, short torqueCoeff, short* result1, short* result2) {
   //�򻯰��������ٶȷ������û���ֵ������ת��ϵ��
   short baseSpeed = (speed > 0) ? 750 : (speed < 0 ? -750 : 0);
   short adjustedSpeed = (speed * 15 >> 1) + baseSpeed; // �ٶȵ�����ʽ:(speed*15)/2 + base
   *result1 = adjustedSpeed;                            // �������ֵ
   *result2 = adjustedSpeed + torqueCoeff * 300;        // ����� = ����ֵ + ת��ϵ��*300
}

//PID���㺯��
void pidspeed_calc(PIDSpeed *v)
{
	//���������޷�
   v->Err = v->Ref - v->Fdb;
   isNormalMode = 1; //Ĭ��Ϊ����ģʽ

   // ����ģʽ��PID����
   if (isNormalMode)
  		 {
       // �����������޷�
       v->Up = v->Kp * v->Err; // ����8λ����256
       v->Up = (v->Up > 3000) ? 3000 : ((v->Up < -3000) ? -3000 : v->Up);

       // ���ּ����봦��
       v->Ui += v->Ki * v->Err * 60; // �ϲ�������:20*3=60
       if (v->Ref == 0 && v->Fdb == 0)
				 { // �ٶ�Ϊ0�������
           v->Ui = 0;
         }

       v->Ui = (v->Ui > 5000 * 32768) ? 5000 * 32768 : ((v->Ui < -5000 * 32768) ? -5000 * 32768 : v->Ui);

       // �ϲ�������޷�
       v->OutPreSat = v->Up + (v->Ui >> 15);
       v->Out = (v->OutPreSat > v->OutMax) ? v->OutMax : ((v->OutPreSat < v->OutMin) ? v->OutMin : v->OutPreSat);
       }
}

