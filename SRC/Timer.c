//����ʵ���˴�100΢�뵽1Сʱ�Ķ༶��ʱ������
// ʹ�ýṹ��TIMER���涨ʱ��״̬������ÿ�ε���Timeing����ʱ�����¼������ͱ�־λ

#include "Timer1.h"
#define M_ENABLE0 1 //���ñ�־λ
void Timeing(TIMER *v);

#include "ExternGlobals.h"   //ygj
//��ʱ�����º�����ʵ�ֶ༶��ʱ��ϵͳ
void Timeing(TIMER *v)
{
    v->Tmer100us ++;      //100΢�������

    if(v->Tmer100us >= 10)  //1ms
    {
        v->Tmer100us = 0;
        v->Flag1ms = M_ENABLE0;  //1ms��־λ
        v->Tmer1ms ++ ;   //1ms����������
        v->Tmer1ms25 ++ ; //25ms����������
        v->Tmer1ms50 ++ ; //50ms����������
    }

    if(v->Tmer1ms >= 10)    //10ms
    {
        v->Flag10ms = M_ENABLE0;  //10ms��־λ
        v->Tmer1ms = 0;
        v->Tmer10ms ++;
        v->Tmer25ms ++;
    }
    if(v->Tmer1ms25 >= 25)  //25ms
    {
        v->Flag25ms = M_ENABLE0;  //25ms��־λ
        v->Tmer1ms25 = 0;
    }
    if(v->Tmer1ms50 >= 50)  //50ms
    {
        v->Flag50ms = M_ENABLE0;
        v->Tmer1ms50 = 0;
    }
    if(v->Tmer10ms >= 10)   //100ms
    {
        v->Flag100ms = M_ENABLE0; //100ms��־λ
        v->Tmer10ms = 0;
    }

    if(v->Tmer25ms >= 25)   //250ms
    {
        v->Flag250ms = M_ENABLE0; //250ms��־λ
        v->Tmer25ms = 0;
        v->Tmer250ms ++;
    }

    if(v->Tmer250ms >= 2)   //0.5s
    {
        v->Flag500ms = M_ENABLE0;  //0.5s��־λ
        v->Tmer250ms = 0;
        v->Tmer500ms ++;

    }

    if(v->Tmer500ms >= 2)   //1s
    {
        v->Flag1s = M_ENABLE0;     //1s��־λ
        v->Tmer500ms = 0;
        v->Tmer1s ++;
    }

    if(v->Tmer1s >= 60)  //1min
    {
        v->Flag1m = M_ENABLE0;     //1min��־λ
        v->Tmer1s = 0;
        v->Tmer1m ++;
    }

    if(v->Tmer1m >= 60)  //1h
    {
        v->Flag1h = M_ENABLE0;     //1h��־λ
        v->Tmer1m = 0;
        v->Tmer1h++;
    }
}
