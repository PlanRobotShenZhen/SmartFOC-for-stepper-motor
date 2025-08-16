#include "Pidholding.h"
#include "ExternGlobals.h"

long long errorsum1 = 0;
long long errorsum11 = 0;
long long errorsum111 = 0;
long long errorsum2 = 0;
long long errorsum22 = 0;
long long errorsum222 = 0;
short shfitnum = 12;

short speed0 = 0;
short speedmax = 1000;
short UoutLinit1, UoutLinit2;
short clearflg = 0;
//�ϵ籣��λ��ʱ��PID
void Pidholding_calc(Pidholding *v)
{
    int  errtmp = 0;
	
		// ����λ�����ͱ�����
    v->Err = v->Ref - v->Fdb;
    v->Up = ((int)v->Kp * (int)v->Err) >> 3 ;


    if(v->Err > 0)
    {
        errorsum1++;
        errorsum11 = errorsum11 + v->Err;
        errorsum111 = errorsum11 >> shfitnum;
    }

    if(v->Err < 0)
    {
        errorsum2++;
        errorsum22 = errorsum22 + v->Err;
        errorsum222 = errorsum22 >> shfitnum;
    }


    if(clearflg == 1)
    {
        clearflg = 0;
        speed0 = 0;
        errorsum1 = 0;
        errorsum11 = 0;
        errorsum111 = 0;
        errorsum2 = 0;
        errorsum22 = 0;
        errorsum222 = 0;
    }
		
		// �������޷�����ֹ�����ѹ��MOS��
    if(v->Up > 3000)  //��ֹ����ʱ��PD ���������ѹ�����MOS �𻵡�
    {
        v->Up = 3000 ;
    }
    else if(v->Up < -3000)
    {
        v->Up = -3000;
    }

		//ȡƫ�����ֵ
    if(v->Err > 0) errtmp = v->Err;
    else if(v->Err < 0) errtmp = 0 - v->Err;
    else  errtmp = 1;

		//����ƫ���С����ʽ���»�����
    if(errtmp >= 250)
        v->Ui += (v->Ki * (v->Err >> 6) * (errtmp >> 7)) ;
    else if((errtmp < 250) && (errtmp >= 100))
        v->Ui += (v->Ki * (v->Err >> 4) * (errtmp >> 5)) ;
    else if((errtmp < 100) && (errtmp >= 30))
        v->Ui += (v->Ki * (v->Err >> 3) * (errtmp >> 2)) ;
    else if((errtmp < 30) && (errtmp >= 1))
        v->Ui += (v->Ki * (v->Err) * (errtmp >> 1)) ;
		
    if(v->Ui < v->UiMin)
    {
        v->Ui = v->UiMin;
    }
    if(v->Ui > v->UiMax)
    {
        v->Ui = v->UiMax;
    }

		//������λ�ñ���PID�������ϴ�ʱҪ��������������ٻ���
    if((v->Speedfdb > 50) || (v->Speedfdb < -50))
    {
        v->Ui = 0;
    }

    v->OutPreSat = v->Up + ((v->Ui) >> 15);

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

		//�ο����ڷ���
    if((v->Err > 0))  //ͬ���� �Ӽ���
    {
        if(v->Out > 3000)
        {
            v->Out =  3000;
            v->Ui =   3000 * 32768;
        }
    }
    else if((v->Err < 0))   //ͬ�� �Ӽ���
    {
        if(v->Out < -3000)
        {
            v->Out =  -3000;
            v->Ui =   -3000 * 32768;
        }
    }
}

