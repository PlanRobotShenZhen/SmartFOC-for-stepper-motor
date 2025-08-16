#include "pidc.h"

short  SpeedFdbpPost2 = 0;
void pidc_calc(PIDC *v)					// PID ֻ�õ���PI ������������Up  = Kp*Err      //Ui =  Ki * Err * 50
{
	//����������ͱ���������ֵ
    v->Err = v->Ref - v->Fdb;

	//�쳣����
    if(v->Err > 26000)
    {
        v->Err = SpeedFdbpPost2;
    }

    if(v->Err < -26000)
    {
        v->Err = SpeedFdbpPost2;
    }

    SpeedFdbpPost2 = v->Err;
	
	
    v->Up = (int)v->Kp * (int)v->Err;


	//�޷�
    if(v->Up > 3000)  //��ֹ����ʱ��PD ���������ѹ�����MOS �𻵡�
    {
        v->Up = 3000 ;
    }
    else if(v->Up < -3000)
    {
        v->Up = -3000;
    }

    v->Ui += v->Ki * v->Err * 50;

    if(v->Ui < v->UiMin)
    {
        v->Ui = v->UiMin;
    }

    if(v->Ui > v->UiMax)
    {
        v->Ui = v->UiMax;
    }


	//�ϳɱ������������
    v->OutPreSat = v->Up + ((v->Ui) >> 15); //��С������Ӱ��

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
