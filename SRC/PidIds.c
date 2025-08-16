#include "pidids.h"
//d��������ߴ�����
void pidids_calc(PIDIds *v)
{   
	  //������Ŀ��ֵ�뷴��ֵ֮��
    v->Err = v->Ref - v->Fdb;
	  //������������;Kp*���/4
	  //����2λ�൱�ڳ���4
    v->Up = (v->Kp * v->Err) >> 2; //������

//�����޷�
	  //�����޷�
    if(v->Up > 1000) //��ֹ����ʱ��PD ���������ѹ�����MOS �𻵡�
    {
        v->Up = 1000 ;
    }
		//�����޷�
    else if(v->Up < -1000)
    {
        v->Up = -1000;
    }
    //��������������+=Ki*���*50
    v->Ui += v->Ki * v->Err * 10; //������
    
//�����޷�
    if(v->Ui > v->UiMax)
    {
        v->Ui = v->UiMax;
    }

    if(v->Ui < v->UiMin)
    {
        v->Ui = v->UiMin;
    }

    v->OutPreSat = v->Up + ((v->Ui) >> 15);

//����������
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
