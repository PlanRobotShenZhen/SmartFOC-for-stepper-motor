#include "pidIqs.h"
#include "ExternGlobals.h"
int temp_Ui;
void pidIqs_calc(PIDIqs *v)
{
	//����������ͱ���������ֵ
    v->Err = v->Ref - v->Fdb;
    v->Up = v->Kp * v->Err;
	  v->Up = (v->Up > 20000) ? 20000 : ((v->Up < -20000) ? -20000 : v->Up);

	//�������ۼ�
	    if((v->Ui > 20000&&v->Err>0)||(v->Ui < -20000&&v->Err<0))
    { }else
    {
        temp_Ui += v->Err ; 

    }
       if (v->Ref == 0 && v->Fdb == 0) { // �ٶ�Ϊ0�������
           v->Ui = 0;
       }
    if(v->Err>0)
	{
		
	}

    if((pidpv.Ref >= 0)&&MotorControler.SpeedFdbp < 0 )  //������ת��Ϊ����ʵ��ת��Ϊ��ֵʱ�������
    {
      //  v->Ref = v->CurrentRef;

        if(temp_Ui < 0)
        {
            temp_Ui = 0;
        }

        if(v->Up < 0)
        {
           v->Up = 0;
        }
    }
    else if((pidpv.Ref < 0)&&MotorControler.SpeedFdbp > 0)  // ����
    {
        if(temp_Ui > 0)
        {
            temp_Ui = 0;
        }

        if(v->Up > 0)
        {
            v->Up = 0;
        }
    }
		v->Ui=v->Ki *((temp_Ui) >> 15);
	//�ϳɱ������������
		v->Ui = (v->Ui > 20000) ? 20000 : ((v->Ui < -20000) ? -20000 : v->Ui);
		v->OutPreSat = v->Up+v->Ui; 

	
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

