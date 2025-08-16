#include "pidIqs.h"
#include "ExternGlobals.h"
int temp_Ui;
void pidIqs_calc(PIDIqs *v)
{
	//计算电流误差和比例项的输出值
    v->Err = v->Ref - v->Fdb;
    v->Up = v->Kp * v->Err;
	  v->Up = (v->Up > 20000) ? 20000 : ((v->Up < -20000) ? -20000 : v->Up);

	//积分项累加
	    if((v->Ui > 20000&&v->Err>0)||(v->Ui < -20000&&v->Err<0))
    { }else
    {
        temp_Ui += v->Err ; 

    }
       if (v->Ref == 0 && v->Fdb == 0) { // 速度为0清零积分
           v->Ui = 0;
       }
    if(v->Err>0)
	{
		
	}

    if((pidpv.Ref >= 0)&&MotorControler.SpeedFdbp < 0 )  //当期望转速为正，实际转速为负值时清零输出
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
    else if((pidpv.Ref < 0)&&MotorControler.SpeedFdbp > 0)  // 反向
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
	//合成比例项与积分项
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

