#include "pidc.h"

short  SpeedFdbpPost2 = 0;
void pidc_calc(PIDC *v)					// PID 只用到了PI ，用来做电流Up  = Kp*Err      //Ui =  Ki * Err * 50
{
	//计算电流误差和比例项的输出值
    v->Err = v->Ref - v->Fdb;

	//异常误差保护
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


	//限幅
    if(v->Up > 3000)  //防止抖动时，PD 产生极大电压，造成MOS 损坏。
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


	//合成比例项与积分项
    v->OutPreSat = v->Up + ((v->Ui) >> 15); //缩小积分项影响

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
