#include "pidids.h"
//d轴电流曲线待测试
void pidids_calc(PIDIds *v)
{   
	  //计算误差，目标值与反馈值之差
    v->Err = v->Ref - v->Fdb;
	  //计算比例项输出;Kp*误差/4
	  //右移2位相当于除以4
    v->Up = (v->Kp * v->Err) >> 2; //待测试

//比例限幅
	  //正向限幅
    if(v->Up > 1000) //防止抖动时，PD 产生极大电压，造成MOS 损坏。
    {
        v->Up = 1000 ;
    }
		//负向限幅
    else if(v->Up < -1000)
    {
        v->Up = -1000;
    }
    //计算积分项：积分项+=Ki*误差*50
    v->Ui += v->Ki * v->Err * 10; //待测试
    
//积分限幅
    if(v->Ui > v->UiMax)
    {
        v->Ui = v->UiMax;
    }

    if(v->Ui < v->UiMin)
    {
        v->Ui = v->UiMin;
    }

    v->OutPreSat = v->Up + ((v->Ui) >> 15);

//最终输出结果
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
