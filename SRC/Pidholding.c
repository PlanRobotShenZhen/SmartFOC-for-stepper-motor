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
//上电保持位置时的PID
void Pidholding_calc(Pidholding *v)
{
    int  errtmp = 0;
	
		// 计算位置误差和比例项
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
		
		// 比例项限幅，防止过大电压损坏MOS管
    if(v->Up > 3000)  //防止抖动时，PD 产生极大电压，造成MOS 损坏。
    {
        v->Up = 3000 ;
    }
    else if(v->Up < -3000)
    {
        v->Up = -3000;
    }

		//取偏差绝对值
    if(v->Err > 0) errtmp = v->Err;
    else if(v->Err < 0) errtmp = 0 - v->Err;
    else  errtmp = 1;

		//根据偏差大小阶梯式更新积分项
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

		//由于是位置保持PID，抖动较大时要清除积分项来快速缓解
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

		//参考大于反馈
    if((v->Err > 0))  //同向正 加减速
    {
        if(v->Out > 3000)
        {
            v->Out =  3000;
            v->Ui =   3000 * 32768;
        }
    }
    else if((v->Err < 0))   //同向负 加减速
    {
        if(v->Out < -3000)
        {
            v->Out =  -3000;
            v->Ui =   -3000 * 32768;
        }
    }
}

