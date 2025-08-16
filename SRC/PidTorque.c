/*转矩PID的功能是根据电机的工作状态（加速、减速、匀速、方向切换）动态调整控制参数，实现更精准的转矩控制
 工作流程： 1.基本误差计算
            2.状态判断和模式选择
						3.查表控制策略
						4.正常模式PID计算
*/
#include "pidtorque.h"
#include "ExternGlobals.h"
//全局变量定义
short integralAdjustFactor = 5;  //积分系数调整参数
short static errorThreshold = 0; //误差临时变量
short Normal_Mode_Flag = 0;      //正常模式标志位
short v1_ValueMax, v2_ValueMax;  //查表结果存储变量

extern void FindTable(short speed, short torquecoeff, short* result1, short* result2);
void pidtorque_calc(PIDTorque *v)
{
	  //基本PID计算
    v->Err = v->Ref - v->Fdb;                //计算误差
    v->Up = ((int)v->Kp * (int)v->Err) >> 2; //计算比例项：Kp*误差（右移两位相当于/4）

    // 最大输入参考电流为800
    // 最大扭矩所需的电压为750*4
    errorThreshold = SystemError.ThresholdCoeff * 3;
    
	  //初始化正常模式标志位
    Normal_Mode_Flag = 0;
	
//判断方向，状态
    if((v->Ref > 0) && (v->Fdb > 0))  //同向正 加减速
    {
        if(v->Err > errorThreshold)  //加速  3000 -500 =2500 >1000
        {   //以速度  反馈的方式  进行查表获取最大输出限制
            FindTable(v->Fdb, SystemError.TorquePIDLimitCoeff, &v1_ValueMax, &v2_ValueMax);  
					  //限制输出不超过最大值
            if(v->Out < v2_ValueMax)
            {
                v->Ui = ((int)(v1_ValueMax) << 15); //设置积分项初始值
                v->Out = v2_ValueMax;               //输出限制
            }
            else  //重负载，采用普通的PID
            {
                Normal_Mode_Flag = 1;
            }
        }
        else if(v->Err < (0 - errorThreshold)) //减速  500- 3000 = -2500 <-1000
        {
            FindTable(v->Fdb, 0 - SystemError.TorquePIDLimitCoeff, &v1_ValueMax, &v2_ValueMax);  //以速度  反馈的方式  进行查表
            //限制输出不超过最小值
            if(v->Out > v2_ValueMax)
            {  
                v->Ui = ((int)(v1_ValueMax) << 15);
                v->Out = v2_ValueMax;
            }
            else  //重负载，采用普通的PID
            {
                Normal_Mode_Flag = 1;
            }
        }
        else   //匀速
        {
            Normal_Mode_Flag = 1;
        }
    }
    else if((v->Ref < 0) && (v->Fdb < 0))  //同向负 加减速
    {
        if(v->Err < 0 - errorThreshold) //加速   -3000 - -500 =-2500 <-1000
        {
            FindTable(v->Fdb, SystemError.TorquePIDLimitCoeff, &v1_ValueMax, &v2_ValueMax);  //以速度  反馈的方式  进行查表

            if(v->Out > 0 - v2_ValueMax)
            {
                v->Ui = 0 - ((int)(v1_ValueMax) << 15);
                v->Out = 0 - v2_ValueMax;
            }
            else  //重负载，采用普通的PID
            {
                Normal_Mode_Flag = 1;
            }
        }
        else if(v->Err > errorThreshold)  //减速  -500 - -300  =2500 >1000
        {
            FindTable(v->Fdb, 0 - SystemError.TorquePIDLimitCoeff, &v1_ValueMax, &v2_ValueMax);  //以速度  反馈的方式  进行查表

            if(v->Out < 0 - v2_ValueMax)
            {
                v->Ui = 0 - ((int)(v1_ValueMax) << 15);
                v->Out = 0 - v2_ValueMax;
            }
            else  //重负载，采用普通的PID
            {
                Normal_Mode_Flag = 1;
            }
        }
        else   //匀速
        {
            Normal_Mode_Flag = 1;
        }
    }
    else if((v->Ref < 0) && (v->Fdb > 0))  //先正向减速
    {
        if(v->Err < (0 - errorThreshold)) //减速 -500 - 2000 =-2500  <-1000
        {

            FindTable(v->Fdb, 0 - SystemError.TorquePIDLimitCoeff, &v1_ValueMax, &v2_ValueMax);  //以速度  反馈的方式  进行查表

            if(v->Out > v2_ValueMax)
            {
                v->Ui = ((int)(v1_ValueMax) << 15);
                v->Out = v2_ValueMax;
            }
            else  //重负载，采用普通的PID
            {
                Normal_Mode_Flag = 1;
            }
        }
        else   //匀速
        {
            Normal_Mode_Flag = 1;
        }
    }
    else if((v->Ref > 0) && (v->Fdb < 0))
    {
        if(v->Err > errorThreshold)  //减速 500 - -2000 =2500  >1000
        {
            FindTable(v->Fdb, 0 - SystemError.TorquePIDLimitCoeff, &v1_ValueMax, &v2_ValueMax);  //以速度  反馈的方式  进行查表

            if(v->Out < 0 - v2_ValueMax)
            {
                v->Ui = 0 - ((int)(v1_ValueMax) << 15);
                v->Out = 0 - v2_ValueMax;
            }
            else  //重负载，采用普通的PID
            {
                Normal_Mode_Flag = 1;
            }
        }
        else   //匀速
        {
            Normal_Mode_Flag = 1;
        }
    }


    if(Normal_Mode_Flag == 1)

    {   //限制比例项输出
        if(v->Up > 3000) //防止抖动时，PD 产生极大电压，造成MOS 损坏。
        {
            v->Up = 3000;
        }
        else if(v->Up < -3000)
        {
            v->Up = -3000 ;
        }
        //计算积分项
        v->Ui += v->Ki * v->Err * integralAdjustFactor;
        //积分限幅
        if(v->Ui < v->UiMin)
        {
            v->Ui = v->UiMin;
        }

        if(v->Ui > v->UiMax)
        {
            v->Ui = v->UiMax;
        }
        //计算最终输出，比例项加积分项（右移15位相当于/2的15次方）
        v->OutPreSat = v->Up + (v->Ui >> 15);

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
}
