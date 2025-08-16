#include "Pidposition.h"
#include "ExternGlobals.h"

//下述注释部分为参考参数，暂时仅供参考 
		/**
 * PID位置控制器结构体
 * 包含位置控制所需的各项参数和状态变量
 */
//typedef struct {
//    int32_t Ref;         // 位置参考值(设定值)
//    int32_t Fdb;         // 位置反馈值
//    int32_t Err;         // 位置误差
//    int32_t Kp;          // 比例系数
//    int32_t Ki;          // 积分系数
//    int32_t Up;          // 比例项输出
//    int32_t Ui;          // 积分项累计值
//    int32_t UiMax;       // 积分项上限
//    int32_t UiMin;       // 积分项下限
//    int32_t OutPreSat;   // 饱和前输出
//    int32_t Out;         // 最终输出
//    int32_t OutMax;      // 输出上限
//    int32_t OutMin;      // 输出下限
//    int32_t Speedref;    // 速度参考值
//} PIDpos;

// 系统参数定义
//static const int32_t PROPORTIONAL_LIMIT = 2500;     // 比例项输出限幅
//static const int32_t INTEGRAL_MAX = 3000 * 32768;   // 积分项上限
//static const int32_t INTEGRAL_MIN = -3000 * 32768;  // 积分项下限
//static const int32_t DIRECTION_BIAS = 750;           // 方向偏置量
//static const int32_t SPEED_SCALE_FACTOR = 15;        // 速度缩放因子
//static const int32_t SPEED_SHIFT = 1;                // 速度缩放位移量
//static const int32_t INTEGRAL_SHIFT = 15;            // 积分项缩放位移量

short v1_ValueMax12, v2_ValueMax12;
short v1_ValueMax13, v2_ValueMax13;
//int atest12345, atest12346, atest12347, atest12348, atest12349, atest1234a;
short  maxt2 = 15; //用于最大转矩的限制
short  maxtt2 = -15; //用于最大转矩的限制

short  tempspeeda = 0;
short  tempspeedb = 0;
short  tempspeedb2 = 0;

void pidposition_calc(PIDpos *v)		//PID 位置计算
{
	static short Err_pass;
		// 1. 计算位置误差
    v->Err = v->Ref - v->Fdb;
		// 2. 计算比例项并限幅，防止过大电压损坏MOS管
    v->Up = ((int)v->Kp * ((int)v->Err) >> 4);
	  // 3. 计算微分项
    v->Ud = (v->Kd * (v->Err - v->Err_pass))/0.001; // 微分项反映误差变化率1khz
    v->Err_pass = v->Err;  // 保存当前误差，用作下一次计算
	
	
	
	if(v->Err<0&&Err_pass>0)
	{
		v->Ui=0;
	}
	if(v->Err>0&&Err_pass<0)
	{
		v->Ui=0;
	}
	
	
	
    if(v->Up > 3500)  //防止抖动时，PD 产生极大电压，造成MOS 损坏。
    {
        v->Up = 3500 ;
    }
    else if(v->Up < -3500)
    {
        v->Up = -3500;
    }

		// 3. 设置积分项限幅范围
    v->UiMax = 5000 * 32768;
    v->UiMin = -5000 * 32768;
		// 4. 计算积分项并防止积分饱和
    v->Ui =  v->Ui + (v->Ki * ((v->Err) >> 4));

    if(v->Ui > v->UiMax)
    {
        v->Ui = v->UiMax;
    }

    if(v->Ui < v->UiMin)
    {
        v->Ui = v->UiMin;
    }
		// 5. 根据速度方向设置基础偏置量
    if(v->Speedref > 0)
        tempspeeda = 750;

    else if(v->Speedref < 0)
        tempspeeda = -750;

    else tempspeeda = 0;

		// 6. 计算速度前馈补偿项
    tempspeedb = ((v->Speedref * 15) >> 1) + tempspeeda;
    tempspeedb2 = ((MotorControler.SpeedFdbpFilter1 * 15) >> 1) + tempspeeda;
		
		// 7. 合成输出并进行最终限幅
    v->OutPreSat = v->Up + ((v->Ui) >> 15) + v->Ud + tempspeedb;

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

