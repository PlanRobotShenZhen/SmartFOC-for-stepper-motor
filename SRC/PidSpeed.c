/*速度环PID控制器
工作流程：1.误差计算和限幅
         2.状态判断
					3.查表控制
					4.正常模式PID计算
*/
#include "pidcspeed.h"
#include "ExternGlobals.h"

//速度反馈
short  errLimit = 0; //误差限幅变量
short  vel_fb;       //速度反馈值

//PID控制参数
static short errorThreshold = 0;  //误差阈值
short  isNormalMode = 0;   //正常模式标志位
short tableResult1=0, tableResult2=0;//查表结果

//转矩限幅
short  maxTorque  = 15; //最大转矩限幅
short  minTorque = -10; //最小转矩限幅
short  maxTorque1000  = 150; //千分比转矩限幅

//积分限幅参数
int integralMaxPos  = 150 * 32767;  //正积分最大值
int integralMaxNeg  = -150 * 32767; //负积分最大值


short v1_ValueMax3, v2_ValueMax3;
int test12345 = 0, test12346 = 0, test12347 = 0, test12348 = 0, test12349 = 0, test1234a = 0;


short Uqs_Speed_Equation_Table[31][2] =
{
   //拟合得到的Uqs-速度公式，如第二项{626,1110}表示100rpm的Uqs=626x+1110，其中x的单位是1/4额定转矩
   //0到 3000rpm，每一项间隔100rpm
   {626, 1110}, \
   {626, 1110}, \
   {617, 1876}, \
   {659, 2460}, \
   {667, 3147}, \
   {666, 3934}, \
   {687, 4540}, \
   {715, 5109}, \
   {725, 5846}, \
   {746, 6496}, \
   {803, 7029}, \
   {818, 7641}, \
   {858, 8219}, \
   {829, 9224}, \
   {935, 9713}, \
   {877, 10457}, \
   {1000, 10780}, \
   {972, 11557}, \
   {970, 12440}, \
   {921, 13057}, \
   {930, 13811}, \
   {1001, 14607}, \
   {965, 15253}, \
   {831, 16344}, \
   {1105, 16036}, \
   {966, 17524}, \
   {1017, 17897}, \
   {1287, 17242}, \
   {1175, 18616}, \
   {1296, 19161}, \
   {959, 21605}\
};




void FindTable2(short speed, short torquecoeff, short* result1, short* result2)//查表，用年是反馈速度 //要考虑 重负载时的工况
{
    short speedtem = 0;
    short index = 0;
    short frac = 0;
    short  resulttmp = 0;

    if(speed >= 0) speedtem = speed;
    else speedtem = 0 - speed;

    index = speedtem / 100; // 计算速度所在的表格区间索引 (100 rpm 为一个区间)
    frac = speedtem - index * 100; // 获取十位和个位数

    if(index < 30)
    {
        resulttmp = (Uqs_Speed_Equation_Table[index][0] + (Uqs_Speed_Equation_Table[index + 1][0] - Uqs_Speed_Equation_Table[index][0]) * frac / 100) * torquecoeff;
        *result1 = Uqs_Speed_Equation_Table[index][1] + (Uqs_Speed_Equation_Table[index + 1][1] - Uqs_Speed_Equation_Table[index][1]) * frac / 100;
        *result2  = *result1 +  resulttmp;       
    }
    else if(index >= 30)
    {
        *result1 = Uqs_Speed_Equation_Table[30][0] * torquecoeff + Uqs_Speed_Equation_Table[30][1];     
    }
}


void FindTable(short speed, short torqueCoeff, short* result1, short* result2) {
   //简化版查表，根据速度符号设置基础值，叠加转矩系数
   short baseSpeed = (speed > 0) ? 750 : (speed < 0 ? -750 : 0);
   short adjustedSpeed = (speed * 15 >> 1) + baseSpeed; // 速度调整公式:(speed*15)/2 + base
   *result1 = adjustedSpeed;                            // 输出基础值
   *result2 = adjustedSpeed + torqueCoeff * 300;        // 总输出 = 基础值 + 转矩系数*300
}

//PID计算函数
void pidspeed_calc(PIDSpeed *v)
{
	//误差计算与限幅
   v->Err = v->Ref - v->Fdb;
   isNormalMode = 1; //默认为正常模式

   // 正常模式下PID计算
   if (isNormalMode)
  		 {
       // 比例计算与限幅
       v->Up = v->Kp * v->Err; // 右移8位除以256
       v->Up = (v->Up > 3000) ? 3000 : ((v->Up < -3000) ? -3000 : v->Up);

       // 积分计算与处理
       v->Ui += v->Ki * v->Err * 60; // 合并常数项:20*3=60
       if (v->Ref == 0 && v->Fdb == 0)
				 { // 速度为0清零积分
           v->Ui = 0;
         }

       v->Ui = (v->Ui > 5000 * 32768) ? 5000 * 32768 : ((v->Ui < -5000 * 32768) ? -5000 * 32768 : v->Ui);

       // 合并输出并限幅
       v->OutPreSat = v->Up + (v->Ui >> 15);
       v->Out = (v->OutPreSat > v->OutMax) ? v->OutMax : ((v->OutPreSat < v->OutMin) ? v->OutMin : v->OutPreSat);
       }
}

