//代码实现了从100微秒到1小时的多级定时器功能
// 使用结构体TIMER储存定时器状态，并在每次调用Timeing函数时，更新计数器和标志位

#include "Timer1.h"
#define M_ENABLE0 1 //启用标志位
void Timeing(TIMER *v);

#include "ExternGlobals.h"   //ygj
//定时器更新函数，实现多级定时器系统
void Timeing(TIMER *v)
{
    v->Tmer100us ++;      //100微秒计数器

    if(v->Tmer100us >= 10)  //1ms
    {
        v->Tmer100us = 0;
        v->Flag1ms = M_ENABLE0;  //1ms标志位
        v->Tmer1ms ++ ;   //1ms基础计数器
        v->Tmer1ms25 ++ ; //25ms基础计数器
        v->Tmer1ms50 ++ ; //50ms基础计数器
    }

    if(v->Tmer1ms >= 10)    //10ms
    {
        v->Flag10ms = M_ENABLE0;  //10ms标志位
        v->Tmer1ms = 0;
        v->Tmer10ms ++;
        v->Tmer25ms ++;
    }
    if(v->Tmer1ms25 >= 25)  //25ms
    {
        v->Flag25ms = M_ENABLE0;  //25ms标志位
        v->Tmer1ms25 = 0;
    }
    if(v->Tmer1ms50 >= 50)  //50ms
    {
        v->Flag50ms = M_ENABLE0;
        v->Tmer1ms50 = 0;
    }
    if(v->Tmer10ms >= 10)   //100ms
    {
        v->Flag100ms = M_ENABLE0; //100ms标志位
        v->Tmer10ms = 0;
    }

    if(v->Tmer25ms >= 25)   //250ms
    {
        v->Flag250ms = M_ENABLE0; //250ms标志位
        v->Tmer25ms = 0;
        v->Tmer250ms ++;
    }

    if(v->Tmer250ms >= 2)   //0.5s
    {
        v->Flag500ms = M_ENABLE0;  //0.5s标志位
        v->Tmer250ms = 0;
        v->Tmer500ms ++;

    }

    if(v->Tmer500ms >= 2)   //1s
    {
        v->Flag1s = M_ENABLE0;     //1s标志位
        v->Tmer500ms = 0;
        v->Tmer1s ++;
    }

    if(v->Tmer1s >= 60)  //1min
    {
        v->Flag1m = M_ENABLE0;     //1min标志位
        v->Tmer1s = 0;
        v->Tmer1m ++;
    }

    if(v->Tmer1m >= 60)  //1h
    {
        v->Flag1h = M_ENABLE0;     //1h标志位
        v->Tmer1m = 0;
        v->Tmer1h++;
    }
}
