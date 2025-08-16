#include "SpeedPlan2.h"
#include "math.h"

/// @brief 根据传入结构体指针设置参数，进而规划S型曲线各段时间和位移
/// @param ptr 用于传递必要的参数
/// @return 返回SP_Error枚举类型，提示错误信息


enum SP_Error Set_SpeedPlant_Para(Set_SP_Para *ptr)
{
    //检查输入数据合法性
    enum SP_Error sp_error = none_err;

    if(ptr->accel_max < 0)
    {
        sp_error = accel_max_neg_err;
    }

    if(ptr->decel_max < 0)
    {
        sp_error = decel_max_neg_err;
    }

    if(ptr->a_accel < 0)
    {
        sp_error = a_accel_neg_err;
    }

    if(ptr->a_decel < 0)
    {
        sp_error = a_decel_neg_err;
    }

    if(fabsf(ptr->vel_tar) < 1.0f) //定点
    {
        sp_error = vel_tar_zero_err;
    }

    //定义指针，避免从ptr开始寻址，简化
    SpeedPlant *sp = ptr->sp;

    //倒转判断
    if(ptr->end_position < ptr->start_position)
    {
        float tem_value = 0;
        sp->forward_flag = -1;
        tem_value = ptr->end_position;
        ptr->end_position = ptr->start_position;
        ptr->start_position = tem_value;//
        ptr->vel_init = -ptr->vel_init;
        ptr->vel_tar = -fabsf(ptr->vel_tar); //目标转动方向一定要跟目标速度同号
    }
    else
    {
        sp->forward_flag = 1;
        ptr->vel_tar = fabsf(ptr->vel_tar);
    }

    //速度方向判断
    sp->vel_flag = 1;
    if((ptr->vel_tar >= 0) && (ptr->vel_init >= ptr->vel_tar)) //Vo>=Vm>=0
    {
        sp->vel_flag = -1;
        ptr->a_accel = ptr->a_decel; //此时需要减速，采用减速部分的参数
        ptr->accel_max = ptr->decel_max;
    }
    else if(ptr->vel_tar < 0) //Vm<0
    {
        if((-1.0f * ptr->vel_init) <= ptr->vel_tar) //Vo<Vm<0
        {
            sp->vel_flag = -1;
            ptr->a_decel = ptr->a_accel; //此时需要加速，采用加速部分的参数
            ptr->decel_max = ptr->accel_max;
        }
        ptr->vel_tar = -ptr->vel_tar; //取反，倒转时所有速度取反，起始和结束位置对调
    }

    float accel_max_tem = ptr->accel_max; //存储最大加速度，用于后面最大加速度判断

    if(sp_error == none_err)
    {
        //这样可以将情况整合在一起，所有情况都能到达最高加速度，只是这种情况下的t2=t1
        if(fabsf(ptr->vel_tar - ptr->vel_init) <= (powf(accel_max_tem, 2) / ptr->a_accel))//Δv<(am^2)/j
        {
            //设置的最大速度较小。很快到达，前半段没有匀加速度阶段，将最大加速度重置为临界加速度
            ptr->accel_max = sqrtf(fabsf((ptr->vel_tar - ptr->vel_init)) * ptr->a_accel);//am=(|Δv|*j)^(1/2)
        }

        if(fabsf(ptr->vel_tar) <= (powf(ptr->decel_max, 2) / ptr->a_decel)) //与上面同理
        {
            ptr->decel_max = sqrtf(fabsf(ptr->vel_tar) * ptr->a_decel);
        }

        //sp->s3=(ptr->vel_tar+ptr->vel_init)*(ptr->accel_max/ptr->a_accel+fabsf(ptr->vel_tar-ptr->vel_init)/ptr->accel_max)/2;
        sp->s7 = (ptr->vel_tar + ptr->vel_init) * (ptr->accel_max / ptr->a_accel + fabsf(ptr->vel_tar - ptr->vel_init) / ptr->accel_max) / 2 \
                 +ptr->vel_tar * (ptr->decel_max / ptr->a_decel + ptr->vel_tar / ptr->decel_max) / 2;//

        //目标位置不足以走完没有匀速阶段的全程
        while((ptr->end_position - ptr->start_position) < sp->s7)
        {
            //如果目标速度与初始速度相同时仍不能实现曲线，则降低目标速度也不能实现，提示出错
            if(fabsf(ptr->vel_tar - ptr->vel_init) < 1e-6f)
            {
                sp_error = distance_too_small_err;
                break;
            }

            ptr->vel_tar = ptr->vel_tar * 0.95f; //降低目标速度

            //如果目标速度过低，认为曲线无法实现
            if(ptr->vel_tar < 1.0f)  //由于速度是定点，至少为1
            {
                sp_error = distance_too_small_err;
                break;
            }

            //如果目标速度降低后由大于初始速度变成小于初始速度，需要将速度标志设为-1
            if((ptr->vel_init > ptr->vel_tar) && (sp->vel_flag == 1))
            {
                sp->vel_flag = -1;
            }

            //修改最大加速度和最大减速度
            if(fabsf(ptr->vel_tar - ptr->vel_init) <= (powf(accel_max_tem, 2) / ptr->a_accel))
            {
                ptr->accel_max = sqrtf(fabsf((ptr->vel_tar - ptr->vel_init)) * ptr->a_accel);
            }

            if(fabsf(ptr->vel_tar) <= (powf(ptr->decel_max, 2) / ptr->a_decel))
            {
                ptr->decel_max = sqrtf(fabsf(ptr->vel_tar) * ptr->a_decel);
            }

            //重新计算总位移
            sp->s7 = (ptr->vel_tar + ptr->vel_init) * (ptr->accel_max / ptr->a_accel + fabsf(ptr->vel_tar - ptr->vel_init) / ptr->accel_max) / 2 \
                     +ptr->vel_tar * (ptr->decel_max / ptr->a_decel + ptr->vel_tar / ptr->decel_max) / 2;
        }

        sp->s7 = sp->s7 + ptr->start_position;
        //计算有匀速阶段的曲线数据
        float accel_aaccel = ptr->accel_max / ptr->a_accel;
        float vel_accel;

        if(ptr->accel_max < 1e-6f)
        {
            vel_accel = accel_aaccel;
        }
        else
        {
            vel_accel = fabsf((ptr->vel_tar - ptr->vel_init)) / ptr->accel_max;
        }

        float decel_adecel = ptr->decel_max / ptr->a_decel;
        float vel_decel;

        if(ptr->decel_max < 1e-6f)
        {
            vel_decel = decel_adecel;
        }
        else
        {
            vel_decel = ptr->vel_tar / ptr->decel_max;
        }

        float t2_t1 = vel_accel - accel_aaccel;
        float t6_t5 = vel_decel - decel_adecel;
        sp->t1 = accel_aaccel;
        sp->t2 = vel_accel;
        sp->t3 = vel_accel + accel_aaccel;
        sp->t4 = sp->t3 + (ptr->end_position - sp->s7) / ptr->vel_tar;//s7是所有变速度阶段走过的位移，
        sp->t5 = sp->t4 + decel_adecel;
        sp->t6 = sp->t4 + vel_decel;
        sp->t7 = sp->t6 + decel_adecel;

        sp->v1 = ptr->vel_init + 0.5f * sp->vel_flag * ptr->a_accel * powf(accel_aaccel, 2);
        sp->v2 = sp->v1 + ptr->accel_max * sp->vel_flag * t2_t1;
        sp->v3 = ptr->vel_tar;
        sp->v4 = sp->v3;
        sp->v5 = sp->v4 - 0.5f * ptr->a_decel * sp->vel_flag * powf(decel_adecel, 2);
        sp->v6 = sp->v5 - ptr->decel_max * sp->vel_flag * t6_t5;

        sp->s1 = ptr->start_position + ptr->vel_init * accel_aaccel + 1.0f / 6.0f * sp->vel_flag * ptr->a_accel * powf(accel_aaccel, 3);
        sp->s2 = sp->s1 + sp->v1 * t2_t1 + 0.5f * ptr->accel_max * sp->vel_flag * powf(t2_t1, 2);
        sp->s3 = sp->s2 + sp->v2 * accel_aaccel + 0.5f * ptr->accel_max * sp->vel_flag * powf(accel_aaccel, 2) - 1.0f / 6.0f * ptr->a_accel * sp->vel_flag * powf(accel_aaccel, 3);
        sp->s4 = sp->s3 + sp->v3 * (sp->t4 - sp->t3);
        sp->s5 = sp->s4 + sp->v4 * decel_adecel - 1.0 / 6 * ptr->a_decel * sp->vel_flag * powf(decel_adecel, 3);
        sp->s6 = sp->s5 + sp->v5 * t6_t5 - 0.5f * ptr->decel_max * sp->vel_flag * powf(t6_t5, 2);
        //float ss=sp->s6+sp->v6*decel_adecel-0.5*ptr->decel_max*sp->vel_flag*powf(decel_adecel,2)+1.0/6*ptr->a_decel*sp->vel_flag*powf(decel_adecel,3);
        sp->s7 = ptr->end_position;
    }

    return sp_error;
}

/// @brief 计算时刻t对应的S型曲线的加速度、速度和路程
/// @param t 时刻
/// @return 返回当前数据结构体，包含上述三个值
Current_value SpeedPlant_positionControl(Set_SP_Para* ptr, float t)
{
    Current_value current_value;
    SpeedPlant* sp = ptr->sp;

    if(t <= 0)
    {
        current_value.accel = 0;
        current_value.vel = ptr->vel_init;
        current_value.position = ptr->start_position;
    }
    else if(t <= sp->t1)
    {
        float t_pow = powf(t, 2);
        current_value.accel = sp->vel_flag * ptr->a_accel * t;
        current_value.vel = ptr->vel_init + 0.5 * sp->vel_flag * ptr->a_accel * t_pow;
        current_value.position = ptr->start_position + 1.0 / 6 * sp->vel_flag * ptr->a_accel * t_pow * t + ptr->vel_init * t;
    }
    else if(t <= sp->t2)
    {
        t = t - sp->t1;
        current_value.accel = sp->vel_flag * ptr->accel_max;
        current_value.vel = sp->v1 + current_value.accel * t;
        current_value.position = sp->s1 + sp->v1 * t + 0.5 * sp->vel_flag * ptr->accel_max * powf(t, 2);
    }
    else if(t <= sp->t3)
    {
        t = t - sp->t2;
        float t_pow = powf(t, 2);
        current_value.accel = sp->vel_flag * (ptr->accel_max - ptr->a_accel * t);
        current_value.vel = sp->v3 - 0.5f * ptr->a_accel * sp->vel_flag * powf(sp->t1 - t, 2);
        current_value.position = sp->s2 + sp->v2 * t + 0.5f * ptr->accel_max * sp->vel_flag * t_pow - 1.0f / 6.0f * ptr->a_accel * sp->vel_flag * t_pow * t;
    }
    else if(t <= sp->t4)
    {
        current_value.accel = 0;
        current_value.vel = sp->v3;
        current_value.position = sp->s3 + sp->v3 * (t - sp->t3);
    }
    else if(t <= sp->t5)
    {
        t = t - sp->t4;
        float t_pow = powf(t, 2);
        current_value.accel = -ptr->a_decel * t;
        current_value.vel = sp->v4 - 0.5f * ptr->a_decel * t_pow;
        current_value.position = sp->s4 + sp->v4 * t - 1.0f / 6.0f * ptr->a_decel * t_pow * t;
    }
    else if(t <= sp->t6)
    {
        t = t - sp->t5;
        current_value.accel = -ptr->decel_max;
        current_value.vel = sp->v5 - ptr->decel_max * t;
        current_value.position = sp->s5 + sp->v5 * t - 0.5f * ptr->decel_max * powf(t, 2);
    }
    else if(t <= sp->t7)
    {
        t = t - sp->t6;
        float t_pow = powf(t, 2);
        current_value.accel = ptr->a_decel * t - ptr->decel_max;
        current_value.vel = sp->v6 - ptr->decel_max * t + 0.5f * ptr->a_decel * t_pow;
        current_value.position = sp->s6 + sp->v6 * t - 0.5f * ptr->decel_max * t_pow + 1.0f / 6.0f * ptr->a_decel * t_pow * t;
    }
    else if(t > sp->t7)
    {
        current_value.accel = 0;
        current_value.vel = 0;
        current_value.position = ptr->end_position;
    }

    if(sp->forward_flag == -1)
    {
        current_value.accel = -current_value.accel;
        current_value.vel = -current_value.vel;
        current_value.position = ptr->end_position + ptr->start_position - current_value.position;
    }

    return current_value;
}

//返回单段曲线总时间
float get_total_time(Set_SP_Para*ptr)
{
    return ptr->sp->t7;
}



/************************************* */


/******以下为多段曲线的变量和函数*********/


/************************************* */

float accel_max_tem_multi = 0;      //存放初始的最大加速度值
float decel_max_tem_multi = 0;      //存放初始的最大减速度值
struct SpeedPlant_Multi* sp_ptr;    //指向计算好的数据
int step_num = 0;                   //曲线的段数
int multi_err_num = 0;              //返回错误的段的编号
//多段曲线存储参数结构体
struct SpeedPlant_Multi
{
    Set_SP_Para para;
    Current_value current_value;
    float vel_end;
    float t1;
    float t2;
    float t3;
    float t4;
    float t5;
    float t6;
    float t7;
    float s1;
    float s2;
    float s3;
    float s4;
    float s5;
    float s6;
    float s7;
    int forward_flag;
    int vel_flag_f;
    int vel_flag_b;
};

/// @brief 根据ptr指针计算t1-t7和s1-s7
/// @param con_vel_flag 为false则计算t1-t7和s1-s7，其中t4=t3；为true则只计算t4（匀速）之后的数据
void sp_cal_multi(struct SpeedPlant_Multi *ptr, int con_vel_flag)  //wjj
{

    if(!con_vel_flag)
    {
        //这样可以将情况整合在一起，所有情况都能到达最高加速度，只是这种情况下的t2=t1
        if(fabsf((*ptr).para.vel_tar - (*ptr).para.vel_init) <= (powf(accel_max_tem_multi, 2) / (*ptr).para.a_accel))
        {
            //设置的最大速度较小。很快到达，前半段没有匀加速度阶段，将最大加速度重置为临界加速度
            (*ptr).para.accel_max = sqrtf(fabsf(((*ptr).para.vel_tar - (*ptr).para.vel_init)) * (*ptr).para.a_accel);
        }

        if(fabsf((*ptr).para.vel_tar - (*ptr).vel_end) <= (powf(decel_max_tem_multi, 2) / (*ptr).para.a_decel)) //与上面同理
        {
            (*ptr).para.decel_max = sqrtf(fabsf((*ptr).para.vel_tar - (*ptr).vel_end) * (*ptr).para.a_decel);
        }

        (*ptr).t1 = (*ptr).para.accel_max / (*ptr).para.a_accel;

        if((*ptr).para.accel_max < 1e-6f)
        {
            (*ptr).t2 = (*ptr).t1;    //避免accel_max为0时出现0/0的情况
        }
        else
        {
            (*ptr).t2 = fabsf(((*ptr).para.vel_tar - (*ptr).para.vel_init)) / (*ptr).para.accel_max;
        }

        (*ptr).t3 = (*ptr).t1 + (*ptr).t2;
        (*ptr).t4 = (*ptr).t3;
        //(*ptr)所有sx值都是包含起始位置值的
        (*ptr).s1 = (*ptr).para.start_position + 1.0f / 6.0f * (*ptr).vel_flag_f * (*ptr).para.a_accel * powf((*ptr).t1, 3) + (*ptr).para.vel_init * (*ptr).t1;
        (*ptr).s2 = (*ptr).s1 + 0.50f * (*ptr).vel_flag_f * (*ptr).para.a_accel * powf((*ptr).t1, 2) * ((*ptr).t2 - (*ptr).t1) \
                    +0.5f * (*ptr).vel_flag_f * (*ptr).para.accel_max * powf((*ptr).t2 - (*ptr).t1, 2) + (*ptr).para.vel_init * ((*ptr).t2 - (*ptr).t1);
        (*ptr).s3 = (*ptr).para.start_position + 0.5f * (*ptr).vel_flag_f * (*ptr).para.a_accel * powf((*ptr).t1, 2) * (*ptr).t2 \
                    +0.5f * (*ptr).vel_flag_f * (*ptr).para.accel_max * powf((*ptr).t2, 2) + (*ptr).para.vel_init * (*ptr).t3;
    }
    else
    {
        (*ptr).t4 = (*ptr).t3 + ((*ptr).para.end_position - (*ptr).s7) / (*ptr).para.vel_tar;
    }

    (*ptr).t5 = (*ptr).t4 + (*ptr).para.decel_max / (*ptr).para.a_decel;

    if((*ptr).para.decel_max < 1e-6f)
    {
        (*ptr).t6 = (*ptr).t5;
    }
    else
    {
        (*ptr).t6 = (*ptr).t4 + fabsf((*ptr).para.vel_tar - (*ptr).vel_end) / (*ptr).para.decel_max;
    }

    (*ptr).t7 = (*ptr).t6 + (*ptr).para.decel_max / (*ptr).para.a_decel;

    (*ptr).s4 = (*ptr).s3 + (*ptr).para.vel_tar * ((*ptr).t4 - (*ptr).t3);
    (*ptr).s5 = (*ptr).s4 + (*ptr).para.vel_tar * ((*ptr).t5 - (*ptr).t4) - 1.0f / 6.0f * (*ptr).vel_flag_b * (*ptr).para.a_decel * powf((*ptr).t5 - (*ptr).t4, 3);
    (*ptr).s6 = (*ptr).s5 + (*ptr).para.vel_tar * ((*ptr).t6 - (*ptr).t5) - 0.5f * (*ptr).vel_flag_b * (*ptr).para.a_decel * powf((*ptr).t5 - (*ptr).t4, 2) * ((*ptr).t6 - (*ptr).t5) \
                -0.5f * (*ptr).vel_flag_b * (*ptr).para.decel_max * powf((*ptr).t6 - (*ptr).t5, 2);
    (*ptr).s7 = (*ptr).s6 + (*ptr).para.vel_tar * ((*ptr).t7 - (*ptr).t6) - 0.5f * (*ptr).vel_flag_b * (*ptr).para.a_decel * powf((*ptr).t5 - (*ptr).t4, 2) * ((*ptr).t7 - (*ptr).t6) \
                -(*ptr).vel_flag_b * (*ptr).para.decel_max * ((*ptr).t6 - (*ptr).t5) * ((*ptr).t7 - (*ptr).t6) - 0.5f * (*ptr).vel_flag_b * (*ptr).para.decel_max * powf((*ptr).t7 - (*ptr).t6, 2) \
                +1.0f / 6.0f * (*ptr).vel_flag_b * (*ptr).para.a_decel * powf((*ptr).t7 - (*ptr).t6, 3);
}


/// @brief 计算时刻t对应的S型曲线的加速度、速度和路程
/// @param t 时刻
/// @return 返回当前数据结构体，包含上述三个值
Current_value SpeedPlant_positionControl_multi(float t)
{
    float t_sum = 0;
    int step = 0;
    int i, j;

    for(i = 0; i < step_num; i++)
    {
        t_sum = t_sum + (*(sp_ptr + i)).t7;

        if(t <= t_sum)
        {
            step = i;

            for(j = 0; j < i; j++)
            {
                t = t - (*(sp_ptr + j)).t7;
            };

            break;
        }
        else if(i == step_num - 1)
        {
            step = i;
        }
    }

    sp_ptr = sp_ptr + step;

    if(t <= 0)
    {
        (*sp_ptr).current_value.accel = 0;
        (*sp_ptr).current_value.vel = (*sp_ptr).para.vel_init;
        (*sp_ptr).current_value.position = (*sp_ptr).para.start_position;
    }
    else if(t <= (*sp_ptr).t1)
    {
        (*sp_ptr).current_value.accel = (*sp_ptr).vel_flag_f * (*sp_ptr).para.a_accel * t;
        (*sp_ptr).current_value.vel = (*sp_ptr).para.vel_init + 0.5f * (*sp_ptr).vel_flag_f * (*sp_ptr).para.a_accel * powf(t, 2);
        (*sp_ptr).current_value.position = (*sp_ptr).para.start_position + 1.0f / 6 * (*sp_ptr).vel_flag_f * (*sp_ptr).para.a_accel * powf(t, 3) + (*sp_ptr).para.vel_init * t;
    }
    else if(t <= (*sp_ptr).t2)
    {
        (*sp_ptr).current_value.accel = (*sp_ptr).vel_flag_f * (*sp_ptr).para.accel_max;
        (*sp_ptr).current_value.vel = (*sp_ptr).para.vel_init + 0.5f * (*sp_ptr).vel_flag_f * (*sp_ptr).para.a_accel * powf((*sp_ptr).t1, 2) + (*sp_ptr).vel_flag_f * (*sp_ptr).para.accel_max * (t - (*sp_ptr).t1);
        (*sp_ptr).current_value.position = (*sp_ptr).s1 + 0.5f * (*sp_ptr).para.a_accel * powf((*sp_ptr).t1, 2) * (t - (*sp_ptr).t1) + 0.5f * (*sp_ptr).para.accel_max * powf(t - (*sp_ptr).t1, 2) + (*sp_ptr).para.vel_init * (t - (*sp_ptr).t1);
    }
    else if(t <= (*sp_ptr).t3)
    {
        (*sp_ptr).current_value.accel = (*sp_ptr).vel_flag_f * (*sp_ptr).para.accel_max - (*sp_ptr).vel_flag_f * (*sp_ptr).para.a_accel * (t - (*sp_ptr).t2);
        (*sp_ptr).current_value.vel = (*sp_ptr).para.vel_init + 0.5f * (*sp_ptr).vel_flag_f * (*sp_ptr).para.a_accel * powf((*sp_ptr).t1, 2) + (*sp_ptr).vel_flag_f * (*sp_ptr).para.accel_max * (t - (*sp_ptr).t1) - 0.5f * (*sp_ptr).vel_flag_f * (*sp_ptr).para.a_accel * powf(t - (*sp_ptr).t2, 2);
        (*sp_ptr).current_value.position = (*sp_ptr).s2 + 0.5f * (*sp_ptr).vel_flag_f * (*sp_ptr).para.a_accel * powf((*sp_ptr).t1, 2) * (t - (*sp_ptr).t2) + (*sp_ptr).vel_flag_f * (*sp_ptr).para.accel_max * ((*sp_ptr).t2 - (*sp_ptr).t1) * (t - (*sp_ptr).t2) \
                                           +0.5f * (*sp_ptr).vel_flag_f * (*sp_ptr).para.accel_max * powf(t - (*sp_ptr).t2, 2) - 1.0f / 6 * (*sp_ptr).vel_flag_f * (*sp_ptr).para.a_accel * powf(t - (*sp_ptr).t2, 3) + (*sp_ptr).para.vel_init * (t - (*sp_ptr).t2);
    }
    else if(t <= (*sp_ptr).t4)
    {
        (*sp_ptr).current_value.accel = 0;
        (*sp_ptr).current_value.vel = (*sp_ptr).para.vel_tar;
        (*sp_ptr).current_value.position = (*sp_ptr).s3 + (*sp_ptr).para.vel_tar * (t - (*sp_ptr).t3);
    }
    else if(t <= (*sp_ptr).t5)
    {
        (*sp_ptr).current_value.accel = -(*sp_ptr).vel_flag_b * (*sp_ptr).para.a_decel * (t - (*sp_ptr).t4);
        (*sp_ptr).current_value.vel = (*sp_ptr).para.vel_tar - 0.5 * (*sp_ptr).vel_flag_b * (*sp_ptr).para.a_decel * powf(t - (*sp_ptr).t4, 2);
        (*sp_ptr).current_value.position = (*sp_ptr).s4 + (*sp_ptr).para.vel_tar * (t - (*sp_ptr).t4) - 1.0 / 6 * (*sp_ptr).vel_flag_b * (*sp_ptr).para.a_decel * powf(t - (*sp_ptr).t4, 3);
    }
    else if(t <= (*sp_ptr).t6)
    {
        (*sp_ptr).current_value.accel = -(*sp_ptr).vel_flag_b * (*sp_ptr).para.decel_max;
        (*sp_ptr).current_value.vel = (*sp_ptr).para.vel_tar - 0.5 * (*sp_ptr).vel_flag_b * (*sp_ptr).para.a_decel * powf((*sp_ptr).t5 - (*sp_ptr).t4, 2) - (*sp_ptr).vel_flag_b * (*sp_ptr).para.decel_max * (t - (*sp_ptr).t5);
        (*sp_ptr).current_value.position = (*sp_ptr).s5 + (*sp_ptr).para.vel_tar * (t - (*sp_ptr).t5) - 0.5 * (*sp_ptr).vel_flag_b * (*sp_ptr).para.a_decel * powf((*sp_ptr).t5 - (*sp_ptr).t4, 2) * (t - (*sp_ptr).t5) \
                                           -0.5 * (*sp_ptr).vel_flag_b * (*sp_ptr).para.decel_max * powf(t - (*sp_ptr).t5, 2);
    }
    else if(t <= (*sp_ptr).t7)
    {
        (*sp_ptr).current_value.accel = (*sp_ptr).vel_flag_b * (*sp_ptr).para.a_decel * (t - (*sp_ptr).t6) - (*sp_ptr).vel_flag_b * (*sp_ptr).para.decel_max;
        (*sp_ptr).current_value.vel = (*sp_ptr).para.vel_tar - 0.5 * (*sp_ptr).vel_flag_b * (*sp_ptr).para.a_decel * powf((*sp_ptr).t5 - (*sp_ptr).t4, 2) - (*sp_ptr).vel_flag_b * (*sp_ptr).para.decel_max * ((*sp_ptr).t6 - (*sp_ptr).t5) \
                                      -(*sp_ptr).vel_flag_b * (*sp_ptr).para.decel_max * (t - (*sp_ptr).t6) + 0.5 * (*sp_ptr).vel_flag_b * (*sp_ptr).para.a_decel * powf(t - (*sp_ptr).t6, 2);
        (*sp_ptr).current_value.position = (*sp_ptr).s6 + (*sp_ptr).para.vel_tar * (t - (*sp_ptr).t6) - 0.5 * (*sp_ptr).vel_flag_b * (*sp_ptr).para.a_decel * powf((*sp_ptr).t5 - (*sp_ptr).t4, 2) * (t - (*sp_ptr).t6) \
                                           -(*sp_ptr).vel_flag_b * (*sp_ptr).para.decel_max * ((*sp_ptr).t6 - (*sp_ptr).t5) * (t - (*sp_ptr).t6) - 0.5 * (*sp_ptr).vel_flag_b * (*sp_ptr).para.decel_max * powf(t - (*sp_ptr).t6, 2) + 1.0 / 6 * (*sp_ptr).vel_flag_b * (*sp_ptr).para.a_decel * powf(t - (*sp_ptr).t6, 3);
    }
    else if(t > (*sp_ptr).t7)
    {
        (*sp_ptr).current_value.accel = 0;
        (*sp_ptr).current_value.vel = 0;
        (*sp_ptr).current_value.position = (*sp_ptr).para.end_position;
    }

    if(!((*sp_ptr).forward_flag == 1))
    {
        (*sp_ptr).current_value.accel = -(*sp_ptr).current_value.accel;
        (*sp_ptr).current_value.vel = -(*sp_ptr).current_value.vel;
        (*sp_ptr).current_value.position = (*sp_ptr).para.end_position + (*sp_ptr).para.start_position - (*sp_ptr).current_value.position;
    }

    sp_ptr = sp_ptr - step;
    return (*(sp_ptr + step)).current_value;
}

//获取多段曲线总时间
float get_total_time_multi(void)
{
    float t_sum = 0;
    int i;

    for(i = 0; i < step_num; i++)
    {
        t_sum = t_sum + (*(sp_ptr + i)).t7;
    }

    return t_sum;
}
