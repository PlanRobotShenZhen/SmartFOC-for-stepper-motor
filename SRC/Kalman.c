#include "ExternGlobals.h"
#include "Kalman.h"

// 每个通道对应一个滤波器
KalmanFilter_t kalman_channel[8] = {
    {0, 1, 0.1, 0.001}, {0, 1, 0.1, 0.001}, {0, 1, 0.1, 0.001}, {0, 1, 0.1, 0.001},
    {0, 1, 0.1, 0.001}, {0, 1, 0.1, 0.001}, {0, 1, 0.1, 0.001}, {0, 1, 0.1, 0.001},
};


void Kalman_Init(KalmanFilter_t *kf, float init_val, float Q, float R)
{
    kf->x = init_val;
    kf->p = 1.0f;  // 初始协方差
    kf->q = Q;
    kf->r = R;
}

short Kalman_Filter(short input, short CHANNEL_ID)
{
    KalmanFilter_t *kf = &kalman_channel[CHANNEL_ID];
    float z = (float)input;  // 输入测量值

    // 预测更新
    kf->p = kf->p + kf->q;

    // 卡尔曼增益
    float k = kf->p / (kf->p + kf->r);

    // 更新估计
    kf->x = kf->x + k * (z - kf->x);

    // 更新协方差
    kf->p = (1 - k) * kf->p;

    return (short)kf->x;
}