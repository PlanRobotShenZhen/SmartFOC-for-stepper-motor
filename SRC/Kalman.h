#ifndef __KALMAN_H__
#define __KALMAN_H__

typedef struct {
    float x;    // 估计值
    float p;    // 估计误差协方差
    float q;    // 过程噪声协方差
    float r;    // 测量噪声协方差
} KalmanFilter_t;


short Kalman_Filter(short input, short CHANNEL_ID);


#endif