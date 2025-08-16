#ifndef __KALMAN_H__
#define __KALMAN_H__

typedef struct {
    float x;    // ����ֵ
    float p;    // �������Э����
    float q;    // ��������Э����
    float r;    // ��������Э����
} KalmanFilter_t;


short Kalman_Filter(short input, short CHANNEL_ID);


#endif