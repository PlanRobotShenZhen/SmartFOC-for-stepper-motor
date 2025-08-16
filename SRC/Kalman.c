#include "ExternGlobals.h"
#include "Kalman.h"

// ÿ��ͨ����Ӧһ���˲���
KalmanFilter_t kalman_channel[8] = {
    {0, 1, 0.1, 0.001}, {0, 1, 0.1, 0.001}, {0, 1, 0.1, 0.001}, {0, 1, 0.1, 0.001},
    {0, 1, 0.1, 0.001}, {0, 1, 0.1, 0.001}, {0, 1, 0.1, 0.001}, {0, 1, 0.1, 0.001},
};


void Kalman_Init(KalmanFilter_t *kf, float init_val, float Q, float R)
{
    kf->x = init_val;
    kf->p = 1.0f;  // ��ʼЭ����
    kf->q = Q;
    kf->r = R;
}

short Kalman_Filter(short input, short CHANNEL_ID)
{
    KalmanFilter_t *kf = &kalman_channel[CHANNEL_ID];
    float z = (float)input;  // �������ֵ

    // Ԥ�����
    kf->p = kf->p + kf->q;

    // ����������
    float k = kf->p / (kf->p + kf->r);

    // ���¹���
    kf->x = kf->x + k * (z - kf->x);

    // ����Э����
    kf->p = (1 - k) * kf->p;

    return (short)kf->x;
}