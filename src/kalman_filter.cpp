//

#include "kalman_filter.h"

void Kalman_Filter_Init(Kalman_Filter_Struct *kalman_filter, float Q_ProcessNoise, float R_MeasureNoise, float init_estimate_value, float init_estimate_variance)
{
    kalman_filter->Q_ProcessNoise = Q_ProcessNoise;
    kalman_filter->R_MeasureNoise = R_MeasureNoise;

    kalman_filter->estimate_value_init = init_estimate_value;
    kalman_filter->estimate_variance_init = init_estimate_variance;

    kalman_filter->estimate_value_curr = kalman_filter->estimate_value_init;
    kalman_filter->estimate_variance_curr = kalman_filter->estimate_variance_init;
}


float Kalman_Filter_Iterate(Kalman_Filter_Struct *kalman_filter, float measure_value_curr)
{
    // 利用当前估计值迭代下次预测值
    kalman_filter->predict_value_next = kalman_filter->estimate_value_curr;
    kalman_filter->predict_variance_next = kalman_filter->estimate_variance_curr + kalman_filter->Q_ProcessNoise;

    // 计算卡尔曼增益
    float k_index = kalman_filter->predict_variance_next / (kalman_filter->predict_variance_next + kalman_filter->R_MeasureNoise);

    // 利用下次预测值迭代下次估计值
    // 例如在进行温度预测时，因为温度是一个连续的状态，我们认为上一时刻的温度和当前时刻的温度相等，则有T(k)=T(k-1)
    kalman_filter->estimate_value_curr = k_index * measure_value_curr + (1 - k_index) * kalman_filter->predict_value_next;
    kalman_filter->estimate_variance_curr = (1 - k_index) * kalman_filter->predict_variance_next;

    // 输出下次估计值
    return kalman_filter->estimate_value_curr;
}

