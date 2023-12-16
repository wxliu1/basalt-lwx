//

#ifndef __FILTER_H__
#define __FILTER_H__

// #include "platform.h"

// #include "FreeRTOS.h"
// #include "portable.h"


/// @brief 定义滤波算法库中使用的动态内存分配方法
// #define FILTER_MALLOC  pvPortMalloc     // 使用FreeRTOS实时系统的内存分配方法
#define FILTER_MALLOC  malloc        // 使用标准C库的内存分配方法

/**
 * @brief 卡尔曼滤波结构体
 *      为每个卡尔曼滤波的通道都定义一个卡尔曼滤波结构体变量
 */
typedef struct Kalman_Filter_Struct_
{
    Kalman_Filter_Struct_()
    {
        Q_ProcessNoise = 0.001;
        R_MeasureNoise = 0.1;
        estimate_value_init = 25;
        estimate_variance_init = 1;
        estimate_value_curr = 25;
        estimate_variance_curr = 1;
    }

    /// @brief Q代表预测值的方差，R代表测量值的方差，更小方差对应的量对下次的预测值有更大的影响权重
    float Q_ProcessNoise;                      // 预测值的方差, Q_ProcessNoise:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
    float R_MeasureNoise;                      // 测量值的方差, R_MeasureNoise:测量噪声，R增大，动态响应变慢，收敛稳定性变好

    float estimate_value_init;    // 初始时刻温度的最优估计值
    float estimate_variance_init; // 初始时刻温度的估计方差

    float estimate_value_curr;    // 当前温度的最优估计值
    float estimate_variance_curr; // 当前温度的估计方差

    float predict_value_next;    // 下一个状态的预测值
    float predict_variance_next; // 下一个状态的估计方差
} Kalman_Filter_Struct;

/**
 * @brief 卡尔曼滤波初始化，主要是实现卡尔曼滤波结构体变量的成员初始化
 *      针对温度采集信号等类似问题设计，即初始状态选取为初始测量值、当前时刻的温度估计值为上一时刻的温度。
 * @param kalman_filter 卡尔曼滤波结构体变量
 * @param Q_ProcessNoise Q值，过程噪声，预测值的方差
 * @param R_MeasureNoise R值，测量噪声，测量值的方差
 * @param init_estimate_value 初始时刻温度的估计值
 * @param init_estimate_variance 初始时刻温度的估计方差
 */
void Kalman_Filter_Init(Kalman_Filter_Struct *kalman_filter, float Q_ProcessNoise, float R_MeasureNoise, float init_estimate_value, float init_estimate_variance);

/**
 * @brief 卡尔曼滤波迭代
 *      滤波器估计过程中某一个时刻的状态，利用量测更新的值作为反馈。所以卡尔曼滤波过程又可以分为状态跟新和测量更新两个部分来实现。
 *      状态更新中主要是为了获得下个时刻的先验估计，量测更新则是为了通过先验估计和量测值获取后验估计。
 * @param kalman_filter 卡尔曼滤波结构体变量
 * @param measure_value_curr 当前量测值
 * @return float 卡尔曼滤波值
 */
float Kalman_Filter_Iterate(Kalman_Filter_Struct *kalman_filter, float measure_value_curr);

#endif


