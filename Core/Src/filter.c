#include "filter.h"
#include <string.h> // 用于 memcpy
#include <stdlib.h> // 用于 qsort

// 一阶低通滤波器
/**
 * @brief 一阶低通滤波器，用于平滑信号，去除高频噪声。
 * @param input 输入信号
 * @param alpha 滤波系数，范围为 0.0 到 1.0，越接近 1.0 响应越快
 * @param state 滤波器的状态变量，保存上一次的输出值
 * @return 滤波后的信号
 */
float lowpass_filter(float input, float alpha, float *state)
{
    *state = alpha * input + (1.0f - alpha) * (*state);
    return *state;
}

// 一阶高通滤波器
/**
 * @brief 一阶高通滤波器，用于去除低频信号，保留高频成分。
 * @param input 输入信号
 * @param alpha 滤波系数，范围为 0.0 到 1.0，越接近 1.0 响应越快
 * @param state 滤波器的状态变量，保存上一次的输出值
 * @param prev_input 保存上一次的输入信号
 * @return 滤波后的信号
 */
float highpass_filter(float input, float alpha, float *state, float *prev_input)
{
    float output = alpha * (*state + input - *prev_input);
    *prev_input = input; // 更新上一次输入
    *state = output;     // 更新滤波器状态
    return output;
}

// 滑动平均滤波器
/**
 * @brief 滑动平均滤波器，用于平滑信号，适合处理周期性噪声。
 * @param input 输入信号
 * @param buffer 滤波器的缓冲区，用于存储最近的输入值
 * @param buffer_size 缓冲区大小
 * @param index 当前缓冲区的索引
 * @param sum 缓冲区中所有值的总和
 * @return 滤波后的信号
 */
float moving_average_filter(float input, float *buffer, int buffer_size, int *index, float *sum)
{
    *sum -= buffer[*index];              // 从总和中减去旧值
    buffer[*index] = input;              // 更新缓冲区
    *sum += input;                       // 将新值加到总和中
    *index = (*index + 1) % buffer_size; // 更新索引
    return *sum / buffer_size;           // 返回平均值
}

// 加权平均滤波器
/**
 * @brief 加权平均滤波器，对输入数据赋予不同的权重。
 * @param inputs 输入信号数组
 * @param weights 权重数组，与输入信号一一对应
 * @param size 数组大小
 * @return 滤波后的信号
 */
float weighted_average_filter(float *inputs, float *weights, int size)
{
    float sum = 0;
    float weight_sum = 0;
    for (int i = 0; i < size; i++)
    {
        sum += inputs[i] * weights[i];
        weight_sum += weights[i];
    }
    return sum / weight_sum; // 返回加权平均值
}

// 中值滤波器
/**
 * @brief 中值滤波器，用于去除尖锐的噪声（如脉冲噪声）。
 * @param buffer 输入信号数组
 * @param size 数组大小
 * @return 滤波后的信号
 */
float median_filter(float *buffer, int size)
{
    float temp[size];
    memcpy(temp, buffer, size * sizeof(float)); // 复制缓冲区
    qsort(temp, size, sizeof(float), compare);  // 对数据排序
    return temp[size / 2];                      // 返回中值
}

// 指数加权移动平均滤波器
/**
 * @brief 指数加权移动平均滤波器，用于实时数据流的平滑处理。
 * @param input 输入信号
 * @param alpha 滤波系数，范围为 0.0 到 1.0，越接近 1.0 响应越快
 * @param state 滤波器的状态变量，保存上一次的输出值
 * @return 滤波后的信号
 */
float exponential_moving_average(float input, float alpha, float *state)
{
    *state = alpha * input + (1.0f - alpha) * (*state);
    return *state;
}

// 卡尔曼滤波器
/**
 * @brief 卡尔曼滤波器，用于动态系统的状态估计。
 * @param input 输入信号
 * @param estimate 滤波器的状态估计值
 * @param error_cov 估计误差协方差
 * @param process_noise 过程噪声协方差
 * @param measurement_noise 测量噪声协方差
 * @return 滤波后的信号
 */
float kalman_filter(float input, float *estimate, float *error_cov, float process_noise, float measurement_noise)
{
    float kalman_gain = *error_cov / (*error_cov + measurement_noise);
    *estimate = *estimate + kalman_gain * (input - *estimate);
    *error_cov = (1 - kalman_gain) * (*error_cov) + process_noise;
    return *estimate;
}

/**
 * @brief 比较函数，用于 qsort 对浮点数排序。
 * @param a 指向第一个元素的指针
 * @param b 指向第二个元素的指针
 * @return 比较结果：-1 表示 a < b，1 表示 a > b，0 表示 a == b
 */
int compare(const void *a, const void *b)
{
    float diff = (*(float *)a) - (*(float *)b);
    if (diff < 0)
        return -1; // a < b
    else if (diff > 0)
        return 1; // a > b
    else
        return 0; // a == b
}