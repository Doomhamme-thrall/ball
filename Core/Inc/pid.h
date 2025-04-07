#ifndef PID_H
#define PID_H

#include "main.h"
#include "tim.h"

typedef struct
{
    float Kp;                     // 比例系数
    float Ki;                     // 积分系数
    float Kd;                     // 微分系数
    float Last_Error;             // 上一次误差
    float Last_Integration;       // 上一次积分值
    float Last_Output;            // 上一次输出值
    unsigned long Timestamp_Last; // 上一次时间戳
    float IntegralMax;            // 积分限幅

} PID_Controller_t;

void pid_init(PID_Controller_t *pid, float Kp, float Ki, float Kd);
float PID_Controller(PID_Controller_t *pid, float target, float current);
float PID_Cal(PID_Controller_t *pid, float NowValue, float AimValue);

#endif /* PID_H */