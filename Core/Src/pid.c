#include "pid.h"
float op = 60;
float ip = 15;
float oi = 1;
float ii = 0;
float od = 300;
float id = 800;
float thereshold = 0.03f; // 误差阈值

// PID控制器的初始化函数
void pid_init(PID_Controller_t *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Last_Error = 0;
    pid->Last_Integration = 0;
    pid->Last_Output = 0;
    pid->Timestamp_Last = HAL_GetTick();
    pid->IntegralMax = 10; // 积分限幅
}

float iError; // 当前误差
float Output; // 控制输出

/**
 * PID控制器的计算函数
 * @param pid PID控制器结构体指针
 * @param NowValue 当前值
 * @param AimValue 目标值
 * @return 控制输出值
 */
float PID_Cal(PID_Controller_t *pid, float NowValue, float AimValue)
{

    // 分段p
    if (fabs(pid->Last_Error) < thereshold)
    {
        pid->Kp = ip;
        pid->Kd = id;
        pid->Ki = ii;
        // pid->Last_Integration = 0;
    }
    else
    {
        pid->Kp = op;
        pid->Kd = od;
        pid->Ki = oi;
    }

    // 计算当前误差
    iError = AimValue - NowValue;

    // 位置式 PID 积分项累加
    pid->Last_Integration += pid->Ki * iError;

    // 积分项限幅
    if (pid->Last_Integration > pid->IntegralMax)
        pid->Last_Integration = pid->IntegralMax;
    else if (pid->Last_Integration < -pid->IntegralMax)
        pid->Last_Integration = -pid->IntegralMax;

    // 计算 PID 输出
    Output = pid->Kp * iError;                      // P
    Output += pid->Kd * (iError - pid->Last_Error); // D
    Output += pid->Last_Integration;                // I

    // 更新上次误差
    pid->Last_Error = iError;

    return Output; // 返回控制输出值
}
