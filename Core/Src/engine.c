#include "engine.h"
#include "tim.h"

int ENGINE_MAX=1648;
int  ENGINE_CMPARER=1448;
int j=200;
int ENGINE_MIN=1248;

int pwm_data = 0; // PWM数据
// counter period需改为20000-1

// 设置舵机角度
// 传入-100~100，对应最高最低
void engine_setangle(int percentage)
{
    if (percentage > 100)
    {
        percentage = 100;
    }
    else if (percentage < -100)
    {
        percentage = -100;
    }

    pwm_data = (percentage + 100) * (ENGINE_MAX-ENGINE_MIN) / 200 + ENGINE_MIN;
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm_data);
}