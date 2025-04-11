#include "motor.h"
#include "tim.h"
#include "pid.h"

#define moter_tim htim1
#define motor_gpio GPIOC

/**
 * @brief 电机初始化
 * @note 需要在TIM1的PWM模式下配置
 * @note TIM1_CH2 TIM1_CH3 TIM1_CH4
 * @note GPIOC6 GPIOC7 GPIOC8 GPIOC9
 */
void motor_init()
{
    HAL_TIM_PWM_Start(&moter_tim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&moter_tim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&moter_tim, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&moter_tim, TIM_CHANNEL_4);

    HAL_GPIO_WritePin(motor_gpio, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor_gpio, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor_gpio, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor_gpio, GPIO_PIN_9, GPIO_PIN_RESET);
}

/**
 * @brief 设置电机速度
 * @param motor_num 电机编号，取值为 1 2
 * @param percentage 速度百分比，范围 -100 ~ 100
 */
void motor_setspeed(int motor_num, int percentage)
{
    if (percentage > 100)
        percentage = 100;
    else if (percentage < -100)
        percentage = -100;

    int dutydata = __HAL_TIM_GetAutoreload(&moter_tim) * abs(percentage) / 100;

    if (motor_num == 1)
    {
        __HAL_TIM_SetCompare(&moter_tim, TIM_CHANNEL_1, dutydata);
        if (percentage >= 0)
        {
            HAL_GPIO_WritePin(motor_gpio, GPIO_PIN_6, GPIO_PIN_SET); // 正转
            HAL_GPIO_WritePin(motor_gpio, GPIO_PIN_7, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(motor_gpio, GPIO_PIN_6, GPIO_PIN_RESET); // 反转
            HAL_GPIO_WritePin(motor_gpio, GPIO_PIN_7, GPIO_PIN_SET);
        }
    }
    else if (motor_num == 2)
    {
        __HAL_TIM_SetCompare(&moter_tim, TIM_CHANNEL_2, dutydata);
        if (percentage >= 0)
        {
            HAL_GPIO_WritePin(motor_gpio, GPIO_PIN_8, GPIO_PIN_SET); // 正转
            HAL_GPIO_WritePin(motor_gpio, GPIO_PIN_9, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(motor_gpio, GPIO_PIN_8, GPIO_PIN_RESET); // 反转
            HAL_GPIO_WritePin(motor_gpio, GPIO_PIN_9, GPIO_PIN_SET);
        }
    }
}
