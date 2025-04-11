#include "jy901s.h"
#include "usart.h"
#include "stdio.h"

#define JY901S_UART huart1

static JY901S_Data_t jy901s_data;

uint8_t cmds[][5] = {
    {0xff, 0xaa, 0x69, 0x88, 0xb5}, // 0解锁指令
    {0xff, 0xaa, 0x00, 0x01, 0x00}, // 1恢复默认设置
    {0xff, 0xaa, 0x00, 0x00, 0x00}, // 2保存
    {0xff, 0xaa, 0x01, 0x01, 0x00}, // 3加速度计校准
    {0xff, 0xaa, 0x01, 0x02, 0x00}, // 4磁场校准
    {0xff, 0xaa, 0x01, 0x03, 0x00}, // 5高度归零
    {0xff, 0xaa, 0x23, 0x00, 0x00}, // 6安装方向水平
    {0xff, 0xaa, 0x22, 0x01, 0x00}, // 7解除休眠
    {0xff, 0xaa, 0x24, 0x01, 0x00}, // 8算法转换
    {0xff, 0xaa, 0x63, 0x00, 0x00}, // 9陀螺仪自动校准
    {0xff, 0xaa, 0x02, 0xff, 0xff}, // 10设置回传内容
    {0xff, 0xaa, 0x03, 0x09, 0x00}  // 11设置回传速率
};

void jy901s_init()
{
    HAL_UART_Transmit(&JY901S_UART, cmds[0], 5, 100);
    HAL_Delay(100);
    HAL_UART_Transmit(&JY901S_UART, cmds[1], 5, 100);
    HAL_Delay(100);
    HAL_UART_Transmit(&JY901S_UART, cmds[10], 5, 100);
    HAL_Delay(100);
    HAL_UART_Transmit(&JY901S_UART, cmds[2], 5, 100);
    HAL_Delay(100);
    HAL_UART_Transmit(&JY901S_UART, cmds[5], 5, 100);
    HAL_Delay(100);
}

static void jy901s_processframe(uint8_t *rxbuffer)
{

}



void jy901s_getdata(uint8_t *rxbuffer, JY901S_Data_t *data)
{
}
