#include "laser.h"

/**
 * @brief 处理激光雷达数据帧
 * @param rxbuffer 接收缓冲区
 * @return 长度(mm) -1为错误
 */
int laser_processframe(uint8_t *rxbuffer)
{
    if (rxbuffer[0] == 0x01 && rxbuffer[1] == 0x03 && rxbuffer[2] == 0x02)
    {
        uint16_t data = (rxbuffer[3] << 8) | rxbuffer[4];
        if (data <= 2000)
        {
            return data;
        }
        else
        {
            return -1;
        }
    }
    return -1;
}