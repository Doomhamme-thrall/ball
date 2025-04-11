#ifndef JY901S_H
#define JY901S_H

#include "main.h"
typedef struct
{
    float m, s, ms;         // 时间 (分:秒:毫秒)
    float ax, ay, az;       // 加速度 (g)
    float wx, wy, wz;       // 角速度 (°/s)
    float roll, pitch, yaw; // 角度 (°)
} JY901S_Data_t;

#endif