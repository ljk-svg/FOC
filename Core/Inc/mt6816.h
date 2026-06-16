#ifndef __MT6816_H
#define __MT6816_H

#include "main.h"

// ---------------- 配置参数 ----------------
// 【极其重要】你的电机极对数！填错的话电机只会抖动转不起来。
// 常见的 2212 无刷航模电机极对数通常是 7。
#define MOTOR_POLE_PAIRS  11.0f 
#define PI                3.14159265358979f

// ---------------- 结构体定义 ----------------
typedef struct {
    uint16_t raw_data;        // 原始 14 位数据 (0 ~ 16383)
    float    angle_deg;       // 机械角度 (0 ~ 360.0度) -> 给人看、给上位机画图用的
    float    elec_angle_rad;  // 电角度 (弧度制，-PI ~ PI) -> 纯粹给 FOC 底层数学计算用的
} MT6816_t;

// 声明外部结构体，方便 main.c 和 FOC 中断随时调用
extern MT6816_t mt6816;

void MT6816_Init(void);
void MT6816_Update(void);

#endif
