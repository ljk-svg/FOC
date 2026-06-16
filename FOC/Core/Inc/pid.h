#ifndef __PID_H
#define __PID_H

#include "main.h"

// ---------------- PID 核心结构体 ----------------
typedef struct {
    // 1. 调参三兄弟 (你需要手动试出来的参数)
    float Kp;           // 比例系数 (决定反应快慢)
    float Ki;           // 积分系数 (消除静态误差)
    float Kd;           // 微分系数 (通常电流环设为0)
    
    // 2. 记忆组件 (系统运行时自己更新的)
    float error_prev;   // 上一次的误差 (给微分用)
    float integral_sum; // 历史误差的累加和 (给积分用)
    
    // 3. 安全钳位限制 (保命符)
    float limit_integral; // 积分限幅 (防止死区饱和)
    float limit_output;   // 输出限幅 (防止输出电压爆表)
    
} PID_Controller_t;

// ---------------- 函数声明 ----------------

// 初始化一个 PID 控制器
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float limit_i, float limit_out);

// 核心计算函数 (在 FOC 中断里高频调用)
float PID_Calculate(PID_Controller_t *pid, float target, float current, float dt);

// 清空记忆 (当你关掉电机重新启动时调用，防止上次的积分残留)
void PID_Reset(PID_Controller_t *pid);

#endif
