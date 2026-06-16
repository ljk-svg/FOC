#ifndef __FOC_H
#define __FOC_H

#include "main.h"
#include "sensorless.h"

// ==========================================
// 外部可调参数 (开放给 Menu 或 main 修改)
// ==========================================
extern float target_Iq;     // 目标力矩电流 (A)
extern FOC_Mode_t foc_mode; // 有感/无感模式切换
extern float Target_Speed;  // 目标机械角速度 (rad/s, 由 menu 控制)

// ==========================================
// ADC 标定值 (供 sensorless 测量使用)
// ==========================================
extern float offset_u;
extern float offset_v;
extern float k_amps;
extern float Vbus;

// ==========================================
// 核心接口函数
// ==========================================
void FOC_Calibrate_ADC(void); // 第一步：ADC 零点偏置校准
void FOC_Align_Zero(void);    // 第二步：转子电磁零点对齐标定
void FOC_Init(void);          // 第三步：初始化 PID 等参数
void FOC_Control_Loop(void);  // 第四步：闭环核心计算 (放在中断里)
void Diagnostic_Test_ADC(void);
#endif
