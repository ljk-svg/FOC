#ifndef __SENSORLESS_H
#define __SENSORLESS_H

#include "main.h"

// ==========================================
// FOC 运行模式
// ==========================================
typedef enum {
    FOC_SENSORED   = 0,  // 有感闭环 (MT6816 编码器)
    FOC_SENSORLESS = 1   // 无感闭环 (SMO 观测器)
} FOC_Mode_t;

// ==========================================
// 电机参数结构体
// ==========================================
typedef struct {
    float Rs;         // 定子电阻 (Ω)
    float Ld;         // d轴电感 (H)
    float Lq;         // q轴电感 (H)
    float Flux;       // 永磁体磁链 (Wb)，可后期从反电动势推算
    float PolePairs;  // 极对数
} Motor_Params_t;

// ==========================================
// 无感运行状态
// ==========================================
typedef enum {
    SENSORLESS_IDLE    = 0,  // 空闲/有感模式
    SENSORLESS_IF      = 1,  // I/f 开环强拖阶段
    SENSORLESS_BLEND   = 2,  // I/f → SMO 混合切换中
    SENSORLESS_CLOSED  = 3   // 纯 SMO 闭环运行
} Sensorless_State_t;

// ==========================================
// Phase 1: 电机参数辨识
// ==========================================
extern Motor_Params_t motor_params;

void Motor_Identify_RsLs(void);  // 直流注入法测 Rs + 阶跃法测 Ls

// ==========================================
// Phase 2: SMO 滑模观测器
// ==========================================
void SMO_Init(void);
void SMO_Update(float Valpha, float Vbeta, float Ialpha, float Ibeta, float dt);
float SMO_Get_Angle(void);
float SMO_Get_Speed(void);
int   SMO_Is_Converged(void);

// ==========================================
// Phase 3: I/f 开环启动
// ==========================================
void IF_Startup_Begin(float target_speed_elec);  // 目标速度: 电角速度 (rad/s)
void IF_Startup_Update(float dt);                 // 每周期调用，更新虚拟角度
float IF_Get_Angle(void);
int   IF_Is_Done(void);

// ==========================================
// 无感控制顶层接口
// ==========================================
void Sensorless_Control(float Valpha, float Vbeta, float Ialpha, float Ibeta, float dt);
float Sensorless_Get_Angle(void);
float Sensorless_Get_Speed(void);
Sensorless_State_t Sensorless_Get_State(void);
void Sensorless_Enter(void);   // 进入无感模式
void Sensorless_Exit(void);    // 退出无感模式

// ==========================================
// Phase 5: HFI 高频注入 (实验性)
// ==========================================
#ifdef HFI_ENABLE
void HFI_Init(void);
void HFI_Update(float *Vd_hfi, float *Vq_hfi, float Iq_meas, float dt);
float HFI_Get_Angle(void);
#endif

#endif
