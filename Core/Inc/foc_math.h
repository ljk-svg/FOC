#ifndef __FOC_MATH_H
#define __FOC_MATH_H

#include "main.h"
#include <math.h>

// ---------------- 宏定义常量 ----------------
#define PI               3.14159265358979f
#define SQRT3            1.73205080756887f
#define ONE_BY_SQRT3     0.57735026918963f  // 1 / 根号3
#define SQRT3_BY_2       0.86602540378443f  // 根号3 / 2

// ---------------- 核心数学库函数 (极致内联加速) ----------------

// 1. 标准库硬件 FPU 加速的三角函数
static inline void Fast_Sin_Cos_Hardware(float angle_rad, float *pSin, float *pCos)
{
    *pSin = sinf(angle_rad);
    *pCos = cosf(angle_rad);
}

// 2. Clark 变换
static inline void Clark_Transform(float Iu, float Iv, float *Ialpha, float *Ibeta)
{
    *Ialpha = Iu;
    *Ibeta = (Iu + 2.0f * Iv) * ONE_BY_SQRT3;
}

// 3. Park 变换
static inline void Park_Transform(float Ialpha, float Ibeta, float sin_val, float cos_val, float *Id, float *Iq)
{
    *Id = Ialpha * cos_val + Ibeta * sin_val;
    *Iq = -Ialpha * sin_val + Ibeta * cos_val;
}

// 4. 反 Park 变换
static inline void Rev_Park_Transform(float Vd, float Vq, float sin_val, float cos_val, float *Valpha, float *Vbeta)
{
    *Valpha = Vd * cos_val - Vq * sin_val;
    *Vbeta  = Vd * sin_val + Vq * cos_val;
}

// 5. SVPWM 空间矢量脉宽调制 (马鞍波生成器)
static inline void SVPWM_Generate(float Valpha, float Vbeta, float Vbus, float *Duty_U, float *Duty_V, float *Duty_W)
{
    float U = Valpha;
    float V = -0.5f * Valpha + SQRT3_BY_2 * Vbeta;
    float W = -0.5f * Valpha - SQRT3_BY_2 * Vbeta;

    float max_V = fmaxf(U, fmaxf(V, W));
    float min_V = fminf(U, fminf(V, W));
    float Vcom = -(max_V + min_V) * 0.5f;

    *Duty_U = (U + Vcom) / Vbus + 0.5f;
    *Duty_V = (V + Vcom) / Vbus + 0.5f;
    *Duty_W = (W + Vcom) / Vbus + 0.5f;

    if(*Duty_U > 1.0f) *Duty_U = 1.0f; else if(*Duty_U < 0.0f) *Duty_U = 0.0f;
    if(*Duty_V > 1.0f) *Duty_V = 1.0f; else if(*Duty_V < 0.0f) *Duty_V = 0.0f;
    if(*Duty_W > 1.0f) *Duty_W = 1.0f; else if(*Duty_W < 0.0f) *Duty_W = 0.0f;
}

#endif
