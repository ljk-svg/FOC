#include "sensorless.h"
#include "foc_math.h"
#include "pid.h"
#include "oled.h"
#include "adc.h"
#include "tim.h"
#include <math.h>

// ==========================================
// 外部引用 (foc.c 中的 ADC 标定值)
// ==========================================
extern float offset_u;
extern float offset_v;
extern float k_amps;
extern float Vbus;

#define ARR_VALUE 4250.0f

// ==========================================
// 电机参数 (全局, 供 SMO / I/f 使用)
// ==========================================
Motor_Params_t motor_params = {
    .Rs        = 0.0f,
    .Ld        = 0.0f,
    .Lq        = 0.0f,
    .Flux      = 0.0f,
    .PolePairs = 11.0f
};

// ==========================================
// Phase 1: 电机参数辨识
// ==========================================

void Motor_Identify_RsLs(void)
{
    OLED_Clear();
    OLED_Printf(0, 0, "Motor Identify...");

    // ---- 确保 PWM 与 ADC 可用 ----
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    __HAL_TIM_MOE_ENABLE(&htim1);
    HAL_TIM_Base_Start(&htim1);

    // ---- Step 1: Rs 测量 (直流注入法) ----
    // 注入 Vα = 1.0V, Vβ = 0 (对应 θ=0, Vd=1.0V, Vq=0)
    // 转子会自动对齐到这个方向，静止后反电动势为零
    float V_test = 1.0f;
    float Valpha = V_test;
    float Vbeta  = 0.0f;

    float Duty_U, Duty_V, Duty_W;
    SVPWM_Generate(Valpha, Vbeta, Vbus, &Duty_U, &Duty_V, &Duty_W);
    TIM1->CCR1 = (uint32_t)(Duty_U * ARR_VALUE);
    TIM1->CCR2 = (uint32_t)(Duty_V * ARR_VALUE);
    TIM1->CCR3 = (uint32_t)(Duty_W * ARR_VALUE);
    TIM1->CCR4 = 4245;

    HAL_Delay(800);  // 等待转子对齐 + 电流稳定

    // 轮询 ADC, 取 200 次平均
    HAL_ADCEx_InjectedStart(&hadc1);
    HAL_ADCEx_InjectedStart(&hadc2);

    float sum_Ialpha = 0.0f;
    for (int i = 0; i < 200; i++) {
        while (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_JEOC) == RESET);
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
        uint32_t raw_u = hadc1.Instance->JDR1;
        uint32_t raw_v = hadc2.Instance->JDR1;

        float Iu = ((float)raw_u - offset_u) * k_amps;
        float Iv = ((float)raw_v - offset_v) * k_amps;
        float Ialpha, Ibeta;
        Clark_Transform(Iu, Iv, &Ialpha, &Ibeta);
        sum_Ialpha += Ialpha;
    }

    float Ialpha_avg = sum_Ialpha / 200.0f;
    if (Ialpha_avg < 0.01f) Ialpha_avg = 0.01f;  // 防止除零
    motor_params.Rs = V_test / Ialpha_avg;

    // ---- Step 2: Ls 测量 (电压阶跃法) ----
    // 从 V_test 阶跃到 V_test * 2.5, 测量电流上升斜率
    float V_step = V_test * 1.5f;  // 增量 1.5V
    Valpha = V_test + V_step;
    Vbeta  = 0.0f;

    SVPWM_Generate(Valpha, Vbeta, Vbus, &Duty_U, &Duty_V, &Duty_W);
    TIM1->CCR1 = (uint32_t)(Duty_U * ARR_VALUE);
    TIM1->CCR2 = (uint32_t)(Duty_V * ARR_VALUE);
    TIM1->CCR3 = (uint32_t)(Duty_W * ARR_VALUE);

    // 等 2 个 PWM 周期后开始高速采样
    HAL_Delay(1);

    // 采 4 个连续点, 每点间隔约 50us (一个 PWM 周期)
    float I_samples[4];
    for (int i = 0; i < 4; i++) {
        while (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_JEOC) == RESET);
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
        uint32_t raw_u = hadc1.Instance->JDR1;
        uint32_t raw_v = hadc2.Instance->JDR1;

        float Iu = ((float)raw_u - offset_u) * k_amps;
        float Iv = ((float)raw_v - offset_v) * k_amps;
        float Ialpha, Ibeta;
        Clark_Transform(Iu, Iv, &Ialpha, &Ibeta);
        I_samples[i] = Ialpha;
    }

    // 用前 3 个差分算斜率, 取中位数
    float dI_dt_sum = 0.0f;
    int valid_slopes = 0;
    for (int i = 1; i < 4; i++) {
        float dI = I_samples[i] - I_samples[i - 1];
        if (dI > 0.001f) {  // 过滤噪声
            dI_dt_sum += dI / 0.00005f;  // dt = 50us
            valid_slopes++;
        }
    }
    if (valid_slopes > 0) {
        float dI_dt_avg = dI_dt_sum / (float)valid_slopes;
        float Ls = V_step / dI_dt_avg;
        motor_params.Ld = Ls;
        motor_params.Lq = Ls;  // SPMSM: Ld ≈ Lq
    } else {
        motor_params.Ld = 0.00005f;  // 默认 50μH (A2212 典型值)
        motor_params.Lq = 0.00005f;
    }

    // ---- 清理 ----
    HAL_ADCEx_InjectedStop(&hadc1);
    HAL_ADCEx_InjectedStop(&hadc2);

    // 关闭 PWM 输出 (回到 50% 占空比, 等主循环接管)
    TIM1->CCR1 = (uint32_t)(0.5f * ARR_VALUE);
    TIM1->CCR2 = (uint32_t)(0.5f * ARR_VALUE);
    TIM1->CCR3 = (uint32_t)(0.5f * ARR_VALUE);

    // 在 OLED 上显示结果
    OLED_Clear();
    OLED_Printf(0, 0,  "Rs=%.3f Ohm", motor_params.Rs);
    OLED_Printf(0, 16, "Ls=%.1f uH",  motor_params.Ld * 1e6f);
    HAL_Delay(3000);
}

// ==========================================
// Phase 2: SMO 滑模观测器
// ==========================================

// ---- SMO 内部状态 ----
static float Ialpha_est = 0.0f;
static float Ibeta_est  = 0.0f;
static float Ealpha_filt = 0.0f;
static float Ebeta_filt  = 0.0f;
static float theta_est   = 0.0f;
static float omega_est   = 0.0f;
static int   smo_converged = 0;

// ---- PLL 锁相环 (复用 PID) ----
static PID_Controller_t PLL_PI;

// ---- 可调参数 ----
// k_slide: 滑模增益, 必须 > max(|Eα|, |Eβ|)。
// 对 12V 系统, BEMF 峰值约 10V, 设 k_slide = 15 留余量。
#define SMO_K_SLIDE     15.0f
// delta: sigmoid 边界层厚度 (A)。越小越接近符号函数, 抖振越大但收敛越快。
#define SMO_DELTA        0.15f
// BEMF 低通滤波器系数: alpha = 2*PI*fc*dt
// fc = 300Hz, dt = 50us → alpha ≈ 0.094
#define SMO_BEMF_ALPHA   0.094f
// PLL 收敛判定: 连续 N 次角度误差 < 阈值则认为收敛
#define SMO_CONVERGE_CNT 200

void SMO_Init(void)
{
    Ialpha_est = 0.0f;
    Ibeta_est  = 0.0f;
    Ealpha_filt = 0.0f;
    Ebeta_filt  = 0.0f;
    theta_est  = 0.0f;
    omega_est  = 0.0f;
    smo_converged = 0;

    // PLL: 只用 PI (Kd=0), 带宽约 30Hz
    PID_Init(&PLL_PI, 30.0f, 500.0f, 0.0f, 500.0f, 2000.0f);
}

void SMO_Update(float Valpha, float Vbeta, float Ialpha, float Ibeta, float dt)
{
    if (motor_params.Rs < 0.0001f || motor_params.Ld < 0.000001f) {
        // 电机参数未测量, 无法运行 SMO
        return;
    }

    float Rs = motor_params.Rs;
    float Ls = motor_params.Ld;  // SPMSM: Ld≈Lq, 用 Ld
    float inv_Ls = 1.0f / Ls;

    // ---- Step 1: 电流观测器 (前向欧拉离散化) ----
    // 模型: dI/dt = (-Rs/Ls)*I + (1/Ls)*(V - E)
    // 其中 E 由滑模开关函数 Z 代替

    float Ialpha_err = Ialpha_est - Ialpha;
    float Ibeta_err  = Ibeta_est  - Ibeta;

    // Sigmoid 开关函数 (平滑, 减少抖振)
    float Zalpha = SMO_K_SLIDE * Ialpha_err / (fabsf(Ialpha_err) + SMO_DELTA);
    float Zbeta  = SMO_K_SLIDE * Ibeta_err  / (fabsf(Ibeta_err)  + SMO_DELTA);

    // 前向欧拉更新
    Ialpha_est += (-Rs * inv_Ls * Ialpha_est + inv_Ls * (Valpha - Zalpha)) * dt;
    Ibeta_est  += (-Rs * inv_Ls * Ibeta_est  + inv_Ls * (Vbeta  - Zbeta))  * dt;

    // ---- Step 2: LPF 提取反电动势 ----
    // 开关函数 Z 经低通滤波 = 反电动势估计
    Ealpha_filt += SMO_BEMF_ALPHA * (Zalpha - Ealpha_filt);
    Ebeta_filt  += SMO_BEMF_ALPHA * (Zbeta  - Ebeta_filt);

    // ---- Step 3: PLL 锁相环提取角度和速度 ----
    // 归一化角度误差: err = Eβ·cos(θ) - Eα·sin(θ)
    // 简化正交误差 (假设 |E| 不为零时, err ∝ Δθ)
    float sin_val, cos_val;
    Fast_Sin_Cos_Hardware(theta_est, &sin_val, &cos_val);
    float E_mag = sqrtf(Ealpha_filt * Ealpha_filt + Ebeta_filt * Ebeta_filt);

    float theta_err = 0.0f;
    if (E_mag > 0.01f) {
        // 归一化: 除以 |E| 使误差与速度无关
        theta_err = (Ebeta_filt * cos_val - Ealpha_filt * sin_val) / E_mag;
    }

    // PI 控制器输出 = 估计电角速度 (rad/s)
    omega_est = PID_Calculate(&PLL_PI, 0.0f, theta_err, dt);

    // 角度积分
    theta_est += omega_est * dt;
    if (theta_est > 2.0f * PI)  theta_est -= 2.0f * PI;
    if (theta_est < 0.0f)       theta_est += 2.0f * PI;

    // ---- Step 4: 收敛判定 ----
    static int converge_cnt = 0;
    if (fabsf(theta_err) < 0.08f) {  // 误差 < 约 4.5°
        converge_cnt++;
        if (converge_cnt > SMO_CONVERGE_CNT) {
            smo_converged = 1;
        }
    } else {
        converge_cnt = 0;
        smo_converged = 0;
    }
}

float SMO_Get_Angle(void)
{
    return theta_est;
}

float SMO_Get_Speed(void)
{
    return omega_est;
}

int SMO_Is_Converged(void)
{
    return smo_converged;
}

// ==========================================
// Phase 3: I/f 开环启动
// ==========================================

// ---- I/f 内部状态 ----
static float if_theta_virtual  = 0.0f;
static float if_omega_current  = 0.0f;
static float if_omega_target   = 0.0f;
static float if_omega_ramp     = 100.0f;    // 加速斜率 (rad/s²)
static int   if_active         = 0;
static int   if_ramp_done      = 0;

// 切换相关
static float if_blend_weight   = 0.0f;
static int   if_blend_active   = 0;
static float if_blend_duration = 0.5f;      // 混合时间 500ms
static float if_blend_elapsed  = 0.0f;

// I/f 推力电流
#define IF_STARTUP_CURRENT  0.4f   // A

void IF_Startup_Begin(float target_speed_elec)
{
    if_theta_virtual = 0.0f;
    if_omega_current = 0.0f;
    if_omega_target  = target_speed_elec;
    if_active        = 1;
    if_ramp_done     = 0;
    if_blend_active  = 0;
    if_blend_weight  = 0.0f;
    if_blend_elapsed = 0.0f;

    // 同步 SMO 初始角度
    theta_est = 0.0f;
}

void IF_Startup_Update(float dt)
{
    if (!if_active) return;

    // ---- 虚拟角度匀速加速 ----
    if (!if_ramp_done) {
        if_omega_current += if_omega_ramp * dt;
        if (if_omega_current >= if_omega_target) {
            if_omega_current = if_omega_target;
            if_ramp_done = 1;
        }
    }

    if_theta_virtual += if_omega_current * dt;
    if (if_theta_virtual > 2.0f * PI)  if_theta_virtual -= 2.0f * PI;
    if (if_theta_virtual < 0.0f)       if_theta_virtual += 2.0f * PI;

    // ---- 切换判定: SMO 收敛 + 角度误差 < 15° 持续 ----
    if (if_ramp_done && SMO_Is_Converged()) {
        float angle_err = if_theta_virtual - SMO_Get_Angle();
        // 归一化到 [-PI, PI]
        if (angle_err > PI)  angle_err -= 2.0f * PI;
        if (angle_err < -PI) angle_err += 2.0f * PI;

        if (fabsf(angle_err) < 0.26f) {  // ~15°
            if (!if_blend_active) {
                if_blend_active = 1;
                if_blend_elapsed = 0.0f;
                if_blend_weight  = 0.0f;
            }
        }
    }

    // ---- 混合切换 ----
    if (if_blend_active) {
        if_blend_elapsed += dt;
        if_blend_weight = if_blend_elapsed / if_blend_duration;
        if (if_blend_weight > 1.0f) {
            if_blend_weight = 1.0f;
            if_active = 0;  // I/f 退出, 完全交给 SMO
        }
    }
}

float IF_Get_Angle(void)
{
    return if_theta_virtual;
}

int IF_Is_Done(void)
{
    return !if_active;
}

// ==========================================
// 无感控制顶层
// ==========================================
static Sensorless_State_t sl_state = SENSORLESS_IDLE;
static float prev_Valpha = 0.0f;
static float prev_Vbeta  = 0.0f;

void Sensorless_Enter(void)
{
    SMO_Init();
    sl_state = SENSORLESS_IF;
    // 启动 I/f: 目标电角速度 80 rad/s ≈ 70 RPM 机械
    IF_Startup_Begin(80.0f);
    prev_Valpha = 0.0f;
    prev_Vbeta  = 0.0f;
}

void Sensorless_Exit(void)
{
    sl_state = SENSORLESS_IDLE;
}

void Sensorless_Control(float Valpha, float Vbeta, float Ialpha, float Ibeta, float dt)
{
    // 始终运行 SMO (即使还在 I/f 阶段, 让它先收敛)
    SMO_Update(prev_Valpha, prev_Vbeta, Ialpha, Ibeta, dt);
    // 保存当前输出电压给下一周期 SMO 使用
    prev_Valpha = Valpha;
    prev_Vbeta  = Vbeta;

    // 更新 I/f 状态机
    IF_Startup_Update(dt);

    // 状态机
    switch (sl_state) {
        case SENSORLESS_IDLE:
            break;

        case SENSORLESS_IF:
            if (IF_Is_Done()) {
                sl_state = SENSORLESS_BLEND;
            }
            break;

        case SENSORLESS_BLEND:
            // 混合权重由 IF_Startup_Update 计算
            break;

        case SENSORLESS_CLOSED:
            break;
    }

    // 判断是否完全进入闭环
    if (sl_state == SENSORLESS_BLEND && if_blend_weight >= 1.0f) {
        sl_state = SENSORLESS_CLOSED;
    }
}

float Sensorless_Get_Angle(void)
{
    switch (sl_state) {
        case SENSORLESS_IDLE:
            return 0.0f;
        case SENSORLESS_IF:
            return IF_Get_Angle();
        case SENSORLESS_BLEND:
            // 权重混合
            return if_blend_weight * SMO_Get_Angle()
                 + (1.0f - if_blend_weight) * IF_Get_Angle();
        case SENSORLESS_CLOSED:
            return SMO_Get_Angle();
        default:
            return 0.0f;
    }
}

float Sensorless_Get_Speed(void)
{
    if (sl_state == SENSORLESS_CLOSED || sl_state == SENSORLESS_BLEND) {
        return SMO_Get_Speed();
    }
    return if_omega_current;
}

Sensorless_State_t Sensorless_Get_State(void)
{
    return sl_state;
}

// ==========================================
// Phase 5: HFI 高频注入 (实验性, #ifdef HFI_ENABLE)
// ==========================================
#ifdef HFI_ENABLE

// HFI 参数
#define HFI_VOLTAGE     1.5f     // 注入电压幅值 (V)
#define HFI_FREQ_HZ     1000.0f  // 注入频率 (Hz)
#define HFI_OMEGA_H     6283.0f  // 2*PI*1000
#define HFI_BPF_ALPHA   0.02f    // 带通滤波器系数 (粗略实现)
#define HFI_LPF_ALPHA   0.01f    // 解调后低通系数

static float hfi_theta_est = 0.0f;
static float hfi_omega_est = 0.0f;
static float hfi_phase_acc  = 0.0f;  // 高频注入相位累积
static float hfi_Iq_bpf     = 0.0f;  // q轴电流带通滤波值
static float hfi_demod      = 0.0f;  // 解调输出

static PID_Controller_t HFI_PLL;

void HFI_Init(void)
{
    hfi_theta_est = 0.0f;
    hfi_omega_est = 0.0f;
    hfi_phase_acc = 0.0f;
    hfi_Iq_bpf    = 0.0f;
    hfi_demod     = 0.0f;

    PID_Init(&HFI_PLL, 10.0f, 100.0f, 0.0f, 500.0f, 2000.0f);
}

void HFI_Update(float *Vd_hfi, float *Vq_hfi, float Iq_meas, float dt)
{
    // 注入相位累积
    hfi_phase_acc += HFI_OMEGA_H * dt;
    if (hfi_phase_acc > 2.0f * PI) hfi_phase_acc -= 2.0f * PI;

    // 脉振正弦电压注入到估算 d 轴
    *Vd_hfi = HFI_VOLTAGE * cosf(hfi_phase_acc);
    *Vq_hfi = 0.0f;

    // 简易带通: 对 Iq 做高通 + 低通 (一阶近似)
    // 实际需要用 IIR 带通, 这里先用高通近似代替
    static float Iq_prev = 0.0f;
    float Iq_highpass = Iq_meas - Iq_prev;  // 简易高通 (差分)
    Iq_prev = Iq_meas;
    hfi_Iq_bpf += HFI_BPF_ALPHA * (Iq_highpass - hfi_Iq_bpf);  // 低通平滑

    // 解调: 乘以 sin(ω_h * t), 再低通
    float demod_raw = hfi_Iq_bpf * sinf(hfi_phase_acc);
    hfi_demod += HFI_LPF_ALPHA * (demod_raw - hfi_demod);

    // PLL 追踪: 解调输出 ∝ (Ld-Lq)*sin(2Δθ)
    // 误差驱动到零
    float pll_err = hfi_demod;
    hfi_omega_est = PID_Calculate(&HFI_PLL, 0.0f, pll_err, dt);
    hfi_theta_est += hfi_omega_est * dt;
    if (hfi_theta_est > 2.0f * PI)  hfi_theta_est -= 2.0f * PI;
    if (hfi_theta_est < 0.0f)       hfi_theta_est += 2.0f * PI;
}

float HFI_Get_Angle(void)
{
    return hfi_theta_est;
}

#endif  // HFI_ENABLE
