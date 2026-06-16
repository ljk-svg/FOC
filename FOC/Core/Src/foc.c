#include "foc.h"
#include "adc.h"
#include "tim.h"
#include "oled.h"
#include "mt6816.h"
#include "pid.h"
#include "foc_math.h"
#include "vofa.h"
#include "menu.h"
#include "sensorless.h"
#include <math.h>

// ==========================================
// 内部私有变量
// ==========================================
static PID_Controller_t PID_Id;
static PID_Controller_t PID_Iq;
static PID_Controller_t PID_Speed;
float offset_u = 2048.0f;
float offset_v = 2048.0f;
static float zero_electrical_offset = 0.0f;

float k_amps = 0.008f;
float Vbus = 12.0f;
#define ARR_VALUE 4250.0f

// 开放给外部的变量
float target_Iq = 0.2f;
FOC_Mode_t foc_mode = FOC_SENSORED;  // 默认有感模式 

// ==========================================
// 1. ADC 零点偏置校准
// ==========================================
void FOC_Calibrate_ADC(void)
{
    OLED_Clear();
    OLED_Printf(0, 0, "Calibrating ADC...");
    
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    __HAL_TIM_MOE_ENABLE(&htim1); // 合闸通电

    TIM1->CCR1 = (uint32_t)(0.5f * ARR_VALUE);
    TIM1->CCR2 = (uint32_t)(0.5f * ARR_VALUE);
    TIM1->CCR3 = (uint32_t)(0.5f * ARR_VALUE);
    TIM1->CCR4 = 4245;
    HAL_Delay(100); 

    HAL_ADCEx_InjectedStart(&hadc1); 
    HAL_ADCEx_InjectedStart(&hadc2);    
    HAL_TIM_Base_Start(&htim1);

    uint32_t sum_u = 0, sum_v = 0;
    for(int i = 0; i < 1000; i++) {
        while(__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_JEOC) == RESET);
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC); 
        sum_u += hadc1.Instance->JDR1;
        sum_v += hadc2.Instance->JDR1;
    }

    HAL_TIM_Base_Stop(&htim1); 
    HAL_ADCEx_InjectedStop(&hadc1);
    HAL_ADCEx_InjectedStop(&hadc2);
    
    offset_u = (float)sum_u / 1000.0f;
    offset_v = (float)sum_v / 1000.0f;
}

// ==========================================
// 2. 转子绝对零点对齐标定
// ==========================================

void FOC_Align_Zero(void)
{
    OLED_Clear();
    OLED_Printf(0, 0, "Aligning Rotor...");
    
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    __HAL_TIM_MOE_ENABLE(&htim1);

    float sin_val, cos_val;
    Fast_Sin_Cos_Hardware(0.0f, &sin_val, &cos_val);
    
    // 给 2.0V 电压对齐，防烧且足够吸住
    float Valpha, Vbeta;
    Rev_Park_Transform(2.0f, 0.0f, sin_val, cos_val, &Valpha, &Vbeta); 
    
    float Duty_U, Duty_V, Duty_W;
    SVPWM_Generate(Valpha, Vbeta, Vbus, &Duty_U, &Duty_V, &Duty_W);

    TIM1->CCR1 = (uint32_t)(Duty_U * ARR_VALUE);
    TIM1->CCR2 = (uint32_t)(Duty_V * ARR_VALUE);
    TIM1->CCR3 = (uint32_t)(Duty_W * ARR_VALUE);

    HAL_Delay(2000); 

    MT6816_Update();
    zero_electrical_offset = mt6816.elec_angle_rad; 

    TIM1->CCR1 = (uint32_t)(0.5f * ARR_VALUE);
    TIM1->CCR2 = (uint32_t)(0.5f * ARR_VALUE);
    TIM1->CCR3 = (uint32_t)(0.5f * ARR_VALUE);
}

// ==========================================
// 3. FOC 控制器初始化
// ==========================================
void FOC_Init(void)
{
    // Kp 降到 0.2，把最高输出电压死死锁在 2.5V！绝对不允许超过！
    PID_Init(&PID_Id, 0.2f, 2.0f, 0.0f, 2.5f, 2.5f); 
    PID_Init(&PID_Iq, 0.2f, 2.0f, 0.0f, 2.5f, 2.5f);
    PID_Init(&PID_Speed, 0.1f, 1.0f, 0.0f, 3.0f, 3.0f);
    OLED_Clear();
    OLED_Printf(0, 0, "=== FOC Ready ===");
}

// 20kHz 的频率，dt_current 就是 0.00005 秒
const float dt_current = 0.00005f;
// 1kHz 的频率，dt_speed 就是 0.001 秒
const float dt_speed = 0.001f;

void FOC_Control_Loop(void)
{
    // ==========================================
    // 1. 获取电角度 (有感: 编码器 / 无感: SMO)
    // ==========================================
    float elec_angle;
    float motor_speed_val;  // 机械角速度 (rad/s)

    if (foc_mode == FOC_SENSORED) {
        MT6816_Update();
        elec_angle = mt6816.elec_angle_rad - zero_electrical_offset;
        while (elec_angle < 0.0f)      { elec_angle += 2.0f * PI; }
        while (elec_angle >= 2.0f * PI) { elec_angle -= 2.0f * PI; }
    } else {
        // 无感模式: SMO 估算的角度
        elec_angle = Sensorless_Get_Angle();
    }

    // ==========================================
    // 2. 读取 ADC 电流 + Clark 变换
    // ==========================================
    uint32_t raw_u = hadc1.Instance->JDR1;
    uint32_t raw_v = hadc2.Instance->JDR1;

    float Iu = ((float)raw_u - offset_u) * k_amps;
    float Iv = ((float)raw_v - offset_v) * k_amps;

    float Ialpha, Ibeta;
    Clark_Transform(Iu, Iv, &Ialpha, &Ibeta);

    // ==========================================
    // 3. Park 变换
    // ==========================================
    float sin_val, cos_val;
    Fast_Sin_Cos_Hardware(elec_angle, &sin_val, &cos_val);

    float Id, Iq;
    Park_Transform(Ialpha, Ibeta, sin_val, cos_val, &Id, &Iq);

    // ==========================================
    // 4. 速度环降频执行 (1kHz)
    // ==========================================
    static uint16_t speed_loop_cnt = 0;
    static float target_Iq_spd = 0.0f;
    static float last_angle = 0.0f;

    speed_loop_cnt++;
    if (speed_loop_cnt >= 20)
    {
        speed_loop_cnt = 0;

        if (foc_mode == FOC_SENSORED) {
            // 有感: 编码器差分算速度
            float delta_angle = elec_angle - last_angle;
            if (delta_angle > PI)  delta_angle -= 2.0f * PI;
            if (delta_angle < -PI) delta_angle += 2.0f * PI;
            last_angle = elec_angle;

            float elec_speed = delta_angle / dt_speed;
            motor_speed_val = elec_speed / MOTOR_POLE_PAIRS;
        } else {
            // 无感: SMO 直接给速度
            float elec_speed = Sensorless_Get_Speed();
            motor_speed_val = elec_speed / MOTOR_POLE_PAIRS;
        }

        // 平滑滤波
        static float motor_speed_filt = 0.0f;
        motor_speed_filt = 0.95f * motor_speed_filt + 0.05f * motor_speed_val;

        // 无感 I/f 阶段: 直接用固定推力电流, 不走速度环
        if (foc_mode == FOC_SENSORLESS && Sensorless_Get_State() < SENSORLESS_CLOSED) {
            target_Iq_spd = 0.4f;  // I/f 启动电流
        } else {
            // 速度环 PID
            target_Iq_spd = PID_Calculate(&PID_Speed, Target_Speed, motor_speed_filt, dt_speed);
            if (target_Iq_spd > 0.5f)  target_Iq_spd = 0.5f;
            if (target_Iq_spd < -0.5f) target_Iq_spd = -0.5f;
        }
    }

    // ==========================================
    // 5. 电流环闭环 (20kHz)
    // ==========================================
    float target_Id = 0.0f;
    float Vd = PID_Calculate(&PID_Id, target_Id, Id, dt_current);
    float Vq = PID_Calculate(&PID_Iq, target_Iq_spd, Iq, dt_current);

    // ==========================================
    // 6. 反 Park 变换
    // ==========================================
    float Valpha, Vbeta;
    Rev_Park_Transform(Vd, Vq, sin_val, cos_val, &Valpha, &Vbeta);

    // ==========================================
    // 7. 无感控制更新 (SMO 观测器 + I/f 状态机)
    // ==========================================
    if (foc_mode == FOC_SENSORLESS) {
        Sensorless_Control(Valpha, Vbeta, Ialpha, Ibeta, dt_current);
    }

    // ==========================================
    // 8. SVPWM 发波
    // ==========================================
    float Duty_U, Duty_V, Duty_W;
    SVPWM_Generate(Valpha, Vbeta, Vbus, &Duty_U, &Duty_V, &Duty_W);

    TIM1->CCR1 = (uint32_t)(Duty_U * ARR_VALUE);
    TIM1->CCR2 = (uint32_t)(Duty_V * ARR_VALUE);
    TIM1->CCR3 = (uint32_t)(Duty_W * ARR_VALUE);
}
