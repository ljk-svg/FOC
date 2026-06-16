#include "pid.h"

/**
 * @brief 初始化 PID 控制器
 */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float limit_i, float limit_out)
{
    // 配置参数
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->limit_integral = limit_i;
    pid->limit_output = limit_out;
    
    // 清空历史记忆
    pid->error_prev = 0.0f;
    pid->integral_sum = 0.0f;
}

/**
 * @brief 清空 PID 的历史记忆 (积分清零)
 */
void PID_Reset(PID_Controller_t *pid)
{
    pid->error_prev = 0.0f;
    pid->integral_sum = 0.0f;
}

/**
 * @brief PID 核心计算 (包含积分抗饱和与输出限幅)
 * @param pid     控制器结构体指针
 * @param target  你想要的目标值 (比如你想让 Iq = 1.0A)
 * @param current ADC实际采样算出来的当前值 (比如当前的 Iq = 0.8A)
 * @param dt      两次计算的时间间隔 (比如你的 PWM 是 20kHz，那 dt 就是 0.00005 秒)
 * @return float  PID 算出来的补偿输出值 (比如要给电机加多少伏电压)
 */
float PID_Calculate(PID_Controller_t *pid, float target, float current, float dt)
{
    // 1. 算当前的偏差
    float error = target - current;

    // 2. 算比例项 (P)
    float proportional = pid->Kp * error;

    // 3. 算积分项 (I)
    // 积分 = 误差 * 时间。把每一次的小面积加起来
    pid->integral_sum += error * dt;
    
    // 【保命机制 1】：积分限幅 (Anti-Windup)
    // 绝对不能让误差无限放大，超过限制就强行截断
    if (pid->integral_sum > pid->limit_integral) {
        pid->integral_sum = pid->limit_integral;
    } else if (pid->integral_sum < -pid->limit_integral) {
        pid->integral_sum = -pid->limit_integral;
    }
    
    float integral = pid->Ki * pid->integral_sum;

    // 4. 算微分项 (D)
    // 微分 = (本次误差 - 上次误差) / 时间
    float derivative = pid->Kd * (error - pid->error_prev) / dt;

    // 5. 综合 P、I、D，算出总输出
    float output = proportional + integral + derivative;

    // 【保命机制 2】：输出限幅 (Output Clamp)
    // 如果算出来的电压超过了你设定的极限，强行截断
    if (output > pid->limit_output) {
        output = pid->limit_output;
    } else if (output < -pid->limit_output) {
        output = -pid->limit_output;
    }

    // 6. 更新记忆，留给下一次计算用
    pid->error_prev = error;

    return output;
}
