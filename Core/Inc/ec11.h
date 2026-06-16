#ifndef __EC11_H
#define __EC11_H

#include "main.h"

// ---------------- 物理引脚宏定义 ----------------
// 请根据你实际画板的引脚修改这里
#define EC11_A_PORT   GPIOB
#define EC11_A_PIN    GPIO_PIN_10
#define EC11_B_PORT   GPIOB
#define EC11_B_PIN    GPIO_PIN_11
#define EC11_SW_PORT  GPIOC
#define EC11_SW_PIN   GPIO_PIN_13

#define READ_EC11_A() HAL_GPIO_ReadPin(EC11_A_PORT, EC11_A_PIN)
#define READ_EC11_B() HAL_GPIO_ReadPin(EC11_B_PORT, EC11_B_PIN)
#define READ_EC11_SW() HAL_GPIO_ReadPin(EC11_SW_PORT, EC11_SW_PIN)

// ---------------- 数据结构 ----------------
typedef struct {
    volatile int32_t  Count;       // 旋钮计数值 (正代表顺时针，负代表逆时针)
    volatile uint8_t  Btn_Flag;    // 按键被按下的标志 (1:被按下了, 需在业务代码里手动清0)
    
    // 内部状态机变量，外部不用管
    uint8_t  Last_A;
    uint16_t Btn_Timer;
    uint8_t  Btn_State;
} EC11_t;

extern EC11_t EC11;

// ---------------- 接口函数 ----------------
void EC11_Init(void);
void EC11_Scan(void); // 必须放在 1ms 定时器中断里调用！

#endif
