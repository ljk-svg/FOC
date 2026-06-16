#include "ec11.h"
EC11_t EC11;

void EC11_Init(void) {
    EC11.Count = 0;
    EC11.Btn_Flag = 0;
    EC11.Last_A = READ_EC11_A();
    EC11.Btn_Timer = 0;
    EC11.Btn_State = 1; // 未按下是高电平
}

// 核心扫描函数 (利用 1ms 轮询天然防抖)
void EC11_Scan(void) {
    uint8_t current_a = READ_EC11_A();
    uint8_t current_b = READ_EC11_B();
    uint8_t current_sw = READ_EC11_SW();

    // ---------------- 1. 旋钮方向解码 ----------------
    // 只有当 A 相发生跳变时，才去检测 B 相
    if (current_a != EC11.Last_A) {
        // 我们只在 A 相的下降沿进行判断 (这样转一格只加减一次)
        if (current_a == 0) {
            // A下降沿时，如果 B 是高电平，说明是顺时针转
            if (current_b == 1) {
                EC11.Count++;
            } 
            // A下降沿时，如果 B 是低电平，说明是逆时针转
            else {
                EC11.Count--;
            }
        }
        EC11.Last_A = current_a; // 记录本次状态
    }

    // ---------------- 2. 按键消抖处理 ----------------
    // 按键按下是低电平
    if (current_sw == 0) {
        EC11.Btn_Timer++;
        // 连续 20ms 都是低电平，才认为真的按下了 (滤除机械杂波)
        if (EC11.Btn_Timer >= 20) {
            EC11.Btn_Timer = 20; // 防止溢出
            if (EC11.Btn_State == 1) { // 之前是抬起状态
                EC11.Btn_State = 0;    // 标记为已按下
                EC11.Btn_Flag = 1;     // 给外部应用层发信号
            }
        }
    } else {
        // 只要有高电平，立刻清零计时器
        EC11.Btn_Timer = 0;
        EC11.Btn_State = 1; // 标记按键抬起
    }
}


