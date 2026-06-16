#include "oled.h"
#include "bsp_i2c.h"  // 核心：引入底层 I2C 头文件
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "OLED_FONT.h" // 必须包含你的字库文件
// ---------------- 封装 OLED 写命令和写数据接口 ----------------
static void OLED_WriteCmd(uint8_t cmd)
{
    I2C_Start();
    I2C_SendByte(OLED_ADDRESS); 
    I2C_SendByte(0x00); // 0x00 表示后面是命令
    I2C_SendByte(cmd);
    I2C_Stop();
}

static void OLED_WriteData(uint8_t data)
{
    I2C_Start();
    I2C_SendByte(OLED_ADDRESS);
    I2C_SendByte(0x40); // 0x40 表示后面是数据
    I2C_SendByte(data);
    I2C_Stop();
}

// ---------------- OLED 业务逻辑代码 ----------------
void OLED_Clear(void)
{
    for (uint8_t i = 0; i < 8; i++) {
        OLED_WriteCmd(0xB0 + i); 
        OLED_WriteCmd(0x00);     
        OLED_WriteCmd(0x10);     
        for (uint8_t n = 0; n < 128; n++) {
            OLED_WriteData(0x00);
        }
    }
}

void OLED_Init(void)
{
    HAL_Delay(100); 

    OLED_WriteCmd(0xAE); // 关闭显示
    OLED_WriteCmd(0x20); // 设置内存寻址模式
    OLED_WriteCmd(0x10); // 页寻址
    OLED_WriteCmd(0xB0); // 起始页
    OLED_WriteCmd(0xC8); // 倒置
    OLED_WriteCmd(0x00); // 低列
    OLED_WriteCmd(0x10); // 高列
    OLED_WriteCmd(0x40); // 起始行
    OLED_WriteCmd(0x81); // 对比度
    OLED_WriteCmd(0xFF); // 亮度
    OLED_WriteCmd(0xA1); // 左右反置
    OLED_WriteCmd(0xA6); // 正常显示
    OLED_WriteCmd(0xA8); // 多路复用率
    OLED_WriteCmd(0x3F); // 1/64
    OLED_WriteCmd(0xA4); // 全局显示
    OLED_WriteCmd(0xD3); // 显示偏移
    OLED_WriteCmd(0x00); 
    OLED_WriteCmd(0xD5); // 时钟分频
    OLED_WriteCmd(0xF0); 
    OLED_WriteCmd(0xD9); // 预充电
    OLED_WriteCmd(0x22); 
    OLED_WriteCmd(0xDA); // COM引脚
    OLED_WriteCmd(0x12);
    OLED_WriteCmd(0xDB); // VCOMH
    OLED_WriteCmd(0x20);
    OLED_WriteCmd(0x8D); // 电荷泵
    OLED_WriteCmd(0x14); 
    OLED_WriteCmd(0xAF); // 开启显示

    OLED_Clear();
}
// 设置光标位置 (X:0~127, Y:0~7)
void OLED_SetCursor(uint8_t x, uint8_t y)
{
    OLED_WriteCmd(0xB0 + y);
    OLED_WriteCmd(((x & 0xF0) >> 4) | 0x10);
    OLED_WriteCmd((x & 0x0F));
}


// 新的 6x8 字符显示函数
void OLED_ShowChar(uint8_t x, uint8_t y, char chr)
{
    uint8_t c = chr - ' '; 
    uint8_t i;

    OLED_SetCursor(x, y);
    // 6x8 字体只有 6 列，高度刚好 1 页 (8 像素)，所以一个 for 循环就搞定！
    for (i = 0; i < 6; i++) {
        OLED_WriteData(OLED_F6x8[c][i]); 
    }
}

void OLED_ShowString(uint8_t x, uint8_t y, char *str)
{
    while (*str != '\0') {
        OLED_ShowChar(x, y, *str);
        
        x += 6; // 【修改点1】字体宽度变成 6 了，所以每次向右移 6
        
        // 自动换行保护
        if (x > 122) { // 128 - 6 = 122，防止最后一个字越界
            x = 0; 
            y += 1; // 【修改点2】字体高度变成 8 了 (刚好1页)，所以换行只加 1！
        } 
        
        str++; 
    }
}

// 终极 Printf 封装 (同步去掉了 size 参数)
void OLED_Printf(uint8_t x, uint8_t y, const char *format, ...)
{
    char StringBuf[32]; 
    va_list args;
    va_start(args, format);
    vsnprintf(StringBuf, sizeof(StringBuf), format, args);
    va_end(args);
    OLED_ShowString(x, y, StringBuf);
}
