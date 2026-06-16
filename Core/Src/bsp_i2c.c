#include "bsp_i2c.h"

// 软件微秒级延时 (适配 G431 的 170MHz 主频)
static void I2C_Delay(void)
{
    uint32_t i = 30; 
    while(i--) { __NOP(); }
}

void I2C_Start(void)
{
    I2C_SDA_H();
    I2C_SCL_H();
    I2C_Delay();
    I2C_SDA_L();
    I2C_Delay();
    I2C_SCL_L();
}

void I2C_Stop(void)
{
    I2C_SDA_L();
    I2C_SCL_H();
    I2C_Delay();
    I2C_SDA_H();
}

uint8_t I2C_WaitAck(void)
{
    uint8_t ack = 0;
    I2C_SDA_H(); // 释放 SDA 线
    I2C_Delay();
    I2C_SCL_H();
    I2C_Delay();
    if (I2C_SDA_READ() == GPIO_PIN_SET) {
        ack = 1; // 无应答
    }
    I2C_SCL_L();
    I2C_Delay();
    return ack;
}

void I2C_SendByte(uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++) {
        if (byte & 0x80) I2C_SDA_H();
        else             I2C_SDA_L();
        I2C_Delay();
        I2C_SCL_H();
        I2C_Delay();
        I2C_SCL_L();
        byte <<= 1;
    }
    I2C_WaitAck(); // 默认发送完等待应答
}
