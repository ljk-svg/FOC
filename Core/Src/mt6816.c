#include "mt6816.h"
#include "bsp_spi.h"
#include <math.h>  // 必须包含这个，我们需要用到里面的 fmodf 求余函数

// 实例化结构体
MT6816_t mt6816;

// 初始化函数
void MT6816_Init(void) {
    // 刚上电时，必须保证片选引脚是高电平 (SPI 空闲状态)
    SPI_CS_H(); 
    
    mt6816.raw_data = 0;
    mt6816.angle_deg = 0.0f;
    mt6816.elec_angle_rad = 0.0f;
}
void MT6816_Update(void) {
    uint16_t reg03, reg04;
    uint16_t angle_data;
    
    SPI_CS_L();
    reg03 = SPI_ReadWriteByte16(0x8300); // 读 0x03 寄存器
    SPI_CS_H();
    
    __NOP(); __NOP(); __NOP(); __NOP(); 

    SPI_CS_L();
    reg04 = SPI_ReadWriteByte16(0x8400); // 读 0x04 寄存器
    SPI_CS_H();

    // 假设 SPI_ReadWriteByte16 返回的是完整的 16 位接收数据。
    // 对于 MT6816，MISO 返回的数据，其有效载荷在低 8 位。
    // 拼接 0x03 的低 8 位作为高字节，0x04 的低 8 位作为低字节。
    angle_data = ((reg03 & 0x00FF) << 8) | (reg04 & 0x00FF);
    
    // 右移 2 位去掉状态位，得到 14 位绝对角度
    mt6816.raw_data = angle_data >> 2;
    
    // 算机械角度 (供观测)
    mt6816.angle_deg = (float)mt6816.raw_data * 360.0f / 16384.0f;
    
    // 算电角度 (供 FOC)
    float mech_angle_rad = ((float)mt6816.raw_data / 16384.0f) * 2.0f * PI;
    float elec_angle_rad = mech_angle_rad * MOTOR_POLE_PAIRS;
    
    // 归一化限制在 0 ~ 2PI 之间
    elec_angle_rad = fmodf(elec_angle_rad, 2.0f * PI);
    if(elec_angle_rad < 0.0f) elec_angle_rad += 2.0f * PI;
    
    mt6816.elec_angle_rad = elec_angle_rad; 
}
