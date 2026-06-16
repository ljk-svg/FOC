#include "bsp_spi.h"
#include "spi.h" // 必须包含这个，里面有 CubeMX 生成的 hspi3 句柄

// 硬件 SPI3 16位全双工收发
uint16_t SPI_ReadWriteByte16(uint16_t tx_data) {
    uint16_t rx_data = 0;
    
    // 调用 HAL 库自带的极速硬件收发函数
    // 强制使用 hspi3！
    // 参数说明: SPI句柄, 发送数据地址, 接收数据地址, 数据帧数量(1帧), 超时时间(10ms)
    HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)&tx_data, (uint8_t *)&rx_data, 1, 10);
    
    return rx_data;
}
