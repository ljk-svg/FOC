#include "vofa.h"
#include "usart.h" // 引入 CubeMX 生成的串口头文件，以调用 huart3

// ---------------- 数据结构定义 ----------------
// 核心操作：强制 1 字节对齐。
// 必须加这个！防止编译器为了 32 位对齐，在 float 和 tail 之间塞入垃圾字节，导致 VOFA+ 无法识别。
#pragma pack(push, 1)
typedef struct {
    float fdata[VOFA_CH_COUNT]; // 浮点数据区 (4 * 4 = 16 bytes)
    uint8_t tail[4];            // JustFloat 帧尾 (4 bytes)
} VOFA_JustFloat_t;
#pragma pack(pop)

// 实例化数据包 (静态全局变量，分配在固定内存区，方便 DMA 搬运)
static VOFA_JustFloat_t vofa_packet;

// ---------------- 接口函数 ----------------
void VOFA_Init(void) {
    // 写入 VOFA+ JustFloat 协议认定的死规定帧尾：0x00 0x00 0x80 0x7F
    vofa_packet.tail[0] = 0x00;
    vofa_packet.tail[1] = 0x00;
    vofa_packet.tail[2] = 0x80;
    vofa_packet.tail[3] = 0x7F;
}

// 极致高效的发送函数：CPU 只负责搬几块内存，剩下的全扔给硬件
void VOFA_Send_DMA(float ch1, float ch2, float ch3, float ch4) {
    
    // 效率保障 1：绝对不阻塞！
    // 检查串口硬件底层是不是空闲的。如果上次的还没发完，说明调用太快了，直接丢弃本次数据！
    // FOC 控制中，丢掉一两帧波形画面的影响为 0，但如果卡住 CPU 死等，电机就会炸机。
    if (huart3.gState == HAL_UART_STATE_READY) {
        
        // 效率保障 2：内存直接赋值，不经过任何 sprintf 字符转换
        vofa_packet.fdata[0] = ch1;
        vofa_packet.fdata[1] = ch2;
        vofa_packet.fdata[2] = ch3;
        vofa_packet.fdata[3] = ch4;
        
        // 效率保障 3：启动 DMA。
        // 告诉 DMA 控制器首地址和总长度，DMA 会在后台偷 CPU 的闲置周期默默把数据推上引脚。
        HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&vofa_packet, sizeof(VOFA_JustFloat_t));
    }
}
