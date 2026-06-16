#ifndef __VOFA_H
#define __VOFA_H

#include "main.h"

// 设定发送的通道数量 (Kp, Ki, 设定值, 实际波形)
#define VOFA_CH_COUNT 4

void VOFA_Init(void);
void VOFA_Send_DMA(float ch1, float ch2, float ch3, float ch4);

#endif
