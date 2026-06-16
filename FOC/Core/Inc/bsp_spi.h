#ifndef __BSP_SPI_H
#define __BSP_SPI_H

#include "main.h"

// ---------------- 物理片选引脚 ----------------
#define SPI_CS_PORT   GPIOA
#define SPI_CS_PIN    GPIO_PIN_0

#define SPI_CS_H()    HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET)
#define SPI_CS_L()    HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_RESET)

// ---------------- 对外接口 ----------------
uint16_t SPI_ReadWriteByte16(uint16_t tx_data);

#endif
