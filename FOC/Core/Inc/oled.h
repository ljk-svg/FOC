#ifndef __OLED_H
#define __OLED_H

#include "main.h"

// OLED µÄ I2C Æ÷¼þµØÖ·
#define OLED_ADDRESS 0x78

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Printf(uint8_t x, uint8_t y, const char *format, ...);

#endif
