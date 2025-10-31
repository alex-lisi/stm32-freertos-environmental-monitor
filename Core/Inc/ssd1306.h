/*
 * ssd1306.h
 *
 *  Created on: Oct 28, 2025
 *      Author: alexl
 */

#ifndef SSD1306_H
#define SSD1306_H

#include "main.h"

// Screen dimensions
#define SSD1306_WIDTH  128
#define SSD1306_HEIGHT 64

// I2C address (common address is 0x3C, some displays use 0x3D)
#define SSD1306_I2C_ADDR 0x78  // 0x3C << 1

// Functions
void SSD1306_Init(void);
void SSD1306_Clear(void);
void SSD1306_UpdateScreen(void);
void SSD1306_GotoXY(uint8_t x, uint8_t y);
void SSD1306_WriteChar(char ch);
void SSD1306_WriteString(char* str);

#endif
