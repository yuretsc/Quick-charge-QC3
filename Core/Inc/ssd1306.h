#ifndef __SSD1306_H
#define __SSD1306_H

// include hal library like in your main.c file
// for example i use STM32F0 series 
#include "stm32g0xx_hal.h"

#include "fonts.h"

#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 32

// awailable colors
typedef enum {
  Black = 0x00, // pixel off
  White = 0x01  // pixel on
} SSD1306_COLOR;

// awailable display communication data types
typedef enum {
  Commands = 0x00, 
  Datas = 0x40  
} SSD1306_DATA_TYPE;

// main struct with display config data
typedef struct {
  uint32_t CurrentX;
  uint32_t CurrentY;
  uint8_t Address;
  I2C_HandleTypeDef I2C_port;
} SSD1306_t;


// interface functions

// initialize display
void ssd1306_Init(I2C_HandleTypeDef i2c_port, uint8_t address);
// fill display buffer with chosen color
void ssd1306_Fill(SSD1306_COLOR color);
// update information on display
void ssd1306_UpdateScreen(void);
// draw single pixel to display buffer
void ssd1306_DrawPixel(uint32_t x, uint32_t y, SSD1306_COLOR color);
// draw char symbol to display buffer
void ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color);
// draw char  string to display buffer
void ssd1306_WriteString(const char* str, FontDef Font, SSD1306_COLOR color);
// set cursor for drawing 
void ssd1306_SetCursor(uint32_t x, uint32_t y);
// set position cursor for drawing
void ssd1306_SetCursorPos(uint32_t x, uint32_t y, FontDef Font);
// set contrast of display (0 to 255)
void ssd1306_SetContrast(uint8_t contrast);

void SSD1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, SSD1306_COLOR color);


void ssd1306_picture (uint8_t x, uint8_t y, const unsigned char* bitmap, uint8_t w, uint8_t h, SSD1306_COLOR color);
// internal functions

// send data or command buffer to display via I2C interface
void ssd1306_SendToDisplay(SSD1306_DATA_TYPE type, const uint8_t *data, uint8_t length);

void ssd1306_WriteNum(int16_t, uint8_t, const char*, SSD1306_COLOR, FontDef Font);


#endif
