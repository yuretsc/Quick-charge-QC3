#include "ssd1306.h"
#include <stdlib.h>
// Databuffer voor het scherm
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

// Een scherm-object om lokaal in te werken
static SSD1306_t SSD1306;

void ssd1306_Init(I2C_HandleTypeDef i2c_port, uint8_t address)
{	
  SSD1306.I2C_port = i2c_port;
  SSD1306.Address = address;
  // Waiting for SEG/COM ON after reset
  HAL_Delay(50);
  // Initialize LCD
  const uint8_t init_data[] = {
		  0xAE,
		  0xA8, 0x1F,
		  0xD3, 0x00,
		  0x40,
		  0xA1,
		  0xC8,
		  0xDA, 0x02,
		  0x81, 0x7F,
		  0xA4,
		  0xA6,
		  0xD5, 0x80,
		  0x8D, 0x14,
		 // 0x20, 0x00,
		 // 0x22, 0x00, 0x7F,
		 // 0x21, 0x00, 0x04,
		  0xAF
		  };

		        // 0xC0, 0xA0};
  // 0xAE - display off
  // 0xA8, 0x3F - multiplex ratio ( разрешение по вертикали, 3F =64, 1F = 32
  // 0xD3, 0x00 - Set Display Offset (D3h)
  // 0x40 - Set Display Start Line
  // 0xA0/A1 - segment re-map (vertical mirroring)
  // 0xC0/C8 - COM scan direction (horizontal mirroring)
  // 0xDA, 0x02 - SETCOMPINS

  // 0x81, 0x7F - contrast ratio 127
  //  - Entire Display ON
  // 0xA6 - normal (not inversed)
  // 0xD5, 0x80 - Set Display Clock Divide Ratio/ Oscillator Frequency
  // 0x8D, 0x14 - enable charge pump
  // 0x20, 0x00 - page horizontal adressing mode
  // 0xAF - display on (only just after enabling charge pump)


  // 0x21, 0x00, 0x7F - column address from 0 to 127
  // 0x22, 0x00, 0x04 - page address from 0 to 4







  // 0xC0 - vertical inversion
  // 0xA0 - horizontal inversion
  ssd1306_SendToDisplay(Commands, init_data, sizeof(init_data));
  // clearing screen
  ssd1306_Fill(Black);
  ssd1306_UpdateScreen();
  // setting default position
  SSD1306.CurrentX = 0;
  SSD1306.CurrentY = 0;
}

void ssd1306_Fill(SSD1306_COLOR color) 
{
  uint8_t toBuffer = 0x00; // black
  if(color == White)
  {
    toBuffer = 0xFF; // white
  }
  for(uint32_t i = 0; i < sizeof(SSD1306_Buffer); i++)
  {
    SSD1306_Buffer[i] = toBuffer;
  }
}

void ssd1306_picture(uint8_t x, uint8_t y, const unsigned char* bitmap, uint8_t w, uint8_t h, SSD1306_COLOR color)
/*{
  for(uint32_t i = 0; i < sizeof(SSD1306_Buffer); i++)
  {
    SSD1306_Buffer[i] = buffer[i];
  }
}
*/
{
	int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
	uint8_t byte = 0;

	if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
		return;
	}

	for (uint8_t j = 0; j < h; j++, y++) {
		for (uint8_t i = 0; i < w; i++) {
			if (i & 7) {
				byte <<= 1;
			} else {
				byte = (*(const unsigned char *)(&bitmap[j * byteWidth + i / 8]));
			}

			if (byte & 0x80) {
            ssd1306_DrawPixel(x + i, y, color);
			}
		}
	}
return;
}


void ssd1306_UpdateScreen(void) 
{
 uint8_t update_region_data[8] = {0x20, 0x00, 0x21, 0x00, 0x7F, 0x22, 0x00, 0x03};
  //0x21, 0x00, 0x7F - column address from 0 to 127
  //0x22, 0x00, 0x03 - page address from 0 to 3
  ssd1306_SendToDisplay(Commands, update_region_data, sizeof(update_region_data));

  //update pages
//  for(uint32_t page = 0; page < SSD1306_HEIGHT/8; page++)
  {

   // ssd1306_SendToDisplay(Datas, &SSD1306_Buffer[SSD1306_WIDTH * 4], SSD1306_WIDTH);

  }
  for(uint8_t i = 0; i < SSD1306_HEIGHT/8; i++) {
	  uint8_t update_data[1]={0xB0 + i};
	  ssd1306_SendToDisplay(Commands, update_data, 1); // Set the current RAM page address.
	  ssd1306_SendToDisplay(Commands, (uint8_t*) (0x00 + 0), 1);
	  ssd1306_SendToDisplay(Commands, (uint8_t*) (0x10 + 0), 1);
	  ssd1306_SendToDisplay(Datas, &SSD1306_Buffer[SSD1306_WIDTH*i],SSD1306_WIDTH);
      }
}


void ssd1306_DrawPixel(uint32_t x, uint32_t y, SSD1306_COLOR color)
{
  if((x < SSD1306_WIDTH) && (y < SSD1306_HEIGHT)) 
  {
    if (color == White)
    {
      SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
    } 
    else 
    {
      SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
    }
  }
}


void ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color)
{
  uint32_t pixel;
  
  if ((((SSD1306.CurrentX + Font.FontWidth) < SSD1306_WIDTH)  && ((SSD1306.CurrentY + Font.FontHeight) < SSD1306_HEIGHT)))
  {
	    // Check remaining space on current line
	    if (SSD1306_WIDTH < (SSD1306.CurrentX + Font.FontWidth) ||
	        SSD1306_HEIGHT < (SSD1306.CurrentY + Font.FontHeight))
	    {
	        // Not enough space on current line
	        return;
	    }


	  // write char to display buffer
    for (uint32_t y = 0; y < Font.FontHeight; y++)
    {

       pixel = Font.font[(ch - 32) * Font.FontHeight + y];

      // write pixel to display buffer
      //x = Font.FontWidth;
      for (uint32_t x = 0; x < Font.FontWidth; x++)
      {

        if (pixel & 0x8000)
        {
          ssd1306_DrawPixel(SSD1306.CurrentX + x, (SSD1306.CurrentY + y), color);
        } 
        else 
        {
          ssd1306_DrawPixel(SSD1306.CurrentX + x, (SSD1306.CurrentY + y), !color);
        }
        pixel <<= 1;

      }
    } 
  }
  
  // going to next position
  SSD1306.CurrentX += Font.FontWidth;

}

void ssd1306_WriteString(const char* str, FontDef Font, SSD1306_COLOR color)
{
  // We schrijven alle char tot een nulbyte
  while (*str) 
  {
    ssd1306_WriteChar(*str, Font, color);
    
    // Volgende char
    str++;
  }
  
}


void ssd1306_SetCursor(uint32_t x, uint32_t y) 
{
	SSD1306.CurrentX = x;
	SSD1306.CurrentY = y;
}

void ssd1306_SetCursorPos(uint32_t x, uint32_t y, FontDef Font)
{
	SSD1306.CurrentX = x*Font.FontWidth;
	SSD1306.CurrentY = y*Font.FontHeight;
}

void ssd1306_SetContrast(uint8_t contrast)
{
  //contrast command with empty data
uint8_t contrast_data[2] = {0x81};
  //adding contrast data
  contrast_data[1] = contrast;
  //and send to display
  ssd1306_SendToDisplay(Commands, contrast_data, 2);
}



void ssd1306_SendToDisplay(SSD1306_DATA_TYPE type, const uint8_t *data, uint8_t length)
{

	HAL_I2C_Mem_Write(&(SSD1306.I2C_port), SSD1306.Address, type, 1, data, length, 10);
}

void ssd1306_WriteNum(int16_t var, uint8_t format, const char* ending, SSD1306_COLOR color, FontDef Font)
{
	//определить сколько цифр в числе
	int16_t temp_dataIn;
	int16_t stringLen=1;
	char buffer[10];

	temp_dataIn = var;
	while (temp_dataIn/10 != 0){//вычисляем длинну числа
	    temp_dataIn = temp_dataIn/10;
	    stringLen++;
	}
	if (stringLen<=format) stringLen++;
	if (var<0)
		{
		stringLen++;// если отрицательное число, то добавим ячейку для минуса
		*(buffer) ='-';// если отрицательное число, то добавим ячейку для минуса
		}
	if (format!=0)
		{
			//*(buffer+stringLen-1) =',';
			stringLen++;// если не целое число, то добавим ячейку для запятой
			if (abs(var)<10) stringLen++;
			format = stringLen-format;
		}
	temp_dataIn = var;
    *(buffer+stringLen) = ending[0];//символ размерности
    *(buffer+stringLen+1) = ending[1];//символ размерности
    *(buffer+stringLen+2) = 0;// конец строки
	do{
	    if (format!=0 && (stringLen==format))
	    {
	    	*(buffer+stringLen-1) =',';
	    }
	    else
	    {
	    	*(buffer+stringLen-1) = (abs(temp_dataIn)%10)+'0';
		    temp_dataIn = (int16_t) temp_dataIn / 10;
	    }

	}while(--stringLen > 0+(var<0));
//	if (var<0) *(buffer+stringLen) ='-';// если отрицательное число, то добавим ячейку для минуса

	ssd1306_WriteString(buffer, Font, color);
}

void SSD1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, SSD1306_COLOR color) {
	int16_t dx, dy, sx, sy, err, e2, i, tmp;

	/* Check for overflow */
	if (x0 >= SSD1306_WIDTH) {
		x0 = SSD1306_WIDTH - 1;
	}
	if (x1 >= SSD1306_WIDTH) {
		x1 = SSD1306_WIDTH - 1;
	}
	if (y0 >= SSD1306_HEIGHT) {
		y0 = SSD1306_HEIGHT - 1;
	}
	if (y1 >= SSD1306_HEIGHT) {
		y1 = SSD1306_HEIGHT - 1;
	}

	dx = (x0 < x1) ? (x1 - x0) : (x0 - x1);
	dy = (y0 < y1) ? (y1 - y0) : (y0 - y1);
	sx = (x0 < x1) ? 1 : -1;
	sy = (y0 < y1) ? 1 : -1;
	err = ((dx > dy) ? dx : -dy) / 2;

	if (dx == 0) {
		if (y1 < y0) {
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}

		if (x1 < x0) {
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}

		/* Vertical line */
		for (i = y0; i <= y1; i++) {
			ssd1306_DrawPixel(x0, i, color);
		}

		/* Return from function */
		return;
	}

	if (dy == 0) {
		if (y1 < y0) {
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}

		if (x1 < x0) {
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}

		/* Horizontal line */
		for (i = x0; i <= x1; i++) {
			ssd1306_DrawPixel(i, y0, color);
		}

		/* Return from function */
		return;
	}

	while (1) {
		ssd1306_DrawPixel(x0, y0, color);
		if (x0 == x1 && y0 == y1) {
			break;
		}
		e2 = err;
		if (e2 > -dx) {
			err -= dy;
			x0 += sx;
		}
		if (e2 < dy) {
			err += dx;
			y0 += sy;
		}
	}
}
