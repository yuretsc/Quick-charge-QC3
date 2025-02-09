#ifndef __FONTS_H
#define __FONTS_H
#define SSD1306_INCLUDE_FONT_16x24
#include <stdint.h>
// font parameters main structure
typedef struct {
  const unsigned char FontWidth;
  const unsigned char FontHeight;
  const uint16_t * font;
} FontDef;

// some available fonts

extern FontDef font8x16;
extern FontDef font16x24;

#endif

