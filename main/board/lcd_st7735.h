#ifndef LCD_ST7735_H
#define LCD_ST7735_H

#include <stdint.h>
#include "esp_err.h"

esp_err_t lcd_st7735_init(void);
void lcd_st7735_fill(uint16_t color);
void lcd_st7735_text(int x, int y, const char *text, uint16_t fg, uint16_t bg, int scale);
void lcd_st7735_status(const char *title, const char *line1, const char *line2,
                       const char *line3, uint16_t accent);

#define LCD_COLOR_BLACK  0x0000
#define LCD_COLOR_WHITE  0xFFFF
#define LCD_COLOR_RED    0xF800
#define LCD_COLOR_GREEN  0x07E0
#define LCD_COLOR_BLUE   0x001F
#define LCD_COLOR_YELLOW 0xFFE0
#define LCD_COLOR_CYAN   0x07FF

#endif
