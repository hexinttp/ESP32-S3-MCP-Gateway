#include "lcd_st7735.h"

#include <ctype.h>
#include <string.h>
#include "board.h"
#include "board_pins.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "LCD";
static spi_device_handle_t s_lcd;

static void lcd_tx(bool data_mode, const void *data, size_t size)
{
    if (s_lcd == NULL || data == NULL || size == 0) return;
    gpio_set_level(BOARD_LCD_PIN_DC, data_mode ? 1 : 0);
    spi_transaction_t t = {.length = size * 8, .tx_buffer = data};
    spi_device_polling_transmit(s_lcd, &t);
}

static void lcd_cmd(uint8_t command) { lcd_tx(false, &command, 1); }
static void lcd_data(const void *data, size_t size) { lcd_tx(true, data, size); }

static void lcd_window(int x0, int y0, int x1, int y1)
{
    uint8_t b[4];
    lcd_cmd(0x2A);
    b[0] = x0 >> 8; b[1] = x0; b[2] = x1 >> 8; b[3] = x1; lcd_data(b, 4);
    lcd_cmd(0x2B);
    b[0] = y0 >> 8; b[1] = y0; b[2] = y1 >> 8; b[3] = y1; lcd_data(b, 4);
    lcd_cmd(0x2C);
}

static uint32_t glyph(char c)
{
    c = (char)toupper((unsigned char)c);
    switch (c) {
    case 'A': return 0x7C12F17C; case 'B': return 0x7E52D12E;
    case 'C': return 0x3C820104; case 'D': return 0x7E82013C;
    case 'E': return 0xFE52D100; case 'F': return 0xFE12D000;
    case 'G': return 0x3C82953C; case 'H': return 0xFE1010FE;
    case 'I': return 0x0082FE82; case 'J': return 0x040281FC;
    case 'K': return 0xFE102844; case 'L': return 0xFE020202;
    case 'M': return 0xFE4030FE; case 'N': return 0xFE2010FE;
    case 'O': return 0x7C82017C; case 'P': return 0xFE90A060;
    case 'Q': return 0x7C82857E; case 'R': return 0xFE90A06E;
    case 'S': return 0x6252918C; case 'T': return 0x8080FE80;
    case 'U': return 0xFC0201FC; case 'V': return 0xF80403F8;
    case 'W': return 0xFE0418FE; case 'X': return 0xC62810C6;
    case 'Y': return 0xC0201EC0; case 'Z': return 0x868A92C2;
    case '0': return 0x7C8A927C; case '1': return 0x0042FE02;
    case '2': return 0x468A9272; case '3': return 0x4482926C;
    case '4': return 0xF010FE10; case '5': return 0xE492928C;
    case '6': return 0x7C92920C; case '7': return 0x808E90E0;
    case '8': return 0x6C92926C; case '9': return 0x6092927C;
    case '-': return 0x00101010; case '_': return 0x02020202;
    case '.': return 0x00000200; case ':': return 0x00280000;
    case '/': return 0x06081060; case '%': return 0xC6081063;
    default: return 0;
    }
}

static void lcd_rect(int x, int y, int w, int h, uint16_t color)
{
    if (x < 0 || y < 0 || w <= 0 || h <= 0 || x >= BOARD_LCD_WIDTH || y >= BOARD_LCD_HEIGHT) return;
    if (x + w > BOARD_LCD_WIDTH) w = BOARD_LCD_WIDTH - x;
    if (y + h > BOARD_LCD_HEIGHT) h = BOARD_LCD_HEIGHT - y;
    lcd_window(x, y, x + w - 1, y + h - 1);
    uint16_t pixels[64];
    uint16_t be = (uint16_t)((color << 8) | (color >> 8));
    for (size_t i = 0; i < 64; ++i) pixels[i] = be;
    int remaining = w * h;
    while (remaining > 0) {
        int count = remaining > 64 ? 64 : remaining;
        lcd_data(pixels, count * sizeof(uint16_t));
        remaining -= count;
    }
}

esp_err_t lcd_st7735_init(void)
{
    gpio_config_t dc = {
        .pin_bit_mask = 1ULL << BOARD_LCD_PIN_DC,
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&dc), TAG, "DC gpio");

    spi_bus_config_t bus = {
        .mosi_io_num = BOARD_LCD_PIN_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = BOARD_LCD_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = BOARD_LCD_WIDTH * 40 * 2,
    };
    esp_err_t err = spi_bus_initialize(BOARD_LCD_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    spi_device_interface_config_t device = {
        .mode = 0,
        .clock_speed_hz = 20 * 1000 * 1000,
        .spics_io_num = BOARD_LCD_PIN_CS,
        .queue_size = 4,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(BOARD_LCD_SPI_HOST, &device, &s_lcd), TAG, "SPI device");

    lcd_cmd(0x01); vTaskDelay(pdMS_TO_TICKS(150));
    lcd_cmd(0x11); vTaskDelay(pdMS_TO_TICKS(120));
    uint8_t colmod = 0x05; lcd_cmd(0x3A); lcd_data(&colmod, 1);
    uint8_t madctl = 0xC0; lcd_cmd(0x36); lcd_data(&madctl, 1);
    lcd_cmd(0x29); vTaskDelay(pdMS_TO_TICKS(20));
    lcd_st7735_fill(LCD_COLOR_BLACK);
    board_set_lcd_ready(true);
    ESP_LOGI(TAG, "ST7735S initialized on SPI3");
    return ESP_OK;
}

void lcd_st7735_fill(uint16_t color)
{
    lcd_rect(0, 0, BOARD_LCD_WIDTH, BOARD_LCD_HEIGHT, color);
}

void lcd_st7735_text(int x, int y, const char *text, uint16_t fg, uint16_t bg, int scale)
{
    if (text == NULL || scale < 1) return;
    for (; *text != '\0'; ++text) {
        if (x + 6 * scale > BOARD_LCD_WIDTH) { x = 0; y += 9 * scale; }
        uint32_t bits = glyph(*text);
        lcd_rect(x, y, 6 * scale, 8 * scale, bg);
        for (int col = 0; col < 4; ++col) {
            uint8_t column = (uint8_t)(bits >> (24 - col * 8));
            for (int row = 0; row < 8; ++row) {
                if (column & (1U << (7 - row))) {
                    lcd_rect(x + col * scale, y + row * scale, scale, scale, fg);
                }
            }
        }
        x += 6 * scale;
    }
}

void lcd_st7735_status(const char *title, const char *line1, const char *line2,
                       const char *line3, uint16_t accent)
{
    lcd_st7735_fill(LCD_COLOR_BLACK);
    lcd_rect(0, 0, BOARD_LCD_WIDTH, 24, accent);
    lcd_st7735_text(6, 7, title, LCD_COLOR_BLACK, accent, 1);
    lcd_st7735_text(4, 38, line1, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1);
    lcd_st7735_text(4, 65, line2, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1);
    lcd_st7735_text(4, 92, line3, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1);
}
