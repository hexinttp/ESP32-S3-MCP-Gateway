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

static const uint8_t s_digits[10][5] = {
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, {0x00, 0x42, 0x7F, 0x40, 0x00},
    {0x42, 0x61, 0x51, 0x49, 0x46}, {0x21, 0x41, 0x45, 0x4B, 0x31},
    {0x18, 0x14, 0x12, 0x7F, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39},
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, {0x01, 0x71, 0x09, 0x05, 0x03},
    {0x36, 0x49, 0x49, 0x49, 0x36}, {0x06, 0x49, 0x49, 0x29, 0x1E},
};

static const uint8_t s_letters[26][5] = {
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, {0x7F, 0x49, 0x49, 0x49, 0x36},
    {0x3E, 0x41, 0x41, 0x41, 0x22}, {0x7F, 0x41, 0x41, 0x22, 0x1C},
    {0x7F, 0x49, 0x49, 0x49, 0x41}, {0x7F, 0x09, 0x09, 0x09, 0x01},
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, {0x7F, 0x08, 0x08, 0x08, 0x7F},
    {0x00, 0x41, 0x7F, 0x41, 0x00}, {0x20, 0x40, 0x41, 0x3F, 0x01},
    {0x7F, 0x08, 0x14, 0x22, 0x41}, {0x7F, 0x40, 0x40, 0x40, 0x40},
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, {0x7F, 0x04, 0x08, 0x10, 0x7F},
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, {0x7F, 0x09, 0x09, 0x09, 0x06},
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, {0x7F, 0x09, 0x19, 0x29, 0x46},
    {0x46, 0x49, 0x49, 0x49, 0x31}, {0x01, 0x01, 0x7F, 0x01, 0x01},
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, {0x1F, 0x20, 0x40, 0x20, 0x1F},
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, {0x63, 0x14, 0x08, 0x14, 0x63},
    {0x07, 0x08, 0x70, 0x08, 0x07}, {0x61, 0x51, 0x49, 0x45, 0x43},
};

static const uint8_t *glyph(char c)
{
    static const uint8_t space[5] = {0};
    static const uint8_t dash[5] = {0x08, 0x08, 0x08, 0x08, 0x08};
    static const uint8_t dot[5] = {0x00, 0x60, 0x60, 0x00, 0x00};
    static const uint8_t colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00};
    static const uint8_t slash[5] = {0x20, 0x10, 0x08, 0x04, 0x02};
    static const uint8_t percent[5] = {0x23, 0x13, 0x08, 0x64, 0x62};
    static const uint8_t question[5] = {0x02, 0x01, 0x51, 0x09, 0x06};

    c = (char)toupper((unsigned char)c);
    if (c >= '0' && c <= '9') return s_digits[c - '0'];
    if (c >= 'A' && c <= 'Z') return s_letters[c - 'A'];
    switch (c) {
    case ' ': return space;
    case '-': return dash;
    case '.': return dot;
    case ':': return colon;
    case '/': return slash;
    case '%': return percent;
    default: return question;
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
    const int advance = 5 * scale + 1;
    for (; *text != '\0'; ++text) {
        if (x + advance > BOARD_LCD_WIDTH) return;
        const uint8_t *bits = glyph(*text);
        lcd_rect(x, y, advance, 7 * scale, bg);
        for (int col = 0; col < 5; ++col) {
            for (int row = 0; row < 7; ++row) {
                if (bits[col] & (1U << row)) {
                    lcd_rect(x + col * scale, y + row * scale, scale, scale, fg);
                }
            }
        }
        x += advance;
    }
}

static int text_width(const char *text, int scale)
{
    if (text == NULL || text[0] == '\0') return 0;
    return (int)strlen(text) * (5 * scale + 1) - 1;
}

static void centered_text(int y, const char *text, uint16_t fg, uint16_t bg, int scale)
{
    int width = text_width(text, scale);
    int x = width < BOARD_LCD_WIDTH ? (BOARD_LCD_WIDTH - width) / 2 : 0;
    lcd_st7735_text(x, y, text, fg, bg, scale);
}

static void dashboard_row(int y, const char *label, const char *value, uint16_t accent,
                          bool alternate)
{
    uint16_t background = alternate ? 0x1082 : LCD_COLOR_BLACK;
    lcd_rect(0, y, BOARD_LCD_WIDTH, 42, background);
    lcd_st7735_text(6, y + 4, label, accent, background, 1);

    int scale = text_width(value, 2) <= BOARD_LCD_WIDTH - 8 ? 2 : 1;
    int value_y = y + (scale == 2 ? 20 : 23);
    centered_text(value_y, value, LCD_COLOR_WHITE, background, scale);
    lcd_rect(0, y + 41, BOARD_LCD_WIDTH, 1, 0x2945);
}

void lcd_st7735_dashboard(const char *title,
                          const char *label1, const char *value1,
                          const char *label2, const char *value2,
                          const char *label3, const char *value3,
                          uint16_t accent)
{
    lcd_st7735_fill(LCD_COLOR_BLACK);
    lcd_rect(0, 0, 5, 29, accent);
    lcd_rect(5, 27, BOARD_LCD_WIDTH - 5, 2, accent);
    lcd_st7735_text(11, 7, title, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 2);
    dashboard_row(31, label1, value1, accent, false);
    dashboard_row(73, label2, value2, accent, true);
    dashboard_row(115, label3, value3, accent, false);
}
