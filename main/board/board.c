#include "board.h"
#include "board_pins.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BOARD";
static board_status_t s_status;

void board_shared_reset(void)
{
    gpio_set_level(BOARD_SHARED_RESET_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(BOARD_SHARED_RESET_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
}

esp_err_t board_init(void)
{
    gpio_config_t reset_cfg = {
        .pin_bit_mask = 1ULL << BOARD_SHARED_RESET_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&reset_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Shared reset GPIO init failed: %s", esp_err_to_name(err));
        return err;
    }

    board_shared_reset();
    s_status = (board_status_t){0};
    ESP_LOGI(TAG, "Board pins configured; shared LCD/W5500 reset released");
    return ESP_OK;
}

const board_status_t *board_get_status(void)
{
    return &s_status;
}

void board_set_lcd_ready(bool ready) { s_status.lcd_ready = ready; }
void board_set_ethernet_ready(bool ready) { s_status.ethernet_ready = ready; }
void board_set_tf_card_ready(bool ready) { s_status.tf_card_ready = ready; }
void board_set_rs485_ready(bool ready) { s_status.rs485_ready = ready; }
