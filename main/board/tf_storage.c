#include "tf_storage.h"

#include <sys/stat.h>
#include "board.h"
#include "board_pins.h"
#include "driver/sdmmc_host.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

static const char *TAG = "TF";
static sdmmc_card_t *s_card;
static bool s_mounted;

esp_err_t tf_storage_mount(void)
{
    if (s_mounted) {
        return ESP_OK;
    }

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.width = 4;
    slot.clk = BOARD_TF_PIN_CLK;
    slot.cmd = BOARD_TF_PIN_CMD;
    slot.d0 = BOARD_TF_PIN_D0;
    slot.d1 = BOARD_TF_PIN_D1;
    slot.d2 = BOARD_TF_PIN_D2;
    slot.d3 = BOARD_TF_PIN_D3;
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_vfs_fat_sdmmc_mount_config_t mount = {
        .format_if_mount_failed = false,
        .max_files = 8,
        .allocation_unit_size = 16 * 1024,
    };
    esp_err_t err = esp_vfs_fat_sdmmc_mount(TF_MOUNT_POINT, &host, &slot, &mount, &s_card);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "TF card unavailable: %s", esp_err_to_name(err));
        board_set_tf_card_ready(false);
        return err;
    }

    mkdir(TF_MOUNT_POINT "/cache", 0775);
    mkdir(TF_MOUNT_POINT "/history", 0775);
    mkdir(TF_MOUNT_POINT "/logs", 0775);
    s_mounted = true;
    board_set_tf_card_ready(true);
    ESP_LOGI(TAG, "TF card mounted at %s", TF_MOUNT_POINT);
    sdmmc_card_print_info(stdout, s_card);
    return ESP_OK;
}

bool tf_storage_is_mounted(void) { return s_mounted; }

esp_err_t tf_storage_get_space(uint64_t *total_bytes, uint64_t *free_bytes)
{
    if (!s_mounted || total_bytes == NULL || free_bytes == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return esp_vfs_fat_info(TF_MOUNT_POINT, total_bytes, free_bytes);
}
