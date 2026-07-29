#include "services/health_service.h"

#include <string.h>
#include "board/board.h"
#include "esp_app_desc.h"
#include "esp_efuse.h"
#include "esp_flash_encrypt.h"
#include "esp_heap_caps.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_secure_boot.h"
#include "esp_timer.h"
#include "nvs.h"
#include "services/time_service.h"

#define HEALTH_NVS_NAMESPACE "health"
#define HEALTH_BOOT_KEY "boot_count"
#define HEALTH_WDT_KEY "wdt_count"

static gateway_health_t s_health;

esp_err_t health_service_init(void)
{
    memset(&s_health, 0, sizeof(s_health));
    s_health.reset_reason = esp_reset_reason();
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(HEALTH_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;
    nvs_get_u32(nvs, HEALTH_BOOT_KEY, &s_health.boot_count);
    nvs_get_u32(nvs, HEALTH_WDT_KEY, &s_health.watchdog_resets);
    ++s_health.boot_count;
    if (s_health.reset_reason == ESP_RST_TASK_WDT ||
        s_health.reset_reason == ESP_RST_INT_WDT ||
        s_health.reset_reason == ESP_RST_WDT) {
        ++s_health.watchdog_resets;
    }
    nvs_set_u32(nvs, HEALTH_BOOT_KEY, s_health.boot_count);
    nvs_set_u32(nvs, HEALTH_WDT_KEY, s_health.watchdog_resets);
    err = nvs_commit(nvs);
    nvs_close(nvs);
    return err;
}

void health_service_get(gateway_health_t *out)
{
    if (out == NULL) return;
    *out = s_health;
    out->uptime_ms = esp_timer_get_time() / 1000;
    out->free_heap = esp_get_free_heap_size();
    out->minimum_free_heap = esp_get_minimum_free_heap_size();
    out->free_internal_heap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    out->largest_internal_block =
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    out->free_dma_heap =
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    out->free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    out->largest_psram_block =
        heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    out->flash_encryption_enabled = esp_flash_encryption_enabled();
    out->secure_boot_enabled = esp_secure_boot_enabled();
    out->ota_capable = esp_ota_get_next_update_partition(NULL) != NULL;
    out->storage_ready = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, "cache") != NULL;
    out->time_synchronized = time_service_is_synchronized();
}
