#ifndef HEALTH_SERVICE_H
#define HEALTH_SERVICE_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_system.h"

typedef struct {
    uint32_t boot_count;
    esp_reset_reason_t reset_reason;
    int64_t uptime_ms;
    uint32_t free_heap;
    uint32_t minimum_free_heap;
    uint32_t free_internal_heap;
    uint32_t largest_internal_block;
    uint32_t free_dma_heap;
    uint32_t free_psram;
    uint32_t largest_psram_block;
    bool flash_encryption_enabled;
    bool secure_boot_enabled;
    bool ota_capable;
    bool storage_ready;
    bool time_synchronized;
    uint32_t watchdog_resets;
} gateway_health_t;

esp_err_t health_service_init(void);
void health_service_get(gateway_health_t *out);

#endif
