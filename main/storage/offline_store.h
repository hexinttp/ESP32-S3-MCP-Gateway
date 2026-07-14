#ifndef OFFLINE_STORE_H
#define OFFLINE_STORE_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "gateway_config.h"

typedef enum {
    OFFLINE_MEDIUM_FLASH = 0,
    OFFLINE_MEDIUM_TF,
} offline_medium_t;

typedef struct {
    uint32_t sequence_id;
    char topic[128];
    char payload[TCM_MAX_JSON_LEN];
    offline_medium_t medium;
} offline_record_t;

esp_err_t offline_store_init(void);
esp_err_t offline_store_put(uint32_t sequence_id, const char *topic, const char *payload);
esp_err_t offline_store_peek_oldest(offline_record_t *out);
esp_err_t offline_store_remove(uint32_t sequence_id);
int offline_store_count(void);
int offline_store_usage_percent(void);
int offline_store_data_loss_count(void);
bool offline_store_flash_ready(void);

#endif
