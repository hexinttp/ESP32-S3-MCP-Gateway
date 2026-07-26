#ifndef OTA_SERVICE_H
#define OTA_SERVICE_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef enum {
    OTA_STATE_IDLE = 0,
    OTA_STATE_DOWNLOADING,
    OTA_STATE_VERIFYING,
    OTA_STATE_READY_TO_REBOOT,
    OTA_STATE_FAILED
} ota_state_t;

typedef struct {
    ota_state_t state;
    esp_err_t last_error;
    char message[96];
} ota_status_t;

esp_err_t ota_service_init(void);
esp_err_t ota_service_start(const char *url, const char *expected_sha256);
void ota_service_get_status(ota_status_t *out);

#endif
