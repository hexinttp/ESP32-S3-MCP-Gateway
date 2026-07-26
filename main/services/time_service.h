#ifndef TIME_SERVICE_H
#define TIME_SERVICE_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    bool enabled;
    bool synchronized;
    int64_t last_sync_ms;
    int64_t current_time_ms;
    char server[64];
} time_service_status_t;

esp_err_t time_service_init(void);
void time_service_get_status(time_service_status_t *out);
bool time_service_is_synchronized(void);

#endif
