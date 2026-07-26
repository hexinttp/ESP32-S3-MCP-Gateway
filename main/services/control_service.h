#ifndef CONTROL_SERVICE_H
#define CONTROL_SERVICE_H

#include "esp_err.h"

typedef enum {
    CONTROL_SOURCE_MQTT = 0,
    CONTROL_SOURCE_WEB,
    CONTROL_SOURCE_MCP,
    CONTROL_SOURCE_AUTOMATION,
} control_source_t;

typedef struct {
    esp_err_t status;
    char reason[128];
} control_result_t;

esp_err_t control_service_write_point(const char *device_id, const char *point_id,
                                      double engineering_value, control_source_t source,
                                      control_result_t *result);

#endif
