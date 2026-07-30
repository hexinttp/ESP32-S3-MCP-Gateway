#ifndef CONTROL_SERVICE_H
#define CONTROL_SERVICE_H

#include "esp_err.h"
#include <stdbool.h>

/* Write origins. Every Modbus write must declare its origin so the control
   service can apply a per-source policy. Local UI / automation / MQTT / REST
   carry their own upstream authorization; the MCP origin is additionally gated
   by the global write switch because MCP writes require the two-phase
   preview/commit flow (added in a later phase) and must never be a single
   direct write. */
typedef enum {
    CONTROL_SOURCE_MQTT = 0,
    CONTROL_SOURCE_REST_API,
    CONTROL_SOURCE_MCP,
    CONTROL_SOURCE_AUTOMATION,
    CONTROL_SOURCE_LOCAL_UI,
} control_source_t;

typedef struct {
    esp_err_t status;
    char reason[128];
    control_source_t source;
} control_result_t;

const char *control_source_name(control_source_t source);

/* Returns true if a write from `source` is permitted at the control-service
   level. Only the MCP origin is gated here (by the global MCP write switch);
   other origins are validated by their own upstream layers. */
bool control_source_may_write(control_source_t source);

esp_err_t control_service_write_point(const char *device_id, const char *point_id,
                                      double engineering_value, control_source_t source,
                                      control_result_t *result);

#endif
