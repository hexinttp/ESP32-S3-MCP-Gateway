#include "services/control_service.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "amm/amm_mapping.h"
#include "eval/eval_logger.h"
#include "modbus/modbus_access.h"
#include "esp_log.h"
#include "config/runtime_config.h"

static const char *TAG = "CONTROL";

const char *control_source_name(control_source_t source)
{
    switch (source) {
    case CONTROL_SOURCE_MQTT: return "MQTT";
    case CONTROL_SOURCE_REST_API: return "REST_API";
    case CONTROL_SOURCE_MCP: return "MCP";
    case CONTROL_SOURCE_AUTOMATION: return "AUTOMATION";
    case CONTROL_SOURCE_LOCAL_UI: return "LOCAL_UI";
    default: return "UNKNOWN";
    }
}

bool control_source_may_write(control_source_t source)
{
    if (source != CONTROL_SOURCE_MCP) return true;
    /* runtime_config_t is ~2 KiB. Keeping it on the stack inflates the frame of
     * every caller of control_service_write_point (automation task included)
     * even when this branch is not taken, which overflowed small task stacks. */
    runtime_config_t *cfg = malloc(sizeof(*cfg));
    if (cfg == NULL) return false;
    runtime_config_get(cfg);
    bool allowed = cfg->mcp_write_enabled;
    free(cfg);
    return allowed;
}

static uint16_t swap_bytes(uint16_t value)
{
    return (uint16_t)(value << 8 | value >> 8);
}

static uint16_t encode_value(double engineering_value, const amm_mapping_entry_t *mapping,
                             uint16_t words[4])
{
    double scale = mapping->scale_factor == 0.0f ? 1.0 : mapping->scale_factor;
    double raw = (engineering_value - mapping->offset) / scale;
    uint64_t bits = 0;
    uint8_t count = 1;
    switch (mapping->data_type) {
    case DT_INT16: words[0] = (uint16_t)(int16_t)llround(raw); return 1;
    case DT_UINT16: words[0] = (uint16_t)llround(raw); return 1;
    case DT_BOOL: words[0] = raw != 0.0; return 1;
    case DT_BCD16: {
        uint16_t value = (uint16_t)llround(raw);
        words[0] = (uint16_t)(((value / 1000U) % 10U) << 12 |
                              ((value / 100U) % 10U) << 8 |
                              ((value / 10U) % 10U) << 4 |
                              (value % 10U));
        return 1;
    }
    case DT_FLOAT32: {
        float value = raw;
        uint32_t value32;
        memcpy(&value32, &value, sizeof(value32));
        bits = value32;
        count = 2;
        break;
    }
    case DT_INT32: bits = (uint32_t)(int32_t)llround(raw); count = 2; break;
    case DT_UINT32: bits = (uint32_t)llround(raw); count = 2; break;
    case DT_FLOAT64: {
        double value = raw;
        memcpy(&bits, &value, sizeof(bits));
        count = 4;
        break;
    }
    case DT_INT64: bits = (uint64_t)(int64_t)llround(raw); count = 4; break;
    case DT_UINT64: bits = (uint64_t)llround(raw); count = 4; break;
    default: return 0;
    }
    for (uint8_t i = 0; i < count; ++i) {
        words[i] = (uint16_t)(bits >> (16U * (count - 1U - i)));
    }
    if (mapping->byte_order == BYTE_ORDER_CDAB || mapping->byte_order == BYTE_ORDER_DCBA) {
        for (uint8_t i = 0; i < count / 2U; ++i) {
            uint16_t temp = words[i];
            words[i] = words[count - 1U - i];
            words[count - 1U - i] = temp;
        }
    }
    if (mapping->byte_order == BYTE_ORDER_BADC || mapping->byte_order == BYTE_ORDER_DCBA) {
        for (uint8_t i = 0; i < count; ++i) words[i] = swap_bytes(words[i]);
    }
    return count;
}

static esp_err_t reject(control_result_t *result, esp_err_t status, const char *reason)
{
    if (result != NULL) {
        result->status = status;
        strlcpy(result->reason, reason, sizeof(result->reason));
    }
    eval_increment_metric("commands_rejected", 1);
    return status;
}

esp_err_t control_service_write_point(const char *device_id, const char *point_id,
                                      double engineering_value, control_source_t source,
                                      control_result_t *result)
{
    if (result != NULL) {
        memset(result, 0, sizeof(*result));
        result->source = source;
    }
    if (device_id == NULL || point_id == NULL || !isfinite(engineering_value)) {
        return reject(result, ESP_ERR_INVALID_ARG, "invalid target or value");
    }
    eval_increment_metric("commands_received", 1);
    /* Source-aware policy. The MCP origin is additionally gated by the global
       write switch; all other origins are validated by their own upstream
       layers. A write must never bypass this single entry point. */
    if (!control_source_may_write(source)) {
        return reject(result, ESP_ERR_NOT_ALLOWED,
                      "writes from this source are not permitted");
    }
    amm_mapping_entry_t mapping;
    if (amm_find_mapping_by_point(device_id, point_id, &mapping) != ESP_OK) {
        return reject(result, ESP_ERR_NOT_FOUND, "semantic point not found");
    }
    if (!mapping.constraint.writable) {
        return reject(result, ESP_ERR_NOT_ALLOWED, "point is read-only");
    }
    if (engineering_value < mapping.constraint.valid_range_min ||
        engineering_value > mapping.constraint.valid_range_max) {
        return reject(result, ESP_ERR_INVALID_ARG, "value outside AMM safety range");
    }

    uint16_t words[4] = {0};
    uint16_t count = encode_value(engineering_value, &mapping, words);
    if (count == 0) return reject(result, ESP_ERR_NOT_SUPPORTED, "unsupported data type");
    uint8_t function_code = mapping.object_type == MODBUS_OBJECT_COIL
        ? (count == 1 ? 5 : 15)
        : (count == 1 ? 6 : 16);
    uint16_t address = modbus_register_offset(function_code, mapping.register_address);
    esp_err_t err = modbus_write_channel(mapping.source_protocol, mapping.channel_id,
                                         mapping.slave_id, function_code, address,
                                         count, words);
    if (err != ESP_OK) {
        char reason[96];
        snprintf(reason, sizeof(reason), "Modbus write failed: %s", esp_err_to_name(err));
        return reject(result, err, reason);
    }
    eval_increment_metric("commands_accepted", 1);
    if (result != NULL) {
        result->status = ESP_OK;
        strlcpy(result->reason, "write accepted", sizeof(result->reason));
    }
    ESP_LOGI(TAG, "%s/%s = %.6g source=%s", device_id, point_id,
             engineering_value, control_source_name(source));
    return ESP_OK;
}
