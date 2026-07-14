#include "services/control_service.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "amm/amm_mapping.h"
#include "eval/eval_logger.h"
#include "modbus/modbus_access.h"
#include "esp_log.h"

static const char *TAG = "CONTROL";

static uint16_t swap_bytes(uint16_t value)
{
    return (uint16_t)(value << 8 | value >> 8);
}

static uint16_t encode_value(float engineering_value, const amm_mapping_entry_t *mapping,
                             uint16_t words[2])
{
    float scale = mapping->scale_factor == 0.0f ? 1.0f : mapping->scale_factor;
    float raw = (engineering_value - mapping->offset) / scale;
    uint32_t value32 = 0;
    switch (mapping->data_type) {
    case DT_INT16: words[0] = (uint16_t)(int16_t)lroundf(raw); return 1;
    case DT_UINT16: words[0] = (uint16_t)lroundf(raw); return 1;
    case DT_FLOAT32: { float value = raw; memcpy(&value32, &value, sizeof(value32)); break; }
    case DT_INT32: value32 = (uint32_t)(int32_t)lroundf(raw); break;
    case DT_UINT32: value32 = (uint32_t)llroundf(raw); break;
    default: return 0;
    }
    words[0] = (uint16_t)(value32 >> 16);
    words[1] = (uint16_t)value32;
    if (mapping->byte_order == BYTE_ORDER_CDAB || mapping->byte_order == BYTE_ORDER_DCBA) {
        uint16_t temp = words[0]; words[0] = words[1]; words[1] = temp;
    }
    if (mapping->byte_order == BYTE_ORDER_BADC || mapping->byte_order == BYTE_ORDER_DCBA) {
        words[0] = swap_bytes(words[0]);
        words[1] = swap_bytes(words[1]);
    }
    return 2;
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
                                      float engineering_value, control_source_t source,
                                      control_result_t *result)
{
    (void)source;
    if (result != NULL) memset(result, 0, sizeof(*result));
    if (device_id == NULL || point_id == NULL || !isfinite(engineering_value)) {
        return reject(result, ESP_ERR_INVALID_ARG, "invalid target or value");
    }
    eval_increment_metric("commands_received", 1);
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

    uint16_t words[2] = {0};
    uint16_t count = encode_value(engineering_value, &mapping, words);
    if (count == 0) return reject(result, ESP_ERR_NOT_SUPPORTED, "unsupported data type");
    uint8_t function_code = count == 1 ? 6 : 16;
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
    ESP_LOGI(TAG, "%s/%s = %.3f", device_id, point_id, engineering_value);
    return ESP_OK;
}
