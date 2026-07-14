#include "tcm_context.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "cJSON.h"
#include "config/runtime_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"

#define TCM_VERSION "1.0"
#define TCM_SEQUENCE_RESERVATION 32U

static const char *TAG = "TCM";
static uint32_t s_sequence_counter;
static uint32_t s_sequence_reserved_to;
static uint32_t s_context_counter;
static SemaphoreHandle_t s_mutex;

static const char *protocol_string(source_protocol_t value)
{
    return value == SRC_MODBUS_TCP ? "MODBUS_TCP" : "MODBUS_RTU";
}

static const char *data_type_string(data_type_t value)
{
    static const char *names[] = {"int16", "uint16", "float32", "int32", "uint32"};
    return value <= DT_UINT32 ? names[value] : "uint16";
}

static const char *byte_order_string(byte_order_t value)
{
    static const char *names[] = {"ABCD", "CDAB", "BADC", "DCBA"};
    return value <= BYTE_ORDER_DCBA ? names[value] : "ABCD";
}

static const char *quality_string(quality_state_t value)
{
    static const char *names[] = {"good", "stale", "invalid"};
    return value <= QUALITY_INVALID ? names[value] : "invalid";
}

static const char *network_string(network_state_t value)
{
    static const char *names[] = {"online", "delayed", "offline", "replayed"};
    return value <= NET_REPLAYED ? names[value] : "offline";
}

static const char *operation_string(operation_type_t value)
{
    static const char *names[] = {"read_publish", "subscribe", "write", "replay", "read_only"};
    return value <= OP_READ_ONLY ? names[value] : "read_publish";
}

static data_type_t parse_data_type(const char *value)
{
    if (!value) return DT_UINT16;
    if (!strcmp(value, "int16")) return DT_INT16;
    if (!strcmp(value, "float32")) return DT_FLOAT32;
    if (!strcmp(value, "int32")) return DT_INT32;
    if (!strcmp(value, "uint32")) return DT_UINT32;
    return DT_UINT16;
}

static byte_order_t parse_byte_order(const char *value)
{
    if (value && !strcmp(value, "CDAB")) return BYTE_ORDER_CDAB;
    if (value && !strcmp(value, "BADC")) return BYTE_ORDER_BADC;
    if (value && !strcmp(value, "DCBA")) return BYTE_ORDER_DCBA;
    return BYTE_ORDER_ABCD;
}

static int64_t current_time_ms(void)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    if (now.tv_sec > 1609459200) return (int64_t)now.tv_sec * 1000 + now.tv_usec / 1000;
    return esp_timer_get_time() / 1000;
}

static void persist_sequence(uint32_t reserved_to)
{
    nvs_handle_t nvs;
    if (nvs_open("tcm_state", NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_u32(nvs, "seq_reserved", reserved_to);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

static uint32_t next_sequence(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_sequence_counter >= s_sequence_reserved_to) {
        s_sequence_reserved_to += TCM_SEQUENCE_RESERVATION;
        persist_sequence(s_sequence_reserved_to);
    }
    uint32_t value = ++s_sequence_counter;
    xSemaphoreGive(s_mutex);
    return value;
}

void tcm_init(void)
{
    if (s_mutex == NULL) s_mutex = xSemaphoreCreateMutex();
    uint32_t reserved = 0;
    nvs_handle_t nvs;
    if (nvs_open("tcm_state", NVS_READONLY, &nvs) == ESP_OK) {
        nvs_get_u32(nvs, "seq_reserved", &reserved);
        nvs_close(nvs);
    }
    s_sequence_counter = reserved;
    s_sequence_reserved_to = reserved + TCM_SEQUENCE_RESERVATION;
    persist_sequence(s_sequence_reserved_to);
    s_context_counter = 0;
    ESP_LOGI(TAG, "TCM %s initialized, sequence starts at %lu", TCM_VERSION,
             (unsigned long)s_sequence_counter + 1);
}

int tcm_build_context(tcm_context_t *ctx, uint8_t slave_id, uint8_t func_code,
                      uint16_t reg_addr, float raw_value, quality_state_t quality,
                      network_state_t net_state)
{
    if (ctx == NULL || slave_id == 0 || (func_code != 3 && func_code != 4)) return -1;
    memset(ctx, 0, sizeof(*ctx));
    runtime_config_t config;
    runtime_config_get(&config);
    strlcpy(ctx->tcm_version, TCM_VERSION, sizeof(ctx->tcm_version));
    strlcpy(ctx->gateway_id, config.gateway_id, sizeof(ctx->gateway_id));
    ctx->context_id = ++s_context_counter;
    snprintf(ctx->device_id, sizeof(ctx->device_id), "modbus_%u", slave_id);
    snprintf(ctx->point_id, sizeof(ctx->point_id), "reg_%u", reg_addr);
    strlcpy(ctx->measurement_name, ctx->point_id, sizeof(ctx->measurement_name));
    ctx->source_protocol = SRC_MODBUS_RTU;
    ctx->slave_id = slave_id;
    ctx->function_code = func_code;
    ctx->register_address = reg_addr;
    ctx->data_type = DT_UINT16;
    ctx->byte_order = BYTE_ORDER_ABCD;
    ctx->raw_value = raw_value;
    ctx->scale_factor = 1.0f;
    ctx->value = raw_value;
    ctx->timestamp_ms = current_time_ms();
    ctx->quality_state = quality;
    ctx->network_state = net_state;
    ctx->operation_type = OP_READ_PUBLISH;
    ctx->sequence_id = next_sequence();
    return 0;
}

bool tcm_validate(const tcm_context_t *ctx, tcm_validation_result_t *result)
{
    if (result == NULL) return false;
    memset(result, 0, sizeof(*result));
    if (ctx == NULL) {
        strlcpy(result->fail_reason, "null context", sizeof(result->fail_reason));
        return false;
    }
    if (ctx->device_id[0] == '\0') result->failed_field_mask |= 1U << 1;
    if (ctx->point_id[0] == '\0') result->failed_field_mask |= 1U << 2;
    if (ctx->slave_id == 0 || ctx->slave_id > 247) result->failed_field_mask |= 1U << 4;
    if (ctx->function_code != 3 && ctx->function_code != 4 &&
        ctx->function_code != 6 && ctx->function_code != 16) result->failed_field_mask |= 1U << 5;
    if (ctx->data_type > DT_UINT32) result->failed_field_mask |= 1U << 7;
    if (ctx->operation_type > OP_READ_ONLY) result->failed_field_mask |= 1U << 14;
    result->passed = result->failed_field_mask == 0;
    if (!result->passed) {
        snprintf(result->fail_reason, sizeof(result->fail_reason),
                 "TCM schema validation failed (mask=0x%08lx)",
                 (unsigned long)result->failed_field_mask);
    }
    return result->passed;
}

int tcm_serialize_json(const tcm_context_t *ctx, char *json_buf, size_t buf_size)
{
    if (ctx == NULL || json_buf == NULL || buf_size == 0) return -1;
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) return -1;
    cJSON_AddStringToObject(root, "tcm_version", ctx->tcm_version);
    cJSON_AddStringToObject(root, "gateway_id", ctx->gateway_id);
    cJSON_AddNumberToObject(root, "context_id", ctx->context_id);
    cJSON_AddNumberToObject(root, "sequence_id", ctx->sequence_id);
    cJSON_AddNumberToObject(root, "mapping_version", ctx->mapping_version);
    cJSON_AddStringToObject(root, "device_id", ctx->device_id);
    cJSON_AddStringToObject(root, "point_id", ctx->point_id);
    cJSON_AddStringToObject(root, "source_protocol", protocol_string(ctx->source_protocol));
    cJSON_AddNumberToObject(root, "channel_id", ctx->channel_id);
    cJSON_AddNumberToObject(root, "slave_id", ctx->slave_id);
    cJSON_AddNumberToObject(root, "function_code", ctx->function_code);
    cJSON_AddNumberToObject(root, "register_address", ctx->register_address);
    cJSON_AddStringToObject(root, "data_type", data_type_string(ctx->data_type));
    cJSON_AddStringToObject(root, "byte_order", byte_order_string(ctx->byte_order));
    cJSON_AddStringToObject(root, "measurement_name", ctx->measurement_name);
    cJSON_AddStringToObject(root, "unit", ctx->unit);
    cJSON_AddNumberToObject(root, "raw_value", ctx->raw_value);
    cJSON_AddNumberToObject(root, "scale_factor", ctx->scale_factor);
    cJSON_AddNumberToObject(root, "offset", ctx->offset);
    cJSON_AddNumberToObject(root, "value", ctx->value);
    cJSON_AddNumberToObject(root, "timestamp_ms", (double)ctx->timestamp_ms);
    cJSON_AddStringToObject(root, "quality_state", quality_string(ctx->quality_state));
    cJSON_AddStringToObject(root, "network_state", network_string(ctx->network_state));
    cJSON_AddStringToObject(root, "operation_type", operation_string(ctx->operation_type));
    cJSON *constraint = cJSON_AddObjectToObject(root, "control_constraint");
    cJSON_AddBoolToObject(constraint, "writable", ctx->control_constraint.writable);
    cJSON_AddNumberToObject(constraint, "min", ctx->control_constraint.valid_range_min);
    cJSON_AddNumberToObject(constraint, "max", ctx->control_constraint.valid_range_max);
    bool ok = cJSON_PrintPreallocated(root, json_buf, (int)buf_size, false);
    cJSON_Delete(root);
    return ok ? (int)strlen(json_buf) : -1;
}

static cJSON *item(cJSON *root, const char *name) { return cJSON_GetObjectItemCaseSensitive(root, name); }
static void copy_json_string(cJSON *root, const char *name, char *out, size_t size)
{
    cJSON *value = item(root, name);
    if (cJSON_IsString(value)) strlcpy(out, value->valuestring, size);
}

int tcm_deserialize_json(const char *json_str, tcm_context_t *ctx)
{
    if (json_str == NULL || ctx == NULL) return -1;
    cJSON *root = cJSON_Parse(json_str);
    if (root == NULL) return -1;
    memset(ctx, 0, sizeof(*ctx));
    strlcpy(ctx->tcm_version, TCM_VERSION, sizeof(ctx->tcm_version));
    copy_json_string(root, "tcm_version", ctx->tcm_version, sizeof(ctx->tcm_version));
    copy_json_string(root, "gateway_id", ctx->gateway_id, sizeof(ctx->gateway_id));
    copy_json_string(root, "device_id", ctx->device_id, sizeof(ctx->device_id));
    copy_json_string(root, "point_id", ctx->point_id, sizeof(ctx->point_id));
    copy_json_string(root, "measurement_name", ctx->measurement_name, sizeof(ctx->measurement_name));
    copy_json_string(root, "unit", ctx->unit, sizeof(ctx->unit));
    cJSON *value;
#define JSON_UINT(field, key) do { value = item(root, key); if (cJSON_IsNumber(value)) ctx->field = value->valuedouble; } while (0)
    JSON_UINT(context_id, "context_id"); JSON_UINT(sequence_id, "sequence_id");
    JSON_UINT(mapping_version, "mapping_version"); JSON_UINT(channel_id, "channel_id");
    JSON_UINT(slave_id, "slave_id"); JSON_UINT(function_code, "function_code");
    JSON_UINT(register_address, "register_address"); JSON_UINT(timestamp_ms, "timestamp_ms");
#undef JSON_UINT
    value = item(root, "value"); if (cJSON_IsNumber(value)) ctx->value = value->valuedouble;
    value = item(root, "raw_value"); if (cJSON_IsNumber(value)) ctx->raw_value = value->valuedouble;
    value = item(root, "scale_factor"); ctx->scale_factor = cJSON_IsNumber(value) ? value->valuedouble : 1.0f;
    value = item(root, "offset"); if (cJSON_IsNumber(value)) ctx->offset = value->valuedouble;
    value = item(root, "source_protocol");
    ctx->source_protocol = cJSON_IsString(value) && !strcmp(value->valuestring, "MODBUS_TCP") ? SRC_MODBUS_TCP : SRC_MODBUS_RTU;
    value = item(root, "data_type"); ctx->data_type = parse_data_type(cJSON_IsString(value) ? value->valuestring : NULL);
    value = item(root, "byte_order"); ctx->byte_order = parse_byte_order(cJSON_IsString(value) ? value->valuestring : NULL);
    value = item(root, "operation_type");
    ctx->operation_type = cJSON_IsString(value) && !strcmp(value->valuestring, "write") ? OP_WRITE : OP_READ_PUBLISH;
    cJSON *constraint = item(root, "control_constraint");
    if (cJSON_IsObject(constraint)) {
        value = item(constraint, "writable"); ctx->control_constraint.writable = cJSON_IsTrue(value);
        value = item(constraint, "min"); if (cJSON_IsNumber(value)) ctx->control_constraint.valid_range_min = value->valuedouble;
        value = item(constraint, "max"); if (cJSON_IsNumber(value)) ctx->control_constraint.valid_range_max = value->valuedouble;
    }
    cJSON_Delete(root);
    return 0;
}

uint32_t tcm_get_sequence_counter(void) { return s_sequence_counter; }
void tcm_update_network_state(tcm_context_t *ctx, network_state_t new_state)
{
    if (ctx != NULL) ctx->network_state = new_state;
}
