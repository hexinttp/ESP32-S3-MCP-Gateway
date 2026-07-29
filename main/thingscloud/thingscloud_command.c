#include "cloud_adapter/cloud_adapter.h"
#include "thingscloud/thingscloud_topics.h"
#include "mqtt_comm/mqtt_handler.h"
#include "amm/amm_mapping.h"
#include "services/control_service.h"

#include <string.h>
#include <stdlib.h>
#include "cJSON.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

static const char *TAG = "TC_CMD";

#define TC_CMD_SNAPSHOT_MAX 256

/* Lazily PSRAM-allocated AMM snapshot buffer (only on first downlink). */
static amm_mapping_entry_t *s_snapshot = NULL;

static bool snapshot_ensure(void)
{
    if (s_snapshot != NULL) return true;
    s_snapshot = heap_caps_calloc(TC_CMD_SNAPSHOT_MAX, sizeof(amm_mapping_entry_t),
                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_snapshot == NULL) {
        s_snapshot = heap_caps_calloc(TC_CMD_SNAPSHOT_MAX, sizeof(amm_mapping_entry_t),
                                      MALLOC_CAP_8BIT);
    }
    if (s_snapshot == NULL) {
        ESP_LOGE(TAG, "AMM snapshot allocation failed");
        return false;
    }
    return true;
}

/* Find the active AMM entry matching a ThingsCloud sub-device address + property key. */
static const amm_mapping_entry_t *find_entry(uint8_t channel_id, uint8_t slave_id,
                                             const char *property_key)
{
    if (!snapshot_ensure()) return NULL;
    amm_mapping_entry_t *snapshot = s_snapshot;
    int n = amm_get_entries(snapshot, TC_CMD_SNAPSHOT_MAX);
    for (int i = 0; i < n; ++i) {
        if (!snapshot[i].active) continue;
        if (snapshot[i].slave_id != slave_id) continue;
        if (channel_id != 0 && snapshot[i].channel_id != channel_id) continue;
        if (strcmp(snapshot[i].point_id, property_key) == 0) {
            return &snapshot[i];
        }
    }
    return NULL;
}

/* Parse one property value into an engineering double. Returns true on success. */
static bool parse_property_value(cJSON *val, double *out)
{
    if (cJSON_IsBool(val)) {
        *out = cJSON_IsTrue(val) ? 1.0 : 0.0;
        return true;
    }
    if (cJSON_IsNumber(val)) {
        *out = val->valuedouble;
        return true;
    }
    if (cJSON_IsString(val)) {
        char *end = NULL;
        double d = strtod(val->valuestring, &end);
        if (end != val->valuestring && *end == '\0') {
            *out = d;
            return true;
        }
    }
    return false;
}

/* Handle a sub-device downlink (attributes/push or command/send). */
static void handle_subdevice_downlink(const char *data, int data_len, cJSON *root)
{
    cJSON *dev = cJSON_GetObjectItemCaseSensitive(root, "device");
    if (!cJSON_IsString(dev)) {
        ESP_LOGW(TAG, "Downlink missing 'device'");
        return;
    }
    uint8_t channel_id = 0, slave_id = 0;
    if (!thingscloud_parse_device_address(dev->valuestring, &channel_id, &slave_id)) {
        ESP_LOGW(TAG, "Downlink invalid device '%s'", dev->valuestring);
        return;
    }

    /* Properties may arrive as an object ("attributes") or a single identifier. */
    cJSON *attrs = cJSON_GetObjectItemCaseSensitive(root, "attributes");
    cJSON *id    = cJSON_GetObjectItemCaseSensitive(root, "identifier");
    cJSON *value = cJSON_GetObjectItemCaseSensitive(root, "value");

    cJSON *reply = cJSON_CreateObject();
    if (reply == NULL) return;
    cJSON_AddStringToObject(reply, "device", dev->valuestring);
    cJSON *results = cJSON_AddArrayToObject(reply, "results");

    cJSON *prop = NULL;
    if (attrs != NULL && cJSON_IsObject(attrs)) {
        cJSON_ArrayForEach(prop, attrs) {
            const char *key = prop->string;
            double eng;
            if (!parse_property_value(prop, &eng)) {
                cJSON *r = cJSON_CreateObject();
                cJSON_AddStringToObject(r, "property", key ? key : "");
                cJSON_AddStringToObject(r, "status", "fail");
                cJSON_AddStringToObject(r, "message", "invalid value");
                cJSON_AddItemToArray(results, r);
                continue;
            }
            const amm_mapping_entry_t *e = find_entry(channel_id, slave_id, key);
            if (e == NULL) {
                cJSON *r = cJSON_CreateObject();
                cJSON_AddStringToObject(r, "property", key ? key : "");
                cJSON_AddStringToObject(r, "status", "fail");
                cJSON_AddStringToObject(r, "message", "no such point");
                cJSON_AddItemToArray(results, r);
                continue;
            }
            if (!e->constraint.writable) {
                cJSON *r = cJSON_CreateObject();
                cJSON_AddStringToObject(r, "property", key ? key : "");
                cJSON_AddStringToObject(r, "status", "fail");
                cJSON_AddStringToObject(r, "message", "read-only");
                cJSON_AddItemToArray(results, r);
                ESP_LOGW(TAG, "Rejected write to read-only point %s/%s",
                         e->device_id, e->point_id);
                continue;
            }
            if (eng < (double)e->constraint.valid_range_min ||
                eng > (double)e->constraint.valid_range_max) {
                cJSON *r = cJSON_CreateObject();
                cJSON_AddStringToObject(r, "property", key ? key : "");
                cJSON_AddStringToObject(r, "status", "fail");
                cJSON_AddStringToObject(r, "message", "out of range");
                cJSON_AddItemToArray(results, r);
                ESP_LOGW(TAG, "Rejected out-of-range write %s/%s=%.6f",
                         e->device_id, e->point_id, eng);
                continue;
            }
            control_result_t result;
            esp_err_t err = control_service_write_point(e->device_id, e->point_id,
                                                        eng, CONTROL_SOURCE_MQTT, &result);
            cJSON *r = cJSON_CreateObject();
            cJSON_AddStringToObject(r, "property", key ? key : "");
            if (err == ESP_OK) {
                cJSON_AddStringToObject(r, "status", "ok");
            } else {
                cJSON_AddStringToObject(r, "status", "fail");
                cJSON_AddStringToObject(r, "message", result.reason);
            }
            cJSON_AddItemToArray(results, r);
            ESP_LOGI(TAG, "Cloud write %s/%s=%.6f -> %s",
                     e->device_id, e->point_id, eng, esp_err_to_name(err));
        }
    } else if (id != NULL && cJSON_IsString(id)) {
        double eng;
        if (parse_property_value(value, &eng)) {
            const amm_mapping_entry_t *e = find_entry(channel_id, slave_id, id->valuestring);
            control_result_t result;
            esp_err_t err = ESP_FAIL;
            if (e != NULL && e->constraint.writable) {
                err = control_service_write_point(e->device_id, e->point_id,
                                                  eng, CONTROL_SOURCE_MQTT, &result);
            }
            cJSON *r = cJSON_CreateObject();
            cJSON_AddStringToObject(r, "property", id->valuestring);
            cJSON_AddStringToObject(r, "status", err == ESP_OK ? "ok" : "fail");
            cJSON_AddItemToArray(results, r);
        }
    }

    char *json = cJSON_PrintUnformatted(reply);
    cJSON_Delete(reply);
    if (json != NULL) {
        mqtt_publish(TC_TOPIC_GATEWAY_CMD_REPLY, json, 0);
        free(json);
    }
}

void thingscloud_on_command_send(const char *topic, const char *data, int data_len)
{
    if (!thingscloud_is_enabled() || data == NULL || data_len <= 0) return;
    cJSON *root = cJSON_ParseWithLength(data, (size_t)data_len);
    if (root == NULL) {
        ESP_LOGW(TAG, "Failed to parse command payload (%d bytes)", data_len);
        return;
    }
    handle_subdevice_downlink(data, data_len, root);
    cJSON_Delete(root);
}

void thingscloud_on_attributes_push(const char *topic, const char *data, int data_len)
{
    /* Sub-device desired attributes (same write path as commands). */
    if (!thingscloud_is_enabled() || data == NULL || data_len <= 0) return;
    cJSON *root = cJSON_ParseWithLength(data, (size_t)data_len);
    if (root == NULL) {
        ESP_LOGW(TAG, "Failed to parse attributes/push payload");
        return;
    }
    handle_subdevice_downlink(data, data_len, root);
    cJSON_Delete(root);
}

void thingscloud_on_gateway_attributes_push(const char *topic, const char *data, int data_len)
{
    /* Gateway self attribute downlink: no writable gateway attributes in this
       firmware; log at info level and ignore. */
    ESP_LOGI(TAG, "Gateway attributes/push received (%d bytes); ignored", data_len);
}
