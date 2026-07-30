#include "cloud_adapter/cloud_adapter.h"
#include "thingscloud/thingscloud_topics.h"
#include "mqtt_comm/mqtt_handler.h"
#include "amm/amm_mapping.h"
#include "services/control_service.h"
#include "uif/uif_persistence.h"
#include "web/web_server.h"

#include <string.h>
#include <stdlib.h>
#include "cJSON.h"
#include "esp_log.h"

static const char *TAG = "TC_CMD";

/* Find the active AMM entry matching a ThingsCloud sub-device address + property key. */
static bool find_entry(uint8_t channel_id, uint8_t slave_id,
                       const char *property_key, amm_mapping_entry_t *out)
{
    if (property_key == NULL || out == NULL) return false;
    int n = amm_get_mapping_count();
    for (int i = 0; i < n; ++i) {
        amm_mapping_entry_t entry;
        if (amm_get_entry_at(i, &entry) != ESP_OK) continue;
        if (!entry.active || entry.source_protocol != SRC_MODBUS_RTU) continue;
        if (entry.channel_id != channel_id || entry.slave_id != slave_id) continue;
        if (strcmp(entry.point_id, property_key) == 0) {
            *out = entry;
            return true;
        }
    }
    return false;
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
            amm_mapping_entry_t entry;
            if (!find_entry(channel_id, slave_id, key, &entry)) {
                cJSON *r = cJSON_CreateObject();
                cJSON_AddStringToObject(r, "property", key ? key : "");
                cJSON_AddStringToObject(r, "status", "fail");
                cJSON_AddStringToObject(r, "message", "no such point");
                cJSON_AddItemToArray(results, r);
                continue;
            }
            const amm_mapping_entry_t *e = &entry;
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
        cJSON *r = cJSON_CreateObject();
        cJSON_AddStringToObject(r, "property", id->valuestring);
        if (!parse_property_value(value, &eng)) {
            cJSON_AddStringToObject(r, "status", "fail");
            cJSON_AddStringToObject(r, "message", "invalid value");
        } else {
            amm_mapping_entry_t entry;
            control_result_t result;
            esp_err_t err = ESP_FAIL;
            const char *message = "no such point";
            if (find_entry(channel_id, slave_id, id->valuestring, &entry)) {
                if (!entry.constraint.writable) {
                    message = "read-only";
                } else if (eng < (double)entry.constraint.valid_range_min ||
                           eng > (double)entry.constraint.valid_range_max) {
                    message = "out of range";
                } else {
                    err = control_service_write_point(entry.device_id, entry.point_id,
                                                  eng, CONTROL_SOURCE_MQTT, &result);
                    message = err == ESP_OK ? "" : result.reason;
                }
            }
            cJSON_AddStringToObject(r, "status", err == ESP_OK ? "ok" : "fail");
            if (err != ESP_OK) cJSON_AddStringToObject(r, "message", message);
        }
        cJSON_AddItemToArray(results, r);
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

static void handle_attributes_response(bool gateway_topic, const char *data,
                                       int data_len)
{
    if (data == NULL || data_len <= 0) return;

    cJSON *root = cJSON_ParseWithLength(data, (size_t)data_len);
    bool accepted = false;
    int error_code = 0;
    const char *message = "";
    if (root != NULL) {
        cJSON *result = cJSON_GetObjectItemCaseSensitive(root, "result");
        cJSON *errcode = cJSON_GetObjectItemCaseSensitive(root, "errcode");
        cJSON *message_item = cJSON_GetObjectItemCaseSensitive(root, "message");
        accepted = (cJSON_IsNumber(result) && result->valueint == 1) ||
                   cJSON_IsTrue(result);
        if (cJSON_IsNumber(errcode)) error_code = errcode->valueint;
        if (cJSON_IsString(message_item)) message = message_item->valuestring;
    }

    char log_text[192];
    if (root == NULL) {
        snprintf(log_text, sizeof(log_text),
                 "[MQTT RX] ThingsCloud %s response has invalid JSON (%d bytes)",
                 gateway_topic ? "sub-device" : "gateway", data_len);
        ESP_LOGW(TAG, "%s", log_text);
        web_server_add_log("error", log_text);
    } else if (accepted) {
        snprintf(log_text, sizeof(log_text),
                 "[MQTT RX] ThingsCloud %s attributes accepted",
                 gateway_topic ? "sub-device" : "gateway");
        ESP_LOGI(TAG, "%s", log_text);
        web_server_add_log("ok", log_text);
    } else {
        snprintf(log_text, sizeof(log_text),
                 "[MQTT RX] ThingsCloud %s attributes rejected: code=%d %.96s",
                 gateway_topic ? "sub-device" : "gateway",
                 error_code, message);
        ESP_LOGW(TAG, "%s", log_text);
        web_server_add_log("error", log_text);
    }

    if (root != NULL) {
        thingscloud_record_publish_response(accepted, error_code);
        uif_replay_cloud_response(gateway_topic, accepted, error_code);
        cJSON_Delete(root);
    }
    if (error_code == 413) {
        /* Keep the MQTT control plane alive. The broker may close this
         * transport after rejecting the message, but ESP-MQTT will reconnect
         * and the adapter guard prevents another telemetry publish. */
        web_server_add_log(
            "warn",
            "[MQTT] Telemetry suspended after cloud 413; control connection will recover");
    }
}

void thingscloud_on_attributes_response(const char *topic, const char *data,
                                        int data_len)
{
    (void)topic;
    handle_attributes_response(false, data, data_len);
}

void thingscloud_on_gateway_attributes_response(const char *topic,
                                                const char *data, int data_len)
{
    (void)topic;
    handle_attributes_response(true, data, data_len);
}
