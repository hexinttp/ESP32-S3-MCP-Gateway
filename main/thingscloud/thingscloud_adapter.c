#include "cloud_adapter/cloud_adapter.h"
#include "thingscloud/thingscloud_topics.h"
#include "mqtt_comm/mqtt_handler.h"
#include "config/runtime_config.h"
#include "tcm/tcm_context.h"

#include <string.h>
#include <math.h>
#include "cJSON.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "network/network_manager.h"
#include "uif/uif_persistence.h"

static const char *TAG = "TC_ADAPTER";

/* Per-cycle aggregation buffer of sub-device property updates.
 * Lazily allocated from PSRAM (only when ThingsCloud mode is active) to
 * avoid consuming scarce internal DRAM in custom-MQTT mode. */
static cloud_property_update_t *s_agg = NULL;
static int s_agg_count = 0;

/* Gateway-mode aggregation buffer: all data reported as the gateway's own
 * attributes (keyed by "<slave_id>_<point_id>"). Separate from the
 * sub-device buffer so the two reporting modes never interleave. */
static cloud_property_update_t *s_gw_agg = NULL;
static int s_gw_count = 0;
static SemaphoreHandle_t s_adapter_mutex = NULL;
static uint32_t s_throttled_count = 0;
static uint32_t s_dropped_count = 0;
static int64_t s_last_publish_ms = 0;

static bool adapter_lock(void)
{
    if (s_adapter_mutex == NULL) {
        s_adapter_mutex = xSemaphoreCreateMutex();
    }
    return s_adapter_mutex != NULL &&
           xSemaphoreTake(s_adapter_mutex, pdMS_TO_TICKS(1000)) == pdTRUE;
}

static void adapter_unlock(void)
{
    if (s_adapter_mutex != NULL) xSemaphoreGive(s_adapter_mutex);
}

/* ---------------------------------------------------------------
 * Upstream publish rate limiter (ThingsCloud flow-control guard)
 * ---------------------------------------------------------------
 * ThingsCloud enforces a per-device message rate limit; exceeding it
 * triggers server-side flow control that blocks the connection for up
 * to ~30 minutes. To stay safely under the limit we gate every telemetry
 * PUBLISH through a token-bucket limiter:
 *   - TC_RATE_BUCKET_TOKENS : burst capacity (tokens available at once)
 *   - TC_RATE_REFILL_MS     : ms to regenerate one token (=> steady rate)
 * Control messages (gateway/connect, gateway/disconnect) are NOT limited
 * because they are low-frequency and must not be dropped. */
#define TC_RATE_BUCKET_TOKENS   8
#define TC_RATE_REFILL_MS       3000    /* 1 token / 3s => ~20 msg/min steady */

static int      s_rate_tokens  = TC_RATE_BUCKET_TOKENS;
static int64_t  s_rate_last_ms = 0;

static void tc_rate_refill(int64_t now_ms)
{
    if (s_rate_last_ms == 0) {
        s_rate_last_ms = now_ms;
        return;
    }
    int64_t elapsed = now_ms - s_rate_last_ms;
    if (elapsed < TC_RATE_REFILL_MS) return;
    int add = (int)(elapsed / TC_RATE_REFILL_MS);
    s_rate_tokens += add;
    if (s_rate_tokens > TC_RATE_BUCKET_TOKENS) s_rate_tokens = TC_RATE_BUCKET_TOKENS;
    s_rate_last_ms += (int64_t)add * TC_RATE_REFILL_MS;
}

/* Consume one token if available. Returns true when a telemetry publish
 * may proceed. When false, the caller must KEEP its aggregation buffer so
 * the data is retried on the next (throttled) flush instead of being lost. */
static bool tc_rate_allow(void)
{
    int64_t now_ms = esp_timer_get_time() / 1000;
    tc_rate_refill(now_ms);
    if (s_rate_tokens > 0) {
        s_rate_tokens--;
        return true;
    }
    return false;
}

/* Telemetry publish that respects the rate limiter. Returns ESP_OK on
 * success, ESP_ERR_NO_MEM when throttled (caller keeps the buffer). */
static esp_err_t tc_publish_telemetry(const char *topic, const char *payload)
{
    if (!tc_rate_allow()) {
        s_throttled_count++;
        ESP_LOGW(TAG, "Flow-control throttle: holding telemetry to '%s' (rate limited)", topic);
        return ESP_ERR_NO_MEM;
    }
    esp_err_t err = mqtt_publish(topic, payload, 0);
    if (err == ESP_OK) s_last_publish_ms = esp_timer_get_time() / 1000;
    return err;
}

/* ================= Gateway property key helpers ================= */
static bool tc_key_char_valid(char c)
{
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
           (c >= '0' && c <= '9') || (c == '_');
}

bool thingscloud_gateway_key_valid(const char *key)
{
    if (key == NULL || key[0] == '\0') return false;
    for (const char *c = key; *c != '\0'; ++c) {
        if (!tc_key_char_valid(*c)) return false;
    }
    return true;
}

esp_err_t build_gateway_property_key(uint8_t port_id, uint8_t slave_id,
                                     const char *property_key,
                                     char *output, size_t output_size)
{
    if (output == NULL || output_size == 0) return ESP_ERR_INVALID_ARG;
    output[0] = '\0';
    if (!thingscloud_gateway_key_valid(property_key)) return ESP_ERR_INVALID_ARG;
    int n = snprintf(output, output_size, "p%u_s%u_%s",
                     (unsigned)port_id, (unsigned)slave_id, property_key);
    if (n < 0 || (size_t)n >= output_size) {
        output[0] = '\0';
        return ESP_ERR_INVALID_SIZE;
    }
    return ESP_OK;
}

/* ================= Report-mode generation guard ================= */
/* Incremented whenever the active report mode changes. Aggregation buffers
 * produced under a previous mode are discarded so stale data is never
 * published under the new mode (see check_mode_change()). */
static volatile uint32_t s_config_generation = 0;
static int s_last_report_mode = -1;

uint32_t thingscloud_get_config_generation(void)
{
    return s_config_generation;
}

/* Detect a report-mode switch; on change bump the generation and clear the
 * per-mode aggregation buffers so no stale (old-mode) points are sent. */
static void check_mode_change(mqtt_report_mode_t mode)
{
    if (s_last_report_mode == (int)mode) return;
    s_last_report_mode = (int)mode;
    s_config_generation++;
    s_agg_count = 0;
    s_gw_count = 0;
    ESP_LOGI(TAG, "Report mode -> %d, generation %lu, aggregation buffers cleared",
             (int)mode, (unsigned long)s_config_generation);
}

static bool agg_ensure_buffer(void)
{
    if (s_agg != NULL) return true;
    s_agg = heap_caps_calloc(THINGSCLOUD_AGG_BUFFER_MAX,
                             sizeof(cloud_property_update_t),
                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_agg == NULL) {
        s_agg = heap_caps_calloc(THINGSCLOUD_AGG_BUFFER_MAX,
                                 sizeof(cloud_property_update_t),
                                 MALLOC_CAP_8BIT);
    }
    if (s_agg == NULL) {
        ESP_LOGE(TAG, "Aggregation buffer allocation failed (%u bytes)",
                 (unsigned)(THINGSCLOUD_AGG_BUFFER_MAX * sizeof(cloud_property_update_t)));
        return false;
    }
    return true;
}

/* ---------------- value setters ---------------- */
void cloud_property_set_bool(cloud_property_update_t *p, const char *dev, const char *key, bool v)
{
    if (!p) return;
    strlcpy(p->device_address, dev ? dev : "", sizeof(p->device_address));
    strlcpy(p->property_key, key ? key : "", sizeof(p->property_key));
    p->value_type = CLOUD_VALUE_BOOL;
    p->value.boolean_value = v;
}

void cloud_property_set_integer(cloud_property_update_t *p, const char *dev, const char *key, int64_t v)
{
    if (!p) return;
    strlcpy(p->device_address, dev ? dev : "", sizeof(p->device_address));
    strlcpy(p->property_key, key ? key : "", sizeof(p->property_key));
    p->value_type = CLOUD_VALUE_INTEGER;
    p->value.integer_value = v;
}

void cloud_property_set_number(cloud_property_update_t *p, const char *dev, const char *key, double v)
{
    if (!p) return;
    strlcpy(p->device_address, dev ? dev : "", sizeof(p->device_address));
    strlcpy(p->property_key, key ? key : "", sizeof(p->property_key));
    p->value_type = CLOUD_VALUE_NUMBER;
    p->value.number_value = v;
}

void cloud_property_set_string(cloud_property_update_t *p, const char *dev, const char *key, const char *v)
{
    if (!p) return;
    strlcpy(p->device_address, dev ? dev : "", sizeof(p->device_address));
    strlcpy(p->property_key, key ? key : "", sizeof(p->property_key));
    p->value_type = CLOUD_VALUE_STRING;
    strlcpy(p->value.string_value, v ? v : "", sizeof(p->value.string_value));
}

/* ---------------- helpers ---------------- */
bool thingscloud_is_enabled(void)
{
    runtime_config_t rt;
    runtime_config_get(&rt);
    return rt.mqtt.platform_type == MQTT_PLATFORM_THINGSCLOUD;
}

void thingscloud_mask_credential(const char *src, char *out, size_t out_size)
{
    if (out == NULL || out_size == 0) return;
    if (src == NULL || src[0] == '\0' || strlen(src) <= 8) {
        strlcpy(out, src ? src : "", out_size);
        return;
    }
    size_t len = strlen(src);
    /* first4****last4 */
    int n = snprintf(out, out_size, "%.4s****%.4s", src, src + len - 4);
    (void)n;
}

static void add_property_to_object(cJSON *obj, const cloud_property_update_t *p)
{
    switch (p->value_type) {
    case CLOUD_VALUE_BOOL:
        cJSON_AddBoolToObject(obj, p->property_key, p->value.boolean_value);
        break;
    case CLOUD_VALUE_INTEGER:
        cJSON_AddNumberToObject(obj, p->property_key, (double)p->value.integer_value);
        break;
    case CLOUD_VALUE_NUMBER:
        cJSON_AddNumberToObject(obj, p->property_key, p->value.number_value);
        break;
    case CLOUD_VALUE_STRING:
        cJSON_AddStringToObject(obj, p->property_key, p->value.string_value);
        break;
    default:
        break;
    }
}

static esp_err_t publish_json_object(cJSON *root, const char *topic)
{
    char *json = cJSON_PrintUnformatted(root);
    if (json == NULL) return ESP_ERR_NO_MEM;
    size_t length = strlen(json);
    if (length > THINGSCLOUD_MAX_PAYLOAD_BYTES) {
        free(json);
        return ESP_ERR_INVALID_SIZE;
    }
    esp_err_t err = tc_publish_telemetry(topic, json);
    free(json);
    return err;
}

/* Build { device: { key: value, ... }, ... } and publish complete JSON
 * packets bounded by THINGSCLOUD_MAX_PAYLOAD_BYTES. */
static esp_err_t publish_subdevice_aggregated(const cloud_property_update_t *props, int count)
{
    if (count <= 0) return ESP_OK;
    if (!mqtt_is_connected()) return ESP_ERR_INVALID_STATE;

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) return ESP_ERR_NO_MEM;
    esp_err_t first_err = ESP_OK;
    int in_pack = 0;
    for (int i = 0; i < count; ++i) {
        const cloud_property_update_t *p = &props[i];
        cJSON *dev = cJSON_GetObjectItem(root, p->device_address);
        if (dev == NULL) {
            dev = cJSON_CreateObject();
            if (dev == NULL) { cJSON_Delete(root); return ESP_ERR_NO_MEM; }
            cJSON_AddItemToObject(root, p->device_address, dev);
        }
        add_property_to_object(dev, p);
        in_pack++;

        char *probe = cJSON_PrintUnformatted(root);
        if (probe == NULL) {
            first_err = ESP_ERR_NO_MEM;
            break;
        }
        size_t length = strlen(probe);
        free(probe);
        if (length <= THINGSCLOUD_MAX_PAYLOAD_BYTES) continue;

        cJSON_DeleteItemFromObject(dev, p->property_key);
        if (cJSON_GetArraySize(dev) == 0) {
            cJSON_DeleteItemFromObject(root, p->device_address);
        }
        in_pack--;
        if (in_pack > 0) {
            esp_err_t err = publish_json_object(root, TC_TOPIC_GATEWAY_ATTR);
            if (err != ESP_OK && first_err == ESP_OK) first_err = err;
            if (err != ESP_OK) break;
        }

        cJSON_Delete(root);
        root = cJSON_CreateObject();
        if (root == NULL) return ESP_ERR_NO_MEM;
        dev = cJSON_CreateObject();
        if (dev == NULL) {
            cJSON_Delete(root);
            return ESP_ERR_NO_MEM;
        }
        cJSON_AddItemToObject(root, p->device_address, dev);
        add_property_to_object(dev, p);
        in_pack = 1;

        probe = cJSON_PrintUnformatted(root);
        if (probe == NULL) {
            first_err = ESP_ERR_NO_MEM;
            break;
        }
        length = strlen(probe);
        free(probe);
        if (length > THINGSCLOUD_MAX_PAYLOAD_BYTES) {
            first_err = ESP_ERR_INVALID_SIZE;
            break;
        }
    }
    if (first_err == ESP_OK && in_pack > 0) {
        first_err = publish_json_object(root, TC_TOPIC_GATEWAY_ATTR);
    }
    cJSON_Delete(root);
    return first_err;
}

esp_err_t thingscloud_publish_subdevice_attributes(const cloud_property_update_t *props, int count)
{
    if (!thingscloud_is_enabled()) return ESP_ERR_INVALID_STATE;
    return publish_subdevice_aggregated(props, count);
}

/* Publish gateway-mode attributes to the "attributes" topic, splitting into
 * multiple packets whenever the serialized payload would exceed
 * THINGSCLOUD_MAX_PAYLOAD_BYTES. Splits always happen on a property boundary
 * (never mid-JSON) and every packet is a complete JSON object. */
static esp_err_t publish_gw_attr_chunked(const cloud_property_update_t *props, int count)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) return ESP_ERR_NO_MEM;
    esp_err_t first_err = ESP_OK;
    int in_pack = 0;
    for (int i = 0; i < count; ++i) {
        const cloud_property_update_t *p = &props[i];
        if (p->property_key[0] == '\0') continue;   /* skip invalid/empty key */
        add_property_to_object(root, p);
        in_pack++;
        char *probe = cJSON_PrintUnformatted(root);
        if (probe == NULL) { first_err = ESP_ERR_NO_MEM; break; }
        size_t len = strlen(probe);
        free(probe);
        if (len > THINGSCLOUD_MAX_PAYLOAD_BYTES && in_pack > 1) {
            /* Remove the just-added property, flush the current pack, then
               re-add it to a fresh packet. */
            cJSON_DeleteItemFromObject(root, p->property_key);
            in_pack--;
            char *out = cJSON_PrintUnformatted(root);
            if (out != NULL) {
                esp_err_t e = tc_publish_telemetry(TC_TOPIC_ATTR, out);
                if (e != ESP_OK && first_err == ESP_OK) first_err = e;
                free(out);
            }
            cJSON_Delete(root);
            root = cJSON_CreateObject();
            if (root == NULL) { first_err = ESP_ERR_NO_MEM; break; }
            add_property_to_object(root, p);
            in_pack = 1;
        }
    }
    if (root != NULL) {
        if (in_pack > 0) {
            char *out = cJSON_PrintUnformatted(root);
            if (out != NULL) {
                esp_err_t e = tc_publish_telemetry(TC_TOPIC_ATTR, out);
                if (e != ESP_OK && first_err == ESP_OK) first_err = e;
                free(out);
            }
        }
        cJSON_Delete(root);
    }
    return first_err;
}

esp_err_t thingscloud_publish_gateway_attributes(const cloud_property_update_t *props, int count)
{
    if (!thingscloud_is_enabled()) return ESP_ERR_INVALID_STATE;
    if (count <= 0 || !mqtt_is_connected()) return ESP_ERR_INVALID_STATE;
    return publish_gw_attr_chunked(props, count);
}

esp_err_t thingscloud_report_subdevice_online(const char *device_address)
{
    if (!thingscloud_is_enabled() || device_address == NULL) return ESP_ERR_INVALID_STATE;
    if (!mqtt_is_connected()) return ESP_ERR_INVALID_STATE;
    char payload[64];
    int n = snprintf(payload, sizeof(payload), "{\"device\":\"%s\"}", device_address);
    if (n < 0 || (size_t)n >= sizeof(payload)) return ESP_ERR_INVALID_SIZE;
    return mqtt_publish(TC_TOPIC_GATEWAY_CONNECT, payload, 0);
}

esp_err_t thingscloud_report_subdevice_offline(const char *device_address)
{
    if (!thingscloud_is_enabled() || device_address == NULL) return ESP_ERR_INVALID_STATE;
    if (!mqtt_is_connected()) return ESP_ERR_INVALID_STATE;
    char payload[64];
    int n = snprintf(payload, sizeof(payload), "{\"device\":\"%s\"}", device_address);
    if (n < 0 || (size_t)n >= sizeof(payload)) return ESP_ERR_INVALID_SIZE;
    return mqtt_publish(TC_TOPIC_GATEWAY_DISCONNECT, payload, 0);
}

esp_err_t thingscloud_publish_gateway_status(void)
{
    if (!thingscloud_is_enabled()) return ESP_ERR_INVALID_STATE;
    if (!mqtt_is_connected()) return ESP_ERR_INVALID_STATE;

    int rssi = 0;
    if (esp_wifi_sta_get_rssi(&rssi) != ESP_OK) rssi = 0;
    network_status_t network = {0};
    network_manager_get_status(&network);

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) return ESP_ERR_NO_MEM;
    cJSON_AddStringToObject(root, "firmware_version", TC_FIRMWARE_VERSION);
    cJSON_AddNumberToObject(root, "free_heap", (double)esp_get_free_heap_size());
    cJSON_AddNumberToObject(root, "uptime", (double)(esp_timer_get_time() / 1000000));
    cJSON_AddStringToObject(root, "network_type",
                            network.active_uplink[0] ? network.active_uplink : "offline");
    cJSON_AddNumberToObject(root, "wifi_rssi", (double)rssi);
    cJSON_AddNumberToObject(root, "rs485_online_count", (double)thingscloud_subdev_online_count());
    cJSON_AddNumberToObject(root, "rs485_error_count", (double)thingscloud_subdev_error_count());
    cJSON_AddNumberToObject(root, "cache_count", (double)uif_get_cached_count());
    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (json == NULL) return ESP_ERR_NO_MEM;
    esp_err_t err = tc_publish_telemetry(TC_TOPIC_ATTR, json);
    ESP_LOGD(TAG, "Gateway status -> %s (%u bytes)", TC_TOPIC_ATTR, (unsigned)strlen(json));
    free(json);
    return err;
}

/* Gateway mode: report every tracked slave's online/status attributes plus the
 * RS485 bus summary, all as the gateway's own attributes. Business values are
 * reported separately; this carries only communication status + summary. */
esp_err_t thingscloud_publish_gateway_slave_status(void)
{
    if (!thingscloud_is_enabled()) return ESP_ERR_INVALID_STATE;
    runtime_config_t rt;
    runtime_config_get(&rt);
    if (rt.mqtt.report_mode != MQ_REPORT_GATEWAY) return ESP_OK;  /* gateway mode only */
    if (!mqtt_is_connected()) return ESP_ERR_INVALID_STATE;

    thingscloud_slave_status_t *slaves = heap_caps_calloc(
        THINGSCLOUD_SUBDEVICE_MAX, sizeof(*slaves),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (slaves == NULL) {
        slaves = calloc(THINGSCLOUD_SUBDEVICE_MAX, sizeof(*slaves));
    }
    if (slaves == NULL) return ESP_ERR_NO_MEM;
    int total = thingscloud_subdev_get_status(slaves, THINGSCLOUD_SUBDEVICE_MAX);
    if (total < 0) total = 0;
    if (total > THINGSCLOUD_SUBDEVICE_MAX) total = THINGSCLOUD_SUBDEVICE_MAX;

    static const char *const status_str[] = {"unknown", "online", "offline"};
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        free(slaves);
        return ESP_ERR_NO_MEM;
    }

    int online = 0, offline = 0, unknown = 0;
    uint32_t err_sum = 0;
    for (int i = 0; i < total; ++i) {
        const thingscloud_slave_status_t *s = &slaves[i];
        int st = s->state;
        if (st < 0 || st > 2) st = 0;
        if (st == 1) online++;
        else if (st == 2) offline++;
        else unknown++;
        err_sum += s->error_count;

        char base[32];
        snprintf(base, sizeof(base), "p%u_s%u",
                 (unsigned)(s->port_id + 1), (unsigned)s->slave_id);
        char key[64];
        snprintf(key, sizeof(key), "%s_online", base);
        cJSON_AddBoolToObject(root, key, st == 1);
        snprintf(key, sizeof(key), "%s_comm_status", base);
        cJSON_AddStringToObject(root, key, status_str[st]);
        snprintf(key, sizeof(key), "%s_last_seen", base);
        cJSON_AddNumberToObject(root, key, (double)s->last_seen_ms);
        snprintf(key, sizeof(key), "%s_error_count", base);
        cJSON_AddNumberToObject(root, key, (double)s->error_count);
        snprintf(key, sizeof(key), "%s_data_valid", base);
        cJSON_AddBoolToObject(root, key, s->data_valid);
    }

    const char *bus;
    if (total == 0) bus = "empty";
    else if (online == 0) bus = "offline";
    else if (offline > 0 || unknown > 0) bus = "degraded";
    else bus = "normal";
    cJSON_AddNumberToObject(root, "rs485_device_total", (double)total);
    cJSON_AddNumberToObject(root, "rs485_online_count", (double)online);
    cJSON_AddNumberToObject(root, "rs485_offline_count", (double)offline);
    cJSON_AddNumberToObject(root, "rs485_unknown_count", (double)unknown);
    cJSON_AddNumberToObject(root, "rs485_error_count", (double)err_sum);
    cJSON_AddStringToObject(root, "rs485_bus_status", bus);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    free(slaves);
    if (json == NULL) return ESP_ERR_NO_MEM;
    esp_err_t err = tc_publish_telemetry(TC_TOPIC_ATTR, json);
    ESP_LOGD(TAG, "Slave status -> %s (%d slaves, %u bytes)",
             TC_TOPIC_ATTR, total, (unsigned)strlen(json));
    free(json);
    return err;
}

/* ---------------- ingestion / flush ---------------- */
/* Fill a cloud property's value fields from a TCM context.
 * Returns false (value dropped) when the sample is non-finite. */
static bool fill_property_value(const tcm_context_t *ctx, cloud_property_update_t *p)
{
    memset(p, 0, sizeof(*p));
    p->timestamp_ms = (int64_t)(esp_timer_get_time() / 1000);
    p->quality = (uint8_t)ctx->quality_state;

    switch (ctx->data_type) {
    case DT_BOOL:
        p->value_type = CLOUD_VALUE_BOOL;
        p->value.boolean_value = (ctx->value != 0.0);
        break;
    case DT_INT16:
    case DT_UINT16:
    case DT_INT32:
    case DT_UINT32:
    case DT_BCD16:
        if (!isfinite(ctx->value)) return false;
        /*
         * Modbus storage type and engineering-value type are different
         * concerns. An integer register may produce a decimal engineering
         * value after TCM scaling (for example 398 * 0.1 = 39.8 degC).
         * Preserve that fraction in the cloud payload.
         */
        if (fabs(ctx->value - nearbyint(ctx->value)) > 1e-9) {
            p->value_type = CLOUD_VALUE_NUMBER;
            p->value.number_value = ctx->value;
        } else {
            p->value_type = CLOUD_VALUE_INTEGER;
            p->value.integer_value = (int64_t)ctx->value;
        }
        break;
    case DT_FLOAT32:
    case DT_FLOAT64:
        if (!isfinite(ctx->value)) return false;   /* drop NaN/Inf */
        p->value_type = CLOUD_VALUE_NUMBER;
        p->value.number_value = ctx->value;
        break;
    case DT_ASCII:
        p->value_type = CLOUD_VALUE_STRING;
        snprintf(p->value.string_value, sizeof(p->value.string_value), "%g", ctx->value);
        break;
    default:
        if (!isfinite(ctx->value)) return false;
        p->value_type = CLOUD_VALUE_NUMBER;
        p->value.number_value = ctx->value;
        break;
    }
    return true;
}

static void add_context_to_agg(const tcm_context_t *ctx)
{
    if (!agg_ensure_buffer()) return;
    if (s_agg_count >= THINGSCLOUD_AGG_BUFFER_MAX) return;
    cloud_property_update_t *p = &s_agg[s_agg_count];
    if (!fill_property_value(ctx, p)) return;
    thingscloud_format_device_address((uint8_t)ctx->channel_id, ctx->slave_id,
                                      p->device_address, sizeof(p->device_address));
    strlcpy(p->property_key, ctx->point_id, sizeof(p->property_key));
    s_agg_count++;
}

static bool agg_ensure_gw_buffer(void)
{
    if (s_gw_agg != NULL) return true;
    s_gw_agg = heap_caps_calloc(THINGSCLOUD_AGG_BUFFER_MAX,
                                sizeof(cloud_property_update_t),
                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_gw_agg == NULL) {
        s_gw_agg = heap_caps_calloc(THINGSCLOUD_AGG_BUFFER_MAX,
                                    sizeof(cloud_property_update_t),
                                    MALLOC_CAP_8BIT);
    }
    if (s_gw_agg == NULL) {
        ESP_LOGE(TAG, "Gateway aggregation buffer allocation failed (%u bytes)",
                 (unsigned)(THINGSCLOUD_AGG_BUFFER_MAX * sizeof(cloud_property_update_t)));
        return false;
    }
    return true;
}

/* Gateway-mode aggregation: property key = the mapping's custom
 * gateway_property_key when set, otherwise the auto-generated
 * "p{port}_s{slave}_{point}". Published later as the gateway's own attributes.
 * Points whose key is empty or invalid are skipped (and logged). */
static void add_context_to_gw_agg(const tcm_context_t *ctx)
{
    if (!agg_ensure_gw_buffer()) return;
    if (s_gw_count >= THINGSCLOUD_AGG_BUFFER_MAX) return;
    cloud_property_update_t *p = &s_gw_agg[s_gw_count];
    if (!fill_property_value(ctx, p)) return;

    char key[64];
    if (ctx->gateway_property_key[0] != '\0') {
        if (!thingscloud_gateway_key_valid(ctx->gateway_property_key)) {
            ESP_LOGE(TAG, "Invalid custom gateway key '%s'; point skipped",
                     ctx->gateway_property_key);
            return;
        }
        strlcpy(key, ctx->gateway_property_key, sizeof(key));
    } else if (build_gateway_property_key((uint8_t)(ctx->channel_id + 1), ctx->slave_id,
                                          ctx->point_id,
                                          key, sizeof(key)) != ESP_OK) {
        ESP_LOGE(TAG, "Gateway key build failed for point '%s'; skipped",
                 ctx->point_id);
        return;
    }
    strlcpy(p->property_key, key, sizeof(p->property_key));
    s_gw_count++;
}

esp_err_t thingscloud_publish_context(const tcm_context_t *ctx)
{
    if (!thingscloud_is_enabled() || ctx == NULL) return ESP_ERR_INVALID_STATE;
    /* Only good-quality samples are reported. */
    if (ctx->quality_state != QUALITY_GOOD) return ESP_OK;
    if (ctx->point_id[0] == '\0') return ESP_OK;

    runtime_config_t rt;
    runtime_config_get(&rt);
    if (!adapter_lock()) return ESP_ERR_TIMEOUT;
    check_mode_change(rt.mqtt.report_mode);
    bool full = rt.mqtt.report_mode == MQ_REPORT_GATEWAY
        ? s_gw_count >= THINGSCLOUD_AGG_BUFFER_MAX
        : s_agg_count >= THINGSCLOUD_AGG_BUFFER_MAX;
    adapter_unlock();
    if (full) thingscloud_flush();

    if (!adapter_lock()) return ESP_ERR_TIMEOUT;
    check_mode_change(rt.mqtt.report_mode);
    full = rt.mqtt.report_mode == MQ_REPORT_GATEWAY
        ? s_gw_count >= THINGSCLOUD_AGG_BUFFER_MAX
        : s_agg_count >= THINGSCLOUD_AGG_BUFFER_MAX;
    if (full) {
        s_dropped_count++;
        adapter_unlock();
        return ESP_ERR_NO_MEM;
    }
    bool should_flush = false;
    if (rt.mqtt.report_mode == MQ_REPORT_GATEWAY) {
        add_context_to_gw_agg(ctx);
        should_flush = s_gw_count >= THINGSCLOUD_AGG_BUFFER_MAX;
    } else {
        add_context_to_agg(ctx);
        should_flush = s_agg_count >= THINGSCLOUD_AGG_BUFFER_MAX;
    }
    adapter_unlock();
    if (should_flush) thingscloud_flush();
    return ESP_OK;
}

void thingscloud_flush(void)
{
    runtime_config_t rt;
    runtime_config_get(&rt);
    if (!adapter_lock()) return;
    check_mode_change(rt.mqtt.report_mode);
    if (rt.mqtt.report_mode == MQ_REPORT_GATEWAY) {
        if (s_gw_agg != NULL && s_gw_count > 0) {
            /* Keep the buffer if throttled so the data is retried later. */
            if (thingscloud_publish_gateway_attributes(s_gw_agg, s_gw_count) == ESP_OK)
                s_gw_count = 0;
        }
    } else {
        if (s_agg == NULL || s_agg_count == 0) {
            adapter_unlock();
            return;
        }
        /* Keep the buffer if throttled so the data is retried later. */
        if (publish_subdevice_aggregated(s_agg, s_agg_count) == ESP_OK)
            s_agg_count = 0;
    }
    adapter_unlock();
}

esp_err_t thingscloud_replay_context(const tcm_context_t *ctx)
{
    if (!thingscloud_is_enabled() || ctx == NULL || !mqtt_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (ctx->quality_state != QUALITY_GOOD || ctx->point_id[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    cloud_property_update_t prop = {0};
    if (!fill_property_value(ctx, &prop)) return ESP_ERR_INVALID_ARG;

    runtime_config_t rt;
    runtime_config_get(&rt);
    if (rt.mqtt.report_mode == MQ_REPORT_GATEWAY) {
        if (ctx->gateway_property_key[0] != '\0') {
            if (!thingscloud_gateway_key_valid(ctx->gateway_property_key)) {
                return ESP_ERR_INVALID_ARG;
            }
            strlcpy(prop.property_key, ctx->gateway_property_key,
                    sizeof(prop.property_key));
        } else {
            esp_err_t err = build_gateway_property_key(
                (uint8_t)(ctx->channel_id + 1), ctx->slave_id, ctx->point_id,
                prop.property_key, sizeof(prop.property_key));
            if (err != ESP_OK) return err;
        }
        return thingscloud_publish_gateway_attributes(&prop, 1);
    }

    if (thingscloud_format_device_address((uint8_t)ctx->channel_id, ctx->slave_id,
                                          prop.device_address,
                                          sizeof(prop.device_address)) < 0) {
        return ESP_ERR_INVALID_SIZE;
    }
    strlcpy(prop.property_key, ctx->point_id, sizeof(prop.property_key));
    return thingscloud_publish_subdevice_attributes(&prop, 1);
}

void thingscloud_get_runtime_status(thingscloud_runtime_status_t *out)
{
    if (out == NULL) return;
    memset(out, 0, sizeof(*out));
    if (!adapter_lock()) return;
    out->pending_points = (uint32_t)(s_agg_count + s_gw_count);
    out->throttled_count = s_throttled_count;
    out->dropped_count = s_dropped_count;
    out->last_publish_ms = s_last_publish_ms;
    adapter_unlock();
}

void thingscloud_on_mqtt_connected(void)
{
    if (!thingscloud_is_enabled()) return;
    runtime_config_t rt;
    runtime_config_get(&rt);
    if (adapter_lock()) {
        check_mode_change(rt.mqtt.report_mode);
        adapter_unlock();
    }
    if (rt.mqtt.report_mode == MQ_REPORT_SUBDEVICE) {
        /* Re-report every ONLINE sub-device, then resume gateway/attributes. */
        thingscloud_subdev_republish_all_online();
        thingscloud_flush();
    } else {
        /* Gateway mode: push all slave states + RS485 summary first, then the
           latest buffered business data. No gateway/connect is published. */
        thingscloud_publish_gateway_slave_status();
        thingscloud_flush();
    }
    thingscloud_publish_gateway_status();
}
