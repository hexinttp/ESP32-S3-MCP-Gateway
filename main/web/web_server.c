/**
 * @file web_server.c
 * @brief HTTP server implementation for ESP32-S3 Gateway web configuration.
 *
 * Uses esp_http_server to serve a configuration web UI and REST API.
 * The HTML page is embedded at build time via CMake FILE_EMBED.
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <time.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_flash.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"
#include "mbedtls/sha256.h"

#include "web_server.h"
#include "gateway_config.h"
#include "amm/amm_mapping.h"
#include "tcm/tcm_context.h"
#include "tcm/tcm_state_pool.h"
#include "modbus/modbus_comm_log.h"
#include "modbus/modbus_tcp_server.h"
#include "mqtt_comm/mqtt_handler.h"
#include "eval/eval_logger.h"
#include "automation/automation_web.h"
#include "mcp/mcp_http.h"
#include "config/runtime_config.h"
#include "cloud_adapter/cloud_adapter.h"
#include "uif/uif_persistence.h"
#include "board/tf_storage.h"
#include "network/network_manager.h"
#include "services/health_service.h"
#include "services/time_service.h"
#include "services/ota_service.h"

static const char *TAG = "WEB";

static httpd_handle_t s_server = NULL;

static void sha256_hex(const char *text, char output[65])
{
    uint8_t digest[32];
    mbedtls_sha256((const unsigned char *)text, strlen(text), digest, 0);
    for (int i = 0; i < 32; ++i) {
        snprintf(output + i * 2, 3, "%02x", digest[i]);
    }
    output[64] = '\0';
}

static const char *data_type_name(data_type_t type)
{
    static const char *names[] = {
        "INT16", "UINT16", "FLOAT32", "INT32", "UINT32", "BOOL",
        "INT64", "UINT64", "FLOAT64", "BCD16", "BITFIELD16", "ASCII"
    };
    return type <= DT_ASCII ? names[type] : "UINT16";
}

static data_type_t parse_data_type_name(const char *name)
{
    if (name == NULL) return DT_UINT16;
    for (int i = DT_INT16; i <= DT_ASCII; ++i) {
        if (strcmp(name, data_type_name((data_type_t)i)) == 0) {
            return (data_type_t)i;
        }
    }
    return DT_UINT16;
}

static esp_err_t require_authorization(httpd_req_t *req)
{
    runtime_config_t config;
    runtime_config_get(&config);
    if (!config.security.auth_enabled) return ESP_OK;

    char authorization[192] = {0};
    if (httpd_req_get_hdr_value_str(req, "Authorization", authorization,
                                    sizeof(authorization)) != ESP_OK ||
        strncmp(authorization, "Bearer ", 7) != 0) {
        httpd_resp_set_status(req, "401 Unauthorized");
        httpd_resp_set_hdr(req, "WWW-Authenticate", "Bearer");
        httpd_resp_send(req, "Unauthorized", HTTPD_RESP_USE_STRLEN);
        return ESP_ERR_NOT_ALLOWED;
    }
    char digest[65];
    sha256_hex(authorization + 7, digest);
    if (strcmp(digest, config.security.password_sha256) != 0) {
        httpd_resp_set_status(req, "401 Unauthorized");
        httpd_resp_set_hdr(req, "WWW-Authenticate", "Bearer");
        httpd_resp_send(req, "Unauthorized", HTTPD_RESP_USE_STRLEN);
        return ESP_ERR_NOT_ALLOWED;
    }
    return ESP_OK;
}

static void httpd_resp_send_400(httpd_req_t *req)
{
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad Request");
}

static esp_err_t httpd_req_get_url_str(httpd_req_t *req, char *buffer, size_t size)
{
    if (req == NULL || buffer == NULL || size == 0) return ESP_ERR_INVALID_ARG;
    strlcpy(buffer, req->uri, size);
    return ESP_OK;
}

/* ---- Embedded HTML page (linked via CMake EMBED_FILES) ---- */
extern const uint8_t web_config_html_gz_start[] asm("_binary_web_config_html_gz_start");
extern const uint8_t web_config_html_gz_end[]   asm("_binary_web_config_html_gz_end");

/* ================================================================
 * Helper: send JSON response
 * ================================================================ */
static esp_err_t send_json(httpd_req_t *req, const char *json_str)
{
    if (json_str == NULL) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "JSON serialization failed");
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, json_str, strlen(json_str));
}

static esp_err_t send_ok(httpd_req_t *req)
{
    return send_json(req, "{\"status\":\"ok\"}");
}

static esp_err_t send_json_string_chunk(httpd_req_t *req, const char *value)
{
    char chunk[160];
    size_t used = 0;
    chunk[used++] = '"';

    const unsigned char *cursor = (const unsigned char *)(value ? value : "");
    while (*cursor != '\0') {
        char escaped[7];
        size_t escaped_len = 0;
        switch (*cursor) {
            case '"':  escaped[0] = '\\'; escaped[1] = '"';  escaped_len = 2; break;
            case '\\': escaped[0] = '\\'; escaped[1] = '\\'; escaped_len = 2; break;
            case '\b': escaped[0] = '\\'; escaped[1] = 'b';  escaped_len = 2; break;
            case '\f': escaped[0] = '\\'; escaped[1] = 'f';  escaped_len = 2; break;
            case '\n': escaped[0] = '\\'; escaped[1] = 'n';  escaped_len = 2; break;
            case '\r': escaped[0] = '\\'; escaped[1] = 'r';  escaped_len = 2; break;
            case '\t': escaped[0] = '\\'; escaped[1] = 't';  escaped_len = 2; break;
            default:
                if (*cursor < 0x20) {
                    snprintf(escaped, sizeof(escaped), "\\u%04x", *cursor);
                    escaped_len = 6;
                } else {
                    escaped[0] = (char)*cursor;
                    escaped_len = 1;
                }
                break;
        }

        if (used + escaped_len + 1 >= sizeof(chunk)) {
            esp_err_t err = httpd_resp_send_chunk(req, chunk, used);
            if (err != ESP_OK) return err;
            used = 0;
        }
        memcpy(chunk + used, escaped, escaped_len);
        used += escaped_len;
        ++cursor;
    }

    chunk[used++] = '"';
    return httpd_resp_send_chunk(req, chunk, used);
}

/* ================================================================
 * Helper: read request body as JSON
 * ================================================================ */
static cJSON *parse_request_json(httpd_req_t *req)
{
    int total_len = req->content_len;
    if (total_len <= 0 || total_len > 4096) return NULL;

    char *buf = (char *)malloc(total_len + 1);
    if (!buf) return NULL;

    int received = 0;
    while (received < total_len) {
        int ret = httpd_req_recv(req, buf + received, total_len - received);
        if (ret <= 0) { free(buf); return NULL; }
        received += ret;
    }
    buf[total_len] = '\0';

    cJSON *root = cJSON_Parse(buf);
    free(buf);
    return root;
}

static cJSON *parse_large_request_json(httpd_req_t *req, size_t max_length)
{
    int total_len = req->content_len;
    if (total_len <= 0 || (size_t)total_len > max_length) return NULL;

    char *buf = malloc((size_t)total_len + 1);
    if (buf == NULL) return NULL;

    int received = 0;
    while (received < total_len) {
        int ret = httpd_req_recv(req, buf + received, total_len - received);
        if (ret <= 0) {
            free(buf);
            return NULL;
        }
        received += ret;
    }
    buf[total_len] = '\0';
    cJSON *root = cJSON_Parse(buf);
    free(buf);
    return root;
}

/* ================================================================
 * GET / -Serve HTML page
 * ================================================================ */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    size_t html_len = (size_t)(web_config_html_gz_end - web_config_html_gz_start);
    char loader[2048];
    int loader_len = snprintf(
        loader, sizeof(loader),
        "<!doctype html><html><head><meta charset=\"utf-8\">"
        "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
        "<link rel=\"icon\" href=\"data:,\">"
        "<title>ESP32-S3 Gateway</title>"
        "<style>body{margin:0;display:grid;place-items:center;min-height:100vh;"
        "font-family:Arial,sans-serif;background:#f2f5f8;color:#102033}"
        ".box{text-align:center}.bar{width:240px;height:5px;background:#d9e2ec;"
        "overflow:hidden;margin:18px auto}.fill{height:100%%;width:0;background:#1473e6}"
        ".error{color:#b42318;max-width:520px}</style></head>"
        "<body><div class=\"box\"><strong>ESP32-S3 Gateway</strong>"
        "<div class=\"bar\"><div class=\"fill\" id=\"p\"></div></div>"
        "<div id=\"s\">Loading configuration...</div></div><script>"
        "(async()=>{try{const total=%u,size=4096,parts=[];"
        "if(!('DecompressionStream'in window))throw new Error('gzip unsupported');"
        "for(let offset=0;offset<total;offset+=size){"
        "const r=await fetch('/ui.bin?offset='+offset,{cache:'no-store'});"
        "if(!r.ok)throw new Error('segment '+offset+' failed');"
        "parts.push(await r.arrayBuffer());"
        "p.style.width=Math.min(100,Math.round((offset+size)*100/total))+'%%';}"
        "const stream=new Blob(parts).stream().pipeThrough(new DecompressionStream('gzip'));"
        "const html=await new Response(stream).text();"
        "document.open();document.write(html);document.close();"
        "}catch(e){s.className='error';s.textContent='Page load failed / 页面加载失败: '+e.message;}})();"
        "</script></body></html>",
        (unsigned)html_len);
    if (loader_len <= 0 || (size_t)loader_len >= sizeof(loader)) {
        return ESP_ERR_HTTPD_RESP_HDR;
    }

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    esp_err_t err = httpd_resp_send(req, loader, loader_len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Web UI loader transfer failed: %s", esp_err_to_name(err));
    }
    return err;
}

/* GET /ui.bin - Serve bounded pieces to stay below the TCP send window. */
static esp_err_t ui_asset_get_handler(httpd_req_t *req)
{
    char query[48] = {0};
    char offset_text[16] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "offset", offset_text, sizeof(offset_text)) != ESP_OK) {
        httpd_resp_send_400(req);
        return ESP_OK;
    }

    char *end = NULL;
    unsigned long requested = strtoul(offset_text, &end, 10);
    size_t html_len = (size_t)(web_config_html_gz_end - web_config_html_gz_start);
    if (end == offset_text || *end != '\0' || requested >= html_len ||
        requested % 4096 != 0) {
        httpd_resp_send_400(req);
        return ESP_OK;
    }

    size_t offset = (size_t)requested;
    size_t part_len = html_len - offset;
    if (part_len > 4096) part_len = 4096;

    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(
        req, (const char *)web_config_html_gz_start + offset, part_len);
}

/* ================================================================
 * GET /api/system/status
 * ================================================================ */
static esp_err_t system_status_get_handler(httpd_req_t *req)
{
    eval_metrics_t m = eval_get_metrics();
    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "mqtt_connected", mqtt_is_connected());
    cJSON_AddBoolToObject(root, "modbus_active", true);
    cJSON_AddNumberToObject(root, "total_polls", m.total_polls);
    cJSON_AddNumberToObject(root, "successful_polls", m.successful_polls);
    cJSON_AddNumberToObject(root, "failed_polls", m.failed_polls);
    cJSON_AddNumberToObject(root, "contexts_created", m.contexts_created);
    cJSON_AddNumberToObject(root, "contexts_validated", m.contexts_validated);
    cJSON_AddNumberToObject(root, "contexts_rejected", m.contexts_rejected);
    cJSON_AddNumberToObject(root, "mqtt_published", m.mqtt_published);
    cJSON_AddNumberToObject(root, "mqtt_failed", m.mqtt_failed);
    cJSON_AddNumberToObject(root, "cached_records", m.cached_records);
    cJSON_AddNumberToObject(root, "replayed_records", m.replayed_records);
    cJSON_AddNumberToObject(root, "data_loss", m.data_loss_count);
    cJSON_AddNumberToObject(root, "commands_received", m.commands_received);
    cJSON_AddNumberToObject(root, "commands_accepted", m.commands_accepted);
    cJSON_AddNumberToObject(root, "commands_rejected", m.commands_rejected);
    cJSON_AddNumberToObject(root, "free_heap", esp_get_free_heap_size());
    cJSON_AddNumberToObject(root, "flash_size", flash_size);
    cJSON_AddNumberToObject(root, "cache_usage_percent", uif_get_cache_usage_percent());
    cJSON_AddBoolToObject(root, "tf_mounted", tf_storage_is_mounted());
    cJSON_AddNumberToObject(root, "amm_model_version", amm_get_model_version());
    cJSON_AddBoolToObject(root, "amm_rollback_available", amm_can_rollback());
    cJSON_AddNumberToObject(root, "mapping_count", amm_get_mapping_count());
    cJSON_AddNumberToObject(root, "mapping_capacity", amm_get_capacity());
    cJSON_AddNumberToObject(root, "state_capacity", tcm_state_pool_get_capacity());
    cJSON_AddNumberToObject(root, "uptime_seconds",
                            (int64_t)(esp_timer_get_time() / 1000000LL));
    cJSON_AddNumberToObject(root, "sequence_counter", tcm_get_sequence_counter());
    gateway_health_t health;
    health_service_get(&health);
    cJSON_AddNumberToObject(root, "boot_count", health.boot_count);
    cJSON_AddNumberToObject(root, "reset_reason", health.reset_reason);
    cJSON_AddNumberToObject(root, "minimum_free_heap", health.minimum_free_heap);
    cJSON_AddNumberToObject(root, "free_internal_heap", health.free_internal_heap);
    cJSON_AddNumberToObject(root, "largest_internal_block",
                           health.largest_internal_block);
    cJSON_AddNumberToObject(root, "free_dma_heap", health.free_dma_heap);
    cJSON_AddNumberToObject(root, "free_psram", health.free_psram);
    cJSON_AddNumberToObject(root, "largest_psram_block",
                           health.largest_psram_block);
    cJSON_AddBoolToObject(root, "time_synchronized", health.time_synchronized);
    cJSON_AddBoolToObject(root, "ota_capable", health.ota_capable);
    cJSON_AddBoolToObject(root, "secure_boot_enabled", health.secure_boot_enabled);
    cJSON_AddBoolToObject(root, "flash_encryption_enabled",
                          health.flash_encryption_enabled);
    cJSON_AddNumberToObject(root, "watchdog_resets", health.watchdog_resets);
    cJSON_AddNumberToObject(root, "online_devices",
                            thingscloud_subdev_online_count());
    cJSON_AddNumberToObject(root, "offline_devices",
                            thingscloud_subdev_offline_count());

    char *json = cJSON_PrintUnformatted(root);
    send_json(req, json);
    free(json);
    cJSON_Delete(root);
    return ESP_OK;
}

/* ================================================================
 * GET /api/wifi/config
 * ================================================================ */
static esp_err_t wifi_config_get_handler(httpd_req_t *req)
{
    runtime_config_t config;
    runtime_config_get(&config);
    network_status_t network;
    network_manager_get_status(&network);
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "enabled", config.wifi.enabled);
    cJSON_AddStringToObject(root, "ssid", config.wifi.ssid);
    cJSON_AddStringToObject(root, "password", config.wifi.password[0] ? "********" : "");
    cJSON_AddNumberToObject(root, "auth_mode", 3);
    cJSON_AddStringToObject(root, "ip", network.wifi_connected ? network.wifi_address : "");
    cJSON_AddStringToObject(root, "config_ap_ssid", network.config_ap_ssid);
    cJSON_AddBoolToObject(root, "prefer_ethernet", config.prefer_ethernet);
    cJSON_AddStringToObject(root, "active_uplink", network.active_uplink);
    cJSON_AddNumberToObject(root, "failover_count", network.failover_count);
    cJSON_AddStringToObject(root, "ethernet_ip", network.ethernet_address);

    char *json = cJSON_PrintUnformatted(root);
    send_json(req, json);
    free(json);
    cJSON_Delete(root);
    return ESP_OK;
}

/* ================================================================
 * PUT /api/wifi/config
 * ================================================================ */
static esp_err_t wifi_config_put_handler(httpd_req_t *req)
{
    if (require_authorization(req) != ESP_OK) return ESP_OK;
    cJSON *root = parse_request_json(req);
    if (!root) {
        httpd_resp_send_400(req);
        return ESP_FAIL;
    }

    runtime_config_t config;
    runtime_config_get(&config);
    cJSON *value = cJSON_GetObjectItem(root, "enabled");
    if (cJSON_IsBool(value)) config.wifi.enabled = cJSON_IsTrue(value);
    value = cJSON_GetObjectItem(root, "ssid");
    if (cJSON_IsString(value)) strlcpy(config.wifi.ssid, value->valuestring, sizeof(config.wifi.ssid));
    value = cJSON_GetObjectItem(root, "password");
    if (cJSON_IsString(value) && strcmp(value->valuestring, "********") != 0)
        strlcpy(config.wifi.password, value->valuestring, sizeof(config.wifi.password));
    value = cJSON_GetObjectItem(root, "prefer_ethernet");
    if (cJSON_IsBool(value)) config.prefer_ethernet = cJSON_IsTrue(value);
    esp_err_t err = runtime_config_set(&config);

    cJSON_Delete(root);
    return err == ESP_OK ? send_json(req, "{\"status\":\"ok\",\"restart_required\":true}")
                         : httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Save failed");
}

/* Run mqtt_restart() in the background so saving config never blocks the
   HTTP response: esp_mqtt_client_stop() may wait on an unreachable broker
   (e.g. a hung DNS lookup), which would otherwise stall the request. */
static volatile bool s_mqtt_restart_pending = false;
static void mqtt_restart_task(void *arg)
{
    (void)arg;
    mqtt_restart();
    s_mqtt_restart_pending = false;
    vTaskDelete(NULL);
}

/* ================================================================
 * GET /api/mqtt/config
 * ================================================================ */
static esp_err_t mqtt_config_get_handler(httpd_req_t *req)
{
    runtime_config_t config;
    runtime_config_get(&config);
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "enabled", config.mqtt.enabled);
    cJSON_AddStringToObject(root, "platform_type",
        (config.mqtt.platform_type == MQTT_PLATFORM_THINGSCLOUD) ? "thingscloud" : "custom");
    cJSON_AddStringToObject(root, "uri", config.mqtt.uri);
    cJSON_AddStringToObject(root, "client_id", config.mqtt.client_id);
    /* Never return full credentials; expose only configured flags + masked. */
    cJSON_AddBoolToObject(root, "username_configured", config.mqtt.username[0] != '\0');
    cJSON_AddBoolToObject(root, "password_configured", config.mqtt.password[0] != '\0');
    {
        char umask[32], pmask[32];
        thingscloud_mask_credential(config.mqtt.username, umask, sizeof(umask));
        thingscloud_mask_credential(config.mqtt.password, pmask, sizeof(pmask));
        cJSON_AddStringToObject(root, "username_masked", umask);
        cJSON_AddStringToObject(root, "password_masked", pmask);
    }
    cJSON_AddNumberToObject(root, "keepalive", config.mqtt.keepalive_sec);
    cJSON_AddNumberToObject(root, "qos", config.mqtt.qos);
    cJSON_AddStringToObject(root, "topic_prefix", config.mqtt.data_prefix);
    cJSON_AddStringToObject(root, "command_prefix", config.mqtt.command_prefix);
    cJSON_AddBoolToObject(root, "clean_session", config.mqtt.clean_session);
    cJSON_AddBoolToObject(root, "retain", config.mqtt.retain);
    cJSON_AddBoolToObject(root, "lwt_enabled", config.mqtt.lwt_enabled);
    cJSON_AddStringToObject(root, "lwt_topic", config.mqtt.lwt_topic);
    cJSON_AddStringToObject(root, "lwt_payload", config.mqtt.lwt_payload);
    cJSON_AddNumberToObject(root, "lwt_qos", config.mqtt.lwt_qos);
    cJSON_AddBoolToObject(root, "lwt_retain", config.mqtt.lwt_retain);
    cJSON_AddStringToObject(root, "report_mode",
        (config.mqtt.report_mode == MQ_REPORT_GATEWAY) ? "gateway" : "subdevice");
    cJSON_AddNumberToObject(root, "config_generation",
                            (double)thingscloud_get_config_generation());
    thingscloud_runtime_status_t tc_status;
    thingscloud_get_runtime_status(&tc_status);
    cJSON_AddNumberToObject(root, "cloud_pending_points",
                            (double)tc_status.pending_points);
    cJSON_AddNumberToObject(root, "cloud_throttled_count",
                            (double)tc_status.throttled_count);
    cJSON_AddNumberToObject(root, "cloud_dropped_count",
                            (double)tc_status.dropped_count);
    cJSON_AddNumberToObject(root, "cloud_last_publish_ms",
                            (double)tc_status.last_publish_ms);
    cJSON_AddNumberToObject(root, "cloud_cached_records",
                            (double)uif_get_cached_count());
    cJSON_AddNumberToObject(root, "cloud_online_devices",
                            (double)thingscloud_subdev_online_count());

    char *json = cJSON_PrintUnformatted(root);
    send_json(req, json);
    free(json);
    cJSON_Delete(root);
    return ESP_OK;
}

/* ================================================================
 * PUT /api/mqtt/config
 * ================================================================ */
static esp_err_t mqtt_config_put_handler(httpd_req_t *req)
{
    if (require_authorization(req) != ESP_OK) return ESP_OK;
    cJSON *root = parse_request_json(req);
    if (!root) {
        httpd_resp_send_400(req);
        return ESP_FAIL;
    }

    runtime_config_t config;
    runtime_config_get(&config);
    mqtt_report_mode_t previous_report_mode = config.mqtt.report_mode;
#define COPY_MQTT_STRING(key, field) do { cJSON *item = cJSON_GetObjectItem(root, key); \
    if (cJSON_IsString(item) && strcmp(item->valuestring, "********") != 0) { \
        char _t[sizeof(config.mqtt.field)]; \
        strlcpy(_t, item->valuestring, sizeof(_t)); \
        /* Trim leading/trailing whitespace so a stray space in client_id/uri   \
         * does not get rejected by the broker. */ \
        char *_s = _t, *_e = _t + strlen(_t); \
        while (*_s == ' ' || *_s == '\t') _s++; \
        while (_e > _s && (_e[-1] == ' ' || _e[-1] == '\t')) _e--; \
        *_e = '\0'; \
        strlcpy(config.mqtt.field, _s, sizeof(config.mqtt.field)); \
    } } while (0)
    COPY_MQTT_STRING("uri", uri);
    COPY_MQTT_STRING("client_id", client_id);
    COPY_MQTT_STRING("topic_prefix", data_prefix);
    COPY_MQTT_STRING("command_prefix", command_prefix);
    COPY_MQTT_STRING("lwt_topic", lwt_topic);
    COPY_MQTT_STRING("lwt_payload", lwt_payload);
#undef COPY_MQTT_STRING

    /* platform_type */
    cJSON *plat = cJSON_GetObjectItem(root, "platform_type");
    if (cJSON_IsString(plat)) {
        if (strcmp(plat->valuestring, "thingscloud") == 0)
            config.mqtt.platform_type = MQTT_PLATFORM_THINGSCLOUD;
        else
            config.mqtt.platform_type = MQTT_PLATFORM_CUSTOM;
    }

    /* report_mode: sub-device vs gateway aggregation (ThingsCloud only) */
    cJSON *rmode = cJSON_GetObjectItem(root, "report_mode");
    if (cJSON_IsString(rmode)) {
        if (strcmp(rmode->valuestring, "gateway") == 0)
            config.mqtt.report_mode = MQ_REPORT_GATEWAY;
        else
            config.mqtt.report_mode = MQ_REPORT_SUBDEVICE;
    }

    /* username / password handling:
       - empty string  -> keep existing value (do NOT clear)
       - "********"     -> keep existing value (mask placeholder echoed back)
       - clear_username / clear_password = true -> explicitly clear */
    bool clear_user = false, clear_pass = false;
    cJSON *cv = cJSON_GetObjectItem(root, "clear_username");
    if (cJSON_IsBool(cv)) clear_user = cJSON_IsTrue(cv);
    cv = cJSON_GetObjectItem(root, "clear_password");
    if (cJSON_IsBool(cv)) clear_pass = cJSON_IsTrue(cv);

    cJSON *ui = cJSON_GetObjectItem(root, "username");
    if (cJSON_IsString(ui)) {
        if (clear_user) config.mqtt.username[0] = '\0';
        else if (ui->valuestring[0] != '\0' && strcmp(ui->valuestring, "********") != 0)
            strlcpy(config.mqtt.username, ui->valuestring, sizeof(config.mqtt.username));
    }
    cJSON *pi = cJSON_GetObjectItem(root, "password");
    if (cJSON_IsString(pi)) {
        if (clear_pass) config.mqtt.password[0] = '\0';
        else if (pi->valuestring[0] != '\0' && strcmp(pi->valuestring, "********") != 0)
            strlcpy(config.mqtt.password, pi->valuestring, sizeof(config.mqtt.password));
    }

    cJSON *value = cJSON_GetObjectItem(root, "enabled");
    if (cJSON_IsBool(value)) config.mqtt.enabled = cJSON_IsTrue(value);
    value = cJSON_GetObjectItem(root, "keepalive");
    if (cJSON_IsNumber(value)) config.mqtt.keepalive_sec = value->valueint;
    value = cJSON_GetObjectItem(root, "qos");
    if (cJSON_IsNumber(value) && value->valueint >= 0 && value->valueint <= 2) config.mqtt.qos = value->valueint;
    value = cJSON_GetObjectItem(root, "clean_session");
    if (cJSON_IsBool(value)) config.mqtt.clean_session = cJSON_IsTrue(value);
    value = cJSON_GetObjectItem(root, "retain");
    if (cJSON_IsBool(value)) config.mqtt.retain = cJSON_IsTrue(value);
    value = cJSON_GetObjectItem(root, "lwt_enabled");
    if (cJSON_IsBool(value)) config.mqtt.lwt_enabled = cJSON_IsTrue(value);
    value = cJSON_GetObjectItem(root, "lwt_qos");
    if (cJSON_IsNumber(value) && value->valueint >= 0 && value->valueint <= 2)
        config.mqtt.lwt_qos = value->valueint;
    value = cJSON_GetObjectItem(root, "lwt_retain");
    if (cJSON_IsBool(value)) config.mqtt.lwt_retain = cJSON_IsTrue(value);

    /* ThingsCloud mandates QoS 0, no retain, and disables the custom LWT. */
    if (config.mqtt.platform_type == MQTT_PLATFORM_THINGSCLOUD) {
        config.mqtt.qos = 0;
        config.mqtt.retain = false;
        config.mqtt.lwt_enabled = false;
        config.mqtt.lwt_retain = false;
    }

    bool report_mode_changed = previous_report_mode != config.mqtt.report_mode;
    if (report_mode_changed) {
        /* Stop the old data route before committing the new ownership model. */
        mqtt_disconnect();
    }
    esp_err_t err = runtime_config_set(&config);
    if (err != ESP_OK) {
        cJSON_Delete(root);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Save failed");
    }

    /* Re-establish the broker connection with the new settings, but in a
       background task so this HTTP request returns immediately (stopping a
       client that was dialing an unreachable broker can take seconds). */
    if (!s_mqtt_restart_pending) {
        s_mqtt_restart_pending = true;
        BaseType_t created = xTaskCreate(mqtt_restart_task, "mqtt_restart",
                                         6144, NULL, 6, NULL);
        if (created != pdPASS) {
            s_mqtt_restart_pending = false;
            ESP_LOGE(TAG, "Unable to create MQTT restart task");
            cJSON_Delete(root);
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                       "MQTT restart unavailable: low internal memory");
        }
    }

    cJSON_Delete(root);
    return send_json(req, "{\"status\":\"ok\",\"restart_required\":false}");
}

/* ================================================================
 * POST /api/mqtt/test
 *  Actually probe the broker URI (with credentials) using a
 *  throwaway MQTT client, so the Web UI "Test Connection" button
 *  reports the REAL result instead of always claiming success.
 * ================================================================ */
static esp_err_t mqtt_test_post_handler(httpd_req_t *req)
{
    if (require_authorization(req) != ESP_OK) return ESP_OK;
    cJSON *root = parse_request_json(req);
    if (!root) {
        httpd_resp_send_400(req);
        return ESP_FAIL;
    }

    char uri[128] = {0};
    char username[64] = {0};
    char password[96] = {0};
    int timeout_ms = MQTT_PUBLISH_TIMEOUT_MS;

    cJSON *item = cJSON_GetObjectItem(root, "uri");
    if (!cJSON_IsString(item) || item->valuestring[0] == '\0') {
        cJSON_Delete(root);
        cJSON *resp = cJSON_CreateObject();
        cJSON_AddBoolToObject(resp, "success", false);
        cJSON_AddStringToObject(resp, "error", "Missing broker URI");
        char *json = cJSON_PrintUnformatted(resp);
        send_json(req, json);
        free(json);
        cJSON_Delete(resp);
        return ESP_OK;
    }
    strlcpy(uri, item->valuestring, sizeof(uri));

    item = cJSON_GetObjectItem(root, "username");
    if (cJSON_IsString(item)) strlcpy(username, item->valuestring, sizeof(username));
    item = cJSON_GetObjectItem(root, "password");
    if (cJSON_IsString(item) && strcmp(item->valuestring, "********") != 0)
        strlcpy(password, item->valuestring, sizeof(password));
    item = cJSON_GetObjectItem(root, "timeout_ms");
    if (cJSON_IsNumber(item) && item->valueint > 0) timeout_ms = item->valueint;

    /* Quick gate: if the board itself has no internet uplink (no Ethernet IP
       and WiFi STA not connected to a router), a public broker can never be
       reached - report that directly instead of a generic failure. */
    if (!network_manager_is_online()) {
        cJSON *resp = cJSON_CreateObject();
        cJSON_AddBoolToObject(resp, "success", false);
        cJSON_AddStringToObject(resp, "error",
            "board has no internet connection (no Ethernet IP and WiFi STA not connected to a router)");
        char *json = cJSON_PrintUnformatted(resp);
        send_json(req, json);
        free(json);
        cJSON_Delete(resp);
        cJSON_Delete(root);
        return ESP_OK;
    }

    /* Configuration reads intentionally mask secrets. Reuse the stored
       credentials when the UI tests the active URI without retyping them. */
    runtime_config_t saved;
    runtime_config_get(&saved);
    if ((username[0] == '\0' || strstr(username, "****") != NULL) &&
        strcmp(uri, saved.mqtt.uri) == 0) {
        strlcpy(username, saved.mqtt.username, sizeof(username));
    }
    if ((password[0] == '\0' || strstr(password, "****") != NULL) &&
        strcmp(uri, saved.mqtt.uri) == 0) {
        strlcpy(password, saved.mqtt.password, sizeof(password));
    }

    char reason[160] = {0};
    esp_err_t res = mqtt_test_connection(uri,
                                         username[0] ? username : NULL,
                                         password[0] ? password : NULL,
                                         timeout_ms, reason, sizeof(reason));

    cJSON *resp = cJSON_CreateObject();
    cJSON_AddBoolToObject(resp, "success", res == ESP_OK);
    if (res == ESP_OK) {
        cJSON_AddStringToObject(resp, "message", reason[0] ? reason : "Connected to broker");
    } else {
        cJSON_AddStringToObject(resp, "error",
            reason[0] ? reason :
            (res == ESP_ERR_INVALID_ARG ? "Invalid broker URI" :
             res == ESP_ERR_TIMEOUT ? "Connection timeout (broker unreachable)" :
             "Connection failed (auth or network error)"));
    }
    char *json = cJSON_PrintUnformatted(resp);
    send_json(req, json);
    free(json);
    cJSON_Delete(resp);
    cJSON_Delete(root);
    return ESP_OK;
}

/* ================================================================
 * POST /api/mqtt/disconnect
 *  Drop the current broker connection on demand. The client is stopped
 *  (no auto-reconnect) but the configuration and subscription table are
 *  preserved, so saving the settings again (or a reboot) will reconnect.
 * ================================================================ */
static esp_err_t mqtt_disconnect_post_handler(httpd_req_t *req)
{
    if (require_authorization(req) != ESP_OK) return ESP_OK;

    esp_err_t err = mqtt_disconnect();
    cJSON *resp = cJSON_CreateObject();
    if (err == ESP_OK) {
        cJSON_AddBoolToObject(resp, "success", true);
        cJSON_AddStringToObject(resp, "message", "MQTT connection disconnected");
    } else {
        cJSON_AddBoolToObject(resp, "success", false);
        cJSON_AddStringToObject(resp, "error",
            err == ESP_ERR_INVALID_STATE ? "MQTT client is not connected" : "Failed to disconnect");
    }
    char *json = cJSON_PrintUnformatted(resp);
    send_json(req, json);
    free(json);
    cJSON_Delete(resp);
    return ESP_OK;
}

/* ================================================================
 * GET /api/modbus/config
 * ================================================================ */
static esp_err_t modbus_config_get_handler(httpd_req_t *req)
{
    runtime_config_t config;
    runtime_config_get(&config);
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "enabled", config.modbus_rtu.enabled);
    cJSON_AddNumberToObject(root, "uart_port", MODBUS_RTU_UART_PORT);
    cJSON_AddNumberToObject(root, "baudrate", config.modbus_rtu.baud_rate);
    cJSON_AddNumberToObject(root, "parity", config.modbus_rtu.parity);
    cJSON_AddNumberToObject(root, "data_bits", 8);
    cJSON_AddNumberToObject(root, "stop_bits", 1);
    cJSON_AddNumberToObject(root, "timeout", config.modbus_rtu.timeout_ms);
    cJSON_AddNumberToObject(root, "tx_pin", MODBUS_RTU_UART_TXD);
    cJSON_AddNumberToObject(root, "rx_pin", MODBUS_RTU_UART_RXD);
    cJSON_AddNumberToObject(root, "rts_pin", MODBUS_RTU_UART_RTS);
    cJSON_AddNumberToObject(root, "poll_interval",
                           amm_get_default_poll_interval());
    modbus_tcp_server_status_t server_status = {0};
    modbus_tcp_server_get_status(&server_status);
    cJSON *server = cJSON_AddObjectToObject(root, "tcp_server");
    cJSON_AddBoolToObject(server, "enabled", config.modbus_tcp_server.enabled);
    cJSON_AddBoolToObject(server, "running", server_status.running);
    cJSON_AddNumberToObject(server, "port", config.modbus_tcp_server.port);
    cJSON_AddNumberToObject(server, "max_clients",
                            config.modbus_tcp_server.max_clients);
    cJSON_AddNumberToObject(server, "active_clients",
                            server_status.active_clients);
    cJSON_AddNumberToObject(server, "requests", server_status.requests);
    cJSON_AddNumberToObject(server, "exceptions", server_status.exceptions);
    cJSON *endpoints = cJSON_AddArrayToObject(root, "tcp_endpoints");
    for (int i = 0; i < config.tcp_endpoint_count; ++i) {
        cJSON *endpoint = cJSON_CreateObject();
        cJSON_AddBoolToObject(endpoint, "enabled", config.tcp_endpoints[i].enabled);
        cJSON_AddNumberToObject(endpoint, "endpoint_id", config.tcp_endpoints[i].endpoint_id);
        cJSON_AddStringToObject(endpoint, "name", config.tcp_endpoints[i].name);
        cJSON_AddStringToObject(endpoint, "host", config.tcp_endpoints[i].host);
        cJSON_AddNumberToObject(endpoint, "port", config.tcp_endpoints[i].port);
        cJSON_AddNumberToObject(endpoint, "timeout", config.tcp_endpoints[i].timeout_ms);
        cJSON_AddItemToArray(endpoints, endpoint);
    }

    char *json = cJSON_PrintUnformatted(root);
    send_json(req, json);
    free(json);
    cJSON_Delete(root);
    return ESP_OK;
}

/* ================================================================
 * PUT /api/modbus/config
 * ================================================================ */
static esp_err_t modbus_config_put_handler(httpd_req_t *req)
{
    if (require_authorization(req) != ESP_OK) return ESP_OK;
    cJSON *root = parse_request_json(req);
    if (!root) {
        httpd_resp_send_400(req);
        return ESP_FAIL;
    }

    runtime_config_t config;
    runtime_config_get(&config);
    cJSON *value = cJSON_GetObjectItem(root, "enabled");
    if (cJSON_IsBool(value)) config.modbus_rtu.enabled = cJSON_IsTrue(value);
    value = cJSON_GetObjectItem(root, "baudrate");
    if (cJSON_IsNumber(value)) config.modbus_rtu.baud_rate = (uint32_t)value->valuedouble;
    value = cJSON_GetObjectItem(root, "parity");
    if (cJSON_IsNumber(value)) config.modbus_rtu.parity = value->valueint;
    value = cJSON_GetObjectItem(root, "timeout");
    if (cJSON_IsNumber(value)) config.modbus_rtu.timeout_ms = value->valueint;
    value = cJSON_GetObjectItem(root, "poll_interval");
    uint32_t poll_interval = amm_get_default_poll_interval();
    if (cJSON_IsNumber(value)) {
        poll_interval = (uint32_t)value->valuedouble;
        if (poll_interval < 100 || poll_interval > 3600000) {
            cJSON_Delete(root);
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "Poll interval must be 100..3600000 ms");
        }
    }
    cJSON *server = cJSON_GetObjectItem(root, "tcp_server");
    if (cJSON_IsObject(server)) {
        value = cJSON_GetObjectItem(server, "enabled");
        if (cJSON_IsBool(value)) {
            config.modbus_tcp_server.enabled = cJSON_IsTrue(value);
        }
        value = cJSON_GetObjectItem(server, "port");
        if (cJSON_IsNumber(value)) {
            config.modbus_tcp_server.port = (uint16_t)value->valueint;
        }
        value = cJSON_GetObjectItem(server, "max_clients");
        if (cJSON_IsNumber(value)) {
            config.modbus_tcp_server.max_clients = (uint8_t)value->valueint;
        }
    }
    cJSON *endpoints = cJSON_GetObjectItem(root, "tcp_endpoints");
    if (cJSON_IsArray(endpoints)) {
        int count = cJSON_GetArraySize(endpoints);
        if (count > RUNTIME_MAX_TCP_ENDPOINTS) count = RUNTIME_MAX_TCP_ENDPOINTS;
        memset(config.tcp_endpoints, 0, sizeof(config.tcp_endpoints));
        config.tcp_endpoint_count = count;
        for (int i = 0; i < count; ++i) {
            cJSON *endpoint = cJSON_GetArrayItem(endpoints, i);
            runtime_modbus_tcp_endpoint_t *target = &config.tcp_endpoints[i];
            cJSON *item = cJSON_GetObjectItem(endpoint, "enabled");
            target->enabled = cJSON_IsTrue(item);
            item = cJSON_GetObjectItem(endpoint, "endpoint_id"); target->endpoint_id = cJSON_IsNumber(item) ? item->valueint : i + 1;
            item = cJSON_GetObjectItem(endpoint, "name"); if (cJSON_IsString(item)) strlcpy(target->name, item->valuestring, sizeof(target->name));
            item = cJSON_GetObjectItem(endpoint, "host"); if (cJSON_IsString(item)) strlcpy(target->host, item->valuestring, sizeof(target->host));
            item = cJSON_GetObjectItem(endpoint, "port"); target->port = cJSON_IsNumber(item) ? item->valueint : 502;
            item = cJSON_GetObjectItem(endpoint, "timeout"); target->timeout_ms = cJSON_IsNumber(item) ? item->valueint : 1000;
        }
    }
    esp_err_t err = runtime_config_set(&config);
    if (err == ESP_OK) {
        err = amm_set_poll_interval_all(poll_interval);
    }
    cJSON_Delete(root);
    return err == ESP_OK ? send_json(req, "{\"status\":\"ok\",\"restart_required\":true}")
                         : httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Save failed");
}

/* Compute the effective gateway-mode property key for a mapping entry: the
 * custom gateway_property_key when set, else the auto-generated
 * "p{port}_s{slave}_{point}" form. */
static void mapping_effective_gw_key(const amm_mapping_entry_t *e, char *out, size_t out_size)
{
    if (out == NULL || out_size == 0) return;
    out[0] = '\0';
    if (e == NULL) return;
    if (e->gateway_property_key[0] != '\0') {
        strlcpy(out, e->gateway_property_key, out_size);
    } else {
        build_gateway_property_key((uint8_t)(e->channel_id + 1), e->slave_id,
                                   e->point_id, out, out_size);
    }
}

/* Returns true when `key` collides with another mapping's effective gateway
 * key, ignoring the same physical point (protocol/channel/slave/register). */
static bool gateway_key_conflicts(const amm_mapping_entry_t *new_entry, const char *key)
{
    if (new_entry == NULL || key == NULL || key[0] == '\0') return false;
    int n = amm_get_mapping_count();
    for (int i = 0; i < n; ++i) {
        amm_mapping_entry_t e;
        if (amm_get_entry_at(i, &e) != ESP_OK) continue;
        if (e.source_protocol == new_entry->source_protocol &&
            e.channel_id == new_entry->channel_id &&
            e.slave_id == new_entry->slave_id &&
            e.register_address == new_entry->register_address) {
            continue;   /* same physical point (overwrite in place) */
        }
        char other[64];
        mapping_effective_gw_key(&e, other, sizeof(other));
        if (other[0] != '\0' && strcmp(other, key) == 0) return true;
    }
    return false;
}

/* ================================================================
 * GET /api/mappings -List all mapping entries
 * ================================================================ */
static esp_err_t mappings_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t err = httpd_resp_send_chunk(req, "[", 1);
    int active_count = amm_get_mapping_count();
    bool first = true;

    for (int i = 0; i < active_count && err == ESP_OK; i++) {
        amm_mapping_entry_t entry;
        if (amm_get_entry_at(i, &entry) != ESP_OK) continue;
        amm_mapping_entry_t *e = &entry;

        cJSON *obj = cJSON_CreateObject();
        if (obj == NULL) {
            err = ESP_ERR_NO_MEM;
            break;
        }
        cJSON_AddStringToObject(obj, "source_protocol", e->source_protocol == SRC_MODBUS_TCP ? "TCP" : "RTU");
        cJSON_AddNumberToObject(obj, "channel_id", e->channel_id);
        cJSON_AddNumberToObject(obj, "slave_id", e->slave_id);
        cJSON_AddNumberToObject(obj, "register_address", e->register_address);
        cJSON_AddStringToObject(obj, "data_type", data_type_name(e->data_type));
        cJSON_AddNumberToObject(obj, "scale_factor", e->scale_factor);
        cJSON_AddNumberToObject(obj, "offset", e->offset);
        cJSON_AddNumberToObject(obj, "function_code", e->function_code);
        cJSON_AddNumberToObject(obj, "byte_order", e->byte_order);
        cJSON_AddNumberToObject(obj, "read_start_address", e->read_start_address);
        cJSON_AddNumberToObject(obj, "read_register_count", e->read_register_count);
        cJSON_AddNumberToObject(obj, "value_register_index", e->value_register_index);
        cJSON_AddNumberToObject(obj, "poll_interval_ms", e->poll_interval_ms);
        cJSON_AddNumberToObject(obj, "priority", e->priority);
        cJSON_AddNumberToObject(obj, "mapping_version", e->mapping_version);
        cJSON_AddNumberToObject(obj, "object_type", e->object_type);
        cJSON_AddNumberToObject(obj, "bit_index", e->bit_index);
        cJSON_AddNumberToObject(obj, "string_length", e->string_length);
        cJSON_AddNumberToObject(obj, "retry_count", e->retry_count);
        cJSON_AddNumberToObject(obj, "retry_backoff_ms", e->retry_backoff_ms);
        cJSON_AddNumberToObject(obj, "semantic_source", e->semantic_source);
        cJSON_AddNumberToObject(obj, "semantic_status", e->semantic_status);
        cJSON_AddStringToObject(obj, "semantic_profile_id",
                                e->semantic_profile_id);
        cJSON_AddNumberToObject(obj, "semantic_profile_version",
                                e->semantic_profile_version);
        cJSON_AddNumberToObject(obj, "semantic_confidence",
                                e->semantic_confidence);
        cJSON_AddStringToObject(obj, "semantic_evidence",
                                e->semantic_evidence);
        cJSON_AddStringToObject(obj, "device_id", e->device_id);
        cJSON_AddStringToObject(obj, "point_id", e->point_id);
        cJSON_AddStringToObject(obj, "measurement_name", e->measurement_name);
        cJSON_AddStringToObject(obj, "unit", e->unit);
        cJSON_AddStringToObject(obj, "mqtt_topic", e->mqtt_topic);
        cJSON_AddStringToObject(obj, "mqtt_topic_mode",
                                e->mqtt_topic_mode == MQTT_TOPIC_CUSTOM
                                    ? "custom" : "auto");
        char effective_topic[AMM_MAX_TOPIC_LEN] = {0};
        if (amm_resolve_mqtt_topic(e, effective_topic,
                                   sizeof(effective_topic)) == ESP_OK) {
            cJSON_AddStringToObject(obj, "effective_mqtt_topic",
                                    effective_topic);
        } else {
            cJSON_AddStringToObject(obj, "effective_mqtt_topic", "");
        }
        cJSON_AddStringToObject(obj, "gateway_property_key", e->gateway_property_key);
        char effective_gateway_key[64] = {0};
        mapping_effective_gw_key(e, effective_gateway_key,
                                 sizeof(effective_gateway_key));
        cJSON_AddStringToObject(obj, "effective_gateway_property_key",
                                effective_gateway_key);
        cJSON_AddBoolToObject(obj, "writable", e->constraint.writable);
        cJSON_AddNumberToObject(obj, "range_min", e->constraint.valid_range_min);
        cJSON_AddNumberToObject(obj, "range_max", e->constraint.valid_range_max);
        tcm_context_t state;
        if (tcm_state_pool_get(e->device_id, e->point_id, &state) == ESP_OK) {
            cJSON_AddNumberToObject(obj, "current_timestamp_ms",
                                    (double)state.timestamp_ms);
            cJSON_AddNumberToObject(obj, "quality_state", state.quality_state);
            if (state.quality_state == QUALITY_GOOD ||
                state.quality_state == QUALITY_STALE) {
                cJSON_AddNumberToObject(obj, "current_raw_value", state.raw_value);
                cJSON_AddNumberToObject(obj, "current_value", state.value);
                cJSON_AddStringToObject(obj, "current_value_text",
                                        state.value_text);
            }
            cJSON_AddStringToObject(obj, "poll_status",
                                    state.quality_state == QUALITY_GOOD ? "ok" :
                                    state.quality_state == QUALITY_STALE ? "stale" :
                                    "error");
        } else {
            cJSON_AddStringToObject(obj, "poll_status", "pending");
        }
        char *json = cJSON_PrintUnformatted(obj);
        cJSON_Delete(obj);
        if (json == NULL) {
            err = ESP_ERR_NO_MEM;
            break;
        }
        if (!first) {
            err = httpd_resp_send_chunk(req, ",", 1);
        }
        if (err == ESP_OK) {
            err = httpd_resp_send_chunk(req, json, strlen(json));
        }
        free(json);
        first = false;
    }

    if (err == ESP_OK) {
        err = httpd_resp_send_chunk(req, "]", 1);
    }
    httpd_resp_send_chunk(req, NULL, 0);
    return err;
}

/* ================================================================
 * POST /api/mappings -Add a new mapping
 * ================================================================ */
static esp_err_t mappings_post_handler(httpd_req_t *req)
{
    if (require_authorization(req) != ESP_OK) return ESP_OK;
    cJSON *root = parse_request_json(req);
    if (!root) {
        httpd_resp_send_400(req);
        return ESP_FAIL;
    }

    amm_mapping_entry_t entry;
    memset(&entry, 0, sizeof(entry));
    entry.source_protocol = SRC_MODBUS_RTU;
    entry.function_code = 3;
    entry.scale_factor = 1.0f;
    entry.poll_interval_ms = POLL_INTERVAL_MS;
    entry.priority = 5;
    entry.retry_count = 2;
    entry.retry_backoff_ms = 50;
    entry.semantic_source = SEMANTIC_SOURCE_USER;
    entry.semantic_status = SEMANTIC_STATUS_VERIFIED;
    entry.semantic_confidence = 100;
    entry.mqtt_topic_mode = MQTT_TOPIC_AUTO;
    strlcpy(entry.semantic_profile_id, "user-confirmed",
            sizeof(entry.semantic_profile_id));
    entry.semantic_profile_version = 1;
    strlcpy(entry.semantic_evidence, "web configuration",
            sizeof(entry.semantic_evidence));

    cJSON *v;
    uint8_t old_slave = 0;
    uint16_t old_register = 0;
    uint8_t old_channel = 0;
    source_protocol_t old_protocol = SRC_MODBUS_RTU;
    if ((v = cJSON_GetObjectItem(root, "old_slave_id")) && cJSON_IsNumber(v)) old_slave = v->valueint;
    if ((v = cJSON_GetObjectItem(root, "old_register_address")) && cJSON_IsNumber(v)) old_register = v->valueint;
    if ((v = cJSON_GetObjectItem(root, "old_channel_id")) && cJSON_IsNumber(v)) old_channel = v->valueint;
    if ((v = cJSON_GetObjectItem(root, "old_source_protocol")) && cJSON_IsString(v))
        old_protocol = strcmp(v->valuestring, "TCP") == 0 ? SRC_MODBUS_TCP : SRC_MODBUS_RTU;
    if ((v = cJSON_GetObjectItem(root, "slave_id")) && cJSON_IsNumber(v))
        entry.slave_id = (uint8_t)v->valuedouble;
    if ((v = cJSON_GetObjectItem(root, "register_address")) && cJSON_IsNumber(v))
        entry.register_address = (uint16_t)v->valuedouble;
    if ((v = cJSON_GetObjectItem(root, "scale_factor")) && cJSON_IsNumber(v))
        entry.scale_factor = (float)v->valuedouble;
    if ((v = cJSON_GetObjectItem(root, "offset")) && cJSON_IsNumber(v))
        entry.offset = (float)v->valuedouble;
    if ((v = cJSON_GetObjectItem(root, "channel_id")) && cJSON_IsNumber(v))
        entry.channel_id = (uint8_t)v->valuedouble;
    if ((v = cJSON_GetObjectItem(root, "function_code")) && cJSON_IsNumber(v))
        entry.function_code = (uint8_t)v->valuedouble;
    entry.object_type = entry.function_code >= 1 && entry.function_code <= 4
        ? (modbus_object_type_t)entry.function_code
        : MODBUS_OBJECT_HOLDING_REGISTER;
    if ((v = cJSON_GetObjectItem(root, "byte_order")) && cJSON_IsNumber(v))
        entry.byte_order = (byte_order_t)v->valueint;
    if ((v = cJSON_GetObjectItem(root, "read_start_address")) && cJSON_IsNumber(v))
        entry.read_start_address = (uint16_t)v->valuedouble;
    if ((v = cJSON_GetObjectItem(root, "read_register_count")) && cJSON_IsNumber(v))
        entry.read_register_count = (uint8_t)v->valuedouble;
    if ((v = cJSON_GetObjectItem(root, "value_register_index")) && cJSON_IsNumber(v))
        entry.value_register_index = (uint8_t)v->valuedouble;
    if ((v = cJSON_GetObjectItem(root, "poll_interval_ms")) && cJSON_IsNumber(v))
        entry.poll_interval_ms = (uint32_t)v->valuedouble;
    if ((v = cJSON_GetObjectItem(root, "priority")) && cJSON_IsNumber(v))
        entry.priority = (uint8_t)v->valuedouble;
    if ((v = cJSON_GetObjectItem(root, "source_protocol")) && cJSON_IsString(v))
        entry.source_protocol = strcmp(v->valuestring, "TCP") == 0 ? SRC_MODBUS_TCP : SRC_MODBUS_RTU;

    /* Data type string to enum */
    if ((v = cJSON_GetObjectItem(root, "data_type")) && cJSON_IsString(v)) {
        entry.data_type = parse_data_type_name(v->valuestring);
    }

    /* String fields */
    if ((v = cJSON_GetObjectItem(root, "device_id")) && cJSON_IsString(v))
        strncpy(entry.device_id, v->valuestring, AMM_MAX_DEVICE_NAME_LEN - 1);
    if ((v = cJSON_GetObjectItem(root, "point_id")) && cJSON_IsString(v))
        strncpy(entry.point_id, v->valuestring, AMM_MAX_POINT_NAME_LEN - 1);
    if ((v = cJSON_GetObjectItem(root, "measurement_name")) && cJSON_IsString(v))
        strncpy(entry.measurement_name, v->valuestring, AMM_MAX_POINT_NAME_LEN - 1);
    if ((v = cJSON_GetObjectItem(root, "unit")) && cJSON_IsString(v))
        strncpy(entry.unit, v->valuestring, AMM_MAX_UNIT_LEN - 1);
    if ((v = cJSON_GetObjectItem(root, "mqtt_topic")) && cJSON_IsString(v))
        strncpy(entry.mqtt_topic, v->valuestring, AMM_MAX_TOPIC_LEN - 1);
    if ((v = cJSON_GetObjectItem(root, "mqtt_topic_mode")) && cJSON_IsString(v))
        entry.mqtt_topic_mode = strcmp(v->valuestring, "custom") == 0
            ? MQTT_TOPIC_CUSTOM : MQTT_TOPIC_AUTO;
    if ((v = cJSON_GetObjectItem(root, "gateway_property_key")) && cJSON_IsString(v))
        strlcpy(entry.gateway_property_key, v->valuestring, sizeof(entry.gateway_property_key));

    /* Constraint */
    if ((v = cJSON_GetObjectItem(root, "writable")) && cJSON_IsBool(v))
        entry.constraint.writable = cJSON_IsTrue(v);
    if ((v = cJSON_GetObjectItem(root, "range_min")) && cJSON_IsNumber(v))
        entry.constraint.valid_range_min = (float)v->valuedouble;
    if ((v = cJSON_GetObjectItem(root, "range_max")) && cJSON_IsNumber(v))
        entry.constraint.valid_range_max = (float)v->valuedouble;

    entry.active = true;

    if (entry.mqtt_topic_mode == MQTT_TOPIC_CUSTOM &&
        entry.mqtt_topic[0] == '\0') {
        cJSON_Delete(root);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req,
            "{\"status\":\"error\",\"reason\":\"custom MQTT topic is required\"}",
            HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    /* Gateway-mode property key checks: a custom key must be well-formed, and
       the effective key (custom or auto-generated) must be unique. Enforced at
       the backend because the frontend is not trusted. */
    if (entry.gateway_property_key[0] != '\0' &&
        !thingscloud_gateway_key_valid(entry.gateway_property_key)) {
        cJSON_Delete(root);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_send(req, "{\"status\":\"error\",\"reason\":\"invalid gateway_property_key\"}",
                        HTTPD_RESP_USE_STRLEN);
        return ESP_FAIL;
    }
    {
        char eff_key[64];
        mapping_effective_gw_key(&entry, eff_key, sizeof(eff_key));
        if (gateway_key_conflicts(&entry, eff_key)) {
            cJSON_Delete(root);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_set_status(req, "409 Conflict");
            httpd_resp_send(req, "{\"status\":\"error\",\"reason\":\"duplicate gateway property key\"}",
                            HTTPD_RESP_USE_STRLEN);
            return ESP_FAIL;
        }
    }

    esp_err_t err = amm_add_mapping(&entry);
    if (err == ESP_OK && old_slave != 0 &&
        (old_slave != entry.slave_id || old_register != entry.register_address ||
         old_channel != entry.channel_id || old_protocol != entry.source_protocol)) {
        (void)amm_remove_mapping_for_channel(old_protocol, old_channel, old_slave, old_register);
    }
    cJSON_Delete(root);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Web API: added mapping %s/%s", entry.device_id, entry.point_id);
        return send_ok(req);
    } else {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"status\":\"error\",\"reason\":\"table full or invalid args\"}",
                        HTTPD_RESP_USE_STRLEN);
        return ESP_FAIL;
    }
}

static bool mapping_entry_from_json(const cJSON *root, amm_mapping_entry_t *entry)
{
    if (!cJSON_IsObject(root) || entry == NULL) return false;

    memset(entry, 0, sizeof(*entry));
    entry->source_protocol = SRC_MODBUS_RTU;
    entry->function_code = 3;
    entry->data_type = DT_UINT16;
    entry->byte_order = BYTE_ORDER_ABCD;
    entry->scale_factor = 1.0f;
    entry->poll_interval_ms = POLL_INTERVAL_MS;
    entry->priority = 5;
    entry->retry_count = 2;
    entry->retry_backoff_ms = 50;
    entry->semantic_source = SEMANTIC_SOURCE_IMPORTED;
    entry->semantic_status = SEMANTIC_STATUS_RESOLVED;
    entry->semantic_confidence = 100;
    strlcpy(entry->semantic_evidence, "mapping profile import",
            sizeof(entry->semantic_evidence));

    const cJSON *value;
    value = cJSON_GetObjectItem(root, "source_protocol");
    if (cJSON_IsString(value) && strcmp(value->valuestring, "TCP") == 0) {
        entry->source_protocol = SRC_MODBUS_TCP;
    }
    value = cJSON_GetObjectItem(root, "channel_id");
    if (cJSON_IsNumber(value)) entry->channel_id = (uint8_t)value->valueint;
    value = cJSON_GetObjectItem(root, "slave_id");
    if (cJSON_IsNumber(value)) entry->slave_id = (uint8_t)value->valueint;
    value = cJSON_GetObjectItem(root, "function_code");
    if (cJSON_IsNumber(value)) entry->function_code = (uint8_t)value->valueint;
    entry->object_type = entry->function_code >= 1 && entry->function_code <= 4
        ? (modbus_object_type_t)entry->function_code
        : MODBUS_OBJECT_HOLDING_REGISTER;
    value = cJSON_GetObjectItem(root, "register_address");
    if (cJSON_IsNumber(value)) entry->register_address = (uint16_t)value->valueint;
    value = cJSON_GetObjectItem(root, "data_type");
    if (cJSON_IsString(value)) {
        entry->data_type = parse_data_type_name(value->valuestring);
    }
    value = cJSON_GetObjectItem(root, "byte_order");
    if (cJSON_IsNumber(value)) entry->byte_order = (byte_order_t)value->valueint;
    value = cJSON_GetObjectItem(root, "scale_factor");
    if (cJSON_IsNumber(value)) entry->scale_factor = (float)value->valuedouble;
    value = cJSON_GetObjectItem(root, "offset");
    if (cJSON_IsNumber(value)) entry->offset = (float)value->valuedouble;
    value = cJSON_GetObjectItem(root, "poll_interval_ms");
    if (cJSON_IsNumber(value)) entry->poll_interval_ms = (uint32_t)value->valuedouble;
    value = cJSON_GetObjectItem(root, "priority");
    if (cJSON_IsNumber(value)) entry->priority = (uint8_t)value->valueint;
    value = cJSON_GetObjectItem(root, "read_start_address");
    if (cJSON_IsNumber(value)) entry->read_start_address = (uint16_t)value->valueint;
    value = cJSON_GetObjectItem(root, "read_register_count");
    if (cJSON_IsNumber(value)) entry->read_register_count = (uint8_t)value->valueint;
    value = cJSON_GetObjectItem(root, "value_register_index");
    if (cJSON_IsNumber(value)) entry->value_register_index = (uint8_t)value->valueint;
    value = cJSON_GetObjectItem(root, "bit_index");
    if (cJSON_IsNumber(value)) entry->bit_index = (uint8_t)value->valueint;
    value = cJSON_GetObjectItem(root, "string_length");
    if (cJSON_IsNumber(value)) entry->string_length = (uint8_t)value->valueint;
    value = cJSON_GetObjectItem(root, "retry_count");
    if (cJSON_IsNumber(value)) entry->retry_count = (uint8_t)value->valueint;
    value = cJSON_GetObjectItem(root, "retry_backoff_ms");
    if (cJSON_IsNumber(value)) entry->retry_backoff_ms = (uint16_t)value->valueint;

    value = cJSON_GetObjectItem(root, "device_id");
    if (cJSON_IsString(value)) strlcpy(entry->device_id, value->valuestring, sizeof(entry->device_id));
    value = cJSON_GetObjectItem(root, "point_id");
    if (cJSON_IsString(value)) strlcpy(entry->point_id, value->valuestring, sizeof(entry->point_id));
    value = cJSON_GetObjectItem(root, "measurement_name");
    if (cJSON_IsString(value)) strlcpy(entry->measurement_name, value->valuestring, sizeof(entry->measurement_name));
    value = cJSON_GetObjectItem(root, "unit");
    if (cJSON_IsString(value)) strlcpy(entry->unit, value->valuestring, sizeof(entry->unit));
    value = cJSON_GetObjectItem(root, "mqtt_topic");
    if (cJSON_IsString(value)) strlcpy(entry->mqtt_topic, value->valuestring, sizeof(entry->mqtt_topic));
    value = cJSON_GetObjectItem(root, "mqtt_topic_mode");
    if (cJSON_IsString(value)) {
        entry->mqtt_topic_mode = strcmp(value->valuestring, "custom") == 0
            ? MQTT_TOPIC_CUSTOM : MQTT_TOPIC_AUTO;
    }
    value = cJSON_GetObjectItem(root, "gateway_property_key");
    if (cJSON_IsString(value)) strlcpy(entry->gateway_property_key, value->valuestring, sizeof(entry->gateway_property_key));
    value = cJSON_GetObjectItem(root, "semantic_profile_id");
    if (cJSON_IsString(value)) {
        strlcpy(entry->semantic_profile_id, value->valuestring,
                sizeof(entry->semantic_profile_id));
    }
    value = cJSON_GetObjectItem(root, "semantic_profile_version");
    if (cJSON_IsNumber(value)) {
        entry->semantic_profile_version = (uint32_t)value->valuedouble;
    }
    value = cJSON_GetObjectItem(root, "semantic_confidence");
    if (cJSON_IsNumber(value)) {
        entry->semantic_confidence = (uint8_t)value->valueint;
    }
    value = cJSON_GetObjectItem(root, "semantic_evidence");
    if (cJSON_IsString(value)) {
        strlcpy(entry->semantic_evidence, value->valuestring,
                sizeof(entry->semantic_evidence));
    }

    value = cJSON_GetObjectItem(root, "writable");
    if (cJSON_IsBool(value)) entry->constraint.writable = cJSON_IsTrue(value);
    value = cJSON_GetObjectItem(root, "range_min");
    if (cJSON_IsNumber(value)) entry->constraint.valid_range_min = (float)value->valuedouble;
    value = cJSON_GetObjectItem(root, "range_max");
    if (cJSON_IsNumber(value)) entry->constraint.valid_range_max = (float)value->valuedouble;
    entry->active = true;

    if (entry->gateway_property_key[0] != '\0' &&
        !thingscloud_gateway_key_valid(entry->gateway_property_key)) return false;
    if (entry->mqtt_topic_mode == MQTT_TOPIC_CUSTOM &&
        entry->mqtt_topic[0] == '\0') return false;

    return entry->slave_id >= 1 &&
           entry->function_code >= 1 && entry->function_code <= 4 &&
           entry->device_id[0] != '\0' && entry->point_id[0] != '\0';
}

/* POST /api/mappings/import - Batch semantic profile import. */
static esp_err_t mappings_import_post_handler(httpd_req_t *req)
{
    if (require_authorization(req) != ESP_OK) return ESP_OK;
    cJSON *root = parse_large_request_json(req, 768U * 1024U);
    if (root == NULL) {
        httpd_resp_set_status(req, "400 Bad Request");
        return send_json(req, "{\"error\":\"Invalid or oversized mapping profile\"}");
    }

    cJSON *items = cJSON_GetObjectItem(root, "mappings");
    cJSON *replace = cJSON_GetObjectItem(root, "replace_devices");
    int count = cJSON_IsArray(items) ? cJSON_GetArraySize(items) : 0;
    if (count <= 0 || count > amm_get_capacity()) {
        cJSON_Delete(root);
        httpd_resp_set_status(req, "400 Bad Request");
        return send_json(req, "{\"error\":\"Mapping count exceeds runtime capacity\"}");
    }

    amm_mapping_entry_t *entries = calloc((size_t)count, sizeof(entries[0]));
    if (entries == NULL) {
        cJSON_Delete(root);
        httpd_resp_set_status(req, "500 Internal Server Error");
        return send_json(req, "{\"error\":\"Out of memory\"}");
    }

    bool valid = true;
    for (int i = 0; i < count; ++i) {
        if (!mapping_entry_from_json(cJSON_GetArrayItem(items, i), &entries[i])) {
            valid = false;
            break;
        }
    }

    int imported = 0;
    esp_err_t err = valid
        ? amm_import_mappings(entries, count, cJSON_IsTrue(replace), &imported)
        : ESP_ERR_INVALID_ARG;
    free(entries);
    cJSON_Delete(root);

    if (err != ESP_OK) {
        httpd_resp_set_status(req, "400 Bad Request");
        char error_json[128];
        snprintf(error_json, sizeof(error_json),
                 "{\"error\":\"Profile import failed: %s\"}",
                 esp_err_to_name(err));
        return send_json(req, error_json);
    }

    char response[128];
    snprintf(response, sizeof(response),
             "{\"status\":\"ok\",\"imported\":%d,\"total_mappings\":%d,\"capacity\":%d}",
             imported, amm_get_mapping_count(), amm_get_capacity());
    return send_json(req, response);
}

/* ================================================================
 * PUT /api/mappings/:idx -Update mapping (re-add with same addr)
 * ================================================================ */
static esp_err_t mappings_put_handler(httpd_req_t *req)
{
    if (require_authorization(req) != ESP_OK) return ESP_OK;
    /* Extract index from URI: /api/mappings/<idx> */
    char uri[64];
    httpd_req_get_url_str(req, uri, sizeof(uri));
    const char *last_slash = strrchr(uri, '/');
    int idx = last_slash ? atoi(last_slash + 1) : -1;
    (void)idx; /* In real implementation, we'd remove old + add new */

    /* For simplicity, treat PUT same as POST (amm_add_mapping overwrites duplicates) */
    return mappings_post_handler(req);
}

/* ================================================================
 * DELETE /api/mappings/:idx
 * ================================================================ */
static esp_err_t mappings_delete_handler(httpd_req_t *req)
{
    if (require_authorization(req) != ESP_OK) return ESP_OK;
    /* Extract slave_id and register_address from query or body */
    char uri[64];
    httpd_req_get_url_str(req, uri, sizeof(uri));

    /* For the simulation, we send slave_id and reg_addr as query params */
    /* Simplified: accept from request body JSON */
    cJSON *root = parse_request_json(req);
    if (root) {
        uint8_t slave = 0;
        uint16_t reg = 0;
        uint8_t channel = 0;
        source_protocol_t protocol = SRC_MODBUS_RTU;
        cJSON *v;
        if ((v = cJSON_GetObjectItem(root, "slave_id")) && cJSON_IsNumber(v))
            slave = (uint8_t)v->valuedouble;
        if ((v = cJSON_GetObjectItem(root, "register_address")) && cJSON_IsNumber(v))
            reg = (uint16_t)v->valuedouble;
        if ((v = cJSON_GetObjectItem(root, "channel_id")) && cJSON_IsNumber(v))
            channel = (uint8_t)v->valuedouble;
        if ((v = cJSON_GetObjectItem(root, "source_protocol")) && cJSON_IsString(v))
            protocol = strcmp(v->valuestring, "TCP") == 0 ? SRC_MODBUS_TCP : SRC_MODBUS_RTU;
        cJSON_Delete(root);

        if (slave > 0) {
            esp_err_t err = amm_remove_mapping_for_channel(protocol, channel, slave, reg);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Web API: removed mapping slave=%u reg=%u", slave, reg);
                return send_ok(req);
            }
        }
    }

    httpd_resp_send_400(req);
    return ESP_FAIL;
}

static esp_err_t mappings_clear_handler(httpd_req_t *req)
{
    if (require_authorization(req) != ESP_OK) return ESP_OK;
    esp_err_t err = amm_clear_mappings();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Web API: clear mappings failed: %s", esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Clear mappings failed");
    }

    tcm_state_pool_clear();
    ESP_LOGI(TAG, "Web API: cleared all mappings and runtime point states");
    return send_json(req, "{\"status\":\"ok\",\"cleared\":true}");
}

static esp_err_t mappings_rollback_post_handler(httpd_req_t *req)
{
    if (require_authorization(req) != ESP_OK) return ESP_OK;
    esp_err_t err = amm_rollback();
    if (err == ESP_OK) return send_ok(req);
    httpd_resp_set_status(req, "409 Conflict");
    return send_json(req, "{\"error\":\"No valid AMM rollback snapshot\"}");
}

/* ================================================================
 * System Log Buffer -ring buffer for recent log entries
 * ================================================================ */
#define WEB_LOG_MAX_ENTRIES   64
#define WEB_LOG_MAX_TEXT_LEN 128

typedef struct {
    char  text[WEB_LOG_MAX_TEXT_LEN];
    char  level; /* 'i'=info, 'w'=warn, 'e'=error, 'o'=ok */
    int64_t timestamp_ms;
    int64_t uptime_ms;
} web_log_entry_t;

static web_log_entry_t s_log_buf[WEB_LOG_MAX_ENTRIES];
static int s_log_head  = 0;   /* next write position */
static int s_log_count = 0;   /* total valid entries */
static SemaphoreHandle_t s_log_mutex = NULL;

void web_server_add_log(const char *level_str, const char *text)
{
    if (!s_log_mutex) return;
    if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;

    web_log_entry_t *e = &s_log_buf[s_log_head];
    e->level = (level_str && *level_str) ? *level_str : 'i';
    snprintf(e->text, WEB_LOG_MAX_TEXT_LEN, "%s", text ? text : "");
    time_t now = time(NULL);
    e->timestamp_ms = now >= 1577836800 ? (int64_t)now * 1000 : 0;
    e->uptime_ms = esp_timer_get_time() / 1000LL;
    s_log_head = (s_log_head + 1) % WEB_LOG_MAX_ENTRIES;
    if (s_log_count < WEB_LOG_MAX_ENTRIES) s_log_count++;

    xSemaphoreGive(s_log_mutex);
}

/* ================================================================
 * GET /api/system/logs -Return recent log entries as JSON array
 * ================================================================ */
static esp_err_t system_logs_get_handler(httpd_req_t *req)
{
    cJSON *arr = cJSON_CreateArray();

    if (s_log_mutex && xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        int start = (s_log_count < WEB_LOG_MAX_ENTRIES) ? 0 : s_log_head;
        for (int i = 0; i < s_log_count; i++) {
            int idx = (start + i) % WEB_LOG_MAX_ENTRIES;
            web_log_entry_t *e = &s_log_buf[idx];

            cJSON *obj = cJSON_CreateObject();
            const char *lvl = "info";
            if (e->level == 'w') lvl = "warn";
            else if (e->level == 'e') lvl = "error";
            else if (e->level == 'o') lvl = "ok";
            cJSON_AddStringToObject(obj, "level", lvl);
            cJSON_AddStringToObject(obj, "text", e->text);
            cJSON_AddNumberToObject(obj, "timestamp_ms",
                                    (double)e->timestamp_ms);
            cJSON_AddNumberToObject(obj, "uptime_ms",
                                    (double)e->uptime_ms);
            cJSON_AddItemToArray(arr, obj);
        }
        xSemaphoreGive(s_log_mutex);
    }

    char *json = cJSON_PrintUnformatted(arr);
    send_json(req, json);
    free(json);
    cJSON_Delete(arr);
    return ESP_OK;
}

static esp_err_t modbus_logs_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t send_err = ESP_OK;
    char output[2048];
    size_t output_used = 0;
    output[output_used++] = '[';
    time_t wall_now = time(NULL);
    int64_t wall_now_ms = wall_now >= 1577836800 ? (int64_t)wall_now * 1000 : 0;
    int64_t boot_now_ms = esp_timer_get_time() / 1000LL;
    int count = modbus_comm_log_count();
    for (int i = 0; i < count; ++i) {
        modbus_comm_log_entry_t entry;
        if (!modbus_comm_log_get(i, &entry)) continue;

        char hex[MODBUS_COMM_FRAME_MAX * 3 + 1];
        size_t used = 0;
        for (uint8_t j = 0; j < entry.frame_length && used + 4 < sizeof(hex); ++j) {
            int written = snprintf(hex + used, sizeof(hex) - used,
                                   j == 0 ? "%02X" : " %02X", entry.frame[j]);
            if (written < 0) break;
            used += (size_t)written;
        }
        hex[used] = '\0';

        char values[MODBUS_COMM_FRAME_MAX * 3 + 1] = "";
        if (entry.direction == MODBUS_COMM_RX && entry.status == ESP_OK &&
            (entry.function_code == 3 || entry.function_code == 4) &&
            entry.frame_length >= 5) {
            uint8_t available = (uint8_t)((entry.frame_length - 5) / 2);
            uint8_t value_count = entry.register_count < available ?
                                  entry.register_count : available;
            size_t value_used = 0;
            for (uint8_t j = 0; j < value_count; ++j) {
                uint16_t value = ((uint16_t)entry.frame[3 + j * 2] << 8) |
                                 entry.frame[4 + j * 2];
                int written = snprintf(values + value_used,
                                       sizeof(values) - value_used,
                                       j == 0 ? "%u" : ",%u", value);
                if (written < 0 || (size_t)written >=
                    sizeof(values) - value_used) break;
                value_used += (size_t)written;
            }
        }
        char chunk[640];
        int length = snprintf(
            chunk, sizeof(chunk),
            "%s{\"sequence\":%lu,\"timestamp_ms\":%" PRId64
            ",\"uptime_ms\":%" PRId64
            ",\"direction\":\"%s\",\"slave_id\":%u,\"function_code\":%u,"
            "\"register_address\":%u,\"register_count\":%u,"
            "\"status_code\":%ld,\"status\":\"%s\",\"truncated\":%s,"
            "\"frame_hex\":\"%s\",\"register_values\":[%s]}",
            i == 0 ? "" : ",", (unsigned long)entry.sequence,
            wall_now_ms > 0 ? wall_now_ms - boot_now_ms + entry.timestamp_ms : 0,
            entry.timestamp_ms,
            entry.direction == MODBUS_COMM_TX ? "TX" : "RX",
            entry.slave_id, entry.function_code, entry.register_address,
            entry.register_count, (long)entry.status,
            esp_err_to_name(entry.status),
            entry.truncated ? "true" : "false", hex, values);
        if (length < 0 || length >= sizeof(chunk)) {
            send_err = ESP_FAIL;
            break;
        }
        if (output_used + (size_t)length > sizeof(output)) {
            send_err = httpd_resp_send_chunk(req, output, output_used);
            if (send_err != ESP_OK) break;
            output_used = 0;
        }
        memcpy(output + output_used, chunk, (size_t)length);
        output_used += (size_t)length;
    }
    if (send_err == ESP_OK) {
        if (output_used == sizeof(output)) {
            send_err = httpd_resp_send_chunk(req, output, output_used);
            output_used = 0;
        }
        if (send_err == ESP_OK) {
            output[output_used++] = ']';
            send_err = httpd_resp_send_chunk(req, output, output_used);
        }
    }
    httpd_resp_send_chunk(req, NULL, 0);
    return send_err;
}

static esp_err_t modbus_logs_delete_handler(httpd_req_t *req)
{
    if (require_authorization(req) != ESP_OK) return ESP_OK;
    modbus_comm_log_clear();
    return send_ok(req);
}

/* ================================================================
 * Discovery API Handlers
 * ================================================================ */

#include "modbus/modbus_discover.h"

/* GET /api/discover/status */
static esp_err_t discover_status_get_handler(httpd_req_t *req)
{
    discover_result_t r = modbus_discover_get_result();

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "total_scanned", r.total_scanned);
    cJSON_AddNumberToObject(root, "slaves_scanned", r.slaves_scanned);
    cJSON_AddNumberToObject(root, "devices_found", r.devices_found);
    cJSON_AddNumberToObject(root, "slaves_skipped", r.slaves_skipped);
    cJSON_AddNumberToObject(root, "registers_found", r.registers_found);
    cJSON_AddNumberToObject(root, "mappings_created", r.mappings_created);
    cJSON_AddNumberToObject(root, "device_capacity", r.device_capacity);
    cJSON_AddNumberToObject(root, "mapping_capacity", amm_get_capacity());
    cJSON_AddNumberToObject(root, "current_slave", r.current_slave);
    cJSON_AddNumberToObject(root, "current_register", r.current_register);
    cJSON_AddNumberToObject(root, "current_function_code", r.current_function_code);
    cJSON_AddNumberToObject(root, "phase", r.phase);
    cJSON_AddNumberToObject(root, "last_error", r.last_error);
    cJSON_AddBoolToObject(root, "scan_complete", r.scan_complete);
    cJSON_AddBoolToObject(root, "scan_in_progress", r.scan_in_progress);

    char *json = cJSON_PrintUnformatted(root);
    send_json(req, json);
    free(json);
    cJSON_Delete(root);
    return ESP_OK;
}

/* GET /api/discover/devices */
static esp_err_t discover_devices_get_handler(httpd_req_t *req)
{
    if (modbus_discover_get_result().scan_in_progress) {
        httpd_resp_set_status(req, "409 Conflict");
        return send_json(req, "{\"error\":\"discovery scan in progress\"}");
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    esp_err_t err = httpd_resp_send_chunk(req, "[", 1);
    bool first_device = true;
    char chunk[512];

    uint16_t count = modbus_discover_get_device_count();
    for (uint16_t i = 0; i < count && err == ESP_OK; i++) {
        const discovered_device_t *dev = modbus_discover_get_device(i);
        if (!dev || !dev->active) continue;

        int length = snprintf(
            chunk, sizeof(chunk),
            "%s{\"slave_id\":%u,\"source_protocol\":\"%s\",\"channel_id\":%u,"
            "\"device_id\":",
            first_device ? "" : ",", dev->slave_id,
            dev->source_protocol == SRC_MODBUS_TCP ? "TCP" : "RTU",
            dev->channel_id);
        if (length < 0 || length >= sizeof(chunk) ||
            (err = httpd_resp_send_chunk(req, chunk, length)) != ESP_OK) break;
        err = send_json_string_chunk(req, dev->device_id);
        if (err != ESP_OK) break;
        err = httpd_resp_send_chunk(req, ",\"name\":", 8);
        if (err != ESP_OK) break;
        err = send_json_string_chunk(req, dev->name[0] ? dev->name : dev->device_id);
        if (err != ESP_OK) break;

        if (dev->description[0]) {
            err = httpd_resp_send_chunk(req, ",\"description\":", 15);
            if (err != ESP_OK) break;
            err = send_json_string_chunk(req, dev->description);
            if (err != ESP_OK) break;
        }
        if (dev->mqtt_topic_prefix[0]) {
            err = httpd_resp_send_chunk(req, ",\"mqtt_topic_prefix\":", 21);
            if (err != ESP_OK) break;
            err = send_json_string_chunk(req, dev->mqtt_topic_prefix);
            if (err != ESP_OK) break;
        }
        if (dev->vendor_name[0]) {
            err = httpd_resp_send_chunk(req, ",\"vendor_name\":", 15);
            if (err != ESP_OK) break;
            err = send_json_string_chunk(req, dev->vendor_name);
            if (err != ESP_OK) break;
        }
        if (dev->product_code[0]) {
            err = httpd_resp_send_chunk(req, ",\"product_code\":", 16);
            if (err != ESP_OK) break;
            err = send_json_string_chunk(req, dev->product_code);
            if (err != ESP_OK) break;
        }
        if (dev->revision[0]) {
            err = httpd_resp_send_chunk(req, ",\"revision\":", 12);
            if (err != ESP_OK) break;
            err = send_json_string_chunk(req, dev->revision);
            if (err != ESP_OK) break;
        }

        length = snprintf(
            chunk, sizeof(chunk),
            ",\"register_count\":%u,\"probe_function_code\":%u,"
            "\"probe_address\":%u,\"probe_register_count\":%u,\"registers\":[",
            dev->reg_count, dev->probe_function_code, dev->probe_address,
            dev->probe_register_count);
        if (length < 0 || length >= sizeof(chunk) ||
            (err = httpd_resp_send_chunk(req, chunk, length)) != ESP_OK) break;

        for (uint16_t j = 0; j < dev->reg_count && err == ESP_OK; j++) {
            const discovered_register_t *reg = &dev->registers[j];
            length = snprintf(
                chunk, sizeof(chunk),
                "%s{\"register_address\":%u,\"function_code\":%u,"
                "\"raw_value\":%u,\"read_start_address\":%u,"
                "\"read_register_count\":%u,\"value_register_index\":%u,"
                "\"data_type\":\"%s\",\"sample_value\":%.9g,\"inferred_name\":",
                j == 0 ? "" : ",", reg->register_address, reg->function_code,
                reg->raw_value, reg->read_start_address,
                reg->read_register_count, reg->value_register_index,
                reg->inferred_type == DT_FLOAT32 ? "FLOAT32" :
                reg->inferred_type == DT_INT16 ? "INT16" : "UINT16",
                (double)reg->sample_value);
            if (length < 0 || length >= sizeof(chunk) ||
                (err = httpd_resp_send_chunk(req, chunk, length)) != ESP_OK) break;
            err = send_json_string_chunk(req, reg->inferred_name);
            if (err != ESP_OK) break;
            err = httpd_resp_send_chunk(req, ",\"inferred_unit\":", 17);
            if (err != ESP_OK) break;
            err = send_json_string_chunk(req, reg->inferred_unit);
            if (err != ESP_OK) break;
            length = snprintf(chunk, sizeof(chunk),
                              ",\"writable\":%s,\"valid\":%s,"
                              "\"range_min\":%.9g,\"range_max\":%.9g,"
                              "\"user_edited\":%s}",
                              reg->writable ? "true" : "false",
                              reg->valid ? "true" : "false",
                              (double)reg->range_min, (double)reg->range_max,
                              reg->user_edited ? "true" : "false");
            if (length < 0 || length >= sizeof(chunk) ||
                (err = httpd_resp_send_chunk(req, chunk, length)) != ESP_OK) break;
        }
        if (err != ESP_OK) break;
        err = httpd_resp_send_chunk(req, "],\"active\":true}", 16);
        first_device = false;
    }

    if (err == ESP_OK) err = httpd_resp_send_chunk(req, "]", 1);
    httpd_resp_send_chunk(req, NULL, 0);
    return err;
}

/* POST /api/discover/scan */
static esp_err_t discover_scan_post_handler(httpd_req_t *req)
{
    if (require_authorization(req) != ESP_OK) return ESP_OK;
    cJSON *body = parse_request_json(req);
    if (body == NULL) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "A valid JSON scan request is required");
    }

    discover_scan_params_t params;
    memset(&params, 0, sizeof(params));
    params.slave_start = 1;
    params.slave_end = 10;
    params.reg_start = 1;
    params.reg_end = 16;
    params.source_protocol = SRC_MODBUS_RTU;
    params.channel_id = 0;
    params.function_codes[0] = 1;
    params.function_codes[1] = 2;
    params.function_codes[2] = 3;
    params.function_codes[3] = 4;
    params.fc_count = 4;
    params.max_empty_gap = DISCOVER_DEFAULT_EMPTY_GAP;

    cJSON *v;
    if ((v = cJSON_GetObjectItem(body, "slave_start")) && cJSON_IsNumber(v))
        params.slave_start = (uint8_t)v->valuedouble;
    if ((v = cJSON_GetObjectItem(body, "slave_end")) && cJSON_IsNumber(v))
        params.slave_end = (uint8_t)v->valuedouble;
    if ((v = cJSON_GetObjectItem(body, "reg_start")) && cJSON_IsNumber(v))
        params.reg_start = (uint16_t)v->valuedouble;
    if ((v = cJSON_GetObjectItem(body, "reg_end")) && cJSON_IsNumber(v))
        params.reg_end = (uint16_t)v->valuedouble;
    if ((v = cJSON_GetObjectItem(body, "max_empty_gap")) && cJSON_IsNumber(v))
        params.max_empty_gap = (uint8_t)v->valuedouble;
    if ((v = cJSON_GetObjectItem(body, "source_protocol")) &&
        cJSON_IsString(v) && strcmp(v->valuestring, "TCP") == 0) {
        params.source_protocol = SRC_MODBUS_TCP;
    }
    if ((v = cJSON_GetObjectItem(body, "channel_id")) && cJSON_IsNumber(v))
        params.channel_id = (uint8_t)v->valuedouble;
    if ((v = cJSON_GetObjectItem(body, "function_codes")) && cJSON_IsArray(v)) {
        params.fc_count = 0;
        cJSON *item;
        cJSON_ArrayForEach(item, v) {
            if (params.fc_count >= 4) break;
            if (cJSON_IsNumber(item) &&
                item->valueint >= 1 && item->valueint <= 4) {
                params.function_codes[params.fc_count++] = (uint8_t)item->valueint;
            }
        }
    }
    cJSON_Delete(body);

    if (params.source_protocol == SRC_MODBUS_TCP) {
        runtime_config_t config;
        runtime_config_get(&config);
        bool endpoint_found = false;
        for (int i = 0; i < config.tcp_endpoint_count; ++i) {
            if (config.tcp_endpoints[i].enabled &&
                config.tcp_endpoints[i].endpoint_id == params.channel_id) {
                endpoint_found = true;
                break;
            }
        }
        if (!endpoint_found) {
            httpd_resp_set_status(req, "400 Bad Request");
            return send_json(req,
                "{\"error\":\"Selected Modbus TCP endpoint is not configured or enabled\"}");
        }
    }

    esp_err_t err = modbus_discover_full_scan(&params);
    if (err == ESP_OK) {
        cJSON *root = cJSON_CreateObject();
        cJSON_AddBoolToObject(root, "scan_started", true);
        cJSON_AddBoolToObject(root, "scan_complete", false);
        char *json = cJSON_PrintUnformatted(root);
        send_json(req, json);
        free(json);
        cJSON_Delete(root);
    } else {
        httpd_resp_send_400(req);
    }
    return ESP_OK;
}

/* POST /api/discover/apply */
static esp_err_t discover_apply_post_handler(httpd_req_t *req)
{
    if (require_authorization(req) != ESP_OK) return ESP_OK;
    if (modbus_discover_get_result().scan_in_progress) {
        httpd_resp_set_status(req, "409 Conflict");
        return send_json(req, "{\"error\":\"discovery scan in progress\"}");
    }
    int created = modbus_discover_apply_mappings();
    discover_apply_result_t apply = modbus_discover_get_apply_result();

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "mappings_created", created);
    cJSON_AddNumberToObject(root, "semantic_mappings", apply.semantic_mappings);
    cJSON_AddNumberToObject(root, "raw_mappings", apply.raw_mappings);
    cJSON_AddNumberToObject(root, "profile_devices", apply.profile_devices);
    cJSON_AddNumberToObject(root, "unresolved_devices", apply.unresolved_devices);
    cJSON_AddNumberToObject(root, "total_mappings", amm_get_mapping_count());

    char *json = cJSON_PrintUnformatted(root);
    send_json(req, json);
    free(json);
    cJSON_Delete(root);
    return ESP_OK;
}

/* POST /api/discover/reset */
static esp_err_t discover_reset_post_handler(httpd_req_t *req)
{
    if (require_authorization(req) != ESP_OK) return ESP_OK;
    modbus_discover_reset();
    return send_ok(req);
}

/* POST /api/discover/export */
static esp_err_t discover_export_post_handler(httpd_req_t *req)
{
    return discover_devices_get_handler(req);
}

/* POST /api/discover/import */
static esp_err_t discover_import_post_handler(httpd_req_t *req)
{
    cJSON *body = parse_request_json(req);
    if (body) cJSON_Delete(body);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "status", "unsupported");
    cJSON_AddStringToObject(root, "reason", "Import editing is available in simulator; ESP32 firmware keeps discovery data from live scans.");
    char *json = cJSON_PrintUnformatted(root);
    send_json(req, json);
    free(json);
    cJSON_Delete(root);
    return ESP_OK;
}

/* GET /api/bridge/status */
static esp_err_t bridge_status_get_handler(httpd_req_t *req)
{
    return send_json(req, "{\"available\":false,\"connected\":false,\"mode\":\"rtu\"}");
}

/* POST /api/bridge/connect */
static esp_err_t bridge_connect_post_handler(httpd_req_t *req)
{
    cJSON *body = parse_request_json(req);
    if (body) cJSON_Delete(body);
    return send_json(req, "{\"available\":false,\"connected\":false,\"reason\":\"MODBUS TCP bridge is only available in the PC simulator\"}");
}

/* POST /api/bridge/disconnect */
static esp_err_t bridge_disconnect_post_handler(httpd_req_t *req)
{
    return send_json(req, "{\"available\":false,\"connected\":false}");
}

/* Helper: parse slave_id and optional reg_addr from discover URI */
static bool parse_discover_device_uri(const char *uri, uint8_t *slave_id,
                                       uint16_t *reg_addr)
{
    /* URI: /api/discover/devices/<slave_id>[/registers/<reg_addr>[/toggle]] */
    const char *p = strstr(uri, "/devices/");
    if (!p) return false;
    p += 9; /* skip "/devices/" */
    *slave_id = (uint8_t)atoi(p);

    if (reg_addr) {
        const char *r = strstr(p, "/registers/");
        if (r) {
            r += 11; /* skip "/registers/" */
            *reg_addr = (uint16_t)atoi(r);
        }
    }
    return true;
}

/* PUT /api/discover/devices/<slave_id> -update device info */
static esp_err_t discover_device_put_handler(httpd_req_t *req)
{
    if (require_authorization(req) != ESP_OK) return ESP_OK;
    char uri[128];
    httpd_req_get_url_str(req, uri, sizeof(uri));

    uint8_t slave_id = 0;
    if (!parse_discover_device_uri(uri, &slave_id, NULL)) {
        httpd_resp_send_400(req);
        return ESP_FAIL;
    }

    cJSON *body = parse_request_json(req);
    if (!body) { httpd_resp_send_400(req); return ESP_FAIL; }

    cJSON *v;
    const char *device_id = NULL, *name = NULL, *mqtt_prefix = NULL;
    if ((v = cJSON_GetObjectItem(body, "device_id")) && cJSON_IsString(v))
        device_id = v->valuestring;
    if ((v = cJSON_GetObjectItem(body, "name")) && cJSON_IsString(v))
        name = v->valuestring;
    if ((v = cJSON_GetObjectItem(body, "mqtt_topic_prefix")) && cJSON_IsString(v))
        mqtt_prefix = v->valuestring;

    esp_err_t err = modbus_discover_update_device(slave_id, device_id,
                                                    name, mqtt_prefix);
    cJSON_Delete(body);

    if (err == ESP_OK) {
        return send_ok(req);
    }
    httpd_resp_send_400(req);
    return ESP_FAIL;
}

/* PUT /api/discover/devices/<slave_id>/registers/<reg_addr> -update register */
static esp_err_t discover_register_put_handler(httpd_req_t *req)
{
    if (require_authorization(req) != ESP_OK) return ESP_OK;
    char uri[128];
    httpd_req_get_url_str(req, uri, sizeof(uri));

    uint8_t slave_id = 0;
    uint16_t reg_addr = 0;
    if (!parse_discover_device_uri(uri, &slave_id, &reg_addr)) {
        httpd_resp_send_400(req);
        return ESP_FAIL;
    }

    cJSON *body = parse_request_json(req);
    if (!body) { httpd_resp_send_400(req); return ESP_FAIL; }

    cJSON *v;
    const char *name = NULL, *unit = NULL;
    data_type_t dtype = DT_FLOAT32;
    bool writable = false;
    float range_min = -999.0f, range_max = 999.0f;

    if ((v = cJSON_GetObjectItem(body, "name")) && cJSON_IsString(v))
        name = v->valuestring;
    else if ((v = cJSON_GetObjectItem(body, "inferred_name")) && cJSON_IsString(v))
        name = v->valuestring;
    if ((v = cJSON_GetObjectItem(body, "unit")) && cJSON_IsString(v))
        unit = v->valuestring;
    else if ((v = cJSON_GetObjectItem(body, "inferred_unit")) && cJSON_IsString(v))
        unit = v->valuestring;
    if ((v = cJSON_GetObjectItem(body, "data_type")) && cJSON_IsString(v)) {
        dtype = parse_data_type_name(v->valuestring);
    }
    if ((v = cJSON_GetObjectItem(body, "writable")) && cJSON_IsBool(v))
        writable = cJSON_IsTrue(v);
    bool has_range_min = false, has_range_max = false;
    if ((v = cJSON_GetObjectItem(body, "range_min")) && cJSON_IsNumber(v)) {
        range_min = (float)v->valuedouble;
        has_range_min = true;
    }
    if ((v = cJSON_GetObjectItem(body, "range_max")) && cJSON_IsNumber(v)) {
        range_max = (float)v->valuedouble;
        has_range_max = true;
    }
    /*
     * The edit form may omit the range fields. Fall back to the register's
     * current range so the AMM write-through does not clobber a previously
     * configured valid range with handler defaults.
     */
    if (!has_range_min || !has_range_max) {
        const discovered_register_t *cur =
            modbus_discover_find_register(slave_id, reg_addr);
        if (cur) {
            if (!has_range_min) range_min = cur->range_min;
            if (!has_range_max) range_max = cur->range_max;
        }
    }

    esp_err_t err = modbus_discover_update_register(slave_id, reg_addr,
                                                      name, unit, dtype,
                                                      writable, range_min, range_max);
    cJSON_Delete(body);

    if (err == ESP_OK) {
        return send_ok(req);
    }
    httpd_resp_send_400(req);
    return ESP_FAIL;
}

/* POST /api/discover/devices/<slave_id>/registers/<reg_addr>/toggle */
static esp_err_t discover_register_toggle_handler(httpd_req_t *req)
{
    if (require_authorization(req) != ESP_OK) return ESP_OK;
    char uri[128];
    httpd_req_get_url_str(req, uri, sizeof(uri));

    uint8_t slave_id = 0;
    uint16_t reg_addr = 0;
    if (!parse_discover_device_uri(uri, &slave_id, &reg_addr)) {
        httpd_resp_send_400(req);
        return ESP_FAIL;
    }

    /* Consume body (may be empty) */
    char buf[16] = {0};
    if (req->content_len > 0 && req->content_len < sizeof(buf)) {
        httpd_req_recv(req, buf, req->content_len);
    }

    bool new_state = false;
    esp_err_t err = modbus_discover_toggle_register(slave_id, reg_addr, &new_state);
    if (err == ESP_OK) {
        char resp[64];
        snprintf(resp, sizeof(resp), "{\"status\":\"ok\",\"valid\":%s}",
                 new_state ? "true" : "false");
        return send_json(req, resp);
    }
    httpd_resp_send_400(req);
    return ESP_FAIL;
}

/* DELETE /api/discover/devices/<slave_id>/registers/<reg_addr> */
static esp_err_t discover_register_delete_handler(httpd_req_t *req)
{
    if (require_authorization(req) != ESP_OK) return ESP_OK;
    char uri[128];
    httpd_req_get_url_str(req, uri, sizeof(uri));

    uint8_t slave_id = 0;
    uint16_t reg_addr = 0;
    if (!parse_discover_device_uri(uri, &slave_id, &reg_addr)) {
        httpd_resp_send_400(req);
        return ESP_FAIL;
    }

    esp_err_t err = modbus_discover_delete_register(slave_id, reg_addr);
    if (err == ESP_OK) {
        return send_ok(req);
    }
    httpd_resp_send_400(req);
    return ESP_FAIL;
}

static esp_err_t gateway_config_get_handler(httpd_req_t *req)
{
    runtime_config_t config;
    runtime_config_get(&config);
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "gateway_id", config.gateway_id);
    cJSON_AddStringToObject(root, "locale", config.locale == UI_LOCALE_ZH_CN ? "zh-CN" : "en-US");
    cJSON_AddBoolToObject(root, "prefer_ethernet", config.prefer_ethernet);
    cJSON_AddBoolToObject(root, "lcd_enabled", config.lcd_enabled);
    cJSON_AddBoolToObject(root, "tf_enabled", config.tf_enabled);
    cJSON_AddBoolToObject(root, "mcp_write_enabled", config.mcp_write_enabled);
    cJSON_AddStringToObject(root, "mcp_endpoint", "/mcp");
    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    esp_err_t err = send_json(req, json);
    free(json);
    return err;
}

static esp_err_t gateway_config_put_handler(httpd_req_t *req)
{
    if (require_authorization(req) != ESP_OK) return ESP_OK;
    cJSON *root = parse_request_json(req);
    if (root == NULL) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    runtime_config_t config;
    runtime_config_get(&config);
    cJSON *value = cJSON_GetObjectItem(root, "gateway_id");
    if (cJSON_IsString(value)) strlcpy(config.gateway_id, value->valuestring, sizeof(config.gateway_id));
    value = cJSON_GetObjectItem(root, "locale");
    if (cJSON_IsString(value)) config.locale = strcmp(value->valuestring, "en-US") == 0 ? UI_LOCALE_EN_US : UI_LOCALE_ZH_CN;
#define COPY_BOOL_CONFIG(key, field) do { value = cJSON_GetObjectItem(root, key); \
    if (cJSON_IsBool(value)) config.field = cJSON_IsTrue(value); } while (0)
    COPY_BOOL_CONFIG("prefer_ethernet", prefer_ethernet);
    COPY_BOOL_CONFIG("lcd_enabled", lcd_enabled);
    COPY_BOOL_CONFIG("tf_enabled", tf_enabled);
    COPY_BOOL_CONFIG("mcp_write_enabled", mcp_write_enabled);
#undef COPY_BOOL_CONFIG
    cJSON_Delete(root);
    esp_err_t err = runtime_config_set(&config);
    return err == ESP_OK ? send_ok(req)
                         : httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Save failed");
}

static esp_err_t time_config_get_handler(httpd_req_t *req)
{
    runtime_config_t config;
    runtime_config_get(&config);
    time_service_status_t status;
    time_service_get_status(&status);
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "enabled", config.time.enabled);
    cJSON_AddStringToObject(root, "server1", config.time.server1);
    cJSON_AddStringToObject(root, "server2", config.time.server2);
    cJSON_AddStringToObject(root, "timezone", config.time.timezone);
    cJSON_AddNumberToObject(root, "sync_interval_ms", config.time.sync_interval_ms);
    cJSON_AddBoolToObject(root, "synchronized", status.synchronized);
    cJSON_AddNumberToObject(root, "last_sync_ms", (double)status.last_sync_ms);
    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    esp_err_t err = send_json(req, json);
    free(json);
    return err;
}

static esp_err_t time_config_put_handler(httpd_req_t *req)
{
    if (require_authorization(req) != ESP_OK) return ESP_OK;
    cJSON *root = parse_request_json(req);
    if (root == NULL) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    runtime_config_t config;
    runtime_config_get(&config);
    cJSON *value = cJSON_GetObjectItem(root, "enabled");
    if (cJSON_IsBool(value)) config.time.enabled = cJSON_IsTrue(value);
#define COPY_TIME_STRING(key, field) do { value = cJSON_GetObjectItem(root, key); \
    if (cJSON_IsString(value)) strlcpy(config.time.field, value->valuestring, \
                                      sizeof(config.time.field)); } while (0)
    COPY_TIME_STRING("server1", server1);
    COPY_TIME_STRING("server2", server2);
    COPY_TIME_STRING("timezone", timezone);
#undef COPY_TIME_STRING
    value = cJSON_GetObjectItem(root, "sync_interval_ms");
    if (cJSON_IsNumber(value)) config.time.sync_interval_ms = (uint32_t)value->valuedouble;
    cJSON_Delete(root);
    esp_err_t err = runtime_config_set(&config);
    return err == ESP_OK
        ? send_json(req, "{\"status\":\"ok\",\"restart_required\":true}")
        : httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid configuration");
}

static esp_err_t security_config_get_handler(httpd_req_t *req)
{
    runtime_config_t config;
    runtime_config_get(&config);
    gateway_health_t health;
    health_service_get(&health);
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "auth_enabled", config.security.auth_enabled);
    cJSON_AddStringToObject(root, "username", config.security.username);
    cJSON_AddBoolToObject(root, "token_configured",
                          config.security.password_sha256[0] != '\0');
    cJSON_AddBoolToObject(root, "ota_enabled", config.security.ota_enabled);
    cJSON_AddBoolToObject(root, "ota_allow_http", config.security.ota_allow_http);
    cJSON_AddBoolToObject(root, "secure_boot_enabled", health.secure_boot_enabled);
    cJSON_AddBoolToObject(root, "flash_encryption_enabled",
                          health.flash_encryption_enabled);
    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    esp_err_t err = send_json(req, json);
    free(json);
    return err;
}

static esp_err_t security_config_put_handler(httpd_req_t *req)
{
    if (require_authorization(req) != ESP_OK) return ESP_OK;
    cJSON *root = parse_request_json(req);
    if (root == NULL) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    runtime_config_t config;
    runtime_config_get(&config);
    cJSON *value = cJSON_GetObjectItem(root, "auth_enabled");
    if (cJSON_IsBool(value)) config.security.auth_enabled = cJSON_IsTrue(value);
    value = cJSON_GetObjectItem(root, "username");
    if (cJSON_IsString(value)) {
        strlcpy(config.security.username, value->valuestring,
                sizeof(config.security.username));
    }
    cJSON *token = cJSON_GetObjectItem(root, "bearer_token");
    cJSON *token_hash = cJSON_GetObjectItem(root, "bearer_token_sha256");
    if (cJSON_IsString(token) && token->valuestring[0] != '\0') {
        sha256_hex(token->valuestring, config.security.password_sha256);
    } else if (cJSON_IsString(token_hash) &&
               strlen(token_hash->valuestring) == 64) {
        strlcpy(config.security.password_sha256, token_hash->valuestring,
                sizeof(config.security.password_sha256));
    }
    value = cJSON_GetObjectItem(root, "ota_enabled");
    if (cJSON_IsBool(value)) config.security.ota_enabled = cJSON_IsTrue(value);
    value = cJSON_GetObjectItem(root, "ota_allow_http");
    if (cJSON_IsBool(value)) config.security.ota_allow_http = cJSON_IsTrue(value);
    cJSON_Delete(root);
    esp_err_t err = runtime_config_set(&config);
    return err == ESP_OK ? send_ok(req)
        : httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid configuration");
}

static esp_err_t ota_status_get_handler(httpd_req_t *req)
{
    ota_status_t status;
    ota_service_get_status(&status);
    char response[192];
    snprintf(response, sizeof(response),
             "{\"state\":%d,\"error\":%ld,\"message\":\"%s\"}",
             status.state, (long)status.last_error, status.message);
    return send_json(req, response);
}

static esp_err_t ota_start_post_handler(httpd_req_t *req)
{
    if (require_authorization(req) != ESP_OK) return ESP_OK;
    cJSON *root = parse_request_json(req);
    if (root == NULL) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    cJSON *url = cJSON_GetObjectItem(root, "url");
    cJSON *sha = cJSON_GetObjectItem(root, "sha256");
    esp_err_t err = cJSON_IsString(url) && cJSON_IsString(sha)
        ? ota_service_start(url->valuestring, sha->valuestring)
        : ESP_ERR_INVALID_ARG;
    cJSON_Delete(root);
    if (err != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   esp_err_to_name(err));
    }
    httpd_resp_set_status(req, "202 Accepted");
    return send_json(req, "{\"status\":\"started\"}");
}

/* ================================================================
 * OPTIONS handler (CORS preflight)
 * ================================================================ */
static esp_err_t cors_options_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, PUT, POST, DELETE, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers",
                       "Content-Type, Authorization");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* ================================================================
 * Server Start / Stop
 * ================================================================ */
esp_err_t web_server_start(uint16_t port)
{
    /* Initialize log buffer mutex */
    if (!s_log_mutex) {
        s_log_mutex = xSemaphoreCreateMutex();
        if (!s_log_mutex) {
            ESP_LOGE(TAG, "Failed to create log mutex");
            return ESP_FAIL;
        }
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = port;
    config.stack_size = 8192;
    config.task_priority = 5;
    config.max_uri_handlers = 60;
    config.max_open_sockets = 4;
    config.backlog_conn = 2;
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 3;
    config.send_wait_timeout = 5;
    config.uri_match_fn = httpd_uri_match_wildcard;

    esp_err_t err = httpd_start(&s_server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(err));
        return err;
    }

    /* Register URI handlers */
    const httpd_uri_t root_get = {
        .uri = "/", .method = HTTP_GET, .handler = root_get_handler,
    };
    const httpd_uri_t ui_asset_get = {
        .uri = "/ui.bin", .method = HTTP_GET, .handler = ui_asset_get_handler,
    };
    const httpd_uri_t sys_status_get = {
        .uri = "/api/system/status", .method = HTTP_GET, .handler = system_status_get_handler,
    };
    const httpd_uri_t sys_logs_get = {
        .uri = "/api/system/logs", .method = HTTP_GET, .handler = system_logs_get_handler,
    };
    const httpd_uri_t gateway_get = {
        .uri = "/api/gateway/config", .method = HTTP_GET, .handler = gateway_config_get_handler,
    };
    const httpd_uri_t gateway_put = {
        .uri = "/api/gateway/config", .method = HTTP_PUT, .handler = gateway_config_put_handler,
    };
    const httpd_uri_t time_get = {
        .uri = "/api/time/config", .method = HTTP_GET, .handler = time_config_get_handler,
    };
    const httpd_uri_t time_put = {
        .uri = "/api/time/config", .method = HTTP_PUT, .handler = time_config_put_handler,
    };
    const httpd_uri_t security_get = {
        .uri = "/api/security/config", .method = HTTP_GET,
        .handler = security_config_get_handler,
    };
    const httpd_uri_t security_put = {
        .uri = "/api/security/config", .method = HTTP_PUT,
        .handler = security_config_put_handler,
    };
    const httpd_uri_t ota_status_get = {
        .uri = "/api/system/ota", .method = HTTP_GET,
        .handler = ota_status_get_handler,
    };
    const httpd_uri_t ota_start_post = {
        .uri = "/api/system/ota", .method = HTTP_POST,
        .handler = ota_start_post_handler,
    };
    const httpd_uri_t wifi_get = {
        .uri = "/api/wifi/config", .method = HTTP_GET, .handler = wifi_config_get_handler,
    };
    const httpd_uri_t wifi_put = {
        .uri = "/api/wifi/config", .method = HTTP_PUT, .handler = wifi_config_put_handler,
    };
    const httpd_uri_t mqtt_get = {
        .uri = "/api/mqtt/config", .method = HTTP_GET, .handler = mqtt_config_get_handler,
    };
    const httpd_uri_t mqtt_put = {
        .uri = "/api/mqtt/config", .method = HTTP_PUT, .handler = mqtt_config_put_handler,
    };
    const httpd_uri_t mqtt_test_post = {
        .uri = "/api/mqtt/test", .method = HTTP_POST, .handler = mqtt_test_post_handler,
    };
    const httpd_uri_t mqtt_disconnect_post = {
        .uri = "/api/mqtt/disconnect", .method = HTTP_POST, .handler = mqtt_disconnect_post_handler,
    };
    const httpd_uri_t modbus_get = {
        .uri = "/api/modbus/config", .method = HTTP_GET, .handler = modbus_config_get_handler,
    };
    const httpd_uri_t modbus_put = {
        .uri = "/api/modbus/config", .method = HTTP_PUT, .handler = modbus_config_put_handler,
    };
    const httpd_uri_t modbus_logs_get = {
        .uri = "/api/modbus/logs", .method = HTTP_GET, .handler = modbus_logs_get_handler,
    };
    const httpd_uri_t modbus_logs_del = {
        .uri = "/api/modbus/logs", .method = HTTP_DELETE, .handler = modbus_logs_delete_handler,
    };
    const httpd_uri_t mappings_get = {
        .uri = "/api/mappings", .method = HTTP_GET, .handler = mappings_get_handler,
    };
    const httpd_uri_t mappings_post = {
        .uri = "/api/mappings", .method = HTTP_POST, .handler = mappings_post_handler,
    };
    const httpd_uri_t mappings_clear = {
        .uri = "/api/mappings", .method = HTTP_DELETE, .handler = mappings_clear_handler,
    };
    const httpd_uri_t mappings_import_post = {
        .uri = "/api/mappings/import", .method = HTTP_POST,
        .handler = mappings_import_post_handler,
    };
    const httpd_uri_t mappings_rollback_post = {
        .uri = "/api/mappings/rollback", .method = HTTP_POST,
        .handler = mappings_rollback_post_handler,
    };
    const httpd_uri_t mappings_put = {
        .uri = "/api/mappings/*", .method = HTTP_PUT, .handler = mappings_put_handler,
    };
    const httpd_uri_t mappings_del = {
        .uri = "/api/mappings/*", .method = HTTP_DELETE, .handler = mappings_delete_handler,
    };
    /* Discovery endpoints */
    const httpd_uri_t discover_status_get = {
        .uri = "/api/discover/status", .method = HTTP_GET, .handler = discover_status_get_handler,
    };
    const httpd_uri_t discover_devices_get = {
        .uri = "/api/discover/devices", .method = HTTP_GET, .handler = discover_devices_get_handler,
    };
    const httpd_uri_t discover_scan_post = {
        .uri = "/api/discover/scan", .method = HTTP_POST, .handler = discover_scan_post_handler,
    };
    const httpd_uri_t discover_apply_post = {
        .uri = "/api/discover/apply", .method = HTTP_POST, .handler = discover_apply_post_handler,
    };
    const httpd_uri_t discover_reset_post = {
        .uri = "/api/discover/reset", .method = HTTP_POST, .handler = discover_reset_post_handler,
    };
    const httpd_uri_t discover_export_post = {
        .uri = "/api/discover/export", .method = HTTP_POST, .handler = discover_export_post_handler,
    };
    const httpd_uri_t discover_import_post = {
        .uri = "/api/discover/import", .method = HTTP_POST, .handler = discover_import_post_handler,
    };
    const httpd_uri_t bridge_status_get = {
        .uri = "/api/bridge/status", .method = HTTP_GET, .handler = bridge_status_get_handler,
    };
    const httpd_uri_t bridge_connect_post = {
        .uri = "/api/bridge/connect", .method = HTTP_POST, .handler = bridge_connect_post_handler,
    };
    const httpd_uri_t bridge_disconnect_post = {
        .uri = "/api/bridge/disconnect", .method = HTTP_POST, .handler = bridge_disconnect_post_handler,
    };
    /* Discovery editing endpoints (register specific patterns first) */
    const httpd_uri_t discover_reg_put = {
        .uri = "/api/discover/devices/*/registers/*", .method = HTTP_PUT,
        .handler = discover_register_put_handler,
    };
    const httpd_uri_t discover_reg_toggle = {
        .uri = "/api/discover/devices/*/registers/*/toggle", .method = HTTP_POST,
        .handler = discover_register_toggle_handler,
    };
    const httpd_uri_t discover_reg_del = {
        .uri = "/api/discover/devices/*/registers/*", .method = HTTP_DELETE,
        .handler = discover_register_delete_handler,
    };
    const httpd_uri_t discover_dev_put = {
        .uri = "/api/discover/devices/*", .method = HTTP_PUT,
        .handler = discover_device_put_handler,
    };
    const httpd_uri_t cors_options = {
        .uri = "/api/*", .method = HTTP_OPTIONS, .handler = cors_options_handler,
    };

    httpd_register_uri_handler(s_server, &root_get);
    httpd_register_uri_handler(s_server, &ui_asset_get);
    httpd_register_uri_handler(s_server, &sys_status_get);
    httpd_register_uri_handler(s_server, &sys_logs_get);
    httpd_register_uri_handler(s_server, &gateway_get);
    httpd_register_uri_handler(s_server, &gateway_put);
    httpd_register_uri_handler(s_server, &time_get);
    httpd_register_uri_handler(s_server, &time_put);
    httpd_register_uri_handler(s_server, &security_get);
    httpd_register_uri_handler(s_server, &security_put);
    httpd_register_uri_handler(s_server, &ota_status_get);
    httpd_register_uri_handler(s_server, &ota_start_post);
    httpd_register_uri_handler(s_server, &wifi_get);
    httpd_register_uri_handler(s_server, &wifi_put);
    httpd_register_uri_handler(s_server, &mqtt_get);
    httpd_register_uri_handler(s_server, &mqtt_put);
    httpd_register_uri_handler(s_server, &mqtt_test_post);
    httpd_register_uri_handler(s_server, &mqtt_disconnect_post);
    httpd_register_uri_handler(s_server, &modbus_get);
    httpd_register_uri_handler(s_server, &modbus_put);
    httpd_register_uri_handler(s_server, &modbus_logs_get);
    httpd_register_uri_handler(s_server, &modbus_logs_del);
    httpd_register_uri_handler(s_server, &mappings_get);
    httpd_register_uri_handler(s_server, &mappings_post);
    httpd_register_uri_handler(s_server, &mappings_clear);
    httpd_register_uri_handler(s_server, &mappings_import_post);
    httpd_register_uri_handler(s_server, &mappings_rollback_post);
    httpd_register_uri_handler(s_server, &mappings_put);
    httpd_register_uri_handler(s_server, &mappings_del);
    httpd_register_uri_handler(s_server, &discover_status_get);
    httpd_register_uri_handler(s_server, &discover_devices_get);
    httpd_register_uri_handler(s_server, &discover_scan_post);
    httpd_register_uri_handler(s_server, &discover_apply_post);
    httpd_register_uri_handler(s_server, &discover_reset_post);
    httpd_register_uri_handler(s_server, &discover_export_post);
    httpd_register_uri_handler(s_server, &discover_import_post);
    httpd_register_uri_handler(s_server, &bridge_status_get);
    httpd_register_uri_handler(s_server, &bridge_connect_post);
    httpd_register_uri_handler(s_server, &bridge_disconnect_post);
    httpd_register_uri_handler(s_server, &discover_reg_put);
    httpd_register_uri_handler(s_server, &discover_reg_toggle);
    httpd_register_uri_handler(s_server, &discover_reg_del);
    httpd_register_uri_handler(s_server, &discover_dev_put);
    httpd_register_uri_handler(s_server, &cors_options);
    ESP_ERROR_CHECK(automation_web_register(s_server));
    ESP_ERROR_CHECK(mcp_http_register(s_server));

    ESP_LOGI(TAG, "Web configuration server started on port %u", port);
    return ESP_OK;
}

void web_server_stop(void)
{
    if (s_server) {
        httpd_stop(s_server);
        s_server = NULL;
        ESP_LOGI(TAG, "Web server stopped");
    }
}


