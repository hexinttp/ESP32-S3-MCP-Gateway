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

#include "web_server.h"
#include "gateway_config.h"
#include "amm/amm_mapping.h"
#include "tcm/tcm_context.h"
#include "tcm/tcm_state_pool.h"
#include "modbus/modbus_comm_log.h"
#include "mqtt_comm/mqtt_handler.h"
#include "eval/eval_logger.h"
#include "automation/automation_web.h"
#include "mcp/mcp_http.h"
#include "config/runtime_config.h"
#include "uif/uif_persistence.h"
#include "board/tf_storage.h"
#include "network/network_manager.h"

static const char *TAG = "WEB";

static httpd_handle_t s_server = NULL;

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
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_send(req, json_str, strlen(json_str));
    return ESP_OK;
}

static esp_err_t send_ok(httpd_req_t *req)
{
    return send_json(req, "{\"status\":\"ok\"}");
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
        "<title>ESP32-S3 Gateway</title>"
        "<style>body{margin:0;display:grid;place-items:center;min-height:100vh;"
        "font-family:Arial,sans-serif;background:#f2f5f8;color:#102033}"
        ".box{text-align:center}.bar{width:240px;height:5px;background:#d9e2ec;"
        "overflow:hidden;margin:18px auto}.fill{height:100%%;width:0;background:#1473e6;"
        "transition:width .12s}.error{color:#b42318;max-width:520px}</style></head>"
        "<body><div class=\"box\"><strong>ESP32-S3 Gateway</strong>"
        "<div class=\"bar\"><div class=\"fill\" id=\"p\"></div></div>"
        "<div id=\"s\">Loading configuration...</div></div><script>"
        "(async()=>{try{if(!('DecompressionStream'in window))"
        "throw new Error('Browser decompression is unavailable');"
        "const total=%u,size=4096,parts=[];"
        "for(let offset=0;offset<total;offset+=size){"
        "const r=await fetch('/ui.bin?offset='+offset,{cache:'no-store'});"
        "if(!r.ok)throw new Error('UI segment '+offset+' failed: '+r.status);"
        "parts.push(await r.arrayBuffer());"
        "document.getElementById('p').style.width="
        "Math.min(100,Math.round((offset+size)*100/total))+'%%';"
        "if(offset+size<total)await new Promise(done=>setTimeout(done,250));}"
        "const stream=new Blob(parts).stream().pipeThrough(new DecompressionStream('gzip'));"
        "const html=await new Response(stream).text();"
        "document.open();document.write(html);document.close();"
        "}catch(e){const s=document.getElementById('s');s.className='error';"
        "s.textContent='Page load failed / 页面加载失败: '+e.message;}})();"
        "</script></body></html>",
        (unsigned)html_len);
    if (loader_len <= 0 || (size_t)loader_len >= sizeof(loader)) {
        return ESP_ERR_HTTPD_RESP_HDR;
    }

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    httpd_resp_set_hdr(req, "Connection", "close");
    esp_err_t err = httpd_resp_send(req, loader, loader_len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Web UI loader transfer failed: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
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
    httpd_resp_set_hdr(req, "Connection", "close");
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
    cJSON_AddNumberToObject(root, "uptime_seconds",
                            (int64_t)(esp_timer_get_time() / 1000000LL));
    cJSON_AddNumberToObject(root, "sequence_counter", tcm_get_sequence_counter());

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
    esp_err_t err = runtime_config_set(&config);

    cJSON_Delete(root);
    return err == ESP_OK ? send_json(req, "{\"status\":\"ok\",\"restart_required\":true}")
                         : httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Save failed");
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
    cJSON_AddStringToObject(root, "uri", config.mqtt.uri);
    cJSON_AddStringToObject(root, "client_id", config.mqtt.client_id);
    cJSON_AddStringToObject(root, "username", config.mqtt.username);
    cJSON_AddStringToObject(root, "password", config.mqtt.password[0] ? "********" : "");
    cJSON_AddNumberToObject(root, "keepalive", config.mqtt.keepalive_sec);
    cJSON_AddNumberToObject(root, "qos", config.mqtt.qos);
    cJSON_AddStringToObject(root, "topic_prefix", config.mqtt.data_prefix);
    cJSON_AddStringToObject(root, "command_prefix", config.mqtt.command_prefix);

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
    cJSON *root = parse_request_json(req);
    if (!root) {
        httpd_resp_send_400(req);
        return ESP_FAIL;
    }

    runtime_config_t config;
    runtime_config_get(&config);
#define COPY_MQTT_STRING(key, field) do { cJSON *item = cJSON_GetObjectItem(root, key); \
    if (cJSON_IsString(item) && strcmp(item->valuestring, "********") != 0) \
        strlcpy(config.mqtt.field, item->valuestring, sizeof(config.mqtt.field)); } while (0)
    COPY_MQTT_STRING("uri", uri);
    COPY_MQTT_STRING("client_id", client_id);
    COPY_MQTT_STRING("username", username);
    COPY_MQTT_STRING("password", password);
    COPY_MQTT_STRING("topic_prefix", data_prefix);
    COPY_MQTT_STRING("command_prefix", command_prefix);
#undef COPY_MQTT_STRING
    cJSON *value = cJSON_GetObjectItem(root, "enabled");
    if (cJSON_IsBool(value)) config.mqtt.enabled = cJSON_IsTrue(value);
    value = cJSON_GetObjectItem(root, "keepalive");
    if (cJSON_IsNumber(value)) config.mqtt.keepalive_sec = value->valueint;
    value = cJSON_GetObjectItem(root, "qos");
    if (cJSON_IsNumber(value) && value->valueint >= 0 && value->valueint <= 2) config.mqtt.qos = value->valueint;
    esp_err_t err = runtime_config_set(&config);
    cJSON_Delete(root);
    return err == ESP_OK ? send_json(req, "{\"status\":\"ok\",\"restart_required\":true}")
                         : httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Save failed");
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
    cJSON_AddNumberToObject(root, "poll_interval", POLL_INTERVAL_MS);
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
    cJSON_Delete(root);
    return err == ESP_OK ? send_json(req, "{\"status\":\"ok\",\"restart_required\":true}")
                         : httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Save failed");
}

/* ================================================================
 * GET /api/mappings -List all mapping entries
 * ================================================================ */
static esp_err_t mappings_get_handler(httpd_req_t *req)
{
    cJSON *arr = cJSON_CreateArray();
    if (arr == NULL) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
    }

    int active_count = amm_get_mapping_count();
    amm_mapping_entry_t *entries = NULL;
    if (active_count > 0) {
        entries = calloc((size_t)active_count, sizeof(*entries));
    }
    if (active_count > 0 && entries == NULL) {
        cJSON_Delete(arr);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
    }
    int entry_count = active_count > 0 ? amm_get_entries(entries, active_count) : 0;

    for (int i = 0; i < entry_count; i++) {
        amm_mapping_entry_t *e = &entries[i];

        cJSON *obj = cJSON_CreateObject();
        cJSON_AddStringToObject(obj, "source_protocol", e->source_protocol == SRC_MODBUS_TCP ? "TCP" : "RTU");
        cJSON_AddNumberToObject(obj, "channel_id", e->channel_id);
        cJSON_AddNumberToObject(obj, "slave_id", e->slave_id);
        cJSON_AddNumberToObject(obj, "register_address", e->register_address);
        cJSON_AddStringToObject(obj, "data_type",
            e->data_type == DT_FLOAT32 ? "FLOAT32" :
            e->data_type == DT_INT16   ? "INT16"   :
            e->data_type == DT_UINT16  ? "UINT16"  :
            e->data_type == DT_INT32   ? "INT32"   : "UINT32");
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
        cJSON_AddStringToObject(obj, "device_id", e->device_id);
        cJSON_AddStringToObject(obj, "point_id", e->point_id);
        cJSON_AddStringToObject(obj, "measurement_name", e->measurement_name);
        cJSON_AddStringToObject(obj, "unit", e->unit);
        cJSON_AddStringToObject(obj, "mqtt_topic", e->mqtt_topic);
        cJSON_AddBoolToObject(obj, "writable", e->constraint.writable);
        cJSON_AddNumberToObject(obj, "range_min", e->constraint.valid_range_min);
        cJSON_AddNumberToObject(obj, "range_max", e->constraint.valid_range_max);
        tcm_context_t state;
        if (tcm_state_pool_get(e->device_id, e->point_id, &state) == ESP_OK) {
            cJSON_AddNumberToObject(obj, "current_raw_value", state.raw_value);
            cJSON_AddNumberToObject(obj, "current_value", state.value);
            cJSON_AddNumberToObject(obj, "current_timestamp_ms",
                                    (double)state.timestamp_ms);
            cJSON_AddNumberToObject(obj, "quality_state", state.quality_state);
        }
        cJSON_AddItemToArray(arr, obj);
    }

    char *json = cJSON_PrintUnformatted(arr);
    send_json(req, json);
    free(json);
    free(entries);
    cJSON_Delete(arr);
    return ESP_OK;
}

/* ================================================================
 * POST /api/mappings -Add a new mapping
 * ================================================================ */
static esp_err_t mappings_post_handler(httpd_req_t *req)
{
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
        if      (strcmp(v->valuestring, "FLOAT32") == 0) entry.data_type = DT_FLOAT32;
        else if (strcmp(v->valuestring, "INT16")   == 0) entry.data_type = DT_INT16;
        else if (strcmp(v->valuestring, "UINT16")  == 0) entry.data_type = DT_UINT16;
        else if (strcmp(v->valuestring, "INT32")   == 0) entry.data_type = DT_INT32;
        else if (strcmp(v->valuestring, "UINT32")  == 0) entry.data_type = DT_UINT32;
        else entry.data_type = DT_FLOAT32;
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

    /* Constraint */
    if ((v = cJSON_GetObjectItem(root, "writable")) && cJSON_IsBool(v))
        entry.constraint.writable = cJSON_IsTrue(v);
    if ((v = cJSON_GetObjectItem(root, "range_min")) && cJSON_IsNumber(v))
        entry.constraint.valid_range_min = (float)v->valuedouble;
    if ((v = cJSON_GetObjectItem(root, "range_max")) && cJSON_IsNumber(v))
        entry.constraint.valid_range_max = (float)v->valuedouble;

    entry.active = true;

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

/* ================================================================
 * PUT /api/mappings/:idx -Update mapping (re-add with same addr)
 * ================================================================ */
static esp_err_t mappings_put_handler(httpd_req_t *req)
{
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

/* ================================================================
 * System Log Buffer -ring buffer for recent log entries
 * ================================================================ */
#define WEB_LOG_MAX_ENTRIES   64
#define WEB_LOG_MAX_TEXT_LEN 128

typedef struct {
    char  text[WEB_LOG_MAX_TEXT_LEN];
    char  level; /* 'i'=info, 'w'=warn, 'e'=error, 'o'=ok */
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
    httpd_resp_set_hdr(req, "Connection", "close");
    esp_err_t send_err = httpd_resp_send_chunk(req, "[", 1);
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
            ",\"direction\":\"%s\",\"slave_id\":%u,\"function_code\":%u,"
            "\"register_address\":%u,\"register_count\":%u,"
            "\"status_code\":%ld,\"status\":\"%s\",\"truncated\":%s,"
            "\"frame_hex\":\"%s\",\"register_values\":[%s]}",
            i == 0 ? "" : ",", (unsigned long)entry.sequence,
            entry.timestamp_ms,
            entry.direction == MODBUS_COMM_TX ? "TX" : "RX",
            entry.slave_id, entry.function_code, entry.register_address,
            entry.register_count, (long)entry.status,
            esp_err_to_name(entry.status),
            entry.truncated ? "true" : "false", hex, values);
        if (length < 0 || length >= sizeof(chunk) ||
            httpd_resp_send_chunk(req, chunk, length) != ESP_OK) {
            send_err = ESP_FAIL;
            break;
        }
    }
    if (send_err == ESP_OK) send_err = httpd_resp_send_chunk(req, "]", 1);
    httpd_resp_send_chunk(req, NULL, 0);
    return send_err;
}

static esp_err_t modbus_logs_delete_handler(httpd_req_t *req)
{
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
    cJSON_AddNumberToObject(root, "devices_found", r.devices_found);
    cJSON_AddNumberToObject(root, "registers_found", r.registers_found);
    cJSON_AddNumberToObject(root, "mappings_created", r.mappings_created);
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
    cJSON *arr = cJSON_CreateArray();

    uint16_t count = modbus_discover_get_device_count();
    for (uint16_t i = 0; i < count; i++) {
        const discovered_device_t *dev = modbus_discover_get_device(i);
        if (!dev || !dev->active) continue;

        cJSON *dobj = cJSON_CreateObject();
        cJSON_AddNumberToObject(dobj, "slave_id", dev->slave_id);
        cJSON_AddStringToObject(dobj, "source_protocol",
                                dev->source_protocol == SRC_MODBUS_TCP ? "TCP" : "RTU");
        cJSON_AddNumberToObject(dobj, "channel_id", dev->channel_id);
        cJSON_AddStringToObject(dobj, "device_id", dev->device_id);
        cJSON_AddStringToObject(dobj, "name", dev->name[0] ? dev->name : dev->device_id);
        if (dev->description[0])
            cJSON_AddStringToObject(dobj, "description", dev->description);
        if (dev->mqtt_topic_prefix[0])
            cJSON_AddStringToObject(dobj, "mqtt_topic_prefix", dev->mqtt_topic_prefix);
        cJSON_AddNumberToObject(dobj, "register_count", dev->reg_count);
        cJSON_AddNumberToObject(dobj, "probe_function_code",
                                dev->probe_function_code);
        cJSON_AddNumberToObject(dobj, "probe_address", dev->probe_address);
        cJSON_AddNumberToObject(dobj, "probe_register_count",
                                dev->probe_register_count);

        cJSON *regs = cJSON_CreateArray();
        for (uint16_t j = 0; j < dev->reg_count; j++) {
            const discovered_register_t *reg = &dev->registers[j];

            cJSON *robj = cJSON_CreateObject();
            cJSON_AddNumberToObject(robj, "register_address", reg->register_address);
            cJSON_AddNumberToObject(robj, "function_code", reg->function_code);
            cJSON_AddNumberToObject(robj, "raw_value", reg->raw_value);
            cJSON_AddNumberToObject(robj, "read_start_address",
                                    reg->read_start_address);
            cJSON_AddNumberToObject(robj, "read_register_count",
                                    reg->read_register_count);
            cJSON_AddNumberToObject(robj, "value_register_index",
                                    reg->value_register_index);
            cJSON_AddStringToObject(robj, "data_type",
                reg->inferred_type == DT_FLOAT32 ? "FLOAT32" :
                reg->inferred_type == DT_INT16   ? "INT16"   : "UINT16");
            cJSON_AddNumberToObject(robj, "sample_value", (double)reg->sample_value);
            cJSON_AddStringToObject(robj, "inferred_name", reg->inferred_name);
            cJSON_AddStringToObject(robj, "inferred_unit", reg->inferred_unit);
            cJSON_AddBoolToObject(robj, "writable", reg->writable);
            cJSON_AddBoolToObject(robj, "valid", reg->valid);
            cJSON_AddItemToArray(regs, robj);
        }
        cJSON_AddItemToObject(dobj, "registers", regs);
        cJSON_AddBoolToObject(dobj, "active", dev->active);
        cJSON_AddItemToArray(arr, dobj);
    }

    char *json = cJSON_PrintUnformatted(arr);
    send_json(req, json);
    free(json);
    cJSON_Delete(arr);
    return ESP_OK;
}

/* POST /api/discover/scan */
static esp_err_t discover_scan_post_handler(httpd_req_t *req)
{
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
    params.function_codes[0] = 3;
    params.function_codes[1] = 4;
    params.fc_count = 2;
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
    if ((v = cJSON_GetObjectItem(body, "function_codes")) && cJSON_IsArray(v)) {
        params.fc_count = 0;
        cJSON *item;
        cJSON_ArrayForEach(item, v) {
            if (params.fc_count >= 2) break;
            if (cJSON_IsNumber(item) &&
                (item->valueint == 3 || item->valueint == 4)) {
                params.function_codes[params.fc_count++] = (uint8_t)item->valueint;
            }
        }
    }
    cJSON_Delete(body);

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
    int created = modbus_discover_apply_mappings();

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "mappings_created", created);
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
        if (strcmp(v->valuestring, "FLOAT32") == 0) dtype = DT_FLOAT32;
        else if (strcmp(v->valuestring, "INT16") == 0) dtype = DT_INT16;
        else dtype = DT_UINT16;
    }
    if ((v = cJSON_GetObjectItem(body, "writable")) && cJSON_IsBool(v))
        writable = cJSON_IsTrue(v);
    if ((v = cJSON_GetObjectItem(body, "range_min")) && cJSON_IsNumber(v))
        range_min = (float)v->valuedouble;
    if ((v = cJSON_GetObjectItem(body, "range_max")) && cJSON_IsNumber(v))
        range_max = (float)v->valuedouble;

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

/* ================================================================
 * OPTIONS handler (CORS preflight)
 * ================================================================ */
static esp_err_t cors_options_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, PUT, POST, DELETE, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
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
    config.max_uri_handlers = 48;
    config.max_open_sockets = 7;
    config.backlog_conn = 5;
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
    httpd_register_uri_handler(s_server, &wifi_get);
    httpd_register_uri_handler(s_server, &wifi_put);
    httpd_register_uri_handler(s_server, &mqtt_get);
    httpd_register_uri_handler(s_server, &mqtt_put);
    httpd_register_uri_handler(s_server, &modbus_get);
    httpd_register_uri_handler(s_server, &modbus_put);
    httpd_register_uri_handler(s_server, &modbus_logs_get);
    httpd_register_uri_handler(s_server, &modbus_logs_del);
    httpd_register_uri_handler(s_server, &mappings_get);
    httpd_register_uri_handler(s_server, &mappings_post);
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


