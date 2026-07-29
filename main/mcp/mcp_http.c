#include "mcp/mcp_http.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "amm/amm_mapping.h"
#include "automation/automation_engine.h"
#include "cJSON.h"
#include "config/runtime_config.h"
#include "esp_timer.h"
#include "mbedtls/sha256.h"
#include "modbus/modbus_discover.h"
#include "services/control_service.h"
#include "tcm/tcm_context.h"
#include "tcm/tcm_state_pool.h"
#include "uif/uif_persistence.h"

#define MCP_AUTH_MAX_FAILURES 5
#define MCP_AUTH_LOCKOUT_MS 30000

static uint8_t s_auth_failures;
static uint32_t s_authorized_requests;
static uint32_t s_auth_failure_total;
static int64_t s_last_authorized_ms;
static int64_t s_auth_locked_until_ms;

static void sha256_hex(const char *text, char output[65])
{
    uint8_t digest[32];
    mbedtls_sha256((const unsigned char *)text, strlen(text), digest, 0);
    for (int i = 0; i < 32; ++i) {
        snprintf(output + i * 2, 3, "%02x", digest[i]);
    }
    output[64] = '\0';
}

static bool constant_time_equal(const char *left, const char *right, size_t size)
{
    unsigned char difference = 0;
    for (size_t i = 0; i < size; ++i) {
        difference |= (unsigned char)left[i] ^ (unsigned char)right[i];
    }
    return difference == 0;
}

static esp_err_t require_mcp_authorization(httpd_req_t *req)
{
    runtime_config_t config;
    runtime_config_get(&config);
    if (!config.security.auth_enabled ||
        strlen(config.security.password_sha256) != 64) {
        httpd_resp_set_status(req, "403 Forbidden");
        httpd_resp_sendstr(req, "MCP access is disabled until authentication is configured");
        return ESP_ERR_NOT_ALLOWED;
    }

    int64_t now_ms = esp_timer_get_time() / 1000;
    if (now_ms < s_auth_locked_until_ms) {
        httpd_resp_set_status(req, "429 Too Many Requests");
        httpd_resp_set_hdr(req, "Retry-After", "30");
        httpd_resp_sendstr(req, "MCP authentication is temporarily locked");
        return ESP_ERR_TIMEOUT;
    }

    char authorization[192] = {0};
    bool accepted =
        httpd_req_get_hdr_value_str(req, "Authorization", authorization,
                                    sizeof(authorization)) == ESP_OK &&
        strncmp(authorization, "Bearer ", 7) == 0;
    char digest[65] = {0};
    if (accepted) {
        sha256_hex(authorization + 7, digest);
        accepted = constant_time_equal(digest, config.security.password_sha256, 64);
    }
    if (!accepted) {
        ++s_auth_failure_total;
        if (++s_auth_failures >= MCP_AUTH_MAX_FAILURES) {
            s_auth_failures = 0;
            s_auth_locked_until_ms = now_ms + MCP_AUTH_LOCKOUT_MS;
        }
        httpd_resp_set_status(req, "401 Unauthorized");
        httpd_resp_set_hdr(req, "WWW-Authenticate", "Bearer");
        httpd_resp_sendstr(req, "Unauthorized");
        return ESP_ERR_NOT_ALLOWED;
    }
    s_auth_failures = 0;
    s_auth_locked_until_ms = 0;
    ++s_authorized_requests;
    s_last_authorized_ms = now_ms;
    return ESP_OK;
}

static cJSON *read_body(httpd_req_t *req)
{
    if (req->content_len <= 0 || req->content_len > 8192) return NULL;
    char *buffer = malloc((size_t)req->content_len + 1);
    if (buffer == NULL) return NULL;
    int received = 0;
    while (received < req->content_len) {
        int count = httpd_req_recv(req, buffer + received, req->content_len - received);
        if (count <= 0) { free(buffer); return NULL; }
        received += count;
    }
    buffer[received] = '\0';
    cJSON *json = cJSON_Parse(buffer);
    free(buffer);
    return json;
}

static esp_err_t send_rpc(httpd_req_t *req, const cJSON *id, cJSON *result,
                          int error_code, const char *error_message)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "jsonrpc", "2.0");
    cJSON_AddItemToObject(root, "id", id != NULL ? cJSON_Duplicate(id, true) : cJSON_CreateNull());
    if (error_code == 0) cJSON_AddItemToObject(root, "result", result);
    else {
        cJSON *error = cJSON_AddObjectToObject(root, "error");
        cJSON_AddNumberToObject(error, "code", error_code);
        cJSON_AddStringToObject(error, "message", error_message);
        if (result != NULL) cJSON_Delete(result);
    }
    char *text = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "MCP-Protocol-Version", "2025-03-26");
    esp_err_t err = httpd_resp_sendstr(req, text);
    free(text);
    cJSON_Delete(root);
    return err;
}

static void add_tool(cJSON *tools, const char *name, const char *description,
                     const char *properties_json, const char *required_json)
{
    cJSON *tool = cJSON_CreateObject();
    cJSON_AddStringToObject(tool, "name", name);
    cJSON_AddStringToObject(tool, "description", description);
    cJSON *schema = cJSON_AddObjectToObject(tool, "inputSchema");
    cJSON_AddStringToObject(schema, "type", "object");
    cJSON *properties = cJSON_Parse(properties_json);
    cJSON_AddItemToObject(schema, "properties", properties != NULL ? properties : cJSON_CreateObject());
    if (required_json != NULL) {
        cJSON *required = cJSON_Parse(required_json);
        if (required != NULL) cJSON_AddItemToObject(schema, "required", required);
    }
    cJSON_AddItemToArray(tools, tool);
}

static cJSON *tools_list_result(void)
{
    cJSON *result = cJSON_CreateObject();
    cJSON *tools = cJSON_AddArrayToObject(result, "tools");
    add_tool(tools, "list_points", "List AMM semantic points and latest values", "{}", NULL);
    add_tool(tools, "read_point", "Read the latest TCM context for a semantic point",
             "{\"device_id\":{\"type\":\"string\"},\"point_id\":{\"type\":\"string\"}}",
             "[\"device_id\",\"point_id\"]");
    add_tool(tools, "write_point", "Write a point through AMM safety constraints",
             "{\"device_id\":{\"type\":\"string\"},\"point_id\":{\"type\":\"string\"},\"value\":{\"type\":\"number\"}}",
             "[\"device_id\",\"point_id\",\"value\"]");
    add_tool(tools, "discover_modbus_devices", "Start asynchronous Modbus RTU device discovery",
             "{\"slave_start\":{\"type\":\"integer\",\"minimum\":1,\"maximum\":247},\"slave_end\":{\"type\":\"integer\",\"minimum\":1,\"maximum\":247}}", NULL);
    add_tool(tools, "get_gateway_config", "Read non-secret gateway runtime configuration", "{}", NULL);
    add_tool(tools, "get_cache_status", "Read UIF offline cache and replay status", "{}", NULL);
    add_tool(tools, "configure_rule_from_natural_language",
             "Translate the user's natural-language request into this structured rule. "
             "Call first with confirmed=false, show the returned preview to the user, "
             "and call again with confirmed=true only after explicit user confirmation.",
             "{\"rule_name\":{\"type\":\"string\"},"
             "\"source_device\":{\"type\":\"string\"},\"source_point\":{\"type\":\"string\"},"
             "\"operator\":{\"type\":\"string\",\"enum\":[\"gt\",\"gte\",\"lt\",\"lte\",\"eq\",\"neq\"]},"
             "\"threshold\":{\"type\":\"number\"},\"hysteresis\":{\"type\":\"number\",\"minimum\":0},"
             "\"hold_ms\":{\"type\":\"integer\",\"minimum\":0},"
             "\"cooldown_ms\":{\"type\":\"integer\",\"minimum\":200},"
             "\"action\":{\"type\":\"string\",\"enum\":[\"mqtt_alert\",\"write_point\"]},"
             "\"target_device\":{\"type\":\"string\"},\"target_point\":{\"type\":\"string\"},"
             "\"target_value\":{\"type\":\"number\"},\"alert_topic\":{\"type\":\"string\"},"
             "\"alert_message\":{\"type\":\"string\"},\"interlock_device\":{\"type\":\"string\"},"
             "\"interlock_point\":{\"type\":\"string\"},\"interlock_required_state\":{\"type\":\"boolean\"},"
             "\"enabled\":{\"type\":\"boolean\"},\"confirmed\":{\"type\":\"boolean\"}}",
             "[\"rule_name\",\"source_device\",\"source_point\",\"operator\",\"threshold\","
             "\"action\",\"confirmed\"]");
    return result;
}

static cJSON *tool_text_result(cJSON *value, bool is_error)
{
    char *serialized = cJSON_PrintUnformatted(value);
    cJSON_Delete(value);
    cJSON *result = cJSON_CreateObject();
    cJSON *content = cJSON_AddArrayToObject(result, "content");
    cJSON *item = cJSON_CreateObject();
    cJSON_AddStringToObject(item, "type", "text");
    cJSON_AddStringToObject(item, "text", serialized != NULL ? serialized : "{}");
    cJSON_AddItemToArray(content, item);
    cJSON_AddBoolToObject(result, "isError", is_error);
    free(serialized);
    return result;
}

static cJSON *call_list_points(void)
{
    int capacity = amm_get_capacity();
    amm_mapping_entry_t *mappings =
        calloc((size_t)capacity, sizeof(*mappings));
    if (mappings == NULL) return cJSON_Parse("{\"error\":\"out of memory\"}");
    int count = amm_get_entries(mappings, capacity);
    cJSON *array = cJSON_CreateArray();
    for (int i = 0; i < count; ++i) {
        cJSON *point = cJSON_CreateObject();
        cJSON_AddStringToObject(point, "device_id", mappings[i].device_id);
        cJSON_AddStringToObject(point, "point_id", mappings[i].point_id);
        cJSON_AddStringToObject(point, "name", mappings[i].measurement_name);
        cJSON_AddStringToObject(point, "unit", mappings[i].unit);
        cJSON_AddBoolToObject(point, "writable", mappings[i].constraint.writable);
        cJSON_AddNumberToObject(point, "mapping_version", mappings[i].mapping_version);
        tcm_context_t state;
        if (tcm_state_pool_get(mappings[i].device_id, mappings[i].point_id, &state) == ESP_OK) {
            cJSON_AddNumberToObject(point, "value", state.value);
            cJSON_AddNumberToObject(point, "timestamp_ms", (double)state.timestamp_ms);
            cJSON_AddNumberToObject(point, "quality_state", state.quality_state);
        }
        cJSON_AddItemToArray(array, point);
    }
    free(mappings);
    return array;
}

static bool json_string(cJSON *root, const char *key, char *out, size_t size,
                        bool required)
{
    cJSON *item = cJSON_GetObjectItem(root, key);
    if (!cJSON_IsString(item)) return !required;
    strlcpy(out, item->valuestring, size);
    return !required || out[0] != '\0';
}

static bool parse_rule_operator(const char *value, automation_operator_t *out)
{
    static const char *names[] = {"gt", "gte", "lt", "lte", "eq", "neq"};
    for (int i = 0; i < (int)(sizeof(names) / sizeof(names[0])); ++i) {
        if (strcmp(value, names[i]) == 0) {
            *out = (automation_operator_t)i;
            return true;
        }
    }
    return false;
}

static cJSON *rule_response(const automation_rule_t *rule, bool valid,
                            bool applied, const char *reason, uint32_t id)
{
    static const char *operators[] = {"gt", "gte", "lt", "lte", "eq", "neq"};
    cJSON *response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "valid", valid);
    cJSON_AddBoolToObject(response, "applied", applied);
    cJSON_AddStringToObject(response, "reason", reason);
    if (id != 0) cJSON_AddNumberToObject(response, "rule_id", id);
    cJSON *preview = cJSON_AddObjectToObject(response, "preview");
    cJSON_AddStringToObject(preview, "name", rule->name);
    cJSON_AddStringToObject(preview, "source_device", rule->source_device);
    cJSON_AddStringToObject(preview, "source_point", rule->source_point);
    cJSON_AddStringToObject(preview, "operator", operators[rule->condition_operator]);
    cJSON_AddNumberToObject(preview, "threshold", rule->threshold);
    cJSON_AddNumberToObject(preview, "hysteresis", rule->hysteresis);
    cJSON_AddNumberToObject(preview, "hold_ms", rule->hold_ms);
    cJSON_AddNumberToObject(preview, "cooldown_ms", rule->cooldown_ms);
    cJSON_AddStringToObject(preview, "action",
                            rule->action == RULE_ACTION_WRITE_POINT
                                ? "write_point" : "mqtt_alert");
    if (rule->action == RULE_ACTION_WRITE_POINT) {
        cJSON_AddStringToObject(preview, "target_device", rule->target_device);
        cJSON_AddStringToObject(preview, "target_point", rule->target_point);
        cJSON_AddNumberToObject(preview, "target_value", rule->target_value);
    } else {
        cJSON_AddStringToObject(preview, "alert_topic", rule->alert_topic);
        cJSON_AddStringToObject(preview, "alert_message", rule->alert_message);
    }
    cJSON_AddBoolToObject(preview, "enabled", rule->enabled);
    return response;
}

static cJSON *call_configure_rule(cJSON *arguments, bool *is_error)
{
    automation_rule_t rule = {
        .enabled = true,
        .cooldown_ms = 1000,
        .action = RULE_ACTION_MQTT_ALERT,
    };
    char operator_name[8] = {0};
    char action_name[16] = {0};
    if (!json_string(arguments, "rule_name", rule.name, sizeof(rule.name), true) ||
        !json_string(arguments, "source_device", rule.source_device,
                     sizeof(rule.source_device), true) ||
        !json_string(arguments, "source_point", rule.source_point,
                     sizeof(rule.source_point), true) ||
        !json_string(arguments, "operator", operator_name,
                     sizeof(operator_name), true) ||
        !json_string(arguments, "action", action_name, sizeof(action_name), true)) {
        *is_error = true;
        return rule_response(&rule, false, false, "missing required rule field", 0);
    }
    cJSON *threshold = cJSON_GetObjectItem(arguments, "threshold");
    if (!cJSON_IsNumber(threshold) ||
        !parse_rule_operator(operator_name, &rule.condition_operator)) {
        *is_error = true;
        return rule_response(&rule, false, false, "invalid threshold or operator", 0);
    }
    rule.threshold = (float)threshold->valuedouble;
    cJSON *item = cJSON_GetObjectItem(arguments, "hysteresis");
    if (cJSON_IsNumber(item)) rule.hysteresis = (float)item->valuedouble;
    item = cJSON_GetObjectItem(arguments, "hold_ms");
    if (cJSON_IsNumber(item)) rule.hold_ms = (uint32_t)item->valuedouble;
    item = cJSON_GetObjectItem(arguments, "cooldown_ms");
    if (cJSON_IsNumber(item)) rule.cooldown_ms = (uint32_t)item->valuedouble;
    item = cJSON_GetObjectItem(arguments, "enabled");
    if (cJSON_IsBool(item)) rule.enabled = cJSON_IsTrue(item);
    json_string(arguments, "interlock_device", rule.interlock_device,
                sizeof(rule.interlock_device), false);
    json_string(arguments, "interlock_point", rule.interlock_point,
                sizeof(rule.interlock_point), false);
    item = cJSON_GetObjectItem(arguments, "interlock_required_state");
    rule.interlock_required_state = cJSON_IsTrue(item);

    amm_mapping_entry_t source;
    if (amm_find_mapping_by_point(rule.source_device, rule.source_point, &source) != ESP_OK) {
        *is_error = true;
        return rule_response(&rule, false, false, "source point is not mapped", 0);
    }
    if ((rule.interlock_device[0] == '\0') != (rule.interlock_point[0] == '\0')) {
        *is_error = true;
        return rule_response(&rule, false, false,
                             "interlock device and point must be provided together", 0);
    }
    if (rule.interlock_device[0] != '\0') {
        amm_mapping_entry_t interlock;
        if (amm_find_mapping_by_point(rule.interlock_device, rule.interlock_point,
                                      &interlock) != ESP_OK) {
            *is_error = true;
            return rule_response(&rule, false, false, "interlock point is not mapped", 0);
        }
    }

    if (strcmp(action_name, "write_point") == 0) {
        rule.action = RULE_ACTION_WRITE_POINT;
        if (!json_string(arguments, "target_device", rule.target_device,
                         sizeof(rule.target_device), true) ||
            !json_string(arguments, "target_point", rule.target_point,
                         sizeof(rule.target_point), true)) {
            *is_error = true;
            return rule_response(&rule, false, false, "missing write target", 0);
        }
        item = cJSON_GetObjectItem(arguments, "target_value");
        amm_mapping_entry_t target;
        if (!cJSON_IsNumber(item) ||
            amm_find_mapping_by_point(rule.target_device, rule.target_point,
                                      &target) != ESP_OK) {
            *is_error = true;
            return rule_response(&rule, false, false, "write target is not mapped", 0);
        }
        rule.target_value = (float)item->valuedouble;
        if (!target.constraint.writable ||
            rule.target_value < target.constraint.valid_range_min ||
            rule.target_value > target.constraint.valid_range_max) {
            *is_error = true;
            return rule_response(&rule, false, false,
                                 "write target is read-only or value is outside its safe range", 0);
        }
    } else if (strcmp(action_name, "mqtt_alert") == 0) {
        json_string(arguments, "alert_topic", rule.alert_topic,
                    sizeof(rule.alert_topic), false);
        json_string(arguments, "alert_message", rule.alert_message,
                    sizeof(rule.alert_message), false);
    } else {
        *is_error = true;
        return rule_response(&rule, false, false, "invalid rule action", 0);
    }
    if (rule.hysteresis < 0 || rule.cooldown_ms < 200) {
        *is_error = true;
        return rule_response(&rule, false, false,
                             "hysteresis must be non-negative and cooldown at least 200 ms", 0);
    }

    item = cJSON_GetObjectItem(arguments, "confirmed");
    if (!cJSON_IsTrue(item)) {
        return rule_response(&rule, true, false,
                             "preview only; obtain explicit user confirmation before applying", 0);
    }
    runtime_config_t config;
    runtime_config_get(&config);
    if (!config.mcp_write_enabled) {
        *is_error = true;
        return rule_response(&rule, true, false,
                             "MCP configuration writes are disabled", 0);
    }
    uint32_t id = 0;
    esp_err_t err = automation_upsert_rule(&rule, &id);
    *is_error = err != ESP_OK;
    return rule_response(&rule, true, err == ESP_OK,
                         err == ESP_OK ? "rule saved" : esp_err_to_name(err), id);
}

static cJSON *tool_call(const char *name, cJSON *arguments, bool *is_error)
{
    *is_error = false;
    if (strcmp(name, "list_points") == 0) return call_list_points();
    if (strcmp(name, "read_point") == 0) {
        cJSON *device = cJSON_GetObjectItem(arguments, "device_id");
        cJSON *point = cJSON_GetObjectItem(arguments, "point_id");
        tcm_context_t state;
        if (!cJSON_IsString(device) || !cJSON_IsString(point) ||
            tcm_state_pool_get(device->valuestring, point->valuestring, &state) != ESP_OK) {
            *is_error = true;
            return cJSON_Parse("{\"error\":\"point has no current state\"}");
        }
        char json[TCM_MAX_JSON_LEN];
        if (tcm_serialize_json(&state, json, sizeof(json)) <= 0) return cJSON_CreateObject();
        return cJSON_Parse(json);
    }
    if (strcmp(name, "write_point") == 0) {
        runtime_config_t config;
        runtime_config_get(&config);
        cJSON *device = cJSON_GetObjectItem(arguments, "device_id");
        cJSON *point = cJSON_GetObjectItem(arguments, "point_id");
        cJSON *value = cJSON_GetObjectItem(arguments, "value");
        control_result_t write_result = {0};
        esp_err_t err = !config.mcp_write_enabled ? ESP_ERR_NOT_ALLOWED :
            (!cJSON_IsString(device) || !cJSON_IsString(point) || !cJSON_IsNumber(value)
                ? ESP_ERR_INVALID_ARG
                : control_service_write_point(device->valuestring, point->valuestring,
                                              (float)value->valuedouble,
                                              CONTROL_SOURCE_MCP, &write_result));
        cJSON *response = cJSON_CreateObject();
        cJSON_AddBoolToObject(response, "accepted", err == ESP_OK);
        cJSON_AddStringToObject(response, "reason", !config.mcp_write_enabled
            ? "MCP writes are disabled by gateway configuration"
            : (write_result.reason[0] ? write_result.reason : esp_err_to_name(err)));
        *is_error = err != ESP_OK;
        return response;
    }
    if (strcmp(name, "discover_modbus_devices") == 0) {
        int start = 1, end = 247;
        cJSON *item = cJSON_GetObjectItem(arguments, "slave_start");
        if (cJSON_IsNumber(item)) start = item->valueint;
        item = cJSON_GetObjectItem(arguments, "slave_end");
        if (cJSON_IsNumber(item)) end = item->valueint;
        esp_err_t err = (start < 1 || end > 247 || start > end) ? ESP_ERR_INVALID_ARG
            : modbus_discover_scan_bus((uint8_t)start, (uint8_t)end);
        cJSON *response = cJSON_CreateObject();
        cJSON_AddBoolToObject(response, "started", err == ESP_OK);
        cJSON_AddStringToObject(response, "status", esp_err_to_name(err));
        *is_error = err != ESP_OK;
        return response;
    }
    if (strcmp(name, "get_gateway_config") == 0) {
        runtime_config_t config;
        runtime_config_get(&config);
        cJSON *response = cJSON_CreateObject();
        cJSON_AddStringToObject(response, "gateway_id", config.gateway_id);
        cJSON_AddStringToObject(response, "locale", config.locale == UI_LOCALE_ZH_CN ? "zh-CN" : "en-US");
        cJSON_AddBoolToObject(response, "prefer_ethernet", config.prefer_ethernet);
        cJSON_AddBoolToObject(response, "mqtt_enabled", config.mqtt.enabled);
        cJSON_AddStringToObject(response, "mqtt_uri", config.mqtt.uri);
        cJSON_AddBoolToObject(response, "modbus_rtu_enabled", config.modbus_rtu.enabled);
        cJSON_AddNumberToObject(response, "amm_model_version", amm_get_model_version());
        return response;
    }
    if (strcmp(name, "get_cache_status") == 0) {
        cJSON *response = cJSON_CreateObject();
        cJSON_AddNumberToObject(response, "records", uif_get_cached_count());
        cJSON_AddNumberToObject(response, "usage_percent", uif_get_cache_usage_percent());
        cJSON_AddNumberToObject(response, "data_loss", uif_get_data_loss_count());
        return response;
    }
    if (strcmp(name, "configure_rule_from_natural_language") == 0) {
        return call_configure_rule(arguments, is_error);
    }
    *is_error = true;
    return cJSON_Parse("{\"error\":\"unknown tool\"}");
}

static esp_err_t mcp_post_handler(httpd_req_t *req)
{
    if (require_mcp_authorization(req) != ESP_OK) return ESP_OK;
    cJSON *request = read_body(req);
    if (request == NULL) return send_rpc(req, NULL, NULL, -32700, "Parse error");
    cJSON *id = cJSON_GetObjectItem(request, "id");
    cJSON *method = cJSON_GetObjectItem(request, "method");
    if (!cJSON_IsString(method)) {
        esp_err_t response = send_rpc(req, id, NULL, -32600, "Invalid Request");
        cJSON_Delete(request);
        return response;
    }
    cJSON *result = NULL;
    int error = 0;
    const char *error_message = NULL;
    if (strcmp(method->valuestring, "initialize") == 0) {
        result = cJSON_CreateObject();
        cJSON_AddStringToObject(result, "protocolVersion", "2025-03-26");
        cJSON *capabilities = cJSON_AddObjectToObject(result, "capabilities");
        cJSON_AddObjectToObject(capabilities, "tools");
        cJSON *server = cJSON_AddObjectToObject(result, "serverInfo");
        cJSON_AddStringToObject(server, "name", "esp32s3-industrial-gateway");
        cJSON_AddStringToObject(server, "version", "1.0.0");
    } else if (strcmp(method->valuestring, "tools/list") == 0) {
        result = tools_list_result();
    } else if (strcmp(method->valuestring, "tools/call") == 0) {
        cJSON *params = cJSON_GetObjectItem(request, "params");
        cJSON *name = cJSON_GetObjectItem(params, "name");
        cJSON *arguments = cJSON_GetObjectItem(params, "arguments");
        if (!cJSON_IsString(name)) {
            error = -32602; error_message = "Missing tool name";
        } else {
            if (!cJSON_IsObject(arguments)) arguments = cJSON_CreateObject();
            bool is_error = false;
            cJSON *value = tool_call(name->valuestring, arguments, &is_error);
            result = tool_text_result(value, is_error);
            if (cJSON_GetObjectItem(params, "arguments") == NULL) cJSON_Delete(arguments);
        }
    } else if (strcmp(method->valuestring, "notifications/initialized") == 0) {
        cJSON_Delete(request);
        httpd_resp_set_status(req, "202 Accepted");
        return httpd_resp_send(req, NULL, 0);
    } else {
        error = -32601; error_message = "Method not found";
    }
    esp_err_t response = send_rpc(req, id, result, error, error_message);
    cJSON_Delete(request);
    return response;
}

static esp_err_t mcp_options_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type, MCP-Protocol-Version");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t mcp_status_get_handler(httpd_req_t *req)
{
    runtime_config_t config;
    runtime_config_get(&config);
    int64_t now_ms = esp_timer_get_time() / 1000;
    int64_t lock_remaining_ms = s_auth_locked_until_ms > now_ms
        ? s_auth_locked_until_ms - now_ms : 0;
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "access_enabled",
                          config.security.auth_enabled &&
                          strlen(config.security.password_sha256) == 64);
    cJSON_AddBoolToObject(root, "auth_enabled", config.security.auth_enabled);
    cJSON_AddBoolToObject(root, "token_configured",
                          strlen(config.security.password_sha256) == 64);
    cJSON_AddBoolToObject(root, "write_enabled", config.mcp_write_enabled);
    cJSON_AddStringToObject(root, "endpoint", "/mcp");
    cJSON_AddNumberToObject(root, "authorized_requests", s_authorized_requests);
    cJSON_AddNumberToObject(root, "authentication_failures", s_auth_failure_total);
    cJSON_AddNumberToObject(root, "last_authorized_uptime_ms",
                            (double)s_last_authorized_ms);
    cJSON_AddNumberToObject(root, "lock_remaining_ms",
                            (double)lock_remaining_ms);
    cJSON_AddNumberToObject(root, "tool_count", 7);
    char *text = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (text == NULL) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Unable to serialize MCP status");
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t err = httpd_resp_sendstr(req, text);
    free(text);
    return err;
}

esp_err_t mcp_http_register(httpd_handle_t server)
{
    const httpd_uri_t post = {.uri = "/mcp", .method = HTTP_POST, .handler = mcp_post_handler};
    const httpd_uri_t options = {.uri = "/mcp", .method = HTTP_OPTIONS, .handler = mcp_options_handler};
    const httpd_uri_t status = {
        .uri = "/api/mcp/status", .method = HTTP_GET,
        .handler = mcp_status_get_handler,
    };
    esp_err_t err = httpd_register_uri_handler(server, &post);
    if (err == ESP_OK) err = httpd_register_uri_handler(server, &options);
    return err == ESP_OK ? httpd_register_uri_handler(server, &status) : err;
}
