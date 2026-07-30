#include "mcp/mcp_http.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "amm/amm_mapping.h"
#include "automation/automation_engine.h"
#include "cJSON.h"
#include "config/runtime_config.h"
#include "esp_timer.h"
#include "freertos/portmacro.h"
#include "mbedtls/sha256.h"
#include "mcp/mcp_token_store.h"
#include "modbus/modbus_discover.h"
#include "services/control_service.h"
#include "tcm/tcm_context.h"
#include "tcm/tcm_state_pool.h"
#include "uif/uif_persistence.h"

/* Per-source rate limiting. The original MCP implementation used a single
 * global failure counter with a global lockout: any client that failed N
 * times would lock out *every* MCP client (a trivial denial-of-service). We
 * now track failures per originating IP, so an attacker can only throttle
 * themselves. */
#define MCP_RL_BUCKETS 16
#define MCP_RL_MAX_FAILURES 8
#define MCP_RL_LOCKOUT_MS 30000

typedef struct {
    uint32_t peer;             /* IPv4 host-order, 0 = empty */
    uint8_t failures;
    int64_t lock_until_ms;
} mcp_rl_t;

static mcp_rl_t s_rl[MCP_RL_BUCKETS];
static portMUX_TYPE s_rl_mux = portMUX_INITIALIZER_UNLOCKED;

static uint32_t s_authorized_requests;
static uint32_t s_auth_failure_total;
static int64_t s_last_authorized_ms;

static void sha256_hex(const char *text, char output[65])
{
    uint8_t digest[32];
    mbedtls_sha256((const unsigned char *)text, strlen(text), digest, 0);
    for (int i = 0; i < 32; ++i) {
        snprintf(output + i * 2, 3, "%02x", digest[i]);
    }
    output[64] = '\0';
}

static uint32_t peer_ip_key(httpd_req_t *req)
{
    int fd = httpd_req_to_sockfd(req);
    if (fd < 0) return 0;
    struct sockaddr_in addr;
    socklen_t len = sizeof(addr);
    if (getpeername(fd, (struct sockaddr *)&addr, &len) != 0) return 0;
    if (addr.sin_family != AF_INET) return 0;
    return ntohl(addr.sin_addr.s_addr);
}

/* Returns ESP_OK if the peer is allowed to attempt authentication, or
 * ESP_ERR_TIMEOUT if it is currently locked out (with lock_remaining filled). */
static esp_err_t mcp_rl_check(uint32_t peer, int64_t now_ms, int64_t *lock_remaining)
{
    *lock_remaining = 0;
    portENTER_CRITICAL(&s_rl_mux);
    int idx = -1, empty = -1;
    for (int i = 0; i < MCP_RL_BUCKETS; ++i) {
        if (s_rl[i].peer == peer) { idx = i; break; }
        if (s_rl[i].peer == 0 && empty < 0) empty = i;
    }
    if (idx < 0) idx = (empty >= 0) ? empty : (int)(peer % MCP_RL_BUCKETS);
    if (s_rl[idx].peer != peer) {
        s_rl[idx].peer = peer;
        s_rl[idx].failures = 0;
        s_rl[idx].lock_until_ms = 0;
    }
    esp_err_t r = ESP_OK;
    if (s_rl[idx].lock_until_ms > now_ms) {
        r = ESP_ERR_TIMEOUT;
        *lock_remaining = s_rl[idx].lock_until_ms - now_ms;
    }
    portEXIT_CRITICAL(&s_rl_mux);
    return r;
}

static void mcp_rl_fail(uint32_t peer, int64_t now_ms)
{
    portENTER_CRITICAL(&s_rl_mux);
    int idx = -1, empty = -1;
    for (int i = 0; i < MCP_RL_BUCKETS; ++i) {
        if (s_rl[i].peer == peer) { idx = i; break; }
        if (s_rl[i].peer == 0 && empty < 0) empty = i;
    }
    if (idx < 0) idx = (empty >= 0) ? empty : (int)(peer % MCP_RL_BUCKETS);
    if (s_rl[idx].peer != peer) {
        s_rl[idx].peer = peer;
        s_rl[idx].failures = 0;
        s_rl[idx].lock_until_ms = 0;
    }
    if (++s_rl[idx].failures >= MCP_RL_MAX_FAILURES) {
        s_rl[idx].lock_until_ms = now_ms + MCP_RL_LOCKOUT_MS;
    }
    portEXIT_CRITICAL(&s_rl_mux);
}

static void mcp_rl_ok(uint32_t peer)
{
    portENTER_CRITICAL(&s_rl_mux);
    for (int i = 0; i < MCP_RL_BUCKETS; ++i) {
        if (s_rl[i].peer == peer) {
            s_rl[i].failures = 0;
            s_rl[i].lock_until_ms = 0;
            break;
        }
    }
    portEXIT_CRITICAL(&s_rl_mux);
}

/* Authenticate the bearer token against the dedicated MCP token store. On
 * success fills `out_token` (used by the caller for scope checks) and returns
 * ESP_OK. Rate limiting is per-peer so a single misbehaving client cannot lock
 * out the others. */
static esp_err_t require_mcp_authorization(httpd_req_t *req, mcp_token_t *out_token)
{
    runtime_config_t config;
    runtime_config_get(&config);
    if (!config.security.auth_enabled) {
        httpd_resp_set_status(req, "403 Forbidden");
        httpd_resp_sendstr(req, "MCP access is disabled until authentication is configured");
        return ESP_ERR_NOT_ALLOWED;
    }

    uint32_t peer = peer_ip_key(req);
    int64_t now_ms = esp_timer_get_time() / 1000;
    int64_t lock_remaining = 0;
    if (mcp_rl_check(peer, now_ms, &lock_remaining) != ESP_OK) {
        httpd_resp_set_status(req, "429 Too Many Requests");
        char retry[24];
        snprintf(retry, sizeof(retry), "%lld", (long long)(lock_remaining / 1000 + 1));
        httpd_resp_set_hdr(req, "Retry-After", retry);
        httpd_resp_sendstr(req, "Too many authentication failures from this address");
        return ESP_ERR_NOT_ALLOWED;
    }

    char authorization[192] = {0};
    bool accepted =
        httpd_req_get_hdr_value_str(req, "Authorization", authorization,
                                    sizeof(authorization)) == ESP_OK &&
        strncmp(authorization, "Bearer ", 7) == 0;
    mcp_token_t token;
    if (accepted) {
        char digest[65] = {0};
        sha256_hex(authorization + 7, digest);
        accepted = (mcp_token_authenticate(digest, &token) == ESP_OK);
    }
    if (!accepted) {
        ++s_auth_failure_total;
        mcp_rl_fail(peer, now_ms);
        httpd_resp_set_status(req, "401 Unauthorized");
        httpd_resp_set_hdr(req, "WWW-Authenticate", "Bearer");
        httpd_resp_sendstr(req, "Unauthorized");
        return ESP_ERR_NOT_ALLOWED;
    }
    mcp_rl_ok(peer);
    ++s_authorized_requests;
    s_last_authorized_ms = now_ms;
    if (out_token != NULL) *out_token = token;
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
    /* Two-phase rule configuration: preview always validates and returns a
     * structured preview without persisting; commit re-validates and persists.
     * This removes the implicit "confirmed=true" single-shot write path. */
    const char *rule_props =
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
        "\"enabled\":{\"type\":\"boolean\"}}";
    const char *rule_required =
        "[\"rule_name\",\"source_device\",\"source_point\",\"operator\",\"threshold\",\"action\"]";
    add_tool(tools, "rule_preview",
             "Validate a natural-language rule and return a structured preview "
             "without persisting it. Always call this first and show the result "
             "to the user before rule_commit.",
             rule_props, rule_required);
    add_tool(tools, "rule_commit",
             "Persist a previously previewed rule. Re-validates all numeric "
             "boundaries before writing.",
             rule_props, rule_required);
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

/* Validate and optionally persist a rule. When `commit` is false the function
 * only validates and returns a preview (the rule_preview tool). When true it
 * re-validates and, if the gateway write switch is enabled, persists it
 * (the rule_commit tool). Numeric boundaries are enforced in both paths. */
static cJSON *call_configure_rule(cJSON *arguments, bool commit, bool *is_error)
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
        !isfinite(threshold->valuedouble) ||
        !parse_rule_operator(operator_name, &rule.condition_operator)) {
        *is_error = true;
        return rule_response(&rule, false, false, "invalid threshold or operator", 0);
    }
    rule.threshold = (float)threshold->valuedouble;
    cJSON *item = cJSON_GetObjectItem(arguments, "hysteresis");
    if (cJSON_IsNumber(item)) {
        if (!isfinite(item->valuedouble)) {
            *is_error = true;
            return rule_response(&rule, false, false, "hysteresis must be finite", 0);
        }
        rule.hysteresis = (float)item->valuedouble;
    }
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
        if (!cJSON_IsNumber(item) || !isfinite(item->valuedouble) ||
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

    if (!commit) {
        return rule_response(&rule, true, false,
                             "preview only; call rule_commit to apply", 0);
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

static uint8_t tool_required_scope(const char *name)
{
    if (strcmp(name, "write_point") == 0) return MCP_SCOPE_WRITE;
    if (strcmp(name, "rule_commit") == 0) return MCP_SCOPE_WRITE;
    if (strcmp(name, "discover_modbus_devices") == 0) return MCP_SCOPE_WRITE;
    return MCP_SCOPE_READ;
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
        if (!cJSON_IsString(device) || !cJSON_IsString(point) || !cJSON_IsNumber(value) ||
            !isfinite(value->valuedouble)) {
            *is_error = true;
            cJSON *response = cJSON_CreateObject();
            cJSON_AddBoolToObject(response, "accepted", false);
            cJSON_AddStringToObject(response, "reason", "invalid device/point/value");
            return response;
        }
        control_result_t write_result = {0};
        esp_err_t err = !config.mcp_write_enabled ? ESP_ERR_NOT_ALLOWED :
            control_service_write_point(device->valuestring, point->valuestring,
                                        (float)value->valuedouble,
                                        CONTROL_SOURCE_MCP, &write_result);
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
    if (strcmp(name, "rule_preview") == 0) return call_configure_rule(arguments, false, is_error);
    if (strcmp(name, "rule_commit") == 0) return call_configure_rule(arguments, true, is_error);
    *is_error = true;
    return cJSON_Parse("{\"error\":\"unknown tool\"}");
}

static esp_err_t mcp_post_handler(httpd_req_t *req)
{
    mcp_token_t active_token;
    if (require_mcp_authorization(req, &active_token) != ESP_OK) return ESP_OK;

    cJSON *request = read_body(req);
    if (request == NULL) {
        return send_rpc(req, NULL, NULL, -32700, "Parse error");
    }

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
    bool owned_args = false;
    cJSON *arguments = NULL;

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
        cJSON *name = params != NULL ? cJSON_GetObjectItem(params, "name") : NULL;
        if (!cJSON_IsString(name)) {
            error = -32602;
            error_message = "Missing tool name";
            goto cleanup;
        }
        /* Scope check is performed *before* any work happens. A token without
         * the required scope gets a clean error rather than executing. */
        if (!mcp_token_has_scope(&active_token, tool_required_scope(name->valuestring))) {
            error = -32602;
            error_message = "Token scope is insufficient for this tool";
            goto cleanup;
        }
        arguments = cJSON_GetObjectItem(params, "arguments");
        if (!cJSON_IsObject(arguments)) {
            arguments = cJSON_CreateObject();
            owned_args = true;
        }
        bool is_error = false;
        cJSON *value = tool_call(name->valuestring, arguments, &is_error);
        result = tool_text_result(value, is_error);
    } else if (strcmp(method->valuestring, "notifications/initialized") == 0) {
        cJSON_Delete(request);
        httpd_resp_set_status(req, "202 Accepted");
        return httpd_resp_send(req, NULL, 0);
    } else {
        error = -32601;
        error_message = "Method not found";
    }

cleanup:
    if (owned_args && arguments != NULL) cJSON_Delete(arguments);
    esp_err_t response = send_rpc(req, id, result, error, error_message);
    cJSON_Delete(request);
    return response;
}

static esp_err_t mcp_options_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type, Authorization, MCP-Protocol-Version");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t mcp_status_get_handler(httpd_req_t *req)
{
    runtime_config_t config;
    runtime_config_get(&config);
    cJSON *root = cJSON_CreateObject();
    size_t tokens = mcp_token_count();
    cJSON_AddBoolToObject(root, "access_enabled", config.security.auth_enabled && tokens > 0);
    cJSON_AddBoolToObject(root, "auth_enabled", config.security.auth_enabled);
    cJSON_AddNumberToObject(root, "token_count", (double)tokens);
    cJSON_AddBoolToObject(root, "token_configured", tokens > 0);
    cJSON_AddBoolToObject(root, "write_enabled", config.mcp_write_enabled);
    cJSON_AddStringToObject(root, "endpoint", "/mcp");
    cJSON_AddNumberToObject(root, "authorized_requests", s_authorized_requests);
    cJSON_AddNumberToObject(root, "authentication_failures", s_auth_failure_total);
    cJSON_AddNumberToObject(root, "last_authorized_uptime_ms",
                            (double)s_last_authorized_ms);
    cJSON_AddNumberToObject(root, "tool_count", 8);
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
    mcp_token_store_init();
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
