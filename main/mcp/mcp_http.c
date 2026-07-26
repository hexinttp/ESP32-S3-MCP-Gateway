#include "mcp/mcp_http.h"

#include <stdlib.h>
#include <string.h>
#include "amm/amm_mapping.h"
#include "cJSON.h"
#include "config/runtime_config.h"
#include "modbus/modbus_discover.h"
#include "services/control_service.h"
#include "tcm/tcm_context.h"
#include "tcm/tcm_state_pool.h"
#include "uif/uif_persistence.h"

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
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
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
    *is_error = true;
    return cJSON_Parse("{\"error\":\"unknown tool\"}");
}

static esp_err_t mcp_post_handler(httpd_req_t *req)
{
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
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type, MCP-Protocol-Version");
    return httpd_resp_send(req, NULL, 0);
}

esp_err_t mcp_http_register(httpd_handle_t server)
{
    const httpd_uri_t post = {.uri = "/mcp", .method = HTTP_POST, .handler = mcp_post_handler};
    const httpd_uri_t options = {.uri = "/mcp", .method = HTTP_OPTIONS, .handler = mcp_options_handler};
    esp_err_t err = httpd_register_uri_handler(server, &post);
    return err == ESP_OK ? httpd_register_uri_handler(server, &options) : err;
}
