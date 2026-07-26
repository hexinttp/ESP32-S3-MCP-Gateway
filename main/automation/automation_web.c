#include "automation/automation_web.h"

#include <stdlib.h>
#include <string.h>
#include "automation/automation_engine.h"
#include "cJSON.h"

static cJSON *read_json(httpd_req_t *req)
{
    if (req->content_len <= 0 || req->content_len > 4096) return NULL;
    char *body = malloc((size_t)req->content_len + 1);
    if (body == NULL) return NULL;
    int offset = 0;
    while (offset < req->content_len) {
        int length = httpd_req_recv(req, body + offset, req->content_len - offset);
        if (length <= 0) { free(body); return NULL; }
        offset += length;
    }
    body[offset] = '\0';
    cJSON *json = cJSON_Parse(body);
    free(body);
    return json;
}

static esp_err_t send_json(httpd_req_t *req, cJSON *json)
{
    char *text = cJSON_PrintUnformatted(json);
    cJSON_Delete(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    esp_err_t err = httpd_resp_sendstr(req, text);
    free(text);
    return err;
}

static void add_rule_json(cJSON *array, const automation_rule_t *rule)
{
    cJSON *item = cJSON_CreateObject();
    cJSON_AddNumberToObject(item, "id", rule->id);
    cJSON_AddBoolToObject(item, "enabled", rule->enabled);
    cJSON_AddStringToObject(item, "name", rule->name);
    cJSON_AddStringToObject(item, "source_device", rule->source_device);
    cJSON_AddStringToObject(item, "source_point", rule->source_point);
    cJSON_AddNumberToObject(item, "operator", rule->condition_operator);
    cJSON_AddNumberToObject(item, "threshold", rule->threshold);
    cJSON_AddNumberToObject(item, "hysteresis", rule->hysteresis);
    cJSON_AddNumberToObject(item, "hold_ms", rule->hold_ms);
    cJSON_AddNumberToObject(item, "cooldown_ms", rule->cooldown_ms);
    cJSON_AddStringToObject(item, "interlock_device", rule->interlock_device);
    cJSON_AddStringToObject(item, "interlock_point", rule->interlock_point);
    cJSON_AddBoolToObject(item, "interlock_required_state",
                          rule->interlock_required_state);
    cJSON_AddNumberToObject(item, "action", rule->action);
    cJSON_AddStringToObject(item, "target_device", rule->target_device);
    cJSON_AddStringToObject(item, "target_point", rule->target_point);
    cJSON_AddNumberToObject(item, "target_value", rule->target_value);
    cJSON_AddStringToObject(item, "alert_topic", rule->alert_topic);
    cJSON_AddStringToObject(item, "alert_message", rule->alert_message);
    cJSON_AddItemToArray(array, item);
}

static esp_err_t rules_get(httpd_req_t *req)
{
    automation_rule_t *rules = calloc(AUTOMATION_MAX_RULES, sizeof(*rules));
    if (rules == NULL) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
    int count = automation_get_rules(rules, AUTOMATION_MAX_RULES);
    automation_stats_t stats = automation_get_stats();
    cJSON *root = cJSON_CreateObject();
    cJSON *array = cJSON_AddArrayToObject(root, "rules");
    for (int i = 0; i < count; ++i) add_rule_json(array, &rules[i]);
    cJSON *stats_json = cJSON_AddObjectToObject(root, "stats");
    cJSON_AddNumberToObject(stats_json, "evaluations", stats.evaluations);
    cJSON_AddNumberToObject(stats_json, "triggers", stats.triggers);
    cJSON_AddNumberToObject(stats_json, "failures", stats.failures);
    free(rules);
    return send_json(req, root);
}

static void copy_string(cJSON *root, const char *key, char *out, size_t size)
{
    cJSON *value = cJSON_GetObjectItem(root, key);
    if (cJSON_IsString(value)) strlcpy(out, value->valuestring, size);
}

static double number(cJSON *root, const char *key, double fallback)
{
    cJSON *value = cJSON_GetObjectItem(root, key);
    return cJSON_IsNumber(value) ? value->valuedouble : fallback;
}

static esp_err_t rules_post(httpd_req_t *req)
{
    cJSON *root = read_json(req);
    if (root == NULL) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    automation_rule_t rule = {0};
    rule.id = (uint32_t)number(root, "id", 0);
    cJSON *enabled = cJSON_GetObjectItem(root, "enabled");
    rule.enabled = !cJSON_IsBool(enabled) || cJSON_IsTrue(enabled);
    copy_string(root, "name", rule.name, sizeof(rule.name));
    copy_string(root, "source_device", rule.source_device, sizeof(rule.source_device));
    copy_string(root, "source_point", rule.source_point, sizeof(rule.source_point));
    rule.condition_operator = (automation_operator_t)(int)number(root, "operator", RULE_OP_GT);
    rule.threshold = (float)number(root, "threshold", 0);
    rule.hysteresis = (float)number(root, "hysteresis", 0);
    rule.hold_ms = (uint32_t)number(root, "hold_ms", 0);
    rule.cooldown_ms = (uint32_t)number(root, "cooldown_ms", 1000);
    copy_string(root, "interlock_device", rule.interlock_device,
                sizeof(rule.interlock_device));
    copy_string(root, "interlock_point", rule.interlock_point,
                sizeof(rule.interlock_point));
    cJSON *interlock_state = cJSON_GetObjectItem(root, "interlock_required_state");
    rule.interlock_required_state = cJSON_IsTrue(interlock_state);
    rule.action = (automation_action_t)(int)number(root, "action", RULE_ACTION_MQTT_ALERT);
    copy_string(root, "target_device", rule.target_device, sizeof(rule.target_device));
    copy_string(root, "target_point", rule.target_point, sizeof(rule.target_point));
    rule.target_value = (float)number(root, "target_value", 0);
    copy_string(root, "alert_topic", rule.alert_topic, sizeof(rule.alert_topic));
    copy_string(root, "alert_message", rule.alert_message, sizeof(rule.alert_message));
    cJSON_Delete(root);
    uint32_t id = 0;
    esp_err_t err = automation_upsert_rule(&rule, &id);
    if (err != ESP_OK) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, esp_err_to_name(err));
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "ok");
    cJSON_AddNumberToObject(response, "id", id);
    return send_json(req, response);
}

static esp_err_t audit_get(httpd_req_t *req)
{
    automation_audit_event_t events[AUTOMATION_AUDIT_CAPACITY];
    int count = automation_get_audit(events, AUTOMATION_AUDIT_CAPACITY);
    cJSON *root = cJSON_CreateArray();
    for (int i = 0; i < count; ++i) {
        cJSON *item = cJSON_CreateObject();
        cJSON_AddNumberToObject(item, "timestamp_ms",
                                (double)events[i].timestamp_ms);
        cJSON_AddNumberToObject(item, "rule_id", events[i].rule_id);
        cJSON_AddBoolToObject(item, "success", events[i].success);
        cJSON_AddNumberToObject(item, "source_value", events[i].source_value);
        cJSON_AddStringToObject(item, "action", events[i].action);
        cJSON_AddStringToObject(item, "detail", events[i].detail);
        cJSON_AddItemToArray(root, item);
    }
    return send_json(req, root);
}

static esp_err_t rules_delete(httpd_req_t *req)
{
    const char *slash = strrchr(req->uri, '/');
    uint32_t id = slash == NULL ? 0 : (uint32_t)strtoul(slash + 1, NULL, 10);
    esp_err_t err = id == 0 ? ESP_ERR_INVALID_ARG : automation_delete_rule(id);
    if (err != ESP_OK) return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Rule not found");
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "ok");
    return send_json(req, response);
}

esp_err_t automation_web_register(httpd_handle_t server)
{
    const httpd_uri_t get = {.uri = "/api/automation/rules", .method = HTTP_GET, .handler = rules_get};
    const httpd_uri_t post = {.uri = "/api/automation/rules", .method = HTTP_POST, .handler = rules_post};
    const httpd_uri_t del = {.uri = "/api/automation/rules/*", .method = HTTP_DELETE, .handler = rules_delete};
    const httpd_uri_t audit = {.uri = "/api/automation/audit", .method = HTTP_GET, .handler = audit_get};
    esp_err_t err = httpd_register_uri_handler(server, &get);
    if (err == ESP_OK) err = httpd_register_uri_handler(server, &post);
    if (err == ESP_OK) err = httpd_register_uri_handler(server, &del);
    return err == ESP_OK ? httpd_register_uri_handler(server, &audit) : err;
}
