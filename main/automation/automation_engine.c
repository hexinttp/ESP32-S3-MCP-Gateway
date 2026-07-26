#include "automation/automation_engine.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "config/runtime_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mqtt_comm/mqtt_handler.h"
#include "nvs.h"
#include "services/control_service.h"
#include "tcm/tcm_state_pool.h"
#include "esp_log.h"
#include "esp_timer.h"

#define RULES_MAGIC 0x41554D31U
#define RULES_NVS_NAMESPACE "automation"
#define RULES_NVS_KEY "rules_v1"

typedef struct {
    uint32_t magic;
    uint32_t next_id;
    uint16_t count;
    automation_rule_t rules[AUTOMATION_MAX_RULES];
} persisted_rules_t;

static const char *TAG = "AUTOMATION";
static automation_rule_t s_rules[AUTOMATION_MAX_RULES];
static int s_count;
static uint32_t s_next_id = 1;
static int64_t s_condition_since[AUTOMATION_MAX_RULES];
static int64_t s_last_trigger[AUTOMATION_MAX_RULES];
static bool s_condition_active[AUTOMATION_MAX_RULES];
static automation_stats_t s_stats;
static automation_audit_event_t s_audit[AUTOMATION_AUDIT_CAPACITY];
static uint16_t s_audit_head;
static uint16_t s_audit_count;
static SemaphoreHandle_t s_mutex;
static TaskHandle_t s_task;

static esp_err_t save_locked(void)
{
    size_t persisted_size = offsetof(persisted_rules_t, rules) +
                            (size_t)s_count * sizeof(automation_rule_t);
    persisted_rules_t *persisted = calloc(1, persisted_size);
    if (persisted == NULL) return ESP_ERR_NO_MEM;
    persisted->magic = RULES_MAGIC;
    persisted->next_id = s_next_id;
    persisted->count = (uint16_t)s_count;
    memcpy(persisted->rules, s_rules, (size_t)s_count * sizeof(automation_rule_t));
    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(RULES_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) { free(persisted); return err; }
    err = nvs_set_blob(nvs, RULES_NVS_KEY, persisted, persisted_size);
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    free(persisted);
    return err;
}

esp_err_t automation_init(void)
{
    if (s_mutex == NULL) s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) return ESP_ERR_NO_MEM;
    size_t size = 0;
    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(RULES_NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err == ESP_OK) {
        err = nvs_get_blob(nvs, RULES_NVS_KEY, NULL, &size);
    }
    persisted_rules_t *persisted = NULL;
    if (err == ESP_OK && size >= offsetof(persisted_rules_t, rules) &&
        size <= sizeof(persisted_rules_t)) {
        persisted = calloc(1, size);
        if (persisted == NULL) err = ESP_ERR_NO_MEM;
        else err = nvs_get_blob(nvs, RULES_NVS_KEY, persisted, &size);
    }
    if (nvs != 0) {
        nvs_close(nvs);
    }
    if (err == ESP_OK && persisted == NULL) err = ESP_ERR_INVALID_SIZE;
    size_t expected_size = persisted == NULL ? 0 : offsetof(persisted_rules_t, rules) +
                           (size_t)persisted->count * sizeof(automation_rule_t);
    if (err == ESP_OK && persisted->magic == RULES_MAGIC &&
        persisted->count <= AUTOMATION_MAX_RULES && size == expected_size) {
        s_count = persisted->count;
        s_next_id = persisted->next_id == 0 ? 1 : persisted->next_id;
        memcpy(s_rules, persisted->rules, (size_t)s_count * sizeof(automation_rule_t));
    }
    free(persisted);
    ESP_LOGI(TAG, "Loaded %d offline automation rules", s_count);
    return ESP_OK;
}

int automation_get_rules(automation_rule_t *out, int max_rules)
{
    if (out == NULL || max_rules <= 0 || s_mutex == NULL) return 0;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int count = s_count < max_rules ? s_count : max_rules;
    memcpy(out, s_rules, count * sizeof(*out));
    xSemaphoreGive(s_mutex);
    return count;
}

esp_err_t automation_upsert_rule(const automation_rule_t *rule, uint32_t *assigned_id)
{
    if (rule == NULL || rule->source_device[0] == '\0' || rule->source_point[0] == '\0' ||
        rule->condition_operator > RULE_OP_NEQ || rule->action > RULE_ACTION_MQTT_ALERT) {
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int slot = -1;
    if (rule->id != 0) {
        for (int i = 0; i < s_count; ++i) if (s_rules[i].id == rule->id) { slot = i; break; }
    }
    if (slot < 0) {
        if (s_count >= AUTOMATION_MAX_RULES) { xSemaphoreGive(s_mutex); return ESP_ERR_NO_MEM; }
        slot = s_count++;
    }
    s_rules[slot] = *rule;
    if (s_rules[slot].id == 0) s_rules[slot].id = s_next_id++;
    if (s_rules[slot].cooldown_ms == 0) s_rules[slot].cooldown_ms = 1000;
    s_condition_since[slot] = 0;
    s_last_trigger[slot] = 0;
    esp_err_t err = save_locked();
    if (assigned_id != NULL) *assigned_id = s_rules[slot].id;
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t automation_delete_rule(uint32_t id)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < s_count; ++i) {
        if (s_rules[i].id == id) {
            for (int j = i; j + 1 < s_count; ++j) s_rules[j] = s_rules[j + 1];
            --s_count;
            memset(&s_rules[s_count], 0, sizeof(s_rules[s_count]));
            memset(s_condition_since, 0, sizeof(s_condition_since));
            memset(s_last_trigger, 0, sizeof(s_last_trigger));
            esp_err_t err = save_locked();
            xSemaphoreGive(s_mutex);
            return err;
        }
    }
    xSemaphoreGive(s_mutex);
    return ESP_ERR_NOT_FOUND;
}

static bool condition_matches(const automation_rule_t *rule, float value,
                              bool was_active)
{
    float hysteresis = fmaxf(rule->hysteresis, 0.0f);
    switch (rule->condition_operator) {
    case RULE_OP_GT:
        return was_active ? value > rule->threshold - hysteresis
                          : value > rule->threshold;
    case RULE_OP_GTE:
        return was_active ? value >= rule->threshold - hysteresis
                          : value >= rule->threshold;
    case RULE_OP_LT:
        return was_active ? value < rule->threshold + hysteresis
                          : value < rule->threshold;
    case RULE_OP_LTE:
        return was_active ? value <= rule->threshold + hysteresis
                          : value <= rule->threshold;
    case RULE_OP_EQ: return fabsf(value - rule->threshold) <= fmaxf(rule->hysteresis, 0.0001f);
    case RULE_OP_NEQ: return fabsf(value - rule->threshold) > fmaxf(rule->hysteresis, 0.0001f);
    default: return false;
    }
}

static void audit_event(const automation_rule_t *rule, const tcm_context_t *state,
                        bool success, const char *detail)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    automation_audit_event_t *event = &s_audit[s_audit_head];
    memset(event, 0, sizeof(*event));
    event->timestamp_ms = state->timestamp_ms;
    event->rule_id = rule->id;
    event->success = success;
    event->source_value = (float)state->value;
    strlcpy(event->action,
            rule->action == RULE_ACTION_WRITE_POINT ? "write_point" : "mqtt_alert",
            sizeof(event->action));
    strlcpy(event->detail, detail, sizeof(event->detail));
    s_audit_head = (s_audit_head + 1U) % AUTOMATION_AUDIT_CAPACITY;
    if (s_audit_count < AUTOMATION_AUDIT_CAPACITY) ++s_audit_count;
    xSemaphoreGive(s_mutex);
}

static bool interlock_allows(const automation_rule_t *rule)
{
    if (rule->interlock_device[0] == '\0' || rule->interlock_point[0] == '\0') {
        return true;
    }
    tcm_context_t state;
    return tcm_state_pool_get(rule->interlock_device, rule->interlock_point,
                              &state) == ESP_OK &&
           (state.value != 0.0) == rule->interlock_required_state &&
           state.quality_state == QUALITY_GOOD;
}

static esp_err_t execute_rule(const automation_rule_t *rule, const tcm_context_t *state)
{
    if (rule->action == RULE_ACTION_WRITE_POINT) {
        control_result_t result;
        return control_service_write_point(rule->target_device, rule->target_point,
                                           rule->target_value,
                                           CONTROL_SOURCE_AUTOMATION, &result);
    }
    runtime_config_t config;
    runtime_config_get(&config);
    char topic[128];
    if (rule->alert_topic[0] != '\0') strlcpy(topic, rule->alert_topic, sizeof(topic));
    else snprintf(topic, sizeof(topic), "%salerts", config.mqtt.data_prefix);
    char payload[512];
    snprintf(payload, sizeof(payload),
             "{\"rule_id\":%lu,\"rule\":\"%s\",\"device_id\":\"%s\","
             "\"point_id\":\"%s\",\"value\":%.6g,\"message\":\"%s\"}",
             (unsigned long)rule->id, rule->name, state->device_id, state->point_id,
             state->value, rule->alert_message);
    return mqtt_publish(topic, payload, config.mqtt.qos);
}

static void automation_task(void *argument)
{
    (void)argument;
    automation_rule_t *rules = calloc(AUTOMATION_MAX_RULES, sizeof(*rules));
    if (rules == NULL) {
        ESP_LOGE(TAG, "Unable to allocate rule snapshot");
        s_task = NULL;
        vTaskDelete(NULL);
        return;
    }
    while (true) {
        int count = automation_get_rules(rules, AUTOMATION_MAX_RULES);
        int64_t now = esp_timer_get_time() / 1000;
        for (int i = 0; i < count; ++i) {
            if (!rules[i].enabled) continue;
            tcm_context_t state;
            if (tcm_state_pool_get(rules[i].source_device, rules[i].source_point, &state) != ESP_OK) continue;
            ++s_stats.evaluations;
            bool matched = condition_matches(&rules[i], (float)state.value,
                                             s_condition_active[i]);
            s_condition_active[i] = matched;
            if (!matched) {
                s_condition_since[i] = 0;
                continue;
            }
            if (s_condition_since[i] == 0) s_condition_since[i] = now;
            if (now - s_condition_since[i] < rules[i].hold_ms ||
                now - s_last_trigger[i] < rules[i].cooldown_ms) continue;
            if (!interlock_allows(&rules[i])) {
                ++s_stats.failures;
                audit_event(&rules[i], &state, false, "interlock blocked action");
                s_last_trigger[i] = now;
                continue;
            }
            if (execute_rule(&rules[i], &state) == ESP_OK) {
                ++s_stats.triggers;
                audit_event(&rules[i], &state, true, "action completed");
            } else {
                ++s_stats.failures;
                audit_event(&rules[i], &state, false, "action failed");
            }
            s_last_trigger[i] = now;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

esp_err_t automation_start(void)
{
    if (s_task != NULL) return ESP_OK;
    return xTaskCreate(automation_task, "automation", 4096, NULL, 4, &s_task) == pdPASS
        ? ESP_OK : ESP_ERR_NO_MEM;
}

automation_stats_t automation_get_stats(void)
{
    return s_stats;
}

int automation_get_audit(automation_audit_event_t *out, int max_events)
{
    if (out == NULL || max_events <= 0) return 0;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int count = s_audit_count < max_events ? s_audit_count : max_events;
    uint16_t start = (uint16_t)((s_audit_head + AUTOMATION_AUDIT_CAPACITY -
                                 s_audit_count) % AUTOMATION_AUDIT_CAPACITY);
    for (int i = 0; i < count; ++i) {
        out[i] = s_audit[(start + i) % AUTOMATION_AUDIT_CAPACITY];
    }
    xSemaphoreGive(s_mutex);
    return count;
}
