#ifndef AUTOMATION_ENGINE_H
#define AUTOMATION_ENGINE_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "gateway_config.h"

#define AUTOMATION_MAX_RULES 16

typedef enum {
    RULE_OP_GT = 0, RULE_OP_GTE, RULE_OP_LT, RULE_OP_LTE, RULE_OP_EQ, RULE_OP_NEQ
} automation_operator_t;

typedef enum {
    RULE_ACTION_WRITE_POINT = 0,
    RULE_ACTION_MQTT_ALERT,
} automation_action_t;

typedef struct {
    uint32_t id;
    bool enabled;
    char name[48];
    char source_device[AMM_MAX_DEVICE_NAME_LEN];
    char source_point[AMM_MAX_POINT_NAME_LEN];
    automation_operator_t condition_operator;
    float threshold;
    float hysteresis;
    uint32_t hold_ms;
    uint32_t cooldown_ms;
    automation_action_t action;
    char target_device[AMM_MAX_DEVICE_NAME_LEN];
    char target_point[AMM_MAX_POINT_NAME_LEN];
    float target_value;
    char alert_topic[96];
    char alert_message[128];
} automation_rule_t;

typedef struct {
    uint32_t evaluations;
    uint32_t triggers;
    uint32_t failures;
} automation_stats_t;

esp_err_t automation_init(void);
esp_err_t automation_start(void);
int automation_get_rules(automation_rule_t *out, int max_rules);
esp_err_t automation_upsert_rule(const automation_rule_t *rule, uint32_t *assigned_id);
esp_err_t automation_delete_rule(uint32_t id);
automation_stats_t automation_get_stats(void);

#endif
