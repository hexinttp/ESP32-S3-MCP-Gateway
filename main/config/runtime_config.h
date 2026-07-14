#ifndef RUNTIME_CONFIG_H
#define RUNTIME_CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define RUNTIME_CONFIG_SCHEMA_VERSION 1
#define RUNTIME_MAX_TCP_ENDPOINTS 8

typedef enum {
    UI_LOCALE_ZH_CN = 0,
    UI_LOCALE_EN_US = 1,
} ui_locale_t;

typedef struct {
    bool enabled;
    char ssid[33];
    char password[65];
} runtime_wifi_config_t;

typedef struct {
    bool enabled;
    char uri[128];
    char client_id[48];
    char username[64];
    char password[96];
    char data_prefix[64];
    char command_prefix[64];
    uint16_t keepalive_sec;
    uint8_t qos;
} runtime_mqtt_config_t;

typedef struct {
    bool enabled;
    uint32_t baud_rate;
    uint8_t parity;
    uint16_t timeout_ms;
} runtime_modbus_rtu_config_t;

typedef struct {
    bool enabled;
    uint8_t endpoint_id;
    char name[32];
    char host[64];
    uint16_t port;
    uint16_t timeout_ms;
} runtime_modbus_tcp_endpoint_t;

typedef struct {
    uint32_t schema_version;
    char gateway_id[48];
    ui_locale_t locale;
    bool prefer_ethernet;
    bool lcd_enabled;
    bool tf_enabled;
    bool mcp_write_enabled;
    runtime_wifi_config_t wifi;
    runtime_mqtt_config_t mqtt;
    runtime_modbus_rtu_config_t modbus_rtu;
    runtime_modbus_tcp_endpoint_t tcp_endpoints[RUNTIME_MAX_TCP_ENDPOINTS];
    uint8_t tcp_endpoint_count;
} runtime_config_t;

esp_err_t runtime_config_init(void);
void runtime_config_get(runtime_config_t *out);
ui_locale_t runtime_config_get_locale(void);
esp_err_t runtime_config_set(const runtime_config_t *config);
esp_err_t runtime_config_reset(void);

#endif
