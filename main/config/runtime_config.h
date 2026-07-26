#ifndef RUNTIME_CONFIG_H
#define RUNTIME_CONFIG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#define RUNTIME_CONFIG_SCHEMA_VERSION 2
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
    bool clean_session;
    bool retain;
    bool lwt_enabled;
    char lwt_topic[96];
    char lwt_payload[64];
    uint8_t lwt_qos;
    bool lwt_retain;
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
    bool enabled;
    char server1[64];
    char server2[64];
    char timezone[48];
    uint32_t sync_interval_ms;
} runtime_time_config_t;

typedef struct {
    bool auth_enabled;
    char username[32];
    char password_sha256[65];
    bool ota_enabled;
    bool ota_allow_http;
} runtime_security_config_t;

typedef struct {
    bool enabled;
    uint16_t port;
    uint8_t max_clients;
    uint16_t response_timeout_ms;
} runtime_modbus_tcp_server_config_t;

typedef struct {
    bool sparkplug_enabled;
    char sparkplug_group[32];
    char sparkplug_node[32];
    char sparkplug_device[32];
} runtime_northbound_config_t;

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
    runtime_time_config_t time;
    runtime_security_config_t security;
    runtime_modbus_tcp_server_config_t modbus_tcp_server;
    runtime_northbound_config_t northbound;
} runtime_config_t;

esp_err_t runtime_config_init(void);
esp_err_t runtime_config_validate(const runtime_config_t *config,
                                  char *reason, size_t reason_size);
void runtime_config_get(runtime_config_t *out);
ui_locale_t runtime_config_get_locale(void);
esp_err_t runtime_config_set(const runtime_config_t *config);
esp_err_t runtime_config_reset(void);

#endif
