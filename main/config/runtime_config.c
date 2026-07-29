#include "runtime_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_check.h"
#include "gateway_config.h"

#define CONFIG_NVS_NAMESPACE "gateway"
#define CONFIG_NVS_KEY_LEGACY "runtime"
#define CONFIG_NVS_KEY_A "runtime_a"
#define CONFIG_NVS_KEY_B "runtime_b"
#define CONFIG_NVS_KEY_ACTIVE "runtime_active"

typedef struct {
    uint32_t magic;
    uint32_t generation;
    uint32_t crc32;
    runtime_config_t config;
} stored_config_t;

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
} runtime_mqtt_config_v1_t;

typedef struct {
    uint32_t schema_version;
    char gateway_id[48];
    ui_locale_t locale;
    bool prefer_ethernet;
    bool lcd_enabled;
    bool tf_enabled;
    bool mcp_write_enabled;
    runtime_wifi_config_t wifi;
    runtime_mqtt_config_v1_t mqtt;
    runtime_modbus_rtu_config_t modbus_rtu;
    runtime_modbus_tcp_endpoint_t tcp_endpoints[RUNTIME_MAX_TCP_ENDPOINTS];
    uint8_t tcp_endpoint_count;
} runtime_config_v1_t;

typedef struct {
    uint32_t magic;
    uint32_t crc32;
    runtime_config_v1_t config;
} stored_config_v1_t;

/* v2 on-disk layout (schema v2, before platform_type was added to the MQTT
   config). Used to migrate pre-platform configurations to CUSTOM mode. */
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
} runtime_mqtt_config_v2_t;

typedef struct {
    uint32_t schema_version;
    char gateway_id[48];
    ui_locale_t locale;
    bool prefer_ethernet;
    bool lcd_enabled;
    bool tf_enabled;
    bool mcp_write_enabled;
    runtime_wifi_config_t wifi;
    runtime_mqtt_config_v2_t mqtt;
    runtime_modbus_rtu_config_t modbus_rtu;
    runtime_modbus_tcp_endpoint_t tcp_endpoints[RUNTIME_MAX_TCP_ENDPOINTS];
    uint8_t tcp_endpoint_count;
    runtime_time_config_t time;
    runtime_security_config_t security;
    runtime_modbus_tcp_server_config_t modbus_tcp_server;
    runtime_northbound_config_t northbound;
} runtime_config_v2_t;

typedef struct {
    uint32_t magic;
    uint32_t generation;
    uint32_t crc32;
    runtime_config_v2_t config;
} stored_config_v2_t;

static void migrate_v2_to_v3(const runtime_config_v2_t *v2, runtime_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->schema_version = RUNTIME_CONFIG_SCHEMA_VERSION;
    strlcpy(cfg->gateway_id, v2->gateway_id, sizeof(cfg->gateway_id));
    cfg->locale = v2->locale;
    cfg->prefer_ethernet = v2->prefer_ethernet;
    cfg->lcd_enabled = v2->lcd_enabled;
    cfg->tf_enabled = v2->tf_enabled;
    cfg->mcp_write_enabled = v2->mcp_write_enabled;
    cfg->wifi = v2->wifi;
    /* Old configurations default to generic custom MQTT mode. */
    cfg->mqtt.enabled = v2->mqtt.enabled;
    cfg->mqtt.platform_type = MQTT_PLATFORM_CUSTOM;
    strlcpy(cfg->mqtt.uri, v2->mqtt.uri, sizeof(cfg->mqtt.uri));
    strlcpy(cfg->mqtt.client_id, v2->mqtt.client_id, sizeof(cfg->mqtt.client_id));
    strlcpy(cfg->mqtt.username, v2->mqtt.username, sizeof(cfg->mqtt.username));
    strlcpy(cfg->mqtt.password, v2->mqtt.password, sizeof(cfg->mqtt.password));
    strlcpy(cfg->mqtt.data_prefix, v2->mqtt.data_prefix, sizeof(cfg->mqtt.data_prefix));
    strlcpy(cfg->mqtt.command_prefix, v2->mqtt.command_prefix, sizeof(cfg->mqtt.command_prefix));
    cfg->mqtt.keepalive_sec = v2->mqtt.keepalive_sec;
    cfg->mqtt.qos = v2->mqtt.qos;
    cfg->mqtt.clean_session = v2->mqtt.clean_session;
    cfg->mqtt.retain = v2->mqtt.retain;
    cfg->mqtt.lwt_enabled = v2->mqtt.lwt_enabled;
    strlcpy(cfg->mqtt.lwt_topic, v2->mqtt.lwt_topic, sizeof(cfg->mqtt.lwt_topic));
    strlcpy(cfg->mqtt.lwt_payload, v2->mqtt.lwt_payload, sizeof(cfg->mqtt.lwt_payload));
    cfg->mqtt.lwt_qos = v2->mqtt.lwt_qos;
    cfg->mqtt.lwt_retain = v2->mqtt.lwt_retain;
    cfg->modbus_rtu = v2->modbus_rtu;
    cfg->tcp_endpoint_count = v2->tcp_endpoint_count;
    memcpy(cfg->tcp_endpoints, v2->tcp_endpoints, sizeof(cfg->tcp_endpoints));
    cfg->time = v2->time;
    cfg->security = v2->security;
    cfg->modbus_tcp_server = v2->modbus_tcp_server;
    cfg->northbound = v2->northbound;
}

/* Forward declaration (defined later in this file). */
static void load_defaults(runtime_config_t *cfg);

/* ---- v3 schema that predates the report_mode field (mqtt_report_mode_t) ---- */
typedef struct {
    bool enabled;
    mqtt_platform_type_t platform_type;
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
} runtime_mqtt_config_v3_t;

typedef struct {
    uint32_t schema_version;
    char gateway_id[48];
    ui_locale_t locale;
    bool prefer_ethernet;
    bool lcd_enabled;
    bool tf_enabled;
    bool mcp_write_enabled;
    runtime_wifi_config_t wifi;
    runtime_mqtt_config_v3_t mqtt;
    runtime_modbus_rtu_config_t modbus_rtu;
    runtime_modbus_tcp_endpoint_t tcp_endpoints[RUNTIME_MAX_TCP_ENDPOINTS];
    uint8_t tcp_endpoint_count;
    runtime_time_config_t time;
    runtime_security_config_t security;
    runtime_modbus_tcp_server_config_t modbus_tcp_server;
    runtime_northbound_config_t northbound;
} runtime_config_v3_t;

typedef struct {
    uint32_t magic;
    uint32_t generation;
    uint32_t crc32;
    runtime_config_v3_t config;
} stored_config_v3_t;

/* Migrate from the v3 schema that predates the report_mode field.
 * Preserves every existing setting (WiFi, MQTT credentials, Modbus, ...) and
 * defaults report_mode to sub-device mode (historic behaviour). */
static void migrate_v3_to_current(const runtime_config_v3_t *v3, runtime_config_t *cfg)
{
    load_defaults(cfg);
    cfg->schema_version = RUNTIME_CONFIG_SCHEMA_VERSION;
    strlcpy(cfg->gateway_id, v3->gateway_id, sizeof(cfg->gateway_id));
    cfg->locale = v3->locale;
    cfg->prefer_ethernet = v3->prefer_ethernet;
    cfg->lcd_enabled = v3->lcd_enabled;
    cfg->tf_enabled = v3->tf_enabled;
    cfg->mcp_write_enabled = v3->mcp_write_enabled;
    cfg->wifi = v3->wifi;
    cfg->mqtt.enabled = v3->mqtt.enabled;
    cfg->mqtt.platform_type = v3->mqtt.platform_type;
    strlcpy(cfg->mqtt.uri, v3->mqtt.uri, sizeof(cfg->mqtt.uri));
    strlcpy(cfg->mqtt.client_id, v3->mqtt.client_id, sizeof(cfg->mqtt.client_id));
    strlcpy(cfg->mqtt.username, v3->mqtt.username, sizeof(cfg->mqtt.username));
    strlcpy(cfg->mqtt.password, v3->mqtt.password, sizeof(cfg->mqtt.password));
    strlcpy(cfg->mqtt.data_prefix, v3->mqtt.data_prefix, sizeof(cfg->mqtt.data_prefix));
    strlcpy(cfg->mqtt.command_prefix, v3->mqtt.command_prefix, sizeof(cfg->mqtt.command_prefix));
    cfg->mqtt.keepalive_sec = v3->mqtt.keepalive_sec;
    cfg->mqtt.qos = v3->mqtt.qos;
    cfg->mqtt.clean_session = v3->mqtt.clean_session;
    cfg->mqtt.retain = v3->mqtt.retain;
    cfg->mqtt.lwt_enabled = v3->mqtt.lwt_enabled;
    strlcpy(cfg->mqtt.lwt_topic, v3->mqtt.lwt_topic, sizeof(cfg->mqtt.lwt_topic));
    strlcpy(cfg->mqtt.lwt_payload, v3->mqtt.lwt_payload, sizeof(cfg->mqtt.lwt_payload));
    cfg->mqtt.lwt_qos = v3->mqtt.lwt_qos;
    cfg->mqtt.lwt_retain = v3->mqtt.lwt_retain;
    cfg->mqtt.report_mode = MQ_REPORT_SUBDEVICE;
    cfg->modbus_rtu = v3->modbus_rtu;
    cfg->tcp_endpoint_count = v3->tcp_endpoint_count;
    memcpy(cfg->tcp_endpoints, v3->tcp_endpoints, sizeof(cfg->tcp_endpoints));
    cfg->time = v3->time;
    cfg->security = v3->security;
    cfg->modbus_tcp_server = v3->modbus_tcp_server;
    cfg->northbound = v3->northbound;
}

static const char *TAG = "CONFIG";
static const uint32_t CONFIG_MAGIC = 0x47435731U;
static runtime_config_t s_config;
static SemaphoreHandle_t s_mutex;

static uint32_t crc32_bytes(const uint8_t *data, size_t size)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xEDB88320U & (uint32_t)-(int32_t)(crc & 1U));
        }
    }
    return ~crc;
}

static void load_defaults(runtime_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->schema_version = RUNTIME_CONFIG_SCHEMA_VERSION;
    strlcpy(cfg->gateway_id, MQTT_CLIENT_ID, sizeof(cfg->gateway_id));
    cfg->locale = UI_LOCALE_ZH_CN;
    cfg->prefer_ethernet = true;
    cfg->lcd_enabled = true;
    cfg->tf_enabled = true;
    cfg->mcp_write_enabled = false;
    cfg->wifi.enabled = true;
    strlcpy(cfg->wifi.ssid, WIFI_SSID, sizeof(cfg->wifi.ssid));
    strlcpy(cfg->wifi.password, WIFI_PASSWORD, sizeof(cfg->wifi.password));
    cfg->mqtt.enabled = true;
    cfg->mqtt.platform_type = MQTT_PLATFORM_CUSTOM;
    strlcpy(cfg->mqtt.uri, MQTT_BROKER_URI, sizeof(cfg->mqtt.uri));
    strlcpy(cfg->mqtt.client_id, MQTT_CLIENT_ID, sizeof(cfg->mqtt.client_id));
    strlcpy(cfg->mqtt.username, MQTT_USERNAME, sizeof(cfg->mqtt.username));
    strlcpy(cfg->mqtt.password, MQTT_PASSWORD, sizeof(cfg->mqtt.password));
    strlcpy(cfg->mqtt.data_prefix, MQTT_DATA_TOPIC_PREFIX, sizeof(cfg->mqtt.data_prefix));
    strlcpy(cfg->mqtt.command_prefix, MQTT_CMD_TOPIC_PREFIX, sizeof(cfg->mqtt.command_prefix));
    cfg->mqtt.keepalive_sec = MQTT_KEEPALIVE_SEC;
    cfg->mqtt.qos = 1;
    cfg->mqtt.clean_session = false;
    cfg->mqtt.retain = false;
    cfg->mqtt.lwt_enabled = true;
    snprintf(cfg->mqtt.lwt_topic, sizeof(cfg->mqtt.lwt_topic),
             "gateway/%s/status", cfg->gateway_id);
    strlcpy(cfg->mqtt.lwt_payload, "offline", sizeof(cfg->mqtt.lwt_payload));
    cfg->mqtt.lwt_qos = 1;
    cfg->mqtt.lwt_retain = true;
    cfg->mqtt.report_mode = MQ_REPORT_SUBDEVICE;
    cfg->modbus_rtu.enabled = true;
    cfg->modbus_rtu.baud_rate = MODBUS_RTU_BAUD_RATE;
    cfg->modbus_rtu.parity = 0;
    cfg->modbus_rtu.timeout_ms = MODBUS_TIMEOUT_MS;
    cfg->time.enabled = true;
    strlcpy(cfg->time.server1, "pool.ntp.org", sizeof(cfg->time.server1));
    strlcpy(cfg->time.server2, "time.cloudflare.com", sizeof(cfg->time.server2));
    strlcpy(cfg->time.timezone, "CST-8", sizeof(cfg->time.timezone));
    cfg->time.sync_interval_ms = 3600000;
    cfg->security.auth_enabled = false;
    cfg->security.ota_enabled = true;
    cfg->security.ota_allow_http = false;
    cfg->modbus_tcp_server.enabled = true;
    cfg->modbus_tcp_server.port = 502;
    cfg->modbus_tcp_server.max_clients = 4;
    cfg->modbus_tcp_server.response_timeout_ms = 1000;
    strlcpy(cfg->northbound.sparkplug_group, "ESP32_Gateways",
            sizeof(cfg->northbound.sparkplug_group));
    strlcpy(cfg->northbound.sparkplug_node, cfg->gateway_id,
            sizeof(cfg->northbound.sparkplug_node));
    strlcpy(cfg->northbound.sparkplug_device, "modbus",
            sizeof(cfg->northbound.sparkplug_device));
}

esp_err_t runtime_config_validate(const runtime_config_t *config,
                                  char *reason, size_t reason_size)
{
    const char *error = NULL;
    if (config == NULL) error = "configuration is null";
    else if (config->schema_version != RUNTIME_CONFIG_SCHEMA_VERSION) error = "schema version mismatch";
    else if (config->gateway_id[0] == '\0') error = "gateway ID is required";
    else if (config->tcp_endpoint_count > RUNTIME_MAX_TCP_ENDPOINTS) error = "too many TCP endpoints";
    else if (config->mqtt.qos > 2 || config->mqtt.lwt_qos > 2) error = "MQTT QoS must be 0..2";
    else if (config->mqtt.platform_type != MQTT_PLATFORM_CUSTOM &&
             config->mqtt.platform_type != MQTT_PLATFORM_THINGSCLOUD)
        error = "invalid MQTT platform type";
    else if (config->mqtt.report_mode != MQ_REPORT_SUBDEVICE &&
             config->mqtt.report_mode != MQ_REPORT_GATEWAY)
        error = "invalid MQTT report mode";
    else if (config->modbus_rtu.baud_rate < 1200 ||
             config->modbus_rtu.baud_rate > 1000000) error = "invalid RTU baud rate";
    else if (config->modbus_rtu.timeout_ms < 20 ||
             config->modbus_rtu.timeout_ms > 30000) error = "invalid RTU timeout";
    else if (config->modbus_tcp_server.enabled &&
             config->modbus_tcp_server.port == 0) error = "invalid Modbus TCP server port";
    else if (config->security.auth_enabled &&
             (config->security.username[0] == '\0' ||
              strlen(config->security.password_sha256) != 64)) error = "web authentication credentials are incomplete";
    if (reason != NULL && reason_size > 0) {
        strlcpy(reason, error != NULL ? error : "ok", reason_size);
    }
    return error == NULL ? ESP_OK : ESP_ERR_INVALID_ARG;
}

static esp_err_t save_locked(void)
{
    stored_config_t *stored = calloc(1, sizeof(*stored));
    if (stored == NULL) return ESP_ERR_NO_MEM;
    stored->magic = CONFIG_MAGIC;
    stored->generation = 1;
    stored->config = s_config;
    stored->crc32 = crc32_bytes((const uint8_t *)&stored->config, sizeof(stored->config));
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(CONFIG_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        free(stored);
        return err;
    }
    uint8_t active = 0;
    (void)nvs_get_u8(nvs, CONFIG_NVS_KEY_ACTIVE, &active);
    const char *target = active == 0 ? CONFIG_NVS_KEY_B : CONFIG_NVS_KEY_A;
    stored_config_t *old = calloc(1, sizeof(*old));
    const char *active_key = active == 0 ? CONFIG_NVS_KEY_A : CONFIG_NVS_KEY_B;
    size_t old_size = old != NULL ? sizeof(*old) : 0;
    if (old != NULL &&
        nvs_get_blob(nvs, active_key, old, &old_size) == ESP_OK &&
        old_size == sizeof(*old)) {
        stored->generation = old->generation + 1;
    }
    free(old);
    err = nvs_set_blob(nvs, target, stored, sizeof(*stored));
    if (err == ESP_OK) err = nvs_commit(nvs);
    if (err == ESP_OK) {
        active = active == 0 ? 1 : 0;
        err = nvs_set_u8(nvs, CONFIG_NVS_KEY_ACTIVE, active);
    }
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    free(stored);
    return err;
}

static bool stored_config_valid(const stored_config_t *stored, size_t size)
{
    if (stored == NULL || size != sizeof(*stored) ||
        stored->magic != CONFIG_MAGIC ||
        stored->config.schema_version != RUNTIME_CONFIG_SCHEMA_VERSION) {
        return false;
    }
    uint32_t crc = crc32_bytes((const uint8_t *)&stored->config,
                               sizeof(stored->config));
    return stored->crc32 == crc &&
           runtime_config_validate(&stored->config, NULL, 0) == ESP_OK;
}

esp_err_t runtime_config_init(void)
{
    if (s_mutex == NULL) s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) return ESP_ERR_NO_MEM;

    load_defaults(&s_config);
    stored_config_t *slots = calloc(2, sizeof(*slots));
    if (slots == NULL) return ESP_ERR_NO_MEM;
    size_t sizes[2] = {sizeof(slots[0]), sizeof(slots[1])};
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(CONFIG_NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err == ESP_OK) {
        esp_err_t err_a = nvs_get_blob(nvs, CONFIG_NVS_KEY_A, &slots[0], &sizes[0]);
        esp_err_t err_b = nvs_get_blob(nvs, CONFIG_NVS_KEY_B, &slots[1], &sizes[1]);
        bool valid_a = err_a == ESP_OK && stored_config_valid(&slots[0], sizes[0]);
        bool valid_b = err_b == ESP_OK && stored_config_valid(&slots[1], sizes[1]);
        if (valid_a || valid_b) {
            stored_config_t *selected = valid_b &&
                (!valid_a || slots[1].generation > slots[0].generation)
                ? &slots[1] : &slots[0];
            s_config = selected->config;
            nvs_close(nvs);
            ESP_LOGI(TAG, "Transactional runtime configuration loaded (generation=%lu)",
                     (unsigned long)selected->generation);
            free(slots);
            return ESP_OK;
        }

        /* Attempt v2 (pre platform_type) migration using the already-read slots. */
        for (int i = 0; i < 2; ++i) {
            if (sizes[i] == sizeof(stored_config_v2_t)) {
                stored_config_v2_t *v2 = (stored_config_v2_t *)&slots[i];
                if (v2->magic == CONFIG_MAGIC &&
                    v2->config.schema_version == 2 &&
                    v2->crc32 == crc32_bytes((const uint8_t *)&v2->config,
                                             sizeof(v2->config))) {
                    migrate_v2_to_v3(&v2->config, &s_config);
                    free(slots);
                    ESP_LOGI(TAG, "Migrated runtime configuration schema v2 to v3 (platform=CUSTOM)");
                    return save_locked();
                }
            }
        }

        /* Attempt v3 (pre report_mode) migration using the already-read slots. */
        for (int i = 0; i < 2; ++i) {
            if (sizes[i] == sizeof(stored_config_v3_t)) {
                stored_config_v3_t *v3 = (stored_config_v3_t *)&slots[i];
                if (v3->magic == CONFIG_MAGIC &&
                    v3->config.schema_version == 3 &&
                    v3->crc32 == crc32_bytes((const uint8_t *)&v3->config,
                                             sizeof(v3->config))) {
                    migrate_v3_to_current(&v3->config, &s_config);
                    free(slots);
                    ESP_LOGI(TAG, "Migrated runtime configuration schema v3 (added report_mode)");
                    return save_locked();
                }
            }
        }

        stored_config_v1_t *legacy = calloc(1, sizeof(*legacy));
        size_t legacy_size = legacy != NULL ? sizeof(*legacy) : 0;
        err = legacy == NULL ? ESP_ERR_NO_MEM :
            nvs_get_blob(nvs, CONFIG_NVS_KEY_LEGACY, legacy, &legacy_size);
        nvs_close(nvs);
        if (err == ESP_OK && legacy_size == sizeof(*legacy) &&
            legacy->magic == CONFIG_MAGIC &&
            legacy->crc32 == crc32_bytes((const uint8_t *)&legacy->config,
                                         sizeof(legacy->config)) &&
            legacy->config.schema_version == 1) {
            s_config.locale = legacy->config.locale;
            s_config.prefer_ethernet = legacy->config.prefer_ethernet;
            s_config.lcd_enabled = legacy->config.lcd_enabled;
            s_config.tf_enabled = legacy->config.tf_enabled;
            s_config.mcp_write_enabled = legacy->config.mcp_write_enabled;
            s_config.wifi = legacy->config.wifi;
            s_config.modbus_rtu = legacy->config.modbus_rtu;
            s_config.tcp_endpoint_count = legacy->config.tcp_endpoint_count;
            memcpy(s_config.tcp_endpoints, legacy->config.tcp_endpoints,
                   sizeof(s_config.tcp_endpoints));
            strlcpy(s_config.gateway_id, legacy->config.gateway_id,
                    sizeof(s_config.gateway_id));
            s_config.mqtt.enabled = legacy->config.mqtt.enabled;
            strlcpy(s_config.mqtt.uri, legacy->config.mqtt.uri,
                    sizeof(s_config.mqtt.uri));
            strlcpy(s_config.mqtt.client_id, legacy->config.mqtt.client_id,
                    sizeof(s_config.mqtt.client_id));
            strlcpy(s_config.mqtt.username, legacy->config.mqtt.username,
                    sizeof(s_config.mqtt.username));
            strlcpy(s_config.mqtt.password, legacy->config.mqtt.password,
                    sizeof(s_config.mqtt.password));
            strlcpy(s_config.mqtt.data_prefix, legacy->config.mqtt.data_prefix,
                    sizeof(s_config.mqtt.data_prefix));
            strlcpy(s_config.mqtt.command_prefix, legacy->config.mqtt.command_prefix,
                    sizeof(s_config.mqtt.command_prefix));
            s_config.mqtt.keepalive_sec = legacy->config.mqtt.keepalive_sec;
            s_config.mqtt.qos = legacy->config.mqtt.qos;
            free(legacy);
            free(slots);
            ESP_LOGI(TAG, "Migrated runtime configuration schema v1 to v2");
            return save_locked();
        }
        free(legacy);
    }
    free(slots);
    ESP_LOGW(TAG, "Using default runtime configuration");
    return save_locked();
}

void runtime_config_get(runtime_config_t *out)
{
    if (out == NULL) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_config;
    xSemaphoreGive(s_mutex);
}

ui_locale_t runtime_config_get_locale(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    ui_locale_t locale = s_config.locale;
    xSemaphoreGive(s_mutex);
    return locale;
}

esp_err_t runtime_config_set(const runtime_config_t *config)
{
    ESP_RETURN_ON_ERROR(runtime_config_validate(config, NULL, 0),
                        TAG, "runtime config validation");
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    runtime_config_t previous = s_config;
    s_config = *config;
    esp_err_t err = save_locked();
    if (err != ESP_OK) s_config = previous;
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t runtime_config_reset(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    load_defaults(&s_config);
    esp_err_t err = save_locked();
    xSemaphoreGive(s_mutex);
    return err;
}
