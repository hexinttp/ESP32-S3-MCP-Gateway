#include "runtime_config.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "esp_log.h"
#include "gateway_config.h"

#define CONFIG_NVS_NAMESPACE "gateway"
#define CONFIG_NVS_KEY "runtime"

typedef struct {
    uint32_t magic;
    uint32_t crc32;
    runtime_config_t config;
} stored_config_t;

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
    strlcpy(cfg->mqtt.uri, MQTT_BROKER_URI, sizeof(cfg->mqtt.uri));
    strlcpy(cfg->mqtt.client_id, MQTT_CLIENT_ID, sizeof(cfg->mqtt.client_id));
    strlcpy(cfg->mqtt.username, MQTT_USERNAME, sizeof(cfg->mqtt.username));
    strlcpy(cfg->mqtt.password, MQTT_PASSWORD, sizeof(cfg->mqtt.password));
    strlcpy(cfg->mqtt.data_prefix, MQTT_DATA_TOPIC_PREFIX, sizeof(cfg->mqtt.data_prefix));
    strlcpy(cfg->mqtt.command_prefix, MQTT_CMD_TOPIC_PREFIX, sizeof(cfg->mqtt.command_prefix));
    cfg->mqtt.keepalive_sec = MQTT_KEEPALIVE_SEC;
    cfg->mqtt.qos = 1;
    cfg->modbus_rtu.enabled = true;
    cfg->modbus_rtu.baud_rate = MODBUS_RTU_BAUD_RATE;
    cfg->modbus_rtu.parity = 0;
    cfg->modbus_rtu.timeout_ms = MODBUS_TIMEOUT_MS;
}

static esp_err_t save_locked(void)
{
    stored_config_t stored = {.magic = CONFIG_MAGIC, .config = s_config};
    stored.crc32 = crc32_bytes((const uint8_t *)&stored.config, sizeof(stored.config));
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(CONFIG_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;
    err = nvs_set_blob(nvs, CONFIG_NVS_KEY, &stored, sizeof(stored));
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    return err;
}

esp_err_t runtime_config_init(void)
{
    if (s_mutex == NULL) s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) return ESP_ERR_NO_MEM;

    load_defaults(&s_config);
    stored_config_t stored = {0};
    size_t size = sizeof(stored);
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(CONFIG_NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err == ESP_OK) {
        err = nvs_get_blob(nvs, CONFIG_NVS_KEY, &stored, &size);
        nvs_close(nvs);
    }
    uint32_t crc = 0;
    if (err == ESP_OK && size == sizeof(stored)) {
        crc = crc32_bytes((const uint8_t *)&stored.config, sizeof(stored.config));
    }
    if (err == ESP_OK && size == sizeof(stored) && stored.magic == CONFIG_MAGIC &&
        stored.crc32 == crc && stored.config.schema_version == RUNTIME_CONFIG_SCHEMA_VERSION) {
        s_config = stored.config;
        ESP_LOGI(TAG, "Runtime configuration loaded from NVS");
        return ESP_OK;
    }

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

esp_err_t runtime_config_set(const runtime_config_t *config)
{
    if (config == NULL || config->schema_version != RUNTIME_CONFIG_SCHEMA_VERSION ||
        config->tcp_endpoint_count > RUNTIME_MAX_TCP_ENDPOINTS) {
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_config = *config;
    esp_err_t err = save_locked();
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
