#include "services/ota_service.h"

#include <ctype.h>
#include <string.h>
#include "config/runtime_config.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "OTA";
static SemaphoreHandle_t s_mutex;
static TaskHandle_t s_task;
static ota_status_t s_status;
static char s_url[256];
static uint8_t s_expected_sha[32];
static bool s_has_expected_sha;

static bool parse_sha256(const char *text, uint8_t out[32])
{
    if (text == NULL || strlen(text) != 64) return false;
    for (int i = 0; i < 32; ++i) {
        char high = (char)tolower((unsigned char)text[i * 2]);
        char low = (char)tolower((unsigned char)text[i * 2 + 1]);
        if (!isxdigit((unsigned char)high) || !isxdigit((unsigned char)low)) return false;
        uint8_t hi = high <= '9' ? high - '0' : high - 'a' + 10;
        uint8_t lo = low <= '9' ? low - '0' : low - 'a' + 10;
        out[i] = (uint8_t)(hi << 4 | lo);
    }
    return true;
}

static void set_status(ota_state_t state, esp_err_t error, const char *message)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_status.state = state;
    s_status.last_error = error;
    strlcpy(s_status.message, message, sizeof(s_status.message));
    xSemaphoreGive(s_mutex);
}

static void ota_task(void *argument)
{
    (void)argument;
    set_status(OTA_STATE_DOWNLOADING, ESP_OK, "downloading and validating image");
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    esp_http_client_config_t http = {
        .url = s_url,
        .timeout_ms = 15000,
        .keep_alive_enable = true,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_https_ota_config_t ota = {
        .http_config = &http,
        .bulk_flash_erase = false,
    };
    esp_err_t err = esp_https_ota(&ota);
    if (err == ESP_OK && s_has_expected_sha && target != NULL) {
        set_status(OTA_STATE_VERIFYING, ESP_OK, "verifying SHA-256");
        uint8_t actual[32];
        err = esp_partition_get_sha256(target, actual);
        if (err == ESP_OK && memcmp(actual, s_expected_sha, sizeof(actual)) != 0) {
            err = ESP_ERR_INVALID_CRC;
        }
        if (err != ESP_OK && running != NULL) {
            (void)esp_ota_set_boot_partition(running);
        }
    }
    if (err == ESP_OK) {
        set_status(OTA_STATE_READY_TO_REBOOT, ESP_OK,
                   "verified image ready; reboot required");
    } else {
        set_status(OTA_STATE_FAILED, err, esp_err_to_name(err));
        ESP_LOGE(TAG, "OTA failed: %s", esp_err_to_name(err));
    }
    s_task = NULL;
    vTaskDelete(NULL);
}

static void confirm_task(void *argument)
{
    (void)argument;
    vTaskDelay(pdMS_TO_TICKS(30000));
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (running != NULL && esp_ota_get_state_partition(running, &state) == ESP_OK &&
        state == ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ota_mark_app_valid_cancel_rollback());
        ESP_LOGI(TAG, "Running OTA image confirmed healthy");
    }
    vTaskDelete(NULL);
}

esp_err_t ota_service_init(void)
{
    if (s_mutex == NULL) s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) return ESP_ERR_NO_MEM;
    memset(&s_status, 0, sizeof(s_status));
    strlcpy(s_status.message, "idle", sizeof(s_status.message));
    return xTaskCreate(confirm_task, "ota_confirm", 3072, NULL, 2, NULL) == pdPASS
        ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t ota_service_start(const char *url, const char *expected_sha256)
{
    runtime_config_t config;
    runtime_config_get(&config);
    if (!config.security.ota_enabled || url == NULL || s_task != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    bool https = strncmp(url, "https://", 8) == 0;
    bool http = strncmp(url, "http://", 7) == 0;
    if (!https && !(http && config.security.ota_allow_http)) {
        return ESP_ERR_NOT_ALLOWED;
    }
    s_has_expected_sha = parse_sha256(expected_sha256, s_expected_sha);
    if (!s_has_expected_sha) return ESP_ERR_INVALID_ARG;
    strlcpy(s_url, url, sizeof(s_url));
    return xTaskCreate(ota_task, "https_ota", 8192, NULL, 5, &s_task) == pdPASS
        ? ESP_OK : ESP_ERR_NO_MEM;
}

void ota_service_get_status(ota_status_t *out)
{
    if (out == NULL || s_mutex == NULL) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_status;
    xSemaphoreGive(s_mutex);
}
