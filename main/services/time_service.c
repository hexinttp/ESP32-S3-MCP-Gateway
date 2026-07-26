#include "services/time_service.h"

#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include "config/runtime_config.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "TIME";
static SemaphoreHandle_t s_mutex;
static time_service_status_t s_status;

static int64_t wall_time_ms(void)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    return (int64_t)now.tv_sec * 1000 + now.tv_usec / 1000;
}

static void time_sync_callback(struct timeval *tv)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_status.synchronized = tv != NULL && tv->tv_sec > 1609459200;
    s_status.last_sync_ms = wall_time_ms();
    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "System time synchronized");
}

esp_err_t time_service_init(void)
{
    if (s_mutex == NULL) s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) return ESP_ERR_NO_MEM;

    runtime_config_t config;
    runtime_config_get(&config);
    memset(&s_status, 0, sizeof(s_status));
    s_status.enabled = config.time.enabled;
    strlcpy(s_status.server, config.time.server1, sizeof(s_status.server));
    if (!config.time.enabled) return ESP_OK;

    setenv("TZ", config.time.timezone[0] ? config.time.timezone : "UTC0", 1);
    tzset();
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, config.time.server1);
    if (config.time.server2[0]) esp_sntp_setservername(1, config.time.server2);
    esp_sntp_set_sync_interval(config.time.sync_interval_ms >= 15000
        ? config.time.sync_interval_ms : 3600000);
    esp_sntp_set_time_sync_notification_cb(time_sync_callback);
    esp_sntp_init();
    ESP_LOGI(TAG, "SNTP started: %s", config.time.server1);
    return ESP_OK;
}

void time_service_get_status(time_service_status_t *out)
{
    if (out == NULL || s_mutex == NULL) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_status;
    out->current_time_ms = wall_time_ms();
    xSemaphoreGive(s_mutex);
}

bool time_service_is_synchronized(void)
{
    if (s_mutex == NULL) return false;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool synchronized = s_status.synchronized;
    xSemaphoreGive(s_mutex);
    return synchronized;
}
