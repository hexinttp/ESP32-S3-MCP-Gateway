#include "cloud_adapter/cloud_adapter.h"
#include "thingscloud/thingscloud_topics.h"
#include "mqtt_comm/mqtt_handler.h"
#include "config/runtime_config.h"

#include <string.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "TC_SUBDEV";

#define SUBDEV_SUCCESS_THRESHOLD 2   /* consecutive successes -> ONLINE  */
#define SUBDEV_FAILURE_THRESHOLD 3   /* consecutive failures  -> OFFLINE */

typedef enum {
    SUBDEV_UNKNOWN = 0,
    SUBDEV_ONLINE,
    SUBDEV_OFFLINE
} subdev_state_t;

typedef struct {
    char address[40];
    subdev_state_t state;
    int success_streak;
    int fail_streak;
    int64_t last_success_ms;   /* time of last valid Modbus response (last_seen) */
} subdev_t;

/* Lazily allocated from PSRAM to avoid consuming internal DRAM
 * when ThingsCloud mode is not in use. */
static subdev_t *s_devs = NULL;
static int s_error_count = 0;   /* cumulative RS485 communication errors */
static SemaphoreHandle_t s_subdev_mutex = NULL;

static bool subdev_lock(void)
{
    if (s_subdev_mutex == NULL) s_subdev_mutex = xSemaphoreCreateMutex();
    return s_subdev_mutex != NULL &&
           xSemaphoreTake(s_subdev_mutex, pdMS_TO_TICKS(1000)) == pdTRUE;
}

static void subdev_unlock(void)
{
    if (s_subdev_mutex != NULL) xSemaphoreGive(s_subdev_mutex);
}

static bool subdev_ensure_table(void)
{
    if (s_devs != NULL) return true;
    s_devs = heap_caps_calloc(THINGSCLOUD_SUBDEVICE_MAX, sizeof(subdev_t),
                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_devs == NULL) {
        s_devs = heap_caps_calloc(THINGSCLOUD_SUBDEVICE_MAX, sizeof(subdev_t),
                                  MALLOC_CAP_8BIT);
    }
    if (s_devs == NULL) {
        ESP_LOGE(TAG, "Sub-device table allocation failed");
        return false;
    }
    return true;
}

static int subdev_find(const char *address)
{
    if (s_devs == NULL) return -1;
    for (int i = 0; i < THINGSCLOUD_SUBDEVICE_MAX; ++i) {
        if (s_devs[i].address[0] != '\0' &&
            strcmp(s_devs[i].address, address) == 0) {
            return i;
        }
    }
    return -1;
}

static int subdev_find_or_create(const char *address)
{
    if (!subdev_ensure_table()) return -1;
    int idx = subdev_find(address);
    if (idx >= 0) return idx;
    for (int i = 0; i < THINGSCLOUD_SUBDEVICE_MAX; ++i) {
        if (s_devs[i].address[0] == '\0') {
            strlcpy(s_devs[i].address, address, sizeof(s_devs[i].address));
            s_devs[i].state = SUBDEV_UNKNOWN;
            s_devs[i].success_streak = 0;
            s_devs[i].fail_streak = 0;
            return i;
        }
    }
    return -1;
}

static void subdev_publish_state(const subdev_t *d, bool online)
{
    if (!mqtt_is_connected()) return;   /* will be re-reported on reconnect */
    char payload[64];
    int n = snprintf(payload, sizeof(payload),
                     "{\"device\":\"%s\"}", d->address);
    if (n < 0 || (size_t)n >= sizeof(payload)) return;
    const char *topic = online ? TC_TOPIC_GATEWAY_CONNECT : TC_TOPIC_GATEWAY_DISCONNECT;
    esp_err_t err = mqtt_publish(topic, payload, 0);
    ESP_LOGI(TAG, "Sub-device %s %s (topic=%s, err=%s)",
             d->address, online ? "ONLINE" : "OFFLINE",
             topic, esp_err_to_name(err));
}

/* A slave crossed the ONLINE/OFFLINE threshold. SUBDEVICE mode publishes the
 * ThingsCloud gateway/connect|disconnect control message; GATEWAY mode instead
 * reports the slave's status as gateway attributes (no gateway/connect). */
static void subdev_report_transition(const subdev_t *d, bool online)
{
    runtime_config_t rt;
    runtime_config_get(&rt);
    if (rt.mqtt.report_mode == MQ_REPORT_SUBDEVICE) {
        subdev_publish_state(d, online);
    } else {
        thingscloud_publish_gateway_slave_status();
    }
}

void thingscloud_subdev_register_success(const char *device_address)
{
    if (!thingscloud_is_enabled() || device_address == NULL) return;
    if (!subdev_lock()) return;
    int idx = subdev_find_or_create(device_address);
    if (idx < 0) {
        subdev_unlock();
        return;
    }
    subdev_t *d = &s_devs[idx];
    d->fail_streak = 0;
    d->success_streak++;
    d->last_success_ms = esp_timer_get_time() / 1000;   /* update last_seen */
    bool transitioned = d->state != SUBDEV_ONLINE &&
                        d->success_streak >= SUBDEV_SUCCESS_THRESHOLD;
    subdev_t snapshot = *d;
    if (transitioned) {
        d->state = SUBDEV_ONLINE;
        snapshot.state = SUBDEV_ONLINE;
    }
    subdev_unlock();
    if (transitioned) subdev_report_transition(&snapshot, true);
}

void thingscloud_subdev_register_failure(const char *device_address)
{
    if (!thingscloud_is_enabled() || device_address == NULL) return;
    if (!subdev_lock()) return;
    int idx = subdev_find_or_create(device_address);
    if (idx < 0) {
        subdev_unlock();
        return;
    }
    subdev_t *d = &s_devs[idx];
    d->success_streak = 0;
    d->fail_streak++;
    s_error_count++;
    bool transitioned = d->state != SUBDEV_OFFLINE &&
                        d->fail_streak >= SUBDEV_FAILURE_THRESHOLD;
    subdev_t snapshot = *d;
    if (transitioned) {
        d->state = SUBDEV_OFFLINE;
        snapshot.state = SUBDEV_OFFLINE;
    }
    subdev_unlock();
    if (transitioned) subdev_report_transition(&snapshot, false);
}

void thingscloud_subdev_republish_all_online(void)
{
    if (!thingscloud_is_enabled() || s_devs == NULL) return;
    if (!subdev_lock()) return;
    for (int i = 0; i < THINGSCLOUD_SUBDEVICE_MAX; ++i) {
        if (s_devs[i].state == SUBDEV_ONLINE && s_devs[i].address[0] != '\0') {
            subdev_t snapshot = s_devs[i];
            subdev_unlock();
            subdev_publish_state(&snapshot, true);
            if (!subdev_lock()) return;
        }
    }
    subdev_unlock();
}

int thingscloud_subdev_online_count(void)
{
    if (s_devs == NULL) return 0;
    if (!subdev_lock()) return 0;
    int count = 0;
    for (int i = 0; i < THINGSCLOUD_SUBDEVICE_MAX; ++i) {
        if (s_devs[i].state == SUBDEV_ONLINE) count++;
    }
    subdev_unlock();
    return count;
}

int thingscloud_subdev_offline_count(void)
{
    if (s_devs == NULL) return 0;
    if (!subdev_lock()) return 0;
    int count = 0;
    for (int i = 0; i < THINGSCLOUD_SUBDEVICE_MAX; ++i) {
        if (s_devs[i].address[0] != '\0' &&
            s_devs[i].state == SUBDEV_OFFLINE) {
            count++;
        }
    }
    subdev_unlock();
    return count;
}

int thingscloud_subdev_error_count(void)
{
    if (!subdev_lock()) return s_error_count;
    int count = s_error_count;
    subdev_unlock();
    return count;
}

int thingscloud_subdev_get_status(thingscloud_slave_status_t *out, int max)
{
    if (out == NULL || max <= 0 || s_devs == NULL) return 0;
    if (!subdev_lock()) return 0;
    int written = 0;
    for (int i = 0; i < THINGSCLOUD_SUBDEVICE_MAX && written < max; ++i) {
        if (s_devs[i].address[0] == '\0') continue;
        thingscloud_slave_status_t *s = &out[written];
        memset(s, 0, sizeof(*s));
        strlcpy(s->address, s_devs[i].address, sizeof(s->address));
        uint8_t ch = 0, sl = 0;
        if (thingscloud_parse_device_address(s_devs[i].address, &ch, &sl)) {
            s->port_id = ch;      /* raw channel_id; caller adds +1 for display */
            s->slave_id = sl;
        }
        s->state = (s_devs[i].state == SUBDEV_ONLINE) ? 1
                 : (s_devs[i].state == SUBDEV_OFFLINE) ? 2 : 0;
        s->last_seen_ms = s_devs[i].last_success_ms;
        s->error_count = (uint32_t)(s_devs[i].fail_streak < 0 ? 0 : s_devs[i].fail_streak);
        s->data_valid = (s_devs[i].state == SUBDEV_ONLINE);
        written++;
    }
    subdev_unlock();
    return written;
}
