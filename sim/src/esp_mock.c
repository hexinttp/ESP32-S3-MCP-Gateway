/**
 * @file esp_mock.c
 * @brief Mock implementations of ALL ESP-IDF functions used by the gateway modules.
 *
 * Provides FreeRTOS, NVS, MODBUS, MQTT, WiFi, and system API mocks so that the
 * original module .c files compile and run unchanged on a PC (Windows/Linux).
 *
 * NOTES ON COMPILATION:
 *   - The original module source files include ESP-IDF headers via paths like
 *     "esp_log.h", "freertos/FreeRTOS.h", "nvs.h", etc.  The mock headers
 *     in sim/include/ must match the exact API signatures used by the modules.
 *   - gmtime_r() is available in MinGW-w64 and glibc.  If your toolchain lacks
 *     it, define _POSIX_C_SOURCE or use localtime instead.
 *   - On Windows, Sleep() is used for delays.  On Linux, usleep() is used.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <inttypes.h>

/* Platform-specific headers */
#ifdef _WIN32
  #include <windows.h>
#else
  #include <unistd.h>
  #include <sys/time.h>
#endif

/* pthreads for FreeRTOS mock */
#include <pthread.h>

/* Mock ESP-IDF headers */
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_flash.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "mqtt_client.h"
#include "mbcontroller.h"
#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

/* ================================================================
 * SECTION 1: FreeRTOS Mocks (using pthreads)
 * ================================================================ */

/* ---- Semaphore / Mutex ---- */

SemaphoreHandle_t xSemaphoreCreateMutex(void)
{
    pthread_mutex_t *mtx = (pthread_mutex_t *)malloc(sizeof(pthread_mutex_t));
    if (!mtx) return NULL;
    pthread_mutex_init(mtx, NULL);
    return (SemaphoreHandle_t)mtx;
}

SemaphoreHandle_t xSemaphoreCreateBinary(void)
{
    /* Binary semaphore: initially "given" (unlocked). */
    pthread_mutex_t *mtx = (pthread_mutex_t *)malloc(sizeof(pthread_mutex_t));
    if (!mtx) return NULL;
    pthread_mutex_init(mtx, NULL);
    return (SemaphoreHandle_t)mtx;
}

BaseType_t xSemaphoreTake(SemaphoreHandle_t xSemaphore, TickType_t xTicksToWait)
{
    (void)xTicksToWait;
    if (!xSemaphore) return pdFALSE;
    pthread_mutex_lock((pthread_mutex_t *)xSemaphore);
    return pdTRUE;
}

BaseType_t xSemaphoreGive(SemaphoreHandle_t xSemaphore)
{
    if (!xSemaphore) return pdFALSE;
    pthread_mutex_unlock((pthread_mutex_t *)xSemaphore);
    return pdTRUE;
}

void vSemaphoreDelete(SemaphoreHandle_t xSemaphore)
{
    if (!xSemaphore) return;
    pthread_mutex_destroy((pthread_mutex_t *)xSemaphore);
    free(xSemaphore);
}

/* ---- Queue ---- */

typedef struct {
    uint8_t        *buffer;
    UBaseType_t     item_size;
    UBaseType_t     capacity;
    UBaseType_t     count;
    UBaseType_t     head;
    UBaseType_t     tail;
    pthread_mutex_t mutex;
    pthread_cond_t  cond_not_empty;
    pthread_cond_t  cond_not_full;
} sim_queue_t;

QueueHandle_t xQueueCreate(UBaseType_t uxQueueLength, UBaseType_t uxItemSize)
{
    sim_queue_t *q = (sim_queue_t *)calloc(1, sizeof(sim_queue_t));
    if (!q) return NULL;

    q->buffer = (uint8_t *)calloc(uxQueueLength, uxItemSize);
    if (!q->buffer) { free(q); return NULL; }

    q->item_size  = uxItemSize;
    q->capacity   = uxQueueLength;
    q->count      = 0;
    q->head       = 0;
    q->tail       = 0;

    pthread_mutex_init(&q->mutex, NULL);
    pthread_cond_init(&q->cond_not_empty, NULL);
    pthread_cond_init(&q->cond_not_full, NULL);

    return (QueueHandle_t)q;
}

BaseType_t xQueueSend(QueueHandle_t xQueue, const void *pvItemToQueue,
                       TickType_t xTicksToWait)
{
    (void)xTicksToWait;
    if (!xQueue || !pvItemToQueue) return pdFALSE;

    sim_queue_t *q = (sim_queue_t *)xQueue;
    pthread_mutex_lock(&q->mutex);

    if (q->count >= q->capacity) {
        /* Queue full - short timed wait for simulation */
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        long wait_ms = (long)xTicksToWait;
        if (wait_ms > 5000 || wait_ms <= 0) wait_ms = 100;
        ts.tv_nsec += wait_ms * 1000000L;
        if (ts.tv_nsec >= 1000000000L) {
            ts.tv_sec  += ts.tv_nsec / 1000000000L;
            ts.tv_nsec %= 1000000000L;
        }
        pthread_cond_timedwait(&q->cond_not_full, &q->mutex, &ts);

        if (q->count >= q->capacity) {
            pthread_mutex_unlock(&q->mutex);
            return pdFALSE;
        }
    }

    memcpy(q->buffer + q->tail * q->item_size, pvItemToQueue, q->item_size);
    q->tail = (q->tail + 1) % q->capacity;
    q->count++;

    pthread_cond_signal(&q->cond_not_empty);
    pthread_mutex_unlock(&q->mutex);
    return pdTRUE;
}

BaseType_t xQueueReceive(QueueHandle_t xQueue, void *pvBuffer,
                          TickType_t xTicksToWait)
{
    if (!xQueue || !pvBuffer) return pdFALSE;

    sim_queue_t *q = (sim_queue_t *)xQueue;
    pthread_mutex_lock(&q->mutex);

    if (q->count == 0) {
        long wait_ms = (long)xTicksToWait;
        if (wait_ms > 5000 || wait_ms <= 0) wait_ms = 100;

        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_nsec += wait_ms * 1000000L;
        if (ts.tv_nsec >= 1000000000L) {
            ts.tv_sec  += ts.tv_nsec / 1000000000L;
            ts.tv_nsec %= 1000000000L;
        }
        pthread_cond_timedwait(&q->cond_not_empty, &q->mutex, &ts);

        if (q->count == 0) {
            pthread_mutex_unlock(&q->mutex);
            return pdFALSE;
        }
    }

    memcpy(pvBuffer, q->buffer + q->head * q->item_size, q->item_size);
    q->head = (q->head + 1) % q->capacity;
    q->count--;

    pthread_cond_signal(&q->cond_not_full);
    pthread_mutex_unlock(&q->mutex);
    return pdTRUE;
}

void vQueueDelete(QueueHandle_t xQueue)
{
    if (!xQueue) return;
    sim_queue_t *q = (sim_queue_t *)xQueue;
    pthread_mutex_destroy(&q->mutex);
    pthread_cond_destroy(&q->cond_not_empty);
    pthread_cond_destroy(&q->cond_not_full);
    free(q->buffer);
    free(q);
}

/* ---- Task (Thread) ---- */

typedef struct {
    TaskFunction_t func;
    void          *param;
    char           name[32];
    volatile int   running;
    pthread_t      thread;
} sim_task_t;

static void *task_wrapper(void *arg)
{
    sim_task_t *t = (sim_task_t *)arg;
    t->func(t->param);
    return NULL;
}

BaseType_t xTaskCreate(TaskFunction_t pvTaskCode,
                        const char *pcName,
                        uint32_t usStackDepth,
                        void *pvParameters,
                        UBaseType_t uxPriority,
                        TaskHandle_t *pxCreatedTask)
{
    (void)usStackDepth;
    (void)uxPriority;

    sim_task_t *t = (sim_task_t *)calloc(1, sizeof(sim_task_t));
    if (!t) return pdFAIL;

    t->func    = pvTaskCode;
    t->param   = pvParameters;
    t->running = 1;
    if (pcName) {
        strncpy(t->name, pcName, sizeof(t->name) - 1);
    }

    int ret = pthread_create(&t->thread, NULL, task_wrapper, t);
    if (ret != 0) {
        free(t);
        return pdFAIL;
    }

    if (pxCreatedTask) {
        *pxCreatedTask = (TaskHandle_t)t;
    }
    return pdPASS;
}

void vTaskDelete(TaskHandle_t xTaskToDelete)
{
    if (!xTaskToDelete) return;
    sim_task_t *t = (sim_task_t *)xTaskToDelete;
    t->running = 0;
    pthread_detach(t->thread);
    free(t);
}

void vTaskDelay(TickType_t xTicksToDelay)
{
    unsigned long ms = (unsigned long)xTicksToDelay;
#ifdef _WIN32
    Sleep((DWORD)ms);
#else
    usleep((useconds_t)(ms * 1000));
#endif
}

UBaseType_t uxTaskGetNumberOfTasks(void)
{
    return 5;
}

/* ---- Event Group ---- */

typedef struct {
    uint32_t        bits;
    pthread_mutex_t mutex;
    pthread_cond_t  cond;
} sim_event_group_t;

EventGroupHandle_t xEventGroupCreate(void)
{
    sim_event_group_t *eg = (sim_event_group_t *)calloc(1, sizeof(sim_event_group_t));
    if (!eg) return NULL;
    eg->bits = 0;
    pthread_mutex_init(&eg->mutex, NULL);
    pthread_cond_init(&eg->cond, NULL);
    return (EventGroupHandle_t)eg;
}

EventBits_t xEventGroupSetBits(EventGroupHandle_t xEventGroup, EventBits_t uxBitsToSet)
{
    if (!xEventGroup) return 0;
    sim_event_group_t *eg = (sim_event_group_t *)xEventGroup;
    pthread_mutex_lock(&eg->mutex);
    eg->bits |= uxBitsToSet;
    uint32_t result = eg->bits;
    pthread_cond_broadcast(&eg->cond);
    pthread_mutex_unlock(&eg->mutex);
    return (EventBits_t)result;
}

EventBits_t xEventGroupWaitBits(EventGroupHandle_t xEventGroup,
                                 EventBits_t uxBitsToWaitFor,
                                 BaseType_t xClearOnExit,
                                 BaseType_t xWaitForAllBits,
                                 TickType_t xTicksToWait)
{
    if (!xEventGroup) return 0;
    sim_event_group_t *eg = (sim_event_group_t *)xEventGroup;

    pthread_mutex_lock(&eg->mutex);

    int satisfied = 0;
    long wait_ms = (long)xTicksToWait;
    if (wait_ms > 5000 || wait_ms <= 0) wait_ms = 200;

    while (!satisfied) {
        uint32_t match = eg->bits & uxBitsToWaitFor;
        if (xWaitForAllBits) {
            satisfied = (match == uxBitsToWaitFor);
        } else {
            satisfied = (match != 0);
        }
        if (satisfied) break;

        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_nsec += wait_ms * 1000000L;
        if (ts.tv_nsec >= 1000000000L) {
            ts.tv_sec  += ts.tv_nsec / 1000000000L;
            ts.tv_nsec %= 1000000000L;
        }
        int ret = pthread_cond_timedwait(&eg->cond, &eg->mutex, &ts);
        if (ret != 0) break;
    }

    uint32_t result = eg->bits;
    if (satisfied && xClearOnExit) {
        eg->bits &= ~uxBitsToWaitFor;
    }

    pthread_mutex_unlock(&eg->mutex);
    return (EventBits_t)result;
}

void vEventGroupDelete(EventGroupHandle_t xEventGroup)
{
    if (!xEventGroup) return;
    sim_event_group_t *eg = (sim_event_group_t *)xEventGroup;
    pthread_mutex_destroy(&eg->mutex);
    pthread_cond_destroy(&eg->cond);
    free(eg);
}

/* ================================================================
 * SECTION 2: ESP System Mocks
 * ================================================================ */

int64_t esp_timer_get_time(void)
{
#ifdef _WIN32
    static LARGE_INTEGER freq = {{0, 0}};
    LARGE_INTEGER counter;
    if (freq.QuadPart == 0) {
        QueryPerformanceFrequency(&freq);
    }
    QueryPerformanceCounter(&counter);
    return (int64_t)(counter.QuadPart * 1000000LL / freq.QuadPart);
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000LL + (int64_t)ts.tv_nsec / 1000LL;
#endif
}

uint32_t esp_get_free_heap_size(void)
{
    return 200000;
}

uint32_t esp_get_minimum_free_heap_size(void)
{
    return 150000;
}

esp_err_t esp_flash_get_size(void *chip, uint32_t *out_size)
{
    (void)chip;
    if (out_size) {
        *out_size = 2 * 1024 * 1024;
    }
    return ESP_OK;
}

void esp_restart(void)
{
    printf("\n[SIM] *** esp_restart() called - exiting simulation ***\n");
    exit(0);
}

const char *esp_err_to_name(esp_err_t code)
{
    switch (code) {
        case ESP_OK:                        return "ESP_OK";
        case ESP_FAIL:                      return "ESP_FAIL";
        case ESP_ERR_NO_MEM:                return "ESP_ERR_NO_MEM";
        case ESP_ERR_INVALID_ARG:           return "ESP_ERR_INVALID_ARG";
        case ESP_ERR_INVALID_STATE:         return "ESP_ERR_INVALID_STATE";
        case ESP_ERR_INVALID_SIZE:          return "ESP_ERR_INVALID_SIZE";
        case ESP_ERR_NOT_FOUND:             return "ESP_ERR_NOT_FOUND";
        case ESP_ERR_NOT_SUPPORTED:         return "ESP_ERR_NOT_SUPPORTED";
        case ESP_ERR_TIMEOUT:               return "ESP_ERR_TIMEOUT";
        case ESP_ERR_NVS_NO_FREE_PAGES:     return "ESP_ERR_NVS_NO_FREE_PAGES";
        case ESP_ERR_NVS_NEW_VERSION_FOUND: return "ESP_ERR_NVS_NEW_VERSION_FOUND";
        case ESP_ERR_NVS_NOT_FOUND:         return "ESP_ERR_NVS_NOT_FOUND";
        default:                            return "UNKNOWN_ERROR";
    }
}

void esp_log_level_set(const char *tag, esp_log_level_t level)
{
    (void)tag;
    (void)level;
}

/* ================================================================
 * SECTION 3: NVS Mocks (in-memory key-value store)
 * ================================================================ */

#define NVS_MAX_ENTRIES     128
#define NVS_MAX_KEY_LEN     32
#define NVS_MAX_BLOB_LEN    512

typedef struct {
    char    key[NVS_MAX_KEY_LEN];
    uint8_t data[NVS_MAX_BLOB_LEN];
    size_t  data_len;
    int     is_int;
    int32_t int_value;
    int     used;
} nvs_entry_t;

static nvs_entry_t s_nvs_store[NVS_MAX_ENTRIES];
static int          s_nvs_initialized = 0;

static nvs_entry_t *nvs_find_entry(const char *key)
{
    for (int i = 0; i < NVS_MAX_ENTRIES; i++) {
        if (s_nvs_store[i].used && strcmp(s_nvs_store[i].key, key) == 0) {
            return &s_nvs_store[i];
        }
    }
    return NULL;
}

static nvs_entry_t *nvs_get_or_create_entry(const char *key)
{
    nvs_entry_t *e = nvs_find_entry(key);
    if (e) return e;

    for (int i = 0; i < NVS_MAX_ENTRIES; i++) {
        if (!s_nvs_store[i].used) {
            s_nvs_store[i].used = 1;
            strncpy(s_nvs_store[i].key, key, NVS_MAX_KEY_LEN - 1);
            s_nvs_store[i].key[NVS_MAX_KEY_LEN - 1] = '\0';
            return &s_nvs_store[i];
        }
    }
    return NULL;
}

esp_err_t nvs_flash_init(void)
{
    if (!s_nvs_initialized) {
        memset(s_nvs_store, 0, sizeof(s_nvs_store));
        s_nvs_initialized = 1;
    }
    return ESP_OK;
}

esp_err_t nvs_flash_erase(void)
{
    memset(s_nvs_store, 0, sizeof(s_nvs_store));
    return ESP_OK;
}

esp_err_t nvs_open(const char *namespace_name, nvs_open_mode_t open_mode,
                    nvs_handle_t *out_handle)
{
    (void)namespace_name;
    (void)open_mode;
    if (!s_nvs_initialized) {
        nvs_flash_init();
    }
    if (out_handle) {
        *out_handle = 1;
    }
    return ESP_OK;
}

void nvs_close(nvs_handle_t handle)
{
    (void)handle;
}

esp_err_t nvs_commit(nvs_handle_t handle)
{
    (void)handle;
    return ESP_OK;
}

esp_err_t nvs_set_i32(nvs_handle_t handle, const char *key, int32_t value)
{
    (void)handle;
    nvs_entry_t *e = nvs_get_or_create_entry(key);
    if (!e) return ESP_ERR_NO_MEM;
    e->int_value = value;
    e->is_int    = 1;
    return ESP_OK;
}

esp_err_t nvs_get_i32(nvs_handle_t handle, const char *key, int32_t *out_value)
{
    (void)handle;
    nvs_entry_t *e = nvs_find_entry(key);
    if (!e || !e->is_int) return ESP_ERR_NVS_NOT_FOUND;
    if (out_value) *out_value = e->int_value;
    return ESP_OK;
}

esp_err_t nvs_set_blob(nvs_handle_t handle, const char *key,
                        const void *value, size_t length)
{
    (void)handle;
    if (length > NVS_MAX_BLOB_LEN) return ESP_ERR_NO_MEM;
    nvs_entry_t *e = nvs_get_or_create_entry(key);
    if (!e) return ESP_ERR_NO_MEM;
    memcpy(e->data, value, length);
    e->data_len = length;
    e->is_int   = 0;
    return ESP_OK;
}

esp_err_t nvs_get_blob(nvs_handle_t handle, const char *key,
                        void *out_value, size_t *length)
{
    (void)handle;
    nvs_entry_t *e = nvs_find_entry(key);
    if (!e || e->is_int) return ESP_ERR_NVS_NOT_FOUND;
    if (out_value && length) {
        size_t copy_len = (*length < e->data_len) ? *length : e->data_len;
        memcpy(out_value, e->data, copy_len);
        *length = e->data_len;
    }
    return ESP_OK;
}

/* ================================================================
 * SECTION 4: Event / WiFi / Netif Mocks (stubs)
 * ================================================================ */

const char *WIFI_EVENT = "WIFI_EVENT";
const char *IP_EVENT   = "IP_EVENT";

esp_err_t esp_event_loop_create_default(void) { return ESP_OK; }

esp_err_t esp_event_handler_instance_register(esp_event_base_t event_base,
                                               int32_t event_id,
                                               esp_event_handler_t event_handler,
                                               void *event_handler_arg,
                                               esp_event_handler_instance_t *context)
{
    (void)event_base; (void)event_id; (void)event_handler;
    (void)event_handler_arg; (void)context;
    return ESP_OK;
}

esp_netif_t *esp_netif_create_default_wifi_sta(void) { return NULL; }
esp_err_t esp_netif_init(void) { return ESP_OK; }

esp_err_t esp_wifi_init(const wifi_init_config_t *config)
{ (void)config; return ESP_OK; }

esp_err_t esp_wifi_set_mode(wifi_mode_t mode)
{ (void)mode; return ESP_OK; }

esp_err_t esp_wifi_set_config(wifi_interface_t iface, wifi_config_t *conf)
{ (void)iface; (void)conf; return ESP_OK; }

esp_err_t esp_wifi_start(void)   { return ESP_OK; }
esp_err_t esp_wifi_stop(void)    { return ESP_OK; }
esp_err_t esp_wifi_connect(void) { return ESP_OK; }
esp_err_t esp_wifi_disconnect(void) { return ESP_OK; }

/* ================================================================
 * SECTION 5: MQTT Mock
 * ================================================================ */

struct esp_mqtt_client {
    esp_event_handler_t_mqtt event_handler;
    void                    *event_handler_arg;
    int                      connected;
    int                      msg_id_counter;
    int                      publish_count;
    char                     uri[128];
    char                     client_id[64];
};

static struct esp_mqtt_client s_mqtt_mock_client;

esp_mqtt_client_handle_t esp_mqtt_client_init(const esp_mqtt_client_config_t *config)
{
    memset(&s_mqtt_mock_client, 0, sizeof(s_mqtt_mock_client));
    if (config && config->broker.address.uri) {
        strncpy(s_mqtt_mock_client.uri, config->broker.address.uri,
                sizeof(s_mqtt_mock_client.uri) - 1);
    }
    if (config && config->credentials.client_id) {
        strncpy(s_mqtt_mock_client.client_id, config->credentials.client_id,
                sizeof(s_mqtt_mock_client.client_id) - 1);
    }
    s_mqtt_mock_client.connected = 0;
    s_mqtt_mock_client.msg_id_counter = 0;
    s_mqtt_mock_client.publish_count  = 0;

    printf("[SIM] MQTT client initialized: uri=%s client_id=%s\n",
           s_mqtt_mock_client.uri, s_mqtt_mock_client.client_id);

    return &s_mqtt_mock_client;
}

esp_err_t esp_mqtt_client_register_event(esp_mqtt_client_handle_t client,
                                          int event,
                                          esp_event_handler_t_mqtt event_handler,
                                          void *event_handler_arg)
{
    (void)event;
    if (!client) return ESP_FAIL;
    client->event_handler     = event_handler;
    client->event_handler_arg = event_handler_arg;
    return ESP_OK;
}

static void mqtt_dispatch_event(struct esp_mqtt_client *client,
                                 esp_mqtt_event_id_t event_id)
{
    if (!client || !client->event_handler) return;

    struct esp_mqtt_event evt;
    memset(&evt, 0, sizeof(evt));
    evt.client   = client;
    evt.event_id = (int)event_id;

    client->event_handler(client->event_handler_arg,
                           "MQTT_EVENTS",
                           (int32_t)event_id,
                           &evt);
}

esp_err_t esp_mqtt_client_start(esp_mqtt_client_handle_t client)
{
    if (!client) return ESP_FAIL;
    client->connected = 1;
    printf("[SIM] MQTT client started (connected)\n");

    /* Fire CONNECTED event so the module's handler sets its state */
    mqtt_dispatch_event(client, MQTT_EVENT_CONNECTED);

    return ESP_OK;
}

esp_err_t esp_mqtt_client_stop(esp_mqtt_client_handle_t client)
{
    if (!client) return ESP_FAIL;
    client->connected = 0;
    printf("[SIM] MQTT client stopped (disconnected)\n");

    mqtt_dispatch_event(client, MQTT_EVENT_DISCONNECTED);
    return ESP_OK;
}

int esp_mqtt_client_publish(esp_mqtt_client_handle_t client,
                             const char *topic,
                             const char *data,
                             int len,
                             int qos,
                             int retain)
{
    (void)retain;
    if (!client) return -1;

    client->msg_id_counter++;
    int msg_id = client->msg_id_counter;

    if (client->connected) {
        client->publish_count++;
        int payload_len = (len > 0) ? len : (int)strlen(data);
        printf("[MQTT PUB] topic=%-40s qos=%d  msg_id=%d  payload(%d bytes): %.120s%s\n",
               topic, qos, msg_id, payload_len,
               data, (payload_len > 120) ? "..." : "");
        return msg_id;
    } else {
        printf("[MQTT PUB] OFFLINE - topic=%s (data discarded)\n", topic);
        return -1;
    }
}

int esp_mqtt_client_subscribe(esp_mqtt_client_handle_t client,
                               const char *topic,
                               int qos)
{
    if (!client) return -1;
    client->msg_id_counter++;
    printf("[SIM] MQTT subscribe: topic=%s qos=%d msg_id=%d\n",
           topic, qos, client->msg_id_counter);
    return client->msg_id_counter;
}

void esp_mqtt_client_destroy(esp_mqtt_client_handle_t client)
{
    if (!client) return;
    printf("[SIM] MQTT client destroyed (total published: %d)\n",
           client->publish_count);
    memset(client, 0, sizeof(struct esp_mqtt_client));
}

bool sim_mqtt_mock_is_connected(void)
{
    return s_mqtt_mock_client.connected != 0;
}

int sim_mqtt_mock_publish_count(void)
{
    return s_mqtt_mock_client.publish_count;
}

/* ================================================================
 * SECTION 6: MODBUS Mock (simulated sensor data)
 * ================================================================ */

static int      s_modbus_initialized = 0;
static uint32_t s_modbus_timeout_ms  = 1000;
static uint32_t s_modbus_request_count = 0;

static float random_variation(float pct)
{
    float r = (float)(rand() % 1000) / 1000.0f;
    return (r * 2.0f - 1.0f) * pct;
}

static void float_to_regs(float value, uint16_t *regs)
{
    uint32_t raw;
    memcpy(&raw, &value, sizeof(float));
    regs[0] = (uint16_t)(raw >> 16);
    regs[1] = (uint16_t)(raw & 0xFFFF);
}

esp_err_t mbc_master_init(mb_port_type_t port_type, void **handler)
{
    (void)port_type;
    if (handler) *handler = (void *)(uintptr_t)0x12345678;
    printf("[SIM] MODBUS master initialized (port_type=%d)\n", (int)port_type);
    return ESP_OK;
}

esp_err_t mbc_master_setup(const mb_communication_info_t *comm_info)
{
    if (comm_info) {
        printf("[SIM] MODBUS setup: mode=%d baud=%u parity=%d\n",
               (int)comm_info->mode, comm_info->baudrate, (int)comm_info->parity);
    }
    return ESP_OK;
}

esp_err_t mbc_master_set_pin(const uart_pin_config_t *pin_config)
{
    if (pin_config) {
        printf("[SIM] MODBUS pins: TX=%d RX=%d RTS=%d CTS=%d\n",
               pin_config->tx_io_num, pin_config->rx_io_num,
               pin_config->rts_io_num, pin_config->cts_io_num);
    }
    return ESP_OK;
}

esp_err_t mbc_master_start(void)
{
    s_modbus_initialized = 1;
    printf("[SIM] MODBUS master started\n");
    return ESP_OK;
}

esp_err_t mbc_master_stop(void)
{
    s_modbus_initialized = 0;
    return ESP_OK;
}

esp_err_t mbc_master_destroy(void)
{
    s_modbus_initialized = 0;
    printf("[SIM] MODBUS master destroyed (total requests: %u)\n",
           s_modbus_request_count);
    return ESP_OK;
}

void mbc_master_set_timeout(uint32_t timeout_ms)
{
    s_modbus_timeout_ms = timeout_ms;
    printf("[SIM] MODBUS timeout set to %u ms\n", timeout_ms);
}

/**
 * Simulate a MODBUS master request.
 *
 * Based on slave_addr and reg_start, fills the output buffer (via param_value
 * in the descriptor) with realistic sensor data including small random variations.
 *
 * Register mapping:
 *   slave=1, reg=0x0000 (40001): float32 ~72.5 degC  (motor temperature)
 *   slave=1, reg=0x0002 (40003): float32 ~4.2 bar     (line pressure)
 *   slave=1, reg=0x0004:         uint16 ~850           (humidity %)
 *   slave=2, reg=0x0000 (40001): uint16/float ~1500    (conveyor speed)
 *   slave=2, reg=0x0002 (40003): int16  ~25 A          (motor current)
 *   slave=2, reg=0x0010:         uint32 ~125000        (total count)
 */
esp_err_t mbc_master_send_request(mb_request_param_t *req,
                                   mb_parameter_descriptor_t *reg_info)
{
    if (!req || !reg_info) return ESP_ERR_INVALID_ARG;
    if (!s_modbus_initialized) return ESP_ERR_INVALID_STATE;

    s_modbus_request_count++;

    uint8_t  slave  = req->slave_addr;
    uint16_t addr   = req->reg_start;
    uint16_t cnt    = req->reg_size;
    void    *out    = reg_info->param_value;

    if (!out) return ESP_ERR_INVALID_ARG;

    uint16_t *regs = (uint16_t *)out;

    /* Generate simulated data based on slave and address */
    if (slave == 1 && addr == 0x0000) {
        /* Motor temperature: float32 ~72.5 degC, +/-1% variation */
        float temp = 72.5f + random_variation(0.725f);
        float_to_regs(temp, regs);
    }
    else if (slave == 1 && addr == 0x0002) {
        /* Line pressure: float32 ~4.2 bar, +/-2% variation */
        float pressure = 4.2f + random_variation(0.084f);
        float_to_regs(pressure, regs);
    }
    else if (slave == 1 && addr == 0x0004) {
        /* Humidity: uint16 ~850 */
        regs[0] = (uint16_t)(850 + (rand() % 20) - 10);
    }
    else if (slave == 2 && addr == 0x0000) {
        /* Conveyor speed */
        if (cnt >= 2) {
            float speed = 1500.0f + random_variation(15.0f);
            float_to_regs(speed, regs);
        } else {
            regs[0] = (uint16_t)(1500 + (rand() % 30) - 15);
        }
    }
    else if (slave == 2 && addr == 0x0002) {
        /* Motor current: int16 ~25 A */
        regs[0] = (uint16_t)(int16_t)(25 + (rand() % 3) - 1);
    }
    else if (slave == 2 && addr == 0x0010) {
        /* Total count: uint32 ~125000 */
        uint32_t count_val = 125000 + (uint32_t)(rand() % 1000);
        regs[0] = (uint16_t)(count_val >> 16);
        regs[1] = (uint16_t)(count_val & 0xFFFF);
    }
    else {
        /* Unknown register: fill with zeros (still return OK) */
        memset(regs, 0, cnt * sizeof(uint16_t));
    }

    /* For write operations, just log and succeed */
    if (req->command == MB_CMD_WRITE_HOLDING ||
        req->command == MB_CMD_WRITE_MULTIPLE) {
        printf("[SIM] MODBUS WRITE: slave=%u addr=0x%04X cnt=%u\n",
               slave, addr, cnt);
    }

    return ESP_OK;
}

/* ================================================================
 * END OF MOCK IMPLEMENTATIONS
 * ================================================================ */
