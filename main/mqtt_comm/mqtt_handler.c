/**
 * @file mqtt_handler.c
 * @brief MQTT communication layer implementation using ESP-IDF esp_mqtt client.
 *
 * Manages the MQTT connection lifecycle, handles incoming messages by dispatching
 * them to registered callbacks, and provides thread-safe publish/subscribe APIs
 * with internal metrics tracking.
 */

#include "mqtt_handler.h"

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "mqtt_client.h"
#include "esp_crt_bundle.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_heap_caps.h"
#include "config/runtime_config.h"
#include "cloud_adapter/cloud_adapter.h"
#include "thingscloud/thingscloud_topics.h"
#include "web/web_server.h"

/* ======================== Logging Tag ======================== */

static const char *TAG = "MQTT";

/* ======================== Internal Types ======================== */

/**
 * @brief Single subscription entry: topic filter paired with a callback.
 */
typedef struct {
    char                topic[MQTT_MAX_TOPIC_LEN];
    int                 qos;
    mqtt_cmd_callback_t callback;
    bool                active;
} mqtt_subscription_t;


/* ======================== Static State ======================== */

static esp_mqtt_client_handle_t s_mqtt_client = NULL;
static mqtt_conn_state_t        s_conn_state  = MQTT_STATE_DISCONNECTED;
static char                     s_last_connect_reason[160] = "not connected";
static mqtt_mode_t              s_mqtt_mode   = MQTT_MODE_STANDARD;
static const char              *s_broker_uri  = MQTT_BROKER_URI;
static const char              *s_client_id   = MQTT_CLIENT_ID;
static const char              *s_username    = MQTT_USERNAME;
static const char              *s_password    = MQTT_PASSWORD;
static char s_runtime_broker[128];
static char s_runtime_client_id[48];
static char s_runtime_username[64];
static char s_runtime_password[96];

/* Subscription table */
static mqtt_subscription_t      s_subscriptions[MQTT_MAX_SUBSCRIPTIONS];
static int                      s_subscription_count = 0;

/* Metrics */
static int                      s_publish_success_count = 0;
static int                      s_publish_fail_count    = 0;
static int64_t                  s_last_publish_time_ms  = 0;
static int64_t                  s_connected_since_ms    = 0;
static int64_t                  s_last_session_duration_ms = 0;
static uint32_t                 s_connect_count = 0;
static uint32_t                 s_disconnect_count = 0;
static uint32_t                 s_error_count = 0;
static bool                     s_suspended = false;
static uint8_t                  s_consecutive_auth_failures = 0;

/* Thread-safety mutex */
static SemaphoreHandle_t        s_mqtt_mutex = NULL;
/* Serializes client start/stop/destroy/restart across Web, reconnect and
 * broker-event tasks. ESP-MQTT client destruction is not concurrency-safe. */
static SemaphoreHandle_t        s_lifecycle_mutex = NULL;
static mqtt_publish_ack_callback_t s_ack_callback = NULL;

typedef struct {
    int msg_id;
    uint32_t token;
    bool active;
} tracked_publish_t;

#define MQTT_MAX_TRACKED_PUBLISHES 32
static tracked_publish_t s_tracked[MQTT_MAX_TRACKED_PUBLISHES];

/* Initialization flag */
static bool                     s_initialized = false;
static bool                     s_client_started = false;
static void                    *s_task_stack_reserve = NULL;
#define MQTT_SUPERVISOR_STACK_BYTES 6144
static StaticTask_t             s_supervisor_task_tcb;
static StackType_t              s_supervisor_task_stack[
    MQTT_SUPERVISOR_STACK_BYTES / sizeof(StackType_t)];
static TaskHandle_t             s_supervisor_task_handle = NULL;

/* ======================== Internal Helpers ======================== */

static inline void mqtt_lock(void);
static inline void mqtt_unlock(void);
static void mqtt_set_connect_reason(const char *reason);

static void mqtt_reserve_future_task_stack(void)
{
    if (s_task_stack_reserve != NULL) return;
    s_task_stack_reserve = heap_caps_malloc(
        MQTT_CLIENT_TASK_STACK_SIZE + 2048,
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (s_task_stack_reserve == NULL) {
        ESP_LOGE(TAG, "Unable to reserve internal SRAM for future MQTT start");
        web_server_add_log(
            "error",
            "[MQTT] Unable to reserve internal SRAM for future client start");
    } else {
        ESP_LOGI(TAG, "Reserved %u bytes of internal SRAM for MQTT task start",
                 (unsigned)(MQTT_CLIENT_TASK_STACK_SIZE + 2048));
    }
}

static void mqtt_release_task_stack_reserve(void)
{
    if (s_task_stack_reserve == NULL) return;
    heap_caps_free(s_task_stack_reserve);
    s_task_stack_reserve = NULL;
}

static void mqtt_supervisor_task(void *arg)
{
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(50));
        char suspend_reason[sizeof(s_last_connect_reason)];
        mqtt_lock();
        strlcpy(suspend_reason, s_last_connect_reason,
                sizeof(suspend_reason));
        mqtt_unlock();
        (void)mqtt_disconnect();
        mqtt_set_connect_reason(suspend_reason);
    }
}

static bool mqtt_start_supervisor(void)
{
    if (s_supervisor_task_handle != NULL) return true;
    s_supervisor_task_handle = xTaskCreateStatic(
        mqtt_supervisor_task,
        "mqtt_supervisor",
        sizeof(s_supervisor_task_stack) / sizeof(s_supervisor_task_stack[0]),
        NULL,
        9,
        s_supervisor_task_stack,
        &s_supervisor_task_tcb);
    if (s_supervisor_task_handle == NULL) {
        ESP_LOGE(TAG, "Unable to create static MQTT supervisor task");
        return false;
    }
    return true;
}

/**
 * @brief Lock the internal mutex (blocking, portMAX_DELAY).
 */
static inline void mqtt_lock(void)
{
    if (s_mqtt_mutex) {
        xSemaphoreTake(s_mqtt_mutex, portMAX_DELAY);
    }
}

/**
 * @brief Unlock the internal mutex.
 */
static inline void mqtt_unlock(void)
{
    if (s_mqtt_mutex) {
        xSemaphoreGive(s_mqtt_mutex);
    }
}

/**
 * @brief Set the connection state under the mutex.
 */
static void mqtt_set_state(mqtt_conn_state_t state)
{
    mqtt_lock();
    s_conn_state = state;
    mqtt_unlock();
}

static void mqtt_set_connect_reason(const char *reason)
{
    mqtt_lock();
    strlcpy(s_last_connect_reason, reason ? reason : "",
            sizeof(s_last_connect_reason));
    mqtt_unlock();
}

/**
 * @brief Find a registered subscription whose topic filter matches an
 *        incoming topic string. Returns the subscription index, or -1
 *        if no match is found.
 *
 * Note: This performs an exact string match. For production use with
 * MQTT wildcards (+/#), a full topic-filter matching algorithm should
 * be implemented per the MQTT 3.1.1 specification section 4.7.
 */
static int mqtt_find_subscription(const char *topic, int topic_len)
{
    for (int i = 0; i < s_subscription_count; i++) {
        if (!s_subscriptions[i].active) {
            continue;
        }
        /* Exact match first (most common case) */
        if ((int)strlen(s_subscriptions[i].topic) == topic_len &&
            memcmp(s_subscriptions[i].topic, topic, topic_len) == 0) {
            return i;
        }
        /*
         * Simple prefix match for wildcard topic filters ending with '#'.
         * e.g. "factory/cmd/#" matches "factory/cmd/set_temp"
         */
        size_t filter_len = strlen(s_subscriptions[i].topic);
        if (filter_len > 0 && s_subscriptions[i].topic[filter_len - 1] == '#') {
            /* Compare everything before the '#' */
            size_t prefix_len = filter_len - 1;
            if ((int)prefix_len <= topic_len &&
                memcmp(s_subscriptions[i].topic, topic, prefix_len) == 0) {
                return i;
            }
        }
    }
    return -1;
}

/* ======================== MQTT Event Handler ======================== */

/**
 * @brief ESP-IDF MQTT event handler.
 *
 * Called by the mqtt client task for every broker event (connect, disconnect,
 * data received, error, etc.).  We update internal state and dispatch data
 * to the registered subscription callbacks.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    (void)handler_args;
    (void)base;

    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id) {

    case MQTT_EVENT_CONNECTED: {
        ESP_LOGI(TAG, "Connected to MQTT broker: %s", s_broker_uri);
        s_connected_since_ms = esp_timer_get_time() / 1000;
        s_connect_count++;
        s_consecutive_auth_failures = 0;
        mqtt_set_state(MQTT_STATE_CONNECTED);
        mqtt_set_connect_reason("connected to broker");
        web_server_add_log("ok", "[MQTT] CONNECT accepted by broker");

        /* Re-subscribe to all registered topics on (re)connect. */
        mqtt_lock();
        for (int i = 0; i < s_subscription_count; i++) {
            if (s_subscriptions[i].active) {
                int msg_id = esp_mqtt_client_subscribe(client,
                                                       s_subscriptions[i].topic,
                                                       s_subscriptions[i].qos);
                ESP_LOGI(TAG, "Re-subscribed to '%s' (msg_id=%d, qos=%d)",
                         s_subscriptions[i].topic, msg_id,
                         s_subscriptions[i].qos);
            }
        }
        mqtt_unlock();
        runtime_config_t runtime;
        runtime_config_get(&runtime);
        if (runtime.mqtt.lwt_enabled && runtime.mqtt.lwt_topic[0] != '\0') {
            esp_mqtt_client_publish(client, runtime.mqtt.lwt_topic, "online", 0,
                                    runtime.mqtt.lwt_qos,
                                    runtime.mqtt.lwt_retain);
        }
        if (runtime.mqtt.platform_type == MQTT_PLATFORM_THINGSCLOUD) {
            thingscloud_on_mqtt_connected();
        }
        break;
    }

    case MQTT_EVENT_DISCONNECTED: {
        int64_t now_ms = esp_timer_get_time() / 1000;
        if (s_connected_since_ms > 0) {
            s_last_session_duration_ms = now_ms - s_connected_since_ms;
        }
        s_disconnect_count++;
        ESP_LOGW(TAG, "Disconnected from MQTT broker");
        s_connected_since_ms = 0;
        mqtt_set_state(MQTT_STATE_DISCONNECTED);
        mqtt_lock();
        bool reconnect_suspended = s_suspended;
        mqtt_unlock();
        char log_text[144];
        snprintf(log_text, sizeof(log_text),
                 reconnect_suspended
                    ? "[MQTT] Broker transport closed after %lld ms; reconnect disabled"
                    : "[MQTT] Broker transport closed after %lld ms; automatic reconnect pending",
                 (long long)s_last_session_duration_ms);
        web_server_add_log("warn", log_text);
        break;
    }

    case MQTT_EVENT_DATA: {
        ESP_LOGD(TAG, "Received message on topic '%.*s' (%d bytes)",
                 event->topic_len, event->topic, event->data_len);

        /* Find matching subscription and invoke callback */
        mqtt_lock();
        int idx = mqtt_find_subscription(event->topic, event->topic_len);
        if (idx >= 0 && s_subscriptions[idx].callback != NULL) {
            mqtt_cmd_callback_t cb = s_subscriptions[idx].callback;
            mqtt_unlock();

            /* Invoke callback outside of lock to avoid deadlocks */
            cb(event->topic, event->data, event->data_len);
        } else {
            mqtt_unlock();
            ESP_LOGW(TAG, "No registered callback for topic '%.*s'",
                     event->topic_len, event->topic);
        }
        break;
    }

    case MQTT_EVENT_SUBSCRIBED: {
        ESP_LOGD(TAG, "Subscription acknowledged (msg_id=%d)", event->msg_id);
        break;
    }

    case MQTT_EVENT_UNSUBSCRIBED: {
        ESP_LOGD(TAG, "Unsubscribe acknowledged (msg_id=%d)", event->msg_id);
        break;
    }

    case MQTT_EVENT_PUBLISHED: {
        ESP_LOGD(TAG, "Publish acknowledged (msg_id=%d)", event->msg_id);
        mqtt_publish_ack_callback_t callback = NULL;
        uint32_t token = 0;
        mqtt_lock();
        for (int i = 0; i < MQTT_MAX_TRACKED_PUBLISHES; ++i) {
            if (s_tracked[i].active && s_tracked[i].msg_id == event->msg_id) {
                token = s_tracked[i].token;
                s_tracked[i].active = false;
                callback = s_ack_callback;
                break;
            }
        }
        mqtt_unlock();
        if (callback != NULL) callback(token);
        break;
    }

    case MQTT_EVENT_ERROR: {
        ESP_LOGE(TAG, "MQTT error event");
        s_error_count++;
        bool suspend_for_auth = false;

        if (event->error_handle) {
            esp_mqtt_error_codes_t *err = event->error_handle;
            if (err->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                char reason[160];
                snprintf(reason, sizeof(reason),
                         "network/TCP error (sock_errno=%d esp_err=0x%x)",
                         err->esp_transport_sock_errno,
                         (unsigned int)err->esp_tls_last_esp_err);
                mqtt_set_connect_reason(reason);
                ESP_LOGE(TAG, "Transport error: esp_tls=%d (0x%x), tls_err=%d (0x%x), sock_errno=%d",
                         err->esp_tls_last_esp_err, err->esp_tls_last_esp_err,
                         err->esp_tls_stack_err, err->esp_tls_stack_err,
                         err->esp_transport_sock_errno);
            } else if (err->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
                char reason[160];
                if (err->connect_return_code == 5) {
                    if (s_consecutive_auth_failures < UINT8_MAX) {
                        s_consecutive_auth_failures++;
                    }
                    suspend_for_auth = s_consecutive_auth_failures >= 3;
                    strlcpy(reason,
                            "broker rejected authentication (CONNACK 5): verify AccessToken and ProjectKey",
                            sizeof(reason));
                } else {
                    snprintf(reason, sizeof(reason),
                             "broker refused connection (CONNACK %d)",
                             err->connect_return_code);
                }
                mqtt_set_connect_reason(reason);
                ESP_LOGE(TAG, "Connection refused, error code: 0x%x",
                         err->connect_return_code);
            } else {
                ESP_LOGE(TAG, "Error type: %d", err->error_type);
            }
        }
        char error_log[192];
        mqtt_lock();
        snprintf(error_log, sizeof(error_log), "[MQTT] Error: %.160s",
                 s_last_connect_reason);
        mqtt_unlock();
        web_server_add_log("error", error_log);
        mqtt_set_state(MQTT_STATE_ERROR);
        if (suspend_for_auth) {
            web_server_add_log(
                "error",
                "[MQTT] Reconnect suspended after 3 authentication rejections");
            mqtt_suspend_async(
                "reconnect suspended after 3 authentication rejections");
        }
        break;
    }

    case MQTT_EVENT_BEFORE_CONNECT:
        ESP_LOGI(TAG, "Starting MQTT CONNECT attempt");
        web_server_add_log("info", "[MQTT] Starting CONNECT attempt");
        break;

    default:
        ESP_LOGD(TAG, "Unhandled MQTT event: %ld", event_id);
        break;
    }
}

/* ======================== Public API ======================== */

static void mqtt_load_runtime_settings(runtime_config_t *runtime)
{
#if ONENET_ENABLED
    s_mqtt_mode   = MQTT_MODE_ONENET;
    s_broker_uri  = ONENET_MQTT_BROKER_URI;
    s_client_id   = ONENET_DEVICE_ID;
    s_username    = ONENET_PRODUCT_ID;
    s_password    = ONENET_DEVICE_KEY;
#else
    s_mqtt_mode = MQTT_MODE_STANDARD;
    strlcpy(s_runtime_broker, runtime->mqtt.uri, sizeof(s_runtime_broker));
    strlcpy(s_runtime_client_id, runtime->mqtt.client_id,
            sizeof(s_runtime_client_id));
    strlcpy(s_runtime_username, runtime->mqtt.username,
            sizeof(s_runtime_username));
    strlcpy(s_runtime_password, runtime->mqtt.password,
            sizeof(s_runtime_password));
    s_broker_uri = s_runtime_broker;
    s_client_id = s_runtime_client_id;
    s_username = s_runtime_username;
    s_password = s_runtime_password;

    if (runtime->mqtt.platform_type == MQTT_PLATFORM_THINGSCLOUD) {
        runtime->mqtt.qos = 0;
        runtime->mqtt.retain = false;
        runtime->mqtt.lwt_enabled = false;
        if (s_runtime_client_id[0] == '\0') {
            uint8_t mac[6];
            if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
                snprintf(s_runtime_client_id, sizeof(s_runtime_client_id),
                         "GW_%02X%02X%02X%02X%02X%02X",
                         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
            } else {
                strlcpy(s_runtime_client_id, MQTT_CLIENT_ID,
                        sizeof(s_runtime_client_id));
            }
            s_client_id = s_runtime_client_id;
        }
    }
#endif
}

static esp_mqtt_client_config_t mqtt_build_client_config(
    const runtime_config_t *runtime, bool disable_auto_reconnect)
{
    bool clean_session = runtime->mqtt.clean_session;
    if (runtime->mqtt.platform_type == MQTT_PLATFORM_THINGSCLOUD) {
        clean_session = true;
    }

    esp_mqtt_client_config_t config = {
        .broker = {
            .address = {
                .uri = s_broker_uri,
            },
            .verification = {
                .crt_bundle_attach = esp_crt_bundle_attach,
            },
        },
        .credentials = {
            .client_id = s_client_id,
        },
        .session = {
            .protocol_ver = MQTT_PROTOCOL_V_3_1_1,
            .keepalive = runtime->mqtt.keepalive_sec,
            .disable_clean_session = !clean_session,
            .last_will = {
                .topic = runtime->mqtt.lwt_enabled
                    ? runtime->mqtt.lwt_topic : NULL,
                .msg = runtime->mqtt.lwt_enabled
                    ? runtime->mqtt.lwt_payload : NULL,
                .qos = runtime->mqtt.lwt_qos,
                .retain = runtime->mqtt.lwt_retain,
            },
        },
        .network = {
            .timeout_ms = MQTT_CONNECT_TIMEOUT_MS,
            .reconnect_timeout_ms = 30000,
            .disable_auto_reconnect = disable_auto_reconnect,
        },
        .task = {
            .stack_size = MQTT_CLIENT_TASK_STACK_SIZE,
            .priority = 8,
        },
    };

    if (s_username[0] != '\0') {
        config.credentials.username = s_username;
    }
    if (s_password[0] != '\0') {
        config.credentials.authentication.password = s_password;
    }
    return config;
}

static void mqtt_register_runtime_subscriptions(const runtime_config_t *runtime)
{
    mqtt_lock();
    s_subscription_count = 0;
    mqtt_unlock();

    if (runtime->mqtt.platform_type == MQTT_PLATFORM_THINGSCLOUD) {
        mqtt_subscribe(TC_TOPIC_ATTR_RESPONSE, 0,
                       thingscloud_on_attributes_response);
        mqtt_subscribe(TC_TOPIC_ATTR_PUSH, 0,
                       thingscloud_on_gateway_attributes_push);
        if (runtime->mqtt.report_mode == MQ_REPORT_SUBDEVICE) {
            mqtt_subscribe(TC_TOPIC_GATEWAY_ATTR_RESPONSE, 0,
                           thingscloud_on_gateway_attributes_response);
            mqtt_subscribe(TC_TOPIC_GATEWAY_ATTR_PUSH, 0,
                           thingscloud_on_attributes_push);
            mqtt_subscribe(TC_TOPIC_GATEWAY_CMD_SEND, 0,
                           thingscloud_on_command_send);
            ESP_LOGI(TAG,
                     "ThingsCloud gateway/sub-device subscriptions registered");
        } else {
            ESP_LOGI(TAG,
                     "ThingsCloud gateway-attribute subscription registered");
        }
    } else {
        char command_topic[128];
        snprintf(command_topic, sizeof(command_topic), "%s%s",
                 runtime->mqtt.command_prefix, runtime->gateway_id);
        mqtt_subscribe(command_topic, 1, mqtt_command_callback);
    }
}

static void mqtt_init_unlocked(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "MQTT handler already initialized");
        return;
    }

    runtime_config_t runtime;
    runtime_config_get(&runtime);
    s_consecutive_auth_failures = 0;
    bool configured = true;
    bool should_start = true;
#if ONENET_ENABLED
    s_mqtt_mode   = MQTT_MODE_ONENET;
    s_broker_uri  = ONENET_MQTT_BROKER_URI;
    s_client_id   = ONENET_DEVICE_ID;
    s_username    = ONENET_PRODUCT_ID;
    s_password    = ONENET_DEVICE_KEY;
    ESP_LOGI(TAG, "Initializing MQTT handler in OneNET mode (product: %s, device: %s)",
             ONENET_PRODUCT_ID, ONENET_DEVICE_ID);
#else
    configured = runtime.mqtt.uri[0] != '\0' &&
        !(strcmp(runtime.mqtt.uri, MQTT_BROKER_URI) == 0 &&
          runtime.mqtt.username[0] == '\0' &&
          runtime.mqtt.password[0] == '\0');
    should_start = runtime.mqtt.enabled && configured;
    s_suspended = !should_start;
    mqtt_load_runtime_settings(&runtime);
    ESP_LOGI(TAG, "Initializing MQTT handler (broker: %s)", s_broker_uri);
    if (!runtime.mqtt.enabled) {
        ESP_LOGW(TAG,
                 "MQTT is disabled; preparing a dormant client for later configuration");
    } else if (!configured) {
        ESP_LOGW(TAG,
                 "MQTT is unconfigured; preparing a dormant client for later configuration");
    }

    /* ThingsCloud: enforce QoS/retain constraints and derive a unique client id
       from the ESP32 MAC when the user left it empty. The broker URI (scheme
       and port) is taken verbatim from the user configuration, because the
       ThingsCloud instance in use serves MQTT on the plaintext 1883 port and
       rejects TLS on 8883. */
    if (runtime.mqtt.platform_type == MQTT_PLATFORM_THINGSCLOUD) {
        ESP_LOGI(TAG, "ThingsCloud broker (as configured): %s", s_broker_uri);
        /* Keep a stable device-scoped ClientId across reconnects. ThingsCloud
           permits arbitrary ClientIds, but an industrial gateway should not
           rely on broker-assigned transient session identity. */
        ESP_LOGI(TAG, "ThingsCloud MQTT ClientId: %s", s_client_id);
    }
#endif

    /* Create the mutex once; it lives for the module lifetime so that
       mqtt_destroy()/mqtt_restart() can never delete a mutex another task
       might be holding. */
    if (s_mqtt_mutex == NULL) {
        s_mqtt_mutex = xSemaphoreCreateMutex();
        if (s_mqtt_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create mutex");
            s_conn_state = MQTT_STATE_ERROR;
            return;
        }
    }

    /* Per-client publish tracking is reset on (re)init. The subscription
       table is intentionally preserved across (re)init so that a
       (re)connect re-subscribes previously registered topics (e.g. the
       command topic); see the MQTT_EVENT_CONNECTED handler. */
    memset(s_tracked, 0, sizeof(s_tracked));

    /* Reset metrics */
    s_publish_success_count = 0;
    s_publish_fail_count    = 0;
    s_last_publish_time_ms  = 0;

    esp_mqtt_client_config_t mqtt_cfg =
        mqtt_build_client_config(&runtime, false);

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (s_mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to create MQTT client");
        s_conn_state = MQTT_STATE_ERROR;
        return;
    }

    /* Register event handler */
    esp_err_t err = esp_mqtt_client_register_event(s_mqtt_client,
                                                    MQTT_EVENT_ANY,
                                                    mqtt_event_handler,
                                                    NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register event handler: %s", esp_err_to_name(err));
        esp_mqtt_client_destroy(s_mqtt_client);
        s_mqtt_client = NULL;
        s_conn_state = MQTT_STATE_ERROR;
        return;
    }

    s_initialized = true;
    s_client_started = false;
    mqtt_set_state(MQTT_STATE_DISCONNECTED);
    mqtt_register_runtime_subscriptions(&runtime);

    if (!should_start) {
        mqtt_reserve_future_task_stack();
        mqtt_set_connect_reason(runtime.mqtt.enabled
            ? "unconfigured; resident MQTT client is waiting for configuration"
            : "disabled; resident MQTT client is waiting for configuration");
        return;
    }

    mqtt_release_task_stack_reserve();
    err = esp_mqtt_client_start(s_mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(err));
        mqtt_set_connect_reason(
            "unable to create MQTT task: insufficient contiguous internal SRAM");
        web_server_add_log(
            "error",
            "[MQTT] Client task start failed: insufficient contiguous internal SRAM");
        mqtt_reserve_future_task_stack();
        s_conn_state = MQTT_STATE_ERROR;
        return;
    }

    s_client_started = true;
    mqtt_set_connect_reason("MQTT client started; waiting for broker");
    ESP_LOGI(TAG, "MQTT client started successfully");
}

void mqtt_init(void)
{
    if (!mqtt_start_supervisor()) {
        mqtt_set_state(MQTT_STATE_ERROR);
        return;
    }
    if (s_lifecycle_mutex == NULL) {
        s_lifecycle_mutex = xSemaphoreCreateMutex();
        if (s_lifecycle_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create MQTT lifecycle mutex");
            mqtt_set_state(MQTT_STATE_ERROR);
            return;
        }
    }
    xSemaphoreTake(s_lifecycle_mutex, portMAX_DELAY);
    mqtt_init_unlocked();
    xSemaphoreGive(s_lifecycle_mutex);
}

esp_err_t mqtt_publish(const char *topic, const char *payload, int qos)
{
    if (!s_initialized || s_mqtt_client == NULL) {
        ESP_LOGE(TAG, "Cannot publish: MQTT not initialized");
        return ESP_FAIL;
    }

    if (topic == NULL || payload == NULL) {
        ESP_LOGE(TAG, "Cannot publish: NULL topic or payload");
        return ESP_ERR_INVALID_ARG;
    }

    int payload_len = (int)strlen(payload);

    runtime_config_t runtime;
    runtime_config_get(&runtime);
    int msg_id = esp_mqtt_client_publish(s_mqtt_client, topic, payload,
                                         payload_len, qos, runtime.mqtt.retain);

    mqtt_lock();
    if (msg_id >= 0) {
        s_publish_success_count++;
        s_last_publish_time_ms = esp_timer_get_time() / 1000; /* us -> ms */
        mqtt_unlock();
        ESP_LOGD(TAG, "Published to '%s' (msg_id=%d, qos=%d, %d bytes)",
                 topic, msg_id, qos, payload_len);
        char log_text[128];
        snprintf(log_text, sizeof(log_text),
                 "[MQTT TX] topic=%s bytes=%d payload=%.48s%s",
                 topic, payload_len, payload,
                 payload_len > 48 ? "..." : "");
        web_server_add_log("ok", log_text);
        return ESP_OK;
    } else {
        s_publish_fail_count++;
        mqtt_unlock();
        ESP_LOGW(TAG, "Failed to publish to '%s' (qos=%d, %d bytes)",
                 topic, qos, payload_len);
        char log_text[128];
        snprintf(log_text, sizeof(log_text),
                 "[MQTT TX FAIL] topic=%s bytes=%d", topic, payload_len);
        web_server_add_log("error", log_text);
        return ESP_FAIL;
    }
}

esp_err_t mqtt_publish_tracked(const char *topic, const char *payload, int qos, uint32_t token)
{
    if (!s_initialized || s_mqtt_client == NULL || topic == NULL || payload == NULL || qos < 1) {
        return ESP_ERR_INVALID_ARG;
    }
    runtime_config_t runtime;
    runtime_config_get(&runtime);
    int msg_id = esp_mqtt_client_publish(s_mqtt_client, topic, payload, 0, qos,
                                         runtime.mqtt.retain);
    if (msg_id < 0) return ESP_FAIL;

    mqtt_lock();
    int slot = -1;
    for (int i = 0; i < MQTT_MAX_TRACKED_PUBLISHES; ++i) {
        if (!s_tracked[i].active) { slot = i; break; }
    }
    if (slot >= 0) {
        s_tracked[slot] = (tracked_publish_t){.msg_id = msg_id, .token = token, .active = true};
        s_publish_success_count++;
        s_last_publish_time_ms = esp_timer_get_time() / 1000;
    }
    mqtt_unlock();
    return slot >= 0 ? ESP_OK : ESP_ERR_NO_MEM;
}

void mqtt_set_publish_ack_callback(mqtt_publish_ack_callback_t callback)
{
    mqtt_lock();
    s_ack_callback = callback;
    mqtt_unlock();
}

esp_err_t mqtt_subscribe(const char *topic, int qos, mqtt_cmd_callback_t cb)
{
    if (!s_initialized || s_mqtt_client == NULL) {
        ESP_LOGE(TAG, "Cannot subscribe: MQTT not initialized");
        return ESP_FAIL;
    }

    if (topic == NULL || cb == NULL) {
        ESP_LOGE(TAG, "Cannot subscribe: NULL topic or callback");
        return ESP_ERR_INVALID_ARG;
    }

    if (strlen(topic) >= MQTT_MAX_TOPIC_LEN) {
        ESP_LOGE(TAG, "Topic too long (max %d chars): %s",
                 MQTT_MAX_TOPIC_LEN - 1, topic);
        return ESP_ERR_INVALID_ARG;
    }

    mqtt_lock();

    /* Check for duplicate subscription (update callback if found) */
    for (int i = 0; i < s_subscription_count; i++) {
        if (s_subscriptions[i].active &&
            strcmp(s_subscriptions[i].topic, topic) == 0) {
            s_subscriptions[i].callback = cb;
            s_subscriptions[i].qos      = qos;
            mqtt_unlock();
            ESP_LOGI(TAG, "Updated callback for existing subscription: '%s'", topic);

            /* Re-subscribe at broker level in case QoS changed */
            int msg_id = esp_mqtt_client_subscribe(s_mqtt_client, topic, qos);
            return (msg_id >= 0) ? ESP_OK : ESP_FAIL;
        }
    }

    /* Check capacity */
    if (s_subscription_count >= MQTT_MAX_SUBSCRIPTIONS) {
        mqtt_unlock();
        ESP_LOGE(TAG, "Subscription table full (max %d)", MQTT_MAX_SUBSCRIPTIONS);
        return ESP_FAIL;
    }

    /* Add new subscription entry */
    int idx = s_subscription_count;
    strncpy(s_subscriptions[idx].topic, topic, MQTT_MAX_TOPIC_LEN - 1);
    s_subscriptions[idx].topic[MQTT_MAX_TOPIC_LEN - 1] = '\0';
    s_subscriptions[idx].qos      = qos;
    s_subscriptions[idx].callback = cb;
    s_subscriptions[idx].active   = true;
    s_subscription_count++;

    mqtt_unlock();

    ESP_LOGI(TAG, "Registered subscription: '%s' (qos=%d, index=%d)",
             topic, qos, idx);

    /*
     * Send SUBSCRIBE packet to the broker.  If the client is not yet
     * connected, this will be re-sent in the MQTT_EVENT_CONNECTED handler.
     */
    if (s_conn_state == MQTT_STATE_CONNECTED) {
        int msg_id = esp_mqtt_client_subscribe(s_mqtt_client, topic, qos);
        if (msg_id < 0) {
            ESP_LOGW(TAG, "Broker subscribe failed for '%s', will retry on connect", topic);
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Sent SUBSCRIBE to broker for '%s' (msg_id=%d)", topic, msg_id);
    } else {
        ESP_LOGI(TAG, "Not connected; subscription will activate on connect");
    }

    return ESP_OK;
}

mqtt_conn_state_t mqtt_get_connection_state(void)
{
    mqtt_lock();
    mqtt_conn_state_t state = s_conn_state;
    mqtt_unlock();
    return state;
}

bool mqtt_is_connected(void)
{
    return mqtt_get_connection_state() == MQTT_STATE_CONNECTED;
}

int64_t mqtt_get_connected_since_ms(void)
{
    mqtt_lock();
    int64_t connected_since_ms = s_connected_since_ms;
    mqtt_unlock();
    return connected_since_ms;
}

void mqtt_get_diagnostics(mqtt_diagnostics_t *out)
{
    if (out == NULL) return;
    mqtt_lock();
    memset(out, 0, sizeof(*out));
    out->state = s_conn_state;
    out->connect_count = s_connect_count;
    out->disconnect_count = s_disconnect_count;
    out->error_count = s_error_count;
    out->last_session_duration_ms = s_last_session_duration_ms;
    TaskHandle_t mqtt_task = xTaskGetHandle("mqtt_task");
    out->task_stack_high_watermark_bytes = mqtt_task != NULL
        ? (uint32_t)uxTaskGetStackHighWaterMark(mqtt_task) *
          sizeof(StackType_t)
        : 0;
    out->suspended = s_suspended;
    strlcpy(out->last_reason, s_last_connect_reason,
            sizeof(out->last_reason));
    mqtt_unlock();
}

/* Reconnect the configured client and return its actual broker result. */
esp_err_t mqtt_test_connection(const char *uri, const char *username,
                               const char *password, int timeout_ms,
                               char *reason, size_t reason_len)
{
    if (reason != NULL && reason_len > 0) reason[0] = '\0';
    if (uri == NULL || uri[0] == '\0') {
        if (reason != NULL && reason_len > 0)
            snprintf(reason, reason_len, "empty broker URI");
        return ESP_ERR_INVALID_ARG;
    }

    runtime_config_t runtime;
    runtime_config_get(&runtime);
    if (strcmp(uri, runtime.mqtt.uri) != 0 ||
        (username && strcmp(username, runtime.mqtt.username) != 0) ||
        (password && strcmp(password, runtime.mqtt.password) != 0)) {
        if (reason && reason_len) {
            snprintf(reason, reason_len,
                     "save the MQTT settings before testing the active connection");
        }
        return ESP_ERR_INVALID_STATE;
    }

    /* Reuse the persistent client instead of allocating a second MQTT task. */
    mqtt_set_connect_reason("connection test in progress");
    esp_err_t err = mqtt_reconnect();
    if (err != ESP_OK) {
        if (reason && reason_len) {
            snprintf(reason, reason_len, "unable to start MQTT reconnect: %s",
                     esp_err_to_name(err));
        }
        return err;
    }

    int wait_ms = timeout_ms > 0 ? timeout_ms : MQTT_CONNECT_TIMEOUT_MS;
    for (int elapsed_ms = 0; elapsed_ms < wait_ms; elapsed_ms += 100) {
        mqtt_conn_state_t state = mqtt_get_connection_state();
        if (state == MQTT_STATE_CONNECTED) {
            if (reason && reason_len) strlcpy(reason, "connected to broker", reason_len);
            return ESP_OK;
        }

        mqtt_lock();
        bool completed = strcmp(s_last_connect_reason,
                                "connection test in progress") != 0;
        if (completed && reason && reason_len) {
            strlcpy(reason, s_last_connect_reason, reason_len);
        }
        mqtt_unlock();
        if (completed && state == MQTT_STATE_ERROR) return ESP_FAIL;
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    mqtt_lock();
    if (reason && reason_len) strlcpy(reason, s_last_connect_reason, reason_len);
    mqtt_unlock();
    if (reason && reason_len &&
        strcmp(reason, "connection test in progress") == 0) {
        strlcpy(reason, "broker connection timed out", reason_len);
    }
    return ESP_ERR_TIMEOUT;
}

mqtt_mode_t mqtt_get_mode(void)
{
    return s_mqtt_mode;
}

static void mqtt_destroy_unlocked(void)
{
    if (!s_initialized) {
        ESP_LOGW(TAG, "MQTT handler not initialized, nothing to destroy");
        return;
    }

    ESP_LOGI(TAG, "Destroying MQTT handler");

    if (s_mqtt_client != NULL && s_client_started) {
        esp_err_t stop_err = esp_mqtt_client_stop(s_mqtt_client);
        if (stop_err != ESP_OK) {
            ESP_LOGE(TAG, "MQTT client stop failed; preserving client object: %s",
                     esp_err_to_name(stop_err));
            return;
        }
        s_client_started = false;
    }
    if (s_mqtt_client != NULL) {
        esp_mqtt_client_destroy(s_mqtt_client);
        s_mqtt_client = NULL;
    }
    mqtt_release_task_stack_reserve();

    /* Reset state.
       NOTE: s_subscriptions is intentionally NOT cleared here so that a
       subsequent mqtt_init()/mqtt_restart() re-subscribes the same topics
       once the new client connects (see MQTT_EVENT_CONNECTED). */
    s_conn_state            = MQTT_STATE_DISCONNECTED;
    memset(s_tracked, 0, sizeof(s_tracked));
    s_ack_callback          = NULL;
    s_publish_success_count = 0;
    s_publish_fail_count    = 0;
    s_last_publish_time_ms  = 0;
    s_connected_since_ms    = 0;
    s_initialized           = false;
    s_client_started        = false;

    ESP_LOGI(TAG, "MQTT handler destroyed");
}

void mqtt_destroy(void)
{
    if (s_lifecycle_mutex == NULL) {
        mqtt_destroy_unlocked();
        return;
    }
    xSemaphoreTake(s_lifecycle_mutex, portMAX_DELAY);
    mqtt_destroy_unlocked();
    xSemaphoreGive(s_lifecycle_mutex);
}

esp_err_t mqtt_restart(void)
{
    ESP_LOGI(TAG, "Restarting MQTT client with current runtime configuration");
    if (s_lifecycle_mutex == NULL) {
        s_lifecycle_mutex = xSemaphoreCreateMutex();
        if (s_lifecycle_mutex == NULL) return ESP_ERR_NO_MEM;
    }
    xSemaphoreTake(s_lifecycle_mutex, portMAX_DELAY);

    runtime_config_t runtime;
    runtime_config_get(&runtime);
    if (!runtime.mqtt.enabled) {
        xSemaphoreGive(s_lifecycle_mutex);
        return mqtt_disconnect();
    }

    if (!s_initialized || s_mqtt_client == NULL) {
        mqtt_init_unlocked();
        bool started = s_initialized && s_mqtt_client != NULL;
        xSemaphoreGive(s_lifecycle_mutex);
        return started ? ESP_OK : ESP_ERR_INVALID_STATE;
    }

    /* Keep the MQTT task and its stack allocated for the product lifetime.
       Recreating it after Web/Modbus activity can fail because internal SRAM
       is fragmented even when total free heap and PSRAM are plentiful. */
    mqtt_load_runtime_settings(&runtime);
    esp_mqtt_client_config_t mqtt_cfg =
        mqtt_build_client_config(&runtime, false);

    mqtt_lock();
    s_suspended = false;
    s_consecutive_auth_failures = 0;
    mqtt_unlock();

    if (!s_client_started) {
        esp_err_t start_err = esp_mqtt_set_config(s_mqtt_client, &mqtt_cfg);
        if (start_err == ESP_OK) {
            mqtt_register_runtime_subscriptions(&runtime);
            mqtt_release_task_stack_reserve();
            start_err = esp_mqtt_client_start(s_mqtt_client);
        }
        if (start_err == ESP_OK) {
            s_client_started = true;
            mqtt_set_state(MQTT_STATE_DISCONNECTED);
            mqtt_set_connect_reason(
                "configuration applied; resident MQTT task started");
        } else {
            mqtt_reserve_future_task_stack();
            mqtt_set_state(MQTT_STATE_ERROR);
            mqtt_set_connect_reason(
                "unable to create MQTT task: insufficient contiguous internal SRAM");
        }
        xSemaphoreGive(s_lifecycle_mutex);
        return start_err;
    }

    esp_err_t err = esp_mqtt_client_disconnect(s_mqtt_client);
    if (err != ESP_OK) {
        xSemaphoreGive(s_lifecycle_mutex);
        return err;
    }

    for (int elapsed_ms = 0; elapsed_ms < 3000; elapsed_ms += 50) {
        if (mqtt_get_connection_state() != MQTT_STATE_CONNECTED) break;
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    err = esp_mqtt_set_config(s_mqtt_client, &mqtt_cfg);
    if (err == ESP_OK) {
        mqtt_register_runtime_subscriptions(&runtime);
        mqtt_set_connect_reason(
            "configuration applied; reconnect requested on resident MQTT task");
        err = esp_mqtt_client_reconnect(s_mqtt_client);
    }

    xSemaphoreGive(s_lifecycle_mutex);
    return err;
}

void mqtt_suspend_async(const char *reason)
{
    mqtt_lock();
    s_suspended = true;
    strlcpy(s_last_connect_reason, reason ? reason : "MQTT suspended",
            sizeof(s_last_connect_reason));
    mqtt_unlock();
    if (!mqtt_start_supervisor()) {
        ESP_LOGE(TAG, "Unable to signal MQTT suspension: no supervisor task");
        return;
    }
    xTaskNotifyGive(s_supervisor_task_handle);
}

esp_err_t mqtt_reconnect(void)
{
    mqtt_lock();
    bool suspended = s_suspended;
    mqtt_unlock();
    if (suspended) {
        ESP_LOGW(TAG, "MQTT reconnect rejected while client is suspended");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_lifecycle_mutex == NULL) {
        return mqtt_restart();
    }
    xSemaphoreTake(s_lifecycle_mutex, portMAX_DELAY);
    if (!s_initialized || s_mqtt_client == NULL) {
        xSemaphoreGive(s_lifecycle_mutex);
        ESP_LOGI(TAG, "MQTT client is stopped; creating it before reconnect");
        return mqtt_restart();
    }
    ESP_LOGI(TAG, "Requesting MQTT reconnect on active client");
    esp_err_t err = esp_mqtt_client_reconnect(s_mqtt_client);
    xSemaphoreGive(s_lifecycle_mutex);
    return err;
}

esp_err_t mqtt_disconnect(void)
{
    if (s_lifecycle_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_lifecycle_mutex, portMAX_DELAY);
    if (!s_initialized) {
        ESP_LOGW(TAG, "MQTT handler not initialized, nothing to disconnect");
        xSemaphoreGive(s_lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (s_mqtt_client == NULL) {
        ESP_LOGW(TAG, "MQTT client already stopped, nothing to disconnect");
        xSemaphoreGive(s_lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Suspending MQTT connection (resident task retained)");
    mqtt_lock();
    s_suspended = true;
    mqtt_unlock();

    if (!s_client_started) {
        s_conn_state = MQTT_STATE_DISCONNECTED;
        mqtt_set_connect_reason(
            "disconnected; resident MQTT client is waiting for configuration");
        xSemaphoreGive(s_lifecycle_mutex);
        return ESP_OK;
    }

    runtime_config_t runtime;
    runtime_config_get(&runtime);
    mqtt_load_runtime_settings(&runtime);
    esp_mqtt_client_config_t mqtt_cfg =
        mqtt_build_client_config(&runtime, true);
    esp_err_t err = esp_mqtt_set_config(s_mqtt_client, &mqtt_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MQTT suspend configuration failed: %s",
                 esp_err_to_name(err));
        mqtt_lock();
        s_suspended = false;
        mqtt_unlock();
        xSemaphoreGive(s_lifecycle_mutex);
        return err;
    }

    err = esp_mqtt_client_disconnect(s_mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MQTT disconnect request failed: %s",
                 esp_err_to_name(err));
        xSemaphoreGive(s_lifecycle_mutex);
        return err;
    }

    s_conn_state  = MQTT_STATE_DISCONNECTED;
    s_connected_since_ms = 0;
    mqtt_set_connect_reason(
        "disconnected; resident MQTT task is waiting for configuration");
    xSemaphoreGive(s_lifecycle_mutex);
    return ESP_OK;
}

int64_t mqtt_get_last_publish_time_ms(void)
{
    mqtt_lock();
    int64_t t = s_last_publish_time_ms;
    mqtt_unlock();
    return t;
}

int mqtt_get_publish_success_count(void)
{
    mqtt_lock();
    int cnt = s_publish_success_count;
    mqtt_unlock();
    return cnt;
}

int mqtt_get_publish_fail_count(void)
{
    mqtt_lock();
    int cnt = s_publish_fail_count;
    mqtt_unlock();
    return cnt;
}

