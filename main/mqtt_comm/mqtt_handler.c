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

/* Thread-safety mutex */
static SemaphoreHandle_t        s_mqtt_mutex = NULL;
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

/* ======================== Internal Helpers ======================== */

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
        mqtt_set_state(MQTT_STATE_CONNECTED);
        mqtt_set_connect_reason("connected to broker");

        /* Re-subscribe to all registered topics on (re)connect */
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
        ESP_LOGW(TAG, "Disconnected from MQTT broker");
        mqtt_set_state(MQTT_STATE_DISCONNECTED);
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
        mqtt_set_state(MQTT_STATE_ERROR);
        break;
    }

    default:
        ESP_LOGD(TAG, "Unhandled MQTT event: %ld", event_id);
        break;
    }
}

/* ======================== Public API ======================== */

void mqtt_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "MQTT handler already initialized");
        return;
    }

    runtime_config_t runtime;
    runtime_config_get(&runtime);
#if ONENET_ENABLED
    s_mqtt_mode   = MQTT_MODE_ONENET;
    s_broker_uri  = ONENET_MQTT_BROKER_URI;
    s_client_id   = ONENET_DEVICE_ID;
    s_username    = ONENET_PRODUCT_ID;
    s_password    = ONENET_DEVICE_KEY;
    ESP_LOGI(TAG, "Initializing MQTT handler in OneNET mode (product: %s, device: %s)",
             ONENET_PRODUCT_ID, ONENET_DEVICE_ID);
#else
    s_mqtt_mode   = MQTT_MODE_STANDARD;
    if (!runtime.mqtt.enabled) {
        ESP_LOGW(TAG, "MQTT is disabled by runtime configuration");
        return;
    }
    if (runtime.mqtt.uri[0] == '\0' ||
        (strcmp(runtime.mqtt.uri, MQTT_BROKER_URI) == 0 &&
         runtime.mqtt.username[0] == '\0' &&
         runtime.mqtt.password[0] == '\0')) {
        ESP_LOGW(TAG, "MQTT broker is still the unconfigured placeholder; client not started");
        return;
    }
    strlcpy(s_runtime_broker, runtime.mqtt.uri, sizeof(s_runtime_broker));
    strlcpy(s_runtime_client_id, runtime.mqtt.client_id, sizeof(s_runtime_client_id));
    strlcpy(s_runtime_username, runtime.mqtt.username, sizeof(s_runtime_username));
    strlcpy(s_runtime_password, runtime.mqtt.password, sizeof(s_runtime_password));
    s_broker_uri = s_runtime_broker;
    s_client_id = s_runtime_client_id;
    s_username = s_runtime_username;
    s_password = s_runtime_password;
    ESP_LOGI(TAG, "Initializing MQTT handler (broker: %s)", s_broker_uri);

    /* ThingsCloud: enforce QoS/retain constraints and derive a unique client id
       from the ESP32 MAC when the user left it empty. The broker URI (scheme
       and port) is taken verbatim from the user configuration, because the
       ThingsCloud instance in use serves MQTT on the plaintext 1883 port and
       rejects TLS on 8883. */
    if (runtime.mqtt.platform_type == MQTT_PLATFORM_THINGSCLOUD) {
        runtime.mqtt.qos = 0;
        runtime.mqtt.retain = false;
        runtime.mqtt.lwt_enabled = false;
        strlcpy(s_runtime_broker, runtime.mqtt.uri, sizeof(s_runtime_broker));
        s_broker_uri = s_runtime_broker;
        ESP_LOGI(TAG, "ThingsCloud broker (as configured): %s", s_broker_uri);
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

    /* ThingsCloud gateways use clean sessions: a stale non-clean session under
       the fixed MAC-derived client_id can make the broker silently drop new
       logins (EOF before CONNACK), matching what we observe on the wire. */
    bool clean_session = runtime.mqtt.clean_session;
    if (runtime.mqtt.platform_type == MQTT_PLATFORM_THINGSCLOUD) {
        clean_session = true;
    }

    /* Configure and create the MQTT client */
    esp_mqtt_client_config_t mqtt_cfg = {
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
            .keepalive = runtime.mqtt.keepalive_sec,
            .disable_clean_session = !clean_session,
            .last_will = {
                .topic = runtime.mqtt.lwt_enabled ? runtime.mqtt.lwt_topic : NULL,
                .msg = runtime.mqtt.lwt_enabled ? runtime.mqtt.lwt_payload : NULL,
                .qos = runtime.mqtt.lwt_qos,
                .retain = runtime.mqtt.lwt_retain,
            },
        },
        .network = {
            .timeout_ms = MQTT_CONNECT_TIMEOUT_MS,
            /* Back off reconnects modestly: long enough to avoid hammering a
               broker that is flow-controlling this client, but short enough to
               recover quickly once the link is usable. */
            /* app_main requests the first reconnect after an interface has an
               IP address. Keep the client's automatic retry outside that
               window so two CONNECT attempts cannot race each other. */
            .reconnect_timeout_ms = 30000,
        },
        /* The ThingsCloud CONNECTED callback builds JSON, publishes initial
           state and registers sub-device state. Hardware testing showed that
           smaller stacks can fail during the broker handshake/callback path. */
        .task = {
            .stack_size = 20480,
        },
    };

    if (s_username[0] != '\0') {
        mqtt_cfg.credentials.username = s_username;
    }
    if (s_password[0] != '\0') {
        mqtt_cfg.credentials.authentication.password = s_password;
    }

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

    /* Start the MQTT client */
    err = esp_mqtt_client_start(s_mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(err));
        esp_mqtt_client_destroy(s_mqtt_client);
        s_mqtt_client = NULL;
        s_conn_state = MQTT_STATE_ERROR;
        return;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "MQTT client started successfully");

    /* Reset the subscription table (platform may have changed via restart) and
       register the topics for the active platform. The CONNECTED handler
       re-subscribes the table on every (re)connect, so this is sufficient. */
    mqtt_lock();
    s_subscription_count = 0;
    mqtt_unlock();
    if (runtime.mqtt.platform_type == MQTT_PLATFORM_THINGSCLOUD) {
        mqtt_subscribe(TC_TOPIC_GATEWAY_ATTR_PUSH, 0, thingscloud_on_attributes_push);
        mqtt_subscribe(TC_TOPIC_GATEWAY_CMD_SEND, 0, thingscloud_on_command_send);
        mqtt_subscribe(TC_TOPIC_ATTR_PUSH, 0, thingscloud_on_gateway_attributes_push);
        ESP_LOGI(TAG, "ThingsCloud downlink subscriptions registered");
    } else {
        char command_topic[128];
        snprintf(command_topic, sizeof(command_topic), "%s%s",
                 runtime.mqtt.command_prefix, runtime.gateway_id);
        mqtt_subscribe(command_topic, 1, mqtt_command_callback);
    }
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

void mqtt_destroy(void)
{
    if (!s_initialized) {
        ESP_LOGW(TAG, "MQTT handler not initialized, nothing to destroy");
        return;
    }

    ESP_LOGI(TAG, "Destroying MQTT handler");

    if (s_mqtt_client != NULL) {
        esp_mqtt_client_stop(s_mqtt_client);
        esp_mqtt_client_destroy(s_mqtt_client);
        s_mqtt_client = NULL;
    }

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
    s_initialized           = false;

    ESP_LOGI(TAG, "MQTT handler destroyed");
}

esp_err_t mqtt_restart(void)
{
    ESP_LOGI(TAG, "Restarting MQTT client with current runtime configuration");
    mqtt_destroy();
    mqtt_init(); /* guarded by s_initialized; reads fresh runtime config */
    return ESP_OK;
}

esp_err_t mqtt_reconnect(void)
{
    if (!s_initialized || s_mqtt_client == NULL) return ESP_ERR_INVALID_STATE;
    ESP_LOGI(TAG, "Requesting MQTT reconnect on active client");
    return esp_mqtt_client_reconnect(s_mqtt_client);
}

esp_err_t mqtt_disconnect(void)
{
    if (!s_initialized) {
        ESP_LOGW(TAG, "MQTT handler not initialized, nothing to disconnect");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_mqtt_client == NULL) {
        ESP_LOGW(TAG, "MQTT client already stopped, nothing to disconnect");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Disconnecting MQTT client (user requested)");
    /* esp_mqtt_client_stop() tears down the client task without scheduling a
       reconnect, so the connection stays down until mqtt_init()/mqtt_restart()
       is invoked again (e.g. when the user saves new broker settings). */
    esp_mqtt_client_stop(s_mqtt_client);
    esp_mqtt_client_destroy(s_mqtt_client);
    s_mqtt_client = NULL;
    s_conn_state  = MQTT_STATE_DISCONNECTED;
    /* s_initialized is intentionally left true so a stray mqtt_init() cannot
       silently re-establish the connection; mqtt_destroy()/mqtt_restart()
       reset it before re-initializing. */
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

