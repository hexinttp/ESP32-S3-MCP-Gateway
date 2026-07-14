#pragma once
/*
 * Mock mqtt_client.h for PC simulation.
 * Mirrors the ESP-IDF esp_mqtt client API with the nested config struct layout.
 */

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Forward declaration for opaque client handle ---- */
struct esp_mqtt_client;
typedef struct esp_mqtt_client *esp_mqtt_client_handle_t;

/* ---- Event IDs ---- */
typedef enum {
    MQTT_EVENT_ANY = -1,
    MQTT_EVENT_ERROR = 0,
    MQTT_EVENT_CONNECTED,
    MQTT_EVENT_DISCONNECTED,
    MQTT_EVENT_SUBSCRIBED,
    MQTT_EVENT_UNSUBSCRIBED,
    MQTT_EVENT_PUBLISHED,
    MQTT_EVENT_DATA,
    MQTT_EVENT_BEFORE_CONNECT,
} esp_mqtt_event_id_t;

/* ---- Error types within MQTT ---- */
typedef enum {
    MQTT_ERROR_TYPE_NONE = 0,
    MQTT_ERROR_TYPE_TCP_TRANSPORT,
    MQTT_ERROR_TYPE_CONNECTION_REFUSED,
} esp_mqtt_error_type_t;

/* ---- Error detail codes ---- */
typedef struct {
    esp_mqtt_error_type_t error_type;
    int     esp_tls_last_esp_err;
    int     esp_tls_stack_err;
    int     esp_transport_sock_errno;
    int     connect_return_code;
} esp_mqtt_error_codes_t;

/* ---- Client configuration (nested layout matching ESP-IDF 5.x) ---- */
typedef struct {
    struct {
        struct {
            const char *uri;
        } address;
    } broker;
    struct {
        const char *client_id;
    } credentials;
    struct {
        int keepalive;
    } session;
    struct {
        int timeout_ms;
    } network;
} esp_mqtt_client_config_t;

/* ---- Event structure ---- */
struct esp_mqtt_event {
    esp_mqtt_client_handle_t client;
    int             event_id;
    char           *topic;
    int             topic_len;
    char           *data;
    int             data_len;
    int             msg_id;
    int             qos;
    esp_mqtt_error_codes_t *error_handle;
};
typedef struct esp_mqtt_event *esp_mqtt_event_handle_t;

/* ---- Event handler callback type (matches esp_event_handler_instance_register) ---- */
typedef void (*esp_event_handler_t_mqtt)(void *handler_args,
                                         const char *base,
                                         int32_t event_id,
                                         void *event_data);

/* ---- API Functions ---- */

esp_mqtt_client_handle_t esp_mqtt_client_init(const esp_mqtt_client_config_t *config);

esp_err_t esp_mqtt_client_register_event(esp_mqtt_client_handle_t client,
                                          int event,
                                          esp_event_handler_t_mqtt event_handler,
                                          void *event_handler_arg);

esp_err_t esp_mqtt_client_start(esp_mqtt_client_handle_t client);
esp_err_t esp_mqtt_client_stop(esp_mqtt_client_handle_t client);

int esp_mqtt_client_publish(esp_mqtt_client_handle_t client,
                            const char *topic,
                            const char *data,
                            int len,
                            int qos,
                            int retain);

int esp_mqtt_client_subscribe(esp_mqtt_client_handle_t client,
                              const char *topic,
                              int qos);

void esp_mqtt_client_destroy(esp_mqtt_client_handle_t client);

/* ---- Simulation control (not part of real ESP-IDF) ---- */

/**
 * @brief Query whether the mock MQTT client considers itself connected.
 */
bool sim_mqtt_mock_is_connected(void);

/**
 * @brief Get the total number of messages published through the mock.
 */
int sim_mqtt_mock_publish_count(void);

#ifdef __cplusplus
}
#endif
