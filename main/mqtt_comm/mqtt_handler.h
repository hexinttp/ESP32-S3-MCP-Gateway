/**
 * @file mqtt_handler.h
 * @brief MQTT communication layer for the ESP32-S3 gateway
 *
 * Provides publish/subscribe operations, connection state tracking,
 * and a callback mechanism for downlink commands from the MQTT broker.
 */
#ifndef MQTT_HANDLER_H
#define MQTT_HANDLER_H

#include "gateway_config.h"

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ======================== Type Definitions ======================== */

typedef enum {
    MQTT_STATE_CONNECTED,
    MQTT_STATE_DISCONNECTED,
    MQTT_STATE_ERROR
} mqtt_conn_state_t;

typedef enum {
    MQTT_MODE_STANDARD,
    MQTT_MODE_ONENET
} mqtt_mode_t;

/**
 * @brief Callback type invoked when a message arrives on a subscribed topic.
 * @param topic  The topic string the message was received on
 * @param data   Pointer to the payload data
 * @param data_len Length of the payload in bytes
 */
typedef void (*mqtt_cmd_callback_t)(const char *topic, const char *data, int data_len);
typedef void (*mqtt_publish_ack_callback_t)(uint32_t token);

/**
 * @brief Maximum number of topic subscriptions supported simultaneously.
 */
#define MQTT_MAX_SUBSCRIPTIONS   16

/**
 * @brief Maximum topic length supported.
 */
#define MQTT_MAX_TOPIC_LEN       128

/**
 * @brief Message structure sent to the MQTT output queue for deferred publishing.
 */
typedef struct {
    char topic[MQTT_MAX_TOPIC_LEN];
    char payload[TCM_MAX_JSON_LEN];
    int  qos;
    uint32_t sequence_id;
} mqtt_out_msg_t;

/* ======================== API Functions ======================== */

/**
 * @brief Initialize the MQTT client and start the connection to the broker.
 *
 * Creates the esp_mqtt client using MQTT_BROKER_URI from gateway_config.h,
 * registers the internal event handler, and starts the client.
 * Must be called once during system startup after WiFi is connected.
 */
void mqtt_init(void);

/**
 * @brief Publish a message to the MQTT broker.
 *
 * Thread-safe. Updates internal success/fail counters and the
 * last-publish timestamp on each call.
 *
 * @param topic   Destination topic string
 * @param payload Null-terminated payload string
 * @param qos     QoS level (0, 1, or 2)
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t mqtt_publish(const char *topic, const char *payload, int qos);

/** Publish a QoS message and associate its acknowledgement with a durable token. */
esp_err_t mqtt_publish_tracked(const char *topic, const char *payload, int qos, uint32_t token);

/** Register the callback invoked after a tracked MQTT publish is acknowledged. */
void mqtt_set_publish_ack_callback(mqtt_publish_ack_callback_t callback);

/**
 * @brief Subscribe to a topic and register a callback for incoming messages.
 *
 * @param topic  Topic filter string (may contain MQTT wildcards)
 * @param qos    QoS level (0, 1, or 2)
 * @param cb     Callback function invoked when a message arrives
 * @return ESP_OK on success, ESP_FAIL on error or if subscription table is full
 */
esp_err_t mqtt_subscribe(const char *topic, int qos, mqtt_cmd_callback_t cb);

/**
 * @brief Get the current MQTT connection state.
 * @return mqtt_conn_state_t
 */
mqtt_conn_state_t mqtt_get_connection_state(void);

/**
 * @brief Convenience check: is the MQTT client currently connected?
 * @return true if connected, false otherwise
 */
bool mqtt_is_connected(void);

/**
 * @brief Shut down and destroy the MQTT client, releasing all resources.
 */
void mqtt_destroy(void);

/**
 * @brief Get the current MQTT mode (standard or OneNET).
 * @return mqtt_mode_t
 */
mqtt_mode_t mqtt_get_mode(void);

/**
 * @brief Get the timestamp (ms since boot) of the last successful publish.
 * @return Timestamp in milliseconds, or 0 if no publish has succeeded yet.
 */
int64_t mqtt_get_last_publish_time_ms(void);

/**
 * @brief Get the cumulative count of successful publish operations.
 */
int mqtt_get_publish_success_count(void);

/**
 * @brief Get the cumulative count of failed publish operations.
 */
int mqtt_get_publish_fail_count(void);

#endif /* MQTT_HANDLER_H */

