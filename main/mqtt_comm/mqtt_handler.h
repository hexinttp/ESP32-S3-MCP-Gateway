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
#include <stddef.h>
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

/** Custom-MQTT command downlink callback (defined in main.c). */
void mqtt_command_callback(const char *topic, const char *data, int data_len);

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
 * @brief Reconnect the configured MQTT client and report its broker result.
 *
 * The submitted values must match the saved runtime configuration. Reusing
 * the persistent client keeps connection ownership deterministic and avoids
 * allocating a competing MQTT task on resource-constrained hardware.
 *
 * @param uri      Broker URI, e.g. "mqtt://host:1883" (must not be empty)
 * @param username Optional username, or NULL/"" for anonymous
 * @param password Optional password, or NULL/"" when none
 * @param timeout_ms Connection timeout in ms (<=0 falls back to MQTT_PUBLISH_TIMEOUT_MS)
 * @param reason   Optional output buffer filled with a human-readable reason
 *                 for the result (e.g. "DNS resolution failed", "broker
 *                 rejected login"). May be NULL.
 * @param reason_len Size of the reason buffer in bytes.
 * @return ESP_OK on successful connect, ESP_ERR_TIMEOUT if no response,
 *         ESP_ERR_INVALID_ARG for empty URI, otherwise ESP_FAIL (auth/network error)
 */
esp_err_t mqtt_test_connection(const char *uri, const char *username,
                               const char *password, int timeout_ms,
                               char *reason, size_t reason_len);

/**
 * @brief Shut down and destroy the MQTT client, releasing all resources.
 */
void mqtt_destroy(void);

/**
 * @brief Tear down and re-create the MQTT client using the current runtime
 *        configuration. Used after the broker settings are changed at runtime
 *        (e.g. via the Web UI) so the new parameters take effect immediately
 *        without a full device reboot. The subscription table is preserved,
 *        so all previously registered topics (e.g. the command topic) are
 *        re-subscribed automatically once the new client connects.
 * @return ESP_OK if the restart was initiated
 */
esp_err_t mqtt_restart(void);

/**
 * @brief Request a reconnect on the existing MQTT client after an uplink
 *        becomes available. Does not destroy or re-create the MQTT task.
 */
esp_err_t mqtt_reconnect(void);

/**
 * @brief Disconnect the MQTT client from the broker without destroying the
 *        module configuration or the subscription table.
 *
 * The connection stays down until mqtt_init()/mqtt_restart() is called again
 * (e.g. when the user saves new broker settings). Intended for the Web UI
 * "Disconnect" action so the user can drop the broker link on demand without
 * losing the configured parameters.
 * @return ESP_OK if the client was stopped, ESP_ERR_INVALID_STATE if the client
 *         was not running.
 */
esp_err_t mqtt_disconnect(void);

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

