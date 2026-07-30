#ifndef CLOUD_ADAPTER_H
#define CLOUD_ADAPTER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "tcm/tcm_context.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Unified cloud data object
 * Modules that produce telemetry (Modbus/semantic) submit this
 * neutral structure instead of platform-specific topics/payloads.
 * The cloud adapter turns it into the correct Topic + JSON.
 * ============================================================ */

typedef enum {
    CLOUD_VALUE_BOOL = 0,
    CLOUD_VALUE_INTEGER,
    CLOUD_VALUE_NUMBER,
    CLOUD_VALUE_STRING
} cloud_value_type_t;

typedef struct {
    char device_address[40];   /* ThingsCloud sub-device address (slave based) */
    char property_key[64];     /* ThingsCloud property identifier (cloud key);
                                  holds "p{port}_s{slave}_{point}" in gateway mode */
    cloud_value_type_t value_type;
    union {
        bool boolean_value;
        int64_t integer_value;
        double number_value;
        char string_value[64];
    } value;
    int64_t timestamp_ms;
    uint8_t quality;           /* QUALITY_GOOD / BAD / UNCERTAIN (semantic) */
} cloud_property_update_t;

void cloud_property_set_bool(cloud_property_update_t *p, const char *dev, const char *key, bool v);
void cloud_property_set_integer(cloud_property_update_t *p, const char *dev, const char *key, int64_t v);
void cloud_property_set_number(cloud_property_update_t *p, const char *dev, const char *key, double v);
void cloud_property_set_string(cloud_property_update_t *p, const char *dev, const char *key, const char *v);

/* ============================================================
 * ThingsCloud platform API
 * ============================================================ */

/* Publish gateway self attributes (topic: attributes). */
esp_err_t thingscloud_publish_gateway_attributes(const cloud_property_update_t *props, int count);
/* Publish sub-device attributes (topic: gateway/attributes). */
esp_err_t thingscloud_publish_subdevice_attributes(const cloud_property_update_t *props, int count);
/* Report a sub-device coming online (topic: gateway/connect). */
esp_err_t thingscloud_report_subdevice_online(const char *device_address);
/* Report a sub-device going offline (topic: gateway/disconnect). */
esp_err_t thingscloud_report_subdevice_offline(const char *device_address);
/* Publish gateway status summary (topic: attributes). */
esp_err_t thingscloud_publish_gateway_status(void);

/* Build the gateway-mode property key "p{port}_s{slave}_{property}".
 * property_key must be non-empty and contain only [A-Za-z0-9_]; the output is
 * always NUL-terminated. Returns ESP_ERR_INVALID_ARG for bad input or
 * ESP_ERR_INVALID_SIZE when the output buffer is too small. */
esp_err_t build_gateway_property_key(uint8_t port_id, uint8_t slave_id,
                                     const char *property_key,
                                     char *output, size_t output_size);
/* Validate a (custom or generated) gateway property key: only [A-Za-z0-9_],
 * non-empty. Spaces, CJK, '/', '.', '-', and MQTT wildcards are rejected. */
bool thingscloud_gateway_key_valid(const char *key);

/* Per-slave communication status snapshot used for gateway-mode reporting. */
typedef struct {
    char     address[40];       /* ThingsCloud sub-device address             */
    uint8_t  port_id;
    uint8_t  slave_id;
    int      state;             /* 0 unknown, 1 online, 2 offline             */
    int64_t  last_seen_ms;      /* last valid Modbus response (not on failure)*/
    uint32_t error_count;       /* consecutive communication failures         */
    bool     data_valid;        /* true when the slave is ONLINE              */
} thingscloud_slave_status_t;

/* Copy the current per-slave runtime states into out[]. Returns the number of
 * slaves written (<= max), or the total number of tracked slaves if that
 * exceeds max. Used by gateway mode to build per-slave status attributes. */
int thingscloud_subdev_get_status(thingscloud_slave_status_t *out, int max);
int thingscloud_subdev_online_count(void);
int thingscloud_subdev_offline_count(void);

/* Gateway mode: publish every slave's online/status attributes plus the RS485
 * bus summary to the "attributes" topic. */
esp_err_t thingscloud_publish_gateway_slave_status(void);

/* Monotonic counter bumped on every report-mode change. Queued/aggregated
 * data produced under a previous generation is dropped on switch. */
uint32_t thingscloud_get_config_generation(void);

/* Sub-device online/offline state machine (keyed by device address). */
void thingscloud_subdev_register_success(const char *device_address);
void thingscloud_subdev_register_failure(const char *device_address);
void thingscloud_subdev_republish_all_online(void);
int thingscloud_subdev_error_count(void);

/* Ingest a single telemetry context from the publish pipeline.
 * Aggregates into the per-cycle buffer and flushes when it grows
 * too large; call thingscloud_flush() periodically to bound latency. */
esp_err_t thingscloud_publish_context(const tcm_context_t *ctx);
/* Replay one durable TCM record through the active ThingsCloud report mode.
 * Returns ESP_OK only when every generated MQTT packet was accepted by the
 * MQTT client, allowing UIF to remove the durable record. */
esp_err_t thingscloud_replay_context(const tcm_context_t *ctx);
/* Flush any pending aggregated sub-device attributes. */
void thingscloud_flush(void);

typedef struct {
    uint32_t pending_points;
    uint32_t throttled_count;
    uint32_t dropped_count;
    int64_t last_publish_ms;
    int last_response_code;
    bool upload_suspended;
} thingscloud_runtime_status_t;

void thingscloud_get_runtime_status(thingscloud_runtime_status_t *out);
void thingscloud_record_publish_response(bool accepted, int error_code);
void thingscloud_clear_publish_guard(void);
bool thingscloud_upload_is_suspended(void);

/* Called by the MQTT layer on (re)connect to re-report online sub-devices. */
void thingscloud_on_mqtt_connected(void);

/* Returns true when the active platform is ThingsCloud. Cheap: reads runtime. */
bool thingscloud_is_enabled(void);

/* Downlink callbacks (registered as MQTT subscriptions). */
void thingscloud_on_attributes_push(const char *topic, const char *data, int data_len);
void thingscloud_on_command_send(const char *topic, const char *data, int data_len);
void thingscloud_on_gateway_attributes_push(const char *topic, const char *data, int data_len);
void thingscloud_on_attributes_response(const char *topic, const char *data, int data_len);
void thingscloud_on_gateway_attributes_response(const char *topic, const char *data,
                                                int data_len);

/* Mask a sensitive credential for logging: shows first4****last4. */
void thingscloud_mask_credential(const char *src, char *out, size_t out_size);

#ifdef __cplusplus
}
#endif

#endif /* CLOUD_ADAPTER_H */
