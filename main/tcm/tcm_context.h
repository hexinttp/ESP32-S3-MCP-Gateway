/**
 * @file tcm_context.h
 * @brief TCM Context Object definition - 16 mandatory fields
 */
#ifndef TCM_CONTEXT_H
#define TCM_CONTEXT_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "gateway_config.h"

/* ======================== Enumerations ======================== */

typedef enum {
    QUALITY_GOOD = 0,
    QUALITY_STALE,
    QUALITY_INVALID
} quality_state_t;

typedef enum {
    NET_ONLINE = 0,
    NET_DELAYED,
    NET_OFFLINE,
    NET_REPLAYED
} network_state_t;

typedef enum {
    OP_READ_PUBLISH = 0,
    OP_SUBSCRIBE,
    OP_WRITE,
    OP_REPLAY,
    OP_READ_ONLY
} operation_type_t;

typedef enum {
    SRC_MODBUS_RTU = 0,
    SRC_MODBUS_TCP
} source_protocol_t;

typedef enum {
    DT_INT16 = 0,
    DT_UINT16,
    DT_FLOAT32,
    DT_INT32,
    DT_UINT32
} data_type_t;

typedef enum {
    BYTE_ORDER_ABCD = 0,
    BYTE_ORDER_CDAB,
    BYTE_ORDER_BADC,
    BYTE_ORDER_DCBA
} byte_order_t;

/* ======================== Control Constraint ======================== */

typedef struct {
    bool writable;
    float valid_range_min;
    float valid_range_max;
} control_constraint_t;

/* ======================== TCM Context Object (16 fields) ======================== */

typedef struct {
    /* TCM envelope metadata */
    char tcm_version[8];
    char gateway_id[48];
    uint8_t channel_id;

    /* Identity Group (4 fields) */
    uint32_t context_id;                              /* Field 1: unique record ID */
    char device_id[AMM_MAX_DEVICE_NAME_LEN];          /* Field 2: device identity */
    char point_id[AMM_MAX_POINT_NAME_LEN];            /* Field 3: measurement point */
    source_protocol_t source_protocol;                /* Field 4: source protocol */

    /* MODBUS Addressing Group (4 fields) */
    uint8_t slave_id;                                 /* Field 5: MODBUS slave/unit ID */
    uint8_t function_code;                            /* Field 6: MODBUS function code */
    uint16_t register_address;                        /* Field 7: register address */
    data_type_t data_type;                            /* Field 8: data representation */

    /* Measurement Group (4 fields) */
    char measurement_name[AMM_MAX_POINT_NAME_LEN];    /* Field 9: human-readable name */
    char unit[AMM_MAX_UNIT_LEN];                      /* Field 10: engineering unit */
    float value;                                      /* Field 11: measured value */
    float raw_value;
    float scale_factor;
    float offset;
    byte_order_t byte_order;
    int64_t timestamp_ms;                             /* Field 12: creation time (ms) */

    /* Operational Group (3 fields) */
    quality_state_t quality_state;                    /* Field 13: data quality */
    network_state_t network_state;                    /* Field 14: network condition */
    operation_type_t operation_type;                  /* Field 15: operation type */

    /* Control Boundary (1 field) */
    control_constraint_t control_constraint;          /* Field 16: write/range safety */

    /* Metadata (not part of the 16 schema fields) */
    uint32_t sequence_id;                             /* Monotonic sequence for replay ordering */
    uint32_t mapping_version;
    bool validated;                                   /* Whether validation has passed */
} tcm_context_t;

/* ======================== Validation Result ======================== */

typedef struct {
    bool passed;
    uint32_t failed_field_mask;      /* Bitmask of which fields failed */
    char fail_reason[128];
} tcm_validation_result_t;

/* ======================== API Functions ======================== */

/**
 * @brief Initialize the TCM module (sequence counter, etc.)
 */
void tcm_init(void);

/**
 * @brief Build a TCM context object from raw MODBUS data and mapping info
 * @param ctx Output context object
 * @param slave_id MODBUS slave ID
 * @param func_code MODBUS function code
 * @param reg_addr Register address
 * @param raw_value Raw register value (as float)
 * @param quality Data quality state
 * @param net_state Current network state
 * @return 0 on success, -1 on error
 */
int tcm_build_context(tcm_context_t *ctx,
                      uint8_t slave_id,
                      uint8_t func_code,
                      uint16_t reg_addr,
                      float raw_value,
                      quality_state_t quality,
                      network_state_t net_state);

/**
 * @brief Validate a TCM context object against the schema
 * @param ctx Context object to validate
 * @param result Output validation result
 * @return true if validation passed
 */
bool tcm_validate(const tcm_context_t *ctx, tcm_validation_result_t *result);

/**
 * @brief Serialize a TCM context object to JSON string
 * @param ctx Context object
 * @param json_buf Output JSON buffer
 * @param buf_size Buffer size
 * @return Length of JSON string, or -1 on error
 */
int tcm_serialize_json(const tcm_context_t *ctx, char *json_buf, size_t buf_size);

/**
 * @brief Deserialize a JSON string to a TCM context object (for downlink commands)
 * @param json_str JSON string
 * @param ctx Output context object
 * @return 0 on success, -1 on error
 */
int tcm_deserialize_json(const char *json_str, tcm_context_t *ctx);

/**
 * @brief Get current sequence counter value
 */
uint32_t tcm_get_sequence_counter(void);

/**
 * @brief Update the network state on an existing context object
 */
void tcm_update_network_state(tcm_context_t *ctx, network_state_t new_state);

#endif /* TCM_CONTEXT_H */
