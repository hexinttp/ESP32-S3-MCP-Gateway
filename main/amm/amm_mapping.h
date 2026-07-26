/**
 * @file amm_mapping.h
 * @brief Adaptive Mapping Model (AMM) - Modbus-to-MQTT mapping registry
 *
 * The AMM layer maintains a registry that maps raw Modbus register addresses
 * to semantic device/point metadata. It enriches TCM context objects with
 * human-readable identities, validates downlink write commands against
 * control constraints, and persists the mapping table to NVS.
 */
#ifndef AMM_MAPPING_H
#define AMM_MAPPING_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "gateway_config.h"
#include "tcm/tcm_context.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== NVS Keys ======================== */

#define AMM_NVS_NAMESPACE       "amm_mapping"
#define AMM_NVS_KEY_COUNT       "entry_cnt"
#define AMM_NVS_KEY_ENTRY_PREFIX "entry_"
#define AMM_NVS_KEY_SCHEMA      "schema_ver"
#define AMM_NVS_SCHEMA_VERSION  4
#define AMM_MAX_READ_REGISTERS  8

/* ======================== Mapping Entry ======================== */

/**
 * @brief A single Modbus-to-MQTT mapping entry.
 *
 * Each entry binds a (slave_id, register_address) pair to its semantic
 * identity (device, point, measurement name, unit) and MQTT routing
 * information, together with a control constraint for downlink safety.
 */
typedef struct {
    uint32_t mapping_version;
    source_protocol_t source_protocol;
    uint8_t  channel_id;
    uint8_t  slave_id;
    uint8_t  function_code;
    uint16_t register_address;
    data_type_t data_type;
    byte_order_t byte_order;
    float    scale_factor;
    float    offset;
    uint32_t poll_interval_ms;
    uint8_t  priority;
    bool     discovered;
    char     device_id[AMM_MAX_DEVICE_NAME_LEN];
    char     point_id[AMM_MAX_POINT_NAME_LEN];
    char     measurement_name[AMM_MAX_POINT_NAME_LEN];
    char     unit[AMM_MAX_UNIT_LEN];
    char     mqtt_topic[AMM_MAX_TOPIC_LEN];
    control_constraint_t constraint;
    bool     active;
    uint16_t read_start_address;  /**< Start of the Modbus read window */
    uint8_t  read_register_count; /**< Total registers read for this point */
    uint8_t  value_register_index;/**< Value offset inside the read window */
} amm_mapping_entry_t;

/* ======================== Validation Result ======================== */

/**
 * @brief Result of validating a downlink MQTT command against AMM rules.
 */
typedef struct {
    bool     accepted;
    char     reject_reason[128];
    amm_mapping_entry_t *matched_entry;
} amm_validation_result_t;

/* ======================== API Functions ======================== */

/**
 * @brief Initialize the AMM mapping table.
 *
 * Creates the internal mutex, attempts to load persisted mappings from NVS,
 * and starts with an empty table when NVS has no mapping data.
 */
void amm_init(void);

/** Runtime capacity: 1000 with PSRAM, 64 in internal-memory fallback mode. */
int amm_get_capacity(void);

/**
 * @brief Add a mapping entry to the registry.
 *
 * The entry is copied into the internal table, marked active, and the full
 * table is persisted to NVS.
 *
 * @param entry Pointer to the mapping entry to add.
 * @return ESP_OK on success, ESP_ERR_NO_MEM if the table is full,
 *         ESP_ERR_INVALID_ARG if entry is NULL.
 */
esp_err_t amm_add_mapping(const amm_mapping_entry_t *entry);

/** Update an entry in place and create a new model version. */
esp_err_t amm_update_mapping(int index, const amm_mapping_entry_t *entry);

/** Copy active entries into a caller-owned snapshot. */
int amm_get_entries(amm_mapping_entry_t *out, int max_entries);

/**
 * Import a semantic mapping profile as one flash transaction.
 *
 * When replace_devices is true, existing mappings for every
 * protocol/channel/slave tuple present in entries are removed first.
 */
esp_err_t amm_import_mappings(const amm_mapping_entry_t *entries, int count,
                              bool replace_devices, int *imported_count);

/** Copy one active entry by its zero-based active-list index. */
esp_err_t amm_get_entry_at(int active_index, amm_mapping_entry_t *out);

/** Current monotonically increasing AMM model version. */
uint32_t amm_get_model_version(void);

/** Remove every mapping and persist the empty table in one NVS transaction. */
esp_err_t amm_clear_mappings(void);

/** Channel-aware lookup used for mixed RTU/TCP gateways. */
esp_err_t amm_find_mapping_for_channel(source_protocol_t protocol, uint8_t channel_id,
                                       uint8_t slave_id, uint16_t reg_addr,
                                       amm_mapping_entry_t *out);

/** Copy a mapping by its stable semantic identity. */
esp_err_t amm_find_mapping_by_point(const char *device_id, const char *point_id,
                                    amm_mapping_entry_t *out);

/**
 * @brief Remove (deactivate) a mapping entry by slave_id + register_address.
 *
 * The entry is not physically deleted; its @c active flag is set to false.
 * The updated table is persisted to NVS.
 *
 * @param slave_id  MODBUS slave/unit ID.
 * @param reg_addr  Register address.
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if no matching entry exists.
 */
esp_err_t amm_remove_mapping(uint8_t slave_id, uint16_t reg_addr);
esp_err_t amm_remove_mapping_for_channel(source_protocol_t protocol, uint8_t channel_id,
                                         uint8_t slave_id, uint16_t reg_addr);

/**
 * @brief Find an active mapping entry for a given slave_id + register_address.
 *
 * @param slave_id  MODBUS slave/unit ID.
 * @param reg_addr  Register address.
 * @return Pointer to the matching entry, or NULL if not found.
 */
amm_mapping_entry_t *amm_find_mapping(uint8_t slave_id, uint16_t reg_addr);

/**
 * @brief Get the total number of active mapping entries.
 */
int amm_get_mapping_count(void);

/**
 * @brief Enrich a TCM context with mapping info.
 *
 * Looks up the mapping entry for ctx->slave_id + ctx->register_address and
 * copies device_id, point_id, measurement_name, unit, and constraint into
 * the context.
 *
 * @param ctx Pointer to the TCM context to enrich.
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if no mapping exists.
 */
esp_err_t amm_enrich_context(tcm_context_t *ctx);

/**
 * @brief Validate a downlink MQTT command against AMM rules.
 *
 * Checks:
 *  1. Target device/point exists in the mapping table.
 *  2. The entry is marked writable.
 *  3. The commanded value is within the valid range.
 *  4. The data_type in the command matches the mapping entry.
 *  5. The function_code is a valid write code (6 or 16).
 *
 * @param cmd_ctx Pointer to the TCM context deserialized from the command.
 * @return Validation result with accepted flag and optional reject_reason.
 */
amm_validation_result_t amm_validate_command(const tcm_context_t *cmd_ctx);

/**
 * @brief Get the MQTT topic for a given slave_id + register_address.
 *
 * @param slave_id  MODBUS slave/unit ID.
 * @param reg_addr  Register address.
 * @return Pointer to the topic string, or NULL if not found.
 */
const char *amm_get_mqtt_topic(uint8_t slave_id, uint16_t reg_addr);

esp_err_t amm_copy_mqtt_topic(source_protocol_t protocol, uint8_t channel_id,
                              uint8_t slave_id, uint16_t reg_addr,
                              char *out, size_t out_size);

/**
 * @brief Load the mapping table from NVS persistent storage.
 *
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if no data stored,
 *         or other ESP_ERR codes on failure.
 */
esp_err_t amm_load_from_nvs(void);

/**
 * @brief Save the current mapping table to NVS persistent storage.
 *
 * @return ESP_OK on success, or an ESP_ERR code on failure.
 */
esp_err_t amm_save_to_nvs(void);

#ifdef __cplusplus
}
#endif

#endif /* AMM_MAPPING_H */
