/**
 * @file modbus_discover.h
 * @brief Automatic MODBUS device discovery and AMM mapping generation.
 *
 * Provides bus scanning, register probing, and intelligent semantic
 * inference to automatically populate the AMM mapping table when new
 * devices are connected to the MODBUS bus.
 *
 * Two scanning modes:
 *  1. Quick broadcast scan  – probes slave IDs 1-247 with a short timeout
 *     to find active devices.  Called automatically at gateway boot.
 *  2. Fine register scan    – for a given slave, probes a configurable
 *     register address range to identify all available data points.
 *     Can be triggered from the web UI.
 */
#ifndef MODBUS_DISCOVER_H
#define MODBUS_DISCOVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "tcm/tcm_context.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== Constants ======================== */

#define DISCOVER_MAX_SLAVES         100   /**< PSRAM-backed discovery capacity */
#define DISCOVER_FALLBACK_SLAVES      8   /**< Capacity when external PSRAM is unavailable */
#define DISCOVER_MAX_REGS_PER_SLAVE   8   /**< First-pass points retained per discovered slave */
#define DISCOVER_MAX_BLOCK_REGS       8   /**< Largest adaptive read block used for discovery */
#define DISCOVER_DEFAULT_EMPTY_GAP    8   /**< Stop after this many empty addresses after data */
#define DISCOVER_BROADCAST_TIMEOUT   100  /**< ms, timeout for quick slave probe */
#define DISCOVER_SCAN_TIMEOUT        500  /**< ms, timeout for register scan */

/* ======================== Data Structures ======================== */

/**
 * @brief A single discovered register on a slave device.
 */
typedef struct {
    uint16_t    register_address;
    uint8_t     function_code;      /**< 03 = holding, 04 = input */
    uint16_t    raw_value;          /**< Unscaled 16-bit sample */
    data_type_t inferred_type;      /**< Best-guess data type */
    float       sample_value;       /**< Value read during scan */
    uint16_t    read_start_address; /**< Start of the successful read window */
    uint8_t     read_register_count;/**< Registers required by the device response */
    uint8_t     value_register_index;/**< This point's index inside the read window */
    char        inferred_name[32];  /**< e.g. "Motor temperature" */
    char        inferred_unit[16];  /**< e.g. "degC" */
    bool        writable;           /**< true if FC06 write succeeds */
    bool        valid;
} discovered_register_t;

/**
 * @brief A discovered MODBUS slave device with its registers.
 */
typedef struct {
    uint8_t  slave_id;
    source_protocol_t source_protocol; /**< RTU bus or TCP endpoint */
    uint8_t  channel_id;               /**< RTU=0, TCP endpoint_id */
    char     device_id[32];         /**< Auto-generated, e.g. "device_slave_01" */
    char     name[48];              /**< User-editable display name */
    char     description[64];       /**< User-editable description */
    char     mqtt_topic_prefix[64]; /**< MQTT topic prefix for this device */
    uint8_t  probe_function_code;   /**< Function code that proved device liveness */
    uint16_t probe_address;         /**< Address used by the successful probe */
    uint8_t  probe_register_count;  /**< Block size required by the successful probe */
    uint16_t reg_count;             /**< Number of discovered registers */
    discovered_register_t registers[DISCOVER_MAX_REGS_PER_SLAVE];
    bool     active;                /**< true if device responded to probe */
} discovered_device_t;

/**
 * @brief Result of a complete bus scan.
 */
typedef struct {
    uint16_t total_scanned;         /**< Total slave IDs probed */
    uint16_t slaves_scanned;        /**< Slave IDs completed so far */
    uint16_t devices_found;        /**< Number of responding slaves */
    uint16_t registers_found;      /**< Total registers across all slaves */
    uint16_t mappings_created;     /**< AMM entries auto-created */
    uint16_t device_capacity;      /**< Runtime capacity after PSRAM/fallback allocation */
    uint16_t current_register;     /**< Register currently being probed */
    uint8_t  current_slave;        /**< Slave currently being probed */
    uint8_t  current_function_code;/**< Function code currently being probed */
    uint8_t  phase;                /**< discover_phase_t */
    esp_err_t last_error;          /**< Final task error, ESP_OK while healthy */
    bool     scan_complete;
    bool     scan_in_progress;
} discover_result_t;

typedef enum {
    DISCOVER_PHASE_IDLE = 0,
    DISCOVER_PHASE_BUS_SCAN,
    DISCOVER_PHASE_REGISTER_SCAN,
    DISCOVER_PHASE_COMPLETE,
    DISCOVER_PHASE_ERROR,
} discover_phase_t;

/**
 * @brief Scan parameter structure (used for web-triggered scans).
 */
typedef struct {
    uint8_t  slave_start;           /**< First slave ID to scan (default: 1) */
    uint8_t  slave_end;             /**< Last slave ID to scan (default: 247) */
    uint16_t reg_start;             /**< First register address (default: 40001) */
    uint16_t reg_end;               /**< Last register address (default: 40100) */
    source_protocol_t source_protocol; /**< MODBUS RTU or TCP */
    uint8_t  channel_id;            /**< TCP endpoint id; RTU uses 0 */
    uint8_t  function_codes[2];     /**< FC list: {0x03, 0x04} */
    uint8_t  fc_count;              /**< Number of function codes */
    uint8_t  max_empty_gap;         /**< Stop after N empty addresses after first data */
} discover_scan_params_t;

/* ======================== API Functions ======================== */

/**
 * @brief Initialise the discovery module.
 *
 * Allocates internal buffers and resets scan state.
 */
void modbus_discover_init(void);

/**
 * @brief Quick broadcast scan: probe slave IDs from @p start to @p end.
 *
 * Sends a single-register read (FC03, address 0) to each slave ID with
 * a short timeout.  Any slave that responds is recorded as "active".
 *
 * This function is non-blocking: it returns immediately and sets
 * scan_in_progress.  Call modbus_discover_get_result() to poll status.
 *
 * @param start  First slave ID (typically 1).
 * @param end    Last slave ID (typically 247).
 * @return ESP_OK if scan started, ESP_ERR_INVALID_STATE if already running.
 */
esp_err_t modbus_discover_scan_bus(uint8_t start, uint8_t end);

/**
 * @brief Fine register scan for a specific slave device.
 *
 * Probes register addresses from @p reg_start to @p reg_end using
 * both FC03 (holding) and FC04 (input).  For each responding register,
 * reads the value and applies semantic inference.
 *
 * @param slave_id   Target slave ID.
 * @param reg_start  First register address (e.g. 40001).
 * @param reg_end    Last register address (e.g. 40100).
 * @return ESP_OK on success.
 */
esp_err_t modbus_discover_scan_device(uint8_t slave_id,
                                      uint16_t reg_start,
                                      uint16_t reg_end);

/**
 * @brief Full scan: broadcast + register scan for all discovered devices.
 *
 * Combines modbus_discover_scan_bus() and modbus_discover_scan_device()
 * in sequence.
 *
 * @param params  Scan parameters (NULL for defaults: 1-247, 40001-40100).
 * @return ESP_OK if scan started.
 */
esp_err_t modbus_discover_full_scan(const discover_scan_params_t *params);

/**
 * @brief Get the current scan result snapshot.
 */
discover_result_t modbus_discover_get_result(void);

/**
 * @brief Get a specific discovered device by index.
 *
 * @param index  0-based index into the device list.
 * @return Pointer to the device, or NULL if index out of range.
 */
const discovered_device_t *modbus_discover_get_device(uint16_t index);

/**
 * @brief Get total number of discovered devices.
 */
uint16_t modbus_discover_get_device_count(void);

/**
 * @brief Get the runtime discovery table capacity.
 *
 * Returns DISCOVER_MAX_SLAVES when PSRAM is available, otherwise the
 * internal-RAM fallback capacity.
 */
uint16_t modbus_discover_get_capacity(void);

/**
 * @brief Apply all discovered registers as AMM mapping entries.
 *
 * For each discovered register, creates an amm_mapping_entry_t with
 * inferred semantic metadata and calls amm_add_mapping().
 *
 * @return Number of mapping entries successfully created.
 */
int modbus_discover_apply_mappings(void);

/**
 * @brief Clear all discovery results and reset state.
 */
void modbus_discover_reset(void);

/* ======================== Editing API ======================== */

/**
 * @brief Find a discovered device by slave ID (mutable).
 * @return Pointer to the device, or NULL if not found.
 */
discovered_device_t *modbus_discover_find_device(uint8_t slave_id);

/**
 * @brief Update device-level fields (device_id, description, mqtt_topic_prefix).
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if device not found.
 */
esp_err_t modbus_discover_update_device(uint8_t slave_id,
                                         const char *device_id,
                                         const char *name,
                                         const char *mqtt_topic_prefix);

/**
 * @brief Find a specific register on a device (mutable).
 * @return Pointer to the register, or NULL if not found.
 */
discovered_register_t *modbus_discover_find_register(uint8_t slave_id,
                                                      uint16_t reg_addr);

/**
 * @brief Update register-level fields.
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if not found.
 */
esp_err_t modbus_discover_update_register(uint8_t slave_id,
                                           uint16_t reg_addr,
                                           const char *name,
                                           const char *unit,
                                           data_type_t dtype,
                                           bool writable,
                                           float range_min,
                                           float range_max);

/**
 * @brief Toggle a register's valid/active state.
 * @param[out] new_state  The new valid state after toggle.
 * @return ESP_OK on success.
 */
esp_err_t modbus_discover_toggle_register(uint8_t slave_id,
                                           uint16_t reg_addr,
                                           bool *new_state);

/**
 * @brief Delete a register from a discovered device.
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if not found.
 */
esp_err_t modbus_discover_delete_register(uint8_t slave_id,
                                           uint16_t reg_addr);

/**
 * @brief Semantic inference: guess measurement name and unit from
 *        register address and sample value.
 *
 * @param[in]  reg_addr   Register address (e.g. 40001).
 * @param[in]  value      Sample floating-point value.
 * @param[out] name       Buffer for inferred name (min 32 bytes).
 * @param[out] unit       Buffer for inferred unit (min 16 bytes).
 * @param[out] range_min  Suggested valid range minimum.
 * @param[out] range_max  Suggested valid range maximum.
 */
void modbus_discover_infer_semantics(uint16_t reg_addr,
                                     float value,
                                     char *name,
                                     char *unit,
                                     float *range_min,
                                     float *range_max);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_DISCOVER_H */
