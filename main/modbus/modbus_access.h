/**
 * @file modbus_access.h
 * @brief MODBUS RTU/TCP master access layer for ESP32-S3 gateway.
 *
 * Provides thread-safe read/write operations against MODBUS slave devices
 * using the ESP-IDF freemodbus library.  All public functions acquire an
 * internal mutex so that multiple tasks may call into this layer safely.
 */
#ifndef MODBUS_ACCESS_H
#define MODBUS_ACCESS_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "tcm/tcm_context.h"   /* data_type_t, quality_state_t */

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== Result Container ======================== */

/**
 * @brief Holds the outcome of a single MODBUS register read, including
 *        the converted floating-point value and data-quality metadata.
 */
typedef struct {
    source_protocol_t source_protocol;
    uint8_t         channel_id;
    uint8_t         slave_id;           /**< MODBUS slave / unit ID          */
    uint8_t         function_code;      /**< MODBUS function code used       */
    uint16_t        register_address;   /**< Starting register address       */
    uint16_t        register_count;     /**< Number of registers read        */
    data_type_t     data_type;          /**< Interpretation of raw bytes     */
    float           raw_value;          /**< Converted floating-point value  */
    quality_state_t quality;            /**< Data quality indicator          */
    bool            valid;              /**< true when read succeeded        */
} modbus_read_result_t;

/* ======================== Initialisation ======================== */

/**
 * @brief Initialise the MODBUS RTU master on the UART port and pins
 *        defined in gateway_config.h.
 *
 * Must be called once before any RTU read/write operation.
 *
 * @return ESP_OK on success, or an appropriate esp_err_t on failure.
 */
esp_err_t modbus_rtu_init(void);

/**
 * @brief Initialise the MODBUS TCP master for the given remote device.
 *
 * @param[in] device_ip  IPv4 address string of the target device (e.g. "192.168.1.50").
 * @param[in] port       TCP port number (typically 502).
 * @return ESP_OK on success, or an appropriate esp_err_t on failure.
 */
esp_err_t modbus_tcp_init(const char *device_ip, uint16_t port);

/* ======================== Read Operations ======================== */

/**
 * @brief Read one or more holding registers (MODBUS function code 03).
 *
 * @param[in]  slave_id   Slave / unit ID (1-247).
 * @param[in]  reg_addr   Starting register address (0-based).
 * @param[in]  reg_count  Number of consecutive registers to read.
 * @param[out] raw_regs   Caller-supplied buffer of at least @p reg_count entries.
 * @return ESP_OK on success.
 */
esp_err_t modbus_read_holding_register(uint8_t slave_id,
                                       uint16_t reg_addr,
                                       uint16_t reg_count,
                                       uint16_t *raw_regs);

/**
 * @brief Read one or more input registers (MODBUS function code 04).
 *
 * @param[in]  slave_id   Slave / unit ID (1-247).
 * @param[in]  reg_addr   Starting register address (0-based).
 * @param[in]  reg_count  Number of consecutive registers to read.
 * @param[out] raw_regs   Caller-supplied buffer of at least @p reg_count entries.
 * @return ESP_OK on success.
 */
esp_err_t modbus_read_input_register(uint8_t slave_id,
                                     uint16_t reg_addr,
                                     uint16_t reg_count,
                                     uint16_t *raw_regs);

/* ======================== Write Operations ======================== */

/**
 * @brief Write a single holding register (MODBUS function code 06).
 *
 * @param[in] slave_id  Slave / unit ID (1-247).
 * @param[in] reg_addr  Register address (0-based).
 * @param[in] value     16-bit value to write.
 * @return ESP_OK on success.
 */
esp_err_t modbus_write_single_register(uint8_t slave_id,
                                       uint16_t reg_addr,
                                       uint16_t value);

/**
 * @brief Write multiple consecutive holding registers (MODBUS function code 16).
 *
 * @param[in] slave_id   Slave / unit ID (1-247).
 * @param[in] reg_addr   Starting register address (0-based).
 * @param[in] reg_count  Number of registers to write.
 * @param[in] values     Array of @p reg_count 16-bit values.
 * @return ESP_OK on success.
 */
esp_err_t modbus_write_multiple_registers(uint8_t slave_id,
                                          uint16_t reg_addr,
                                          uint16_t reg_count,
                                          uint16_t *values);

/* ======================== Conversion Utility ======================== */

/**
 * @brief Convert raw 16-bit register values to a float based on the
 *        specified data type.
 *
 * Supported conversions:
 *  - DT_INT16   : single register, sign-extended to float
 *  - DT_UINT16  : single register, zero-extended to float
 *  - DT_FLOAT32 : two consecutive registers (big-endian word order)
 *  - DT_INT32   : two consecutive registers (big-endian word order)
 *  - DT_UINT32  : two consecutive registers (big-endian word order)
 *
 * @param[in] raw_regs  Pointer to one or more raw 16-bit register values.
 * @param[in] dtype     Target data type interpretation.
 * @return Converted floating-point value (0.0f on unsupported type).
 */
float modbus_convert_to_float(uint16_t *raw_regs, data_type_t dtype);
float modbus_convert_to_float_order(const uint16_t *raw_regs, data_type_t dtype,
                                    byte_order_t byte_order);

uint16_t modbus_register_offset(uint8_t function_code, uint16_t configured_address);
uint16_t modbus_register_count_for_type(data_type_t data_type);

esp_err_t modbus_read_channel(source_protocol_t protocol, uint8_t channel_id,
                              uint8_t slave_id, uint8_t function_code,
                              uint16_t reg_addr, uint16_t reg_count, uint16_t *raw_regs);
esp_err_t modbus_write_channel(source_protocol_t protocol, uint8_t channel_id,
                               uint8_t slave_id, uint8_t function_code,
                               uint16_t reg_addr, uint16_t reg_count, uint16_t *values);

/* ======================== Teardown ======================== */

/**
 * @brief Stop and destroy the active MODBUS master controller, releasing
 *        all associated resources (UART, sockets, mutex).
 */
void modbus_destroy(void);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_ACCESS_H */
