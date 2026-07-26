/**
 * @file modbus_access.c
 * @brief MODBUS RTU/TCP master access layer implementation.
 *
 * Wraps the ESP-IDF freemodbus library (mbcontroller.h) and exposes a
 * simple, thread-safe API for reading and writing registers on MODBUS
 * slave devices.  A single static mutex serialises every bus transaction
 * so that multiple FreeRTOS tasks may call into this layer concurrently
 * without corrupting the shared UART or TCP socket.
 */

#include <string.h>
#include <math.h>
#include <errno.h>
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "config/runtime_config.h"

#include "modbus_access.h"
#include "modbus/modbus_comm_log.h"
#include "gateway_config.h"

#include "mbcontroller.h"           /* ESP-IDF freemodbus master API     */
#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

/* ======================== Compile-time Constants ======================== */

static const char *TAG = "MODBUS";

/** Maximum number of 16-bit registers we ever read / write in one call. */
#define MODBUS_MAX_REG_COUNT        64

/** Timeout for acquiring the bus mutex (ms). */
#define MODBUS_MUTEX_TIMEOUT_MS     3000

/* ======================== Static State ======================== */

/** Serialises every MODBUS bus transaction. */
static SemaphoreHandle_t s_modbus_mutex = NULL;
static volatile bool s_probe_mode = false;

/** Opaque handle returned by mbc_master_create_serial(). */
static void *s_master_handle = NULL;

/** true once a controller (RTU or TCP) has been successfully started. */
static bool s_initialised = false;

static uint16_t s_tcp_transaction_id;

/*
 * esp-modbus 2.x requires a non-empty descriptor table before start, even
 * when all transactions use mbc_master_send_request() dynamically.
 */
static const mb_parameter_descriptor_t s_dynamic_request_descriptor[] = {
    {
        .cid = 0,
        .param_key = "dynamic_request",
        .param_units = "",
        .mb_slave_addr = 1,
        .mb_param_type = MB_PARAM_HOLDING,
        .mb_reg_start = 0,
        .mb_size = 1,
        .param_offset = 0,
        .param_type = PARAM_TYPE_U16,
        .param_size = sizeof(uint16_t),
        .param_opts = {.opt1 = 0, .opt2 = 0, .opt3 = 0},
        .access = PAR_PERMS_READ_WRITE,
    },
};

static uint16_t modbus_rtu_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 1U) ? (uint16_t)((crc >> 1) ^ 0xA001U) :
                               (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

static size_t append_rtu_crc(uint8_t *frame, size_t payload_length)
{
    uint16_t crc = modbus_rtu_crc16(frame, payload_length);
    frame[payload_length] = (uint8_t)crc;
    frame[payload_length + 1] = (uint8_t)(crc >> 8);
    return payload_length + 2;
}

static size_t build_rtu_read_request(uint8_t *frame, uint8_t slave_id,
                                     uint8_t function_code, uint16_t reg_addr,
                                     uint16_t reg_count)
{
    frame[0] = slave_id;
    frame[1] = function_code;
    frame[2] = (uint8_t)(reg_addr >> 8);
    frame[3] = (uint8_t)reg_addr;
    frame[4] = (uint8_t)(reg_count >> 8);
    frame[5] = (uint8_t)reg_count;
    return append_rtu_crc(frame, 6);
}

static size_t build_rtu_read_response(uint8_t *frame, uint8_t slave_id,
                                      uint8_t function_code,
                                      const uint16_t *values,
                                      uint16_t reg_count)
{
    frame[0] = slave_id;
    frame[1] = function_code;
    frame[2] = (uint8_t)(reg_count * 2U);
    for (uint16_t i = 0; i < reg_count; ++i) {
        frame[3 + i * 2] = (uint8_t)(values[i] >> 8);
        frame[4 + i * 2] = (uint8_t)values[i];
    }
    return append_rtu_crc(frame, 3 + reg_count * 2U);
}

static size_t build_rtu_write_request(uint8_t *frame, uint8_t slave_id,
                                      uint8_t function_code, uint16_t reg_addr,
                                      uint16_t reg_count,
                                      const uint16_t *values)
{
    frame[0] = slave_id;
    frame[1] = function_code;
    frame[2] = (uint8_t)(reg_addr >> 8);
    frame[3] = (uint8_t)reg_addr;
    if (function_code == 6) {
        frame[4] = (uint8_t)(values[0] >> 8);
        frame[5] = (uint8_t)values[0];
        return append_rtu_crc(frame, 6);
    }
    frame[4] = (uint8_t)(reg_count >> 8);
    frame[5] = (uint8_t)reg_count;
    frame[6] = (uint8_t)(reg_count * 2U);
    for (uint16_t i = 0; i < reg_count; ++i) {
        frame[7 + i * 2] = (uint8_t)(values[i] >> 8);
        frame[8 + i * 2] = (uint8_t)values[i];
    }
    return append_rtu_crc(frame, 7 + reg_count * 2U);
}

static esp_err_t socket_send_all(int socket_fd, const uint8_t *data, size_t size)
{
    size_t sent = 0;
    while (sent < size) {
        int result = send(socket_fd, data + sent, size - sent, 0);
        if (result <= 0) return ESP_FAIL;
        sent += (size_t)result;
    }
    return ESP_OK;
}

static esp_err_t socket_receive_all(int socket_fd, uint8_t *data, size_t size)
{
    size_t received = 0;
    while (received < size) {
        int result = recv(socket_fd, data + received, size - received, 0);
        if (result <= 0) return result == 0 ? ESP_ERR_INVALID_RESPONSE : ESP_FAIL;
        received += (size_t)result;
    }
    return ESP_OK;
}

static esp_err_t tcp_endpoint(uint8_t channel_id, runtime_modbus_tcp_endpoint_t *out)
{
    runtime_config_t config;
    runtime_config_get(&config);
    for (int i = 0; i < config.tcp_endpoint_count; ++i) {
        if (config.tcp_endpoints[i].enabled && config.tcp_endpoints[i].endpoint_id == channel_id) {
            *out = config.tcp_endpoints[i];
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t tcp_exchange(uint8_t channel_id, uint8_t unit_id,
                              const uint8_t *pdu, size_t pdu_size,
                              uint8_t *response, size_t *response_size)
{
    runtime_modbus_tcp_endpoint_t endpoint;
    ESP_RETURN_ON_ERROR(tcp_endpoint(channel_id, &endpoint), TAG, "TCP endpoint not found");
    char port[8];
    snprintf(port, sizeof(port), "%u", endpoint.port ? endpoint.port : 502);
    struct addrinfo hints = {.ai_family = AF_INET, .ai_socktype = SOCK_STREAM};
    struct addrinfo *address = NULL;
    if (getaddrinfo(endpoint.host, port, &hints, &address) != 0 || address == NULL) return ESP_ERR_NOT_FOUND;
    int fd = socket(address->ai_family, address->ai_socktype, address->ai_protocol);
    if (fd < 0) { freeaddrinfo(address); return ESP_FAIL; }
    struct timeval timeout = {
        .tv_sec = endpoint.timeout_ms / 1000,
        .tv_usec = (endpoint.timeout_ms % 1000) * 1000,
    };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    if (connect(fd, address->ai_addr, address->ai_addrlen) != 0) {
        freeaddrinfo(address); close(fd); return ESP_ERR_TIMEOUT;
    }
    freeaddrinfo(address);

    uint8_t request[260];
    uint16_t transaction = ++s_tcp_transaction_id;
    request[0] = transaction >> 8; request[1] = transaction;
    request[2] = 0; request[3] = 0;
    uint16_t length = (uint16_t)(pdu_size + 1);
    request[4] = length >> 8; request[5] = length;
    request[6] = unit_id;
    memcpy(request + 7, pdu, pdu_size);
    esp_err_t err = socket_send_all(fd, request, 7 + pdu_size);
    uint8_t header[7];
    if (err == ESP_OK) err = socket_receive_all(fd, header, sizeof(header));
    if (err == ESP_OK) {
        uint16_t body_size = ((uint16_t)header[4] << 8 | header[5]);
        if (body_size < 2 || body_size - 1 > *response_size || header[0] != request[0] || header[1] != request[1]) {
            err = ESP_ERR_INVALID_RESPONSE;
        } else {
            *response_size = body_size - 1;
            err = socket_receive_all(fd, response, *response_size);
            if (err == ESP_OK && (response[0] & 0x80U)) err = ESP_ERR_INVALID_RESPONSE;
        }
    }
    close(fd);
    return err;
}

static esp_err_t modbus_tcp_read(uint8_t channel_id, uint8_t slave_id, uint8_t function_code,
                                 uint16_t reg_addr, uint16_t reg_count, uint16_t *raw_regs)
{
    uint8_t pdu[5] = {function_code, reg_addr >> 8, reg_addr, reg_count >> 8, reg_count};
    uint8_t response[260];
    size_t response_size = sizeof(response);
    esp_err_t err = tcp_exchange(channel_id, slave_id, pdu, sizeof(pdu), response, &response_size);
    if (err != ESP_OK) return err;
    if (response[0] != function_code) return ESP_ERR_INVALID_RESPONSE;
    if (function_code == 1 || function_code == 2) {
        size_t byte_count = (reg_count + 7U) / 8U;
        if (response_size != 2U + byte_count || response[1] != byte_count) {
            return ESP_ERR_INVALID_RESPONSE;
        }
        for (uint16_t i = 0; i < reg_count; ++i) {
            raw_regs[i] = (response[2 + i / 8U] >> (i % 8U)) & 1U;
        }
        return ESP_OK;
    }
    if (response_size != (size_t)(2 + reg_count * 2) ||
        response[1] != reg_count * 2) return ESP_ERR_INVALID_RESPONSE;
    for (int i = 0; i < reg_count; ++i) {
        raw_regs[i] = (uint16_t)response[2 + i * 2] << 8 | response[3 + i * 2];
    }
    return ESP_OK;
}

static esp_err_t modbus_tcp_write(uint8_t channel_id, uint8_t slave_id, uint8_t function_code,
                                  uint16_t reg_addr, uint16_t reg_count, uint16_t *values)
{
    uint8_t pdu[260];
    size_t pdu_size;
    if (function_code == 5 && reg_count == 1) {
        uint16_t coil_value = values[0] ? 0xFF00U : 0x0000U;
        pdu[0] = 5; pdu[1] = reg_addr >> 8; pdu[2] = reg_addr;
        pdu[3] = coil_value >> 8; pdu[4] = coil_value; pdu_size = 5;
    } else if (function_code == 15 && reg_count <= 1968) {
        uint8_t byte_count = (uint8_t)((reg_count + 7U) / 8U);
        pdu[0] = 15; pdu[1] = reg_addr >> 8; pdu[2] = reg_addr;
        pdu[3] = reg_count >> 8; pdu[4] = reg_count; pdu[5] = byte_count;
        memset(pdu + 6, 0, byte_count);
        for (uint16_t i = 0; i < reg_count; ++i) {
            if (values[i]) pdu[6 + i / 8U] |= 1U << (i % 8U);
        }
        pdu_size = 6 + byte_count;
    } else if (function_code == 6 && reg_count == 1) {
        pdu[0] = 6; pdu[1] = reg_addr >> 8; pdu[2] = reg_addr;
        pdu[3] = values[0] >> 8; pdu[4] = values[0]; pdu_size = 5;
    } else if (function_code == 16 && reg_count <= 123) {
        pdu[0] = 16; pdu[1] = reg_addr >> 8; pdu[2] = reg_addr;
        pdu[3] = reg_count >> 8; pdu[4] = reg_count; pdu[5] = reg_count * 2;
        for (int i = 0; i < reg_count; ++i) { pdu[6 + i * 2] = values[i] >> 8; pdu[7 + i * 2] = values[i]; }
        pdu_size = 6 + reg_count * 2;
    } else return ESP_ERR_INVALID_ARG;
    uint8_t response[16];
    size_t response_size = sizeof(response);
    esp_err_t err = tcp_exchange(channel_id, slave_id, pdu, pdu_size, response, &response_size);
    if (err != ESP_OK) return err;
    return response_size == 5 && response[0] == function_code ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

/* ======================== Internal Helpers ======================== */

/**
 * @brief Lock the bus mutex.
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if the lock cannot be acquired.
 */
static esp_err_t modbus_lock(void)
{
    if (s_modbus_mutex == NULL) {
        ESP_LOGE(TAG, "Mutex not initialised");
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_modbus_mutex,
                       pdMS_TO_TICKS(MODBUS_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire bus mutex within %d ms",
                 MODBUS_MUTEX_TIMEOUT_MS);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

/**
 * @brief Release the bus mutex.
 */
static void modbus_unlock(void)
{
    if (s_modbus_mutex != NULL) {
        xSemaphoreGive(s_modbus_mutex);
    }
}

/**
 * @brief Execute a MODBUS read request (FC 03 or FC 04).
 *
 * Builds an @c mb_parameter_descriptor_t and @c mb_request_param_t on the
 * stack, sends the request through the freemodbus master controller, and
 * returns the raw 16-bit register values to the caller.
 *
 * @param[in]  slave_id   Target slave / unit ID.
 * @param[in]  reg_type   MB_PARAM_HOLDING or MB_PARAM_INPUT.
 * @param[in]  reg_addr   Starting register address.
 * @param[in]  reg_count  Number of consecutive registers to read.
 * @param[out] raw_regs   Caller buffer of at least @p reg_count entries.
 * @return ESP_OK on success.
 */
static esp_err_t modbus_read_registers_internal(uint8_t slave_id,
                                                mb_param_type_t reg_type,
                                                uint16_t reg_addr,
                                                uint16_t reg_count,
                                                uint16_t *raw_regs)
{
    if (raw_regs == NULL || reg_count == 0 ||
        reg_count > MODBUS_MAX_REG_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialised) {
        ESP_LOGE(TAG, "Controller not initialised");
        return ESP_ERR_INVALID_STATE;
    }

    /* Zero the output buffer so partial failures don't leak stale data. */
    memset(raw_regs, 0, reg_count * sizeof(uint16_t));

    mb_param_request_t req = {
        .slave_addr      = slave_id,
        .reg_start       = reg_addr,
        .reg_size        = reg_count,
        .command         = (reg_type == MB_PARAM_HOLDING) ? 3 : 4,
    };
    uint8_t function_code = (reg_type == MB_PARAM_HOLDING) ? 3 : 4;
    uint8_t tx_frame[8];
    size_t tx_length = build_rtu_read_request(tx_frame, slave_id, function_code,
                                              reg_addr, reg_count);

    /* ---- Send request (under lock) ---- */
    esp_err_t err = modbus_lock();
    if (err != ESP_OK) {
        return err;
    }

    modbus_comm_log_add(MODBUS_COMM_TX, slave_id, function_code, reg_addr,
                        reg_count, ESP_OK, tx_frame, tx_length);
    esp_err_t mb_err = mbc_master_send_request(s_master_handle, &req, raw_regs);

    modbus_unlock();

    if (mb_err == ESP_OK) {
        uint8_t rx_frame[3 + MODBUS_MAX_REG_COUNT * 2 + 2];
        size_t rx_length = build_rtu_read_response(rx_frame, slave_id,
                                                   function_code, raw_regs,
                                                   reg_count);
        modbus_comm_log_add(MODBUS_COMM_RX, slave_id, function_code, reg_addr,
                            reg_count, ESP_OK, rx_frame, rx_length);
    } else {
        modbus_comm_log_add(MODBUS_COMM_RX, slave_id, function_code, reg_addr,
                            reg_count, mb_err, NULL, 0);
    }

    if (mb_err != ESP_OK && !s_probe_mode) {
        ESP_LOGE(TAG, "Read FC%02d slave=%u addr=%u cnt=%u failed: %s",
                 (reg_type == MB_PARAM_HOLDING) ? 3 : 4,
                 slave_id, reg_addr, reg_count, esp_err_to_name(mb_err));
    } else {
        ESP_LOGD(TAG, "Read FC%02d slave=%u addr=%u cnt=%u OK",
                 (reg_type == MB_PARAM_HOLDING) ? 3 : 4,
                 slave_id, reg_addr, reg_count);
    }

    return mb_err;
}

static esp_err_t modbus_read_bits_internal(uint8_t slave_id, uint8_t function_code,
                                           uint16_t address, uint16_t count,
                                           uint16_t *values)
{
    if (values == NULL || count == 0 || count > MODBUS_MAX_REG_COUNT ||
        (function_code != 1 && function_code != 2) || !s_initialised) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t packed[(MODBUS_MAX_REG_COUNT + 7) / 8] = {0};
    mb_param_request_t req = {
        .slave_addr = slave_id,
        .reg_start = address,
        .reg_size = count,
        .command = function_code,
    };
    uint8_t tx_frame[8];
    size_t tx_length = build_rtu_read_request(tx_frame, slave_id, function_code,
                                              address, count);
    ESP_RETURN_ON_ERROR(modbus_lock(), TAG, "RTU bit lock");
    modbus_comm_log_add(MODBUS_COMM_TX, slave_id, function_code, address,
                        count, ESP_OK, tx_frame, tx_length);
    esp_err_t err = mbc_master_send_request(s_master_handle, &req, packed);
    modbus_unlock();
    if (err == ESP_OK) {
        for (uint16_t i = 0; i < count; ++i) {
            values[i] = (packed[i / 8U] >> (i % 8U)) & 1U;
        }
        uint8_t rx_frame[3 + (MODBUS_MAX_REG_COUNT + 7) / 8 + 2] = {
            slave_id, function_code, (uint8_t)((count + 7U) / 8U)
        };
        memcpy(rx_frame + 3, packed, rx_frame[2]);
        size_t rx_length = append_rtu_crc(rx_frame, 3 + rx_frame[2]);
        modbus_comm_log_add(MODBUS_COMM_RX, slave_id, function_code, address,
                            count, ESP_OK, rx_frame, rx_length);
    } else {
        modbus_comm_log_add(MODBUS_COMM_RX, slave_id, function_code, address,
                            count, err, NULL, 0);
    }
    return err;
}

void modbus_access_set_probe_mode(bool enabled)
{
    s_probe_mode = enabled;
}

/**
 * @brief Execute a MODBUS write request (FC 06 or FC 16).
 *
 * @param[in] slave_id   Target slave / unit ID.
 * @param[in] reg_addr   Starting register address.
 * @param[in] reg_count  Number of registers to write (1 for FC06, >1 for FC16).
 * @param[in] values     Array of 16-bit values to write.
 * @param[in] cmd_type   MB_CMD_WRITE_HOLDING or MB_CMD_WRITE_MULTIPLE.
 * @return ESP_OK on success.
 */
static esp_err_t modbus_write_registers_internal(uint8_t slave_id,
                                                 uint16_t reg_addr,
                                                 uint16_t reg_count,
                                                 uint16_t *values,
                                                 uint8_t cmd_type)
{
    if (values == NULL || reg_count == 0 ||
        reg_count > MODBUS_MAX_REG_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialised) {
        ESP_LOGE(TAG, "Controller not initialised");
        return ESP_ERR_INVALID_STATE;
    }

    mb_param_request_t req = {
        .slave_addr      = slave_id,
        .reg_start       = reg_addr,
        .reg_size        = reg_count,
        .command         = cmd_type,
    };
    uint8_t function_code = (cmd_type == 6) ? 6 : 16;
    uint8_t tx_frame[9 + MODBUS_MAX_REG_COUNT * 2];
    size_t tx_length = build_rtu_write_request(tx_frame, slave_id,
                                               function_code, reg_addr,
                                               reg_count, values);

    /* ---- Send request (under lock) ---- */
    esp_err_t err = modbus_lock();
    if (err != ESP_OK) {
        return err;
    }

    modbus_comm_log_add(MODBUS_COMM_TX, slave_id, function_code, reg_addr,
                        reg_count, ESP_OK, tx_frame, tx_length);
    esp_err_t mb_err = mbc_master_send_request(s_master_handle, &req, values);

    modbus_unlock();

    if (mb_err == ESP_OK) {
        uint8_t rx_frame[8];
        size_t rx_length;
        if (function_code == 6) {
            memcpy(rx_frame, tx_frame, sizeof(rx_frame));
            rx_length = sizeof(rx_frame);
        } else {
            rx_frame[0] = slave_id;
            rx_frame[1] = function_code;
            rx_frame[2] = (uint8_t)(reg_addr >> 8);
            rx_frame[3] = (uint8_t)reg_addr;
            rx_frame[4] = (uint8_t)(reg_count >> 8);
            rx_frame[5] = (uint8_t)reg_count;
            rx_length = append_rtu_crc(rx_frame, 6);
        }
        modbus_comm_log_add(MODBUS_COMM_RX, slave_id, function_code, reg_addr,
                            reg_count, ESP_OK, rx_frame, rx_length);
    } else {
        modbus_comm_log_add(MODBUS_COMM_RX, slave_id, function_code, reg_addr,
                            reg_count, mb_err, NULL, 0);
    }

    if (mb_err != ESP_OK) {
        ESP_LOGE(TAG, "Write FC%02d slave=%u addr=%u cnt=%u failed: %s",
                 (cmd_type == 6) ? 6 : 16,
                 slave_id, reg_addr, reg_count, esp_err_to_name(mb_err));
    } else {
        ESP_LOGD(TAG, "Write FC%02d slave=%u addr=%u cnt=%u OK",
                 (cmd_type == 6) ? 6 : 16,
                 slave_id, reg_addr, reg_count);
    }

    return mb_err;
}

static esp_err_t modbus_write_coils_internal(uint8_t slave_id, uint16_t address,
                                             uint16_t count, uint16_t *values,
                                             uint8_t function_code)
{
    if (values == NULL || count == 0 || count > MODBUS_MAX_REG_COUNT ||
        (function_code != 5 && function_code != 15) || !s_initialised) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t packed[(MODBUS_MAX_REG_COUNT + 7) / 8] = {0};
    uint16_t single = values[0] ? 0xFF00U : 0x0000U;
    for (uint16_t i = 0; i < count; ++i) {
        if (values[i]) packed[i / 8U] |= 1U << (i % 8U);
    }
    mb_param_request_t req = {
        .slave_addr = slave_id,
        .reg_start = address,
        .reg_size = count,
        .command = function_code,
    };
    void *payload = function_code == 5 ? (void *)&single : (void *)packed;
    ESP_RETURN_ON_ERROR(modbus_lock(), TAG, "RTU coil lock");
    esp_err_t err = mbc_master_send_request(s_master_handle, &req, payload);
    modbus_unlock();
    return err;
}

/* ======================== Public API ======================== */

/* ------------------------------------------------------------------ */
esp_err_t modbus_rtu_init(void)
{
    esp_err_t err;
    modbus_comm_log_init();
    runtime_config_t runtime;
    runtime_config_get(&runtime);

    ESP_LOGI(TAG, "Initialising MODBUS RTU master  UART%d  baud=%d  "
             "TX=%d RX=%d RTS=%d",
             MODBUS_RTU_UART_PORT, (int)runtime.modbus_rtu.baud_rate,
             MODBUS_RTU_UART_TXD, MODBUS_RTU_UART_RXD,
             MODBUS_RTU_UART_RTS);

    /* ---- Create the mutex (idempotent) ---- */
    if (s_modbus_mutex == NULL) {
        s_modbus_mutex = xSemaphoreCreateMutex();
        if (s_modbus_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    /* If already running, tear down first so we get a clean state. */
    if (s_initialised) {
        ESP_LOGW(TAG, "Controller already active -- destroying before re-init");
        modbus_destroy();
    }

    mb_communication_info_t comm_info = {
        .ser_opts.port = MODBUS_RTU_UART_PORT,
        .ser_opts.mode = MB_RTU,
        .ser_opts.baudrate = runtime.modbus_rtu.baud_rate,
        .ser_opts.parity = (uart_parity_t)runtime.modbus_rtu.parity,
        .ser_opts.uid = 0,
        .ser_opts.response_tout_ms = runtime.modbus_rtu.timeout_ms,
        .ser_opts.data_bits = UART_DATA_8_BITS,
        .ser_opts.stop_bits = UART_STOP_BITS_1,
    };

    err = mbc_master_create_serial(&comm_info, &s_master_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mbc_master_create_serial() failed: %s", esp_err_to_name(err));
        s_master_handle = NULL;
        return err;
    }

    err = uart_set_pin(MODBUS_RTU_UART_PORT, MODBUS_RTU_UART_TXD,
                       MODBUS_RTU_UART_RXD, MODBUS_RTU_UART_RTS,
                       UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin() failed: %s", esp_err_to_name(err));
        mbc_master_delete(s_master_handle);
        s_master_handle = NULL;
        return err;
    }
    err = mbc_master_set_descriptor(
        s_master_handle, s_dynamic_request_descriptor,
        sizeof(s_dynamic_request_descriptor) /
            sizeof(s_dynamic_request_descriptor[0]));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mbc_master_set_descriptor() failed: %s",
                 esp_err_to_name(err));
        mbc_master_delete(s_master_handle);
        s_master_handle = NULL;
        return err;
    }
    err = uart_set_mode(MODBUS_RTU_UART_PORT, UART_MODE_RS485_HALF_DUPLEX);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_mode() failed: %s", esp_err_to_name(err));
        mbc_master_delete(s_master_handle);
        s_master_handle = NULL;
        return err;
    }

    /* ---- Start the controller ---- */
    err = mbc_master_start(s_master_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mbc_master_start() failed: %s", esp_err_to_name(err));
        mbc_master_delete(s_master_handle);
        s_master_handle = NULL;
        return err;
    }
    s_initialised = true;

    ESP_LOGI(TAG, "MODBUS RTU master ready");
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
esp_err_t modbus_tcp_init(const char *device_ip, uint16_t port)
{
    (void)port;
    return device_ip != NULL ? ESP_OK : ESP_ERR_INVALID_ARG;
}

/* ------------------------------------------------------------------ */
esp_err_t modbus_read_holding_register(uint8_t slave_id,
                                       uint16_t reg_addr,
                                       uint16_t reg_count,
                                       uint16_t *raw_regs)
{
    return modbus_read_registers_internal(slave_id,
                                          MB_PARAM_HOLDING,
                                          reg_addr,
                                          reg_count,
                                          raw_regs);
}

/* ------------------------------------------------------------------ */
esp_err_t modbus_read_input_register(uint8_t slave_id,
                                     uint16_t reg_addr,
                                     uint16_t reg_count,
                                     uint16_t *raw_regs)
{
    return modbus_read_registers_internal(slave_id,
                                          MB_PARAM_INPUT,
                                          reg_addr,
                                          reg_count,
                                          raw_regs);
}

esp_err_t modbus_read_coils(uint8_t slave_id, uint16_t address,
                            uint16_t count, uint16_t *values)
{
    return modbus_read_bits_internal(slave_id, 1, address, count, values);
}

esp_err_t modbus_read_discrete_inputs(uint8_t slave_id, uint16_t address,
                                      uint16_t count, uint16_t *values)
{
    return modbus_read_bits_internal(slave_id, 2, address, count, values);
}

/* ------------------------------------------------------------------ */
esp_err_t modbus_write_single_register(uint8_t slave_id,
                                       uint16_t reg_addr,
                                       uint16_t value)
{
    uint16_t val = value;
    return modbus_write_registers_internal(slave_id,
                                           reg_addr,
                                           1,
                                           &val,
                                           6);
}

/* ------------------------------------------------------------------ */
esp_err_t modbus_write_multiple_registers(uint8_t slave_id,
                                          uint16_t reg_addr,
                                          uint16_t reg_count,
                                          uint16_t *values)
{
    return modbus_write_registers_internal(slave_id,
                                           reg_addr,
                                           reg_count,
                                           values,
                                           16);
}

esp_err_t modbus_write_single_coil(uint8_t slave_id, uint16_t address, bool value)
{
    uint16_t word = value ? 1U : 0U;
    return modbus_write_coils_internal(slave_id, address, 1, &word, 5);
}

esp_err_t modbus_write_multiple_coils(uint8_t slave_id, uint16_t address,
                                      uint16_t count, uint16_t *values)
{
    return modbus_write_coils_internal(slave_id, address, count, values, 15);
}

/* ------------------------------------------------------------------ */
static uint16_t swap_word_bytes(uint16_t value)
{
    return (uint16_t)(value << 8 | value >> 8);
}

static uint64_t ordered_bits(const uint16_t *raw_regs, uint8_t count,
                             byte_order_t byte_order)
{
    uint16_t words[4] = {0};
    for (uint8_t i = 0; i < count; ++i) words[i] = raw_regs[i];
    if (byte_order == BYTE_ORDER_CDAB || byte_order == BYTE_ORDER_DCBA) {
        for (uint8_t i = 0; i < count / 2U; ++i) {
            uint16_t temp = words[i];
            words[i] = words[count - 1U - i];
            words[count - 1U - i] = temp;
        }
    }
    if (byte_order == BYTE_ORDER_BADC || byte_order == BYTE_ORDER_DCBA) {
        for (uint8_t i = 0; i < count; ++i) words[i] = swap_word_bytes(words[i]);
    }
    uint64_t bits = 0;
    for (uint8_t i = 0; i < count; ++i) bits = (bits << 16) | words[i];
    return bits;
}

double modbus_convert_to_number(const uint16_t *raw_regs, data_type_t dtype,
                                byte_order_t byte_order, uint8_t bit_index)
{
    if (raw_regs == NULL) return 0.0;
    uint8_t count = modbus_register_count_for_type(dtype);
    uint64_t bits = ordered_bits(raw_regs, count, byte_order);
    switch (dtype) {
    case DT_BOOL: return raw_regs[0] != 0 ? 1.0 : 0.0;
    case DT_INT16: return (double)(int16_t)bits;
    case DT_UINT16: return (double)(uint16_t)bits;
    case DT_FLOAT32: {
        uint32_t raw32 = (uint32_t)bits;
        float value;
        memcpy(&value, &raw32, sizeof(value));
        return value;
    }
    case DT_INT32: return (double)(int32_t)bits;
    case DT_UINT32: return (double)(uint32_t)bits;
    case DT_INT64: return (double)(int64_t)bits;
    case DT_UINT64: return (double)bits;
    case DT_FLOAT64: {
        double value;
        memcpy(&value, &bits, sizeof(value));
        return value;
    }
    case DT_BCD16:
        return ((bits >> 12) & 0xF) * 1000 + ((bits >> 8) & 0xF) * 100 +
               ((bits >> 4) & 0xF) * 10 + (bits & 0xF);
    case DT_BITFIELD16:
        return ((uint16_t)bits >> (bit_index & 15U)) & 1U;
    case DT_ASCII:
    default:
        return 0.0;
    }
}

size_t modbus_decode_ascii(const uint16_t *raw_regs, uint16_t register_count,
                           byte_order_t byte_order, char *out, size_t out_size)
{
    if (raw_regs == NULL || out == NULL || out_size == 0) return 0;
    size_t written = 0;
    for (uint16_t i = 0; i < register_count && written + 1 < out_size; ++i) {
        uint16_t word = raw_regs[i];
        if (byte_order == BYTE_ORDER_BADC || byte_order == BYTE_ORDER_DCBA) {
            word = swap_word_bytes(word);
        }
        char high = (char)(word >> 8);
        char low = (char)word;
        if (high == '\0') break;
        out[written++] = high;
        if (low == '\0' || written + 1 >= out_size) break;
        out[written++] = low;
    }
    out[written] = '\0';
    return written;
}

uint8_t modbus_encode_number(double value, data_type_t dtype,
                             byte_order_t byte_order, uint16_t words[4])
{
    if (words == NULL) return 0;
    memset(words, 0, sizeof(uint16_t) * 4);
    uint64_t bits = 0;
    uint8_t count = modbus_register_count_for_type(dtype);
    switch (dtype) {
    case DT_BOOL: words[0] = value != 0.0; return 1;
    case DT_INT16: words[0] = (uint16_t)(int16_t)llround(value); return 1;
    case DT_UINT16:
    case DT_BITFIELD16: words[0] = (uint16_t)llround(value); return 1;
    case DT_BCD16: {
        uint16_t number = (uint16_t)llround(value);
        words[0] = (uint16_t)(((number / 1000U) % 10U) << 12 |
                              ((number / 100U) % 10U) << 8 |
                              ((number / 10U) % 10U) << 4 |
                              (number % 10U));
        return 1;
    }
    case DT_FLOAT32: {
        float number = (float)value;
        uint32_t raw32;
        memcpy(&raw32, &number, sizeof(raw32));
        bits = raw32;
        break;
    }
    case DT_INT32: bits = (uint32_t)(int32_t)llround(value); break;
    case DT_UINT32: bits = (uint32_t)llround(value); break;
    case DT_FLOAT64: memcpy(&bits, &value, sizeof(bits)); break;
    case DT_INT64: bits = (uint64_t)(int64_t)llround(value); break;
    case DT_UINT64: bits = (uint64_t)llround(value); break;
    default: return 0;
    }
    for (uint8_t i = 0; i < count; ++i) {
        words[i] = (uint16_t)(bits >> (16U * (count - 1U - i)));
    }
    if (byte_order == BYTE_ORDER_CDAB || byte_order == BYTE_ORDER_DCBA) {
        for (uint8_t i = 0; i < count / 2U; ++i) {
            uint16_t temporary = words[i];
            words[i] = words[count - 1U - i];
            words[count - 1U - i] = temporary;
        }
    }
    if (byte_order == BYTE_ORDER_BADC || byte_order == BYTE_ORDER_DCBA) {
        for (uint8_t i = 0; i < count; ++i) words[i] = swap_word_bytes(words[i]);
    }
    return count;
}

uint16_t modbus_register_offset(uint8_t function_code, uint16_t configured_address)
{
    if (function_code == 1 && configured_address >= 1 && configured_address < 10001) {
        return configured_address - 1;
    }
    if (function_code == 2 && configured_address >= 10001 && configured_address < 20001) {
        return configured_address - 10001;
    }
    if ((function_code == 3 || function_code == 6 || function_code == 16) && configured_address >= 40001) {
        return configured_address - 40001;
    }
    if (function_code == 4 && configured_address >= 30001) {
        return configured_address - 30001;
    }
    return configured_address;
}

uint16_t modbus_register_count_for_type(data_type_t data_type)
{
    if (data_type == DT_FLOAT32 || data_type == DT_INT32 || data_type == DT_UINT32) return 2;
    if (data_type == DT_FLOAT64 || data_type == DT_INT64 || data_type == DT_UINT64) return 4;
    return 1;
}

esp_err_t modbus_read_channel(source_protocol_t protocol, uint8_t channel_id,
                              uint8_t slave_id, uint8_t function_code,
                              uint16_t reg_addr, uint16_t reg_count, uint16_t *raw_regs)
{
    if (protocol == SRC_MODBUS_RTU) {
        if (function_code == 1) return modbus_read_coils(slave_id, reg_addr, reg_count, raw_regs);
        if (function_code == 2) return modbus_read_discrete_inputs(slave_id, reg_addr, reg_count, raw_regs);
        if (function_code == 4) return modbus_read_input_register(slave_id, reg_addr, reg_count, raw_regs);
        if (function_code == 3) return modbus_read_holding_register(slave_id, reg_addr, reg_count, raw_regs);
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (s_modbus_mutex == NULL) s_modbus_mutex = xSemaphoreCreateMutex();
    ESP_RETURN_ON_ERROR(modbus_lock(), TAG, "TCP lock");
    esp_err_t err = modbus_tcp_read(channel_id, slave_id, function_code,
                                    reg_addr, reg_count, raw_regs);
    modbus_unlock();
    return err;
}

esp_err_t modbus_write_channel(source_protocol_t protocol, uint8_t channel_id,
                               uint8_t slave_id, uint8_t function_code,
                               uint16_t reg_addr, uint16_t reg_count, uint16_t *values)
{
    if (protocol == SRC_MODBUS_RTU) {
        if (function_code == 5 && reg_count == 1) {
            return modbus_write_single_coil(slave_id, reg_addr, values[0] != 0);
        }
        if (function_code == 15) {
            return modbus_write_multiple_coils(slave_id, reg_addr, reg_count, values);
        }
        if (function_code == 6 && reg_count == 1) {
            return modbus_write_single_register(slave_id, reg_addr, values[0]);
        }
        if (function_code == 16) {
            return modbus_write_multiple_registers(slave_id, reg_addr, reg_count, values);
        }
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (s_modbus_mutex == NULL) s_modbus_mutex = xSemaphoreCreateMutex();
    ESP_RETURN_ON_ERROR(modbus_lock(), TAG, "TCP lock");
    esp_err_t err = modbus_tcp_write(channel_id, slave_id, function_code,
                                     reg_addr, reg_count, values);
    modbus_unlock();
    return err;
}

esp_err_t modbus_read_device_identity_channel(source_protocol_t protocol,
                                              uint8_t channel_id,
                                              uint8_t slave_id,
                                              modbus_device_identity_t *identity)
{
    if (identity == NULL) return ESP_ERR_INVALID_ARG;
    memset(identity, 0, sizeof(*identity));
    if (protocol != SRC_MODBUS_TCP) return ESP_ERR_NOT_SUPPORTED;

    uint8_t pdu[] = {43, 14, 1, 0};
    uint8_t response[253];
    size_t response_size = sizeof(response);
    if (s_modbus_mutex == NULL) s_modbus_mutex = xSemaphoreCreateMutex();
    ESP_RETURN_ON_ERROR(modbus_lock(), TAG, "TCP identity lock");
    esp_err_t err = tcp_exchange(channel_id, slave_id, pdu, sizeof(pdu),
                                 response, &response_size);
    modbus_unlock();
    if (err != ESP_OK) return err;
    if (response_size < 8 || response[0] != 43 || response[1] != 14) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    uint8_t object_count = response[7];
    size_t cursor = 8;
    for (uint8_t i = 0; i < object_count && cursor + 2 <= response_size; ++i) {
        uint8_t object_id = response[cursor++];
        uint8_t length = response[cursor++];
        if (cursor + length > response_size) return ESP_ERR_INVALID_RESPONSE;
        char *destination = NULL;
        size_t capacity = 0;
        if (object_id == 0) {
            destination = identity->vendor_name;
            capacity = sizeof(identity->vendor_name);
        } else if (object_id == 1) {
            destination = identity->product_code;
            capacity = sizeof(identity->product_code);
        } else if (object_id == 2) {
            destination = identity->revision;
            capacity = sizeof(identity->revision);
        }
        if (destination != NULL) {
            size_t copy = length < capacity - 1U ? length : capacity - 1U;
            memcpy(destination, response + cursor, copy);
            destination[copy] = '\0';
        }
        cursor += length;
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
void modbus_destroy(void)
{
    if (!s_initialised) {
        ESP_LOGD(TAG, "modbus_destroy(): nothing to tear down");
        return;
    }

    ESP_LOGI(TAG, "Destroying MODBUS master controller");

    /* Best-effort stop -- ignore errors since we are tearing down. */
    (void)mbc_master_stop(s_master_handle);
    (void)mbc_master_delete(s_master_handle);

    s_master_handle = NULL;
    s_initialised   = false;

    /* Release and nullify the mutex. */
    if (s_modbus_mutex != NULL) {
        vSemaphoreDelete(s_modbus_mutex);
        s_modbus_mutex = NULL;
    }

    ESP_LOGI(TAG, "MODBUS master controller destroyed");
}
