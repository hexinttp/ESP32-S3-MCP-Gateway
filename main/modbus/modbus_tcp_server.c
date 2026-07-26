#include "modbus/modbus_tcp_server.h"

#include <string.h>
#include "amm/amm_mapping.h"
#include "config/runtime_config.h"
#include "lwip/sockets.h"
#include "modbus/modbus_access.h"
#include "services/control_service.h"
#include "tcm/tcm_state_pool.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "MB_TCP_SERVER";
static TaskHandle_t s_server_task;
static SemaphoreHandle_t s_mutex;
static int s_listen_fd = -1;
static modbus_tcp_server_status_t s_status;
static uint8_t s_max_clients = 4;

static int receive_all(int fd, uint8_t *buffer, size_t size)
{
    size_t received = 0;
    while (received < size) {
        int count = recv(fd, buffer + received, size - received, 0);
        if (count <= 0) return count;
        received += (size_t)count;
    }
    return (int)received;
}

static int send_all(int fd, const uint8_t *buffer, size_t size)
{
    size_t sent = 0;
    while (sent < size) {
        int count = send(fd, buffer + sent, size - sent, 0);
        if (count <= 0) return count;
        sent += (size_t)count;
    }
    return (int)sent;
}

static esp_err_t find_mapping(uint8_t unit, uint8_t function_code,
                              uint16_t offset, amm_mapping_entry_t *mapping,
                              uint8_t *word_index)
{
    if (amm_find_mapping_covering(unit, function_code, offset,
                                  mapping, word_index) == ESP_OK) {
        return ESP_OK;
    }
    uint16_t logical = offset;
    if (function_code == 1) logical = offset + 1U;
    else if (function_code == 2) logical = offset + 10001U;
    else if (function_code == 3) logical = offset + 40001U;
    else if (function_code == 4) logical = offset + 30001U;
    return amm_find_mapping_covering(unit, function_code, logical,
                                     mapping, word_index);
}

static bool read_word(uint8_t unit, uint8_t function_code, uint16_t offset,
                      uint16_t *word)
{
    amm_mapping_entry_t mapping;
    uint8_t word_index = 0;
    if (find_mapping(unit, function_code, offset, &mapping, &word_index) != ESP_OK) {
        return false;
    }
    tcm_context_t state;
    if (tcm_state_pool_get(mapping.device_id, mapping.point_id, &state) != ESP_OK ||
        state.quality_state == QUALITY_INVALID) {
        return false;
    }
    if (function_code <= 2) {
        *word = state.value != 0.0 ? 1U : 0U;
        return true;
    }
    uint16_t words[4];
    uint8_t count = modbus_encode_number(state.raw_value, mapping.data_type,
                                         mapping.byte_order, words);
    if (word_index >= count) return false;
    *word = words[word_index];
    return true;
}

static uint8_t write_single(uint8_t unit, uint8_t function_code,
                            uint16_t offset, uint16_t raw)
{
    uint8_t read_function = function_code == 5 ? 1 : 3;
    amm_mapping_entry_t mapping;
    uint8_t word_index = 0;
    if (find_mapping(unit, read_function, offset, &mapping, &word_index) != ESP_OK ||
        word_index != 0 || !mapping.constraint.writable) {
        return 2;
    }
    double raw_numeric = function_code == 5 ? (raw == 0xFF00U ? 1.0 : 0.0)
                                            : (double)raw;
    double engineering = raw_numeric * mapping.scale_factor + mapping.offset;
    control_result_t result;
    return control_service_write_point(mapping.device_id, mapping.point_id,
                                       engineering, CONTROL_SOURCE_WEB,
                                       &result) == ESP_OK ? 0 : 4;
}

static uint8_t write_multiple_coils(uint8_t unit, uint16_t address,
                                    uint16_t quantity, const uint8_t *packed)
{
    for (uint16_t index = 0; index < quantity; ++index) {
        uint16_t raw = packed[index / 8U] & (1U << (index % 8U))
            ? 0xFF00U : 0x0000U;
        uint8_t exception = write_single(unit, 5, address + index, raw);
        if (exception != 0) return exception;
    }
    return 0;
}

static uint8_t write_multiple_registers(uint8_t unit, uint16_t address,
                                        uint16_t quantity, const uint8_t *bytes)
{
    uint16_t values[123];
    for (uint16_t index = 0; index < quantity; ++index) {
        values[index] = (uint16_t)bytes[index * 2U] << 8 |
                        bytes[index * 2U + 1U];
    }

    for (uint16_t index = 0; index < quantity;) {
        amm_mapping_entry_t mapping;
        uint8_t word_index = 0;
        if (find_mapping(unit, 3, address + index, &mapping, &word_index) != ESP_OK ||
            word_index != 0 || !mapping.constraint.writable) {
            return 2;
        }
        uint8_t count = modbus_register_count_for_type(mapping.data_type);
        if (mapping.data_type == DT_ASCII) count = mapping.read_register_count;
        if (count == 0 || count > 4 || index + count > quantity ||
            mapping.data_type == DT_ASCII) {
            return 3;
        }
        double raw = modbus_convert_to_number(&values[index], mapping.data_type,
                                              mapping.byte_order,
                                              mapping.bit_index);
        double engineering = raw * mapping.scale_factor + mapping.offset;
        control_result_t result;
        if (control_service_write_point(mapping.device_id, mapping.point_id,
                                        engineering, CONTROL_SOURCE_WEB,
                                        &result) != ESP_OK) {
            return 4;
        }
        index += count;
    }
    return 0;
}

static size_t exception_response(uint8_t function_code, uint8_t exception,
                                 uint8_t *response)
{
    response[0] = function_code | 0x80U;
    response[1] = exception;
    return 2;
}

static size_t process_pdu(uint8_t unit, const uint8_t *request, size_t request_size,
                          uint8_t *response)
{
    if (request_size < 5) return exception_response(0, 3, response);
    uint8_t function_code = request[0];
    uint16_t address = (uint16_t)request[1] << 8 | request[2];
    uint16_t quantity = (uint16_t)request[3] << 8 | request[4];

    if (function_code >= 1 && function_code <= 4) {
        uint16_t limit = function_code <= 2 ? 2000 : 125;
        if (quantity == 0 || quantity > limit) {
            return exception_response(function_code, 3, response);
        }
        if (function_code <= 2) {
            uint8_t byte_count = (uint8_t)((quantity + 7U) / 8U);
            response[0] = function_code;
            response[1] = byte_count;
            memset(response + 2, 0, byte_count);
            for (uint16_t i = 0; i < quantity; ++i) {
                uint16_t value;
                if (!read_word(unit, function_code, address + i, &value)) {
                    return exception_response(function_code, 2, response);
                }
                if (value) response[2 + i / 8U] |= 1U << (i % 8U);
            }
            return 2U + byte_count;
        }
        response[0] = function_code;
        response[1] = (uint8_t)(quantity * 2U);
        for (uint16_t i = 0; i < quantity; ++i) {
            uint16_t value;
            if (!read_word(unit, function_code, address + i, &value)) {
                return exception_response(function_code, 2, response);
            }
            response[2 + i * 2U] = (uint8_t)(value >> 8);
            response[3 + i * 2U] = (uint8_t)value;
        }
        return 2U + quantity * 2U;
    }

    if ((function_code == 5 || function_code == 6) && request_size == 5) {
        uint8_t exception = write_single(unit, function_code, address, quantity);
        if (exception != 0) return exception_response(function_code, exception, response);
        memcpy(response, request, 5);
        return 5;
    }
    if (function_code == 15 || function_code == 16) {
        uint16_t limit = function_code == 15 ? 1968 : 123;
        if (request_size < 6 || quantity == 0 || quantity > limit) {
            return exception_response(function_code, 3, response);
        }
        uint8_t byte_count = request[5];
        uint16_t expected = function_code == 15
            ? (uint16_t)((quantity + 7U) / 8U)
            : (uint16_t)(quantity * 2U);
        if (byte_count != expected || request_size != (size_t)byte_count + 6U) {
            return exception_response(function_code, 3, response);
        }
        uint8_t exception = function_code == 15
            ? write_multiple_coils(unit, address, quantity, request + 6)
            : write_multiple_registers(unit, address, quantity, request + 6);
        if (exception != 0) {
            return exception_response(function_code, exception, response);
        }
        memcpy(response, request, 5);
        return 5;
    }
    return exception_response(function_code, 1, response);
}

static void client_task(void *argument)
{
    int fd = (int)(intptr_t)argument;
    uint8_t header[7];
    uint8_t pdu[253];
    uint8_t response[260];
    while (receive_all(fd, header, sizeof(header)) == sizeof(header)) {
        uint16_t length = (uint16_t)header[4] << 8 | header[5];
        if (length < 2 || length > 254 ||
            receive_all(fd, pdu, length - 1U) != length - 1U) break;
        size_t response_size = process_pdu(header[6], pdu, length - 1U,
                                           response + 7);
        memcpy(response, header, 4);
        uint16_t response_length = (uint16_t)(response_size + 1U);
        response[4] = (uint8_t)(response_length >> 8);
        response[5] = (uint8_t)response_length;
        response[6] = header[6];
        if (send_all(fd, response, response_size + 7U) <= 0) break;
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        ++s_status.requests;
        if (response[7] & 0x80U) ++s_status.exceptions;
        else if (response[7] == 5 || response[7] == 6 ||
                 response[7] == 15 || response[7] == 16) ++s_status.writes;
        xSemaphoreGive(s_mutex);
    }
    close(fd);
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_status.active_clients > 0) --s_status.active_clients;
    xSemaphoreGive(s_mutex);
    vTaskDelete(NULL);
}

static void server_task(void *argument)
{
    (void)argument;
    runtime_config_t config;
    runtime_config_get(&config);
    s_listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    int reuse = 1;
    setsockopt(s_listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    struct sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_port = htons(config.modbus_tcp_server.port),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (s_listen_fd < 0 ||
        bind(s_listen_fd, (struct sockaddr *)&address, sizeof(address)) != 0 ||
        listen(s_listen_fd, config.modbus_tcp_server.max_clients) != 0) {
        ESP_LOGE(TAG, "Unable to listen on port %u", config.modbus_tcp_server.port);
        if (s_listen_fd >= 0) close(s_listen_fd);
        s_listen_fd = -1;
        s_server_task = NULL;
        vTaskDelete(NULL);
        return;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_status.running = true;
    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "Northbound Modbus TCP server listening on %u",
             config.modbus_tcp_server.port);

    while (true) {
        int fd = accept(s_listen_fd, NULL, NULL);
        if (fd < 0) break;
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        bool full = s_status.active_clients >= s_max_clients;
        if (!full) ++s_status.active_clients;
        xSemaphoreGive(s_mutex);
        if (full || xTaskCreate(client_task, "mbtcp_client", 4096,
                                (void *)(intptr_t)fd, 4, NULL) != pdPASS) {
            close(fd);
            if (!full) {
                xSemaphoreTake(s_mutex, portMAX_DELAY);
                --s_status.active_clients;
                xSemaphoreGive(s_mutex);
            }
        }
    }
    s_server_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t modbus_tcp_server_start(void)
{
    runtime_config_t config;
    runtime_config_get(&config);
    if (!config.modbus_tcp_server.enabled) return ESP_OK;
    if (s_mutex == NULL) s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) return ESP_ERR_NO_MEM;
    if (s_server_task != NULL) return ESP_OK;
    memset(&s_status, 0, sizeof(s_status));
    s_status.port = config.modbus_tcp_server.port;
    s_max_clients = config.modbus_tcp_server.max_clients > 0
        ? config.modbus_tcp_server.max_clients : 4;
    return xTaskCreate(server_task, "mb_tcp_server", 4096, NULL, 4,
                       &s_server_task) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void modbus_tcp_server_stop(void)
{
    if (s_listen_fd >= 0) {
        shutdown(s_listen_fd, SHUT_RDWR);
        close(s_listen_fd);
        s_listen_fd = -1;
    }
    if (s_mutex != NULL) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_status.running = false;
        xSemaphoreGive(s_mutex);
    }
}

void modbus_tcp_server_get_status(modbus_tcp_server_status_t *out)
{
    if (out == NULL || s_mutex == NULL) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_status;
    xSemaphoreGive(s_mutex);
}
