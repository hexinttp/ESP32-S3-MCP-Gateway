#ifndef MODBUS_COMM_LOG_H
#define MODBUS_COMM_LOG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MODBUS_COMM_LOG_CAPACITY 64
#define MODBUS_COMM_FRAME_MAX    48

typedef enum {
    MODBUS_COMM_TX = 0,
    MODBUS_COMM_RX = 1,
} modbus_comm_direction_t;

typedef struct {
    uint32_t sequence;
    int64_t timestamp_ms;
    modbus_comm_direction_t direction;
    uint8_t slave_id;
    uint8_t function_code;
    uint16_t register_address;
    uint16_t register_count;
    esp_err_t status;
    uint8_t frame_length;
    bool truncated;
    uint8_t frame[MODBUS_COMM_FRAME_MAX];
} modbus_comm_log_entry_t;

void modbus_comm_log_init(void);
void modbus_comm_log_add(modbus_comm_direction_t direction,
                         uint8_t slave_id,
                         uint8_t function_code,
                         uint16_t register_address,
                         uint16_t register_count,
                         esp_err_t status,
                         const uint8_t *frame,
                         size_t frame_length);
int modbus_comm_log_snapshot(modbus_comm_log_entry_t *out, int max_entries);
int modbus_comm_log_count(void);
bool modbus_comm_log_get(int index, modbus_comm_log_entry_t *out);
void modbus_comm_log_clear(void);

#ifdef __cplusplus
}
#endif

#endif
