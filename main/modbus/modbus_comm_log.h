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
    int64_t timestamp_ms;   /* Fixed Unix epoch time, or 0 before time sync. */
    int64_t uptime_ms;      /* Fixed monotonic event time since boot. */
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
void modbus_comm_log_anchor_wall_time(int64_t wall_now_ms,
                                      int64_t uptime_now_ms);
void modbus_comm_log_clear(void);

/** Render a single entry in the same line format used on the SD card. */
void modbus_comm_log_format_line(const modbus_comm_log_entry_t *e,
                                 char *buf, size_t len);

/**
 * @brief Invoke @p cb for every RAM-buffer entry whose uptime exceeds
 *        @p min_uptime_ms (i.e. not yet flushed to the SD card), oldest first.
 *        Used by the combined export to append the live tail after the SD
 *        history without duplicating persisted lines.
 */
void modbus_comm_log_iterate_tail(int64_t min_uptime_ms,
                                  void (*cb)(const char *line, void *arg),
                                  void *arg);

#ifdef __cplusplus
}
#endif

#endif
