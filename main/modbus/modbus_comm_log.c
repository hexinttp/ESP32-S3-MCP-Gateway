#include "modbus/modbus_comm_log.h"

#include <string.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static modbus_comm_log_entry_t s_entries[MODBUS_COMM_LOG_CAPACITY];
static SemaphoreHandle_t s_mutex;
static uint16_t s_next;
static uint16_t s_count;
static uint32_t s_sequence;

void modbus_comm_log_init(void)
{
    if (s_mutex == NULL) s_mutex = xSemaphoreCreateMutex();
}

void modbus_comm_log_add(modbus_comm_direction_t direction,
                         uint8_t slave_id,
                         uint8_t function_code,
                         uint16_t register_address,
                         uint16_t register_count,
                         esp_err_t status,
                         const uint8_t *frame,
                         size_t frame_length)
{
    modbus_comm_log_init();
    if (s_mutex == NULL ||
        xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }

    modbus_comm_log_entry_t *entry = &s_entries[s_next];
    memset(entry, 0, sizeof(*entry));
    entry->sequence = ++s_sequence;
    entry->timestamp_ms = esp_timer_get_time() / 1000;
    entry->direction = direction;
    entry->slave_id = slave_id;
    entry->function_code = function_code;
    entry->register_address = register_address;
    entry->register_count = register_count;
    entry->status = status;
    entry->truncated = frame_length > MODBUS_COMM_FRAME_MAX;
    entry->frame_length = (uint8_t)(entry->truncated ?
                                    MODBUS_COMM_FRAME_MAX : frame_length);
    if (frame != NULL && entry->frame_length > 0) {
        memcpy(entry->frame, frame, entry->frame_length);
    }

    s_next = (uint16_t)((s_next + 1) % MODBUS_COMM_LOG_CAPACITY);
    if (s_count < MODBUS_COMM_LOG_CAPACITY) ++s_count;
    xSemaphoreGive(s_mutex);
}

int modbus_comm_log_snapshot(modbus_comm_log_entry_t *out, int max_entries)
{
    if (out == NULL || max_entries <= 0 || s_mutex == NULL) return 0;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return 0;

    uint16_t copy_count = s_count < (uint16_t)max_entries ?
                          s_count : (uint16_t)max_entries;
    uint16_t oldest = (uint16_t)((s_next + MODBUS_COMM_LOG_CAPACITY -
                                  s_count) % MODBUS_COMM_LOG_CAPACITY);
    uint16_t skip = s_count - copy_count;
    uint16_t start = (uint16_t)((oldest + skip) % MODBUS_COMM_LOG_CAPACITY);
    for (uint16_t i = 0; i < copy_count; ++i) {
        out[i] = s_entries[(start + i) % MODBUS_COMM_LOG_CAPACITY];
    }
    xSemaphoreGive(s_mutex);
    return copy_count;
}

int modbus_comm_log_count(void)
{
    if (s_mutex == NULL ||
        xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return 0;
    }
    int count = s_count;
    xSemaphoreGive(s_mutex);
    return count;
}

bool modbus_comm_log_get(int index, modbus_comm_log_entry_t *out)
{
    if (out == NULL || index < 0 || s_mutex == NULL ||
        xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }
    bool found = index < s_count;
    if (found) {
        uint16_t oldest = (uint16_t)((s_next + MODBUS_COMM_LOG_CAPACITY -
                                      s_count) % MODBUS_COMM_LOG_CAPACITY);
        *out = s_entries[(oldest + (uint16_t)index) %
                         MODBUS_COMM_LOG_CAPACITY];
    }
    xSemaphoreGive(s_mutex);
    return found;
}

void modbus_comm_log_clear(void)
{
    modbus_comm_log_init();
    if (s_mutex == NULL ||
        xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    memset(s_entries, 0, sizeof(s_entries));
    s_next = 0;
    s_count = 0;
    xSemaphoreGive(s_mutex);
}
