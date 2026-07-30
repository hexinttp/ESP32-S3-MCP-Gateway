#include "modbus/modbus_comm_log.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include "board/tf_storage.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "services/time_service.h"
#include "storage/log_store.h"

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
    int64_t uptime_ms = esp_timer_get_time() / 1000LL;
    int64_t timestamp_ms = 0;
    if (time_service_is_synchronized()) {
        struct timeval now;
        gettimeofday(&now, NULL);
        timestamp_ms = (int64_t)now.tv_sec * 1000LL + now.tv_usec / 1000;
    }

    modbus_comm_log_init();
    if (s_mutex == NULL ||
        xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }

    modbus_comm_log_entry_t *entry = &s_entries[s_next];
    memset(entry, 0, sizeof(*entry));
    entry->sequence = ++s_sequence;
    entry->timestamp_ms = timestamp_ms;
    entry->uptime_ms = uptime_ms;
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

    /* Format the on-disk/export line while the slot is still ours; it may be
     * reused by a following add() once we drop the lock. */
    char line[256];
    modbus_comm_log_format_line(entry, line, sizeof(line));
    xSemaphoreGive(s_mutex);

    /* Mirror the entry to the SD card (if mounted) for later export. */
    if (tf_storage_is_mounted() && line[0] != '\0') {
        log_store_append_modbus(line, uptime_ms);
    }
}

/* Format an entry the same way it is stored on the SD card, so the live RAM
 * tail can be concatenated onto the SD history during a full export without
 * any parsing. */
void modbus_comm_log_format_line(const modbus_comm_log_entry_t *e, char *buf, size_t len)
{
    if (buf == NULL || len == 0) { if (buf) buf[0] = '\0'; return; }
    if (e == NULL) { buf[0] = '\0'; return; }

    char ts[40];
    if (e->timestamp_ms > 0) {
        time_t sec = (time_t)(e->timestamp_ms / 1000);
        int ms = (int)(e->timestamp_ms % 1000);
        struct tm tm_info;
        if (localtime_r(&sec, &tm_info) != NULL) {
            strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &tm_info);
            int n = (int)strlen(ts);
            snprintf(ts + n, sizeof(ts) - (size_t)n, ".%03d", ms);
        } else {
            snprintf(ts, sizeof(ts), "U%lld", (long long)e->uptime_ms);
        }
    } else {
        snprintf(ts, sizeof(ts), "U%lld", (long long)e->uptime_ms);
    }

    const char *dir = (e->direction == MODBUS_COMM_TX) ? "TX" : "RX";
    const char *st  = (e->status == ESP_OK) ? "OK" : esp_err_to_name(e->status);
    char hex[MODBUS_COMM_FRAME_MAX * 3 + 1];
    size_t h = 0;
    for (uint8_t i = 0; i < e->frame_length && h < sizeof(hex) - 1; ++i) {
        h += (size_t)snprintf(hex + h, sizeof(hex) - h, "%02X", e->frame[i]);
    }
    snprintf(buf, len,
             "%s [%s] slave=%u fn=0x%02X addr=%u cnt=%u len=%u status=%s frame=%s%s\n",
             ts, dir, e->slave_id, e->function_code, e->register_address,
             e->register_count, e->frame_length, st, hex,
             e->truncated ? "(trunc)" : "");
}

/*
 * Invoke @p cb for every RAM-buffer entry whose uptime is greater than
 * @p min_uptime_ms (i.e. not yet persisted to the SD card), in chronological
 * order. Used by the combined export so the live tail is appended after the
 * SD history without duplicating what is already on the card.
 */
void modbus_comm_log_iterate_tail(int64_t min_uptime_ms,
                                  void (*cb)(const char *line, void *arg),
                                  void *arg)
{
    modbus_comm_log_init();
    if (s_mutex == NULL || cb == NULL) return;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return;

    uint16_t oldest = (uint16_t)((s_next + MODBUS_COMM_LOG_CAPACITY - s_count)
                                 % MODBUS_COMM_LOG_CAPACITY);
    uint16_t n = 0;
    for (uint16_t i = 0; i < s_count; ++i) {
        if (s_entries[(oldest + i) % MODBUS_COMM_LOG_CAPACITY].uptime_ms
                > min_uptime_ms) {
            ++n;
        }
    }
    modbus_comm_log_entry_t *copy = (n > 0) ? calloc(n, sizeof(*copy)) : NULL;
    uint16_t k = 0;
    if (copy != NULL) {
        for (uint16_t i = 0; i < s_count; ++i) {
            modbus_comm_log_entry_t *e =
                &s_entries[(oldest + i) % MODBUS_COMM_LOG_CAPACITY];
            if (e->uptime_ms > min_uptime_ms) copy[k++] = *e;
        }
    }
    xSemaphoreGive(s_mutex);

    for (uint16_t i = 0; i < k; ++i) {
        char line[256];
        modbus_comm_log_format_line(&copy[i], line, sizeof(line));
        cb(line, arg);
    }
    free(copy);
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

void modbus_comm_log_anchor_wall_time(int64_t wall_now_ms,
                                      int64_t uptime_now_ms)
{
    if (wall_now_ms <= 0 || uptime_now_ms < 0 || s_mutex == NULL ||
        xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    for (uint16_t i = 0; i < s_count; ++i) {
        uint16_t oldest = (uint16_t)((s_next + MODBUS_COMM_LOG_CAPACITY -
                                      s_count) % MODBUS_COMM_LOG_CAPACITY);
        modbus_comm_log_entry_t *entry =
            &s_entries[(oldest + i) % MODBUS_COMM_LOG_CAPACITY];
        if (entry->timestamp_ms == 0 && entry->uptime_ms <= uptime_now_ms) {
            entry->timestamp_ms =
                wall_now_ms - uptime_now_ms + entry->uptime_ms;
        }
    }
    xSemaphoreGive(s_mutex);
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
