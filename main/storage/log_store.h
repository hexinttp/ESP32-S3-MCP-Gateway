#ifndef LOG_STORE_H
#define LOG_STORE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Log streams persisted to the SD card (TF).
 *
 *  - SYSTEM: the Web "system log" (high-level gateway events).
 *  - MODBUS: the RS485/Modbus communication log (one line per frame).
 *
 * Both streams are buffered and flushed at most once per second (and whenever
 * the pending buffer reaches LOG_FLUSH_THRESHOLD) so that high-frequency
 * Modbus traffic does not hammer the SD card. Rotation and a free-space guard
 * prevent the card from filling up.
 */
typedef enum {
    LOG_STORE_STREAM_SYSTEM = 0,
    LOG_STORE_STREAM_MODBUS,
    LOG_STORE_STREAM_COUNT
} log_store_stream_t;

/** Append a system-log entry (mirrors web_server_add_log).
 *  @param uptime_ms uptime (ms) of the source entry; used by export to skip
 *         RAM entries that are already persisted to the SD card. */
void log_store_append(const char *level, const char *text, int64_t uptime_ms);

/** Append a pre-formatted RS485/Modbus communication-log line.
 *  @param uptime_ms uptime (ms) of the source entry (see log_store_append). */
void log_store_append_modbus(const char *line, int64_t uptime_ms);

/** Path of the system stream's active file (for export/serving). */
const char *log_store_active_path(void);

/** Path of a given stream's active file. */
const char *log_store_path(log_store_stream_t stream);

/** True if the system stream currently has an open, writable file. */
bool log_store_is_active(void);

/**
 * @brief Uptime (ms) of the most recent line flushed to the SD card for a
 *        stream. Entries in the RAM buffer with a larger uptime are not yet
 *        on the card and must be appended when exporting "all" data.
 */
int64_t log_store_last_flushed_uptime(log_store_stream_t stream);

/**
 * @brief Enumerate every file (rotated history + active) for a stream in
 *        chronological order, invoking @p cb with the full path of each.
 *        The active file is always reported last. Used by the combined export.
 */
void log_store_list_files(log_store_stream_t stream,
                          void (*cb)(const char *path, void *arg),
                          void *arg);

#ifdef __cplusplus
}
#endif

#endif
