#include "storage/log_store.h"

#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "board/tf_storage.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define LOG_DIRECTORY       TF_MOUNT_POINT "/logs"
#define LOG_MAX_FILE_BYTES_SYSTEM  (1ULL * 1024ULL * 1024ULL)   /* 1 MiB */
#define LOG_MAX_FILE_BYTES_MODBUS  (4ULL * 1024ULL * 1024ULL)   /* 4 MiB */
#define LOG_MIN_FREE_BYTES  (2ULL * 1024ULL * 1024ULL)          /* keep >= 2 MiB */
#define LOG_PENDING_CAP     4096                                /* per-stream buffer */
#define LOG_FLUSH_THRESHOLD 2048                                /* wake writer at this size */
#define LOG_FLUSH_PERIOD_MS 1000
#define LOG_LINE_MAX        320

/*
 * IMPORTANT (crash-safety):
 * FATFS + SD-card access needs several KiB of stack. Doing it in the context of
 * the caller (automation / modbus / mqtt / httpd tasks) or in the FreeRTOS timer
 * service task (2 KiB stack) overflows those stacks and panics the device.
 * Therefore every file operation happens exclusively in the dedicated writer
 * task below; the append path only copies bytes into a RAM buffer.
 */
#define LOG_WRITER_STACK_PRIMARY   5120
#define LOG_WRITER_STACK_FALLBACK  4096
#define LOG_WRITER_PRIO            2

static const char *TAG = "LOGSTORE";

typedef struct {
    const char *path;
    uint64_t    max_bytes;
    FILE       *file;         /* writer task only */
    uint64_t    file_size;    /* writer task only */
    char       *pending;      /* producer buffer (mutex protected) */
    size_t      pending_len;
    char       *flushing;     /* buffer handed over to the writer task */
    size_t      flushing_len;
    uint32_t    dropped;
    bool        active;
    /* Uptime tracking for export de-duplication (see log_store_last_flushed_uptime):
     * pending_uptime = uptime of the newest line currently queued in 'pending',
     * flushing_uptime = uptime of the newest line handed to the writer task,
     * last_flushed_uptime = uptime of the newest line actually written to SD. */
    int64_t     pending_uptime;
    int64_t     flushing_uptime;
    int64_t     last_flushed_uptime;
} stream_t;

static stream_t s_streams[LOG_STORE_STREAM_COUNT] = {
    [LOG_STORE_STREAM_SYSTEM] = {
        .path = LOG_DIRECTORY "/system.log",
        .max_bytes = LOG_MAX_FILE_BYTES_SYSTEM,
    },
    [LOG_STORE_STREAM_MODBUS] = {
        .path = LOG_DIRECTORY "/modbus_comm.log",
        .max_bytes = LOG_MAX_FILE_BYTES_MODBUS,
    },
};

static SemaphoreHandle_t s_mutex  = NULL;
static TaskHandle_t      s_writer = NULL;
static bool              s_writer_failed = false;

/* ---------- helpers (writer task context only) ---------- */

static bool is_log_file(const char *name)
{
    if (name == NULL) return false;
    bool prefix = strncmp(name, "system_", 7) == 0 ||
                  strncmp(name, "modbus_comm_", 13) == 0;
    return prefix && strstr(name, ".log") != NULL;
}

static void ensure_dir(void)
{
    DIR *d = opendir(LOG_DIRECTORY);
    if (d == NULL) mkdir(LOG_DIRECTORY, 0755);
    else closedir(d);
}

static uint64_t file_size_of(const char *path)
{
    struct stat st;
    if (stat(path, &st) != 0) return 0;
    return st.st_size > 0 ? (uint64_t)st.st_size : 0;
}

static void make_timestamp(char *buf, size_t len)
{
    time_t now = time(NULL);
    struct tm tm_info;
    if (now == (time_t)-1 || localtime_r(&now, &tm_info) == NULL) {
        long long up = esp_timer_get_time() / 1000000LL;
        snprintf(buf, len, "uptime-%lld", up);
        return;
    }
    strftime(buf, len, "%Y-%m-%d %H:%M:%S", &tm_info);
}

static esp_err_t delete_oldest(void)
{
    DIR *dir = opendir(LOG_DIRECTORY);
    if (dir == NULL) return ESP_ERR_NOT_FOUND;
    char oldest[64] = {0};
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        if (is_log_file(entry->d_name) &&
            (oldest[0] == '\0' || strcmp(entry->d_name, oldest) < 0)) {
            strlcpy(oldest, entry->d_name, sizeof(oldest));
        }
    }
    closedir(dir);
    if (oldest[0] == '\0') return ESP_ERR_NOT_FOUND;
    char path[128];
    snprintf(path, sizeof(path), "%s/%s", LOG_DIRECTORY, oldest);
    if (unlink(path) != 0) return ESP_FAIL;
    ESP_LOGW(TAG, "TF nearly full: removed oldest log file %s", oldest);
    return ESP_OK;
}

static void ensure_free_space(void)
{
    uint64_t total = 0, free_bytes = 0;
    for (int i = 0; i < 16; ++i) {
        if (tf_storage_get_space(&total, &free_bytes) != ESP_OK ||
            free_bytes >= LOG_MIN_FREE_BYTES) {
            return;
        }
        if (delete_oldest() != ESP_OK) return;
    }
}

static void rotate_stream(stream_t *s)
{
    if (s->file != NULL) {
        fclose(s->file);
        s->file = NULL;
    }
    char ts[32];
    make_timestamp(ts, sizeof(ts));
    char rotated[128];
    const char *base = (strstr(s->path, "modbus") != NULL) ? "modbus_comm_" : "system_";
    for (int i = 1; i < 1000; ++i) {
        if (i == 1) snprintf(rotated, sizeof(rotated), "%s/%s%s.log", LOG_DIRECTORY, base, ts);
        else snprintf(rotated, sizeof(rotated), "%s/%s%s_%d.log", LOG_DIRECTORY, base, ts, i);
        if (access(rotated, F_OK) != 0) break;
    }
    rename(s->path, rotated);
    s->file = fopen(s->path, "a");
    s->file_size = (s->file != NULL) ? file_size_of(s->path) : 0;
    s->active = (s->file != NULL);
}

/* Writes s->flushing to disk. Runs in the writer task only. */
static void write_pending(stream_t *s)
{
    if (s->flushing == NULL || s->flushing_len == 0) return;

    if (!tf_storage_is_mounted()) {
        if (s->file != NULL) { fclose(s->file); s->file = NULL; }
        s->active = false;
        s->flushing_len = 0;
        return;
    }

    if (s->file == NULL) {
        ensure_dir();
        s->file = fopen(s->path, "a");
        if (s->file == NULL) {
            s->active = false;
            s->flushing_len = 0;
            return;
        }
        s->file_size = file_size_of(s->path);
        s->active = true;
    }

    ensure_free_space();
    if (s->file_size + s->flushing_len > s->max_bytes) {
        rotate_stream(s);
        if (s->file == NULL) { s->flushing_len = 0; return; }
    }

    size_t written = fwrite(s->flushing, 1, s->flushing_len, s->file);
    if (written == s->flushing_len) {
        fflush(s->file);
        s->file_size += written;
        s->last_flushed_uptime = s->flushing_uptime;
    } else {
        fclose(s->file);
        s->file = NULL;
        s->active = false;
        ESP_LOGW(TAG, "log write failed; will retry on next flush");
    }
    s->flushing_len = 0;

    /* Release the active file handle after each flush so other tasks (e.g. the
     * HTTP log-export handler) can open and read it concurrently. The file is
     * reopened in append mode on the next flush, so existing content is kept.
     * Keeping it permanently open blocks readers (FATFS) and made the
     * system-log export - which has only a tiny RAM tail - download empty. */
    if (s->file != NULL) {
        fclose(s->file);
        s->file = NULL;
    }
}

static void flush_all(void)
{
    if (s_mutex == NULL) return;

    /* Swap producer/consumer buffers under the mutex, write outside of it so
     * that producers are never blocked by SD-card latency. */
    for (int i = 0; i < LOG_STORE_STREAM_COUNT; ++i) {
        stream_t *s = &s_streams[i];
        uint32_t dropped = 0;

        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(200)) != pdTRUE) return;
        if (s->pending_len > 0 && s->flushing_len == 0) {
            char *tmp   = s->flushing;
            s->flushing = s->pending;
            s->flushing_len = s->pending_len;
            s->flushing_uptime = s->pending_uptime;
            s->pending  = tmp;          /* may be NULL on first swap */
            s->pending_len = 0;
            s->pending_uptime = 0;
        }
        dropped = s->dropped;
        s->dropped = 0;
        xSemaphoreGive(s_mutex);

        if (dropped > 0) {
            ESP_LOGW(TAG, "%s: dropped %u log line(s), SD too slow",
                     (i == LOG_STORE_STREAM_MODBUS) ? "modbus" : "system",
                     (unsigned)dropped);
        }

        write_pending(s);

        /* Give the (now free) buffer back so the producer can reuse it. */
        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            if (s->pending == NULL && s->flushing != NULL) {
                s->pending = s->flushing;
                s->pending_len = 0;
                s->flushing = NULL;
            }
            xSemaphoreGive(s_mutex);
        }
    }
}

static void log_writer_task(void *arg)
{
    (void)arg;
    while (true) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(LOG_FLUSH_PERIOD_MS));
        flush_all();
    }
}

/* Must be called with the mutex held. */
static bool ensure_writer_task(void)
{
    if (s_writer != NULL) return true;
    if (s_writer_failed) return false;

    BaseType_t ok = xTaskCreate(log_writer_task, "logwriter",
                                LOG_WRITER_STACK_PRIMARY, NULL,
                                LOG_WRITER_PRIO, &s_writer);
    if (ok != pdPASS) {
        ok = xTaskCreate(log_writer_task, "logwriter",
                         LOG_WRITER_STACK_FALLBACK, NULL,
                         LOG_WRITER_PRIO, &s_writer);
    }
    if (ok != pdPASS) {
        s_writer = NULL;
        s_writer_failed = true;
        ESP_LOGE(TAG, "cannot create log writer task; SD logging disabled");
        return false;
    }
    return true;
}

static char *alloc_buffer(void)
{
    char *p = heap_caps_malloc(LOG_PENDING_CAP, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (p == NULL) p = malloc(LOG_PENDING_CAP);
    return p;
}

/* ---------- public API (producer context, no file I/O) ---------- */

static void append_to_stream(log_store_stream_t stream, const char *line,
                              size_t len, int64_t uptime_ms)
{
    if (len == 0 || !tf_storage_is_mounted() || s_writer_failed) return;

    if (s_mutex == NULL) {
        SemaphoreHandle_t m = xSemaphoreCreateMutex();
        if (m == NULL) return;
        /* First caller wins; a duplicate is impossible in practice because the
         * very first append happens long before concurrency ramps up, but stay
         * safe and free the extra one. */
        if (s_mutex == NULL) s_mutex = m;
        else vSemaphoreDelete(m);
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;

    stream_t *s = &s_streams[stream];
    if (s->pending == NULL) {
        s->pending = alloc_buffer();
        if (s->pending == NULL) { xSemaphoreGive(s_mutex); return; }
        s->pending_len = 0;
    }
    if (!ensure_writer_task()) { xSemaphoreGive(s_mutex); return; }

    bool wake = false;
    if (s->pending_len + len > LOG_PENDING_CAP) {
        s->dropped++;               /* writer is behind - drop instead of block */
        wake = true;
    } else {
        memcpy(s->pending + s->pending_len, line, len);
        s->pending_len += len;
        s->pending_uptime = uptime_ms;
        wake = (s->pending_len >= LOG_FLUSH_THRESHOLD);
    }
    TaskHandle_t writer = s_writer;
    xSemaphoreGive(s_mutex);

    if (wake && writer != NULL) xTaskNotifyGive(writer);
}

void log_store_append(const char *level, const char *text, int64_t uptime_ms)
{
    if (!tf_storage_is_mounted() || s_writer_failed) return;
    char ts[32];
    make_timestamp(ts, sizeof(ts));
    char line[LOG_LINE_MAX];
    int n = snprintf(line, sizeof(line), "%s [%s] %s\n",
                     ts, (level && *level) ? level : "info",
                     (text && *text) ? text : "");
    if (n <= 0) return;
    if (n >= (int)sizeof(line)) n = (int)sizeof(line) - 1;
    append_to_stream(LOG_STORE_STREAM_SYSTEM, line, (size_t)n, uptime_ms);
}

void log_store_append_modbus(const char *line, int64_t uptime_ms)
{
    if (line == NULL) return;
    if (!tf_storage_is_mounted() || s_writer_failed) return;

    size_t len = strnlen(line, LOG_LINE_MAX - 1);
    if (len == 0) return;

    if (line[len - 1] == '\n') {
        append_to_stream(LOG_STORE_STREAM_MODBUS, line, len, uptime_ms);
        return;
    }
    /* Small stack copy just to add the trailing newline. */
    char buf[LOG_LINE_MAX];
    memcpy(buf, line, len);
    buf[len++] = '\n';
    append_to_stream(LOG_STORE_STREAM_MODBUS, buf, len, uptime_ms);
}

const char *log_store_active_path(void)
{
    return s_streams[LOG_STORE_STREAM_SYSTEM].path;
}

const char *log_store_path(log_store_stream_t stream)
{
    if (stream < 0 || stream >= LOG_STORE_STREAM_COUNT) return NULL;
    return s_streams[stream].path;
}

bool log_store_is_active(void)
{
    return s_streams[LOG_STORE_STREAM_SYSTEM].active;
}

int64_t log_store_last_flushed_uptime(log_store_stream_t stream)
{
    if (stream < 0 || stream >= LOG_STORE_STREAM_COUNT) return 0;
    return s_streams[stream].last_flushed_uptime;
}

/* Compare two rotated log file names (e.g. "system_2026-07-30 12:00:00.log").
 * ISO-ish timestamps sort lexically in chronological order. */
static int cmp_name(const void *a, const void *b)
{
    return strcmp(*(const char *const *)a, *(const char *const *)b);
}

void log_store_list_files(log_store_stream_t stream,
                          void (*cb)(const char *path, void *arg), void *arg)
{
    if (stream < 0 || stream >= LOG_STORE_STREAM_COUNT || cb == NULL) return;
    if (!tf_storage_is_mounted()) return;

    DIR *dir = opendir(LOG_DIRECTORY);
    if (dir == NULL) return;

    const stream_t *s = &s_streams[stream];
    const char *base = (stream == LOG_STORE_STREAM_MODBUS) ? "modbus_comm_"
                                                            : "system_";

    /* Collect rotated history files ("<base><ts>.log"); the active file is
     * reported separately and always last. */
    char **names = NULL;
    size_t count = 0, cap = 0;
    struct dirent *ent;
    while ((ent = readdir(dir)) != NULL) {
        const char *name = ent->d_name;
        size_t len = strlen(name);
        if (len < strlen(base) + 4) continue;                 /* base + ".log" */
        if (strncmp(name, base, strlen(base)) != 0) continue;  /* system_/modbus_comm_ prefix */
        if (strncmp(name + len - 4, ".log", 4) != 0) continue; /* suffix .log */
        /* The active file (base without trailing '_') is "<base[:-1]>.log". */
        if (strncmp(name, base, strlen(base) - 1) == 0 &&
            strncmp(name + strlen(base) - 1, ".log", 4) == 0) {
            continue;  /* this is the active file, handled below */
        }
        if (count + 1 > cap) {
            cap = (cap == 0) ? 8 : cap * 2;
            char **tmp = realloc(names, cap * sizeof(*tmp));
            if (tmp == NULL) break;
            names = tmp;
        }
        names[count] = strdup(name);
        if (names[count] == NULL) break;
        ++count;
    }
    closedir(dir);

    qsort(names, count, sizeof(*names), cmp_name);
    for (size_t i = 0; i < count; ++i) {
        char path[160];
        snprintf(path, sizeof(path), "%s/%s", LOG_DIRECTORY, names[i]);
        cb(path, arg);
        free(names[i]);
    }
    free(names);

    /* Active file last (newest data). */
    if (s->path != NULL) cb(s->path, arg);
}
