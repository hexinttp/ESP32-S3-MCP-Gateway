/**
 * @file eval_logger.h
 * @brief Evaluation and metrics logging module for gateway performance monitoring
 *
 * Provides structured event logging and atomic metric counters for tracking
 * poll success rates, context pipeline throughput, MQTT reliability, cache
 * operations, and system resource utilisation.
 */
#ifndef EVAL_LOGGER_H
#define EVAL_LOGGER_H

#include <stdint.h>

/* ======================== Event Structure ======================== */

/**
 * @brief A single evaluation event record
 */
typedef struct {
    int64_t  timestamp_ms;        /* Wall-clock time of the event (ms since boot) */
    uint32_t sequence_id;         /* Monotonic event sequence number */
    char     event_type[32];      /* Short event category tag (e.g. "poll_ok") */
    char     detail[256];         /* Human-readable event detail */
} eval_event_t;

/* ======================== Metrics Snapshot ======================== */

/**
 * @brief Aggregated performance counters
 *
 * All counters are monotonically increasing unless explicitly reset.
 */
typedef struct {
    uint32_t total_polls;         /* Total MODBUS poll attempts */
    uint32_t successful_polls;    /* Polls that returned valid data */
    uint32_t failed_polls;        /* Polls that returned errors */
    uint32_t contexts_created;    /* TCM contexts built from raw data */
    uint32_t contexts_validated;  /* Contexts that passed schema validation */
    uint32_t contexts_rejected;   /* Contexts that failed validation */
    uint32_t mqtt_published;      /* Messages successfully enqueued for MQTT */
    uint32_t mqtt_failed;         /* MQTT serialization or enqueue failures */
    uint32_t cached_records;      /* Records written to UIF flash cache */
    uint32_t replayed_records;    /* Records replayed from cache on reconnect */
    uint32_t data_loss_count;     /* Dropped records due to queue overflow */
    uint32_t commands_received;   /* Downlink MQTT commands received */
    uint32_t commands_accepted;   /* Commands that passed validation */
    uint32_t commands_rejected;   /* Commands that failed validation */
    uint32_t free_heap_min;       /* Minimum free heap observed (bytes) */
    uint32_t cpu_load_percent;    /* Most recent CPU load estimate (%) */
} eval_metrics_t;

/* ======================== API Functions ======================== */

/**
 * @brief Initialize the evaluation logger (zero counters, create mutex)
 */
void eval_init(void);

/**
 * @brief Log an evaluation event and update related metric counters
 * @param event Pointer to the event record (copied internally)
 */
void eval_log_event(const eval_event_t *event);

/**
 * @brief Thread-safe increment of a named metric counter
 * @param metric_name Name matching a field in eval_metrics_t
 * @param delta Amount to add (typically 1)
 */
void eval_increment_metric(const char *metric_name, uint32_t delta);

/**
 * @brief Get a thread-safe snapshot of all current metrics
 * @return Copy of the metrics structure
 */
eval_metrics_t eval_get_metrics(void);

/**
 * @brief Reset all metric counters to zero
 */
void eval_reset_metrics(void);

/**
 * @brief Print a formatted summary table of all metrics via ESP_LOGI
 */
void eval_print_summary(void);

#endif /* EVAL_LOGGER_H */
