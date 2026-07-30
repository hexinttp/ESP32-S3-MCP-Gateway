/**
 * @file uif_persistence.h
 * @brief UIF (Uninterrupted Information Flow) persistence layer.
 *
 * Provides a durable SPI Flash queue with TF-card overflow. Pending records are
 * replayed by sequence_id and removed only after MQTT acknowledgement.
 */
#ifndef UIF_PERSISTENCE_H
#define UIF_PERSISTENCE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "gateway_config.h"
#include "tcm/tcm_context.h"

/* ======================== Type Definitions ======================== */

/**
 * @brief A single cached record in the UIF circular buffer.
 */
typedef struct {
    char     json_data[TCM_MAX_JSON_LEN];   /**< Serialized JSON payload */
    uint32_t sequence_id;                    /**< Monotonic sequence from TCM */
    int64_t  timestamp_ms;                   /**< Time of capture (ms since boot) */
    bool     pending;                        /**< true = awaiting replay */
} uif_cache_entry_t;

/* ======================== API Functions ======================== */

/**
 * @brief Initialize the UIF persistence module.
 *
 * Creates the internal mutex and resets the circular buffer.
 * Must be called once before any other uif_* function.
 *
 * @return ESP_OK on success, ESP_FAIL on error (e.g. mutex creation failure)
 */
esp_err_t uif_init(void);

/**
 * @brief Record a TCM context into the UIF cache.
 *
 * Serializes the context to JSON via tcm_serialize_json(), stores it in the
 * next available slot with its sequence_id and timestamp.  If the buffer is
 * full, the oldest replayed entry is overwritten; if none are replayed, the
 * data_loss_count counter is incremented.
 *
 * @param ctx  Pointer to a fully-built TCM context object
 * @return ESP_OK on success, ESP_FAIL if the buffer is full and no slot
 *         could be reclaimed, or ESP_ERR_INVALID_ARG on NULL input
 */
esp_err_t uif_cache_record(const tcm_context_t *ctx);

/**
 * @brief Get the number of records currently stored in the cache
 *        (both pending and replayed).
 */
int uif_get_cached_count(void);

/**
 * @brief Retrieve a cached record by its buffer index.
 *
 * @param index  Zero-based index into the circular buffer
 *               (0 = oldest entry still in the buffer)
 * @param out    Output: copy of the cache entry
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if index is out of range
 */
esp_err_t uif_get_cached_record(int index, uif_cache_entry_t *out);

/**
 * @brief Mark a cached record as replayed (pending = false).
 *
 * @param index  Zero-based buffer index of the record to mark
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if out of range
 */
esp_err_t uif_mark_replayed(int index);

/**
 * @brief Remove all non-pending (replayed) entries and compact the buffer.
 *
 * After compaction, the remaining pending entries occupy contiguous slots
 * starting from index 0.
 *
 * @return ESP_OK on success
 */
esp_err_t uif_clear_replayed(void);

/**
 * @brief Replay all pending cached records to the MQTT output queue.
 *
 * Pending entries are sorted by sequence_id before being enqueued.
 * Each successfully enqueued record is then marked as replayed.
 *
 * @param mqtt_out_queue  FreeRTOS queue handle accepting mqtt_out_msg_t items
 * @return ESP_OK on success, ESP_FAIL if the queue is unavailable
 */
esp_err_t uif_replay_all(QueueHandle_t mqtt_out_queue);

/** Release an in-flight replay token after connection loss. The durable
 * record remains in Flash/TF and will be retried after reconnect. */
void uif_replay_connection_lost(void);

/** Complete the single in-flight ThingsCloud replay after the platform's
 * application-level attributes response. A rejected record remains durable. */
void uif_replay_cloud_response(bool gateway_topic, bool accepted, int error_code);

/** True while a ThingsCloud replay is waiting for an application response. */
bool uif_replay_is_waiting_response(void);

/**
 * @brief Get cache utilization as a percentage (0-100).
 */
int uif_get_cache_usage_percent(void);

/**
 * @brief Get the cumulative count of records lost due to a full buffer.
 */
int uif_get_data_loss_count(void);

/**
 * @brief Destroy the UIF persistence module, releasing all resources.
 *
 * The mutex is deleted and the buffer is invalidated.  Any data still
 * in the buffer that has not been replayed will be lost.
 */
void uif_destroy(void);

#endif /* UIF_PERSISTENCE_H */
