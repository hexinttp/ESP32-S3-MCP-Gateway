/**
 * @file eval_logger.c
 * @brief Evaluation logger implementation - thread-safe metrics and event logging
 *
 * All metric increments are protected by a FreeRTOS mutex to guarantee
 * correctness when called from multiple tasks concurrently.
 */

#include "eval_logger.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include <string.h>
#include <stdio.h>

static const char *TAG = "EVAL";

/* ======================== Internal State ======================== */

static eval_metrics_t s_metrics;
static SemaphoreHandle_t s_metrics_mutex = NULL;
static uint32_t s_event_sequence = 0;

/* ======================== Internal Helpers ======================== */

/**
 * @brief Map an event_type string to the corresponding metric field and
 *        increment it by 1.  Keeps eval_log_event() self-contained so
 *        callers do not need a separate eval_increment_metric() call for
 *        standard events.
 */
static void auto_increment_for_event(const char *event_type)
{
    /* Comparison order chosen for most-common-first */
    if (strcmp(event_type, "poll_ok") == 0) {
        s_metrics.successful_polls++;
    } else if (strcmp(event_type, "poll_fail") == 0) {
        s_metrics.failed_polls++;
    } else if (strcmp(event_type, "poll_total") == 0) {
        s_metrics.total_polls++;
    } else if (strcmp(event_type, "ctx_created") == 0) {
        s_metrics.contexts_created++;
    } else if (strcmp(event_type, "ctx_validated") == 0) {
        s_metrics.contexts_validated++;
    } else if (strcmp(event_type, "ctx_rejected") == 0) {
        s_metrics.contexts_rejected++;
    } else if (strcmp(event_type, "mqtt_pub") == 0) {
        s_metrics.mqtt_published++;
    } else if (strcmp(event_type, "mqtt_fail") == 0) {
        s_metrics.mqtt_failed++;
    } else if (strcmp(event_type, "cache_write") == 0) {
        s_metrics.cached_records++;
    } else if (strcmp(event_type, "replay") == 0) {
        s_metrics.replayed_records++;
    } else if (strcmp(event_type, "data_loss") == 0) {
        s_metrics.data_loss_count++;
    } else if (strcmp(event_type, "cmd_recv") == 0) {
        s_metrics.commands_received++;
    } else if (strcmp(event_type, "cmd_accept") == 0) {
        s_metrics.commands_accepted++;
    } else if (strcmp(event_type, "cmd_reject") == 0) {
        s_metrics.commands_rejected++;
    }
    /* "resource_stats" and unknown types do not auto-increment */
}

/* ======================== Public API ======================== */

void eval_init(void)
{
    if (s_metrics_mutex == NULL) {
        s_metrics_mutex = xSemaphoreCreateMutex();
        if (s_metrics_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create metrics mutex");
            return;
        }
    }

    memset(&s_metrics, 0, sizeof(s_metrics));
    s_event_sequence = 0;

    ESP_LOGI(TAG, "Evaluation logger initialized");
}

void eval_log_event(const eval_event_t *event)
{
    if (event == NULL) {
        return;
    }

    /* Log the event via ESP_LOGI */
    ESP_LOGI(TAG, "[seq=%lu][%s] %s",
             (unsigned long)event->sequence_id,
             event->event_type,
             event->detail);

    /* Thread-safe metric update */
    if (xSemaphoreTake(s_metrics_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        auto_increment_for_event(event->event_type);
        xSemaphoreGive(s_metrics_mutex);
    }
}

void eval_increment_metric(const char *metric_name, uint32_t delta)
{
    if (metric_name == NULL || s_metrics_mutex == NULL) {
        return;
    }

    if (xSemaphoreTake(s_metrics_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        ESP_LOGW(TAG, "Metric mutex timeout, skipping increment for '%s'", metric_name);
        return;
    }

    /* Map metric name to struct field.
     * Using explicit comparisons rather than offsetof tricks for clarity
     * and compile-time safety on embedded targets.
     */
    if (strcmp(metric_name, "total_polls") == 0) {
        s_metrics.total_polls += delta;
    } else if (strcmp(metric_name, "successful_polls") == 0) {
        s_metrics.successful_polls += delta;
    } else if (strcmp(metric_name, "failed_polls") == 0) {
        s_metrics.failed_polls += delta;
    } else if (strcmp(metric_name, "contexts_created") == 0) {
        s_metrics.contexts_created += delta;
    } else if (strcmp(metric_name, "contexts_validated") == 0) {
        s_metrics.contexts_validated += delta;
    } else if (strcmp(metric_name, "contexts_rejected") == 0) {
        s_metrics.contexts_rejected += delta;
    } else if (strcmp(metric_name, "mqtt_published") == 0) {
        s_metrics.mqtt_published += delta;
    } else if (strcmp(metric_name, "mqtt_failed") == 0) {
        s_metrics.mqtt_failed += delta;
    } else if (strcmp(metric_name, "cached_records") == 0) {
        s_metrics.cached_records += delta;
    } else if (strcmp(metric_name, "replayed_records") == 0) {
        s_metrics.replayed_records += delta;
    } else if (strcmp(metric_name, "data_loss_count") == 0) {
        s_metrics.data_loss_count += delta;
    } else if (strcmp(metric_name, "commands_received") == 0) {
        s_metrics.commands_received += delta;
    } else if (strcmp(metric_name, "commands_accepted") == 0) {
        s_metrics.commands_accepted += delta;
    } else if (strcmp(metric_name, "commands_rejected") == 0) {
        s_metrics.commands_rejected += delta;
    } else if (strcmp(metric_name, "free_heap_min") == 0) {
        /* For min-heap, store the value directly (not additive) */
        s_metrics.free_heap_min = delta;
    } else if (strcmp(metric_name, "cpu_load_percent") == 0) {
        /* For CPU load, store the latest reading (not additive) */
        s_metrics.cpu_load_percent = delta;
    } else {
        ESP_LOGW(TAG, "Unknown metric name: '%s'", metric_name);
    }

    xSemaphoreGive(s_metrics_mutex);
}

eval_metrics_t eval_get_metrics(void)
{
    eval_metrics_t snapshot;
    memset(&snapshot, 0, sizeof(snapshot));

    if (s_metrics_mutex != NULL &&
        xSemaphoreTake(s_metrics_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(&snapshot, &s_metrics, sizeof(snapshot));
        xSemaphoreGive(s_metrics_mutex);
    }

    return snapshot;
}

void eval_reset_metrics(void)
{
    if (s_metrics_mutex == NULL) {
        return;
    }

    if (xSemaphoreTake(s_metrics_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memset(&s_metrics, 0, sizeof(s_metrics));
        s_event_sequence = 0;
        xSemaphoreGive(s_metrics_mutex);
        ESP_LOGI(TAG, "All metrics reset to zero");
    }
}

void eval_print_summary(void)
{
    eval_metrics_t m = eval_get_metrics();

    ESP_LOGI(TAG, "+================================================================+");
    ESP_LOGI(TAG, "|                  GATEWAY EVALUATION SUMMARY                    |");
    ESP_LOGI(TAG, "+================================================================+");
    ESP_LOGI(TAG, "|  MODBUS Polling                                                |");
    ESP_LOGI(TAG, "|    Total polls       : %10lu                            |",
             (unsigned long)m.total_polls);
    ESP_LOGI(TAG, "|    Successful        : %10lu                            |",
             (unsigned long)m.successful_polls);
    ESP_LOGI(TAG, "|    Failed            : %10lu                            |",
             (unsigned long)m.failed_polls);

    uint32_t poll_rate = 0;
    if (m.total_polls > 0) {
        poll_rate = (m.successful_polls * 100) / m.total_polls;
    }
    ESP_LOGI(TAG, "|    Success rate      : %9lu%%                             |",
             (unsigned long)poll_rate);

    ESP_LOGI(TAG, "|----------------------------------------------------------------|");
    ESP_LOGI(TAG, "|  TCM Context Pipeline                                          |");
    ESP_LOGI(TAG, "|    Contexts created  : %10lu                            |",
             (unsigned long)m.contexts_created);
    ESP_LOGI(TAG, "|    Contexts validated: %10lu                            |",
             (unsigned long)m.contexts_validated);
    ESP_LOGI(TAG, "|    Contexts rejected : %10lu                            |",
             (unsigned long)m.contexts_rejected);

    ESP_LOGI(TAG, "|----------------------------------------------------------------|");
    ESP_LOGI(TAG, "|  MQTT Communication                                            |");
    ESP_LOGI(TAG, "|    Published         : %10lu                            |",
             (unsigned long)m.mqtt_published);
    ESP_LOGI(TAG, "|    Failed            : %10lu                            |",
             (unsigned long)m.mqtt_failed);
    ESP_LOGI(TAG, "|    Cached (offline)  : %10lu                            |",
             (unsigned long)m.cached_records);
    ESP_LOGI(TAG, "|    Replayed          : %10lu                            |",
             (unsigned long)m.replayed_records);
    ESP_LOGI(TAG, "|    Data loss events  : %10lu                            |",
             (unsigned long)m.data_loss_count);

    ESP_LOGI(TAG, "|----------------------------------------------------------------|");
    ESP_LOGI(TAG, "|  Downlink Commands                                              |");
    ESP_LOGI(TAG, "|    Received          : %10lu                            |",
             (unsigned long)m.commands_received);
    ESP_LOGI(TAG, "|    Accepted          : %10lu                            |",
             (unsigned long)m.commands_accepted);
    ESP_LOGI(TAG, "|    Rejected          : %10lu                            |",
             (unsigned long)m.commands_rejected);

    ESP_LOGI(TAG, "|----------------------------------------------------------------|");
    ESP_LOGI(TAG, "|  System Resources                                               |");
    ESP_LOGI(TAG, "|    Min free heap     : %10lu bytes                      |",
             (unsigned long)m.free_heap_min);
    ESP_LOGI(TAG, "|    CPU load (est.)   : %9lu%%                              |",
             (unsigned long)m.cpu_load_percent);

    ESP_LOGI(TAG, "+================================================================+");
}
