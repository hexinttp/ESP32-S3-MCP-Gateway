/**
 * @file scheduler.c
 * @brief Scheduler implementation - FreeRTOS task management for the MODBUS-MQTT gateway
 *
 * Tasks managed:
 *   1. modbus_poll_task    - Periodic MODBUS register reads
 *   2. context_build_task  - Build and validate TCM contexts from raw data
 *   3. publish_task        - Serialize and enqueue MQTT publishes or cache offline
 *   4. replay_task         - Replay cached records when network comes online
 *   5. resource_monitor_task - Log heap, CPU, and flash statistics
 */

#include "scheduler.h"
#include "eval/eval_logger.h"
#include "mqtt_comm/mqtt_handler.h"
#include "amm/amm_mapping.h"
#include "uif/uif_persistence.h"
#include "tcm/tcm_state_pool.h"
#include "storage/history_store.h"
#include "config/runtime_config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_flash.h"

#include <string.h>
#include <stdlib.h>

static const char *TAG = "SCHED";

/* ======================== Internal State ======================== */

static QueueHandle_t s_raw_data_queue   = NULL;
static QueueHandle_t s_context_queue    = NULL;
static QueueHandle_t s_mqtt_out_queue  = NULL;
static QueueHandle_t s_mqtt_cmd_queue  = NULL;
static QueueHandle_t s_eval_queue       = NULL;

static TaskHandle_t s_task_modbus_poll    = NULL;
static TaskHandle_t s_task_context_build  = NULL;
static TaskHandle_t s_task_publish        = NULL;
static TaskHandle_t s_task_replay         = NULL;
static TaskHandle_t s_task_resource_mon   = NULL;

static volatile network_state_t s_network_state = NET_OFFLINE;
static SemaphoreHandle_t s_network_state_mutex  = NULL;

static volatile bool s_running = false;

/* ======================== Helper: Thread-safe network state ======================== */

static network_state_t get_network_state_locked(void)
{
    network_state_t state;
    if (s_network_state_mutex != NULL &&
        xSemaphoreTake(s_network_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        state = s_network_state;
        xSemaphoreGive(s_network_state_mutex);
    } else {
        state = NET_OFFLINE;
    }
    return state;
}

/* ======================== Task 1: modbus_poll_task ======================== */

/**
 * @brief Periodically poll configured MODBUS registers and push results to raw_data_queue.
 *
 * Each poll cycle iterates through s_poll_table[], performs a MODBUS read via
 * modbus_read_holding_register() or modbus_read_input_register(), converts
 * raw registers to float, and enqueues a modbus_read_result_t for each entry.
 */
static void modbus_poll_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "modbus_poll_task started");

    int64_t last_poll_ms[AMM_MAX_MAPPING_ENTRIES] = {0};
    uint32_t observed_model_version = 0;
    amm_mapping_entry_t *mappings = calloc(AMM_MAX_MAPPING_ENTRIES, sizeof(*mappings));
    if (mappings == NULL) {
        ESP_LOGE(TAG, "Unable to allocate AMM poll snapshot");
        s_task_modbus_poll = NULL;
        vTaskDelete(NULL);
        return;
    }

    while (s_running) {
        int mapping_count = amm_get_entries(mappings, AMM_MAX_MAPPING_ENTRIES);
        uint32_t model_version = amm_get_model_version();
        if (model_version != observed_model_version) {
            memset(last_poll_ms, 0, sizeof(last_poll_ms));
            observed_model_version = model_version;
            ESP_LOGI(TAG, "AMM poll plan updated: version=%lu entries=%d",
                     (unsigned long)model_version, mapping_count);
        }
        if (mapping_count == 0) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        int64_t now_ms = esp_timer_get_time() / 1000;
        int selected = -1;
        for (int i = 0; i < mapping_count; ++i) {
            uint32_t interval = mappings[i].poll_interval_ms ?: POLL_INTERVAL_MS;
            if (now_ms - last_poll_ms[i] >= interval &&
                (selected < 0 || mappings[i].priority > mappings[selected].priority)) {
                selected = i;
            }
        }
        if (selected < 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        amm_mapping_entry_t *cfg = &mappings[selected];
        last_poll_ms[selected] = now_ms;

        modbus_read_result_t result;
        memset(&result, 0, sizeof(result));
        result.source_protocol  = cfg->source_protocol;
        result.channel_id       = cfg->channel_id;
        result.slave_id         = cfg->slave_id;
        result.function_code    = cfg->function_code;
        result.register_address = cfg->register_address;
        result.register_count   = modbus_register_count_for_type(cfg->data_type);
        result.data_type        = cfg->data_type;
        result.raw_value        = 0.0f;
        result.quality          = QUALITY_INVALID;
        result.valid            = false;

        /* Read raw registers from the MODBUS device */
        uint16_t raw_regs[4] = {0};  /* Max 4 registers for FLOAT32/INT32/UINT32 */
        uint16_t offset = modbus_register_offset(cfg->function_code, cfg->register_address);
        esp_err_t err = modbus_read_channel(cfg->source_protocol, cfg->channel_id,
                                             cfg->slave_id, cfg->function_code,
                                             offset, result.register_count, raw_regs);

        if (err == ESP_OK) {
            result.raw_value = modbus_convert_to_float_order(raw_regs, cfg->data_type,
                                                              cfg->byte_order);
            result.quality   = QUALITY_GOOD;
            result.valid     = true;
            eval_increment_metric("successful_polls", 1);
        } else {
            result.quality = QUALITY_INVALID;
            result.valid   = false;
            ESP_LOGW(TAG, "MODBUS read failed: slave=%u fc=0x%02X addr=0x%04X err=0x%x",
                     cfg->slave_id, cfg->function_code, cfg->register_address, err);
            eval_increment_metric("failed_polls", 1);
        }

        eval_increment_metric("total_polls", 1);

        /* Enqueue result; drop oldest on overflow to prevent blocking */
        if (xQueueSend(s_raw_data_queue, &result, pdMS_TO_TICKS(50)) != pdTRUE) {
            ESP_LOGW(TAG, "raw_data_queue full, dropping poll result");
            eval_increment_metric("data_loss_count", 1);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    free(mappings);
    s_task_modbus_poll = NULL;
    vTaskDelete(NULL);
}

/* ======================== Task 2: context_build_task ======================== */

/**
 * @brief Read raw MODBUS results, build TCM contexts, enrich with AMM metadata,
 *        validate, and push valid contexts to context_queue.
 */
static void context_build_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "context_build_task started");

    modbus_read_result_t raw;

    while (s_running) {
        if (xQueueReceive(s_raw_data_queue, &raw, pdMS_TO_TICKS(200)) != pdTRUE) {
            continue;
        }

        /* Skip invalid reads */
        if (!raw.valid) {
            eval_increment_metric("contexts_rejected", 1);
            continue;
        }

        /* Step 1: Build TCM context from raw MODBUS data */
        tcm_context_t ctx;
        network_state_t net_state = get_network_state_locked();

        int ret = tcm_build_context(&ctx,
                                    raw.slave_id,
                                    raw.function_code,
                                    raw.register_address,
                                    raw.raw_value,
                                    raw.quality,
                                    net_state);
        if (ret != 0) {
            ESP_LOGW(TAG, "tcm_build_context failed for slave=%u addr=0x%04X",
                     raw.slave_id, raw.register_address);
            eval_increment_metric("contexts_rejected", 1);
            continue;
        }

        eval_increment_metric("contexts_created", 1);
        ctx.source_protocol = raw.source_protocol;
        ctx.channel_id = raw.channel_id;

        /* Step 2: Enrich context with AMM mapping metadata
         * (device name, point name, unit, measurement name, constraints)
         */
        esp_err_t amm_err = amm_enrich_context(&ctx);
        if (amm_err != ESP_OK) {
            ESP_LOGW(TAG, "amm_enrich_context failed for slave=%u addr=0x%04X (err=0x%x)",
                     raw.slave_id, raw.register_address, amm_err);
            /* Continue anyway - context still usable with raw addressing info */
        }

        /* Step 3: Validate the context against the TCM schema */
        tcm_validation_result_t val_result;
        bool valid = tcm_validate(&ctx, &val_result);
        if (!valid) {
            ESP_LOGW(TAG, "Context validation failed: %s (field_mask=0x%08lX)",
                     val_result.fail_reason,
                     (unsigned long)val_result.failed_field_mask);
            eval_increment_metric("contexts_rejected", 1);
            continue;
        }

        ctx.validated = true;
        eval_increment_metric("contexts_validated", 1);
        tcm_state_pool_update(&ctx);
        (void)history_store_append(&ctx);

        /* Push validated context downstream */
        if (xQueueSend(s_context_queue, &ctx, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGW(TAG, "context_queue full, dropping validated context");
            eval_increment_metric("data_loss_count", 1);
        }
    }

    s_task_context_build = NULL;
    vTaskDelete(NULL);
}

/* ======================== Task 3: publish_task ======================== */

/**
 * @brief Read validated contexts from context_queue.
 *        - If network is ONLINE: build mqtt_out_msg_t and push to mqtt_out_queue.
 *        - If network is OFFLINE/DELAYED: persist via uif_cache_record().
 */
static void publish_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "publish_task started");

    tcm_context_t ctx;

    while (s_running) {
        if (xQueueReceive(s_context_queue, &ctx, pdMS_TO_TICKS(200)) != pdTRUE) {
            continue;
        }

        network_state_t net_state = get_network_state_locked();

        if (net_state == NET_ONLINE) {
            /* Build MQTT output message with topic and serialized payload */
            mqtt_out_msg_t out_msg;
            memset(&out_msg, 0, sizeof(out_msg));

            /* Determine topic from AMM mapping, or use default data topic */
            if (amm_copy_mqtt_topic(ctx.source_protocol, ctx.channel_id, ctx.slave_id,
                                    ctx.register_address, out_msg.topic,
                                    sizeof(out_msg.topic)) == ESP_OK) {
            } else {
                runtime_config_t runtime;
                runtime_config_get(&runtime);
                strlcpy(out_msg.topic, runtime.mqtt.data_prefix, sizeof(out_msg.topic));
                strlcat(out_msg.topic, ctx.device_id, sizeof(out_msg.topic));
                strlcat(out_msg.topic, "/", sizeof(out_msg.topic));
                strlcat(out_msg.topic, ctx.point_id, sizeof(out_msg.topic));
            }

            int json_len = tcm_serialize_json(&ctx, out_msg.payload,
                                              sizeof(out_msg.payload));
            if (json_len <= 0) {
                ESP_LOGE(TAG, "JSON serialization failed for context_id=%lu",
                         (unsigned long)ctx.context_id);
                eval_increment_metric("mqtt_failed", 1);
                continue;
            }

            out_msg.qos = 1;
            out_msg.sequence_id = ctx.sequence_id;

            if (xQueueSend(s_mqtt_out_queue, &out_msg, pdMS_TO_TICKS(100)) != pdTRUE) {
                ESP_LOGW(TAG, "mqtt_out_queue full, caching record");
                /* Fall back to cache when queue is full */
                uif_cache_record(&ctx);
                eval_increment_metric("cached_records", 1);
            } else {
                eval_increment_metric("mqtt_published", 1);
            }
        } else {
            /* Network unavailable - persist to cache */
            esp_err_t cache_err = uif_cache_record(&ctx);
            if (cache_err == ESP_OK) {
                eval_increment_metric("cached_records", 1);
                ESP_LOGD(TAG, "Cached record seq=%lu (net_state=%d)",
                         (unsigned long)ctx.sequence_id, net_state);
            } else {
                ESP_LOGE(TAG, "Cache write failed for seq=%lu (err=0x%x)",
                         (unsigned long)ctx.sequence_id, cache_err);
                eval_increment_metric("data_loss_count", 1);
            }
        }
    }

    s_task_publish = NULL;
    vTaskDelete(NULL);
}

/* ======================== Task 4: replay_task ======================== */

/**
 * @brief Monitor for network transitions to ONLINE and replay all cached records.
 *
 * This task blocks on a notification semaphore. When the network state changes
 * to NET_ONLINE (via scheduler_set_network_state), it triggers uif_replay_all()
 * which reads cached records and pushes them to mqtt_out_queue.
 */
static SemaphoreHandle_t s_replay_trigger = NULL;

static void replay_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "replay_task started");

    while (s_running) {
        /* Wake on reconnect and periodically continue an acknowledged replay chain. */
        xSemaphoreTake(s_replay_trigger, pdMS_TO_TICKS(500));

        if (!s_running) {
            break;
        }

        network_state_t net_state = get_network_state_locked();
        if (net_state != NET_ONLINE) {
            continue;
        }

        if (uif_get_cached_count() == 0) continue;

        esp_err_t replay_err = uif_replay_all(s_mqtt_out_queue);
        if (replay_err == ESP_OK) {
            int cached_count = uif_get_cached_count();
            ESP_LOGI(TAG, "Cache replay completed, %d records still cached", cached_count);
            eval_increment_metric("replayed_records", (uint32_t)cached_count);
        } else {
            ESP_LOGE(TAG, "Cache replay failed (err=0x%x)", replay_err);
        }
    }

    s_task_replay = NULL;
    vTaskDelete(NULL);
}

/* ======================== Task 5: resource_monitor_task ======================== */

/**
 * @brief Periodically collect system resource metrics and push to eval_queue.
 *
 * Logs free heap, minimum free heap, CPU load estimate, and flash usage.
 */
static void resource_monitor_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "resource_monitor_task started");

    uint32_t min_heap_seen = UINT32_MAX;

    while (s_running) {
        vTaskDelay(pdMS_TO_TICKS(EVAL_LOG_INTERVAL_MS));

        if (!s_running) {
            break;
        }

        /* Heap statistics */
        uint32_t free_heap = esp_get_free_heap_size();
        uint32_t min_heap  = esp_get_minimum_free_heap_size();
        if (min_heap < min_heap_seen) {
            min_heap_seen = min_heap;
        }

        /* Flash statistics */
        uint32_t flash_chip_size = 0;
        esp_flash_get_size(NULL, &flash_chip_size);

        /* CPU load - approximate using idle task counters if available.
         * For a more accurate reading, use vTaskGetRunTimeStats() with
         * configGENERATE_RUN_TIME_STATS enabled.
         * Here we estimate load from the number of active tasks.
         */
        uint32_t task_count = uxTaskGetNumberOfTasks();
        /* Rough heuristic: more tasks = more load; real impl uses run-time stats */
        uint32_t cpu_load_est = (task_count > 10) ? 80 : (task_count * 8);
        if (cpu_load_est > 100) {
            cpu_load_est = 100;
        }

        /* Update eval metrics */
        eval_increment_metric("free_heap_min", min_heap_seen);
        eval_increment_metric("cpu_load_percent", cpu_load_est);

        /* Push a resource event to eval queue */
        eval_event_t evt = {
            .timestamp_ms = esp_timer_get_time() / 1000LL,
            .sequence_id  = 0,
        };
        snprintf(evt.event_type, sizeof(evt.event_type), "resource_stats");
        snprintf(evt.detail, sizeof(evt.detail),
                 "heap_free=%lu heap_min=%lu flash_size=%lu tasks=%lu cpu_est=%lu%%",
                 (unsigned long)free_heap,
                 (unsigned long)min_heap_seen,
                 (unsigned long)flash_chip_size,
                 (unsigned long)task_count,
                 (unsigned long)cpu_load_est);

        eval_log_event(&evt);

        ESP_LOGI(TAG, "RES: heap=%lu min_heap=%lu flash=%lu tasks=%lu cpu~%lu%%",
                 (unsigned long)free_heap,
                 (unsigned long)min_heap_seen,
                 (unsigned long)flash_chip_size,
                 (unsigned long)task_count,
                 (unsigned long)cpu_load_est);
    }

    s_task_resource_mon = NULL;
    vTaskDelete(NULL);
}

/* ======================== Public API ======================== */

void scheduler_init(QueueHandle_t raw_q,
                    QueueHandle_t ctx_q,
                    QueueHandle_t mqtt_out_q,
                    QueueHandle_t mqtt_cmd_q,
                    QueueHandle_t eval_q)
{
    ESP_LOGI(TAG, "Initializing scheduler");

    s_raw_data_queue  = raw_q;
    s_context_queue   = ctx_q;
    s_mqtt_out_queue  = mqtt_out_q;
    s_mqtt_cmd_queue  = mqtt_cmd_q;
    s_eval_queue      = eval_q;

    /* Create synchronization primitives */
    s_network_state_mutex = xSemaphoreCreateMutex();
    if (s_network_state_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create network state mutex");
    }

    s_replay_trigger = xSemaphoreCreateBinary();
    if (s_replay_trigger == NULL) {
        ESP_LOGE(TAG, "Failed to create replay trigger semaphore");
    }

    /* Create tasks - they begin executing immediately.
     * The s_running flag gates their main loops.
     */
    BaseType_t ret;

    ret = xTaskCreate(modbus_poll_task,
                      "modbus_poll",
                      TASK_STACK_SIZE_MODBUS,
                      NULL,
                      TASK_PRIORITY_MODBUS,
                      &s_task_modbus_poll);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create modbus_poll_task");
    }

    ret = xTaskCreate(context_build_task,
                      "ctx_build",
                      TASK_STACK_SIZE_TCM,
                      NULL,
                      TASK_PRIORITY_TCM,
                      &s_task_context_build);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create context_build_task");
    }

    ret = xTaskCreate(publish_task,
                      "publish",
                      TASK_STACK_SIZE_MQTT,
                      NULL,
                      TASK_PRIORITY_MQTT,
                      &s_task_publish);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create publish_task");
    }

    ret = xTaskCreate(replay_task,
                      "replay",
                      TASK_STACK_SIZE_SCHED,
                      NULL,
                      TASK_PRIORITY_SCHED,
                      &s_task_replay);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create replay_task");
    }

    ret = xTaskCreate(resource_monitor_task,
                      "res_monitor",
                      TASK_STACK_SIZE_EVAL,
                      NULL,
                      TASK_PRIORITY_EVAL,
                      &s_task_resource_mon);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create resource_monitor_task");
    }

    ESP_LOGI(TAG, "Scheduler initialized with dynamic AMM polling");
}

void scheduler_start(void)
{
    ESP_LOGI(TAG, "Starting scheduler tasks");
    s_running = true;
    ESP_LOGI(TAG, "All scheduler tasks started");
}

void scheduler_stop(void)
{
    ESP_LOGI(TAG, "Stopping scheduler tasks");
    s_running = false;

    /* Give tasks time to exit their loops cleanly */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* Delete any tasks that did not self-terminate */
    if (s_task_modbus_poll) {
        vTaskDelete(s_task_modbus_poll);
        s_task_modbus_poll = NULL;
    }
    if (s_task_context_build) {
        vTaskDelete(s_task_context_build);
        s_task_context_build = NULL;
    }
    if (s_task_publish) {
        vTaskDelete(s_task_publish);
        s_task_publish = NULL;
    }
    if (s_task_replay) {
        vTaskDelete(s_task_replay);
        s_task_replay = NULL;
    }
    if (s_task_resource_mon) {
        vTaskDelete(s_task_resource_mon);
        s_task_resource_mon = NULL;
    }

    /* Clean up synchronization primitives */
    if (s_network_state_mutex) {
        vSemaphoreDelete(s_network_state_mutex);
        s_network_state_mutex = NULL;
    }
    if (s_replay_trigger) {
        vSemaphoreDelete(s_replay_trigger);
        s_replay_trigger = NULL;
    }

    ESP_LOGI(TAG, "Scheduler stopped");
}

network_state_t scheduler_get_network_state(void)
{
    return get_network_state_locked();
}

void scheduler_set_network_state(network_state_t state)
{
    network_state_t prev;

    if (s_network_state_mutex != NULL &&
        xSemaphoreTake(s_network_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        prev = s_network_state;
        s_network_state = state;
        xSemaphoreGive(s_network_state_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to acquire network state mutex");
        return;
    }

    ESP_LOGI(TAG, "Network state transition: %d -> %d", prev, state);

    /* Trigger replay when transitioning to ONLINE from a non-online state */
    if (state == NET_ONLINE && prev != NET_ONLINE) {
        ESP_LOGI(TAG, "Signaling replay task for cache flush");
        if (s_replay_trigger) {
            xSemaphoreGive(s_replay_trigger);
        }
    }
}
