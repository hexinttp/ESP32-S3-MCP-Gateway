#include "uif_persistence.h"

#include <string.h>
#include "amm/amm_mapping.h"
#include "esp_log.h"
#include "mqtt_comm/mqtt_handler.h"
#include "storage/offline_store.h"
#include "config/runtime_config.h"
#include "cloud_adapter/cloud_adapter.h"

static const char *TAG = "UIF";
static bool s_initialized;
static volatile bool s_replay_outstanding;
static volatile uint32_t s_replay_sequence;

static void replay_acknowledged(uint32_t sequence_id)
{
    esp_err_t err = offline_store_remove(sequence_id);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Replay acknowledged and removed: seq=%lu", (unsigned long)sequence_id);
    }
    if (s_replay_sequence == sequence_id) s_replay_outstanding = false;
}

esp_err_t uif_init(void)
{
    if (s_initialized) return ESP_OK;
    esp_err_t err = offline_store_init();
    if (err != ESP_OK) return err;
    mqtt_set_publish_ack_callback(replay_acknowledged);
    s_initialized = true;
    ESP_LOGI(TAG, "Durable UIF initialized: SPI Flash primary, TF overflow");
    return ESP_OK;
}

esp_err_t uif_cache_record(const tcm_context_t *ctx)
{
    if (!s_initialized || ctx == NULL) return ESP_ERR_INVALID_STATE;
    char payload[TCM_MAX_JSON_LEN];
    if (tcm_serialize_json(ctx, payload, sizeof(payload)) < 0) return ESP_FAIL;
    char topic[128];
    if (amm_copy_mqtt_topic(ctx->source_protocol, ctx->channel_id, ctx->slave_id,
                            ctx->register_address, topic, sizeof(topic)) != ESP_OK) {
        runtime_config_t runtime;
        runtime_config_get(&runtime);
        strlcpy(topic, runtime.mqtt.data_prefix, sizeof(topic));
        strlcat(topic, ctx->device_id, sizeof(topic));
        strlcat(topic, "/", sizeof(topic));
        strlcat(topic, ctx->point_id, sizeof(topic));
    }
    return offline_store_put(ctx->sequence_id, topic, payload);
}

int uif_get_cached_count(void) { return offline_store_count(); }

esp_err_t uif_get_cached_record(int index, uif_cache_entry_t *out)
{
    if (out == NULL || index != 0) return ESP_ERR_INVALID_ARG;
    offline_record_t record;
    esp_err_t err = offline_store_peek_oldest(&record);
    if (err != ESP_OK) return err;
    memset(out, 0, sizeof(*out));
    out->sequence_id = record.sequence_id;
    out->pending = true;
    strlcpy(out->json_data, record.payload, sizeof(out->json_data));
    return ESP_OK;
}

esp_err_t uif_mark_replayed(int index)
{
    if (index != 0) return ESP_ERR_INVALID_ARG;
    offline_record_t record;
    esp_err_t err = offline_store_peek_oldest(&record);
    return err == ESP_OK ? offline_store_remove(record.sequence_id) : err;
}

esp_err_t uif_clear_replayed(void) { return ESP_OK; }

esp_err_t uif_replay_all(QueueHandle_t mqtt_out_queue)
{
    (void)mqtt_out_queue;
    if (!s_initialized || !mqtt_is_connected()) return ESP_ERR_INVALID_STATE;
    if (s_replay_outstanding) return ESP_OK;

    runtime_config_t runtime;
    runtime_config_get(&runtime);
    if (runtime.mqtt.platform_type == MQTT_PLATFORM_THINGSCLOUD) {
        offline_record_t record;
        esp_err_t err = offline_store_peek_oldest(&record);
        if (err == ESP_ERR_NOT_FOUND) return ESP_OK;
        if (err != ESP_OK) return err;

        tcm_context_t ctx;
        if (tcm_deserialize_json(record.payload, &ctx) != 0) {
            ESP_LOGE(TAG, "Invalid cached TCM record, retaining seq=%lu",
                     (unsigned long)record.sequence_id);
            return ESP_ERR_INVALID_RESPONSE;
        }
        err = thingscloud_replay_context(&ctx);
        if (err != ESP_OK) return err;
        err = offline_store_remove(record.sequence_id);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "ThingsCloud replay accepted and removed: seq=%lu",
                     (unsigned long)record.sequence_id);
        }
        return err;
    }

    int scheduled = 0;
    while (scheduled < 16) {
        offline_record_t record;
        esp_err_t err = offline_store_peek_oldest(&record);
        if (err == ESP_ERR_NOT_FOUND) return ESP_OK;
        if (err != ESP_OK) return err;
        s_replay_sequence = record.sequence_id;
        s_replay_outstanding = true;
        err = mqtt_publish_tracked(record.topic, record.payload, 1, record.sequence_id);
        if (err != ESP_OK) {
            s_replay_outstanding = false;
            return scheduled > 0 ? ESP_OK : err;
        }
        ++scheduled;
        /* The file remains durable until MQTT_EVENT_PUBLISHED acknowledges it. */
        break;
    }
    return ESP_OK;
}

int uif_get_cache_usage_percent(void) { return offline_store_usage_percent(); }
int uif_get_data_loss_count(void) { return offline_store_data_loss_count(); }
void uif_replay_connection_lost(void)
{
    s_replay_outstanding = false;
    s_replay_sequence = 0;
}
void uif_destroy(void) { s_initialized = false; s_replay_outstanding = false; }
