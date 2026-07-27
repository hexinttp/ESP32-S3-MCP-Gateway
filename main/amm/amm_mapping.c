/**
 * @file amm_mapping.c
 * @brief Adaptive Mapping Model (AMM) - Implementation
 *
 * Maintains a thread-safe registry of Modbus-to-MQTT mapping entries with
 * NVS persistence and downlink command validation.
 */

#include <string.h>
#include <stdio.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_psram.h"

#include "amm/amm_mapping.h"
#include "gateway_config.h"
#include "tcm/tcm_context.h"

static const char *TAG = "AMM";

/* ======================== Private State ======================== */

/** Mapping table lives in PSRAM on the target board. */
static amm_mapping_entry_t *s_mapping_table = NULL;
static int s_mapping_capacity = 0;

/** Number of entries currently loaded (active + inactive slots). */
static int s_mapping_count = 0;

/** Mutex guarding all reads/writes to the mapping table. */
static SemaphoreHandle_t s_amm_mutex = NULL;

/** Flag indicating whether amm_init() has completed. */
static bool s_amm_initialized = false;
static uint32_t s_model_version = 0;
static uint32_t s_loaded_schema_version = 0;
static const char *s_nvs_partition = AMM_NVS_PARTITION;

/* Binary layout used by AMM schema v2, before grouped-read metadata existed. */
typedef struct {
    uint32_t mapping_version;
    source_protocol_t source_protocol;
    uint8_t channel_id;
    uint8_t slave_id;
    uint8_t function_code;
    uint16_t register_address;
    data_type_t data_type;
    byte_order_t byte_order;
    float scale_factor;
    float offset;
    uint32_t poll_interval_ms;
    uint8_t priority;
    bool discovered;
    char device_id[AMM_MAX_DEVICE_NAME_LEN];
    char point_id[AMM_MAX_POINT_NAME_LEN];
    char measurement_name[AMM_MAX_POINT_NAME_LEN];
    char unit[AMM_MAX_UNIT_LEN];
    char mqtt_topic[AMM_MAX_TOPIC_LEN];
    control_constraint_t constraint;
    bool active;
} amm_mapping_entry_v2_t;

/* Binary layout used by schema v4, before industrial type/provenance fields. */
typedef struct {
    uint32_t mapping_version;
    source_protocol_t source_protocol;
    uint8_t channel_id;
    uint8_t slave_id;
    uint8_t function_code;
    uint16_t register_address;
    data_type_t data_type;
    byte_order_t byte_order;
    float scale_factor;
    float offset;
    uint32_t poll_interval_ms;
    uint8_t priority;
    bool discovered;
    char device_id[AMM_MAX_DEVICE_NAME_LEN];
    char point_id[AMM_MAX_POINT_NAME_LEN];
    char measurement_name[AMM_MAX_POINT_NAME_LEN];
    char unit[AMM_MAX_UNIT_LEN];
    char mqtt_topic[AMM_MAX_TOPIC_LEN];
    control_constraint_t constraint;
    bool active;
    uint16_t read_start_address;
    uint8_t read_register_count;
    uint8_t value_register_index;
} amm_mapping_entry_v4_t;

/* Forward declaration – used by add/remove before its definition below. */
static esp_err_t amm_save_to_nvs_unlocked(void);
static esp_err_t amm_save_entry_to_nvs_unlocked(int index);
static esp_err_t amm_save_rollback_unlocked(void);

static uint8_t amm_point_register_count(data_type_t data_type)
{
    if (data_type == DT_FLOAT32 || data_type == DT_INT32 ||
        data_type == DT_UINT32) return 2;
    if (data_type == DT_FLOAT64 || data_type == DT_INT64 ||
        data_type == DT_UINT64) return 4;
    return 1;
}

static void amm_normalize_read_window(amm_mapping_entry_t *entry)
{
    if (entry->object_type < MODBUS_OBJECT_COIL ||
        entry->object_type > MODBUS_OBJECT_INPUT_REGISTER) {
        entry->object_type = entry->function_code >= 1 && entry->function_code <= 4
            ? (modbus_object_type_t)entry->function_code
            : MODBUS_OBJECT_HOLDING_REGISTER;
    }
    if (entry->retry_count == 0) entry->retry_count = 2;
    if (entry->retry_backoff_ms == 0) entry->retry_backoff_ms = 50;
    if (entry->semantic_confidence > 100) entry->semantic_confidence = 100;
    uint8_t point_count = amm_point_register_count(entry->data_type);
    if (entry->data_type == DT_ASCII && entry->string_length > 0) {
        point_count = (entry->string_length + 1U) / 2U;
    }
    if (entry->read_register_count == 0 ||
        entry->read_register_count > AMM_MAX_READ_REGISTERS ||
        entry->value_register_index + point_count > entry->read_register_count) {
        entry->read_start_address = entry->register_address;
        entry->read_register_count = point_count;
        entry->value_register_index = 0;
    }
}

/* ======================== Lock Helpers ======================== */

static inline void amm_lock(void)
{
    if (s_amm_mutex) {
        xSemaphoreTake(s_amm_mutex, portMAX_DELAY);
    }
}

static inline void amm_unlock(void)
{
    if (s_amm_mutex) {
        xSemaphoreGive(s_amm_mutex);
    }
}

/* ======================== Initial State And Migration ======================== */

static void amm_initialize_empty(void)
{
    if (s_mapping_table != NULL) {
        memset(s_mapping_table, 0,
               (size_t)s_mapping_capacity * sizeof(s_mapping_table[0]));
    }
    s_mapping_count = 0;
    s_model_version = 0;
    ESP_LOGI(TAG, "Initialized empty mapping table");
}

static bool amm_is_legacy_demo_entry(const amm_mapping_entry_t *entry)
{
    if (entry == NULL || entry->source_protocol != SRC_MODBUS_RTU ||
        entry->channel_id != 0) {
        return false;
    }

    return (strcmp(entry->device_id, "plc_line1_01") == 0 &&
            (strcmp(entry->point_id, "motor_temp_01") == 0 ||
             strcmp(entry->point_id, "pressure_01") == 0)) ||
           (strcmp(entry->device_id, "plc_line2_01") == 0 &&
            (strcmp(entry->point_id, "speed_01") == 0 ||
             strcmp(entry->point_id, "current_01") == 0));
}

/**
 * Remove only the four mappings created by older firmware. The schema marker
 * makes this a one-time migration and leaves all real/discovered points intact.
 */
static int amm_remove_legacy_demo_entries(void)
{
    int write_index = 0;
    int removed = 0;

    for (int read_index = 0; read_index < s_mapping_count; ++read_index) {
        if (amm_is_legacy_demo_entry(&s_mapping_table[read_index])) {
            ++removed;
            continue;
        }
        if (write_index != read_index) {
            s_mapping_table[write_index] = s_mapping_table[read_index];
        }
        ++write_index;
    }

    if (removed > 0) {
        memset(&s_mapping_table[write_index], 0,
               (size_t)(s_mapping_count - write_index) * sizeof(s_mapping_table[0]));
        s_mapping_count = write_index;
        ++s_model_version;
        ESP_LOGI(TAG, "Removed %d legacy demo mapping entries", removed);
    }
    return removed;
}

/* ======================== Public API ======================== */

void amm_init(void)
{
    if (s_amm_initialized) {
        ESP_LOGW(TAG, "AMM already initialized – skipping");
        return;
    }

    /* Create mutex */
    s_amm_mutex = xSemaphoreCreateMutex();
    if (!s_amm_mutex) {
        ESP_LOGE(TAG, "Failed to create AMM mutex");
        return;
    }

    if (esp_psram_is_initialized()) {
        s_mapping_table = heap_caps_calloc(
            AMM_MAX_MAPPING_ENTRIES, sizeof(s_mapping_table[0]),
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_mapping_table != NULL) {
            s_mapping_capacity = AMM_MAX_MAPPING_ENTRIES;
        }
    }
    if (s_mapping_table == NULL) {
        s_mapping_table = heap_caps_calloc(
            AMM_FALLBACK_MAPPING_ENTRIES, sizeof(s_mapping_table[0]),
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (s_mapping_table != NULL) {
            s_mapping_capacity = AMM_FALLBACK_MAPPING_ENTRIES;
            ESP_LOGW(TAG, "PSRAM unavailable; AMM capacity reduced to %d",
                     s_mapping_capacity);
        }
    }
    if (s_mapping_table == NULL) {
        ESP_LOGE(TAG, "Failed to allocate AMM mapping table");
        return;
    }

    esp_err_t partition_err = nvs_flash_init_partition(AMM_NVS_PARTITION);
    if (partition_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        partition_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(
            nvs_flash_erase_partition(AMM_NVS_PARTITION));
        partition_err = nvs_flash_init_partition(AMM_NVS_PARTITION);
    }
    if (partition_err != ESP_OK) {
        ESP_LOGE(TAG, "AMM NVS partition init failed: %s",
                 esp_err_to_name(partition_err));
    }

    memset(s_mapping_table, 0,
           (size_t)s_mapping_capacity * sizeof(s_mapping_table[0]));
    s_mapping_count = 0;

    /* Load the dedicated AMM partition, then migrate the legacy default NVS. */
    esp_err_t ret = partition_err == ESP_OK ? amm_load_from_nvs() : partition_err;
    if (ret != ESP_OK && partition_err == ESP_OK) {
        s_nvs_partition = "nvs";
        esp_err_t legacy_ret = amm_load_from_nvs();
        s_nvs_partition = AMM_NVS_PARTITION;
        if (legacy_ret == ESP_OK) {
            ESP_LOGI(TAG, "Migrating legacy AMM mappings to %s", AMM_NVS_PARTITION);
            ret = amm_save_to_nvs();
        }
    }
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "NVS load failed (%s) - starting with no mappings",
                 esp_err_to_name(ret));
        amm_initialize_empty();

        /* Persist the empty state so a reboot remains deterministic. */
        esp_err_t save_ret = amm_save_to_nvs();
        if (save_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to persist empty mapping table to NVS: %s",
                     esp_err_to_name(save_ret));
        }
    } else {
        ESP_LOGI(TAG, "Loaded %d mapping entries from NVS", s_mapping_count);
        if (s_loaded_schema_version < AMM_NVS_SCHEMA_VERSION) {
            int removed = amm_remove_legacy_demo_entries();
            esp_err_t save_ret = amm_save_to_nvs();
            if (save_ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to persist AMM schema migration: %s",
                         esp_err_to_name(save_ret));
            } else {
                ESP_LOGI(TAG, "AMM schema migrated to version %d (%d demo entries removed)",
                         AMM_NVS_SCHEMA_VERSION, removed);
            }
        }
    }

    s_amm_initialized = true;
    ESP_LOGI(TAG, "AMM initialized (%d active entries, capacity=%d)",
             amm_get_mapping_count(), s_mapping_capacity);
}

int amm_get_capacity(void)
{
    return s_mapping_capacity;
}

esp_err_t amm_add_mapping(const amm_mapping_entry_t *entry)
{
    if (!entry) {
        ESP_LOGE(TAG, "amm_add_mapping: entry is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    amm_lock();

    if (s_mapping_table == NULL) {
        amm_unlock();
        return ESP_ERR_NO_MEM;
    }

    /*
     * Object space is part of the key: identical offsets in coils, discrete
     * inputs, holding registers and input registers are distinct points.
     * If one exists, overwrite it in place rather than consuming a new slot.
     */
    for (int i = 0; i < s_mapping_count; i++) {
        if (s_mapping_table[i].active &&
            s_mapping_table[i].source_protocol == entry->source_protocol &&
            s_mapping_table[i].channel_id == entry->channel_id &&
            s_mapping_table[i].slave_id == entry->slave_id &&
            s_mapping_table[i].function_code == entry->function_code &&
            s_mapping_table[i].register_address == entry->register_address) {
            esp_err_t snapshot = amm_save_rollback_unlocked();
            if (snapshot != ESP_OK && snapshot != ESP_ERR_NOT_FOUND) {
                amm_unlock();
                return snapshot;
            }
            ESP_LOGW(TAG, "amm_add_mapping: overwriting existing entry "
                     "slave=%u reg=%u", entry->slave_id, entry->register_address);
            memcpy(&s_mapping_table[i], entry, sizeof(amm_mapping_entry_t));
            s_mapping_table[i].active = true;
            amm_normalize_read_window(&s_mapping_table[i]);
            s_mapping_table[i].mapping_version = ++s_model_version;
            esp_err_t ret = amm_save_entry_to_nvs_unlocked(i);
            amm_unlock();
            return ret;
        }
    }

    esp_err_t snapshot = amm_save_rollback_unlocked();
    if (snapshot != ESP_OK && snapshot != ESP_ERR_NOT_FOUND) {
        amm_unlock();
        return snapshot;
    }
    int target = -1;
    for (int i = 0; i < s_mapping_count; ++i) {
        if (!s_mapping_table[i].active) {
            target = i;
            break;
        }
    }
    if (target < 0) {
        if (s_mapping_count >= s_mapping_capacity) {
            amm_unlock();
            ESP_LOGE(TAG, "amm_add_mapping: table full (%d/%d)",
                     s_mapping_count, s_mapping_capacity);
            return ESP_ERR_NO_MEM;
        }
        target = s_mapping_count++;
    }

    memcpy(&s_mapping_table[target], entry, sizeof(amm_mapping_entry_t));
    s_mapping_table[target].active = true;
    amm_normalize_read_window(&s_mapping_table[target]);
    s_mapping_table[target].mapping_version = ++s_model_version;
    if (s_mapping_table[target].poll_interval_ms == 0) {
        s_mapping_table[target].poll_interval_ms = POLL_INTERVAL_MS;
    }

    esp_err_t ret = amm_save_entry_to_nvs_unlocked(target);
    amm_unlock();

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Added mapping: slave=%u reg=%u -> %s/%s",
                 entry->slave_id, entry->register_address,
                 entry->device_id, entry->point_id);
    }
    return ret;
}

esp_err_t amm_remove_mapping(uint8_t slave_id, uint16_t reg_addr)
{
    amm_lock();

    for (int i = 0; i < s_mapping_count; i++) {
        if (s_mapping_table[i].active &&
            s_mapping_table[i].slave_id == slave_id &&
            s_mapping_table[i].register_address == reg_addr) {
            esp_err_t snapshot = amm_save_rollback_unlocked();
            if (snapshot != ESP_OK) {
                amm_unlock();
                return snapshot;
            }
            s_mapping_table[i].active = false;
            ++s_model_version;
            ESP_LOGI(TAG, "Deactivated mapping: slave=%u reg=%u", slave_id, reg_addr);

            esp_err_t ret = amm_save_entry_to_nvs_unlocked(i);
            amm_unlock();
            return ret;
        }
    }

    amm_unlock();
    ESP_LOGW(TAG, "amm_remove_mapping: entry not found slave=%u reg=%u",
             slave_id, reg_addr);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t amm_remove_mapping_for_channel(source_protocol_t protocol, uint8_t channel_id,
                                         uint8_t slave_id, uint16_t reg_addr)
{
    amm_lock();
    for (int i = 0; i < s_mapping_count; ++i) {
        amm_mapping_entry_t *entry = &s_mapping_table[i];
        if (entry->active && entry->source_protocol == protocol &&
            entry->channel_id == channel_id && entry->slave_id == slave_id &&
            entry->register_address == reg_addr) {
            esp_err_t snapshot = amm_save_rollback_unlocked();
            if (snapshot != ESP_OK) {
                amm_unlock();
                return snapshot;
            }
            entry->active = false;
            ++s_model_version;
            esp_err_t err = amm_save_entry_to_nvs_unlocked(i);
            amm_unlock();
            return err;
        }
    }
    amm_unlock();
    return ESP_ERR_NOT_FOUND;
}

amm_mapping_entry_t *amm_find_mapping(uint8_t slave_id, uint16_t reg_addr)
{
    amm_lock();

    for (int i = 0; i < s_mapping_count; i++) {
        if (s_mapping_table[i].active &&
            s_mapping_table[i].slave_id == slave_id &&
            s_mapping_table[i].register_address == reg_addr) {
            amm_unlock();
            return &s_mapping_table[i];
        }
    }

    amm_unlock();
    return NULL;
}

int amm_get_mapping_count(void)
{
    int count = 0;

    amm_lock();
    for (int i = 0; i < s_mapping_count; i++) {
        if (s_mapping_table[i].active) {
            count++;
        }
    }
    amm_unlock();

    return count;
}

esp_err_t amm_clear_mappings(void)
{
    amm_lock();
    esp_err_t snapshot_err = amm_save_rollback_unlocked();
    if (snapshot_err != ESP_OK && snapshot_err != ESP_ERR_NOT_FOUND) {
        amm_unlock();
        return snapshot_err;
    }

    nvs_handle_t handle = 0;
    esp_err_t ret = nvs_open_from_partition(
        s_nvs_partition, AMM_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret == ESP_OK) {
        ret = nvs_set_i32(handle, AMM_NVS_KEY_COUNT, 0);
    }
    if (ret == ESP_OK) {
        ret = nvs_set_u32(handle, AMM_NVS_KEY_SCHEMA, AMM_NVS_SCHEMA_VERSION);
    }
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    if (handle != 0) {
        nvs_close(handle);
    }

    if (ret == ESP_OK) {
        memset(s_mapping_table, 0,
               (size_t)s_mapping_capacity * sizeof(s_mapping_table[0]));
        s_mapping_count = 0;
        ++s_model_version;
        ESP_LOGI(TAG, "Cleared all AMM mappings");
    }

    amm_unlock();
    return ret;
}

esp_err_t amm_update_mapping(int index, const amm_mapping_entry_t *entry)
{
    if (entry == NULL || index < 0 || index >= s_mapping_capacity) return ESP_ERR_INVALID_ARG;
    amm_lock();
    if (index >= s_mapping_count || !s_mapping_table[index].active) {
        amm_unlock();
        return ESP_ERR_NOT_FOUND;
    }
    esp_err_t snapshot = amm_save_rollback_unlocked();
    if (snapshot != ESP_OK) {
        amm_unlock();
        return snapshot;
    }
    s_mapping_table[index] = *entry;
    s_mapping_table[index].active = true;
    amm_normalize_read_window(&s_mapping_table[index]);
    s_mapping_table[index].mapping_version = ++s_model_version;
    if (s_mapping_table[index].poll_interval_ms == 0) {
        s_mapping_table[index].poll_interval_ms = POLL_INTERVAL_MS;
    }
    esp_err_t err = amm_save_entry_to_nvs_unlocked(index);
    amm_unlock();
    return err;
}

int amm_get_entries(amm_mapping_entry_t *out, int max_entries)
{
    if (out == NULL || max_entries <= 0) return 0;
    int copied = 0;
    amm_lock();
    for (int i = 0; i < s_mapping_count && copied < max_entries; ++i) {
        if (s_mapping_table[i].active) out[copied++] = s_mapping_table[i];
    }
    amm_unlock();
    return copied;
}

static bool amm_import_contains_device(const amm_mapping_entry_t *entries, int count,
                                       const amm_mapping_entry_t *candidate)
{
    for (int i = 0; i < count; ++i) {
        if (entries[i].source_protocol == candidate->source_protocol &&
            entries[i].channel_id == candidate->channel_id &&
            entries[i].slave_id == candidate->slave_id) {
            return true;
        }
    }
    return false;
}

esp_err_t amm_import_mappings(const amm_mapping_entry_t *entries, int count,
                              bool replace_devices, int *imported_count)
{
    if (entries == NULL || count <= 0 || s_mapping_table == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    for (int i = 0; i < count; ++i) {
        for (int j = i + 1; j < count; ++j) {
            bool same_source = entries[i].source_protocol == entries[j].source_protocol &&
                entries[i].channel_id == entries[j].channel_id &&
                entries[i].slave_id == entries[j].slave_id &&
                entries[i].function_code == entries[j].function_code &&
                entries[i].register_address == entries[j].register_address;
            bool same_semantic = strcmp(entries[i].device_id, entries[j].device_id) == 0 &&
                strcmp(entries[i].point_id, entries[j].point_id) == 0;
            if ((same_source && !same_semantic) || (!same_source && same_semantic)) {
                ESP_LOGE(TAG, "AMM profile conflict between entries %d and %d", i, j);
                if (imported_count != NULL) *imported_count = 0;
                return ESP_ERR_INVALID_STATE;
            }
        }
    }

    amm_mapping_entry_t *staging = heap_caps_calloc(
        (size_t)s_mapping_capacity, sizeof(staging[0]),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (staging == NULL) {
        staging = heap_caps_calloc(
            (size_t)s_mapping_capacity, sizeof(staging[0]),
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (staging == NULL) return ESP_ERR_NO_MEM;

    int staged_count = 0;
    int imported = 0;
    esp_err_t result = ESP_OK;

    amm_lock();
    if (amm_save_rollback_unlocked() != ESP_OK && s_mapping_count > 0) {
        amm_unlock();
        free(staging);
        return ESP_FAIL;
    }
    for (int i = 0; i < s_mapping_count; ++i) {
        const amm_mapping_entry_t *current = &s_mapping_table[i];
        if (!current->active) continue;
        if (replace_devices &&
            amm_import_contains_device(entries, count, current)) {
            continue;
        }
        if (staged_count >= s_mapping_capacity) {
            result = ESP_ERR_NO_MEM;
            break;
        }
        staging[staged_count++] = *current;
    }

    for (int i = 0; result == ESP_OK && i < count; ++i) {
        amm_mapping_entry_t entry = entries[i];
        if (entry.slave_id < 1 || entry.function_code < 1 ||
            entry.function_code > 4 ||
            entry.device_id[0] == '\0' || entry.point_id[0] == '\0') {
            result = ESP_ERR_INVALID_ARG;
            break;
        }

        entry.active = true;
        entry.mapping_version = ++s_model_version;
        if (entry.poll_interval_ms == 0) entry.poll_interval_ms = POLL_INTERVAL_MS;
        if (entry.scale_factor == 0.0f) entry.scale_factor = 1.0f;
        amm_normalize_read_window(&entry);

        int duplicate = -1;
        for (int j = 0; j < staged_count; ++j) {
            if (staging[j].source_protocol == entry.source_protocol &&
                staging[j].channel_id == entry.channel_id &&
                staging[j].slave_id == entry.slave_id &&
                staging[j].function_code == entry.function_code &&
                staging[j].register_address == entry.register_address) {
                duplicate = j;
                break;
            }
        }
        if (duplicate >= 0) {
            staging[duplicate] = entry;
        } else if (staged_count < s_mapping_capacity) {
            staging[staged_count++] = entry;
        } else {
            result = ESP_ERR_NO_MEM;
            break;
        }
        ++imported;
    }

    if (result == ESP_OK) {
        memset(s_mapping_table, 0,
               (size_t)s_mapping_capacity * sizeof(s_mapping_table[0]));
        memcpy(s_mapping_table, staging,
               (size_t)staged_count * sizeof(s_mapping_table[0]));
        s_mapping_count = staged_count;
        result = amm_save_to_nvs_unlocked();
    }
    amm_unlock();
    free(staging);

    if (imported_count != NULL) *imported_count = result == ESP_OK ? imported : 0;
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "Imported %d semantic mappings (%d active total)",
                 imported, s_mapping_count);
    }
    return result;
}

esp_err_t amm_get_entry_at(int active_index, amm_mapping_entry_t *out)
{
    if (active_index < 0 || out == NULL) return ESP_ERR_INVALID_ARG;

    int current = 0;
    amm_lock();
    for (int i = 0; i < s_mapping_count; ++i) {
        if (!s_mapping_table[i].active) continue;
        if (current++ == active_index) {
            *out = s_mapping_table[i];
            amm_unlock();
            return ESP_OK;
        }
    }
    amm_unlock();
    return ESP_ERR_NOT_FOUND;
}

uint32_t amm_get_model_version(void)
{
    amm_lock();
    uint32_t version = s_model_version;
    amm_unlock();
    return version;
}

esp_err_t amm_find_mapping_for_channel(source_protocol_t protocol, uint8_t channel_id,
                                       uint8_t slave_id, uint16_t reg_addr,
                                       amm_mapping_entry_t *out)
{
    if (out == NULL) return ESP_ERR_INVALID_ARG;
    amm_lock();
    for (int i = 0; i < s_mapping_count; ++i) {
        amm_mapping_entry_t *entry = &s_mapping_table[i];
        if (entry->active && entry->source_protocol == protocol &&
            entry->channel_id == channel_id && entry->slave_id == slave_id &&
            entry->register_address == reg_addr) {
            *out = *entry;
            amm_unlock();
            return ESP_OK;
        }
    }
    amm_unlock();
    return ESP_ERR_NOT_FOUND;
}

esp_err_t amm_find_mapping_for_object(source_protocol_t protocol, uint8_t channel_id,
                                      uint8_t slave_id, uint8_t function_code,
                                      uint16_t address, amm_mapping_entry_t *out)
{
    if (out == NULL) return ESP_ERR_INVALID_ARG;
    amm_lock();
    for (int i = 0; i < s_mapping_count; ++i) {
        amm_mapping_entry_t *entry = &s_mapping_table[i];
        if (entry->active && entry->source_protocol == protocol &&
            entry->channel_id == channel_id && entry->slave_id == slave_id &&
            entry->function_code == function_code &&
            entry->register_address == address) {
            *out = *entry;
            amm_unlock();
            return ESP_OK;
        }
    }
    amm_unlock();
    return ESP_ERR_NOT_FOUND;
}

esp_err_t amm_find_mapping_covering(uint8_t slave_id, uint8_t function_code,
                                    uint16_t address, amm_mapping_entry_t *out,
                                    uint8_t *word_index)
{
    if (out == NULL) return ESP_ERR_INVALID_ARG;
    amm_lock();
    for (int i = 0; i < s_mapping_count; ++i) {
        amm_mapping_entry_t *entry = &s_mapping_table[i];
        uint8_t width = amm_point_register_count(entry->data_type);
        if (entry->data_type == DT_ASCII && entry->read_register_count > 0) {
            width = entry->read_register_count;
        }
        if (entry->active && entry->slave_id == slave_id &&
            entry->function_code == function_code &&
            address >= entry->register_address &&
            (uint32_t)address < (uint32_t)entry->register_address + width) {
            *out = *entry;
            if (word_index != NULL) {
                *word_index = (uint8_t)(address - entry->register_address);
            }
            amm_unlock();
            return ESP_OK;
        }
    }
    amm_unlock();
    return ESP_ERR_NOT_FOUND;
}

esp_err_t amm_find_mapping_by_point(const char *device_id, const char *point_id,
                                    amm_mapping_entry_t *out)
{
    if (device_id == NULL || point_id == NULL || out == NULL) return ESP_ERR_INVALID_ARG;
    amm_lock();
    for (int i = 0; i < s_mapping_count; ++i) {
        amm_mapping_entry_t *entry = &s_mapping_table[i];
        if (entry->active && strcmp(entry->device_id, device_id) == 0 &&
            strcmp(entry->point_id, point_id) == 0) {
            *out = *entry;
            amm_unlock();
            return ESP_OK;
        }
    }
    amm_unlock();
    return ESP_ERR_NOT_FOUND;
}

esp_err_t amm_enrich_context(tcm_context_t *ctx)
{
    if (!ctx) {
        ESP_LOGE(TAG, "amm_enrich_context: ctx is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    amm_mapping_entry_t entry_value;
    if (amm_find_mapping_for_object(ctx->source_protocol, ctx->channel_id,
                                    ctx->slave_id, ctx->function_code,
                                    ctx->register_address,
                                    &entry_value) != ESP_OK) {
        ESP_LOGW(TAG, "amm_enrich_context: no mapping for slave=%u reg=%u",
                 ctx->slave_id, ctx->register_address);
        return ESP_ERR_NOT_FOUND;
    }
    amm_mapping_entry_t *entry = &entry_value;

    /* Copy semantic metadata into the TCM context */
    strncpy(ctx->device_id,        entry->device_id,        AMM_MAX_DEVICE_NAME_LEN - 1);
    ctx->device_id[AMM_MAX_DEVICE_NAME_LEN - 1] = '\0';

    strncpy(ctx->point_id,         entry->point_id,         AMM_MAX_POINT_NAME_LEN  - 1);
    ctx->point_id[AMM_MAX_POINT_NAME_LEN - 1] = '\0';

    strncpy(ctx->measurement_name, entry->measurement_name, AMM_MAX_POINT_NAME_LEN  - 1);
    ctx->measurement_name[AMM_MAX_POINT_NAME_LEN - 1] = '\0';

    strncpy(ctx->unit,             entry->unit,             AMM_MAX_UNIT_LEN        - 1);
    ctx->unit[AMM_MAX_UNIT_LEN - 1] = '\0';

    ctx->control_constraint = entry->constraint;
    ctx->source_protocol = entry->source_protocol;
    ctx->channel_id = entry->channel_id;
    ctx->data_type = entry->data_type;
    ctx->byte_order = entry->byte_order;
    ctx->scale_factor = entry->scale_factor;
    ctx->offset = entry->offset;
    ctx->mapping_version = entry->mapping_version;
    ctx->object_type = entry->object_type;
    ctx->semantic_source = entry->semantic_source;
    ctx->semantic_status = entry->semantic_status;
    strlcpy(ctx->semantic_profile_id, entry->semantic_profile_id,
            sizeof(ctx->semantic_profile_id));
    ctx->semantic_profile_version = entry->semantic_profile_version;
    ctx->semantic_confidence = entry->semantic_confidence;
    strlcpy(ctx->semantic_evidence, entry->semantic_evidence,
            sizeof(ctx->semantic_evidence));
    ctx->raw_value = ctx->value;
    ctx->value = ctx->raw_value * entry->scale_factor + entry->offset;

    ESP_LOGD(TAG, "Enriched ctx: %s/%s [%s] slave=%u reg=%u",
             ctx->device_id, ctx->point_id, ctx->unit,
             ctx->slave_id, ctx->register_address);

    return ESP_OK;
}

amm_validation_result_t amm_validate_command(const tcm_context_t *cmd_ctx)
{
    amm_validation_result_t result;
    memset(&result, 0, sizeof(result));

    /* ---- Check 1: target exists in mapping ---- */
    amm_mapping_entry_t *entry =
        amm_find_mapping(cmd_ctx->slave_id, cmd_ctx->register_address);

    if (!entry) {
        result.accepted = false;
        result.matched_entry = NULL;
        snprintf(result.reject_reason, sizeof(result.reject_reason),
                 "No mapping entry for slave=%u reg=%u",
                 cmd_ctx->slave_id, cmd_ctx->register_address);
        ESP_LOGW(TAG, "Command rejected: %s", result.reject_reason);
        return result;
    }

    result.matched_entry = entry;

    /* ---- Check 5: function_code must be a valid write code (6 or 16) ---- */
    if (cmd_ctx->function_code != 5 && cmd_ctx->function_code != 6 &&
        cmd_ctx->function_code != 15 && cmd_ctx->function_code != 16) {
        result.accepted = false;
        snprintf(result.reject_reason, sizeof(result.reject_reason),
                 "Invalid write function code %u (expected 5/6/15/16)",
                 cmd_ctx->function_code);
        ESP_LOGW(TAG, "Command rejected: %s", result.reject_reason);
        return result;
    }

    /* ---- Check 2: writable flag ---- */
    if (!entry->constraint.writable) {
        result.accepted = false;
        snprintf(result.reject_reason, sizeof(result.reject_reason),
                 "Point '%s/%s' is read-only",
                 entry->device_id, entry->point_id);
        ESP_LOGW(TAG, "Command rejected: %s", result.reject_reason);
        return result;
    }

    /* ---- Check 4: data_type must match ---- */
    if (cmd_ctx->data_type != entry->data_type) {
        result.accepted = false;
        snprintf(result.reject_reason, sizeof(result.reject_reason),
                 "Data type mismatch: cmd=%d expected=%d for '%s/%s'",
                 (int)cmd_ctx->data_type, (int)entry->data_type,
                 entry->device_id, entry->point_id);
        ESP_LOGW(TAG, "Command rejected: %s", result.reject_reason);
        return result;
    }

    /* ---- Check 3: value within valid range ---- */
    float cmd_value = cmd_ctx->value;
    if (cmd_value < entry->constraint.valid_range_min ||
        cmd_value > entry->constraint.valid_range_max) {
        result.accepted = false;
        snprintf(result.reject_reason, sizeof(result.reject_reason),
                 "Value %.4f out of range [%.4f, %.4f] for '%s/%s'",
                 cmd_value,
                 entry->constraint.valid_range_min,
                 entry->constraint.valid_range_max,
                 entry->device_id, entry->point_id);
        ESP_LOGW(TAG, "Command rejected: %s", result.reject_reason);
        return result;
    }

    /* All checks passed */
    result.accepted = true;
    result.reject_reason[0] = '\0';
    ESP_LOGD(TAG, "Command accepted for '%s/%s' value=%.4f",
             entry->device_id, entry->point_id, cmd_value);
    return result;
}

const char *amm_get_mqtt_topic(uint8_t slave_id, uint16_t reg_addr)
{
    amm_mapping_entry_t *entry = amm_find_mapping(slave_id, reg_addr);
    if (entry) {
        return entry->mqtt_topic;
    }
    return NULL;
}

esp_err_t amm_copy_mqtt_topic(source_protocol_t protocol, uint8_t channel_id,
                              uint8_t slave_id, uint16_t reg_addr,
                              char *out, size_t out_size)
{
    if (out == NULL || out_size == 0) return ESP_ERR_INVALID_ARG;
    amm_mapping_entry_t entry;
    esp_err_t err = amm_find_mapping_for_channel(protocol, channel_id, slave_id, reg_addr, &entry);
    if (err != ESP_OK || entry.mqtt_topic[0] == '\0') return ESP_ERR_NOT_FOUND;
    strlcpy(out, entry.mqtt_topic, out_size);
    return ESP_OK;
}

/* ======================== NVS Persistence ======================== */

/**
 * @brief Build an NVS key for a given entry index.
 *
 * Format: "entry_<index>" (e.g. "entry_0", "entry_12").
 * NVS keys are limited to 15 characters; with a 6-digit max index this
 * stays well within bounds.
 */
static void amm_nvs_entry_key(int index, char *key_buf, size_t buf_len)
{
    snprintf(key_buf, buf_len, "%s%d", AMM_NVS_KEY_ENTRY_PREFIX, index);
}

static esp_err_t amm_save_entry_to_nvs_unlocked(int index)
{
    if (index < 0 || index >= s_mapping_count) return ESP_ERR_INVALID_ARG;

    nvs_handle_t handle;
    esp_err_t ret = nvs_open_from_partition(
        s_nvs_partition, AMM_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) return ret;

    char key[20];
    amm_nvs_entry_key(index, key, sizeof(key));
    ret = nvs_set_i32(handle, AMM_NVS_KEY_COUNT, (int32_t)s_mapping_count);
    if (ret == ESP_OK) {
        ret = nvs_set_u32(handle, AMM_NVS_KEY_SCHEMA, AMM_NVS_SCHEMA_VERSION);
    }
    if (ret == ESP_OK) {
        ret = nvs_set_blob(handle, key, &s_mapping_table[index],
                           sizeof(s_mapping_table[index]));
    }
    if (ret == ESP_OK) ret = nvs_commit(handle);
    nvs_close(handle);
    return ret;
}

/**
 * @brief Internal save helper – caller must already hold s_amm_mutex.
 *
 * Serialises each mapping entry to NVS as a binary blob, together with the
 * total slot count.
 */
static esp_err_t amm_save_to_nvs_unlocked(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open_from_partition(
        s_nvs_partition, AMM_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed for save: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Persist total slot count (active + inactive) */
    ret = nvs_set_i32(handle, AMM_NVS_KEY_COUNT, (int32_t)s_mapping_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS set count failed: %s", esp_err_to_name(ret));
        nvs_close(handle);
        return ret;
    }

    ret = nvs_set_u32(handle, AMM_NVS_KEY_SCHEMA, AMM_NVS_SCHEMA_VERSION);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS set schema version failed: %s", esp_err_to_name(ret));
        nvs_close(handle);
        return ret;
    }

    /* Persist each entry as a binary blob */
    char key[20];
    for (int i = 0; i < s_mapping_count; i++) {
        amm_nvs_entry_key(i, key, sizeof(key));
        ret = nvs_set_blob(handle, key,
                           &s_mapping_table[i],
                           sizeof(amm_mapping_entry_t));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "NVS set blob[%d] failed: %s", i, esp_err_to_name(ret));
            nvs_close(handle);
            return ret;
        }
    }

    ret = nvs_commit(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS commit failed: %s", esp_err_to_name(ret));
    }

    nvs_close(handle);
    ESP_LOGD(TAG, "Saved %d mapping entries to NVS", s_mapping_count);
    return ret;
}

static esp_err_t amm_save_rollback_unlocked(void)
{
    if (s_mapping_count <= 0) return ESP_ERR_NOT_FOUND;
    nvs_handle_t handle;
    esp_err_t err = nvs_open_from_partition(
        s_nvs_partition, AMM_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    size_t size = (size_t)s_mapping_count * sizeof(s_mapping_table[0]);
    err = nvs_set_i32(handle, AMM_NVS_KEY_ROLLBACK_COUNT, s_mapping_count);
    if (err == ESP_OK) {
        err = nvs_set_blob(handle, AMM_NVS_KEY_ROLLBACK,
                           s_mapping_table, size);
    }
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

bool amm_can_rollback(void)
{
    nvs_handle_t handle;
    int32_t count = 0;
    esp_err_t err = nvs_open_from_partition(
        s_nvs_partition, AMM_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_OK) {
        err = nvs_get_i32(handle, AMM_NVS_KEY_ROLLBACK_COUNT, &count);
        nvs_close(handle);
    }
    return err == ESP_OK && count > 0 && count <= s_mapping_capacity;
}

esp_err_t amm_rollback(void)
{
    amm_lock();
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open_from_partition(
        s_nvs_partition, AMM_NVS_NAMESPACE, NVS_READWRITE, &handle);
    int32_t count = 0;
    if (err == ESP_OK) {
        err = nvs_get_i32(handle, AMM_NVS_KEY_ROLLBACK_COUNT, &count);
    }
    size_t size = count > 0 ? (size_t)count * sizeof(s_mapping_table[0]) : 0;
    amm_mapping_entry_t *snapshot = NULL;
    if (err == ESP_OK && count > 0 && count <= s_mapping_capacity) {
        snapshot = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (snapshot == NULL) snapshot = malloc(size);
        if (snapshot == NULL) err = ESP_ERR_NO_MEM;
    } else if (err == ESP_OK) {
        err = ESP_ERR_INVALID_SIZE;
    }
    if (err == ESP_OK) {
        err = nvs_get_blob(handle, AMM_NVS_KEY_ROLLBACK, snapshot, &size);
    }
    if (handle != 0) nvs_close(handle);
    if (err == ESP_OK) {
        memset(s_mapping_table, 0,
               (size_t)s_mapping_capacity * sizeof(s_mapping_table[0]));
        memcpy(s_mapping_table, snapshot, size);
        s_mapping_count = count;
        ++s_model_version;
        for (int i = 0; i < s_mapping_count; ++i) {
            s_mapping_table[i].mapping_version = s_model_version;
        }
        err = amm_save_to_nvs_unlocked();
    }
    free(snapshot);
    amm_unlock();
    return err;
}

esp_err_t amm_save_to_nvs(void)
{
    amm_lock();
    esp_err_t ret = amm_save_to_nvs_unlocked();
    amm_unlock();
    return ret;
}

esp_err_t amm_load_from_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open_from_partition(
        s_nvs_partition, AMM_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "NVS namespace '%s' not found: %s",
                 AMM_NVS_NAMESPACE, esp_err_to_name(ret));
        return ret;
    }

    int32_t stored_count = 0;
    s_loaded_schema_version = 0;
    esp_err_t schema_ret = nvs_get_u32(handle, AMM_NVS_KEY_SCHEMA,
                                       &s_loaded_schema_version);
    if (schema_ret != ESP_OK && schema_ret != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "NVS schema version read failed: %s",
                 esp_err_to_name(schema_ret));
    }

    ret = nvs_get_i32(handle, AMM_NVS_KEY_COUNT, &stored_count);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "NVS key '%s' not found: %s",
                 AMM_NVS_KEY_COUNT, esp_err_to_name(ret));
        nvs_close(handle);
        return (ret == ESP_ERR_NVS_NOT_FOUND) ? ESP_ERR_NOT_FOUND : ret;
    }

    if (stored_count < 0 || stored_count > s_mapping_capacity) {
        ESP_LOGE(TAG, "NVS stored count %ld is out of range [0, %d]",
                 (long)stored_count, s_mapping_capacity);
        nvs_close(handle);
        return ESP_ERR_INVALID_SIZE;
    }

    char key[20];
    memset(s_mapping_table, 0,
           (size_t)s_mapping_capacity * sizeof(s_mapping_table[0]));

    for (int i = 0; i < stored_count; i++) {
        amm_nvs_entry_key(i, key, sizeof(key));

        size_t blob_len = 0;
        ret = nvs_get_blob(handle, key, NULL, &blob_len);
        if (ret == ESP_OK && blob_len == sizeof(amm_mapping_entry_t)) {
            ret = nvs_get_blob(handle, key, &s_mapping_table[i], &blob_len);
        } else if (ret == ESP_OK &&
                   blob_len == sizeof(amm_mapping_entry_v4_t) &&
                   s_loaded_schema_version == 4) {
            amm_mapping_entry_v4_t legacy;
            size_t legacy_len = sizeof(legacy);
            ret = nvs_get_blob(handle, key, &legacy, &legacy_len);
            if (ret == ESP_OK) {
                amm_mapping_entry_t *entry = &s_mapping_table[i];
                memset(entry, 0, sizeof(*entry));
                entry->mapping_version = legacy.mapping_version;
                entry->source_protocol = legacy.source_protocol;
                entry->channel_id = legacy.channel_id;
                entry->slave_id = legacy.slave_id;
                entry->function_code = legacy.function_code;
                entry->register_address = legacy.register_address;
                entry->data_type = legacy.data_type;
                entry->byte_order = legacy.byte_order;
                entry->scale_factor = legacy.scale_factor;
                entry->offset = legacy.offset;
                entry->poll_interval_ms = legacy.poll_interval_ms;
                entry->priority = legacy.priority;
                entry->discovered = legacy.discovered;
                strlcpy(entry->device_id, legacy.device_id,
                        sizeof(entry->device_id));
                strlcpy(entry->point_id, legacy.point_id,
                        sizeof(entry->point_id));
                strlcpy(entry->measurement_name, legacy.measurement_name,
                        sizeof(entry->measurement_name));
                strlcpy(entry->unit, legacy.unit, sizeof(entry->unit));
                strlcpy(entry->mqtt_topic, legacy.mqtt_topic,
                        sizeof(entry->mqtt_topic));
                entry->constraint = legacy.constraint;
                entry->active = legacy.active;
                entry->read_start_address = legacy.read_start_address;
                entry->read_register_count = legacy.read_register_count;
                entry->value_register_index = legacy.value_register_index;
                entry->object_type = legacy.function_code == 4
                    ? MODBUS_OBJECT_INPUT_REGISTER
                    : MODBUS_OBJECT_HOLDING_REGISTER;
                entry->semantic_source = SEMANTIC_SOURCE_IMPORTED;
                entry->semantic_status = SEMANTIC_STATUS_RESOLVED;
                entry->semantic_confidence = 100;
                strlcpy(entry->semantic_evidence, "schema-v4 migration",
                        sizeof(entry->semantic_evidence));
                amm_normalize_read_window(entry);
            }
        } else if (ret == ESP_OK &&
                   blob_len == sizeof(amm_mapping_entry_v2_t) &&
                   s_loaded_schema_version < AMM_NVS_SCHEMA_VERSION) {
            amm_mapping_entry_v2_t legacy;
            size_t legacy_len = sizeof(legacy);
            ret = nvs_get_blob(handle, key, &legacy, &legacy_len);
            if (ret == ESP_OK) {
                memset(&s_mapping_table[i], 0, sizeof(s_mapping_table[i]));
                memcpy(&s_mapping_table[i], &legacy, sizeof(legacy));
                s_mapping_table[i].read_start_address =
                    s_mapping_table[i].register_address;
                s_mapping_table[i].read_register_count =
                    amm_point_register_count(s_mapping_table[i].data_type);
                s_mapping_table[i].value_register_index = 0;
                amm_normalize_read_window(&s_mapping_table[i]);
            }
        } else if (ret == ESP_OK) {
            ESP_LOGE(TAG, "NVS blob[%d] size mismatch: got %u expected %u",
                     i, (unsigned)blob_len, (unsigned)sizeof(amm_mapping_entry_t));
            ret = ESP_ERR_INVALID_SIZE;
        }

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "NVS get blob[%d] failed: %s", i, esp_err_to_name(ret));
            memset(s_mapping_table, 0,
                   (size_t)s_mapping_capacity * sizeof(s_mapping_table[0]));
            s_mapping_count = 0;
            nvs_close(handle);
            return ret;
        }
    }

    s_mapping_count = (int)stored_count;
    s_model_version = 1;
    for (int i = 0; i < s_mapping_count; ++i) {
        if (s_mapping_table[i].poll_interval_ms == 0) {
            s_mapping_table[i].poll_interval_ms = POLL_INTERVAL_MS;
        }
        if (s_mapping_table[i].scale_factor == 0.0f) {
            s_mapping_table[i].scale_factor = 1.0f;
        }
        amm_normalize_read_window(&s_mapping_table[i]);
        if (s_mapping_table[i].mapping_version > s_model_version) {
            s_model_version = s_mapping_table[i].mapping_version;
        }
    }
    nvs_close(handle);

    ESP_LOGI(TAG, "Loaded %d mapping entries from NVS", s_mapping_count);
    return ESP_OK;
}
