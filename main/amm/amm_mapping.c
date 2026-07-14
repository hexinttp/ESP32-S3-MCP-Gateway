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

#include "amm/amm_mapping.h"
#include "gateway_config.h"
#include "tcm/tcm_context.h"

static const char *TAG = "AMM";

/* ======================== Private State ======================== */

/** Internal mapping table (static allocation – no heap dependency). */
static amm_mapping_entry_t s_mapping_table[AMM_MAX_MAPPING_ENTRIES];

/** Number of entries currently loaded (active + inactive slots). */
static int s_mapping_count = 0;

/** Mutex guarding all reads/writes to the mapping table. */
static SemaphoreHandle_t s_amm_mutex = NULL;

/** Flag indicating whether amm_init() has completed. */
static bool s_amm_initialized = false;
static uint32_t s_model_version = 1;

/* Forward declaration – used by add/remove before its definition below. */
static esp_err_t amm_save_to_nvs_unlocked(void);

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

/* ======================== Default Entries ======================== */

/**
 * @brief Populate the mapping table with built-in default entries.
 *
 * Called by amm_init() when no NVS data is available, giving the gateway
 * a known-good starting configuration for four representative points.
 */
static void amm_populate_defaults(void)
{
    memset(s_mapping_table, 0, sizeof(s_mapping_table));
    s_mapping_count = 0;

    /* Entry 0: Motor temperature – read-only float32 */
    {
        amm_mapping_entry_t *e = &s_mapping_table[s_mapping_count];
        e->slave_id          = 1;
        e->function_code     = 3;   /* Read Holding Registers */
        e->register_address  = 40001;
        e->data_type         = DT_FLOAT32;
        e->scale_factor      = 1.0f;
        strncpy(e->device_id,        "plc_line1_01",        AMM_MAX_DEVICE_NAME_LEN - 1);
        strncpy(e->point_id,         "motor_temp_01",       AMM_MAX_POINT_NAME_LEN  - 1);
        strncpy(e->measurement_name, "Motor temperature",   AMM_MAX_POINT_NAME_LEN  - 1);
        strncpy(e->unit,             "degC",                AMM_MAX_UNIT_LEN        - 1);
        strncpy(e->mqtt_topic,       "factory/line1/plc01/motor/temp", AMM_MAX_TOPIC_LEN - 1);
        e->constraint.writable          = false;
        e->constraint.valid_range_min   = 0.0f;
        e->constraint.valid_range_max   = 120.0f;
        e->active = true;
        s_mapping_count++;
    }

    /* Entry 1: Line pressure – read-only float32 */
    {
        amm_mapping_entry_t *e = &s_mapping_table[s_mapping_count];
        e->slave_id          = 1;
        e->function_code     = 3;
        e->register_address  = 40003;
        e->data_type         = DT_FLOAT32;
        e->scale_factor      = 1.0f;
        strncpy(e->device_id,        "plc_line1_01",   AMM_MAX_DEVICE_NAME_LEN - 1);
        strncpy(e->point_id,         "pressure_01",    AMM_MAX_POINT_NAME_LEN  - 1);
        strncpy(e->measurement_name, "Line pressure",  AMM_MAX_POINT_NAME_LEN  - 1);
        strncpy(e->unit,             "bar",            AMM_MAX_UNIT_LEN        - 1);
        strncpy(e->mqtt_topic,       "factory/line1/plc01/pressure", AMM_MAX_TOPIC_LEN - 1);
        e->constraint.writable          = false;
        e->constraint.valid_range_min   = 0.0f;
        e->constraint.valid_range_max   = 10.0f;
        e->active = true;
        s_mapping_count++;
    }

    /* Entry 2: Conveyor speed – writable uint16 */
    {
        amm_mapping_entry_t *e = &s_mapping_table[s_mapping_count];
        e->slave_id          = 2;
        e->function_code     = 3;
        e->register_address  = 40001;
        e->data_type         = DT_UINT16;
        e->scale_factor      = 1.0f;
        strncpy(e->device_id,        "plc_line2_01",   AMM_MAX_DEVICE_NAME_LEN - 1);
        strncpy(e->point_id,         "speed_01",       AMM_MAX_POINT_NAME_LEN  - 1);
        strncpy(e->measurement_name, "Conveyor speed", AMM_MAX_POINT_NAME_LEN  - 1);
        strncpy(e->unit,             "rpm",            AMM_MAX_UNIT_LEN        - 1);
        strncpy(e->mqtt_topic,       "factory/line2/plc01/speed", AMM_MAX_TOPIC_LEN - 1);
        e->constraint.writable          = true;
        e->constraint.valid_range_min   = 0.0f;
        e->constraint.valid_range_max   = 3000.0f;
        e->active = true;
        s_mapping_count++;
    }

    /* Entry 3: Motor current – read-only int16 */
    {
        amm_mapping_entry_t *e = &s_mapping_table[s_mapping_count];
        e->slave_id          = 2;
        e->function_code     = 3;
        e->register_address  = 40003;
        e->data_type         = DT_INT16;
        e->scale_factor      = 1.0f;
        strncpy(e->device_id,        "plc_line2_01",  AMM_MAX_DEVICE_NAME_LEN - 1);
        strncpy(e->point_id,         "current_01",    AMM_MAX_POINT_NAME_LEN  - 1);
        strncpy(e->measurement_name, "Motor current", AMM_MAX_POINT_NAME_LEN  - 1);
        strncpy(e->unit,             "A",             AMM_MAX_UNIT_LEN        - 1);
        strncpy(e->mqtt_topic,       "factory/line2/plc01/current", AMM_MAX_TOPIC_LEN - 1);
        e->constraint.writable          = false;
        e->constraint.valid_range_min   = -50.0f;
        e->constraint.valid_range_max   = 50.0f;
        e->active = true;
        s_mapping_count++;
    }

    for (int i = 0; i < s_mapping_count; ++i) {
        s_mapping_table[i].source_protocol = SRC_MODBUS_RTU;
        s_mapping_table[i].channel_id = 0;
        s_mapping_table[i].byte_order = BYTE_ORDER_ABCD;
        s_mapping_table[i].poll_interval_ms = POLL_INTERVAL_MS;
        s_mapping_table[i].priority = 5;
        s_mapping_table[i].mapping_version = (uint32_t)i + 1;
    }
    s_model_version = (uint32_t)s_mapping_count;
    ESP_LOGI(TAG, "Populated %d default mapping entries", s_mapping_count);
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

    memset(s_mapping_table, 0, sizeof(s_mapping_table));
    s_mapping_count = 0;

    /* Try to load persisted mappings from NVS */
    esp_err_t ret = amm_load_from_nvs();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "NVS load failed (%s) – using built-in defaults",
                 esp_err_to_name(ret));
        amm_populate_defaults();

        /* Persist the defaults so they survive a reboot */
        esp_err_t save_ret = amm_save_to_nvs();
        if (save_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to persist default mappings to NVS: %s",
                     esp_err_to_name(save_ret));
        }
    } else {
        ESP_LOGI(TAG, "Loaded %d mapping entries from NVS", s_mapping_count);
    }

    s_amm_initialized = true;
    ESP_LOGI(TAG, "AMM initialized (%d active entries)", amm_get_mapping_count());
}

esp_err_t amm_add_mapping(const amm_mapping_entry_t *entry)
{
    if (!entry) {
        ESP_LOGE(TAG, "amm_add_mapping: entry is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    amm_lock();

    if (s_mapping_count >= AMM_MAX_MAPPING_ENTRIES) {
        amm_unlock();
        ESP_LOGE(TAG, "amm_add_mapping: table full (%d/%d)",
                 s_mapping_count, AMM_MAX_MAPPING_ENTRIES);
        return ESP_ERR_NO_MEM;
    }

    /*
     * Check for duplicate (slave_id, register_address) among active entries.
     * If one exists, overwrite it in place rather than consuming a new slot.
     */
    for (int i = 0; i < s_mapping_count; i++) {
        if (s_mapping_table[i].active &&
            s_mapping_table[i].source_protocol == entry->source_protocol &&
            s_mapping_table[i].channel_id == entry->channel_id &&
            s_mapping_table[i].slave_id == entry->slave_id &&
            s_mapping_table[i].register_address == entry->register_address) {
            ESP_LOGW(TAG, "amm_add_mapping: overwriting existing entry "
                     "slave=%u reg=%u", entry->slave_id, entry->register_address);
            memcpy(&s_mapping_table[i], entry, sizeof(amm_mapping_entry_t));
            s_mapping_table[i].active = true;
            s_mapping_table[i].mapping_version = ++s_model_version;
            esp_err_t ret = amm_save_to_nvs_unlocked();
            amm_unlock();
            return ret;
        }
    }

    /* Append to the end of the table */
    memcpy(&s_mapping_table[s_mapping_count], entry, sizeof(amm_mapping_entry_t));
    s_mapping_table[s_mapping_count].active = true;
    s_mapping_table[s_mapping_count].mapping_version = ++s_model_version;
    if (s_mapping_table[s_mapping_count].poll_interval_ms == 0) {
        s_mapping_table[s_mapping_count].poll_interval_ms = POLL_INTERVAL_MS;
    }
    s_mapping_count++;

    esp_err_t ret = amm_save_to_nvs_unlocked();
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

            s_mapping_table[i].active = false;
            ESP_LOGI(TAG, "Deactivated mapping: slave=%u reg=%u", slave_id, reg_addr);

            esp_err_t ret = amm_save_to_nvs_unlocked();
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
            entry->active = false;
            ++s_model_version;
            esp_err_t err = amm_save_to_nvs_unlocked();
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

esp_err_t amm_update_mapping(int index, const amm_mapping_entry_t *entry)
{
    if (entry == NULL || index < 0 || index >= AMM_MAX_MAPPING_ENTRIES) return ESP_ERR_INVALID_ARG;
    amm_lock();
    if (index >= s_mapping_count || !s_mapping_table[index].active) {
        amm_unlock();
        return ESP_ERR_NOT_FOUND;
    }
    s_mapping_table[index] = *entry;
    s_mapping_table[index].active = true;
    s_mapping_table[index].mapping_version = ++s_model_version;
    if (s_mapping_table[index].poll_interval_ms == 0) {
        s_mapping_table[index].poll_interval_ms = POLL_INTERVAL_MS;
    }
    esp_err_t err = amm_save_to_nvs_unlocked();
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
    if (amm_find_mapping_for_channel(ctx->source_protocol, ctx->channel_id,
                                     ctx->slave_id, ctx->register_address,
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
    if (cmd_ctx->function_code != 6 && cmd_ctx->function_code != 16) {
        result.accepted = false;
        snprintf(result.reject_reason, sizeof(result.reject_reason),
                 "Invalid write function code %u (expected 6 or 16)",
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

/**
 * @brief Internal save helper – caller must already hold s_amm_mutex.
 *
 * Serialises each mapping entry to NVS as a binary blob, together with the
 * total slot count.
 */
static esp_err_t amm_save_to_nvs_unlocked(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(AMM_NVS_NAMESPACE, NVS_READWRITE, &handle);
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
    esp_err_t ret = nvs_open(AMM_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "NVS namespace '%s' not found: %s",
                 AMM_NVS_NAMESPACE, esp_err_to_name(ret));
        return ret;
    }

    int32_t stored_count = 0;
    ret = nvs_get_i32(handle, AMM_NVS_KEY_COUNT, &stored_count);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "NVS key '%s' not found: %s",
                 AMM_NVS_KEY_COUNT, esp_err_to_name(ret));
        nvs_close(handle);
        return (ret == ESP_ERR_NVS_NOT_FOUND) ? ESP_ERR_NOT_FOUND : ret;
    }

    if (stored_count < 0 || stored_count > AMM_MAX_MAPPING_ENTRIES) {
        ESP_LOGE(TAG, "NVS stored count %ld is out of range [0, %d]",
                 (long)stored_count, AMM_MAX_MAPPING_ENTRIES);
        nvs_close(handle);
        return ESP_ERR_INVALID_SIZE;
    }

    char key[20];
    memset(s_mapping_table, 0, sizeof(s_mapping_table));

    for (int i = 0; i < stored_count; i++) {
        amm_nvs_entry_key(i, key, sizeof(key));

        size_t blob_len = sizeof(amm_mapping_entry_t);
        ret = nvs_get_blob(handle, key, &s_mapping_table[i], &blob_len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "NVS get blob[%d] failed: %s", i, esp_err_to_name(ret));
            memset(s_mapping_table, 0, sizeof(s_mapping_table));
            s_mapping_count = 0;
            nvs_close(handle);
            return ret;
        }

        if (blob_len != sizeof(amm_mapping_entry_t)) {
            ESP_LOGE(TAG, "NVS blob[%d] size mismatch: got %u expected %u",
                     i, (unsigned)blob_len, (unsigned)sizeof(amm_mapping_entry_t));
            memset(s_mapping_table, 0, sizeof(s_mapping_table));
            s_mapping_count = 0;
            nvs_close(handle);
            return ESP_ERR_INVALID_SIZE;
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
        if (s_mapping_table[i].mapping_version > s_model_version) {
            s_model_version = s_mapping_table[i].mapping_version;
        }
    }
    nvs_close(handle);

    ESP_LOGI(TAG, "Loaded %d mapping entries from NVS", s_mapping_count);
    return ESP_OK;
}
