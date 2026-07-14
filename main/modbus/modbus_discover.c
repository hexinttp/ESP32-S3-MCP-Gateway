/**
 * @file modbus_discover.c
 * @brief Automatic MODBUS device discovery and semantic inference.
 */

#include <string.h>
#include <stdio.h>
#include <math.h>

#include "modbus_discover.h"
#include "modbus_access.h"
#include "amm/amm_mapping.h"
#include "gateway_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "DISCOVER";

/* ======================== Internal State ======================== */

static discovered_device_t s_devices[DISCOVER_MAX_SLAVES];
static uint16_t            s_device_count = 0;
static discover_result_t   s_result;
static TaskHandle_t        s_scan_task;

/* ======================== Semantic Profile Database ======================== */

/**
 * Register-address-based semantic profiles.
 * Maps (address_range, value_range) → (name, unit, writable, min, max).
 */
typedef struct {
    uint16_t addr_lo;
    uint16_t addr_hi;
    const char *name;
    const char *unit;
    float range_min;
    float range_max;
    bool  writable;
} semantic_profile_t;

static const semantic_profile_t s_profiles[] = {
    /* Holding register 40001-40099: Temperature sensors */
    { 40001, 40099, "Temperature",     "degC",   -40.0f,  500.0f,  false },
    /* 40101-40199: Pressure */
    { 40101, 40199, "Pressure",        "bar",     0.0f,  100.0f,  false },
    /* 40201-40299: Flow rate */
    { 40201, 40299, "Flow rate",       "L/min",   0.0f, 5000.0f,  false },
    /* 40301-40399: Speed / frequency */
    { 40301, 40399, "Speed",           "rpm",     0.0f, 6000.0f,  true  },
    /* 40401-40499: Electrical (voltage/current/power) */
    { 40401, 40499, "Electrical",      "V",       0.0f,  600.0f,  false },
    /* 40501-40599: Humidity */
    { 40501, 40599, "Humidity",        "%RH",     0.0f,  100.0f,  false },
    /* 40601-40699: Level / position */
    { 40601, 40699, "Level",           "mm",      0.0f, 5000.0f,  false },
    /* 40701-40799: Counter / totalizer */
    { 40701, 40799, "Counter",         "pcs",     0.0f, 99999.0f, false },
    /* 40801-40899: Control setpoints */
    { 40801, 40899, "Setpoint",        "",       -999.0f, 999.0f, true  },
    /* 40901-40999: Status / digital I/O */
    { 40901, 40999, "Status",          "",        0.0f,  255.0f,  true  },
    /* Sentinel */
    { 0, 0, NULL, NULL, 0, 0, false }
};

/**
 * Value-based inference refinements — adjusts the profile-based guess
 * when the sample value falls in a distinctive range.
 */
typedef struct {
    float val_lo;
    float val_hi;
    const char *refined_name;
    const char *refined_unit;
} value_hint_t;

static const value_hint_t s_temp_hints[] = {
    { -40.0f,   0.0f, "Temperature (cold)",  "degC" },
    {   0.0f,  50.0f, "Ambient temperature", "degC" },
    {  50.0f, 150.0f, "Process temperature", "degC" },
    { 150.0f, 500.0f, "High temperature",    "degC" },
};

static const value_hint_t s_pressure_hints[] = {
    { 0.0f,  1.0f, "Vacuum pressure",  "bar" },
    { 1.0f, 10.0f, "Line pressure",    "bar" },
    { 10.0f, 100.0f, "High pressure",  "bar" },
};

/* ======================== Helper: Data Type Inference ======================== */

/**
 * Attempt to determine data type from a pair of raw registers.
 * Returns DT_FLOAT32 if the two registers form a valid IEEE 754 float
 * in a reasonable range; otherwise DT_UINT16 for single registers.
 */
static data_type_t infer_data_type(uint16_t *raw_regs, uint16_t reg_addr)
{
    /* Try FLOAT32 interpretation (2 registers, big-endian word order) */
    union { uint32_t u; float f; } conv;
    conv.u = ((uint32_t)raw_regs[0] << 16) | raw_regs[1];

    if (!isnan(conv.f) && isfinite(conv.f) &&
        fabsf(conv.f) > 0.001f && fabsf(conv.f) < 1e8f) {
        return DT_FLOAT32;
    }

    /* Check if value looks like signed 16-bit */
    int16_t sv = (int16_t)raw_regs[0];
    if (sv < 0) {
        return DT_INT16;
    }

    return DT_UINT16;
}

/* ======================== Semantic Inference ======================== */

void modbus_discover_infer_semantics(uint16_t reg_addr, float value,
                                     char *name, char *unit,
                                     float *range_min, float *range_max)
{
    const semantic_profile_t *match = NULL;

    for (int i = 0; s_profiles[i].name != NULL; i++) {
        if (reg_addr >= s_profiles[i].addr_lo &&
            reg_addr <= s_profiles[i].addr_hi) {
            match = &s_profiles[i];
            break;
        }
    }

    if (match) {
        snprintf(name, 32, "%s", match->name);
        snprintf(unit, 16, "%s", match->unit);
        *range_min = match->range_min;
        *range_max = match->range_max;

        /* Apply value-based refinement for known categories */
        if (strcmp(match->name, "Temperature") == 0) {
            for (int i = 0; i < (int)(sizeof(s_temp_hints)/sizeof(s_temp_hints[0])); i++) {
                if (value >= s_temp_hints[i].val_lo &&
                    value <  s_temp_hints[i].val_hi) {
                    snprintf(name, 32, "%s", s_temp_hints[i].refined_name);
                    snprintf(unit, 16, "%s", s_temp_hints[i].refined_unit);
                    break;
                }
            }
        } else if (strcmp(match->name, "Pressure") == 0) {
            for (int i = 0; i < (int)(sizeof(s_pressure_hints)/sizeof(s_pressure_hints[0])); i++) {
                if (value >= s_pressure_hints[i].val_lo &&
                    value <  s_pressure_hints[i].val_hi) {
                    snprintf(name, 32, "%s", s_pressure_hints[i].refined_name);
                    snprintf(unit, 16, "%s", s_pressure_hints[i].refined_unit);
                    break;
                }
            }
        }
    } else {
        /* Fallback: generic naming based on address */
        snprintf(name, 32, "Register_%u", reg_addr);
        unit[0] = '\0';
        *range_min = -32768.0f;
        *range_max =  65535.0f;
    }
}

/* ======================== Init / Reset ======================== */

void modbus_discover_init(void)
{
    memset(s_devices, 0, sizeof(s_devices));
    s_device_count = 0;
    memset(&s_result, 0, sizeof(s_result));
    ESP_LOGI(TAG, "Discovery module initialized");
}

void modbus_discover_reset(void)
{
    if (s_scan_task != NULL || s_result.scan_in_progress) {
        ESP_LOGW(TAG, "Cannot reset discovery while a scan is running");
        return;
    }
    modbus_discover_init();
    ESP_LOGI(TAG, "Discovery state reset");
}

/* ======================== Broadcast Scan ======================== */

static esp_err_t scan_bus_sync(uint8_t start, uint8_t end)
{
    if (start < 1) start = 1;
    if (end > 247) end = 247;
    if (start > end) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "Broadcast scan: slave IDs %u .. %u", start, end);
    s_result.scan_in_progress = true;
    s_result.total_scanned = end - start + 1;

    /* Reset device list */
    memset(s_devices, 0, sizeof(s_devices));
    s_device_count = 0;

    uint16_t raw_reg;
    for (uint8_t sid = start; sid <= end; sid++) {
        /* Probe with FC03, address 0, count 1 — quick liveness check */
        esp_err_t err = modbus_read_holding_register(sid, 0, 1, &raw_reg);
        if (err == ESP_OK) {
            if (s_device_count < DISCOVER_MAX_SLAVES) {
                discovered_device_t *dev = &s_devices[s_device_count];
                dev->slave_id = sid;
                snprintf(dev->device_id, sizeof(dev->device_id),
                         "device_slave_%02u", sid);
                dev->name[0] = '\0';  /* Empty — uses device_id as display name */
                dev->description[0] = '\0';
                snprintf(dev->mqtt_topic_prefix, sizeof(dev->mqtt_topic_prefix),
                         "factory/data/device_slave_%02u", sid);
                dev->active = true;
                dev->reg_count = 0;
                s_device_count++;

                ESP_LOGI(TAG, "  Found slave %u (responded to probe)", sid);
            }
        }
    }

    s_result.devices_found = s_device_count;
    s_result.scan_in_progress = false;
    s_result.scan_complete = true;

    ESP_LOGI(TAG, "Broadcast scan complete: %u devices found out of %u probed",
             s_device_count, s_result.total_scanned);
    return ESP_OK;
}

typedef struct { uint8_t start; uint8_t end; } bus_scan_request_t;
static bus_scan_request_t s_bus_request;
static discover_scan_params_t s_full_request;

static void bus_scan_task(void *argument)
{
    (void)argument;
    scan_bus_sync(s_bus_request.start, s_bus_request.end);
    s_scan_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t modbus_discover_scan_bus(uint8_t start, uint8_t end)
{
    if (s_scan_task != NULL || s_result.scan_in_progress) return ESP_ERR_INVALID_STATE;
    if (start < 1 || end > 247 || start > end) return ESP_ERR_INVALID_ARG;
    s_bus_request = (bus_scan_request_t){.start = start, .end = end};
    s_result.scan_in_progress = true;
    if (xTaskCreate(bus_scan_task, "mb_discover", 4096, NULL, 3, &s_scan_task) != pdPASS) {
        s_result.scan_in_progress = false;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

/* ======================== Device Register Scan ======================== */

esp_err_t modbus_discover_scan_device(uint8_t slave_id,
                                      uint16_t reg_start,
                                      uint16_t reg_end)
{
    /* Find or create device entry */
    discovered_device_t *dev = NULL;
    for (int i = 0; i < s_device_count; i++) {
        if (s_devices[i].slave_id == slave_id && s_devices[i].active) {
            dev = &s_devices[i];
            break;
        }
    }

    if (!dev) {
        /* Device not yet discovered — add it */
        if (s_device_count >= DISCOVER_MAX_SLAVES) {
            ESP_LOGE(TAG, "Device table full");
            return ESP_ERR_NO_MEM;
        }
        dev = &s_devices[s_device_count++];
        dev->slave_id = slave_id;
        snprintf(dev->device_id, sizeof(dev->device_id),
                 "device_slave_%02u", slave_id);
        dev->name[0] = '\0';
        dev->description[0] = '\0';
        snprintf(dev->mqtt_topic_prefix, sizeof(dev->mqtt_topic_prefix),
                 "factory/data/device_slave_%02u", slave_id);
        dev->active = true;
        dev->reg_count = 0;
    }

    ESP_LOGI(TAG, "Register scan: slave %u, addr %u .. %u",
             slave_id, reg_start, reg_end);

    uint16_t raw_regs[2];
    s_result.scan_in_progress = true;

    /* Scan with FC03 (holding registers) */
    for (uint16_t addr = reg_start; addr <= reg_end &&
         dev->reg_count < DISCOVER_MAX_REGS_PER_SLAVE; addr++) {

        /* Try single register first */
        esp_err_t err = modbus_read_holding_register(slave_id, addr - 40001, 1, raw_regs);
        if (err != ESP_OK) continue;

        discovered_register_t *reg = &dev->registers[dev->reg_count];
        reg->register_address = addr;
        reg->function_code = 0x03;
        reg->valid = true;

        /* Try reading 2 registers for FLOAT32 detection */
        uint16_t raw2[2] = {0};
        bool is_multi = false;
        if (addr + 1 <= reg_end) {
            esp_err_t err2 = modbus_read_holding_register(slave_id,
                                                           addr - 40001, 2, raw2);
            if (err2 == ESP_OK) {
                data_type_t dt = infer_data_type(raw2, addr);
                if (dt == DT_FLOAT32) {
                    reg->inferred_type = DT_FLOAT32;
                    reg->sample_value = modbus_convert_to_float(raw2, DT_FLOAT32);
                    is_multi = true;
                    addr++; /* Skip next register (part of 32-bit value) */
                }
            }
        }

        if (!is_multi) {
            reg->inferred_type = infer_data_type(raw_regs, addr);
            reg->sample_value = modbus_convert_to_float(raw_regs, reg->inferred_type);
        }

        /* Discovery is read-only. Write permission must be explicitly granted in AMM. */
        reg->writable = false;

        /* Semantic inference */
        float rmin, rmax;
        modbus_discover_infer_semantics(addr, reg->sample_value,
                                        reg->inferred_name, reg->inferred_unit,
                                        &rmin, &rmax);

        ESP_LOGI(TAG, "  [%u] %s = %.2f %s (%s, %s)",
                 addr, reg->inferred_name, reg->sample_value,
                 reg->inferred_unit,
                 reg->inferred_type == DT_FLOAT32 ? "FLOAT32" :
                 reg->inferred_type == DT_INT16   ? "INT16"   : "UINT16",
                 reg->writable ? "R/W" : "R/O");

        dev->reg_count++;
        s_result.registers_found++;
    }

    s_result.scan_in_progress = false;
    s_result.scan_complete = true;

    ESP_LOGI(TAG, "Device scan complete: slave %u has %u registers",
             slave_id, dev->reg_count);
    return ESP_OK;
}

/* ======================== Full Scan ======================== */

static esp_err_t full_scan_sync(const discover_scan_params_t *params)
{
    uint8_t  s_start = params ? params->slave_start : 1;
    uint8_t  s_end   = params ? params->slave_end   : 247;
    uint16_t r_start = params ? params->reg_start   : 40001;
    uint16_t r_end   = params ? params->reg_end     : 40100;

    /* Phase 1: Broadcast scan */
    esp_err_t err = scan_bus_sync(s_start, s_end);
    if (err != ESP_OK) return err;

    /* Phase 2: Register scan for each discovered device */
    for (int i = 0; i < s_device_count; i++) {
        if (s_devices[i].active) {
            modbus_discover_scan_device(s_devices[i].slave_id, r_start, r_end);
        }
    }

    ESP_LOGI(TAG, "Full scan complete: %u devices, %u registers",
             s_device_count, s_result.registers_found);
    return ESP_OK;
}

static void full_scan_task(void *argument)
{
    (void)argument;
    full_scan_sync(&s_full_request);
    s_result.scan_in_progress = false;
    s_result.scan_complete = true;
    s_scan_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t modbus_discover_full_scan(const discover_scan_params_t *params)
{
    if (s_scan_task != NULL || s_result.scan_in_progress) return ESP_ERR_INVALID_STATE;
    s_full_request = params != NULL ? *params : (discover_scan_params_t){
        .slave_start = 1, .slave_end = 247, .reg_start = 40001, .reg_end = 40100,
        .function_codes = {3, 4}, .fc_count = 2,
    };
    if (s_full_request.slave_start < 1 || s_full_request.slave_end > 247 ||
        s_full_request.slave_start > s_full_request.slave_end ||
        s_full_request.reg_start > s_full_request.reg_end) return ESP_ERR_INVALID_ARG;
    s_result.scan_in_progress = true;
    s_result.scan_complete = false;
    if (xTaskCreate(full_scan_task, "mb_full_scan", 6144, NULL, 3, &s_scan_task) != pdPASS) {
        s_result.scan_in_progress = false;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

/* ======================== Result Accessors ======================== */

discover_result_t modbus_discover_get_result(void)
{
    return s_result;
}

const discovered_device_t *modbus_discover_get_device(uint16_t index)
{
    if (index >= s_device_count) return NULL;
    return &s_devices[index];
}

uint16_t modbus_discover_get_device_count(void)
{
    return s_device_count;
}

/* ======================== Apply Mappings ======================== */

int modbus_discover_apply_mappings(void)
{
    int created = 0;

    for (int i = 0; i < s_device_count; i++) {
        const discovered_device_t *dev = &s_devices[i];
        if (!dev->active) continue;

        for (int j = 0; j < dev->reg_count; j++) {
            const discovered_register_t *reg = &dev->registers[j];
            if (!reg->valid) continue;

            /* Skip if mapping already exists for this (slave, addr) pair */
            if (amm_find_mapping(dev->slave_id, reg->register_address)) {
                ESP_LOGI(TAG, "Mapping already exists for slave %u / addr %u",
                         dev->slave_id, reg->register_address);
                continue;
            }

            amm_mapping_entry_t entry;
            memset(&entry, 0, sizeof(entry));

            entry.slave_id = dev->slave_id;
            entry.function_code = reg->function_code;
            entry.register_address = reg->register_address;
            entry.data_type = reg->inferred_type;
            entry.scale_factor = 1.0f;

            /* Device and point IDs */
            snprintf(entry.device_id, AMM_MAX_DEVICE_NAME_LEN, "%s",
                     dev->device_id);
            snprintf(entry.point_id, AMM_MAX_POINT_NAME_LEN, "%s",
                     reg->inferred_name);
            snprintf(entry.measurement_name, AMM_MAX_POINT_NAME_LEN, "%s",
                     reg->inferred_name);
            snprintf(entry.unit, AMM_MAX_UNIT_LEN, "%s", reg->inferred_unit);

            /* MQTT topic: factory/data/<device_id>/<point_id> */
            snprintf(entry.mqtt_topic, AMM_MAX_TOPIC_LEN,
                     "factory/data/%s/%s", dev->device_id, entry.point_id);

            /* Constraint from semantic inference */
            entry.constraint.writable = reg->writable;
            float rmin, rmax;
            modbus_discover_infer_semantics(reg->register_address,
                                            reg->sample_value,
                                            entry.point_id, entry.unit,
                                            &rmin, &rmax);
            entry.constraint.valid_range_min = rmin;
            entry.constraint.valid_range_max = rmax;

            entry.active = true;

            esp_err_t err = amm_add_mapping(&entry);
            if (err == ESP_OK) {
                created++;
                ESP_LOGI(TAG, "Created mapping: %s/%s @ slave %u addr %u",
                         entry.device_id, entry.point_id,
                         dev->slave_id, reg->register_address);
            } else {
                ESP_LOGW(TAG, "Failed to add mapping: %s", esp_err_to_name(err));
            }
        }
    }

    s_result.mappings_created += created;
    ESP_LOGI(TAG, "Applied %d new AMM mapping entries", created);
    return created;
}

/* ======================== Editing API ======================== */

discovered_device_t *modbus_discover_find_device(uint8_t slave_id)
{
    for (int i = 0; i < s_device_count; i++) {
        if (s_devices[i].slave_id == slave_id && s_devices[i].active) {
            return &s_devices[i];
        }
    }
    return NULL;
}

esp_err_t modbus_discover_update_device(uint8_t slave_id,
                                         const char *device_id,
                                         const char *name,
                                         const char *mqtt_topic_prefix)
{
    discovered_device_t *dev = modbus_discover_find_device(slave_id);
    if (!dev) return ESP_ERR_NOT_FOUND;

    if (device_id && device_id[0]) {
        strncpy(dev->device_id, device_id, sizeof(dev->device_id) - 1);
        dev->device_id[sizeof(dev->device_id) - 1] = '\0';
    }
    if (name) {
        strncpy(dev->name, name, sizeof(dev->name) - 1);
        dev->name[sizeof(dev->name) - 1] = '\0';
    }
    if (mqtt_topic_prefix) {
        strncpy(dev->mqtt_topic_prefix, mqtt_topic_prefix,
                sizeof(dev->mqtt_topic_prefix) - 1);
        dev->mqtt_topic_prefix[sizeof(dev->mqtt_topic_prefix) - 1] = '\0';
    }

    ESP_LOGI(TAG, "Device updated: slave %u, id='%s'", slave_id, dev->device_id);
    return ESP_OK;
}

discovered_register_t *modbus_discover_find_register(uint8_t slave_id,
                                                      uint16_t reg_addr)
{
    discovered_device_t *dev = modbus_discover_find_device(slave_id);
    if (!dev) return NULL;

    for (int j = 0; j < dev->reg_count; j++) {
        if (dev->registers[j].register_address == reg_addr) {
            return &dev->registers[j];
        }
    }
    return NULL;
}

esp_err_t modbus_discover_update_register(uint8_t slave_id,
                                           uint16_t reg_addr,
                                           const char *name,
                                           const char *unit,
                                           data_type_t dtype,
                                           bool writable,
                                           float range_min,
                                           float range_max)
{
    discovered_register_t *reg = modbus_discover_find_register(slave_id, reg_addr);
    if (!reg) return ESP_ERR_NOT_FOUND;

    if (name) {
        strncpy(reg->inferred_name, name, sizeof(reg->inferred_name) - 1);
        reg->inferred_name[sizeof(reg->inferred_name) - 1] = '\0';
    }
    if (unit) {
        strncpy(reg->inferred_unit, unit, sizeof(reg->inferred_unit) - 1);
        reg->inferred_unit[sizeof(reg->inferred_unit) - 1] = '\0';
    }
    reg->inferred_type = dtype;
    reg->writable = writable;
    /* range_min/max stored in AMM constraint, not in register struct currently;
       they are noted for future AMM update */
    (void)range_min;
    (void)range_max;

    ESP_LOGI(TAG, "Register updated: slave %u addr %u name='%s'",
             slave_id, reg_addr, reg->inferred_name);
    return ESP_OK;
}

esp_err_t modbus_discover_toggle_register(uint8_t slave_id,
                                           uint16_t reg_addr,
                                           bool *new_state)
{
    discovered_register_t *reg = modbus_discover_find_register(slave_id, reg_addr);
    if (!reg) return ESP_ERR_NOT_FOUND;

    reg->valid = !reg->valid;
    if (new_state) *new_state = reg->valid;

    ESP_LOGI(TAG, "Register toggled: slave %u addr %u -> %s",
             slave_id, reg_addr, reg->valid ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t modbus_discover_delete_register(uint8_t slave_id,
                                           uint16_t reg_addr)
{
    discovered_device_t *dev = modbus_discover_find_device(slave_id);
    if (!dev) return ESP_ERR_NOT_FOUND;

    for (int j = 0; j < dev->reg_count; j++) {
        if (dev->registers[j].register_address == reg_addr) {
            /* Shift remaining registers down */
            for (int k = j; k < dev->reg_count - 1; k++) {
                dev->registers[k] = dev->registers[k + 1];
            }
            dev->reg_count--;
            if (s_result.registers_found > 0) s_result.registers_found--;

            ESP_LOGI(TAG, "Register deleted: slave %u addr %u (remaining: %u)",
                     slave_id, reg_addr, dev->reg_count);
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}
