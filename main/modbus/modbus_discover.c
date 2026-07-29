/**
 * @file modbus_discover.c
 * @brief Automatic MODBUS device discovery and semantic inference.
 */

#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <stdlib.h>

#include "modbus_discover.h"
#include "modbus_access.h"
#include "amm/amm_mapping.h"
#include "semantic/semantic_inference.h"
#include "scheduler/scheduler.h"
#include "gateway_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_psram.h"
#include "cJSON.h"

static const char *TAG = "DISCOVER";

extern const uint8_t lab_profile_json_start[] asm("_binary_lab_profile_json_start");
extern const uint8_t lab_profile_json_end[] asm("_binary_lab_profile_json_end");

/* ======================== Internal State ======================== */

static discovered_device_t *s_devices = NULL;
static uint16_t            s_device_capacity = 0;
static uint16_t            s_device_count = 0;
static discover_result_t   s_result;
static discover_apply_result_t s_apply_result;
static TaskHandle_t        s_scan_task;
static portMUX_TYPE         s_result_lock = portMUX_INITIALIZER_UNLOCKED;

static void discover_set_task_state(bool in_progress, bool complete,
                                    discover_phase_t phase, esp_err_t error)
{
    taskENTER_CRITICAL(&s_result_lock);
    s_result.scan_in_progress = in_progress;
    s_result.scan_complete = complete;
    s_result.phase = phase;
    s_result.last_error = error;
    taskEXIT_CRITICAL(&s_result_lock);
}

static void discover_set_bus_progress(uint8_t slave_id, uint16_t completed)
{
    taskENTER_CRITICAL(&s_result_lock);
    s_result.current_slave = slave_id;
    s_result.slaves_scanned = completed;
    s_result.devices_found = s_device_count;
    s_result.slaves_skipped =
        completed >= s_device_count ? completed - s_device_count : 0;
    taskEXIT_CRITICAL(&s_result_lock);
}

static void discover_set_register_progress(uint8_t slave_id, uint8_t function_code,
                                           uint16_t address)
{
    taskENTER_CRITICAL(&s_result_lock);
    s_result.phase = DISCOVER_PHASE_REGISTER_SCAN;
    s_result.current_slave = slave_id;
    s_result.current_function_code = function_code;
    s_result.current_register = address;
    taskEXIT_CRITICAL(&s_result_lock);
}

static uint16_t discover_register_offset(uint8_t function_code, uint16_t configured_address)
{
    return modbus_register_offset(function_code, configured_address);
}

static esp_err_t discover_read_registers(source_protocol_t protocol, uint8_t channel_id,
                                         uint8_t slave_id, uint8_t function_code,
                                         uint16_t configured_address,
                                         uint16_t reg_count, uint16_t *raw_regs)
{
    uint16_t offset = discover_register_offset(function_code, configured_address);
    return modbus_read_channel(protocol, channel_id, slave_id, function_code,
                               offset, reg_count, raw_regs);
}

/* Defined in the apply section; used by the profile-guided probe. */
static data_type_t profile_data_type(const char *name);
static uint8_t profile_type_width(data_type_t type);
static bool discovered_has_register(const discovered_device_t *device,
                                    uint16_t address, uint8_t function_code);

/*
 * Append one discovered register word to a device, including the
 * address-range-based semantic guess and suggested engineering range.
 * `origin` distinguishes linear-scan hits from profile-guided probes in logs.
 */
static void discover_append_register(discovered_device_t *dev,
                                     uint8_t function_code,
                                     uint16_t point_address,
                                     uint16_t raw_value,
                                     uint16_t read_start,
                                     uint8_t read_count,
                                     uint8_t value_index,
                                     const char *origin)
{
    discovered_register_t *reg = &dev->registers[dev->reg_count++];
    memset(reg, 0, sizeof(*reg));
    reg->register_address = point_address;
    reg->function_code = function_code;
    reg->raw_value = raw_value;
    reg->inferred_type = function_code <= 2 ? DT_BOOL :
        (((int16_t)raw_value < 0) ? DT_INT16 : DT_UINT16);
    reg->sample_value = reg->inferred_type == DT_INT16 ?
                        (float)(int16_t)raw_value : (float)raw_value;
    reg->read_start_address = read_start;
    reg->read_register_count = read_count;
    reg->value_register_index = value_index;
    reg->writable = false;
    reg->valid = true;
    modbus_discover_infer_semantics(point_address, reg->sample_value,
                                    reg->inferred_name, reg->inferred_unit,
                                    &reg->range_min, &reg->range_max);
    ESP_LOGI(TAG, "  FC%02u [%u] raw=%u window=%u+%u index=%u%s",
             function_code, point_address, raw_value, read_start, read_count,
             value_index, origin ? origin : "");
    taskENTER_CRITICAL(&s_result_lock);
    ++s_result.registers_found;
    taskEXIT_CRITICAL(&s_result_lock);
}

static void discover_default_functions(discover_scan_params_t *params)
{
    if (params->fc_count == 0) {
        params->function_codes[0] = 1;
        params->function_codes[1] = 2;
        params->function_codes[2] = 3;
        params->function_codes[3] = 4;
        params->fc_count = 4;
    }
    if (params->fc_count > 4) {
        params->fc_count = 4;
    }
    for (uint8_t i = 0; i < params->fc_count; ++i) {
        if (params->function_codes[i] < 1 || params->function_codes[i] > 4) {
            params->function_codes[i] = (uint8_t)(i + 1);
        }
    }
    if (params->max_empty_gap == 0) {
        params->max_empty_gap = DISCOVER_DEFAULT_EMPTY_GAP;
    }
}

typedef struct {
    uint8_t function_code;
    uint16_t address;
    uint8_t register_count;
    uint16_t values[DISCOVER_MAX_BLOCK_REGS];
} discover_probe_t;

static uint8_t append_unique_u8(uint8_t *values, uint8_t count, uint8_t value)
{
    for (uint8_t i = 0; i < count; ++i) {
        if (values[i] == value) return count;
    }
    values[count++] = value;
    return count;
}

static uint8_t append_unique_u16(uint16_t *values, uint8_t count, uint16_t value)
{
    for (uint8_t i = 0; i < count; ++i) {
        if (values[i] == value) return count;
    }
    values[count++] = value;
    return count;
}

static uint8_t build_count_order(uint8_t preferred, uint8_t *counts)
{
    uint8_t count = 0;
    if (preferred > 0 && preferred <= DISCOVER_MAX_BLOCK_REGS) {
        count = append_unique_u8(counts, count, preferred);
    }
    count = append_unique_u8(counts, count, 1);
    count = append_unique_u8(counts, count, 2);
    count = append_unique_u8(counts, count, 4);
    count = append_unique_u8(counts, count, 8);
    return count;
}

static uint8_t build_function_order(const discover_scan_params_t *params,
                                    uint8_t preferred, uint8_t *functions)
{
    uint8_t count = 0;
    if (preferred >= 1 && preferred <= 4) {
        count = append_unique_u8(functions, count, preferred);
    }
    for (uint8_t i = 0; i < params->fc_count; ++i) {
        count = append_unique_u8(functions, count, params->function_codes[i]);
    }
    return count;
}

static esp_err_t discover_probe_address(const discover_scan_params_t *params,
                                        uint8_t slave_id, uint16_t address,
                                        uint8_t preferred_function,
                                        uint8_t preferred_count,
                                        discover_probe_t *probe)
{
    uint8_t counts[5];
    uint8_t functions[4];
    uint8_t count_count = build_count_order(preferred_count, counts);
    uint8_t function_count = build_function_order(params, preferred_function, functions);

    for (uint8_t ci = 0; ci < count_count; ++ci) {
        uint8_t reg_count = counts[ci];
        if ((uint32_t)address + reg_count > 65536U) continue;
        for (uint8_t fi = 0; fi < function_count; ++fi) {
            memset(probe, 0, sizeof(*probe));
            esp_err_t err = discover_read_registers(params->source_protocol,
                                                    params->channel_id,
                                                    slave_id, functions[fi],
                                                    address, reg_count,
                                                    probe->values);
            if (err == ESP_OK) {
                probe->function_code = functions[fi];
                probe->address = address;
                probe->register_count = reg_count;
                return ESP_OK;
            }
        }
    }
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t discover_probe_device(const discover_scan_params_t *params,
                                       uint8_t slave_id, discover_probe_t *probe)
{
    static const uint16_t common_register_entries[] = {
        40001, /* Generic holding register offset 0 */
        40002, /* Common meters, including PAC3200 */
        43000, /* Schneider iEM series */
        43202, /* Variable-frequency drives */
        48193, /* Omron E5 series */
        59001, /* Power-quality meters */
        30001, /* Generic input register offset 0 */
    };
    uint16_t addresses[12];
    uint8_t address_count = 0;
    address_count = append_unique_u16(addresses, address_count, params->reg_start);
    address_count = append_unique_u16(addresses, address_count, 1);
    address_count = append_unique_u16(addresses, address_count, 0);
    for (size_t i = 0;
         i < sizeof(common_register_entries) / sizeof(common_register_entries[0]);
         ++i) {
        address_count = append_unique_u16(
            addresses, address_count, common_register_entries[i]);
    }

    uint8_t functions[4];
    uint8_t function_count = build_function_order(params, 0, functions);
    uint8_t invalid_responses = 0;

    for (uint8_t i = 0; i < address_count; ++i) {
        for (uint8_t fi = 0; fi < function_count; ++fi) {
            memset(probe, 0, sizeof(*probe));
            esp_err_t err = discover_read_registers(
                params->source_protocol, params->channel_id, slave_id,
                functions[fi], addresses[i], 1, probe->values);
            if (err == ESP_OK) {
                probe->function_code = functions[fi];
                probe->address = addresses[i];
                probe->register_count = 1;
                return ESP_OK;
            }
            if (err == ESP_ERR_INVALID_RESPONSE) {
                ++invalid_responses;
            }
        }

        /*
         * An absent RTU slave times out for both functions. Avoid probing every
         * profile address in that case. A present device normally returns data
         * or a Modbus exception (reported by esp-modbus as INVALID_RESPONSE).
         */
        if (i == 0 && invalid_responses == 0) {
            return ESP_ERR_NOT_FOUND;
        }
    }

    /*
     * On RTU, an exception proves that a slave addressed the request because
     * an absent slave stays silent. A Modbus TCP gateway can return exception
     * 0x0B for any unavailable Unit ID, so exceptions alone must not create
     * TCP discovery entries.
     */
    if (params->source_protocol == SRC_MODBUS_RTU &&
        invalid_responses >= 2) {
        memset(probe, 0, sizeof(*probe));
        probe->function_code = functions[0];
        probe->address = params->reg_start;
        probe->register_count = 0;
        return ESP_OK;
    }
    return ESP_ERR_NOT_FOUND;
}

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
    if (s_devices == NULL) {
        if (esp_psram_is_initialized()) {
            s_devices = heap_caps_calloc(
                DISCOVER_MAX_SLAVES, sizeof(*s_devices),
                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (s_devices != NULL) {
                s_device_capacity = DISCOVER_MAX_SLAVES;
                ESP_LOGI(TAG, "Allocated %u-device discovery table in PSRAM (%u bytes)",
                         s_device_capacity,
                         (unsigned)(s_device_capacity * sizeof(*s_devices)));
            }
        }

        if (s_devices == NULL) {
            s_devices = heap_caps_calloc(
                DISCOVER_FALLBACK_SLAVES, sizeof(*s_devices),
                MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            if (s_devices != NULL) {
                s_device_capacity = DISCOVER_FALLBACK_SLAVES;
                ESP_LOGW(TAG,
                         "PSRAM unavailable; discovery capacity reduced to %u devices",
                         s_device_capacity);
            }
        }
    }

    if (s_devices != NULL) {
        memset(s_devices, 0,
               (size_t)s_device_capacity * sizeof(*s_devices));
    }
    s_device_count = 0;
    taskENTER_CRITICAL(&s_result_lock);
    memset(&s_result, 0, sizeof(s_result));
    s_result.device_capacity = s_device_capacity;
    s_result.phase = DISCOVER_PHASE_IDLE;
    s_result.last_error = s_devices != NULL ? ESP_OK : ESP_ERR_NO_MEM;
    taskEXIT_CRITICAL(&s_result_lock);
    ESP_LOGI(TAG, "Discovery module initialized (capacity=%u)",
             s_device_capacity);
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

static esp_err_t scan_bus_sync(const discover_scan_params_t *params)
{
    if (s_devices == NULL || s_device_capacity == 0) {
        return ESP_ERR_NO_MEM;
    }
    uint8_t start = params->slave_start;
    uint8_t end = params->slave_end;
    if (start < 1) start = 1;
    if (end > 247) end = 247;
    if (start > end) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "%s scan: channel=%u slave IDs %u .. %u",
             params->source_protocol == SRC_MODBUS_TCP ? "TCP" : "RTU",
             params->channel_id, start, end);
    taskENTER_CRITICAL(&s_result_lock);
    s_result.phase = DISCOVER_PHASE_BUS_SCAN;
    s_result.total_scanned = end - start + 1;
    s_result.slaves_scanned = 0;
    s_result.current_slave = start;
    taskEXIT_CRITICAL(&s_result_lock);

    /* Reset device list */
    memset(s_devices, 0,
           (size_t)s_device_capacity * sizeof(*s_devices));
    s_device_count = 0;

    for (uint16_t sid = start; sid <= end; ++sid) {
        discover_probe_t probe;
        /* Probe with FC03, address 0, count 1 — quick liveness check */
        esp_err_t err = discover_probe_device(params, (uint8_t)sid, &probe);
        if (err == ESP_OK) {
            if (s_device_count < s_device_capacity) {
                discovered_device_t *dev = &s_devices[s_device_count];
                dev->slave_id = (uint8_t)sid;
                dev->source_protocol = params->source_protocol;
                dev->channel_id = params->channel_id;
                snprintf(dev->device_id, sizeof(dev->device_id),
                         "%s_ch%u_slave_%02u",
                         params->source_protocol == SRC_MODBUS_TCP ? "tcp" : "rtu",
                         params->channel_id, (unsigned)sid);
                dev->name[0] = '\0';  /* Empty — uses device_id as display name */
                dev->description[0] = '\0';
                snprintf(dev->mqtt_topic_prefix, sizeof(dev->mqtt_topic_prefix),
                         "factory/data/%s_ch%u_slave_%02u",
                         params->source_protocol == SRC_MODBUS_TCP ? "tcp" : "rtu",
                         params->channel_id, (unsigned)sid);
                dev->probe_function_code = probe.function_code;
                dev->probe_address = probe.address;
                dev->probe_register_count = probe.register_count;
                modbus_device_identity_t identity;
                if (modbus_read_device_identity_channel(
                        dev->source_protocol, dev->channel_id, dev->slave_id,
                        &identity) == ESP_OK) {
                    strlcpy(dev->vendor_name, identity.vendor_name,
                            sizeof(dev->vendor_name));
                    strlcpy(dev->product_code, identity.product_code,
                            sizeof(dev->product_code));
                    strlcpy(dev->revision, identity.revision,
                            sizeof(dev->revision));
                }
                dev->active = true;
                dev->reg_count = 0;
                s_device_count++;

                ESP_LOGI(TAG, "  Found slave %u: FC%02u addr=%u count=%u",
                         (unsigned)sid, probe.function_code, probe.address,
                         probe.register_count);
            }
        }
        discover_set_bus_progress((uint8_t)sid, (uint16_t)(sid - start + 1));
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    ESP_LOGI(TAG, "Bus scan complete: %u devices found out of %u probed",
             s_device_count, end - start + 1);
    return ESP_OK;
}

typedef struct {
    source_protocol_t protocol;
    uint8_t channel_id;
    uint8_t start;
    uint8_t end;
} bus_scan_request_t;
static bus_scan_request_t s_bus_request;
static discover_scan_params_t s_full_request;

static void bus_scan_task(void *argument)
{
    (void)argument;
    discover_scan_params_t params = {
        .slave_start = s_bus_request.start,
        .slave_end = s_bus_request.end,
        .reg_start = 0,
        .reg_end = 100,
        .source_protocol = s_bus_request.protocol,
        .channel_id = s_bus_request.channel_id,
        .function_codes = {1, 2, 3, 4},
        .fc_count = 4,
        .max_empty_gap = DISCOVER_DEFAULT_EMPTY_GAP,
    };
    scheduler_pause_modbus_polling(true);
    modbus_access_set_probe_mode(true);
    esp_log_level_set("MB_CONTROLLER_MASTER", ESP_LOG_NONE);
    esp_err_t err = scan_bus_sync(&params);
    esp_log_level_set("MB_CONTROLLER_MASTER", ESP_LOG_ERROR);
    modbus_access_set_probe_mode(false);
    scheduler_pause_modbus_polling(false);
    discover_set_task_state(false, true,
                            err == ESP_OK ? DISCOVER_PHASE_COMPLETE : DISCOVER_PHASE_ERROR,
                            err);
    s_scan_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t modbus_discover_scan_bus(uint8_t start, uint8_t end)
{
    if (s_scan_task != NULL || s_result.scan_in_progress) return ESP_ERR_INVALID_STATE;
    if (start < 1 || end > 247 || start > end) return ESP_ERR_INVALID_ARG;
    s_bus_request = (bus_scan_request_t){
        .protocol = SRC_MODBUS_RTU,
        .channel_id = 0,
        .start = start,
        .end = end
    };
    discover_set_task_state(true, false, DISCOVER_PHASE_BUS_SCAN, ESP_OK);
    if (xTaskCreate(bus_scan_task, "mb_discover", 4096, NULL, 3, &s_scan_task) != pdPASS) {
        discover_set_task_state(false, true, DISCOVER_PHASE_ERROR, ESP_ERR_NO_MEM);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

/* ======================== Device Register Scan ======================== */

esp_err_t modbus_discover_scan_device(uint8_t slave_id,
                                      uint16_t reg_start,
                                      uint16_t reg_end)
{
    /* Phase 2 is only allowed for a slave accepted by the liveness scan. */
    discovered_device_t *dev = NULL;
    for (int i = 0; i < s_device_count; i++) {
        if (s_devices[i].slave_id == slave_id &&
            s_devices[i].source_protocol == s_full_request.source_protocol &&
            s_devices[i].channel_id == s_full_request.channel_id &&
            s_devices[i].active) {
            dev = &s_devices[i];
            break;
        }
    }

    if (!dev) {
        ESP_LOGD(TAG, "Skip register scan for unknown/offline slave %u", slave_id);
        return ESP_ERR_NOT_FOUND;
    }

    /*
     * The device may disappear between the bus scan and this phase. Reuse the
     * successful probe first; only a second positive liveness result permits
     * the more expensive address-range scan.
     */
    discover_probe_t verification;
    esp_err_t verify_err;
    if (dev->probe_register_count > 0) {
        verify_err = discover_probe_address(
            &s_full_request, slave_id, dev->probe_address,
            dev->probe_function_code, dev->probe_register_count, &verification);
    } else {
        verify_err = discover_probe_device(&s_full_request, slave_id, &verification);
    }
    if (verify_err != ESP_OK) {
        dev->active = false;
        dev->reg_count = 0;
        ESP_LOGW(TAG,
                 "Slave %u stopped responding after discovery; register scan skipped",
                 slave_id);
        return ESP_ERR_NOT_FOUND;
    }
    dev->probe_function_code = verification.function_code;
    dev->probe_address = verification.address;
    dev->probe_register_count = verification.register_count;

    ESP_LOGI(TAG, "Register scan: %s channel=%u slave %u, addr %u .. %u",
             dev->source_protocol == SRC_MODBUS_TCP ? "TCP" : "RTU",
             dev->channel_id, slave_id, reg_start, reg_end);

    discover_default_functions(&s_full_request);
    dev->reg_count = 0;

    uint32_t address = reg_start;
    uint8_t empty_gap = 0;
    while (address <= reg_end &&
           dev->reg_count < DISCOVER_MAX_REGS_PER_SLAVE) {
        discover_probe_t probe;
        esp_err_t err;
        uint8_t preferred_count = dev->probe_register_count;
        uint8_t remaining = (uint8_t)((reg_end - address + 1U) >
                            DISCOVER_MAX_BLOCK_REGS ? DISCOVER_MAX_BLOCK_REGS :
                            (reg_end - address + 1U));

        discover_set_register_progress(slave_id, dev->probe_function_code,
                                       (uint16_t)address);
        if (preferred_count > remaining) preferred_count = remaining;
        if (dev->reg_count > 0 && preferred_count > 0) {
            memset(&probe, 0, sizeof(probe));
            err = discover_read_registers(dev->source_protocol, dev->channel_id,
                                          slave_id, dev->probe_function_code,
                                          (uint16_t)address, preferred_count,
                                          probe.values);
            if (err == ESP_OK) {
                probe.function_code = dev->probe_function_code;
                probe.address = (uint16_t)address;
                probe.register_count = preferred_count;
            }
        } else {
            err = discover_probe_address(&s_full_request, slave_id,
                                         (uint16_t)address,
                                         dev->probe_function_code,
                                         preferred_count, &probe);
        }

        if (err != ESP_OK) {
            ++address;
            if (dev->reg_count > 0 && ++empty_gap >= s_full_request.max_empty_gap) {
                ESP_LOGI(TAG, "Stopping register scan after %u empty addresses",
                         empty_gap);
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        dev->probe_function_code = probe.function_code;
        dev->probe_address = probe.address;
        dev->probe_register_count = probe.register_count;
        empty_gap = 0;

        for (uint8_t i = 0; i < probe.register_count &&
             dev->reg_count < DISCOVER_MAX_REGS_PER_SLAVE; ++i) {
            uint16_t point_address = probe.address + i;
            if (point_address > reg_end) break;
            discover_append_register(dev, probe.function_code, point_address,
                                     probe.values[i], probe.address,
                                     probe.register_count, i, NULL);
        }
        address += probe.register_count;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    ESP_LOGI(TAG, "Device scan complete: slave %u has %u registers",
             slave_id, dev->reg_count);
    return ESP_OK;
}

/* ======================== Full Scan ======================== */

/*
 * Transport scoping for semantic profiles. A profile device may declare
 * "protocols": ["RTU", ...] and the profile root may declare
 * "source_protocol". When either is present it must match the discovered
 * device's transport, otherwise an RTU profile would also claim TCP
 * endpoints that happen to share the same slave ID (duplicate semantics).
 */
static bool profile_protocol_allows(const cJSON *profile_device,
                                    const char *default_protocol,
                                    source_protocol_t protocol)
{
    const char *want = protocol == SRC_MODBUS_TCP ? "TCP" : "RTU";
    cJSON *protocols = cJSON_GetObjectItem(profile_device, "protocols");
    if (cJSON_IsArray(protocols) && cJSON_GetArraySize(protocols) > 0) {
        cJSON *item = NULL;
        cJSON_ArrayForEach(item, protocols) {
            if (cJSON_IsString(item) && item->valuestring != NULL &&
                strcasecmp(item->valuestring, want) == 0) {
                return true;
            }
        }
        return false;
    }
    if (default_protocol != NULL) {
        return strcasecmp(default_protocol, want) == 0;
    }
    return true;
}

/*
 * Profile-guided register probe. The linear scan covers a single contiguous
 * window per device, so devices with sparse or multi-section register maps
 * can end up with too few discovered registers to reach the profile-match
 * overlap threshold, silently falling back to unit-less raw points. While
 * the scan task still owns the bus, probe every register the embedded
 * profile declares for this slave (same transport only) that the linear
 * scan did not already find.
 */
static void profile_guided_register_probe(const cJSON *profile_devices,
                                          const char *default_protocol,
                                          discovered_device_t *dev)
{
    if (!cJSON_IsArray(profile_devices) || dev == NULL || !dev->active) {
        return;
    }

    cJSON *profile_device = NULL;
    cJSON_ArrayForEach(profile_device, profile_devices) {
        if (dev->reg_count >= DISCOVER_MAX_REGS_PER_SLAVE) break;
        cJSON *slave = cJSON_GetObjectItem(profile_device, "slave_id");
        if (!cJSON_IsNumber(slave) ||
            (uint8_t)slave->valuedouble != dev->slave_id) {
            continue;
        }
        if (!profile_protocol_allows(profile_device, default_protocol,
                                     dev->source_protocol)) {
            continue;
        }
        cJSON *registers = cJSON_GetObjectItem(profile_device, "registers");
        if (!cJSON_IsArray(registers)) continue;

        cJSON *reg = NULL;
        cJSON_ArrayForEach(reg, registers) {
            if (dev->reg_count >= DISCOVER_MAX_REGS_PER_SLAVE) break;
            cJSON *address = cJSON_GetObjectItem(reg, "address");
            cJSON *function = cJSON_GetObjectItem(reg, "function_code");
            cJSON *type = cJSON_GetObjectItem(reg, "data_type");
            if (!cJSON_IsNumber(address)) continue;
            uint16_t addr = (uint16_t)address->valuedouble;
            uint8_t fc = cJSON_IsNumber(function)
                ? (uint8_t)function->valuedouble : 3;
            if (fc < 1 || fc > 4) continue;
            if (discovered_has_register(dev, addr, fc)) continue;

            uint8_t width = profile_type_width(profile_data_type(
                cJSON_IsString(type) ? type->valuestring : NULL));
            if (width == 0) width = 1;
            if (width > DISCOVER_MAX_BLOCK_REGS) width = DISCOVER_MAX_BLOCK_REGS;
            if ((uint32_t)addr + width > 65536U) continue;

            uint16_t values[DISCOVER_MAX_BLOCK_REGS] = {0};
            esp_err_t err = discover_read_registers(
                dev->source_protocol, dev->channel_id, dev->slave_id,
                fc, addr, width, values);
            if (err != ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(2));
                continue;
            }
            for (uint8_t i = 0; i < width &&
                 dev->reg_count < DISCOVER_MAX_REGS_PER_SLAVE; ++i) {
                discover_append_register(dev, fc, (uint16_t)(addr + i),
                                         values[i], addr, width, i,
                                         " (profile-guided)");
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
}

static esp_err_t full_scan_sync(const discover_scan_params_t *params)
{
    uint16_t r_start = params->reg_start;
    uint16_t r_end = params->reg_end;
    uint32_t scan_span = (uint32_t)r_end - r_start;
    taskENTER_CRITICAL(&s_result_lock);
    s_result.registers_found = 0;
    s_result.mappings_created = 0;
    taskEXIT_CRITICAL(&s_result_lock);

    /* Phase 1: bus / endpoint scan */
    esp_err_t err = scan_bus_sync(params);
    if (err != ESP_OK) return err;

    /*
     * Parse the embedded semantic profile once so the register scan can
     * re-probe profile-declared addresses the linear window missed.
     */
    cJSON *profile_root = cJSON_ParseWithLength(
        (const char *)lab_profile_json_start,
        (size_t)(lab_profile_json_end - lab_profile_json_start));
    cJSON *profile_devices = profile_root
        ? cJSON_GetObjectItem(profile_root, "devices") : NULL;
    cJSON *profile_proto = profile_root
        ? cJSON_GetObjectItem(profile_root, "source_protocol") : NULL;
    const char *default_protocol =
        cJSON_IsString(profile_proto) ? profile_proto->valuestring : NULL;
    if (!cJSON_IsArray(profile_devices)) {
        profile_devices = NULL;
        default_protocol = NULL;
    }

    /* Phase 2: Register scan for each discovered device */
    for (int i = 0; i < s_device_count; i++) {
        if (s_devices[i].active) {
            uint16_t device_start = r_start;
            uint16_t device_end = r_end;
            if (s_devices[i].probe_register_count > 0 &&
                (s_devices[i].probe_address < r_start ||
                 s_devices[i].probe_address > r_end)) {
                device_start = s_devices[i].probe_address;
                uint32_t candidate_end = (uint32_t)device_start + scan_span;
                device_end = candidate_end > UINT16_MAX
                    ? UINT16_MAX : (uint16_t)candidate_end;
            }
            esp_err_t scan_err = modbus_discover_scan_device(
                s_devices[i].slave_id, device_start, device_end);
            if (scan_err == ESP_ERR_NOT_FOUND) {
                taskENTER_CRITICAL(&s_result_lock);
                if (s_result.devices_found > 0) --s_result.devices_found;
                ++s_result.slaves_skipped;
                taskEXIT_CRITICAL(&s_result_lock);
            } else if (scan_err != ESP_OK) {
                cJSON_Delete(profile_root);
                return scan_err;
            } else if (profile_devices != NULL) {
                profile_guided_register_probe(profile_devices,
                                              default_protocol,
                                              &s_devices[i]);
            }
        }
    }
    cJSON_Delete(profile_root);

    uint16_t active_count = 0;
    for (uint16_t i = 0; i < s_device_count; ++i) {
        if (!s_devices[i].active) continue;
        if (active_count != i) {
            s_devices[active_count] = s_devices[i];
        }
        ++active_count;
    }
    s_device_count = active_count;
    taskENTER_CRITICAL(&s_result_lock);
    s_result.devices_found = active_count;
    taskEXIT_CRITICAL(&s_result_lock);

    discover_result_t result = modbus_discover_get_result();
    ESP_LOGI(TAG, "Full scan complete: %u devices, %u registers",
             s_device_count, result.registers_found);
    return ESP_OK;
}

static void full_scan_task(void *argument)
{
    bool stack_uses_caps = argument != NULL;
    scheduler_pause_modbus_polling(true);
    modbus_access_set_probe_mode(true);
    esp_log_level_set("MB_CONTROLLER_MASTER", ESP_LOG_NONE);
    esp_err_t err = full_scan_sync(&s_full_request);
    esp_log_level_set("MB_CONTROLLER_MASTER", ESP_LOG_ERROR);
    modbus_access_set_probe_mode(false);
    scheduler_pause_modbus_polling(false);
    discover_set_task_state(false, true,
                            err == ESP_OK ? DISCOVER_PHASE_COMPLETE : DISCOVER_PHASE_ERROR,
                            err);
    s_scan_task = NULL;
    if (stack_uses_caps) {
        vTaskDeleteWithCaps(NULL);
    } else {
        vTaskDelete(NULL);
    }
}

esp_err_t modbus_discover_full_scan(const discover_scan_params_t *params)
{
    if (s_scan_task != NULL || s_result.scan_in_progress) return ESP_ERR_INVALID_STATE;
    if (s_devices == NULL || s_device_capacity == 0) return ESP_ERR_NO_MEM;
    s_full_request = params != NULL ? *params : (discover_scan_params_t){
        .slave_start = 1, .slave_end = 247, .reg_start = 0, .reg_end = 100,
        .source_protocol = SRC_MODBUS_RTU, .channel_id = 0,
        .function_codes = {1, 2, 3, 4}, .fc_count = 4,
        .max_empty_gap = DISCOVER_DEFAULT_EMPTY_GAP,
    };
    discover_default_functions(&s_full_request);
    if (s_full_request.slave_start < 1 || s_full_request.slave_end > 247 ||
        s_full_request.slave_start > s_full_request.slave_end ||
        s_full_request.reg_start > s_full_request.reg_end) return ESP_ERR_INVALID_ARG;
    if (s_full_request.max_empty_gap > 64) s_full_request.max_empty_gap = 64;
    taskENTER_CRITICAL(&s_result_lock);
    memset(&s_result, 0, sizeof(s_result));
    s_result.device_capacity = s_device_capacity;
    s_result.scan_in_progress = true;
    s_result.phase = DISCOVER_PHASE_BUS_SCAN;
    s_result.last_error = ESP_OK;
    taskEXIT_CRITICAL(&s_result_lock);
    BaseType_t task_created = xTaskCreateWithCaps(
        full_scan_task, "mb_full_scan", 6144, (void *)1, 3, &s_scan_task,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (task_created != pdPASS) {
        task_created = xTaskCreate(
            full_scan_task, "mb_full_scan", 6144, NULL, 3, &s_scan_task);
    }
    if (task_created != pdPASS) {
        discover_set_task_state(false, true, DISCOVER_PHASE_ERROR, ESP_ERR_NO_MEM);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

/* ======================== Result Accessors ======================== */

discover_result_t modbus_discover_get_result(void)
{
    discover_result_t snapshot;
    taskENTER_CRITICAL(&s_result_lock);
    snapshot = s_result;
    taskEXIT_CRITICAL(&s_result_lock);
    return snapshot;
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

uint16_t modbus_discover_get_capacity(void)
{
    return s_device_capacity;
}

/* ======================== Apply Mappings ======================== */

static data_type_t profile_data_type(const char *name)
{
    if (name && strcmp(name, "INT16") == 0) return DT_INT16;
    if (name && strcmp(name, "FLOAT32") == 0) return DT_FLOAT32;
    if (name && strcmp(name, "INT32") == 0) return DT_INT32;
    if (name && strcmp(name, "UINT32") == 0) return DT_UINT32;
    if (name && strcmp(name, "BOOL") == 0) return DT_BOOL;
    if (name && strcmp(name, "INT64") == 0) return DT_INT64;
    if (name && strcmp(name, "UINT64") == 0) return DT_UINT64;
    if (name && strcmp(name, "FLOAT64") == 0) return DT_FLOAT64;
    if (name && strcmp(name, "BCD16") == 0) return DT_BCD16;
    if (name && strcmp(name, "BITFIELD16") == 0) return DT_BITFIELD16;
    if (name && strcmp(name, "ASCII") == 0) return DT_ASCII;
    return DT_UINT16;
}

static byte_order_t profile_byte_order(const char *name)
{
    if (name && strcmp(name, "CDAB") == 0) return BYTE_ORDER_CDAB;
    if (name && strcmp(name, "BADC") == 0) return BYTE_ORDER_BADC;
    if (name && strcmp(name, "DCBA") == 0) return BYTE_ORDER_DCBA;
    return BYTE_ORDER_ABCD;
}

static uint8_t profile_type_width(data_type_t type)
{
    if (type == DT_FLOAT32 || type == DT_INT32 || type == DT_UINT32) return 2;
    if (type == DT_FLOAT64 || type == DT_INT64 || type == DT_UINT64) return 4;
    return 1;
}

static bool discovered_has_register(const discovered_device_t *device,
                                    uint16_t address, uint8_t function_code)
{
    uint16_t protocol_offset =
        discover_register_offset(function_code, address);
    for (int i = 0; i < device->reg_count; ++i) {
        const discovered_register_t *reg = &device->registers[i];
        if (reg->valid &&
            (reg->register_address == address ||
             reg->register_address == protocol_offset) &&
            reg->function_code == function_code) {
            return true;
        }
    }
    return false;
}

/*
 * A profile match requires at least two semantic register starts. This is a
 * lightweight fingerprint: slave ID alone is not enough to claim semantics.
 * The profile's transport scope ("protocols"/"source_protocol") must also
 * match, so an RTU profile never claims a TCP endpoint with the same ID.
 */
static bool profile_matches_discovered_device(const cJSON *profile_device,
                                              const discovered_device_t *device,
                                              const char *default_protocol)
{
    cJSON *slave = cJSON_GetObjectItem(profile_device, "slave_id");
    cJSON *registers = cJSON_GetObjectItem(profile_device, "registers");
    if (!cJSON_IsNumber(slave) || !cJSON_IsArray(registers) ||
        (uint8_t)slave->valuedouble != device->slave_id) {
        return false;
    }
    if (!profile_protocol_allows(profile_device, default_protocol,
                                 device->source_protocol)) {
        return false;
    }

    int overlap = 0;
    cJSON *reg = NULL;
    cJSON_ArrayForEach(reg, registers) {
        cJSON *address = cJSON_GetObjectItem(reg, "address");
        cJSON *function = cJSON_GetObjectItem(reg, "function_code");
        if (!cJSON_IsNumber(address)) continue;
        uint8_t fc = cJSON_IsNumber(function) ? (uint8_t)function->valuedouble : 3;
        if (discovered_has_register(device, (uint16_t)address->valuedouble, fc)) {
            ++overlap;
            if (overlap >= 2) return true;
        }
    }
    return false;
}

static cJSON *find_profile_device(cJSON *profile_devices,
                                  const discovered_device_t *device,
                                  const char *default_protocol)
{
    cJSON *profile_device = NULL;
    cJSON_ArrayForEach(profile_device, profile_devices) {
        if (profile_matches_discovered_device(profile_device, device,
                                              default_protocol)) {
            return profile_device;
        }
    }
    return NULL;
}

static void profile_copy_string(cJSON *object, const char *key,
                                char *destination, size_t destination_size)
{
    cJSON *value = cJSON_GetObjectItem(object, key);
    if (cJSON_IsString(value) && value->valuestring != NULL) {
        strlcpy(destination, value->valuestring, destination_size);
    }
}

static float profile_number(cJSON *object, const char *key, float fallback)
{
    cJSON *value = cJSON_GetObjectItem(object, key);
    return cJSON_IsNumber(value) ? (float)value->valuedouble : fallback;
}

static int apply_embedded_semantic_profile(bool *matched_devices)
{
    size_t profile_length =
        (size_t)(lab_profile_json_end - lab_profile_json_start);
    cJSON *root = cJSON_ParseWithLength(
        (const char *)lab_profile_json_start, profile_length);
    if (root == NULL) {
        ESP_LOGE(TAG, "Embedded TCM semantic profile is invalid");
        return 0;
    }

    cJSON *profile_devices = cJSON_GetObjectItem(root, "devices");
    cJSON *profile_name = cJSON_GetObjectItem(root, "name");
    cJSON *profile_version = cJSON_GetObjectItem(root, "profile_version");
    cJSON *profile_proto = cJSON_GetObjectItem(root, "source_protocol");
    const char *default_protocol =
        cJSON_IsString(profile_proto) ? profile_proto->valuestring : NULL;
    int capacity = amm_get_capacity();
    amm_mapping_entry_t *entries = heap_caps_calloc(
        (size_t)capacity, sizeof(entries[0]),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (entries == NULL) {
        entries = heap_caps_calloc(
            (size_t)capacity, sizeof(entries[0]),
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (!cJSON_IsArray(profile_devices) || entries == NULL) {
        cJSON_Delete(root);
        free(entries);
        return 0;
    }

    int entry_count = 0;
    int matched_count = 0;
    for (int i = 0; i < s_device_count && entry_count < capacity; ++i) {
        const discovered_device_t *device = &s_devices[i];
        if (!device->active) continue;

        cJSON *profile_device = find_profile_device(profile_devices, device,
                                                    default_protocol);
        if (profile_device == NULL) continue;

        matched_devices[i] = true;
        ++matched_count;
        cJSON *poll = cJSON_GetObjectItem(profile_device, "poll_interval_ms");
        cJSON *priority = cJSON_GetObjectItem(profile_device, "priority");
        cJSON *registers = cJSON_GetObjectItem(profile_device, "registers");
        cJSON *reg = NULL;
        cJSON_ArrayForEach(reg, registers) {
            if (entry_count >= capacity) break;
            cJSON *address = cJSON_GetObjectItem(reg, "address");
            cJSON *function = cJSON_GetObjectItem(reg, "function_code");
            cJSON *type = cJSON_GetObjectItem(reg, "data_type");
            cJSON *order = cJSON_GetObjectItem(reg, "byte_order");
            if (!cJSON_IsNumber(address)) continue;

            amm_mapping_entry_t *entry = &entries[entry_count++];
            entry->source_protocol = device->source_protocol;
            entry->channel_id = device->channel_id;
            entry->slave_id = device->slave_id;
            entry->function_code =
                cJSON_IsNumber(function) ? (uint8_t)function->valuedouble : 3;
            entry->object_type = entry->function_code >= 1 &&
                                 entry->function_code <= 4
                ? (modbus_object_type_t)entry->function_code
                : MODBUS_OBJECT_HOLDING_REGISTER;
            entry->register_address = (uint16_t)address->valuedouble;
            entry->data_type = profile_data_type(
                cJSON_IsString(type) ? type->valuestring : NULL);
            entry->byte_order = profile_byte_order(
                cJSON_IsString(order) ? order->valuestring : NULL);
            entry->scale_factor = profile_number(reg, "scale", 1.0f);
            entry->offset = profile_number(reg, "offset", 0.0f);
            entry->poll_interval_ms =
                cJSON_IsNumber(poll) ? (uint32_t)poll->valuedouble : 1000;
            entry->priority =
                cJSON_IsNumber(priority) ? (uint8_t)priority->valuedouble : 5;
            entry->read_start_address = entry->register_address;
            entry->read_register_count = profile_type_width(entry->data_type);
            entry->value_register_index = 0;
            entry->active = true;
            entry->discovered = true;
            entry->retry_count = 2;
            entry->retry_backoff_ms = 50;
            entry->semantic_source = SEMANTIC_SOURCE_PROFILE;
            entry->semantic_status = SEMANTIC_STATUS_VERIFIED;
            entry->semantic_confidence = 100;
            if (cJSON_IsString(profile_name)) {
                strlcpy(entry->semantic_profile_id, profile_name->valuestring,
                        sizeof(entry->semantic_profile_id));
            }
            entry->semantic_profile_version = cJSON_IsNumber(profile_version)
                ? (uint32_t)profile_version->valuedouble : 1;
            strlcpy(entry->semantic_evidence, "device fingerprint + register profile",
                    sizeof(entry->semantic_evidence));

            if (device->source_protocol == SRC_MODBUS_RTU) {
                profile_copy_string(profile_device, "device_id",
                                    entry->device_id, sizeof(entry->device_id));
            }
            if (entry->device_id[0] == '\0') {
                snprintf(entry->device_id, sizeof(entry->device_id),
                         "%s_ch%u_slave_%02u",
                         device->source_protocol == SRC_MODBUS_TCP ? "tcp" : "rtu",
                         device->channel_id, device->slave_id);
            }
            profile_copy_string(reg, "point_id",
                                entry->point_id, sizeof(entry->point_id));
            profile_copy_string(reg, "name",
                                entry->measurement_name,
                                sizeof(entry->measurement_name));
            profile_copy_string(reg, "unit",
                                entry->unit, sizeof(entry->unit));
            if (entry->point_id[0] == '\0') {
                snprintf(entry->point_id, sizeof(entry->point_id),
                         "register_%u", entry->register_address);
            }
            if (entry->measurement_name[0] == '\0') {
                strlcpy(entry->measurement_name, entry->point_id,
                        sizeof(entry->measurement_name));
            }
            snprintf(entry->mqtt_topic, sizeof(entry->mqtt_topic),
                     "factory/data/%s/%s", entry->device_id, entry->point_id);
            entry->constraint.writable =
                cJSON_IsTrue(cJSON_GetObjectItem(reg, "writable"));
            entry->constraint.valid_range_min =
                profile_number(reg, "minimum", -32768.0f);
            entry->constraint.valid_range_max =
                profile_number(reg, "maximum", 65535.0f);
        }
    }

    int imported = 0;
    esp_err_t error = entry_count > 0
        ? amm_import_mappings(entries, entry_count, true, &imported)
        : ESP_ERR_NOT_FOUND;
    if (error != ESP_OK) {
        memset(matched_devices, 0,
               (size_t)s_device_count * sizeof(matched_devices[0]));
        matched_count = 0;
        imported = 0;
        ESP_LOGW(TAG, "TCM semantic profile apply failed: %s",
                 esp_err_to_name(error));
    } else {
        ESP_LOGI(TAG, "TCM semantic profile resolved %d devices / %d points",
                 matched_count, imported);
    }

    s_apply_result.profile_devices = matched_count;
    s_apply_result.semantic_mappings = imported;
    free(entries);
    cJSON_Delete(root);
    return imported;
}

int modbus_discover_apply_mappings(void)
{
    memset(&s_apply_result, 0, sizeof(s_apply_result));
    bool matched_devices[DISCOVER_MAX_SLAVES] = {false};
    int created = apply_embedded_semantic_profile(matched_devices);

    int capacity = amm_get_capacity();
    amm_mapping_entry_t *raw_entries = heap_caps_calloc(
        (size_t)capacity, sizeof(raw_entries[0]),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (raw_entries == NULL) {
        raw_entries = heap_caps_calloc(
            (size_t)capacity, sizeof(raw_entries[0]),
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }

    int raw_capacity = 0;
    if (raw_entries != NULL) {
        amm_mapping_entry_t *current = heap_caps_calloc(
            (size_t)capacity, sizeof(current[0]),
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (current == NULL) {
            current = heap_caps_calloc(
                (size_t)capacity, sizeof(current[0]),
                MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        }
        int active_count = current != NULL
            ? amm_get_entries(current, capacity) : amm_get_mapping_count();
        int replaceable = 0;
        for (int entry_index = 0;
             current != NULL && entry_index < active_count; ++entry_index) {
            for (int device_index = 0;
                 device_index < s_device_count; ++device_index) {
                const discovered_device_t *device = &s_devices[device_index];
                if (!device->active || matched_devices[device_index]) continue;
                if (current[entry_index].source_protocol ==
                        device->source_protocol &&
                    current[entry_index].channel_id == device->channel_id &&
                    current[entry_index].slave_id == device->slave_id) {
                    ++replaceable;
                    break;
                }
            }
        }
        raw_capacity = capacity - active_count + replaceable;
        if (raw_capacity < 0) raw_capacity = 0;
        free(current);
    }

    int raw_count = 0;
    for (int i = 0; i < s_device_count; i++) {
        const discovered_device_t *dev = &s_devices[i];
        if (!dev->active || matched_devices[i]) continue;

        ++s_apply_result.unresolved_devices;
        int device_point_count = 0;
        for (int j = 0; j < dev->reg_count; ++j) {
            if (dev->registers[j].valid) ++device_point_count;
        }
        if (raw_entries == NULL ||
            raw_count + device_point_count > raw_capacity) {
            ESP_LOGE(TAG,
                     "No AMM capacity for unknown %s channel %u slave %u "
                     "(need %d, available %d)",
                     dev->source_protocol == SRC_MODBUS_TCP ? "TCP" : "RTU",
                     dev->channel_id, dev->slave_id, device_point_count,
                     raw_capacity - raw_count);
            continue;
        }

        semantic_register_signature_t signatures[DISCOVER_MAX_REGS_PER_SLAVE];
        size_t signature_count = 0;
        for (int j = 0; j < dev->reg_count &&
                        j < DISCOVER_MAX_REGS_PER_SLAVE; ++j) {
            if (!dev->registers[j].valid) continue;
            signatures[signature_count].function_code =
                dev->registers[j].function_code;
            signatures[signature_count].address =
                dev->registers[j].register_address;
            ++signature_count;
        }

        semantic_device_features_t features = {
            .source_protocol = dev->source_protocol,
            .channel_id = dev->channel_id,
            .slave_id = dev->slave_id,
            .vendor_name = dev->vendor_name,
            .product_code = dev->product_code,
            .revision = dev->revision,
            .probe_function_code = dev->probe_function_code,
            .probe_address = dev->probe_address,
            .registers = signatures,
            .register_count = signature_count,
        };
        semantic_device_identity_t identity;
        semantic_inference_identify(&features, &identity);

        for (int j = 0; j < dev->reg_count; ++j) {
            const discovered_register_t *reg = &dev->registers[j];
            if (!reg->valid) continue;
            semantic_raw_point_t point = {
                .function_code = reg->function_code,
                .address = reg->register_address,
                .raw_value = reg->raw_value,
                .read_start_address = reg->read_start_address,
                .read_register_count = reg->read_register_count,
                .value_register_index = reg->value_register_index,
                /*
                 * Only user-confirmed edits flow into the raw mapping;
                 * scan-time semantic guesses stay display-only hints.
                 */
                .user_edited = reg->user_edited,
                .name = reg->inferred_name,
                .unit = reg->inferred_unit,
                .data_type = reg->inferred_type,
                .writable = reg->writable,
                .range_min = reg->range_min,
                .range_max = reg->range_max,
            };
            semantic_inference_build_raw_mapping(
                &features, &identity, &point, dev->device_id,
                &raw_entries[raw_count++]);
        }
        ESP_LOGW(TAG,
                 "Unknown %s channel %u slave %u fingerprint %s: "
                 "%u raw points require semantic confirmation",
                 dev->source_protocol == SRC_MODBUS_TCP ? "TCP" : "RTU",
                 dev->channel_id, dev->slave_id, identity.fingerprint,
                 dev->reg_count);
    }

    if (raw_count > 0 && raw_entries != NULL) {
        int imported = 0;
        esp_err_t error =
            amm_import_mappings(raw_entries, raw_count, true, &imported);
        if (error == ESP_OK) {
            s_apply_result.raw_mappings = imported;
            created += imported;
        } else {
            ESP_LOGE(TAG, "Unknown-device raw mapping import failed: %s",
                     esp_err_to_name(error));
        }
    }
    free(raw_entries);

    taskENTER_CRITICAL(&s_result_lock);
    s_result.mappings_created += created;
    taskEXIT_CRITICAL(&s_result_lock);
    ESP_LOGI(TAG, "Applied %d new AMM mapping entries", created);
    return created;
}

discover_apply_result_t modbus_discover_get_apply_result(void)
{
    return s_apply_result;
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
    discovered_device_t *dev = modbus_discover_find_device(slave_id);
    if (!dev) return ESP_ERR_NOT_FOUND;
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
    reg->range_min = range_min;
    reg->range_max = range_max;
    reg->user_edited = true;

    /*
     * Write through to the AMM mapping when this point is already mapped.
     * Without this, the edit only lived in the volatile discovery table and
     * the next "apply mappings" silently rebuilt the point with profile or
     * raw defaults, dropping the user's unit/scale/range. The entry is
     * promoted to USER/VERIFIED so later re-imports preserve it.
     */
    amm_mapping_entry_t amm_entry;
    if (amm_find_mapping_for_object(dev->source_protocol, dev->channel_id,
                                    slave_id, reg->function_code, reg_addr,
                                    &amm_entry) == ESP_OK) {
        if (name && name[0]) {
            strlcpy(amm_entry.measurement_name, name,
                    sizeof(amm_entry.measurement_name));
        }
        if (unit) {
            strlcpy(amm_entry.unit, unit, sizeof(amm_entry.unit));
        }
        amm_entry.data_type = dtype;
        amm_entry.constraint.writable = writable;
        amm_entry.constraint.valid_range_min = range_min;
        amm_entry.constraint.valid_range_max = range_max;
        amm_entry.semantic_source = SEMANTIC_SOURCE_USER;
        amm_entry.semantic_status = SEMANTIC_STATUS_VERIFIED;
        amm_entry.semantic_confidence = 100;
        strlcpy(amm_entry.semantic_evidence, "user edit via discovery editor",
                sizeof(amm_entry.semantic_evidence));
        esp_err_t amm_err = amm_add_mapping(&amm_entry);
        if (amm_err != ESP_OK) {
            ESP_LOGW(TAG, "AMM write-through failed for slave %u reg %u: %s",
                     slave_id, reg_addr, esp_err_to_name(amm_err));
        }
    }

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
            taskENTER_CRITICAL(&s_result_lock);
            if (s_result.registers_found > 0) s_result.registers_found--;
            taskEXIT_CRITICAL(&s_result_lock);

            ESP_LOGI(TAG, "Register deleted: slave %u addr %u (remaining: %u)",
                     slave_id, reg_addr, dev->reg_count);
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}
