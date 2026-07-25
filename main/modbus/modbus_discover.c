/**
 * @file modbus_discover.c
 * @brief Automatic MODBUS device discovery and semantic inference.
 */

#include <string.h>
#include <stdio.h>

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

static uint16_t discover_register_offset(uint8_t function_code, uint16_t configured_address)
{
    if ((function_code == 3 || function_code == 6 || function_code == 16) &&
        configured_address >= 40001) {
        return configured_address - 40001;
    }
    if (function_code == 4 && configured_address >= 30001 &&
        configured_address < 40001) {
        return configured_address - 30001;
    }
    return configured_address;
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

static void discover_default_functions(discover_scan_params_t *params)
{
    if (params->fc_count == 0) {
        params->function_codes[0] = 3;
        params->function_codes[1] = 4;
        params->fc_count = 2;
    }
    if (params->fc_count > 2) {
        params->fc_count = 2;
    }
    for (uint8_t i = 0; i < params->fc_count; ++i) {
        if (params->function_codes[i] != 3 && params->function_codes[i] != 4) {
            params->function_codes[i] = i == 0 ? 3 : 4;
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
    if (preferred == 3 || preferred == 4) {
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
    uint8_t functions[2];
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
    uint16_t addresses[3];
    uint8_t address_count = 0;
    addresses[address_count++] = params->reg_start;
    if (params->reg_start != 1) addresses[address_count++] = 1;
    if (params->reg_start != 0) addresses[address_count++] = 0;

    for (uint8_t i = 0; i < address_count; ++i) {
        esp_err_t err = discover_probe_address(params, slave_id, addresses[i],
                                               0, 0, probe);
        if (err == ESP_OK) return ESP_OK;
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

static esp_err_t scan_bus_sync(const discover_scan_params_t *params)
{
    uint8_t start = params->slave_start;
    uint8_t end = params->slave_end;
    if (start < 1) start = 1;
    if (end > 247) end = 247;
    if (start > end) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "%s scan: channel=%u slave IDs %u .. %u",
             params->source_protocol == SRC_MODBUS_TCP ? "TCP" : "RTU",
             params->channel_id, start, end);
    s_result.scan_in_progress = true;
    s_result.total_scanned = end - start + 1;

    /* Reset device list */
    memset(s_devices, 0, sizeof(s_devices));
    s_device_count = 0;

    for (uint16_t sid = start; sid <= end; ++sid) {
        discover_probe_t probe;
        /* Probe with FC03, address 0, count 1 — quick liveness check */
        esp_err_t err = discover_probe_device(params, (uint8_t)sid, &probe);
        if (err == ESP_OK) {
            if (s_device_count < DISCOVER_MAX_SLAVES) {
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
                dev->active = true;
                dev->reg_count = 0;
                s_device_count++;

                ESP_LOGI(TAG, "  Found slave %u: FC%02u addr=%u count=%u",
                         (unsigned)sid, probe.function_code, probe.address,
                         probe.register_count);
            }
        }
    }

    s_result.devices_found = s_device_count;
    s_result.scan_in_progress = false;
    s_result.scan_complete = true;

    ESP_LOGI(TAG, "Bus scan complete: %u devices found out of %u probed",
             s_device_count, s_result.total_scanned);
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
        .function_codes = {3, 4},
        .fc_count = 2,
        .max_empty_gap = DISCOVER_DEFAULT_EMPTY_GAP,
    };
    scan_bus_sync(&params);
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
        if (s_devices[i].slave_id == slave_id &&
            s_devices[i].source_protocol == s_full_request.source_protocol &&
            s_devices[i].channel_id == s_full_request.channel_id &&
            s_devices[i].active) {
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
        dev->source_protocol = s_full_request.source_protocol;
        dev->channel_id = s_full_request.channel_id;
        snprintf(dev->device_id, sizeof(dev->device_id),
                 "%s_ch%u_slave_%02u",
                 dev->source_protocol == SRC_MODBUS_TCP ? "tcp" : "rtu",
                 dev->channel_id, slave_id);
        dev->name[0] = '\0';
        dev->description[0] = '\0';
        snprintf(dev->mqtt_topic_prefix, sizeof(dev->mqtt_topic_prefix),
                 "factory/data/%s_ch%u_slave_%02u",
                 dev->source_protocol == SRC_MODBUS_TCP ? "tcp" : "rtu",
                 dev->channel_id, slave_id);
        dev->active = true;
        dev->reg_count = 0;
    }

    ESP_LOGI(TAG, "Register scan: %s channel=%u slave %u, addr %u .. %u",
             dev->source_protocol == SRC_MODBUS_TCP ? "TCP" : "RTU",
             dev->channel_id, slave_id, reg_start, reg_end);

    discover_default_functions(&s_full_request);
    dev->reg_count = 0;
    s_result.scan_in_progress = true;

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

            discovered_register_t *reg = &dev->registers[dev->reg_count++];
            memset(reg, 0, sizeof(*reg));
            reg->register_address = point_address;
            reg->function_code = probe.function_code;
            reg->raw_value = probe.values[i];
            reg->inferred_type = ((int16_t)probe.values[i] < 0) ?
                                 DT_INT16 : DT_UINT16;
            reg->sample_value = reg->inferred_type == DT_INT16 ?
                                (float)(int16_t)probe.values[i] :
                                (float)probe.values[i];
            reg->read_start_address = probe.address;
            reg->read_register_count = probe.register_count;
            reg->value_register_index = i;
            reg->writable = false;
            reg->valid = true;

            float range_min;
            float range_max;
            modbus_discover_infer_semantics(point_address, reg->sample_value,
                                            reg->inferred_name,
                                            reg->inferred_unit,
                                            &range_min, &range_max);
            ESP_LOGI(TAG, "  FC%02u [%u] raw=%u window=%u+%u index=%u",
                     reg->function_code, point_address, reg->raw_value,
                     reg->read_start_address, reg->read_register_count,
                     reg->value_register_index);
            ++s_result.registers_found;
        }
        address += probe.register_count;
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
    uint16_t r_start = params->reg_start;
    uint16_t r_end = params->reg_end;
    s_result.registers_found = 0;
    s_result.mappings_created = 0;

    /* Phase 1: bus / endpoint scan */
    esp_err_t err = scan_bus_sync(params);
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
        .slave_start = 1, .slave_end = 247, .reg_start = 0, .reg_end = 100,
        .source_protocol = SRC_MODBUS_RTU, .channel_id = 0,
        .function_codes = {3, 4}, .fc_count = 2,
        .max_empty_gap = DISCOVER_DEFAULT_EMPTY_GAP,
    };
    discover_default_functions(&s_full_request);
    if (s_full_request.slave_start < 1 || s_full_request.slave_end > 247 ||
        s_full_request.slave_start > s_full_request.slave_end ||
        s_full_request.reg_start > s_full_request.reg_end) return ESP_ERR_INVALID_ARG;
    if (s_full_request.max_empty_gap > 64) s_full_request.max_empty_gap = 64;
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

            /* Skip if mapping already exists for this protocol/channel/slave/address. */
            amm_mapping_entry_t existing;
            if (amm_find_mapping_for_channel(dev->source_protocol, dev->channel_id,
                                             dev->slave_id, reg->register_address,
                                             &existing) == ESP_OK) {
                ESP_LOGI(TAG, "Mapping already exists for %s channel %u slave %u / addr %u",
                         dev->source_protocol == SRC_MODBUS_TCP ? "TCP" : "RTU",
                         dev->channel_id, dev->slave_id, reg->register_address);
                continue;
            }

            amm_mapping_entry_t entry;
            memset(&entry, 0, sizeof(entry));

            entry.slave_id = dev->slave_id;
            entry.source_protocol = dev->source_protocol;
            entry.channel_id = dev->channel_id;
            entry.function_code = reg->function_code;
            entry.register_address = reg->register_address;
            entry.data_type = reg->inferred_type;
            entry.read_start_address = reg->read_start_address;
            entry.read_register_count = reg->read_register_count;
            entry.value_register_index = reg->value_register_index;
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
