/**
 * @file sim_main.c
 * @brief Simulation entry point for the ESP32-S3 MODBUS-MQTT Gateway.
 *
 * Demonstrates the full system pipeline on a PC without real hardware:
 *   Scenario 1: Normal operation (poll -> build -> enrich -> validate -> publish)
 *   Scenario 2: MQTT disconnection + offline caching
 *   Scenario 3: Reconnection + ordered replay
 *   Scenario 4: Downlink command validation
 *   Scenario 5: AMM adaptive mapping (runtime device addition)
 *
 * Everything runs sequentially in main() for deterministic, easy-to-follow output.
 * The scheduler.c module is NOT used; instead we call module APIs directly.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <inttypes.h>

/* ESP-IDF mock headers */
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_flash.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/* Project module headers */
#include "gateway_config.h"
#include "tcm/tcm_context.h"
#include "amm/amm_mapping.h"
#include "modbus/modbus_access.h"
#include "mqtt_comm/mqtt_handler.h"
#include "uif/uif_persistence.h"
#include "eval/eval_logger.h"

/* MQTT mock control (defined in esp_mock.c) */
extern bool sim_mqtt_mock_is_connected(void);
extern int  sim_mqtt_mock_publish_count(void);

static const char *TAG = "SIM";

/* ======================== Simulation Poll Table ======================== */

/**
 * Poll table matching the 4 default AMM mapping entries.
 * Register addresses use the 40001 convention to match the AMM/TCM lookup tables.
 */
typedef struct {
    uint8_t     slave_id;
    uint8_t     function_code;
    uint16_t    register_address;
    uint16_t    register_count;
    data_type_t data_type;
} sim_poll_entry_t;

static const sim_poll_entry_t s_sim_poll_table[] = {
    /* slave  FC   addr    cnt  type        description                */
    {   1,   0x03, 40001,   2,  DT_FLOAT32 },  /* Motor temperature    */
    {   1,   0x03, 40003,   2,  DT_FLOAT32 },  /* Line pressure        */
    {   2,   0x03, 40001,   2,  DT_UINT16  },  /* Conveyor speed       */
    {   2,   0x03, 40003,   1,  DT_INT16   },  /* Motor current        */
};

#define SIM_POLL_TABLE_COUNT  (sizeof(s_sim_poll_table) / sizeof(s_sim_poll_table[0]))

/* ======================== Helper: Full Pipeline for One Poll ======================== */

/**
 * @brief Execute the full pipeline for one poll entry:
 *   1. Read MODBUS registers
 *   2. Convert to float
 *   3. Build TCM context
 *   4. Enrich with AMM metadata
 *   5. Validate
 *   6. Serialize to JSON
 *   7. Publish via MQTT or cache via UIF
 *
 * @return 0 on success, -1 on failure
 */
static int sim_poll_and_publish(const sim_poll_entry_t *poll_cfg,
                                 network_state_t net_state,
                                 bool cache_mode,
                                 QueueHandle_t mqtt_out_queue)
{
    /* Step 1: Read MODBUS registers */
    uint16_t raw_regs[4] = {0};
    esp_err_t err;

    if (poll_cfg->function_code == 0x04) {
        err = modbus_read_input_register(poll_cfg->slave_id,
                                         poll_cfg->register_address,
                                         poll_cfg->register_count,
                                         raw_regs);
    } else {
        err = modbus_read_holding_register(poll_cfg->slave_id,
                                           poll_cfg->register_address,
                                           poll_cfg->register_count,
                                           raw_regs);
    }

    if (err != ESP_OK) {
        printf("  [POLL FAIL] slave=%u addr=%u err=%s\n",
               poll_cfg->slave_id, poll_cfg->register_address, esp_err_to_name(err));
        eval_increment_metric("failed_polls", 1);
        eval_increment_metric("total_polls", 1);
        return -1;
    }

    eval_increment_metric("successful_polls", 1);
    eval_increment_metric("total_polls", 1);

    /* Step 2: Convert raw registers to float */
    float value = modbus_convert_to_float(raw_regs, poll_cfg->data_type);

    /* Step 3: Build TCM context */
    tcm_context_t ctx;
    int ret = tcm_build_context(&ctx,
                                poll_cfg->slave_id,
                                poll_cfg->function_code,
                                poll_cfg->register_address,
                                value,
                                QUALITY_GOOD,
                                net_state);
    if (ret != 0) {
        printf("  [TCM FAIL] build_context returned %d\n", ret);
        eval_increment_metric("contexts_rejected", 1);
        return -1;
    }

    /* Override data_type from poll config (build_context defaults to FLOAT32) */
    ctx.data_type = poll_cfg->data_type;

    eval_increment_metric("contexts_created", 1);

    /* Step 4: Enrich with AMM metadata */
    esp_err_t amm_err = amm_enrich_context(&ctx);
    if (amm_err == ESP_OK) {
        /* Enrichment successful - context now has device_id, point_id, etc. */
    }

    /* Step 5: Validate */
    tcm_validation_result_t val_result;
    bool valid = tcm_validate(&ctx, &val_result);
    if (!valid) {
        printf("  [VALIDATE FAIL] %s\n", val_result.fail_reason);
        eval_increment_metric("contexts_rejected", 1);
        return -1;
    }
    eval_increment_metric("contexts_validated", 1);

    /* Step 6: Serialize to JSON */
    char json_buf[TCM_MAX_JSON_LEN];
    int json_len = tcm_serialize_json(&ctx, json_buf, sizeof(json_buf));
    if (json_len <= 0) {
        printf("  [SERIALIZE FAIL]\n");
        return -1;
    }

    /* Step 7: Publish or cache */
    if (cache_mode) {
        /* Offline mode: cache to UIF */
        esp_err_t cache_err = uif_cache_record(&ctx);
        if (cache_err == ESP_OK) {
            eval_increment_metric("cached_records", 1);
        } else {
            eval_increment_metric("data_loss_count", 1);
        }
    } else {
        /* Online mode: publish via MQTT */
        const char *topic = amm_get_mqtt_topic(ctx.slave_id, ctx.register_address);
        char topic_buf[AMM_MAX_TOPIC_LEN];
        if (!topic) {
            snprintf(topic_buf, sizeof(topic_buf), "%s%s/%u/%u",
                     MQTT_DATA_TOPIC_PREFIX, MQTT_CLIENT_ID,
                     ctx.slave_id, ctx.register_address);
            topic = topic_buf;
        }

        esp_err_t pub_err = mqtt_publish(topic, json_buf, 1);
        if (pub_err == ESP_OK) {
            eval_increment_metric("mqtt_published", 1);
        } else {
            eval_increment_metric("mqtt_failed", 1);
            /* Fallback: cache the record */
            uif_cache_record(&ctx);
            eval_increment_metric("cached_records", 1);
        }
    }

    return 0;
}

/**
 * @brief Run one full poll cycle across all entries in the poll table.
 */
static void sim_run_poll_cycle(int cycle_num, network_state_t net_state,
                                bool cache_mode, QueueHandle_t mqtt_out_queue)
{
    printf("\n  --- Poll Cycle %d (net=%s, cache_mode=%s) ---\n",
           cycle_num,
           (net_state == NET_ONLINE) ? "ONLINE" : "OFFLINE",
           cache_mode ? "YES" : "NO");

    int success = 0, fail = 0;
    for (size_t i = 0; i < SIM_POLL_TABLE_COUNT; i++) {
        int ret = sim_poll_and_publish(&s_sim_poll_table[i], net_state,
                                        cache_mode, mqtt_out_queue);
        if (ret == 0) success++;
        else fail++;
    }
    printf("  Cycle %d result: %d success, %d fail\n", cycle_num, success, fail);
}

/* ======================== Simulation Helpers ======================== */

static void print_banner(void)
{
    printf("\n");
    printf("+================================================================+\n");
    printf("|   ESP32-S3 MODBUS-MQTT Gateway - PC Simulation                |\n");
    printf("|   All modules compiled from original source with mock APIs     |\n");
    printf("+================================================================+\n");
    printf("\n");
    printf("Simulation parameters:\n");
    printf("  Poll table entries : %d\n", (int)SIM_POLL_TABLE_COUNT);
    printf("  TCM max JSON len   : %d\n", TCM_MAX_JSON_LEN);
    printf("  AMM max entries    : %d\n", AMM_MAX_MAPPING_ENTRIES);
    printf("  UIF cache capacity : %d records\n", UIF_CACHE_MAX_RECORDS);
    printf("  MQTT broker URI    : %s\n", MQTT_BROKER_URI);
    printf("  MQTT client ID     : %s\n", MQTT_CLIENT_ID);
    printf("\n");
}

static void print_metrics_snapshot(const char *label)
{
    eval_metrics_t m = eval_get_metrics();
    printf("\n  === Metrics: %s ===\n", label);
    printf("  Polls      : total=%lu  ok=%lu  fail=%lu\n",
           (unsigned long)m.total_polls,
           (unsigned long)m.successful_polls,
           (unsigned long)m.failed_polls);
    printf("  Contexts   : created=%lu  validated=%lu  rejected=%lu\n",
           (unsigned long)m.contexts_created,
           (unsigned long)m.contexts_validated,
           (unsigned long)m.contexts_rejected);
    printf("  MQTT       : published=%lu  failed=%lu\n",
           (unsigned long)m.mqtt_published,
           (unsigned long)m.mqtt_failed);
    printf("  Cache      : cached=%lu  replayed=%lu  lost=%lu\n",
           (unsigned long)m.cached_records,
           (unsigned long)m.replayed_records,
           (unsigned long)m.data_loss_count);
    printf("  Commands   : recv=%lu  accepted=%lu  rejected=%lu\n",
           (unsigned long)m.commands_received,
           (unsigned long)m.commands_accepted,
           (unsigned long)m.commands_rejected);
}

/* ======================== SCENARIO 1: Normal Operation ======================== */

static void scenario_1_normal_operation(void)
{
    printf("\n");
    printf("+================================================================+\n");
    printf("|  SCENARIO 1: Normal Operation (10 poll cycles)                |\n");
    printf("+================================================================+\n");

    for (int cycle = 1; cycle <= 10; cycle++) {
        sim_run_poll_cycle(cycle, NET_ONLINE, false, NULL);
    }

    print_metrics_snapshot("After Scenario 1 (10 cycles online)");
}

/* ======================== SCENARIO 2: MQTT Disconnection + Offline Caching ==== */

static void scenario_2_offline_caching(void)
{
    printf("\n");
    printf("+================================================================+\n");
    printf("|  SCENARIO 2: MQTT Disconnection + Offline Caching (5 cycles)  |\n");
    printf("+================================================================+\n");

    /* Simulate MQTT disconnection by destroying the client */
    printf("\n[SIM] Simulating MQTT disconnection...\n");
    mqtt_destroy();
    printf("[SIM] MQTT is now DISCONNECTED. Data will be cached.\n");

    /* Cache mode: poll and cache to UIF instead of publishing */
    for (int cycle = 1; cycle <= 5; cycle++) {
        sim_run_poll_cycle(cycle, NET_OFFLINE, true, NULL);
    }

    /* Print cache status */
    int cached_count = uif_get_cached_count();
    int cache_pct    = uif_get_cache_usage_percent();
    int data_loss    = uif_get_data_loss_count();

    printf("\n  === UIF Cache Status ===\n");
    printf("  Cached records  : %d\n", cached_count);
    printf("  Cache usage     : %d%%\n", cache_pct);
    printf("  Data loss count : %d\n", data_loss);
    printf("  Est. flash usage: ~%d bytes (at %d bytes/record)\n",
           cached_count * TCM_MAX_JSON_LEN, TCM_MAX_JSON_LEN);

    print_metrics_snapshot("After Scenario 2 (5 cycles offline)");
}

/* ======================== SCENARIO 3: Reconnection + Ordered Replay ============ */

static void scenario_3_reconnect_and_replay(void)
{
    printf("\n");
    printf("+================================================================+\n");
    printf("|  SCENARIO 3: Reconnection + Ordered Replay                    |\n");
    printf("+================================================================+\n");

    /* Step 1: Simulate MQTT reconnection */
    printf("\n[SIM] Simulating MQTT reconnection...\n");
    mqtt_init();  /* Re-initialize: creates client, fires CONNECTED event */
    printf("[SIM] MQTT is now CONNECTED.\n");

    /* Step 2: Create a queue for replay output */
    QueueHandle_t mqtt_out_queue = xQueueCreate(QUEUE_MQTT_OUT_SIZE,
                                                 sizeof(mqtt_out_msg_t));
    if (!mqtt_out_queue) {
        printf("[SIM] ERROR: Failed to create mqtt_out_queue\n");
        return;
    }

    /* Step 3: Trigger UIF replay */
    int pre_replay_count = uif_get_cached_count();
    printf("\n[SIM] Triggering replay of %d cached records...\n", pre_replay_count);

    int64_t replay_start = esp_timer_get_time();
    esp_err_t replay_err = uif_replay_all(mqtt_out_queue);
    int64_t replay_end = esp_timer_get_time();
    double replay_ms = (double)(replay_end - replay_start) / 1000.0;

    if (replay_err != ESP_OK) {
        printf("[SIM] Replay failed: %s\n", esp_err_to_name(replay_err));
    }

    /* Step 4: Drain the queue and "publish" each replayed message */
    mqtt_out_msg_t out_msg;
    int replayed_count = 0;
    uint32_t last_seq = 0;
    bool order_correct = true;

    printf("\n  --- Replayed Messages (in sequence order) ---\n");
    while (xQueueReceive(mqtt_out_queue, &out_msg, pdMS_TO_TICKS(100)) == pdTRUE) {
        replayed_count++;

        /* Extract sequence_id from the JSON payload */
        uint32_t seq = 0;
        const char *seq_pos = strstr(out_msg.payload, "\"sequence_id\":");
        if (seq_pos) {
            seq = (uint32_t)strtoul(seq_pos + 14, NULL, 10);
        }

        if (seq < last_seq) {
            order_correct = false;
        }
        last_seq = seq;

        printf("  [REPLAY %d] topic=%s  seq=%u  payload_len=%d\n",
               replayed_count, out_msg.topic, seq,
               (int)strlen(out_msg.payload));

        /* Actually publish the replayed message */
        mqtt_publish(out_msg.topic, out_msg.payload, out_msg.qos);
        eval_increment_metric("replayed_records", 1);
    }

    /* Step 5: Compact the cache */
    uif_clear_replayed();

    int post_replay_count = uif_get_cached_count();

    printf("\n  === Recovery Metrics ===\n");
    printf("  Pre-replay cached  : %d records\n", pre_replay_count);
    printf("  Replayed           : %d records\n", replayed_count);
    printf("  Post-replay cached : %d records\n", post_replay_count);
    printf("  Sequence ordering  : %s\n", order_correct ? "CORRECT (monotonic)" : "INCORRECT");
    printf("  Replay latency     : %.2f ms\n", replay_ms);
    printf("  Replay throughput  : %.1f records/sec\n",
           (replay_ms > 0) ? (replayed_count * 1000.0 / replay_ms) : 0.0);

    vQueueDelete(mqtt_out_queue);
    print_metrics_snapshot("After Scenario 3 (reconnect + replay)");
}

/* ======================== SCENARIO 4: Downlink Command Validation ============== */

static void scenario_4_downlink_validation(void)
{
    printf("\n");
    printf("+================================================================+\n");
    printf("|  SCENARIO 4: Downlink Command Validation                     |\n");
    printf("+================================================================+\n");

    tcm_context_t cmd_ctx;
    amm_validation_result_t result;

    /* --- Test 4a: Valid command (device exists, writable=true, value in range) --- */
    printf("\n  --- Test 4a: Valid write command ---\n");
    memset(&cmd_ctx, 0, sizeof(cmd_ctx));
    cmd_ctx.context_id        = 100;
    cmd_ctx.slave_id          = 2;       /* plc_line2_01 */
    cmd_ctx.function_code     = 6;       /* FC06 = write single register */
    cmd_ctx.register_address  = 40001;   /* speed_01 (writable, 0-3000 rpm) */
    cmd_ctx.data_type         = DT_UINT16;
    cmd_ctx.value             = 1800.0f; /* Within range [0, 3000] */
    cmd_ctx.timestamp_ms      = esp_timer_get_time() / 1000;
    strcpy(cmd_ctx.device_id, "plc_line2_01");
    strcpy(cmd_ctx.point_id, "speed_01");
    strcpy(cmd_ctx.measurement_name, "Conveyor speed");
    strcpy(cmd_ctx.unit, "rpm");

    eval_increment_metric("commands_received", 1);
    result = amm_validate_command(&cmd_ctx);
    if (result.accepted) {
        eval_increment_metric("commands_accepted", 1);
        printf("  RESULT: ACCEPTED - device=%s point=%s value=%.0f\n",
               cmd_ctx.device_id, cmd_ctx.point_id, cmd_ctx.value);
    } else {
        eval_increment_metric("commands_rejected", 1);
        printf("  RESULT: REJECTED - %s\n", result.reject_reason);
    }

    /* --- Test 4b: Invalid command (device not writable) --- */
    printf("\n  --- Test 4b: Write to read-only device ---\n");
    memset(&cmd_ctx, 0, sizeof(cmd_ctx));
    cmd_ctx.context_id        = 101;
    cmd_ctx.slave_id          = 1;       /* plc_line1_01 */
    cmd_ctx.function_code     = 6;       /* FC06 = write single register */
    cmd_ctx.register_address  = 40001;   /* motor_temp_01 (read-only) */
    cmd_ctx.data_type         = DT_FLOAT32;
    cmd_ctx.value             = 80.0f;
    cmd_ctx.timestamp_ms      = esp_timer_get_time() / 1000;
    strcpy(cmd_ctx.device_id, "plc_line1_01");
    strcpy(cmd_ctx.point_id, "motor_temp_01");
    strcpy(cmd_ctx.measurement_name, "Motor temperature");
    strcpy(cmd_ctx.unit, "degC");

    eval_increment_metric("commands_received", 1);
    result = amm_validate_command(&cmd_ctx);
    if (result.accepted) {
        eval_increment_metric("commands_accepted", 1);
        printf("  RESULT: ACCEPTED (unexpected!)\n");
    } else {
        eval_increment_metric("commands_rejected", 1);
        printf("  RESULT: REJECTED - %s\n", result.reject_reason);
    }

    /* --- Test 4c: Out-of-range command --- */
    printf("\n  --- Test 4c: Value out of range ---\n");
    memset(&cmd_ctx, 0, sizeof(cmd_ctx));
    cmd_ctx.context_id        = 102;
    cmd_ctx.slave_id          = 2;       /* plc_line2_01 */
    cmd_ctx.function_code     = 6;       /* FC06 */
    cmd_ctx.register_address  = 40001;   /* speed_01 (range 0-3000) */
    cmd_ctx.data_type         = DT_UINT16;
    cmd_ctx.value             = 5000.0f; /* OUT OF RANGE [0, 3000] */
    cmd_ctx.timestamp_ms      = esp_timer_get_time() / 1000;
    strcpy(cmd_ctx.device_id, "plc_line2_01");
    strcpy(cmd_ctx.point_id, "speed_01");
    strcpy(cmd_ctx.measurement_name, "Conveyor speed");
    strcpy(cmd_ctx.unit, "rpm");

    eval_increment_metric("commands_received", 1);
    result = amm_validate_command(&cmd_ctx);
    if (result.accepted) {
        eval_increment_metric("commands_accepted", 1);
        printf("  RESULT: ACCEPTED (unexpected!)\n");
    } else {
        eval_increment_metric("commands_rejected", 1);
        printf("  RESULT: REJECTED - %s\n", result.reject_reason);
    }

    /* --- Test 4d: Non-existent device --- */
    printf("\n  --- Test 4d: Non-existent device ---\n");
    memset(&cmd_ctx, 0, sizeof(cmd_ctx));
    cmd_ctx.context_id        = 103;
    cmd_ctx.slave_id          = 99;      /* No such slave */
    cmd_ctx.function_code     = 6;
    cmd_ctx.register_address  = 40001;
    cmd_ctx.data_type         = DT_UINT16;
    cmd_ctx.value             = 100.0f;
    cmd_ctx.timestamp_ms      = esp_timer_get_time() / 1000;

    eval_increment_metric("commands_received", 1);
    result = amm_validate_command(&cmd_ctx);
    if (result.accepted) {
        eval_increment_metric("commands_accepted", 1);
        printf("  RESULT: ACCEPTED (unexpected!)\n");
    } else {
        eval_increment_metric("commands_rejected", 1);
        printf("  RESULT: REJECTED - %s\n", result.reject_reason);
    }

    print_metrics_snapshot("After Scenario 4 (downlink validation)");
}

/* ======================== SCENARIO 5: AMM Adaptive Mapping ==================== */

static void scenario_5_adaptive_mapping(void)
{
    printf("\n");
    printf("+================================================================+\n");
    printf("|  SCENARIO 5: AMM Adaptive Mapping (runtime device addition)  |\n");
    printf("+================================================================+\n");

    /* Step 1: Record current mapping count */
    int pre_count = amm_get_mapping_count();
    printf("\n  Current active mappings: %d\n", pre_count);

    /* Step 2: Add a new device mapping at runtime */
    printf("\n  Adding new mapping: slave=3, reg=40010 -> flow_meter_01/mag_flow_rate\n");

    int64_t add_start = esp_timer_get_time();

    amm_mapping_entry_t new_entry;
    memset(&new_entry, 0, sizeof(new_entry));
    new_entry.slave_id          = 3;
    new_entry.function_code     = 3;
    new_entry.register_address  = 40010;
    new_entry.data_type         = DT_FLOAT32;
    new_entry.scale_factor      = 1.0f;
    strncpy(new_entry.device_id,        "flow_meter_01",       AMM_MAX_DEVICE_NAME_LEN - 1);
    strncpy(new_entry.point_id,         "mag_flow_rate",       AMM_MAX_POINT_NAME_LEN - 1);
    strncpy(new_entry.measurement_name, "Volumetric Flow Rate", AMM_MAX_POINT_NAME_LEN - 1);
    strncpy(new_entry.unit,             "L/min",               AMM_MAX_UNIT_LEN - 1);
    strncpy(new_entry.mqtt_topic,       "factory/line3/flow01/rate", AMM_MAX_TOPIC_LEN - 1);
    new_entry.constraint.writable          = false;
    new_entry.constraint.valid_range_min   = 0.0f;
    new_entry.constraint.valid_range_max   = 500.0f;
    new_entry.active = true;

    esp_err_t add_err = amm_add_mapping(&new_entry);

    int64_t add_end = esp_timer_get_time();
    double add_ms = (double)(add_end - add_start) / 1000.0;

    if (add_err == ESP_OK) {
        printf("  Mapping added successfully (adaptation latency: %.2f ms)\n", add_ms);
    } else {
        printf("  Mapping add FAILED: %s\n", esp_err_to_name(add_err));
    }

    int post_count = amm_get_mapping_count();
    printf("  Active mappings after add: %d (was %d)\n", post_count, pre_count);

    /* Step 3: Poll the new device */
    printf("\n  Polling new device (slave=3, reg=40010)...\n");

    sim_poll_entry_t new_poll = {
        .slave_id         = 3,
        .function_code    = 0x03,
        .register_address = 40010,
        .register_count   = 2,
        .data_type        = DT_FLOAT32,
    };

    /* The MODBUS mock doesn't know slave=3 reg=0x000A (40010 in decimal).
     * But it will return zeros for unknown registers, which is fine for
     * demonstrating the pipeline. The TCM lookup table DOES have this entry. */
    int ret = sim_poll_and_publish(&new_poll, NET_ONLINE, false, NULL);

    if (ret == 0) {
        printf("  New device polled and published successfully!\n");
    } else {
        printf("  New device poll pipeline completed (value may be 0 for unmocked register).\n");
    }

    /* Step 4: Verify the mapping is used for enrichment */
    amm_mapping_entry_t *found = amm_find_mapping(3, 40010);
    if (found) {
        printf("\n  Verified mapping:\n");
        printf("    device_id   : %s\n", found->device_id);
        printf("    point_id    : %s\n", found->point_id);
        printf("    measurement : %s\n", found->measurement_name);
        printf("    unit        : %s\n", found->unit);
        printf("    mqtt_topic  : %s\n", found->mqtt_topic);
    } else {
        printf("  WARNING: New mapping not found in registry!\n");
    }

    printf("\n  Adaptation latency: %.2f ms\n", add_ms);

    print_metrics_snapshot("After Scenario 5 (adaptive mapping)");
}

/* ======================== Main Entry Point ======================== */

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    /* Seed random number generator for MODBUS data variation */
    srand((unsigned int)time(NULL));

    /* Print simulation banner */
    print_banner();

    /* ============================================================
     * Step 1: Initialize NVS (required before AMM can persist data)
     * ============================================================ */
    printf("[INIT] Initializing NVS flash...\n");
    nvs_flash_init();

    /* ============================================================
     * Step 2: Initialize all core modules
     * ============================================================ */
    printf("[INIT] Initializing TCM module...\n");
    tcm_init();

    printf("[INIT] Initializing AMM module (loads/creates mapping table)...\n");
    amm_init();

    printf("[INIT] Initializing MQTT handler...\n");
    mqtt_init();

    printf("[INIT] Initializing UIF persistence layer...\n");
    uif_init();

    printf("[INIT] Initializing evaluation logger...\n");
    eval_init();

    /* ============================================================
     * Step 3: Initialize MODBUS RTU master
     * ============================================================ */
    printf("[INIT] Initializing MODBUS RTU master...\n");
    esp_err_t mb_err = modbus_rtu_init();
    if (mb_err != ESP_OK) {
        printf("[INIT] WARNING: MODBUS init failed: %s\n", esp_err_to_name(mb_err));
    }

    printf("\n[INIT] All modules initialized successfully.\n");
    printf("[INIT] Active AMM mappings: %d\n", amm_get_mapping_count());
    printf("[INIT] MQTT connected: %s\n", mqtt_is_connected() ? "YES" : "NO");
    printf("\n");

    /* ============================================================
     * Run Simulation Scenarios
     * ============================================================ */

    /* SCENARIO 1: Normal Operation */
    scenario_1_normal_operation();

    /* SCENARIO 2: MQTT Disconnection + Offline Caching */
    scenario_2_offline_caching();

    /* SCENARIO 3: Reconnection + Ordered Replay */
    scenario_3_reconnect_and_replay();

    /* SCENARIO 4: Downlink Command Validation */
    scenario_4_downlink_validation();

    /* SCENARIO 5: AMM Adaptive Mapping */
    scenario_5_adaptive_mapping();

    /* ============================================================
     * Final Comprehensive Metrics Summary
     * ============================================================ */
    printf("\n");
    printf("+================================================================+\n");
    printf("|           FINAL SIMULATION SUMMARY                             |\n");
    printf("+================================================================+\n");

    eval_print_summary();

    printf("\n  System Resources (simulated):\n");
    printf("    Free heap      : %u bytes\n", esp_get_free_heap_size());
    printf("    Min free heap  : %u bytes\n", esp_get_minimum_free_heap_size());
    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);
    printf("    Flash size     : %u bytes\n", flash_size);
    printf("    UIF cache usage: %d%% (%d records)\n",
           uif_get_cache_usage_percent(), uif_get_cached_count());
    printf("    Data loss      : %d events\n", uif_get_data_loss_count());
    printf("    MQTT published : %d messages\n", sim_mqtt_mock_publish_count());

    printf("\n+================================================================+\n");
    printf("|           SIMULATION COMPLETE                                  |\n");
    printf("+================================================================+\n\n");

    /* ============================================================
     * Cleanup
     * ============================================================ */
    modbus_destroy();
    mqtt_destroy();
    uif_destroy();

    return 0;
}
