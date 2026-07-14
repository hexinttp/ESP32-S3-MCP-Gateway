#include <stdio.h>
#include <string.h>

#include "amm/amm_mapping.h"
#include "automation/automation_engine.h"
#include "board/board.h"
#include "board/lcd_st7735.h"
#include "board/tf_storage.h"
#include "config/runtime_config.h"
#include "eval/eval_logger.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "modbus/modbus_access.h"
#include "modbus/modbus_discover.h"
#include "mqtt_comm/mqtt_handler.h"
#include "network/network_manager.h"
#include "nvs_flash.h"
#include "scheduler/scheduler.h"
#include "storage/offline_store.h"
#include "storage/history_store.h"
#include "tcm/tcm_context.h"
#include "tcm/tcm_state_pool.h"
#include "services/control_service.h"
#include "uif/uif_persistence.h"
#include "web/web_server.h"
#include "esp_log.h"

static const char *TAG = "MAIN";

static void init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

static void mqtt_command_callback(const char *topic, const char *payload, int length)
{
    (void)topic;
    char json[TCM_MAX_JSON_LEN];
    int size = length < (int)sizeof(json) - 1 ? length : (int)sizeof(json) - 1;
    memcpy(json, payload, size);
    json[size] = '\0';
    tcm_context_t command;
    if (tcm_deserialize_json(json, &command) != 0) {
        eval_increment_metric("commands_rejected", 1);
        return;
    }
    control_result_t result;
    esp_err_t err = control_service_write_point(command.device_id, command.point_id,
                                                command.value, CONTROL_SOURCE_MQTT, &result);
    if (err != ESP_OK) ESP_LOGW(TAG, "Command rejected: %s", result.reason);
}

static void mqtt_publish_task(void *argument)
{
    QueueHandle_t queue = argument;
    mqtt_out_msg_t message;
    while (true) {
        if (xQueueReceive(queue, &message, portMAX_DELAY) != pdTRUE) continue;
        esp_err_t err = mqtt_is_connected()
            ? mqtt_publish(message.topic, message.payload, message.qos)
            : ESP_ERR_INVALID_STATE;
        if (err != ESP_OK) {
            offline_store_put(message.sequence_id, message.topic, message.payload);
            eval_increment_metric("mqtt_failed", 1);
        }
    }
}

static void lcd_menu_task(void *argument)
{
    (void)argument;
    int page = 0;
    while (true) {
        network_status_t network;
        network_manager_get_status(&network);
        char line1[40], line2[40], line3[40];
        if (page == 0) {
            snprintf(line1, sizeof(line1), "ETH %s", network.ethernet_ip ? network.ethernet_address : "OFFLINE");
            snprintf(line2, sizeof(line2), "WIFI %s", network.wifi_connected ? network.wifi_address : "OFFLINE");
            snprintf(line3, sizeof(line3), "MQTT %s", mqtt_is_connected() ? "ONLINE" : "OFFLINE");
            lcd_st7735_status("GATEWAY NETWORK", line1, line2, line3,
                              network_manager_is_online() ? LCD_COLOR_GREEN : LCD_COLOR_YELLOW);
        } else if (page == 1) {
            snprintf(line1, sizeof(line1), "POINTS %d", amm_get_mapping_count());
            snprintf(line2, sizeof(line2), "CACHE %d", uif_get_cached_count());
            snprintf(line3, sizeof(line3), "FLASH %d%%", uif_get_cache_usage_percent());
            lcd_st7735_status("TCM AMM UIF", line1, line2, line3, LCD_COLOR_CYAN);
        } else {
            snprintf(line1, sizeof(line1), "AP %s", network.config_ap_ssid);
            strlcpy(line2, "WEB 192.168.4.1", sizeof(line2));
            snprintf(line3, sizeof(line3), "TF %s", tf_storage_is_mounted() ? "READY" : "ABSENT");
            lcd_st7735_status("CONFIGURATION", line1, line2, line3, LCD_COLOR_BLUE);
        }
        page = (page + 1) % 3;
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 TCM/AMM/UIF gateway starting");
    init_nvs();
    ESP_ERROR_CHECK(runtime_config_init());
    ESP_ERROR_CHECK(board_init());
    runtime_config_t config;
    runtime_config_get(&config);
    if (config.lcd_enabled) {
        esp_err_t err = lcd_st7735_init();
        if (err != ESP_OK) ESP_LOGW(TAG, "LCD unavailable: %s", esp_err_to_name(err));
    }
    if (config.tf_enabled) tf_storage_mount();
    ESP_ERROR_CHECK(network_manager_init());

    tcm_init();
    amm_init();
    ESP_ERROR_CHECK(tcm_state_pool_init());
    ESP_ERROR_CHECK(history_store_init());
    ESP_ERROR_CHECK(automation_init());
    modbus_discover_init();
    eval_init();
    mqtt_init();
    ESP_ERROR_CHECK(uif_init());

    if (config.modbus_rtu.enabled) {
        esp_err_t err = modbus_rtu_init();
        board_set_rs485_ready(err == ESP_OK);
        if (err != ESP_OK) ESP_LOGW(TAG, "RS485 Modbus RTU unavailable: %s", esp_err_to_name(err));
    }

    QueueHandle_t raw = xQueueCreate(QUEUE_RAW_DATA_SIZE, sizeof(modbus_read_result_t));
    QueueHandle_t context = xQueueCreate(QUEUE_CONTEXT_SIZE, sizeof(tcm_context_t));
    QueueHandle_t mqtt_out = xQueueCreate(QUEUE_MQTT_OUT_SIZE, sizeof(mqtt_out_msg_t));
    QueueHandle_t mqtt_cmd = xQueueCreate(QUEUE_MQTT_CMD_SIZE, TCM_MAX_JSON_LEN);
    QueueHandle_t eval = xQueueCreate(QUEUE_EVAL_SIZE, sizeof(eval_event_t));
    if (!raw || !context || !mqtt_out || !mqtt_cmd || !eval) abort();

    scheduler_init(raw, context, mqtt_out, mqtt_cmd, eval);
    xTaskCreate(mqtt_publish_task, "mqtt_out", TASK_STACK_SIZE_MQTT, mqtt_out,
                TASK_PRIORITY_MQTT, NULL);
    char command_topic[128];
    snprintf(command_topic, sizeof(command_topic), "%s%s", config.mqtt.command_prefix,
             config.gateway_id);
    mqtt_subscribe(command_topic, 1, mqtt_command_callback);
    ESP_ERROR_CHECK(web_server_start(80));
    scheduler_start();
    ESP_ERROR_CHECK(automation_start());
    if (config.lcd_enabled && board_get_status()->lcd_ready) {
        xTaskCreate(lcd_menu_task, "lcd_menu", 4096, NULL, 2, NULL);
    }

    network_state_t previous = NET_OFFLINE;
    while (true) {
        network_state_t current = mqtt_is_connected() ? NET_ONLINE : NET_OFFLINE;
        if (current != previous) {
            scheduler_set_network_state(current);
            previous = current;
        }
        eval_print_summary();
        vTaskDelay(pdMS_TO_TICKS(EVAL_LOG_INTERVAL_MS));
    }
}
