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
        char value1[24], value2[24], value3[24];
        const char *title;
        const char *label1;
        const char *label2;
        const char *label3;
        uint16_t accent;

        if (page == 0) {
            title = "NETWORK";
            label1 = "ETHERNET";
            label2 = "WI-FI";
            label3 = "MQTT";
            strlcpy(value1, network.ethernet_ip ? "ONLINE" :
                    (network.ethernet_link ? "LINK" : "OFFLINE"), sizeof(value1));
            strlcpy(value2, network.wifi_connected ? "ONLINE" : "OFFLINE", sizeof(value2));
            strlcpy(value3, mqtt_is_connected() ? "ONLINE" : "OFFLINE", sizeof(value3));
            accent = network_manager_is_online() ? LCD_COLOR_GREEN : LCD_COLOR_YELLOW;
        } else if (page == 1) {
            title = "IP ADDRESS";
            label1 = "ETH IP";
            label2 = "WIFI IP";
            label3 = "WEB AP";
            strlcpy(value1, network.ethernet_ip ? network.ethernet_address : "NO ADDRESS", sizeof(value1));
            strlcpy(value2, network.wifi_connected ? network.wifi_address : "NO ADDRESS", sizeof(value2));
            strlcpy(value3, "192.168.4.1", sizeof(value3));
            accent = LCD_COLOR_CYAN;
        } else if (page == 2) {
            title = "GATEWAY";
            label1 = "POINTS";
            label2 = "CACHE";
            label3 = "FLASH USED";
            snprintf(value1, sizeof(value1), "%d", amm_get_mapping_count());
            snprintf(value2, sizeof(value2), "%d", uif_get_cached_count());
            snprintf(value3, sizeof(value3), "%d%%", uif_get_cache_usage_percent());
            accent = LCD_COLOR_CYAN;
        } else {
            title = "SYSTEM";
            label1 = "TF CARD";
            label2 = "RS485";
            label3 = "LANGUAGE";
            strlcpy(value1, tf_storage_is_mounted() ? "READY" : "ABSENT", sizeof(value1));
            strlcpy(value2, board_get_status()->rs485_ready ? "READY" : "OFFLINE", sizeof(value2));
            strlcpy(value3, runtime_config_get_locale() == UI_LOCALE_ZH_CN ?
                    "ZH-CN" : "EN-US", sizeof(value3));
            accent = LCD_COLOR_ORANGE;
        }

        lcd_st7735_dashboard(title, label1, value1, label2, value2,
                             label3, value3, accent);
        page = (page + 1) % 4;
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
    if (!raw || !context || !mqtt_out) {
        ESP_LOGE(TAG, "Pipeline queue allocation failed: raw=%p context=%p mqtt=%p",
                 raw, context, mqtt_out);
        abort();
    }

    scheduler_init(raw, context, mqtt_out);
    xTaskCreate(mqtt_publish_task, "mqtt_out", 4096, mqtt_out,
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
