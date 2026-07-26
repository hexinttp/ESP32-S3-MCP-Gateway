/**
 * @file gateway_config.h
 * @brief Global configuration constants for the MCP-Adapted MODBUS-MQTT Gateway
 */
#ifndef GATEWAY_CONFIG_H
#define GATEWAY_CONFIG_H

/* ======================== WiFi Configuration ======================== */
#define WIFI_SSID               "YOUR_WIFI_SSID"
#define WIFI_PASSWORD            "YOUR_WIFI_PASSWORD"
#define WIFI_MAX_RETRY           10

/* ======================== MQTT Configuration ======================== */
#define MQTT_BROKER_URI          "mqtt://192.168.1.100:1883"
#define MQTT_CLIENT_ID           "esp32s3_gateway_01"
#define MQTT_USERNAME            ""
#define MQTT_PASSWORD            ""
#define MQTT_KEEPALIVE_SEC       60
#define MQTT_PUBLISH_TIMEOUT_MS  5000
#define MQTT_CMD_TOPIC_PREFIX    "factory/cmd/"
#define MQTT_DATA_TOPIC_PREFIX   "factory/data/"

/* ======================== OneNET Cloud Platform Configuration ======================== */
#define ONENET_ENABLED           0
#define ONENET_MQTT_BROKER_URI   "mqtts://mqtt.heclouds.com:8883"
#define ONENET_PRODUCT_ID        "YOUR_PRODUCT_ID"
#define ONENET_DEVICE_ID         "YOUR_DEVICE_ID"
#define ONENET_DEVICE_KEY        "YOUR_DEVICE_KEY"
#define ONENET_TOPIC_FORMAT      1

/* ======================== MODBUS Configuration ======================== */
#define MODBUS_RTU_UART_PORT     UART_NUM_1
#define MODBUS_RTU_UART_TXD      39
#define MODBUS_RTU_UART_RXD      40
#define MODBUS_RTU_UART_RTS      41
#define MODBUS_RTU_BAUD_RATE     9600
#define MODBUS_RTU_PARITY        MB_PARITY_NONE

/* ======================== Polling Configuration ======================== */
#define POLL_INTERVAL_MS         1000
#define MAX_REGISTERS            32
#define MODBUS_TIMEOUT_MS        1000

/* ======================== TCM Context Configuration ======================== */
#define TCM_CONTEXT_ID_PREFIX    "ctx-"
#define TCM_MAX_FIELD_STR_LEN    64
#define TCM_MAX_JSON_LEN         1024
#define TCM_MANDATORY_FIELD_CNT  16

/* ======================== AMM Mapping Configuration ======================== */
#define AMM_MAX_MAPPING_ENTRIES  1000
#define AMM_FALLBACK_MAPPING_ENTRIES 64
#define AMM_NVS_PARTITION        "amm_nvs"
#define AMM_MAX_TOPIC_LEN        128
#define AMM_MAX_DEVICE_NAME_LEN  32
#define AMM_MAX_POINT_NAME_LEN   32
#define AMM_MAX_UNIT_LEN         16

/* ======================== UIF Cache Configuration ======================== */
#define UIF_CACHE_MAX_RECORDS    512
#define UIF_CACHE_PARTITION      "cache"
#define UIF_FLASH_WRITE_SIZE     TCM_MAX_JSON_LEN
#define UIF_REPLAY_INTERVAL_MS   100

/* ======================== Scheduler Configuration ======================== */
#define TASK_STACK_SIZE_MODBUS   4096
#define TASK_STACK_SIZE_TCM      4096
#define TASK_STACK_SIZE_MQTT     12288
#define TASK_STACK_SIZE_SCHED    4096
#define TASK_STACK_SIZE_EVAL     4096

#define TASK_PRIORITY_MODBUS    5
#define TASK_PRIORITY_TCM       4
#define TASK_PRIORITY_MQTT      6
#define TASK_PRIORITY_SCHED     7
#define TASK_PRIORITY_EVAL      2

/* ======================== Queue Configuration ======================== */
#define QUEUE_RAW_DATA_SIZE      8
#define QUEUE_CONTEXT_SIZE       4
#define QUEUE_MQTT_OUT_SIZE      4
#define QUEUE_CACHE_SIZE         32

/* ======================== Evaluation Configuration ======================== */
#define EVAL_LOG_INTERVAL_MS     5000

#endif /* GATEWAY_CONFIG_H */
