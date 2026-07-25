/**
 * @file scheduler.h
 * @brief Scheduler module - manages FreeRTOS tasks for MODBUS polling,
 *        context building, MQTT publishing, replay, and resource monitoring
 */
#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "tcm/tcm_context.h"
#include "modbus/modbus_access.h"
#include "gateway_config.h"

/* ======================== Poll Configuration ======================== */

/**
 * @brief Configuration for a single MODBUS register poll
 */
typedef struct {
    uint8_t slave_id;             /* MODBUS slave/unit ID */
    uint8_t function_code;        /* MODBUS function code (0x03, 0x04, etc.) */
    uint16_t register_address;    /* Starting register address */
    uint16_t register_count;      /* Number of registers to read */
    data_type_t data_type;        /* Data type interpretation */
} modbus_poll_config_t;

/* modbus_read_result_t is provided by modbus/modbus_access.h */

/* ======================== API Functions ======================== */

/**
 * @brief Initialize the scheduler with queue handles and create FreeRTOS tasks
 * @param raw_q Queue for raw MODBUS read results (modbus_read_result_t)
 * @param ctx_q Queue for validated TCM contexts (tcm_context_t)
 * @param mqtt_out_q Queue for outbound MQTT messages (mqtt_out_msg_t)
 */
void scheduler_init(QueueHandle_t raw_q,
                    QueueHandle_t ctx_q,
                    QueueHandle_t mqtt_out_q);

/**
 * @brief Start all scheduler tasks
 */
void scheduler_start(void);

/**
 * @brief Stop and delete all scheduler tasks
 */
void scheduler_stop(void);

/**
 * @brief Pause or resume periodic MODBUS polling.
 *
 * Discovery uses this to get exclusive logical access to the bus without
 * stopping context, persistence, replay, or resource-monitor tasks.
 */
void scheduler_pause_modbus_polling(bool paused);

/**
 * @brief Get current network state
 * @return Current network_state_t value
 */
network_state_t scheduler_get_network_state(void);

/**
 * @brief Set network state (triggers replay on transition to ONLINE)
 * @param state New network state
 */
void scheduler_set_network_state(network_state_t state);

#endif /* SCHEDULER_H */
