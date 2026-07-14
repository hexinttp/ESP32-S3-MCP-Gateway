#pragma once
/*
 * Mock nvs_flash.h for PC simulation.
 */

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the NVS flash (mock: returns ESP_OK).
 */
esp_err_t nvs_flash_init(void);

/**
 * @brief Erase all NVS data (mock: clears in-memory store).
 */
esp_err_t nvs_flash_erase(void);

#ifdef __cplusplus
}
#endif
