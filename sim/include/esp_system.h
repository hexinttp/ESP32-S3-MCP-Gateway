#pragma once
/*
 * Mock esp_system.h for PC simulation.
 */

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Return simulated free heap size in bytes.
 */
uint32_t esp_get_free_heap_size(void);

/**
 * @brief Return simulated minimum-ever free heap size in bytes.
 */
uint32_t esp_get_minimum_free_heap_size(void);

/**
 * @brief Simulate a system restart (prints message and calls exit).
 */
void esp_restart(void);

#ifdef __cplusplus
}
#endif
