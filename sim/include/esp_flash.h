#pragma once
/*
 * Mock esp_flash.h for PC simulation.
 */

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Return simulated flash chip size.
 * @param chip  Unused (may be NULL).
 * @param out_size  Receives the flash size in bytes.
 * @return ESP_OK.
 */
esp_err_t esp_flash_get_size(void *chip, uint32_t *out_size);

#ifdef __cplusplus
}
#endif
