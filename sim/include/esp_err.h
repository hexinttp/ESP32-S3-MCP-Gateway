#pragma once
/*
 * Mock esp_err.h for PC simulation.
 * Defines ESP-IDF error codes and the esp_err_t type.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int esp_err_t;

/* Success */
#define ESP_OK                          0

/* General failure */
#define ESP_FAIL                       -1

/* Standard error codes (matching ESP-IDF values) */
#define ESP_ERR_NO_MEM                  0x101
#define ESP_ERR_INVALID_ARG             0x102
#define ESP_ERR_INVALID_STATE           0x103
#define ESP_ERR_INVALID_SIZE            0x104
#define ESP_ERR_NOT_FOUND               0x105
#define ESP_ERR_NOT_SUPPORTED           0x106
#define ESP_ERR_TIMEOUT                 0x107

/* NVS-specific error codes */
#define ESP_ERR_NVS_BASE                0x1100
#define ESP_ERR_NVS_NO_FREE_PAGES       (ESP_ERR_NVS_BASE + 0x10)
#define ESP_ERR_NVS_NEW_VERSION_FOUND   (ESP_ERR_NVS_BASE + 0x11)
#define ESP_ERR_NVS_NOT_FOUND           (ESP_ERR_NVS_BASE + 0x02)

/**
 * @brief Convert an esp_err_t value to its human-readable name string.
 */
const char *esp_err_to_name(esp_err_t code);

/**
 * @brief ESP_ERROR_CHECK: in real ESP-IDF this aborts on error.
 *        For simulation we just evaluate the expression and ignore the result.
 */
#define ESP_ERROR_CHECK(x) do {                       \
    esp_err_t __err_rc = (x);                         \
    (void)__err_rc;                                   \
} while(0)

#ifdef __cplusplus
}
#endif
