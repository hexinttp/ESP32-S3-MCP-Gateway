#pragma once
/*
 * Mock nvs.h for PC simulation.
 * Provides an in-memory key-value store as NVS substitute.
 */

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** NVS handle type */
typedef uint32_t nvs_handle_t;

/** NVS open mode */
typedef enum {
    NVS_READONLY  = 0,
    NVS_READWRITE = 1,
} nvs_open_mode_t;

esp_err_t nvs_open(const char *namespace_name, nvs_open_mode_t open_mode,
                    nvs_handle_t *out_handle);
void      nvs_close(nvs_handle_t handle);
esp_err_t nvs_commit(nvs_handle_t handle);

/* Integer operations */
esp_err_t nvs_set_i32(nvs_handle_t handle, const char *key, int32_t value);
esp_err_t nvs_get_i32(nvs_handle_t handle, const char *key, int32_t *out_value);

/* Blob (arbitrary binary) operations */
esp_err_t nvs_set_blob(nvs_handle_t handle, const char *key,
                        const void *value, size_t length);
esp_err_t nvs_get_blob(nvs_handle_t handle, const char *key,
                        void *out_value, size_t *length);

#ifdef __cplusplus
}
#endif
