#pragma once
/*
 * Mock esp_log.h for PC simulation.
 * Maps ESP_LOGx macros to printf-based output.
 */

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ESP_LOG_NONE,
    ESP_LOG_ERROR,
    ESP_LOG_WARN,
    ESP_LOG_INFO,
    ESP_LOG_DEBUG,
    ESP_LOG_VERBOSE
} esp_log_level_t;

/* Simulation logging macros - map to printf with level/tag prefix */
#define ESP_LOGE(tag, fmt, ...)  fprintf(stderr, "[E][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...)  fprintf(stdout, "[W][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...)  fprintf(stdout, "[I][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...)  ((void)0)
#define ESP_LOGV(tag, fmt, ...)  ((void)0)

/**
 * @brief Set log level for a given tag (stub - no-op in simulation).
 */
void esp_log_level_set(const char *tag, esp_log_level_t level);

#ifdef __cplusplus
}
#endif
