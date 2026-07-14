#pragma once
/*
 * Mock esp_timer.h for PC simulation.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Return monotonic time in microseconds since an arbitrary epoch.
 *        Uses QueryPerformanceCounter on Windows, clock_gettime on Linux.
 */
int64_t esp_timer_get_time(void);

#ifdef __cplusplus
}
#endif
