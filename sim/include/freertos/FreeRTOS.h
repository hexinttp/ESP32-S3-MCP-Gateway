#pragma once

#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int BaseType_t;
typedef uint32_t TickType_t;
typedef unsigned int UBaseType_t;

#define pdTRUE   1
#define pdFALSE  0
#define pdPASS   1
#define pdFAIL   0

#define portMAX_DELAY    0xFFFFFFFF
#define configTICK_RATE_HZ  1000
#define pdMS_TO_TICKS(ms)   ((TickType_t)(ms))
#define portTICK_PERIOD_MS  1

#ifdef __cplusplus
}
#endif
