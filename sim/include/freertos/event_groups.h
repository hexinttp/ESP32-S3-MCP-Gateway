#pragma once

#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void* EventGroupHandle_t;
typedef uint32_t EventBits_t;

#define BIT0  0x01
#define BIT1  0x02

EventGroupHandle_t xEventGroupCreate(void);

EventBits_t xEventGroupSetBits(EventGroupHandle_t xEventGroup, EventBits_t uxBitsToSet);

EventBits_t xEventGroupWaitBits(
    EventGroupHandle_t xEventGroup,
    EventBits_t uxBitsToWaitFor,
    BaseType_t xClearOnExit,
    BaseType_t xWaitForAllBits,
    TickType_t xTicksToWait
);

void vEventGroupDelete(EventGroupHandle_t xEventGroup);

#ifdef __cplusplus
}
#endif
