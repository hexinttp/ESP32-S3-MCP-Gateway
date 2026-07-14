#include "tcm/tcm_state_pool.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "gateway_config.h"

static tcm_context_t s_states[AMM_MAX_MAPPING_ENTRIES];
static int s_count;
static SemaphoreHandle_t s_mutex;

esp_err_t tcm_state_pool_init(void)
{
    if (s_mutex == NULL) s_mutex = xSemaphoreCreateMutex();
    return s_mutex == NULL ? ESP_ERR_NO_MEM : ESP_OK;
}

void tcm_state_pool_update(const tcm_context_t *context)
{
    if (context == NULL || s_mutex == NULL) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int slot = -1;
    int oldest = 0;
    for (int i = 0; i < s_count; ++i) {
        if (strcmp(s_states[i].device_id, context->device_id) == 0 &&
            strcmp(s_states[i].point_id, context->point_id) == 0) {
            slot = i;
            break;
        }
        if (s_states[i].timestamp_ms < s_states[oldest].timestamp_ms) oldest = i;
    }
    if (slot < 0) {
        slot = s_count < AMM_MAX_MAPPING_ENTRIES ? s_count++ : oldest;
    }
    s_states[slot] = *context;
    xSemaphoreGive(s_mutex);
}

esp_err_t tcm_state_pool_get(const char *device_id, const char *point_id,
                             tcm_context_t *out)
{
    if (device_id == NULL || point_id == NULL || out == NULL || s_mutex == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t result = ESP_ERR_NOT_FOUND;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < s_count; ++i) {
        if (strcmp(s_states[i].device_id, device_id) == 0 &&
            strcmp(s_states[i].point_id, point_id) == 0) {
            *out = s_states[i];
            result = ESP_OK;
            break;
        }
    }
    xSemaphoreGive(s_mutex);
    return result;
}

int tcm_state_pool_snapshot(tcm_context_t *out, int max_items)
{
    if (out == NULL || max_items <= 0 || s_mutex == NULL) return 0;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int count = s_count < max_items ? s_count : max_items;
    memcpy(out, s_states, count * sizeof(*out));
    xSemaphoreGive(s_mutex);
    return count;
}
