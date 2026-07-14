#ifndef TCM_STATE_POOL_H
#define TCM_STATE_POOL_H

#include "esp_err.h"
#include "tcm/tcm_context.h"

esp_err_t tcm_state_pool_init(void);
void tcm_state_pool_update(const tcm_context_t *context);
esp_err_t tcm_state_pool_get(const char *device_id, const char *point_id,
                             tcm_context_t *out);
int tcm_state_pool_snapshot(tcm_context_t *out, int max_items);

#endif
