#ifndef HISTORY_STORE_H
#define HISTORY_STORE_H

#include "esp_err.h"
#include "tcm/tcm_context.h"

esp_err_t history_store_init(void);
esp_err_t history_store_append(const tcm_context_t *context);
int history_store_deleted_files(void);

#endif
