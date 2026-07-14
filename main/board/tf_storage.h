#ifndef TF_STORAGE_H
#define TF_STORAGE_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define TF_MOUNT_POINT "/tf"

esp_err_t tf_storage_mount(void);
bool tf_storage_is_mounted(void);
esp_err_t tf_storage_get_space(uint64_t *total_bytes, uint64_t *free_bytes);

#endif
