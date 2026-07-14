#pragma once
/*
 * Mock esp_netif.h for PC simulation.
 */

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Opaque handle for a network interface */
typedef void *esp_netif_t;

esp_err_t esp_netif_init(void);
esp_netif_t *esp_netif_create_default_wifi_sta(void);

#ifdef __cplusplus
}
#endif
