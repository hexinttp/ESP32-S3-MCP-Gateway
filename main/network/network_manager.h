#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

typedef struct {
    bool ethernet_link;
    bool ethernet_ip;
    bool wifi_connected;
    bool config_ap_active;
    bool ethernet_preferred;
    uint32_t failover_count;
    char ethernet_address[16];
    char wifi_address[16];
    char config_ap_ssid[33];
    char active_uplink[12];
} network_status_t;

esp_err_t network_manager_init(void);
bool network_manager_is_online(void);
void network_manager_get_status(network_status_t *out);

#endif
