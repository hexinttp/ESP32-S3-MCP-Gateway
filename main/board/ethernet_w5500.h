#ifndef ETHERNET_W5500_H
#define ETHERNET_W5500_H

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "esp_netif.h"

esp_err_t ethernet_w5500_init(void);
bool ethernet_w5500_is_linked(void);
bool ethernet_w5500_has_ip(void);
void ethernet_w5500_get_ip(char *buffer, size_t size);
esp_netif_t *ethernet_w5500_get_netif(void);

#endif
