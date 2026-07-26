#ifndef MODBUS_TCP_SERVER_H
#define MODBUS_TCP_SERVER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    bool running;
    uint16_t port;
    uint8_t active_clients;
    uint32_t requests;
    uint32_t exceptions;
    uint32_t writes;
} modbus_tcp_server_status_t;

esp_err_t modbus_tcp_server_start(void);
void modbus_tcp_server_stop(void);
void modbus_tcp_server_get_status(modbus_tcp_server_status_t *out);

#endif
