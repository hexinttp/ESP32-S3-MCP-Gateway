#pragma once
/*
 * Mock driver/uart.h for PC simulation.
 * Only defines the UART port enum used by gateway_config.h.
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UART_NUM_0 = 0,
    UART_NUM_1 = 1,
    UART_NUM_2 = 2,
} uart_port_t;

#ifdef __cplusplus
}
#endif
