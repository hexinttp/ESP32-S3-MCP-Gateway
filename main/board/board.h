#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    bool lcd_ready;
    bool ethernet_ready;
    bool tf_card_ready;
    bool rs485_ready;
} board_status_t;

esp_err_t board_init(void);
const board_status_t *board_get_status(void);
void board_shared_reset(void);
void board_set_lcd_ready(bool ready);
void board_set_ethernet_ready(bool ready);
void board_set_tf_card_ready(bool ready);
void board_set_rs485_ready(bool ready);

#endif
