#ifndef BOARD_PINS_H
#define BOARD_PINS_H

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/uart.h"

/* W5500 uses SPI2. GPIO15 is a board-level shared reset with the LCD. */
#define BOARD_W5500_SPI_HOST       SPI2_HOST
#define BOARD_W5500_PIN_CS         GPIO_NUM_10
#define BOARD_W5500_PIN_MOSI       GPIO_NUM_11
#define BOARD_W5500_PIN_SCLK       GPIO_NUM_12
#define BOARD_W5500_PIN_MISO       GPIO_NUM_13
#define BOARD_W5500_PIN_INT        GPIO_NUM_14
#define BOARD_SHARED_RESET_PIN     GPIO_NUM_15

/* On-board ST7735S 1.77-inch LCD. */
#define BOARD_LCD_SPI_HOST         SPI3_HOST
#define BOARD_LCD_PIN_RST          BOARD_SHARED_RESET_PIN
#define BOARD_LCD_PIN_DC           GPIO_NUM_16
#define BOARD_LCD_PIN_MOSI         GPIO_NUM_17
#define BOARD_LCD_PIN_SCLK         GPIO_NUM_18
#define BOARD_LCD_PIN_CS           GPIO_NUM_21
#define BOARD_LCD_WIDTH            128
#define BOARD_LCD_HEIGHT           160

/* On-board TF card in native SDMMC 4-bit mode. */
#define BOARD_TF_PIN_D2            GPIO_NUM_33
#define BOARD_TF_PIN_D3            GPIO_NUM_34
#define BOARD_TF_PIN_CMD           GPIO_NUM_35
#define BOARD_TF_PIN_CLK           GPIO_NUM_36
#define BOARD_TF_PIN_D0            GPIO_NUM_37
#define BOARD_TF_PIN_D1            GPIO_NUM_38

/* External RS485 transceiver, UART1 with automatic RTS direction control. */
#define BOARD_RS485_UART           UART_NUM_1
#define BOARD_RS485_PIN_TX         GPIO_NUM_39
#define BOARD_RS485_PIN_RX         GPIO_NUM_40
#define BOARD_RS485_PIN_DE_RE      GPIO_NUM_41

/* Reserved by board functions. */
#define BOARD_UART0_PIN_TX         GPIO_NUM_43
#define BOARD_UART0_PIN_RX         GPIO_NUM_44
#define BOARD_USB_PIN_DM           GPIO_NUM_19
#define BOARD_USB_PIN_DP           GPIO_NUM_20

#endif
