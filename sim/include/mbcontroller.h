#pragma once
/*
 * Mock mbcontroller.h for PC simulation.
 * Defines the freemodbus master controller API types and functions.
 */

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Port type enumeration ---- */
typedef enum {
    MB_PORT_SERIAL_MASTER = 0,
    MB_PORT_TCP_MASTER,
} mb_port_type_t;

/* ---- Register type enumeration ---- */
typedef enum {
    MB_PARAM_HOLDING = 0,
    MB_PARAM_INPUT,
} mb_reg_type_t;

/* ---- Communication mode ---- */
typedef enum {
    MB_MODE_RTU = 0,
    MB_MODE_TCP,
    MB_MODE_ASCII,
} mb_mode_t;

/* ---- Parity ---- */
typedef enum {
    MB_PARITY_NONE = 0,
    MB_PARITY_EVEN,
    MB_PARITY_ODD,
} mb_parity_t;

/* ---- Command type ---- */
typedef enum {
    MB_CMD_READ_HOLDING = 0,
    MB_CMD_READ_INPUT,
    MB_CMD_WRITE_HOLDING,
    MB_CMD_WRITE_MULTIPLE,
} mb_cmd_type_t;

/* ---- Parameter type ---- */
typedef enum {
    PARAM_TYPE_U8 = 0,
    PARAM_TYPE_U16_REG,
    PARAM_TYPE_U32,
    PARAM_TYPE_FLOAT,
} mb_param_type_t;

/* ---- Access permissions ---- */
#define PAR_PERMS_READ          0x01
#define PAR_PERMS_WRITE         0x02
#define PAR_PERMS_READ_WRITE    0x03

/* ---- Option structure ---- */
typedef struct {
    int32_t opt1;
    int32_t opt2;
    int32_t opt3;
} mb_parameter_opt_t;

/* ---- Parameter descriptor ---- */
typedef struct {
    uint16_t            cid;
    const char         *param_key;
    const char         *param_units;
    mb_parameter_opt_t  param_opts;
    mb_reg_type_t       reg_type;
    uint16_t            reg_size;
    uint16_t            param_addr;
    mb_param_type_t     param_type;
    uint16_t            param_size;
    uint16_t            param_offset;
    void               *param_desc;
    void               *param_value;
    uint8_t             access;
} mb_parameter_descriptor_t;

/* ---- Request parameter block ---- */
typedef struct {
    uint8_t                     slave_addr;
    mb_reg_type_t               mb_reg_type;
    uint16_t                    reg_start;
    uint16_t                    reg_size;
    mb_cmd_type_t               command;
    mb_parameter_descriptor_t  *param_descriptor;
} mb_request_param_t;

/* ---- Communication info (union for RTU/TCP) ---- */
typedef struct {
    /* RTU fields */
    uint8_t     port;
    mb_mode_t   mode;
    uint32_t    baudrate;
    mb_parity_t parity;
    /* TCP fields */
    char       *ip_addr;
    uint16_t    ip_port;
} mb_communication_info_t;

/* ---- UART pin config (used by mbc_master_set_pin) ---- */
typedef struct {
    int tx_io_num;
    int rx_io_num;
    int rts_io_num;
    int cts_io_num;
} uart_pin_config_t;

/* ---- Master controller API ---- */

esp_err_t mbc_master_init(mb_port_type_t port_type, void **handler);
esp_err_t mbc_master_setup(const mb_communication_info_t *comm_info);
esp_err_t mbc_master_set_pin(const uart_pin_config_t *pin_config);
esp_err_t mbc_master_start(void);
esp_err_t mbc_master_stop(void);
esp_err_t mbc_master_destroy(void);
void      mbc_master_set_timeout(uint32_t timeout_ms);

/**
 * @brief Send a MODBUS request and fill the parameter descriptor with response data.
 *
 * In the mock, this generates simulated sensor data based on slave_addr and reg_start.
 */
esp_err_t mbc_master_send_request(mb_request_param_t *req,
                                   mb_parameter_descriptor_t *reg_info);

#ifdef __cplusplus
}
#endif
