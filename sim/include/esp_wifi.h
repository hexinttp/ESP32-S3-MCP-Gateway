#pragma once
/*
 * Mock esp_wifi.h for PC simulation.
 * Defines WiFi configuration types and API stubs.
 */

#include "esp_err.h"
#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

/* WiFi mode */
typedef enum {
    WIFI_MODE_NULL = 0,
    WIFI_MODE_STA,
    WIFI_MODE_AP,
    WIFI_MODE_APSTA,
} wifi_mode_t;

/* WiFi interface */
typedef enum {
    WIFI_IF_STA = 0,
    WIFI_IF_AP,
} wifi_interface_t;

/* WiFi auth mode */
typedef enum {
    WIFI_AUTH_OPEN = 0,
    WIFI_AUTH_WEP,
    WIFI_AUTH_WPA_PSK,
    WIFI_AUTH_WPA2_PSK,
    WIFI_AUTH_WPA_WPA2_PSK,
} wifi_auth_mode_t;

/* WiFi STA configuration */
typedef struct {
    char ssid[32];
    char password[64];
    struct {
        wifi_auth_mode_t authmode;
    } threshold;
} wifi_sta_config_t;

/* WiFi AP configuration (minimal stub) */
typedef struct {
    char ssid[32];
    char password[64];
} wifi_ap_config_t;

/* Union-based WiFi configuration */
typedef union {
    wifi_ap_config_t  ap;
    wifi_sta_config_t sta;
} wifi_config_t;

/* WiFi init config */
typedef struct {
    int dummy;
} wifi_init_config_t;

/**
 * Macro to produce a default wifi_init_config_t.
 */
#define WIFI_INIT_CONFIG_DEFAULT()  ((wifi_init_config_t){0})

/* Function declarations (all stubs returning ESP_OK) */
esp_err_t esp_wifi_init(const wifi_init_config_t *config);
esp_err_t esp_wifi_set_mode(wifi_mode_t mode);
esp_err_t esp_wifi_set_config(wifi_interface_t iface, wifi_config_t *conf);
esp_err_t esp_wifi_start(void);
esp_err_t esp_wifi_stop(void);
esp_err_t esp_wifi_connect(void);
esp_err_t esp_wifi_disconnect(void);

#ifdef __cplusplus
}
#endif
