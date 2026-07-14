#pragma once
/*
 * Mock esp_event.h for PC simulation.
 * Defines event base types, handler registration, and WiFi/IP event identifiers.
 */

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Event base type (pointer to a tag string, matching ESP-IDF convention) */
typedef const char *esp_event_base_t;

/* Opaque handle for registered event handler instances */
typedef void *esp_event_handler_instance_t;

/* Event handler callback type */
typedef void (*esp_event_handler_t)(void *event_handler_arg,
                                    esp_event_base_t event_base,
                                    int32_t event_id,
                                    void *event_data);

/* Sentinel for matching any event ID */
#define ESP_EVENT_ANY_ID    (-1)

/* Event base declarations */
extern const char *WIFI_EVENT;
extern const char *IP_EVENT;

/* WiFi event IDs */
typedef enum {
    WIFI_EVENT_STA_START = 0,
    WIFI_EVENT_STA_STOP,
    WIFI_EVENT_STA_CONNECTED,
    WIFI_EVENT_STA_DISCONNECTED,
} wifi_event_t;

/* IP event IDs */
typedef enum {
    IP_EVENT_STA_GOT_IP = 0,
} ip_event_t;

/**
 * @brief IP address and related info (matches esp_netif layout).
 */
typedef struct {
    struct {
        uint32_t addr;     /* IPv4 address in host byte order */
    } ip;
} esp_netif_ip_info_t;

/**
 * @brief Event data for IP_EVENT_STA_GOT_IP.
 */
typedef struct {
    esp_netif_ip_info_t ip_info;
} ip_event_got_ip_t;

/* Macros for printing IP addresses */
#define IPSTR   "%d.%d.%d.%d"
#define IP2STR(ip)  (int)((ip)->addr & 0xFF), \
                    (int)(((ip)->addr >> 8) & 0xFF), \
                    (int)(((ip)->addr >> 16) & 0xFF), \
                    (int)(((ip)->addr >> 24) & 0xFF)

/* Function declarations */
esp_err_t esp_event_loop_create_default(void);

esp_err_t esp_event_handler_instance_register(esp_event_base_t event_base,
                                               int32_t event_id,
                                               esp_event_handler_t event_handler,
                                               void *event_handler_arg,
                                               esp_event_handler_instance_t *context);

#ifdef __cplusplus
}
#endif
