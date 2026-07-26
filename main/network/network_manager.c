#include "network_manager.h"

#include <string.h>
#include "board/ethernet_w5500.h"
#include "config/runtime_config.h"
#include "esp_event.h"
#include "esp_eth.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "NETWORK";
static network_status_t s_status;
static SemaphoreHandle_t s_mutex;
static esp_netif_t *s_sta_netif;
static bool s_prefer_ethernet;

static void select_uplink(bool ethernet)
{
    esp_netif_t *target = ethernet ? ethernet_w5500_get_netif() : s_sta_netif;
    if (target == NULL || esp_netif_set_default_netif(target) != ESP_OK) return;
    const char *name = ethernet ? "ethernet" : "wifi";
    if (strcmp(s_status.active_uplink, name) != 0) {
        if (s_status.active_uplink[0] != '\0') ++s_status.failover_count;
        strlcpy(s_status.active_uplink, name, sizeof(s_status.active_uplink));
        ESP_LOGI(TAG, "Active uplink: %s", name);
    }
}

static void route_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)data;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (base == IP_EVENT && id == IP_EVENT_ETH_GOT_IP && s_prefer_ethernet) {
        select_uplink(true);
    } else if (base == ETH_EVENT && id == ETHERNET_EVENT_DISCONNECTED &&
               s_status.wifi_connected) {
        select_uplink(false);
    }
    xSemaphoreGive(s_mutex);
}

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)data;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_status.wifi_connected = false;
        strcpy(s_status.wifi_address, "0.0.0.0");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = data;
        snprintf(s_status.wifi_address, sizeof(s_status.wifi_address),
                 IPSTR, IP2STR(&event->ip_info.ip));
        s_status.wifi_connected = true;
        if (!s_prefer_ethernet || !ethernet_w5500_has_ip()) {
            select_uplink(false);
        }
        ESP_LOGI(TAG, "WiFi address: %s", s_status.wifi_address);
    }
    xSemaphoreGive(s_mutex);
}

esp_err_t network_manager_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) return ESP_ERR_NO_MEM;
    strcpy(s_status.ethernet_address, "0.0.0.0");
    strcpy(s_status.wifi_address, "0.0.0.0");

    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    runtime_config_t config;
    runtime_config_get(&config);
    s_prefer_ethernet = config.prefer_ethernet;
    s_status.ethernet_preferred = s_prefer_ethernet;

    err = ethernet_w5500_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "W5500 unavailable; WiFi/AP remains available: %s", esp_err_to_name(err));
    }
    ESP_RETURN_ON_ERROR(esp_event_handler_register(ETH_EVENT,
                                                   ETHERNET_EVENT_DISCONNECTED,
                                                   route_event, NULL),
                        TAG, "ethernet route event");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT,
                                                   IP_EVENT_ETH_GOT_IP,
                                                   route_event, NULL),
                        TAG, "ethernet ip route event");

    s_sta_netif = esp_netif_create_default_wifi_sta();
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    if (s_sta_netif == NULL || ap_netif == NULL) return ESP_ERR_NO_MEM;

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init), TAG, "wifi init");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                   wifi_event, NULL), TAG, "wifi event");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                   wifi_event, NULL), TAG, "wifi ip event");

    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(s_status.config_ap_ssid, sizeof(s_status.config_ap_ssid),
             "MCP-Gateway-%02X%02X", mac[4], mac[5]);
    wifi_config_t ap = {0};
    strlcpy((char *)ap.ap.ssid, s_status.config_ap_ssid, sizeof(ap.ap.ssid));
    ap.ap.ssid_len = strlen(s_status.config_ap_ssid);
    ap.ap.channel = 1;
    ap.ap.max_connection = 4;
    ap.ap.authmode = WIFI_AUTH_OPEN;
    wifi_config_t sta = {0};
    if (config.wifi.enabled && strcmp(config.wifi.ssid, "YOUR_WIFI_SSID") != 0) {
        strlcpy((char *)sta.sta.ssid, config.wifi.ssid, sizeof(sta.sta.ssid));
        strlcpy((char *)sta.sta.password, config.wifi.password, sizeof(sta.sta.password));
        sta.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    }
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_APSTA), TAG, "wifi mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &ap), TAG, "AP config");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &sta), TAG, "STA config");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start");
    ESP_RETURN_ON_ERROR(esp_wifi_set_ps(WIFI_PS_NONE), TAG, "disable WiFi power save");

    s_status.config_ap_active = true;
    ESP_LOGI(TAG, "Configuration AP active: %s (http://192.168.4.1)", s_status.config_ap_ssid);
    return ESP_OK;
}

bool network_manager_is_online(void)
{
    return ethernet_w5500_has_ip() || s_status.wifi_connected;
}

void network_manager_get_status(network_status_t *out)
{
    if (out == NULL) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_status;
    xSemaphoreGive(s_mutex);
    out->ethernet_link = ethernet_w5500_is_linked();
    out->ethernet_ip = ethernet_w5500_has_ip();
    ethernet_w5500_get_ip(out->ethernet_address, sizeof(out->ethernet_address));
}
