#include "ethernet_w5500.h"

#include <string.h>
#include "board.h"
#include "board_pins.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_eth.h"
#include "esp_eth_mac_spi.h"
#include "esp_eth_phy.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"

static const char *TAG = "W5500";
static esp_eth_handle_t s_eth_handle;
static esp_netif_t *s_eth_netif;
static esp_eth_netif_glue_handle_t s_eth_glue;
static esp_eth_mac_t *s_mac;
static esp_eth_phy_t *s_phy;
static bool s_linked;
static bool s_has_ip;
static char s_ip[16] = "0.0.0.0";

static void eth_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;
    (void)data;
    if (id == ETHERNET_EVENT_CONNECTED) {
        s_linked = true;
        ESP_LOGI(TAG, "Ethernet link up");
    } else if (id == ETHERNET_EVENT_DISCONNECTED) {
        s_linked = false;
        s_has_ip = false;
        strcpy(s_ip, "0.0.0.0");
        ESP_LOGW(TAG, "Ethernet link down");
    } else if (id == ETHERNET_EVENT_STOP) {
        s_linked = false;
        s_has_ip = false;
    }
}

static void eth_ip_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;
    if (id != IP_EVENT_ETH_GOT_IP || data == NULL) {
        return;
    }
    ip_event_got_ip_t *event = data;
    snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&event->ip_info.ip));
    s_has_ip = true;
    ESP_LOGI(TAG, "Ethernet address: %s", s_ip);
}

esp_err_t ethernet_w5500_init(void)
{
    if (s_eth_handle != NULL) {
        return ESP_OK;
    }

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    spi_bus_config_t bus = {
        .mosi_io_num = BOARD_W5500_PIN_MOSI,
        .miso_io_num = BOARD_W5500_PIN_MISO,
        .sclk_io_num = BOARD_W5500_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    err = spi_bus_initialize(BOARD_W5500_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI2 init failed: %s", esp_err_to_name(err));
        return err;
    }

    spi_device_interface_config_t spi_device = {
        .mode = 0,
        .clock_speed_hz = 20 * 1000 * 1000,
        .spics_io_num = BOARD_W5500_PIN_CS,
        .queue_size = 20,
    };
    eth_w5500_config_t w5500 = ETH_W5500_DEFAULT_CONFIG(BOARD_W5500_SPI_HOST, &spi_device);
    w5500.int_gpio_num = BOARD_W5500_PIN_INT;
    w5500.poll_period_ms = 0;

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = 1;
    phy_config.reset_gpio_num = -1;

    s_mac = esp_eth_mac_new_w5500(&w5500, &mac_config);
    s_phy = esp_eth_phy_new_w5500(&phy_config);
    if (s_mac == NULL || s_phy == NULL) {
        ESP_LOGE(TAG, "Failed to create W5500 MAC/PHY");
        return ESP_ERR_NO_MEM;
    }

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(s_mac, s_phy);
    err = esp_eth_driver_install(&eth_config, &s_eth_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ethernet driver install failed: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t mac_addr[6];
    if (esp_read_mac(mac_addr, ESP_MAC_ETH) == ESP_OK) {
        esp_eth_ioctl(s_eth_handle, ETH_CMD_S_MAC_ADDR, mac_addr);
    }

    esp_netif_inherent_config_t netif_base = ESP_NETIF_INHERENT_DEFAULT_ETH();
    netif_base.route_prio = 150;
    esp_netif_config_t netif_config = {
        .base = &netif_base,
        .driver = NULL,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH,
    };
    s_eth_netif = esp_netif_new(&netif_config);
    if (s_eth_netif == NULL) {
        return ESP_ERR_NO_MEM;
    }
    s_eth_glue = esp_eth_new_netif_glue(s_eth_handle);
    ESP_RETURN_ON_ERROR(esp_netif_attach(s_eth_netif, s_eth_glue), TAG, "netif attach");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                                   eth_event_handler, NULL), TAG, "eth event");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                                   eth_ip_handler, NULL), TAG, "ip event");
    ESP_RETURN_ON_ERROR(esp_eth_start(s_eth_handle), TAG, "eth start");

    board_set_ethernet_ready(true);
    ESP_LOGI(TAG, "W5500 started on SPI2 (CS=%d INT=%d)",
             BOARD_W5500_PIN_CS, BOARD_W5500_PIN_INT);
    return ESP_OK;
}

bool ethernet_w5500_is_linked(void) { return s_linked; }
bool ethernet_w5500_has_ip(void) { return s_has_ip; }

void ethernet_w5500_get_ip(char *buffer, size_t size)
{
    if (buffer != NULL && size > 0) {
        strlcpy(buffer, s_ip, size);
    }
}

esp_netif_t *ethernet_w5500_get_netif(void) { return s_eth_netif; }
