/**
 * @file web_server.h
 * @brief Lightweight HTTP server for gateway web configuration.
 *
 * Serves a single-page HTML configuration UI and provides RESTful JSON
 * API endpoints for reading/writing all gateway settings at runtime.
 *
 * Endpoints:
 *   GET  /                    → Serve embedded HTML page
 *   GET  /api/system/status   → System metrics (heap, uptime, counters)
 *   GET  /api/system/logs     → Recent log entries (JSON array)
 *   GET  /api/wifi/config     → Current WiFi settings
 *   PUT  /api/wifi/config     → Update WiFi settings
 *   GET  /api/mqtt/config     → Current MQTT settings
 *   PUT  /api/mqtt/config     → Update MQTT settings
 *   GET  /api/modbus/config   → Current MODBUS settings
 *   PUT  /api/modbus/config   → Update MODBUS settings
 *   GET  /api/mappings        → List all AMM mapping entries
 *   POST /api/mappings        → Add new mapping entry
 *   PUT  /api/mappings/:idx   → Update existing mapping entry
 *   DELETE /api/mappings/:idx → Remove mapping entry
 *   GET  /api/discover/status  → Discovery scan status
 *   GET  /api/discover/devices → List discovered devices
 *   POST /api/discover/scan    → Start bus scan
 *   POST /api/discover/apply   → Apply discovered as AMM mappings
 *   POST /api/discover/reset   → Reset discovery state
 *   PUT  /api/discover/devices/:id           → Update device info
 *   PUT  /api/discover/devices/:id/registers/:addr → Update register
 *   POST /api/discover/devices/:id/registers/:addr/toggle → Toggle register
 *   DELETE /api/discover/devices/:id/registers/:addr → Delete register
 */
#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the HTTP server on the given port.
 *
 * Registers all URI handlers and begins listening.  The server runs in its
 * own FreeRTOS task managed by esp_http_server internally.
 *
 * @param port  TCP port to listen on (typically 80).
 * @return ESP_OK on success.
 */
esp_err_t web_server_start(uint16_t port);

/**
 * @brief Stop and destroy the HTTP server, freeing all resources.
 */
void web_server_stop(void);

/**
 * @brief Add a log entry to the web server's ring buffer.
 *
 * Call this from other modules (TCM, AMM, MQTT, etc.) to make their
 * log messages visible in the web UI's "System Logs" page.
 *
 * @param level_str  Log level string: "info", "warn", "error", or "ok".
 * @param text       Log message text (will be truncated to 127 chars).
 */
void web_server_add_log(const char *level_str, const char *text);

#ifdef __cplusplus
}
#endif

#endif /* WEB_SERVER_H */
