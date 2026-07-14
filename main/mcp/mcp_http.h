#ifndef MCP_HTTP_H
#define MCP_HTTP_H

#include "esp_err.h"
#include "esp_http_server.h"

esp_err_t mcp_http_register(httpd_handle_t server);

#endif
