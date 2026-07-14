#ifndef AUTOMATION_WEB_H
#define AUTOMATION_WEB_H

#include "esp_err.h"
#include "esp_http_server.h"

esp_err_t automation_web_register(httpd_handle_t server);

#endif
