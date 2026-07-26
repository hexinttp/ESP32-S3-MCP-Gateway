idf_component_register(
    SRCS "main.c"
         "board/board.c"
         "board/ethernet_w5500.c"
         "board/lcd_st7735.c"
         "board/tf_storage.c"
         "config/runtime_config.c"
         "network/network_manager.c"
         "storage/offline_store.c"
         "storage/history_store.c"
         "tcm/tcm_context.c"
         "tcm/tcm_state_pool.c"
         "modbus/modbus_access.c"
         "modbus/modbus_comm_log.c"
         "modbus/modbus_discover.c"
         "modbus/modbus_tcp_server.c"
         "amm/amm_mapping.c"
         "mqtt_comm/mqtt_handler.c"
         "uif/uif_persistence.c"
         "scheduler/scheduler.c"
         "eval/eval_logger.c"
         "services/control_service.c"
         "services/time_service.c"
         "services/health_service.c"
         "services/ota_service.c"
         "automation/automation_engine.c"
         "automation/automation_web.c"
         "mcp/mcp_http.c"
         "web/web_server.c"
    INCLUDE_DIRS "." "web"
    REQUIRES driver esp_wifi esp_event esp_netif esp_eth lwip nvs_flash mqtt esp-modbus esp_timer
             fatfs wear_levelling sdmmc esp_driver_sdmmc esp_http_server json esp_psram
             app_update esp_https_ota mbedtls efuse
)

set(WEB_HTML "${CMAKE_CURRENT_LIST_DIR}/../web/web_config.html")
set(WEB_HTML_GZIP "${CMAKE_CURRENT_BINARY_DIR}/web_config.html.gz")
set(LAB_PROFILE "${CMAKE_CURRENT_LIST_DIR}/../profiles/lab_rtu_146_points.json")
set(LAB_PROFILE_EMBED "${CMAKE_CURRENT_BINARY_DIR}/lab_profile.json")

configure_file("${LAB_PROFILE}" "${LAB_PROFILE_EMBED}" COPYONLY)

add_custom_command(
    OUTPUT "${WEB_HTML_GZIP}"
    COMMAND "${PYTHON}" "${CMAKE_CURRENT_LIST_DIR}/gzip_asset.py"
            "${WEB_HTML}" "${WEB_HTML_GZIP}"
    DEPENDS "${WEB_HTML}" "${CMAKE_CURRENT_LIST_DIR}/gzip_asset.py"
    COMMENT "Compressing gateway web UI"
    VERBATIM
)

target_add_binary_data(
    ${COMPONENT_LIB} "${WEB_HTML_GZIP}" BINARY
    DEPENDS "${WEB_HTML}" "${CMAKE_CURRENT_LIST_DIR}/gzip_asset.py"
)

target_add_binary_data(
    ${COMPONENT_LIB} "${LAB_PROFILE_EMBED}" TEXT
    DEPENDS "${LAB_PROFILE}"
)

