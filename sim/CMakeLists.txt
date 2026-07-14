cmake_minimum_required(VERSION 3.16)
project(gateway_sim C)

set(CMAKE_C_STANDARD 11)

# Compiler warnings
if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR CMAKE_C_COMPILER_ID STREQUAL "Clang")
    add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers)
    # Ensure POSIX functions like gmtime_r and clock_gettime are available
    add_definitions(-D_POSIX_C_SOURCE=200809L)
endif()

# Include paths: mock headers + project headers
include_directories(
    include                    # Mock ESP-IDF headers (esp_log.h, nvs.h, etc.)
    ../main                    # Project headers (gateway_config.h, tcm/tcm_context.h, etc.)
)

# Source files: mock implementations + original module code (compiled UNCHANGED)
set(SOURCES
    src/esp_mock.c
    src/sim_main.c
    ../main/tcm/tcm_context.c
    ../main/amm/amm_mapping.c
    ../main/uif/uif_persistence.c
    ../main/mqtt_comm/mqtt_handler.c
    ../main/modbus/modbus_access.c
    ../main/eval/eval_logger.c
)

# NOTE: scheduler.c is intentionally excluded from the build.
# The simulation runs sequentially in sim_main.c and calls module APIs directly,
# so the FreeRTOS task-based scheduler is not needed.  Stub functions are not
# required because sim_main.c does not call any scheduler_* functions.

add_executable(gateway_sim ${SOURCES})

# Link libraries
if(WIN32)
    # MinGW: link pthread for FreeRTOS mock (pthreads-win32 or winpthreads)
    target_link_libraries(gateway_sim pthread)
endif()

if(UNIX)
    # Linux/macOS: link pthread and math
    target_link_libraries(gateway_sim pthread m)
endif()
