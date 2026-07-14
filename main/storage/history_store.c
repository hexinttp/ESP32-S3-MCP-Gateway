#include "storage/history_store.h"

#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "board/tf_storage.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "tcm/tcm_context.h"
#include "esp_log.h"

#define HISTORY_DIRECTORY TF_MOUNT_POINT "/history"
#define HISTORY_MIN_FREE_BYTES (2ULL * 1024ULL * 1024ULL)
#define HISTORY_RECORDS_PER_FILE 256U

static const char *TAG = "HISTORY";
static SemaphoreHandle_t s_mutex;
static int s_deleted_files;

esp_err_t history_store_init(void)
{
    if (s_mutex == NULL) s_mutex = xSemaphoreCreateMutex();
    return s_mutex == NULL ? ESP_ERR_NO_MEM : ESP_OK;
}

static bool history_name(const char *name)
{
    return name != NULL && strncmp(name, "history_", 8) == 0 && strstr(name, ".jsonl") != NULL;
}

static esp_err_t delete_oldest(void)
{
    DIR *dir = opendir(HISTORY_DIRECTORY);
    if (dir == NULL) return ESP_ERR_NOT_FOUND;
    char oldest[64] = {0};
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        if (history_name(entry->d_name) && (oldest[0] == '\0' || strcmp(entry->d_name, oldest) < 0)) {
            strlcpy(oldest, entry->d_name, sizeof(oldest));
        }
    }
    closedir(dir);
    if (oldest[0] == '\0') return ESP_ERR_NOT_FOUND;
    char path[128];
    snprintf(path, sizeof(path), "%s/%s", HISTORY_DIRECTORY, oldest);
    if (unlink(path) != 0) return ESP_FAIL;
    ++s_deleted_files;
    ESP_LOGW(TAG, "TF full: removed oldest history file %s", oldest);
    return ESP_OK;
}

static void ensure_free_space(void)
{
    uint64_t total = 0, free_bytes = 0;
    for (int attempts = 0; attempts < 16; ++attempts) {
        if (tf_storage_get_space(&total, &free_bytes) != ESP_OK || free_bytes >= HISTORY_MIN_FREE_BYTES) return;
        if (delete_oldest() != ESP_OK) return;
    }
}

esp_err_t history_store_append(const tcm_context_t *context)
{
    if (context == NULL) return ESP_ERR_INVALID_ARG;
    if (!tf_storage_is_mounted()) return ESP_ERR_INVALID_STATE;
    if (s_mutex == NULL) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    ensure_free_space();
    char path[128];
    unsigned long chunk = (unsigned long)(context->sequence_id / HISTORY_RECORDS_PER_FILE);
    snprintf(path, sizeof(path), HISTORY_DIRECTORY "/history_%010lu.jsonl", chunk);
    FILE *file = fopen(path, "a");
    if (file == NULL) {
        xSemaphoreGive(s_mutex);
        return ESP_FAIL;
    }
    char json[TCM_MAX_JSON_LEN];
    int length = tcm_serialize_json(context, json, sizeof(json));
    esp_err_t err = ESP_FAIL;
    if (length > 0 && fwrite(json, 1, (size_t)length, file) == (size_t)length &&
        fwrite("\n", 1, 1, file) == 1) {
        fflush(file);
        err = ESP_OK;
    }
    fclose(file);
    xSemaphoreGive(s_mutex);
    return err;
}

int history_store_deleted_files(void)
{
    return s_deleted_files;
}
