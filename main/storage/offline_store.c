#include "offline_store.h"

#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include "board/tf_storage.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "wear_levelling.h"

#define FLASH_MOUNT_POINT "/flash"
#define FLASH_CACHE_DIR FLASH_MOUNT_POINT "/cache"
#define TF_CACHE_DIR TF_MOUNT_POINT "/cache"
#define FLASH_RESERVE_BYTES (64 * 1024ULL)

static const char *TAG = "OFFLINE_STORE";
static wl_handle_t s_wl = WL_INVALID_HANDLE;
static SemaphoreHandle_t s_mutex;
static bool s_flash_ready;
static int s_data_loss;
static int s_flash_count;
static int s_tf_count;
static uint32_t s_flash_oldest;
static uint32_t s_tf_oldest;
static bool s_flash_oldest_valid;
static bool s_tf_oldest_valid;

static void record_path(char *out, size_t size, const char *directory, uint32_t sequence_id)
{
    /* Eight hexadecimal characters keep the filename compatible with FAT 8.3. */
    snprintf(out, size, "%s/%08lX.TCM", directory, (unsigned long)sequence_id);
}

static bool parse_sequence(const char *name, uint32_t *sequence)
{
    size_t len = strlen(name);
    if (len != 12 || name[8] != '.' ||
        (name[9] != 't' && name[9] != 'T') ||
        (name[10] != 'c' && name[10] != 'C') ||
        (name[11] != 'm' && name[11] != 'M')) {
        return false;
    }
    char stem[9];
    memcpy(stem, name, 8);
    stem[8] = '\0';
    char *end = NULL;
    unsigned long value = strtoul(stem, &end, 16);
    if (end != stem + 8) return false;
    *sequence = (uint32_t)value;
    return true;
}

static bool find_oldest_in(const char *directory, uint32_t *sequence)
{
    DIR *dir = opendir(directory);
    if (dir == NULL) return false;
    bool found = false;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        uint32_t current;
        if (parse_sequence(entry->d_name, &current) && (!found || current < *sequence)) {
            *sequence = current;
            found = true;
        }
    }
    closedir(dir);
    return found;
}

static int count_in(const char *directory)
{
    int count = 0;
    DIR *dir = opendir(directory);
    if (dir == NULL) return 0;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        uint32_t ignored;
        if (parse_sequence(entry->d_name, &ignored)) ++count;
    }
    closedir(dir);
    return count;
}

static void load_store_state(const char *directory, int *count,
                             uint32_t *oldest, bool *oldest_valid)
{
    *count = count_in(directory);
    *oldest = 0;
    *oldest_valid = find_oldest_in(directory, oldest);
}

static void advance_oldest(const char *directory, int count,
                           uint32_t removed_sequence, uint32_t *oldest,
                           bool *oldest_valid)
{
    if (count <= 0) {
        *oldest = 0;
        *oldest_valid = false;
        return;
    }
    uint32_t candidate = removed_sequence + 1;
    char path[64];
    record_path(path, sizeof(path), directory, candidate);
    if (access(path, F_OK) == 0) {
        *oldest = candidate;
        *oldest_valid = true;
        return;
    }
    *oldest = 0;
    *oldest_valid = find_oldest_in(directory, oldest);
}

static bool remove_oldest(const char *directory, int *count,
                          uint32_t *oldest, bool *oldest_valid)
{
    if (*count <= 0 || !*oldest_valid) return false;
    uint32_t removed = *oldest;
    char path[64];
    record_path(path, sizeof(path), directory, removed);
    if (unlink(path) != 0) {
        load_store_state(directory, count, oldest, oldest_valid);
        return false;
    }
    --(*count);
    advance_oldest(directory, *count, removed, oldest, oldest_valid);
    return true;
}

static esp_err_t write_record(const char *directory, uint32_t sequence_id,
                              const char *topic, const char *payload)
{
    char path[64];
    record_path(path, sizeof(path), directory, sequence_id);
    FILE *file = fopen(path, "wb");
    if (file == NULL) {
        ESP_LOGE(TAG, "Open %s failed: errno=%d (%s)", path, errno, strerror(errno));
        return ESP_FAIL;
    }
    int result = fprintf(file, "%lu\n%s\n%s", (unsigned long)sequence_id, topic, payload);
    if (fflush(file) != 0) result = -1;
    fsync(fileno(file));
    fclose(file);
    return result > 0 ? ESP_OK : ESP_FAIL;
}

static esp_err_t read_record(const char *directory, uint32_t sequence_id,
                             offline_medium_t medium, offline_record_t *out)
{
    char path[64];
    record_path(path, sizeof(path), directory, sequence_id);
    FILE *file = fopen(path, "rb");
    if (file == NULL) return ESP_ERR_NOT_FOUND;
    char seq_line[16];
    if (fgets(seq_line, sizeof(seq_line), file) == NULL ||
        fgets(out->topic, sizeof(out->topic), file) == NULL) {
        fclose(file);
        return ESP_FAIL;
    }
    out->topic[strcspn(out->topic, "\r\n")] = '\0';
    size_t read = fread(out->payload, 1, sizeof(out->payload) - 1, file);
    out->payload[read] = '\0';
    fclose(file);
    out->sequence_id = sequence_id;
    out->medium = medium;
    return ESP_OK;
}

esp_err_t offline_store_init(void)
{
    if (s_mutex == NULL) s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) return ESP_ERR_NO_MEM;

    esp_vfs_fat_mount_config_t config = {
        .format_if_mount_failed = true,
        .max_files = 8,
        .allocation_unit_size = 4096,
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(FLASH_MOUNT_POINT, UIF_CACHE_PARTITION,
                                                      &config, &s_wl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI Flash cache mount failed: %s", esp_err_to_name(err));
        return err;
    }
    if (mkdir(FLASH_CACHE_DIR, 0775) != 0 && errno != EEXIST) {
        ESP_LOGE(TAG, "Create cache directory failed: errno=%d (%s)",
                 errno, strerror(errno));
        return ESP_FAIL;
    }
    s_flash_ready = true;
    load_store_state(FLASH_CACHE_DIR, &s_flash_count, &s_flash_oldest,
                     &s_flash_oldest_valid);
    int pruned = 0;
    while (s_flash_count > UIF_CACHE_MAX_RECORDS &&
           remove_oldest(FLASH_CACHE_DIR, &s_flash_count, &s_flash_oldest,
                         &s_flash_oldest_valid)) {
        ++pruned;
        ++s_data_loss;
    }
    if (pruned > 0) {
        ESP_LOGW(TAG, "Pruned %d legacy cache records to capacity %d",
                 pruned, UIF_CACHE_MAX_RECORDS);
    }
    if (tf_storage_is_mounted()) {
        load_store_state(TF_CACHE_DIR, &s_tf_count, &s_tf_oldest,
                         &s_tf_oldest_valid);
    }
    ESP_LOGI(TAG, "Primary SPI Flash cache mounted at %s", FLASH_MOUNT_POINT);
    return ESP_OK;
}

esp_err_t offline_store_put(uint32_t sequence_id, const char *topic, const char *payload)
{
    if (!s_flash_ready || topic == NULL || payload == NULL) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    uint64_t total = 0, free = 0;
    esp_err_t info_err = esp_vfs_fat_info(FLASH_MOUNT_POINT, &total, &free);
    esp_err_t err = ESP_FAIL;
    if (info_err != ESP_OK) {
        ESP_LOGE(TAG, "SPI Flash space query failed: %s", esp_err_to_name(info_err));
    } else if (s_flash_count < UIF_CACHE_MAX_RECORDS &&
               free > FLASH_RESERVE_BYTES + strlen(payload) + strlen(topic) + 64) {
        err = write_record(FLASH_CACHE_DIR, sequence_id, topic, payload);
        if (err == ESP_OK) {
            ++s_flash_count;
            if (!s_flash_oldest_valid || sequence_id < s_flash_oldest) {
                s_flash_oldest = sequence_id;
                s_flash_oldest_valid = true;
            }
        }
    } else {
        ESP_LOGD(TAG, "SPI Flash cache capacity reached: records=%d free=%llu",
                 s_flash_count, (unsigned long long)free);
    }
    if (err != ESP_OK && tf_storage_is_mounted()) {
        if (s_tf_count >= UIF_CACHE_MAX_RECORDS &&
            remove_oldest(TF_CACHE_DIR, &s_tf_count, &s_tf_oldest,
                          &s_tf_oldest_valid)) {
            ++s_data_loss;
        }
        err = write_record(TF_CACHE_DIR, sequence_id, topic, payload);
        if (err != ESP_OK &&
            remove_oldest(TF_CACHE_DIR, &s_tf_count, &s_tf_oldest,
                          &s_tf_oldest_valid)) {
            ++s_data_loss;
            err = write_record(TF_CACHE_DIR, sequence_id, topic, payload);
        }
        if (err == ESP_OK) {
            ++s_tf_count;
            if (!s_tf_oldest_valid || sequence_id < s_tf_oldest) {
                s_tf_oldest = sequence_id;
                s_tf_oldest_valid = true;
            }
        }
    } else if (err != ESP_OK && !tf_storage_is_mounted() &&
               remove_oldest(FLASH_CACHE_DIR, &s_flash_count,
                             &s_flash_oldest, &s_flash_oldest_valid)) {
        ++s_data_loss;
        err = write_record(FLASH_CACHE_DIR, sequence_id, topic, payload);
        if (err == ESP_OK) {
            ++s_flash_count;
            if (!s_flash_oldest_valid) {
                s_flash_oldest = sequence_id;
                s_flash_oldest_valid = true;
            }
        }
    }
    if (err != ESP_OK) ++s_data_loss;
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t offline_store_peek_oldest(offline_record_t *out)
{
    if (out == NULL) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    uint32_t flash_seq = 0, tf_seq = 0;
    bool flash = find_oldest_in(FLASH_CACHE_DIR, &flash_seq);
    bool tf = tf_storage_is_mounted() && find_oldest_in(TF_CACHE_DIR, &tf_seq);
    esp_err_t err = ESP_ERR_NOT_FOUND;
    if (flash && (!tf || flash_seq <= tf_seq)) {
        err = read_record(FLASH_CACHE_DIR, flash_seq, OFFLINE_MEDIUM_FLASH, out);
    } else if (tf) {
        err = read_record(TF_CACHE_DIR, tf_seq, OFFLINE_MEDIUM_TF, out);
    }
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t offline_store_remove(uint32_t sequence_id)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    char flash_path[64], tf_path[64];
    record_path(flash_path, sizeof(flash_path), FLASH_CACHE_DIR, sequence_id);
    record_path(tf_path, sizeof(tf_path), TF_CACHE_DIR, sequence_id);
    int flash_result = unlink(flash_path);
    int tf_result = unlink(tf_path);
    if (flash_result == 0 && s_flash_count > 0) {
        --s_flash_count;
        if (s_flash_oldest_valid && s_flash_oldest == sequence_id) {
            advance_oldest(FLASH_CACHE_DIR, s_flash_count, sequence_id,
                           &s_flash_oldest, &s_flash_oldest_valid);
        }
    }
    if (tf_result == 0 && s_tf_count > 0) {
        --s_tf_count;
        if (s_tf_oldest_valid && s_tf_oldest == sequence_id) {
            advance_oldest(TF_CACHE_DIR, s_tf_count, sequence_id,
                           &s_tf_oldest, &s_tf_oldest_valid);
        }
    }
    xSemaphoreGive(s_mutex);
    return (flash_result == 0 || tf_result == 0) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

int offline_store_count(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int count = s_flash_count + (tf_storage_is_mounted() ? s_tf_count : 0);
    xSemaphoreGive(s_mutex);
    return count;
}

int offline_store_usage_percent(void)
{
    uint64_t total = 0, free = 0;
    if (!s_flash_ready || esp_vfs_fat_info(FLASH_MOUNT_POINT, &total, &free) != ESP_OK || total == 0) return 0;
    return (int)(((total - free) * 100ULL) / total);
}

int offline_store_data_loss_count(void) { return s_data_loss; }
bool offline_store_flash_ready(void) { return s_flash_ready; }
