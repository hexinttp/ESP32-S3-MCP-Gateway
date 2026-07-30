#include "mcp/mcp_token_store.h"

#include <string.h>
#include "config/runtime_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "mbedtls/sha256.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "MCP_TOK";
static const char *NVS_NS = "mcp_tokens";

static mcp_token_t s_tokens[MCP_TOKEN_MAX];
static bool s_initialised = false;
static nvs_handle_t s_nvs;

/* Constant-time comparison of two fixed-length hex digests (both are exactly
 * 64 chars). Avoids short-circuiting that would leak timing information. */
static bool digest_equal(const char *a, const char *b)
{
    unsigned char diff = 0;
    for (int i = 0; i < 64; ++i) {
        diff |= (unsigned char)a[i] ^ (unsigned char)b[i];
    }
    return diff == 0;
}

static void sha256_hex(const char *text, char out[65])
{
    uint8_t digest[32];
    mbedtls_sha256((const unsigned char *)text, strlen(text), digest, 0);
    for (int i = 0; i < 32; ++i) {
        snprintf(out + i * 2, 3, "%02x", digest[i]);
    }
    out[64] = '\0';
}

static bool digest_is_valid(const char *hex)
{
    if (hex == NULL) return false;
    if (strlen(hex) != 64) return false;
    for (int i = 0; i < 64; ++i) {
        char c = hex[i];
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') ||
              (c >= 'A' && c <= 'F'))) return false;
    }
    return true;
}

static void persist_slot(int index)
{
    char key[8];
    snprintf(key, sizeof(key), "tok%d", index);
    if (s_tokens[index].valid) {
        nvs_set_blob(s_nvs, key, &s_tokens[index], sizeof(mcp_token_t));
    } else {
        nvs_erase_key(s_nvs, key);
    }
    nvs_commit(s_nvs);
}

static int find_empty_slot(void)
{
    for (int i = 0; i < MCP_TOKEN_MAX; ++i) {
        if (!s_tokens[i].valid) return i;
    }
    return -1;
}

static int find_by_id(const char *id)
{
    for (int i = 0; i < MCP_TOKEN_MAX; ++i) {
        if (s_tokens[i].valid && strcmp(s_tokens[i].id, id) == 0) return i;
    }
    return -1;
}

esp_err_t mcp_token_store_init(void)
{
    if (s_initialised) return ESP_OK;

    memset(s_tokens, 0, sizeof(s_tokens));
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &s_nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s (tokens will be RAM-only)",
                 esp_err_to_name(err));
        s_nvs = 0;
    } else {
        for (int i = 0; i < MCP_TOKEN_MAX; ++i) {
            char key[8];
            snprintf(key, sizeof(key), "tok%d", i);
            size_t len = sizeof(mcp_token_t);
            if (nvs_get_blob(s_nvs, key, &s_tokens[i], &len) == ESP_OK &&
                s_tokens[i].valid) {
                /* loaded */
            } else {
                memset(&s_tokens[i], 0, sizeof(s_tokens[i]));
            }
        }
    }

    /* Bootstrap: if no token exists yet and a Web admin password digest is
     * configured, seed an admin-scoped token so existing deployments that
     * already authenticate MCP with the same bearer keep working. This gives
     * zero-migration; the bootstrap token can later be deleted once dedicated
     * tokens are created. */
    bool any = false;
    for (int i = 0; i < MCP_TOKEN_MAX; ++i) {
        if (s_tokens[i].valid) { any = true; break; }
    }
    if (!any) {
        runtime_config_t cfg;
        runtime_config_get(&cfg);
        /* Only bootstrap an MCP admin token from the Web password when MCP access
         * is explicitly enabled. This keeps the Web login password (default
         * admin/admin123) separate from the MCP token store, so seeding the Web
         * credentials does not silently create an MCP token. */
        if (digest_is_valid(cfg.security.password_sha256) && cfg.security.auth_enabled) {
            int idx = find_empty_slot();
            if (idx >= 0) {
                mcp_token_t *t = &s_tokens[idx];
                t->valid = true;
                t->enabled = true;
                strlcpy(t->id, "admin", sizeof(t->id));
                t->scope = MCP_SCOPE_READ | MCP_SCOPE_WRITE | MCP_SCOPE_ADMIN;
                t->created_at_ms = esp_timer_get_time() / 1000;
                t->rotated_at_ms = t->created_at_ms;
                strlcpy(t->digest, cfg.security.password_sha256,
                        sizeof(t->digest));
                if (s_nvs) persist_slot(idx);
                ESP_LOGI(TAG, "bootstrap admin token created from Web password");
            }
        }
    }

    s_initialised = true;
    runtime_config_t *cfg = malloc(sizeof(*cfg));
    if (cfg != NULL) {
        runtime_config_get(cfg);
    }
    ESP_LOGI(TAG, "token store ready: %u token(s), auth bootstrap %s",
             (unsigned)mcp_token_count(),
             any ? "skipped (tokens present)" :
             (cfg != NULL && digest_is_valid(cfg->security.password_sha256)
                  ? "done" : "skipped (no Web password digest)"));
    free(cfg);
    return ESP_OK;
}

esp_err_t mcp_token_authenticate(const char *digest_hex, mcp_token_t *out_token)
{
    if (digest_hex == NULL || !digest_is_valid(digest_hex)) {
        return ESP_ERR_INVALID_ARG;
    }
    for (int i = 0; i < MCP_TOKEN_MAX; ++i) {
        if (!s_tokens[i].valid || !s_tokens[i].enabled) continue;
        if (digest_equal(s_tokens[i].digest, digest_hex)) {
            if (out_token != NULL) *out_token = s_tokens[i];
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_ALLOWED;
}

esp_err_t mcp_token_create(const char *id, uint8_t scope, const char *secret,
                           mcp_token_t *out_token)
{
    if (id == NULL || id[0] == '\0' || strlen(id) >= MCP_TOKEN_ID_LEN ||
        secret == NULL || secret[0] == '\0' || scope == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (find_by_id(id) >= 0) return ESP_ERR_INVALID_STATE;
    int idx = find_empty_slot();
    if (idx < 0) return ESP_ERR_INVALID_STATE;

    mcp_token_t *t = &s_tokens[idx];
    memset(t, 0, sizeof(*t));
    t->valid = true;
    t->enabled = true;
    strlcpy(t->id, id, sizeof(t->id));
    t->scope = (uint8_t)(scope & (MCP_SCOPE_READ | MCP_SCOPE_WRITE | MCP_SCOPE_ADMIN));
    t->created_at_ms = esp_timer_get_time() / 1000;
    t->rotated_at_ms = t->created_at_ms;
    sha256_hex(secret, t->digest);
    if (s_nvs) persist_slot(idx);
    if (out_token != NULL) *out_token = *t;
    ESP_LOGI(TAG, "token '%s' created (scope=0x%02x)", id, t->scope);
    return ESP_OK;
}

esp_err_t mcp_token_list(mcp_token_t *out, size_t *count)
{
    if (out == NULL || count == NULL) return ESP_ERR_INVALID_ARG;
    size_t n = 0;
    for (int i = 0; i < MCP_TOKEN_MAX && n < *count; ++i) {
        if (s_tokens[i].valid) out[n++] = s_tokens[i];
    }
    *count = n;
    return ESP_OK;
}

esp_err_t mcp_token_delete(const char *id)
{
    int idx = find_by_id(id);
    if (idx < 0) return ESP_ERR_NOT_FOUND;

    /* Prevent deleting the last enabled admin token to avoid lockout. */
    bool last_admin = (s_tokens[idx].scope & MCP_SCOPE_ADMIN) != 0;
    if (last_admin) {
        int admin_count = 0;
        for (int i = 0; i < MCP_TOKEN_MAX; ++i) {
            if (s_tokens[i].valid && s_tokens[i].enabled &&
                (s_tokens[i].scope & MCP_SCOPE_ADMIN) != 0) {
                ++admin_count;
            }
        }
        if (admin_count <= 1) return ESP_ERR_INVALID_STATE;
    }

    memset(&s_tokens[idx], 0, sizeof(s_tokens[idx]));
    if (s_nvs) persist_slot(idx);
    ESP_LOGI(TAG, "token '%s' deleted", id);
    return ESP_OK;
}

esp_err_t mcp_token_rotate(const char *id, const char *new_secret,
                           mcp_token_t *out_token)
{
    int idx = find_by_id(id);
    if (idx < 0) return ESP_ERR_NOT_FOUND;
    if (new_secret == NULL || new_secret[0] == '\0') return ESP_ERR_INVALID_ARG;
    sha256_hex(new_secret, s_tokens[idx].digest);
    s_tokens[idx].rotated_at_ms = esp_timer_get_time() / 1000;
    if (s_nvs) persist_slot(idx);
    if (out_token != NULL) *out_token = s_tokens[idx];
    ESP_LOGI(TAG, "token '%s' rotated", id);
    return ESP_OK;
}

size_t mcp_token_count(void)
{
    size_t n = 0;
    for (int i = 0; i < MCP_TOKEN_MAX; ++i) {
        if (s_tokens[i].valid) ++n;
    }
    return n;
}
