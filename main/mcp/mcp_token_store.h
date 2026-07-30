#ifndef MCP_TOKEN_STORE_H
#define MCP_TOKEN_STORE_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* Dedicated, NVS-backed store for MCP bearer tokens. MCP secrets are kept
 * completely separate from the runtime configuration DTO (which is serialised
 * to JSON and exposed through the Web API), so a config dump never leaks a
 * usable credential. Each token carries an id, a scope bitmask, an enabled
 * flag and rotation timestamps; only the SHA-256 digest of the secret is ever
 * persisted (never the plaintext). */

#define MCP_TOKEN_MAX 8
#define MCP_TOKEN_ID_LEN 33        /* NUL-terminated C string */
#define MCP_TOKEN_DIGEST_LEN 65    /* 64 hex chars + NUL */

typedef enum {
    MCP_SCOPE_READ  = (1 << 0),   /* read points / status */
    MCP_SCOPE_WRITE = (1 << 1),   /* write points / create rules */
    MCP_SCOPE_ADMIN = (1 << 2),   /* full access (includes read+write) */
} mcp_token_scope_t;

typedef struct {
    bool valid;                       /* slot in use */
    bool enabled;
    char id[MCP_TOKEN_ID_LEN];
    uint8_t scope;                    /* bitmask of mcp_token_scope_t */
    int64_t created_at_ms;
    int64_t rotated_at_ms;
    char digest[MCP_TOKEN_DIGEST_LEN];/* hex SHA-256 of the secret */
} mcp_token_t;

/* Load tokens from NVS into RAM. Idempotent. If the store is empty and a Web
 * admin password digest is configured, a bootstrap "admin" token (full scope)
 * is created so existing deployments keep working without manual migration. */
esp_err_t mcp_token_store_init(void);

/* Authenticate a bearer secret by digest. On success fills `out_token` and
 * returns ESP_OK. Disabled or unknown tokens return ESP_ERR_NOT_ALLOWED. */
esp_err_t mcp_token_authenticate(const char *digest_hex, mcp_token_t *out_token);

static inline bool mcp_token_has_scope(const mcp_token_t *token,
                                       mcp_token_scope_t scope)
{
    return token != NULL && (token->scope & (uint8_t)scope) != 0;
}

/* Create a token. `secret` is the plaintext bearer; its digest is stored.
 * `scope` is a bitmask of mcp_token_scope_t. Returns ESP_ERR_INVALID_STATE if
 * the store is full, ESP_ERR_INVALID_ARG for bad input. */
esp_err_t mcp_token_create(const char *id, uint8_t scope, const char *secret,
                           mcp_token_t *out_token);

/* List all tokens (including disabled). `out` must hold MCP_TOKEN_MAX entries;
 * `*count` receives the number filled. No secrets are returned by the caller
 * through this struct except the digest; the Web layer must hide `digest`. */
esp_err_t mcp_token_list(mcp_token_t *out, size_t *count);

/* Delete a token by id. The last enabled admin token cannot be deleted to
 * avoid locking everyone out. */
esp_err_t mcp_token_delete(const char *id);

/* Rotate a token's secret. Updates digest and rotated_at_ms. */
esp_err_t mcp_token_rotate(const char *id, const char *new_secret,
                           mcp_token_t *out_token);

/* Number of valid (active) tokens. */
size_t mcp_token_count(void);

#endif /* MCP_TOKEN_STORE_H */
