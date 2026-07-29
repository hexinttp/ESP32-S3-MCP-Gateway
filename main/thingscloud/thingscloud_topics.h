#ifndef THINGSCLOUD_TOPICS_H
#define THINGSCLOUD_TOPICS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ThingsCloud MQTT topics (per official MQTT gateway & sub-device protocol).
 * https://www.thingscloud.xyz/docs/ */
#define TC_TOPIC_GATEWAY_ATTR        "gateway/attributes"
#define TC_TOPIC_GATEWAY_ATTR_SERIES "gateway/attributes/series"
#define TC_TOPIC_GATEWAY_ATTR_PUSH   "gateway/attributes/push"
#define TC_TOPIC_GATEWAY_CMD_SEND    "gateway/command/send"
#define TC_TOPIC_GATEWAY_CMD_REPLY   "gateway/command/reply"
#define TC_TOPIC_GATEWAY_CONNECT     "gateway/connect"
#define TC_TOPIC_GATEWAY_DISCONNECT  "gateway/disconnect"
#define TC_TOPIC_ATTR                "attributes"
#define TC_TOPIC_ATTR_PUSH           "attributes/push"

#define TC_FIRMWARE_VERSION          "1.0.0"

/* Format a ThingsCloud sub-device address from channel + slave id.
 *   - Single primary RS485 port (channel == 0): decimal slave id, e.g. "1"
 *   - Additional ports: "rs485_{channel}_{slave}" to avoid station conflicts
 * Returns the length written (excluding NUL) or < 0 on overflow. */
int thingscloud_format_device_address(uint8_t channel_id, uint8_t slave_id,
                                      char *out, size_t out_size);

/* Parse a ThingsCloud sub-device address back to channel + slave id.
 * Accepts both "1" and "rs485_1_1" forms. Returns true on success. */
bool thingscloud_parse_device_address(const char *addr,
                                      uint8_t *channel_id, uint8_t *slave_id);

#ifdef __cplusplus
}
#endif

#endif /* THINGSCLOUD_TOPICS_H */
