#include "thingscloud_topics.h"
#include <stdio.h>
#include <string.h>

int thingscloud_format_device_address(uint8_t channel_id, uint8_t slave_id,
                                      char *out, size_t out_size)
{
    if (out == NULL || out_size == 0) return -1;
    int n;
    if (channel_id == 0) {
        n = snprintf(out, out_size, "%u", (unsigned)slave_id);
    } else {
        n = snprintf(out, out_size, "rs485_%u_%u", (unsigned)channel_id, (unsigned)slave_id);
    }
    if (n < 0 || (size_t)n >= out_size) return -1;
    return n;
}

bool thingscloud_parse_device_address(const char *addr,
                                      uint8_t *channel_id, uint8_t *slave_id)
{
    if (addr == NULL || addr[0] == '\0') return false;
    if (strncmp(addr, "rs485_", 7) == 0) {
        unsigned ch = 0, sl = 0;
        if (sscanf(addr + 7, "%u_%u", &ch, &sl) != 2) return false;
        if (channel_id) *channel_id = (uint8_t)ch;
        if (slave_id) *slave_id = (uint8_t)sl;
        return true;
    }
    unsigned sl = 0;
    if (sscanf(addr, "%u", &sl) != 1) return false;
    if (channel_id) *channel_id = 0;
    if (slave_id) *slave_id = (uint8_t)sl;
    return true;
}
