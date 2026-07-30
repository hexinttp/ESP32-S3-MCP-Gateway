/**
 * @file semantic_inference.c
 * @brief Conservative unknown-device semantic bootstrap.
 */

#include "semantic_inference.h"

#include <stdio.h>
#include <string.h>

static uint32_t fnv1a_bytes(uint32_t hash, const void *data, size_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    for (size_t i = 0; i < length; ++i) {
        hash ^= bytes[i];
        hash *= 16777619U;
    }
    return hash;
}

static uint32_t fnv1a_string(uint32_t hash, const char *value)
{
    if (value == NULL) return hash;
    return fnv1a_bytes(hash, value, strlen(value));
}

static bool has_device_metadata(const semantic_device_features_t *features)
{
    return (features->vendor_name != NULL && features->vendor_name[0] != '\0') ||
           (features->product_code != NULL && features->product_code[0] != '\0') ||
           (features->revision != NULL && features->revision[0] != '\0');
}

void semantic_inference_identify(const semantic_device_features_t *features,
                                 semantic_device_identity_t *identity)
{
    if (identity == NULL) return;
    memset(identity, 0, sizeof(*identity));
    if (features == NULL) return;

    uint32_t hash = 2166136261U;
    hash = fnv1a_bytes(hash, &features->source_protocol,
                       sizeof(features->source_protocol));
    hash = fnv1a_bytes(hash, &features->channel_id,
                       sizeof(features->channel_id));
    hash = fnv1a_bytes(hash, &features->slave_id, sizeof(features->slave_id));
    hash = fnv1a_string(hash, features->vendor_name);
    hash = fnv1a_string(hash, features->product_code);
    hash = fnv1a_string(hash, features->revision);
    hash = fnv1a_bytes(hash, &features->probe_function_code,
                       sizeof(features->probe_function_code));
    hash = fnv1a_bytes(hash, &features->probe_address,
                       sizeof(features->probe_address));
    for (size_t i = 0; i < features->register_count; ++i) {
        hash = fnv1a_bytes(hash, &features->registers[i].function_code,
                           sizeof(features->registers[i].function_code));
        hash = fnv1a_bytes(hash, &features->registers[i].address,
                           sizeof(features->registers[i].address));
    }

    snprintf(identity->fingerprint, sizeof(identity->fingerprint),
             "%08lx", (unsigned long)hash);
    snprintf(identity->profile_id, sizeof(identity->profile_id),
             "unresolved-%08lx", (unsigned long)hash);
    identity->confidence = has_device_metadata(features) ? 40 : 20;
    snprintf(identity->evidence, sizeof(identity->evidence),
             "fp=%08lx; live FC%02u response",
             (unsigned long)hash, features->probe_function_code);
}

static void raw_range(uint8_t function_code, float *minimum, float *maximum)
{
    if (function_code == 1 || function_code == 2) {
        *minimum = 0.0f;
        *maximum = 1.0f;
    } else {
        *minimum = 0.0f;
        *maximum = 65535.0f;
    }
}

void semantic_inference_build_raw_mapping(
    const semantic_device_features_t *device,
    const semantic_device_identity_t *identity,
    const semantic_raw_point_t *point,
    const char *device_id,
    amm_mapping_entry_t *entry)
{
    if (device == NULL || identity == NULL || point == NULL || entry == NULL) {
        return;
    }

    memset(entry, 0, sizeof(*entry));
    entry->source_protocol = device->source_protocol;
    entry->channel_id = device->channel_id;
    entry->slave_id = device->slave_id;
    entry->function_code = point->function_code;
    entry->object_type = point->function_code >= 1 && point->function_code <= 4
        ? (modbus_object_type_t)point->function_code
        : MODBUS_OBJECT_HOLDING_REGISTER;
    entry->register_address = point->address;
    entry->data_type = point->function_code == 1 || point->function_code == 2
        ? DT_BOOL : DT_UINT16;
    entry->byte_order = BYTE_ORDER_ABCD;
    entry->scale_factor = 1.0f;
    entry->offset = 0.0f;
    /* Zero means "inherit the gateway-wide AMM polling interval". */
    entry->poll_interval_ms = 0;
    entry->priority = 3;
    entry->active = true;
    entry->discovered = true;
    entry->retry_count = 2;
    entry->retry_backoff_ms = 50;
    entry->read_start_address = point->read_register_count > 0
        ? point->read_start_address : point->address;
    entry->read_register_count =
        point->read_register_count > 0 ? point->read_register_count : 1;
    entry->value_register_index = point->value_register_index;

    if (device_id != NULL && device_id[0] != '\0') {
        strlcpy(entry->device_id, device_id, sizeof(entry->device_id));
    } else {
        snprintf(entry->device_id, sizeof(entry->device_id),
                 "%s_ch%u_slave_%02u",
                 device->source_protocol == SRC_MODBUS_TCP ? "tcp" : "rtu",
                 device->channel_id, device->slave_id);
    }

    if (point->user_edited) {
        /*
         * The user confirmed this point in the discovery editor before the
         * mapping was (re)applied: keep their semantics instead of the
         * conservative raw defaults and mark the point USER/VERIFIED.
         */
        snprintf(entry->point_id, sizeof(entry->point_id), "raw_fc%02u_%u",
                 point->function_code, point->address);
        if (point->name != NULL && point->name[0] != '\0') {
            strlcpy(entry->measurement_name, point->name,
                    sizeof(entry->measurement_name));
        } else {
            snprintf(entry->measurement_name, sizeof(entry->measurement_name),
                     "Raw FC%02u %u", point->function_code, point->address);
        }
        if (point->unit != NULL) {
            strlcpy(entry->unit, point->unit, sizeof(entry->unit));
        }
        entry->data_type = point->data_type;
        entry->constraint.writable = point->writable;
        entry->constraint.valid_range_min = point->range_min;
        entry->constraint.valid_range_max = point->range_max;
        entry->semantic_source = SEMANTIC_SOURCE_USER;
        entry->semantic_status = SEMANTIC_STATUS_VERIFIED;
        entry->semantic_confidence = 100;
        strlcpy(entry->semantic_evidence, "user edit via discovery editor",
                sizeof(entry->semantic_evidence));
    } else {
        snprintf(entry->point_id, sizeof(entry->point_id), "raw_fc%02u_%u",
                 point->function_code, point->address);
        snprintf(entry->measurement_name, sizeof(entry->measurement_name),
                 "Raw FC%02u %u", point->function_code, point->address);
        entry->unit[0] = '\0';
        entry->constraint.writable = false;
        raw_range(point->function_code,
                  &entry->constraint.valid_range_min,
                  &entry->constraint.valid_range_max);
        entry->semantic_source = SEMANTIC_SOURCE_DISCOVERY;
        entry->semantic_status = SEMANTIC_STATUS_UNRESOLVED;
        entry->semantic_confidence = identity->confidence;
        strlcpy(entry->semantic_evidence, identity->evidence,
                sizeof(entry->semantic_evidence));
    }
    snprintf(entry->mqtt_topic, sizeof(entry->mqtt_topic),
             "factory/data/%s/%s", entry->device_id, entry->point_id);

    strlcpy(entry->semantic_profile_id, identity->profile_id,
            sizeof(entry->semantic_profile_id));
    entry->semantic_profile_version = 0;
}
