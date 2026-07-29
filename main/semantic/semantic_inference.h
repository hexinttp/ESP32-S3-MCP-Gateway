/**
 * @file semantic_inference.h
 * @brief Safe semantic bootstrap for previously unknown MODBUS devices.
 *
 * Unknown devices are represented as stable, read-only raw points.  The
 * module deliberately does not guess an engineering quantity, unit, scale,
 * signedness, or multi-register layout without a verified device profile.
 */
#ifndef SEMANTIC_INFERENCE_H
#define SEMANTIC_INFERENCE_H

#include <stddef.h>
#include <stdint.h>

#include "amm/amm_mapping.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t function_code;
    uint16_t address;
} semantic_register_signature_t;

typedef struct {
    source_protocol_t source_protocol;
    uint8_t channel_id;
    uint8_t slave_id;
    const char *vendor_name;
    const char *product_code;
    const char *revision;
    uint8_t probe_function_code;
    uint16_t probe_address;
    const semantic_register_signature_t *registers;
    size_t register_count;
} semantic_device_features_t;

typedef struct {
    char fingerprint[17];
    char profile_id[32];
    uint8_t confidence;
    char evidence[48];
} semantic_device_identity_t;

typedef struct {
    uint8_t function_code;
    uint16_t address;
    uint16_t raw_value;
    uint16_t read_start_address;
    uint8_t read_register_count;
    uint8_t value_register_index;
    /*
     * User-confirmed semantics from the discovery register editor. Only
     * consulted when user_edited is true; auto-guessed scan hints are never
     * forwarded so unknown devices stay conservative by default.
     */
    bool        user_edited;
    const char *name;
    const char *unit;
    data_type_t data_type;
    bool        writable;
    float       range_min;
    float       range_max;
} semantic_raw_point_t;

/**
 * Build a stable topology and register-signature fingerprint.
 *
 * MODBUS has no universal device identity service. Vendor/product/revision
 * metadata is included when available; otherwise the bus location and live
 * register signature form a deterministic provisional identity.
 */
void semantic_inference_identify(const semantic_device_features_t *features,
                                 semantic_device_identity_t *identity);

/**
 * Create one safe unresolved AMM entry for a live raw MODBUS object.
 *
 * The result is read-only and carries no unit or inferred engineering
 * quantity. A user edit or verified profile can promote it later.
 */
void semantic_inference_build_raw_mapping(
    const semantic_device_features_t *device,
    const semantic_device_identity_t *identity,
    const semantic_raw_point_t *point,
    const char *device_id,
    amm_mapping_entry_t *entry);

#ifdef __cplusplus
}
#endif

#endif /* SEMANTIC_INFERENCE_H */
