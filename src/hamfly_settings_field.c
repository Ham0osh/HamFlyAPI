/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright 2026 Hamish Johnson
 * Quantum Information Systems Lab, SFU Physics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 * implied. See the License for the specific language governing
 * permissions and limitations under the License.
 *
 * NOT build-verified -- no C compiler was available when this was written.
 */

#include "hamfly_settings_field.h"

#include <string.h>
#include <math.h>

const hamfly_attr_field_t *hamfly_settings_find_field(uint16_t attr, const char *name)
{
    if (!name) return NULL;
    for (uint16_t i = 0u; i < HAMFLY_ATTR_FIELD_COUNT; ++i) {
        if (hamfly_attr_fields[i].attr == attr &&
            strcmp(hamfly_attr_fields[i].name, name) == 0) {
            return &hamfly_attr_fields[i];
        }
    }
    return NULL;
}

uint8_t hamfly_settings_field_count(uint16_t attr)
{
    uint8_t n = 0u;
    for (uint16_t i = 0u; i < HAMFLY_ATTR_FIELD_COUNT; ++i) {
        if (hamfly_attr_fields[i].attr == attr) n++;
    }
    return n;
}

const hamfly_attr_field_t *hamfly_settings_field_at(uint16_t attr, uint8_t index)
{
    uint8_t seen = 0u;
    for (uint16_t i = 0u; i < HAMFLY_ATTR_FIELD_COUNT; ++i) {
        if (hamfly_attr_fields[i].attr != attr) continue;
        if (seen == index) return &hamfly_attr_fields[i];
        seen++;
    }
    return NULL;
}

/* Big-endian load, sign-extended when is_signed.
 *
 * Returns int64_t, NOT int32_t: a 4-byte UNSIGNED field ranges to 4294967295,
 * which does not fit in int32_t. Accumulating into int32_t silently wrapped
 * attr 121 "Device Serial Number" (0xF5BA9024 = 4122644516) to -172322780.
 * This matches raw_bounds() below, which already used int64_t and already knew
 * the unsigned-32 ceiling. Affected fields: attr 121 Device Serial Number,
 * attr 119 Puck Serial Number / Puck Hardware Rev (all read-only).
 *
 * Because the accumulator is now wider than 32 bits, size==4 signed fields must
 * be sign-extended explicitly too — int32_t used to get that for free by being
 * exactly the right width, so the old guard was `is_signed && size < 4u`. */
static int64_t load_be(const uint8_t *p, uint8_t size, uint8_t is_signed)
{
    int64_t v = 0;
    for (uint8_t i = 0u; i < size; ++i) {
        v = (v << 8) | (int64_t)p[i];
    }
    if (is_signed) {
        const int64_t sign_bit = (int64_t)1 << (size * 8u - 1u);
        if (v & sign_bit) {
            v -= (int64_t)1 << (size * 8u);
        }
    }
    return v;
}

/* Big-endian store, truncating to size bytes. Mirrors store_be() in
 * build/re-notes/hamfly_rmw_reference.c exactly. */
static void store_be(uint8_t *p, uint8_t size, int32_t raw)
{
    for (int8_t i = (int8_t)size - 1; i >= 0; --i) {
        p[i] = (uint8_t)(raw & 0xFF);
        raw >>= 8;
    }
}

/* Inclusive [lo, hi] that f->size/is_signed can represent, so a wildly
 * out-of-range physical*scale can never undefined-behaviour its way past a
 * narrow cast on the way into store_be(). */
static void raw_bounds(uint8_t size, uint8_t is_signed, int64_t *lo, int64_t *hi)
{
    if (size >= 4u) {
        if (is_signed) { *lo = -2147483647LL - 1; *hi = 2147483647LL; }
        else            { *lo = 0;                 *hi = 4294967295LL; }
        return;
    }
    const int64_t bits = (int64_t)size * 8;
    if (is_signed) {
        *hi = ((int64_t)1 << (bits - 1)) - 1;
        *lo = -((int64_t)1 << (bits - 1));
    } else {
        *lo = 0;
        *hi = ((int64_t)1 << bits) - 1;
    }
}

bool hamfly_settings_decode_field(const hamfly_attr_field_t *f,
                                   const uint8_t *payload, uint8_t payload_len,
                                   float *out_physical)
{
    if (!f || !payload || !out_physical) return false;
    /* Rule 3: the observed payload must actually contain this field. */
    if ((uint16_t)f->offset + (uint16_t)f->size > (uint16_t)payload_len) return false;
    if (f->scale == 0.0f) return false; /* guard a malformed table row */

    const int64_t raw = load_be(&payload[f->offset], f->size, f->is_signed);
    *out_physical = (float)raw / f->scale;
    return true;
}

bool hamfly_settings_get(uint16_t attr, const char *name,
                          const uint8_t *payload, uint8_t payload_len,
                          float *out_physical)
{
    const hamfly_attr_field_t *f = hamfly_settings_find_field(attr, name);
    return hamfly_settings_decode_field(f, payload, payload_len, out_physical);
}

bool hamfly_settings_encode_field(const hamfly_attr_field_t *f, float physical,
                                   int32_t *out_raw,
                                   float *out_clamped_physical,
                                   bool *out_was_clamped)
{
    if (!f || !out_raw) return false;
    if (isnan(physical) || isinf(physical)) return false; /* never encode a NaN/Inf raw */
    if (f->scale == 0.0f) return false; /* guard a malformed table row */

    float v = physical;
    bool clamped = false;
    if (f->has_range) {
        if (v < f->vmin) { v = f->vmin; clamped = true; }
        else if (v > f->vmax) { v = f->vmax; clamped = true; }
    }

    const double scaled = (double)v * (double)f->scale;
    /* Round-half-away-from-zero, matching AddFloatAsSignedShort() in
     * hamfly_qx_protocol.c so a value that also happens to be sent over the
     * QX277 control path would encode identically. */
    double rounded = (scaled >= 0.0) ? floor(scaled + 0.5) : ceil(scaled - 0.5);

    int64_t lo, hi;
    raw_bounds(f->size, f->is_signed, &lo, &hi);
    if (rounded < (double)lo) rounded = (double)lo;
    if (rounded > (double)hi) rounded = (double)hi;

    *out_raw = (int32_t)rounded;
    if (out_clamped_physical) *out_clamped_physical = v;
    if (out_was_clamped) *out_was_clamped = clamped;
    return true;
}

bool hamfly_settings_set(uint16_t attr, const char *name, float physical,
                          int32_t *out_raw,
                          float *out_clamped_physical,
                          bool *out_was_clamped)
{
    const hamfly_attr_field_t *f = hamfly_settings_find_field(attr, name);
    if (!f || !f->writable) return false; /* refuse writes where writable=0 */
    return hamfly_settings_encode_field(f, physical, out_raw, out_clamped_physical, out_was_clamped);
}

void hamfly_settings_store_raw(const hamfly_attr_field_t *f,
                                uint8_t *payload, uint8_t payload_len,
                                int32_t raw)
{
    if (!f || !payload) return;
    if ((uint16_t)f->offset + (uint16_t)f->size > (uint16_t)payload_len) return; /* Rule 3 guard */
    store_be(&payload[f->offset], f->size, raw);
}
