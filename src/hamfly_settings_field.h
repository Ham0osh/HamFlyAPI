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
 * ============================================================================
 * Lookup accessors and typed physical <-> raw codec for hamfly_attr_table.h.
 *
 * Pure functions only: no I/O, no gimbal struct, nothing here touches the
 * wire. Every function is directly callable on a byte array with no
 * hardware attached, which is exactly what makes the golden-vector tests in
 * hamfly_settings_tests.c possible offline.
 *
 * NOT build-verified -- no C compiler was available when this was written.
 */

#ifndef HAMFLY_SETTINGS_FIELD_H
#define HAMFLY_SETTINGS_FIELD_H

#include <stdint.h>
#include <stdbool.h>

#include "hamfly_attr_table.h"

/* Linear lookup of one named field under one attribute. Returns NULL if the
 * attribute has no rows in the table (e.g. attr 110 -- deliberately absent,
 * see hamfly_attr_table.h) or the name doesn't match any row under it. */
const hamfly_attr_field_t *hamfly_settings_find_field(uint16_t attr, const char *name);

/* How many rows (fields) exist under attr, and the Nth one (0-indexed) --
 * for a caller that wants to walk a whole struct rather than name one
 * field, e.g. to log every field after a read. field_at() returns NULL if
 * index is out of range. */
uint8_t hamfly_settings_field_count(uint16_t attr);
const hamfly_attr_field_t *hamfly_settings_field_at(uint16_t attr, uint8_t index);

/* ----------------------------------------------------------------------
 * Typed field get/set (task step 4)
 * ---------------------------------------------------------------------- */

/* Decode ONE field out of an already-read payload buffer into physical
 * units (physical = raw / scale, raw sign-extended per is_signed).
 * `payload`/`payload_len` are whatever was actually observed on the wire
 * for this attribute (e.g. g->pending_payload / g->pending_payload_len
 * after a read, already corrected for the QB attr-byte offset -- see
 * hamfly_settings_rmw.c). Returns false, *out untouched, if f is NULL or
 * the payload does not reach [f->offset : f->offset+f->size) -- this is
 * Rule 3 (never trust a table-assumed length over what was observed). */
bool hamfly_settings_decode_field(const hamfly_attr_field_t *f,
                                   const uint8_t *payload, uint8_t payload_len,
                                   float *out_physical);

/* Convenience: find_field() + decode_field() in one call. */
bool hamfly_settings_get(uint16_t attr, const char *name,
                          const uint8_t *payload, uint8_t payload_len,
                          float *out_physical);

/* Convert a physical value to the raw wire integer for field f:
 *   - refuses (returns false) NaN/Inf -- never encoded, never written.
 *   - clamps to [f->vmin, f->vmax] when f->has_range (never wraps).
 *   - clamps the *integer* result to what f->size/is_signed can represent,
 *     so the cast to a narrow field width is never undefined behaviour
 *     even if scale/physical combine to something absurd.
 * *out_clamped_physical (optional) is the physical value actually encoded
 * (after range clamp). *out_was_clamped (optional) is true if clamping
 * changed the caller's requested value. */
bool hamfly_settings_encode_field(const hamfly_attr_field_t *f, float physical,
                                   int32_t *out_raw,
                                   float *out_clamped_physical,
                                   bool *out_was_clamped);

/* Convenience: find_field() (refusing unknown or writable=0 fields) +
 * encode_field() in one call. Returns false without encoding anything if
 * the field is unknown or not proven writable on hardware. */
bool hamfly_settings_set(uint16_t attr, const char *name, float physical,
                          int32_t *out_raw,
                          float *out_clamped_physical,
                          bool *out_was_clamped);

/* Patch raw (big-endian, f->size bytes) into payload[f->offset : +size),
 * leaving every other byte untouched -- this is the "modify" step of
 * read-modify-write. No-op if the field does not fit within payload_len
 * (Rule 3 guard, defensive -- callers should already have checked this via
 * decode_field()/the RMW engine before calling store). */
void hamfly_settings_store_raw(const hamfly_attr_field_t *f,
                                uint8_t *payload, uint8_t payload_len,
                                int32_t raw);

#endif /* HAMFLY_SETTINGS_FIELD_H */
