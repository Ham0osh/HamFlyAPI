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
 * Settings primitives: attribute-field access, framing policy, QB/QX wire.
 *
 * Consolidated 2026-07-22 from: hamfly_settings_field.h, hamfly_settings_framing.h, hamfly_settings_wire.h
 * These three were separate modules; they are always linked together, so the split bought nothing.
 * Content is unchanged apart from the merge itself (and the static renames
 * noted below where two files used the same internal helper name).
 */

#ifndef HAMFLY_SETTINGS_H
#define HAMFLY_SETTINGS_H

#include <stdbool.h>
#include <stdint.h>
#include "hamfly_attr_table.h"
#include "hamfly_core_gimbal.h"

/* ==== from hamfly_settings_field.h ==== */

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

/* ==== from hamfly_settings_framing.h ==== */

typedef enum {
    /* Default. Unchanged behaviour from before this task: QB for attr <=
     * 255, QX for attr > 255. Every existing caller that never touches this
     * policy sees no behaviour change. */
    HAMFLY_FRAMING_AUTO = 0,

    /* All READS go over QX, attr <= 255 included, with automatic +1
     * payload-shift compensation applied per hamfly_settings_qx_shift() so
     * decoded values match HAMFLY_FRAMING_AUTO exactly. Writes are UNCHANGED
     * -- still QB for attr <= 255 (see file header). */
    HAMFLY_FRAMING_QX_READS = 1
} hamfly_framing_policy_t;

/* Pin the policy. Takes effect on the next hamfly_settings_read_start() call
 * -- does not affect a transaction already in flight. */
void hamfly_settings_set_framing(hamfly_framing_policy_t p);

/* Current policy. HAMFLY_FRAMING_AUTO until set otherwise. */
hamfly_framing_policy_t hamfly_settings_get_framing(void);

/* ==== from hamfly_settings_wire.h ==== */

/* Idempotent, process-wide. Installs the QB legacy RX header parser (see
 * file header above). Every sender below calls this itself, so an
 * integrator never has to remember to; exposed publicly only in case a
 * caller wants to arm it once, early, e.g. right after hamfly_init(). Safe
 * to call any number of times, from any number of gimbal instances -- the
 * hook it installs is stateless with respect to which gimbal is talking. */
void hamfly_settings_wire_ensure_installed(void);

/* ----------------------------------------------------------------------
 * Pure frame builders -- no I/O, directly unit-testable against the golden
 * vectors in hamfly_test_vectors.md.
 * ---------------------------------------------------------------------- */

/* QB read request: 'Q','B', 16-bit BE length(=1), attr byte (plain, no
 * write flag, no type byte -- reads carry neither). Always exactly 6 bytes.
 * Returns 6, or 0 if attr > 255 / out_max < 6. */
uint8_t hamfly_qb_build_read(uint16_t attr, uint8_t *out_buf, uint8_t out_max);

/* QB write frame. `payload` is the FULL struct payload (type byte already
 * patched to 0x01 by the caller -- Rule 4) of length payload_len, which
 * MUST equal the length observed on the preceding read (Rule 3 -- this
 * function does not second-guess it, the RMW engine is responsible for
 * length-matching before calling here). Frame =
 * 'Q','B',lenHi,lenLo,(attr|0x80),payload[0..len-1],checksum.
 * Returns the frame length (payload_len + 5), or 0 on error (attr > 255,
 * payload_len == 0 or > 64, output buffer too small). */
uint8_t hamfly_qb_build_write(uint16_t attr, const uint8_t *payload, uint8_t payload_len,
                               uint8_t *out_buf, uint8_t out_max);

/* QX write frame, generalising hamfly_write_attr_u8()'s hand-rolled single
 * byte case to an arbitrary payload (needed because attr 383 alone has 13
 * fields -- QX struct writes need RMW exactly like QB ones do). Frame =
 * 'Q','X',len,<attrib varint>,0x02(WRITE_ABS),0x0A(src),gimbal_id(tgt),
 * 0x00(trid),0x00(rrid),payload[0..len-1],checksum. Unlike
 * hamfly_write_attr_u8() this uses the FULL 4-byte/28-bit varint encoding
 * (matching QX_AddExtdValToBuf's own algorithm) rather than assuming
 * attr <= 0x3FFF -- see review Finding 9. Refuses (returns 0) rather than
 * emit the 2-byte extended QX length form; no settings attribute needs a
 * body over 127 bytes. */
uint8_t hamfly_qx_build_write(uint16_t attr, uint8_t gimbal_id,
                               const uint8_t *payload, uint8_t payload_len,
                               uint8_t *out_buf, uint8_t out_max);

/* ----------------------------------------------------------------------
 * Senders -- build the frame above, push it through hal.uart_putc() (same
 * direct-to-HAL pattern as hamfly_write_attr_u8(), bypassing g->txbuf), and
 * update g->statistics the same way every other sender in this API does.
 * ---------------------------------------------------------------------- */

hamfly_result_t hamfly_settings_send_raw(hamfly_gimbal_t *g, const uint8_t *buf, uint8_t len);

/* Sends the QB read and sets g->pending_attr/pending_sent_ms/pending_ready
 * exactly like hamfly_request_attr() does, so hamfly_pump()'s existing
 * capture logic (now able to see QB replies) delivers the response the
 * normal way. */
hamfly_result_t hamfly_settings_send_qb_read(hamfly_gimbal_t *g, uint16_t attr);

hamfly_result_t hamfly_settings_send_qb_write(hamfly_gimbal_t *g, uint16_t attr,
                                               const uint8_t *payload, uint8_t payload_len);

hamfly_result_t hamfly_settings_send_qx_write(hamfly_gimbal_t *g, uint16_t attr,
                                               const uint8_t *payload, uint8_t payload_len);

#endif /* HAMFLY_SETTINGS_H */
