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
 * Settings transactions: the read engine and the read-modify-write engine.
 *
 * Consolidated 2026-07-22 from: hamfly_settings_rmw.h, hamfly_settings_read.h
 * Both are non-blocking, poll-per-tick state machines over the same shared request slot; keeping them together makes that relationship visible.
 * Content is unchanged apart from the merge itself (and the static renames
 * noted below where two files used the same internal helper name).
 */

#ifndef HAMFLY_SETTINGS_TXN_H
#define HAMFLY_SETTINGS_TXN_H

#include <stdbool.h>
#include <stdint.h>
#include "hamfly_core_gimbal.h"
#include "hamfly_attr_table.h"
#include "hamfly_settings.h"

/* ==== from hamfly_settings_rmw.h ==== */

/* Retries the read (or the post-write verify read) this many times before
 * giving up -- matches Rule 1 in hamfly_rmw_reference.c. */
#ifndef HAMFLY_SETTINGS_RMW_READ_TRIES
#define HAMFLY_SETTINGS_RMW_READ_TRIES 4u
#endif

/* Per-attempt budget in poll() calls, used when hal.get_tick_ms is NULL. */
#ifndef HAMFLY_SETTINGS_RMW_POLL_BUDGET
#define HAMFLY_SETTINGS_RMW_POLL_BUDGET 100u
#endif

/* Per-attempt budget in milliseconds, used when hal.get_tick_ms is present. */
#ifndef HAMFLY_SETTINGS_RMW_TIMEOUT_MS
#define HAMFLY_SETTINGS_RMW_TIMEOUT_MS 200u
#endif

typedef enum {
    HAMFLY_SETTINGS_RMW_IDLE = 0,

    /* In flight -- call poll() again next tick. */
    HAMFLY_SETTINGS_RMW_READING,
    HAMFLY_SETTINGS_RMW_VERIFYING,

    /* Terminal: success. */
    HAMFLY_SETTINGS_RMW_OK,

    /* Terminal: refused before any I/O -- nothing was sent. */
    HAMFLY_SETTINGS_RMW_ERR_ARG,
    HAMFLY_SETTINGS_RMW_ERR_UNKNOWN_FIELD,
    HAMFLY_SETTINGS_RMW_ERR_NOT_WRITABLE,
    HAMFLY_SETTINGS_RMW_ERR_RANGE,     /* physical value was NaN/Inf */
    HAMFLY_SETTINGS_RMW_ERR_BUSY,      /* another request already in flight */

    /* Terminal: refused after the read failed -- Rule 2. No write sent;
     * siblings on the gimbal (if the struct exists at all) are untouched. */
    HAMFLY_SETTINGS_RMW_ERR_NO_READ,

    /* Terminal: a reply DID arrive but did not reach [offset:offset+size)
     * -- Rule 3, guards a stale/wrong table row. No write sent. */
    HAMFLY_SETTINGS_RMW_ERR_TOO_SHORT,

    /* Terminal: a frame failed to transmit (UART fault). May have occurred
     * while sending the read (nothing changed on the gimbal) or while
     * sending the write itself (gimbal state now INDETERMINATE -- see the
     * file header). Check statistics.uart_err_flags for which. */
    HAMFLY_SETTINGS_RMW_ERR_TX,

    /* Terminal: the write was sent, but the post-write verify read either
     * did not confirm the new value or never completed. The write DID go
     * out; this only means confirmation failed. */
    HAMFLY_SETTINGS_RMW_ERR_UNVERIFIED
} hamfly_settings_rmw_state_t;

typedef struct {
    hamfly_settings_rmw_state_t state;
    hamfly_gimbal_t            *g;
    const hamfly_attr_field_t  *field;
    uint16_t attr;

    uint8_t  buf[64];    /* working copy of the struct payload (Rules 2/3) */
    uint8_t  len;        /* OBSERVED payload length from the read (Rule 3) */

    int32_t  target_raw;
    float    target_physical;  /* clamped target, fixed at start() */
    float    prev_physical;    /* value read back before the patch, for a
                                 * caller-driven restore -- write persistence
                                 * across a power cycle is unknown on this
                                 * platform (see hamfly_rmw_reference.c) */
    bool     was_clamped;
    bool     verify_readback;

    uint8_t  attempt;         /* 1 .. HAMFLY_SETTINGS_RMW_READ_TRIES */
    uint16_t polls_left;      /* remaining poll() calls this attempt, no-clock path */
    uint32_t deadline_ms;     /* attempt deadline, clock path */
    bool     have_clock;
} hamfly_settings_rmw_t;

/* Validate + send the first read. Never blocks. Returns the new state
 * (HAMFLY_SETTINGS_RMW_READING on success, or a terminal ERR_* if refused
 * before any I/O -- see the enum above for exactly which checks are
 * I/O-free). `physical_value` is clamped to the field's declared range
 * before anything is sent; use txn->was_clamped / txn->target_physical
 * after this call to see whether/what it was clamped to. */
hamfly_settings_rmw_state_t hamfly_settings_write_start(
    hamfly_settings_rmw_t *txn, hamfly_gimbal_t *g,
    uint16_t attr, const char *field_name,
    float physical_value, bool verify_readback);

/* Advance the transaction by one tick. Call once per main-loop iteration,
 * AFTER hamfly_pump(), until hamfly_settings_write_done() is true. Never
 * blocks; a no-op once terminal. Returns the current state. */
hamfly_settings_rmw_state_t hamfly_settings_write_poll(hamfly_settings_rmw_t *txn);

/* True once state is OK or any ERR_* (i.e. no longer READING/VERIFYING). */
bool hamfly_settings_write_done(const hamfly_settings_rmw_t *txn);

/* ==== from hamfly_settings_read.h ==== */

/* Retries a read this many times before giving up. Mirrors
 * HAMFLY_SETTINGS_RMW_READ_TRIES (hamfly_settings_rmw.h) -- same established
 * budget, Rule 1 (a busy gimbal misses the first request) applies here too. */
#ifndef HAMFLY_SETTINGS_READ_TRIES
#define HAMFLY_SETTINGS_READ_TRIES 4u
#endif

/* Per-attempt budget in poll() calls, used when hal.get_tick_ms is NULL. */
#ifndef HAMFLY_SETTINGS_READ_POLL_BUDGET
#define HAMFLY_SETTINGS_READ_POLL_BUDGET 100u
#endif

/* Per-attempt budget in milliseconds, used when hal.get_tick_ms is present. */
#ifndef HAMFLY_SETTINGS_READ_TIMEOUT_MS
#define HAMFLY_SETTINGS_READ_TIMEOUT_MS 200u
#endif

typedef enum {
    HAMFLY_SETTINGS_READ_IDLE = 0,

    /* In flight -- call poll() again next tick. */
    HAMFLY_SETTINGS_READ_READING,

    /* Terminal: success -- txn now holds a decodable payload. */
    HAMFLY_SETTINGS_READ_OK,

    /* Terminal: refused before any I/O -- nothing was sent. */
    HAMFLY_SETTINGS_READ_ERR_ARG,
    HAMFLY_SETTINGS_READ_ERR_BUSY,     /* another request already in flight
                                         * -- see hamfly_request_busy() */

    /* Terminal: all HAMFLY_SETTINGS_READ_TRIES attempts timed out with no
     * reply -- fail closed, same shape as RMW's ERR_NO_READ. */
    HAMFLY_SETTINGS_READ_ERR_NO_READ,

    /* Terminal: a frame failed to transmit (UART fault). Nothing on the
     * gimbal is touched by a read either way. */
    HAMFLY_SETTINGS_READ_ERR_TX,

    /* Terminal: a reply DID arrive but its observed length (plus the shift
     * filler byte, if any) would not fit txn->buf -- Rule 3 style guard,
     * mirrors HAMFLY_SETTINGS_RMW_ERR_TOO_SHORT. Unreachable given today's
     * 64-byte g->pending_payload cap and 65-byte txn->buf, but a payload
     * that somehow arrived wider than expected is refused outright rather
     * than silently truncated. */
    HAMFLY_SETTINGS_READ_ERR_TOO_SHORT
} hamfly_settings_read_state_t;

typedef struct {
    hamfly_settings_read_state_t state;
    hamfly_gimbal_t              *g;
    uint16_t attr;
    bool     used_qx;    /* which sender THIS transaction actually used --
                           * frozen at start() from the framing policy at
                           * that moment, so a policy change mid-flight
                           * cannot retroactively reinterpret an in-flight
                           * transaction's bytes. */
    uint8_t  shift;      /* 0 or 1 filler bytes prepended ahead of buf[]
                           * before the real payload -- see
                           * hamfly_settings_qx_shift(). Always 0 when
                           * used_qx is false (QB payloads are never
                           * shifted; the RMW engine's available_payload_len()
                           * precedent already establishes QB needs no such
                           * correction here). */

    uint8_t  buf[65];    /* shift byte (<=1) + up to 64 payload bytes,
                           * decodable directly against hamfly_attr_table.h's
                           * QB-relative offsets regardless of which framing
                           * was actually used on the wire. */
    uint8_t  len;         /* bytes valid in buf[] (shift + observed payload) */

    uint8_t  attempt;      /* 1 .. HAMFLY_SETTINGS_READ_TRIES */
    uint16_t polls_left;   /* remaining poll() calls this attempt, no-clock path */
    uint32_t deadline_ms;  /* attempt deadline, clock path */
    bool     have_clock;
} hamfly_settings_read_t;

/* Scan hamfly_attr_fields[] for every row under attr and return the minimum
 * offset seen (0 if attr has no rows at all -- nothing to shift either way).
 * Pure, table-only, no I/O -- directly unit-testable and reusable by a
 * caller that wants to know the shift before a reply has even arrived (e.g.
 * to decide up front whether HAMFLY_FRAMING_QX_READS is safe for a given
 * attr on a table it has audited itself). Returns 1 if that minimum is >= 1
 * (a leading byte QB carries and QX drops -- needs the filler byte), 0
 * otherwise. See the file header for the full derivation rationale. */
uint8_t hamfly_settings_qx_shift(uint16_t attr);

/* Validate + send the first read for attr, using the sender the CURRENT
 * hamfly_settings_get_framing() policy selects (frozen into txn->used_qx for
 * the lifetime of this transaction). Never blocks. Returns the new state:
 * HAMFLY_SETTINGS_READ_READING on success, or a terminal ERR_* if refused
 * before any I/O. Refuses with HAMFLY_SETTINGS_READ_ERR_BUSY if
 * hamfly_request_busy(g) is already true -- this engine will not stomp a
 * transaction (settings or telemetry) already in flight on the shared slot. */
hamfly_settings_read_state_t hamfly_settings_read_start(
    hamfly_settings_read_t *txn, hamfly_gimbal_t *g, uint16_t attr);

/* Advance the transaction by one tick. Call once per main-loop iteration,
 * AFTER hamfly_pump(), until hamfly_settings_read_done() is true. Never
 * blocks; a no-op once terminal. Releases the shared request slot itself
 * (hamfly_request_release()) the moment a matching reply is consumed or the
 * transaction gives up -- the caller does not need to call it separately.
 * Returns the current state. */
hamfly_settings_read_state_t hamfly_settings_read_poll(hamfly_settings_read_t *txn);

/* True once state is OK or any ERR_* (i.e. no longer READING). */
bool hamfly_settings_read_done(const hamfly_settings_read_t *txn);

/* ----------------------------------------------------------------------
 * UI-oriented helpers -- operate on a COMPLETED (state == OK) transaction.
 * All are read-only views over txn->buf; none send anything. Every one
 * returns false / 0 / NULL on a txn that is not yet OK, so a UI can poll
 * these speculatively without checking state twice.
 * ---------------------------------------------------------------------- */

/* Raw decodable payload from the completed read: *out_payload points at
 * txn->buf (valid only as long as txn is), *out_len is the byte count
 * (shift already applied). For a caller that wants the whole struct rather
 * than named/enumerated fields (e.g. to hex-dump it, or to hand to a
 * caller-supplied decoder for a field the table has no row for). Returns
 * false (out-pointers untouched) if txn is NULL or not in state OK. */
bool hamfly_settings_read_payload(const hamfly_settings_read_t *txn,
                                   const uint8_t **out_payload, uint8_t *out_len);

/* Number of table rows (fields) under the attribute just read -- thin
 * wrapper over hamfly_settings_field_count(txn->attr) that also requires
 * the read to have completed successfully, so a UI walking 0..count-1 never
 * does so against a payload that never arrived. */
uint8_t hamfly_settings_read_field_count(const hamfly_settings_read_t *txn);

/* Enumerate: the Nth field (0-indexed, table order) under the completed
 * read's attribute, decoded against txn's payload. *out_field is the table
 * row (name/units/writable/range all available from it); *out_physical is
 * the decoded value. Either out-pointer may be NULL if not wanted. Returns
 * false (nothing decoded) if txn is not OK, index is out of range, or the
 * field does not fit within the observed payload length (Rule 3 -- guards a
 * stale/wrong table row exactly like hamfly_settings_decode_field() does). */
bool hamfly_settings_read_field_at(const hamfly_settings_read_t *txn, uint8_t index,
                                    const hamfly_attr_field_t **out_field,
                                    float *out_physical);

/* Convenience: find field `name` under the completed read's attribute and
 * decode it. Returns false if txn is not OK, the name is unknown for this
 * attribute, or Rule 3 refuses (payload too short for this field). */
bool hamfly_settings_read_get(const hamfly_settings_read_t *txn, const char *name,
                               float *out_physical);

#endif /* HAMFLY_SETTINGS_TXN_H */
