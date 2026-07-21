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
 * Read-modify-write engine for MoVI Pro GCU struct settings.
 *
 * THE CENTRAL DESIGN PROBLEM AND THE CHOICE MADE HERE
 * --------------------------------------------------------------------------
 * A correct write needs read-THEN-write of the same struct (see the four
 * rules in build/re-notes/hamfly_rmw_reference.c). But the read is
 * asynchronous -- hamfly_request_attr()/hamfly_settings_send_qb_read() only
 * send it; the reply lands later, only when the application calls
 * hamfly_pump(), from the same single-threaded main loop this code also
 * runs on. Blocking here (loop calling hamfly_pump() until pending_ready)
 * is not available: it would stall whatever else that main loop does (the
 * ~100 Hz control send, for one), and on the one HAL this API ships today
 * (hamfly_psoc5_hal()), hal.get_tick_ms is NULL, so a blocking helper
 * couldn't even bound its own wait by wall-clock time.
 *
 * CHOICE: an explicit application-driven state machine (start / poll /
 * done), not a blocking helper and not a caller-supplied pump callback.
 *   - start() validates everything I/O-free (unknown field, writable=0,
 *     NaN/Inf, a transaction already in flight) and, only if all of that
 *     passes, sends ONE read request.
 *   - poll() does O(1) work and never blocks: call it once per main-loop
 *     tick, after hamfly_pump(). It checks g->pending_ready, and otherwise
 *     advances a per-attempt budget.
 *   - Timeout without a clock: hal.get_tick_ms is optional (may be NULL --
 *     F1). When present, each attempt is bounded by a real millisecond
 *     deadline. When absent -- the common case today -- each attempt is
 *     instead bounded by a POLL-CALL COUNT (HAMFLY_SETTINGS_RMW_POLL_BUDGET
 *     poll() invocations). This is only a proxy for elapsed time (it
 *     stretches or shrinks with however often the app actually calls
 *     poll()), but it guarantees the one property that actually matters
 *     for fail-closed behaviour: poll() can NEVER wedge waiting forever,
 *     with or without any HAL clock. A caller that needs a true wall-clock
 *     bound should wire up hal.get_tick_ms; one that can't still gets a
 *     transaction that provably terminates.
 *   - No hamfly_settings_write_blocking() convenience wrapper is provided
 *     on purpose. Any implementation of one could only bound itself by
 *     looping hamfly_pump() a fixed number of times when no clock exists --
 *     exactly the footgun the state machine exists to avoid. The
 *     application must drive start()/poll() itself.
 *
 * FAIL-CLOSED, PRECISELY
 * --------------------------------------------------------------------------
 * The write is sent from exactly one place (inside poll(), the instant a
 * matching, sufficiently-long reply arrives) and only after the read has
 * fully succeeded. If the read never succeeds (all
 * HAMFLY_SETTINGS_RMW_READ_TRIES attempts time out), the engine reaches
 * HAMFLY_SETTINGS_RMW_ERR_NO_READ and NO write frame is ever built or sent
 * -- the struct on the gimbal, if it exists at all, is untouched and every
 * sibling field is exactly as it was. This is Rule 2, enforced structurally
 * rather than by convention: there is no code path from "read did not
 * succeed" to "send a write".
 *
 * The one failure mode this design cannot retroactively undo is a UART
 * fault occurring DURING the write transmission itself (after the read
 * succeeded and the frame was correctly built) -- see
 * HAMFLY_SETTINGS_RMW_ERR_TX and the Risks section of the delivery report.
 * In that case the field's true value on the gimbal is INDETERMINATE (old,
 * new, or a corrupted partial frame the checksum will reject) and the
 * caller must treat it as unknown until a fresh read confirms it -- there
 * is no software mechanism that can make a byte already handed to
 * hal.uart_putc() un-sent.
 *
 * SINGLE-OUTSTANDING-REQUEST CONSTRAINT
 * --------------------------------------------------------------------------
 * g->pending_attr/pending_ready/pending_payload is a single shared slot on
 * hamfly_gimbal_t (pre-existing design, not introduced here) -- one request
 * in flight at a time, matched purely by attribute id. While a transaction
 * is in HAMFLY_SETTINGS_RMW_READING or _VERIFYING, do not also call
 * hamfly_request_attr(), hamfly_request_attr_capture(), or start a second
 * hamfly_settings_rmw_t transaction on the same hamfly_gimbal_t: whichever
 * request goes out last claims the slot, and an earlier one's reply (or
 * this transaction's) will never be matched. start() refuses with
 * HAMFLY_SETTINGS_RMW_ERR_BUSY if the slot is already claimed when called,
 * but cannot protect a transaction already in flight from a later,
 * unrelated request the application makes directly.
 *
 * QB vs QX ROUTING
 * --------------------------------------------------------------------------
 * attr <= 255 (QB): read via hamfly_settings_send_qb_read() (this task's
 * new code -- see hamfly_settings_wire.h for why the existing
 * hamfly_request_attr() cannot be reused here), write via
 * hamfly_settings_send_qb_write().
 * attr > 255 (QX): read via the EXISTING, unmodified hamfly_request_attr()
 * -- the general QX build/parse pipeline is already exercised by the
 * 277/287 control/status exchange and is trusted -- write via
 * hamfly_settings_send_qx_write() (new: the existing QX client parser
 * callback only knows attr 277/287 and would refuse anything else with
 * AttNotHandled).
 *
 * NOT build-verified -- no C compiler was available when this was written.
 */

#ifndef HAMFLY_SETTINGS_RMW_H
#define HAMFLY_SETTINGS_RMW_H

#include <stdint.h>
#include <stdbool.h>

#include "hamfly_core_gimbal.h"
#include "hamfly_attr_table.h"

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

#endif /* HAMFLY_SETTINGS_RMW_H */
