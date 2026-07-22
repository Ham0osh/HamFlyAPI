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
 * Settings transaction engines implementation.
 *
 * Consolidated 2026-07-22 from: hamfly_settings_rmw.c, hamfly_settings_read.c
 * RENAMED on merge: the read engine deliberately mirrored the RMW engine, so both defined static arm_attempt() and available_payload_len(). The read engine copies are now read_arm_attempt() and read_available_payload_len(). No behaviour change.
 * Content is unchanged apart from the merge itself (and the static renames
 * noted below where two files used the same internal helper name).
 */

#include <string.h>
#include "hamfly_settings_txn.h"
#include "hamfly_settings.h"

/* ==== from hamfly_settings_rmw.c ==== */

/* Start (or restart, on a Rule-1 retry) one attempt: send the appropriate
 * read for txn->attr and (re)arm the per-attempt budget. Returns HAMFLY_OK
 * on a successful send. */
static hamfly_result_t arm_attempt(hamfly_settings_rmw_t *txn)
{
    hamfly_gimbal_t *g = txn->g;
    const hamfly_result_t r = HAMFLY_ATTR_IS_QX(txn->attr)
        ? hamfly_request_attr(g, txn->attr)
        : hamfly_settings_send_qb_read(g, txn->attr);

    txn->polls_left  = HAMFLY_SETTINGS_RMW_POLL_BUDGET;
    txn->deadline_ms = txn->have_clock
        ? (g->hal.get_tick_ms(g->hal.ctx) + HAMFLY_SETTINGS_RMW_TIMEOUT_MS)
        : 0u;
    return r;
}

/* Bytes actually usable at g->pending_payload for txn->attr's reply. For
 * QX, pending_payload_len (= Header.MsgLength, set by the existing,
 * trusted QX_ParseHeader()) already IS just the payload. For QB, it is the
 * DATA length (attr byte + payload) because hamfly_qb_parse_header()
 * advances BufPayloadStart_p past the attr byte but hamfly_pump() snapshots
 * pending_payload_len from the pre-advance Header.MsgLength -- one MORE
 * than the bytes actually sitting at pending_payload. Correct for that
 * here, once, rather than trusting the raw length anywhere else (Rule 3:
 * length-match to what was OBSERVED). */
static uint8_t available_payload_len(const hamfly_settings_rmw_t *txn)
{
    const hamfly_gimbal_t *g = txn->g;
    if (HAMFLY_ATTR_IS_QX(txn->attr)) {
        return g->pending_payload_len;
    }
    return (g->pending_payload_len > 0u) ? (uint8_t)(g->pending_payload_len - 1u) : 0u;
}

hamfly_settings_rmw_state_t hamfly_settings_write_start(
    hamfly_settings_rmw_t *txn, hamfly_gimbal_t *g,
    uint16_t attr, const char *field_name,
    float physical_value, bool verify_readback)
{
    if (!txn) return HAMFLY_SETTINGS_RMW_ERR_ARG;
    memset(txn, 0, sizeof(*txn));

    if (!g || !field_name) {
        txn->state = HAMFLY_SETTINGS_RMW_ERR_ARG;
        return txn->state;
    }
    txn->g    = g;
    txn->attr = attr;

    const hamfly_attr_field_t *f = hamfly_settings_find_field(attr, field_name);
    if (!f) {
        txn->state = HAMFLY_SETTINGS_RMW_ERR_UNKNOWN_FIELD;
        return txn->state;
    }
    if (!f->writable) {
        txn->state = HAMFLY_SETTINGS_RMW_ERR_NOT_WRITABLE;
        return txn->state;
    }
    txn->field = f;

    int32_t raw;
    float   clamped_p;
    bool    was_clamped;
    if (!hamfly_settings_encode_field(f, physical_value, &raw, &clamped_p, &was_clamped)) {
        txn->state = HAMFLY_SETTINGS_RMW_ERR_RANGE;
        return txn->state;
    }
    txn->target_raw      = raw;
    txn->target_physical = clamped_p;
    txn->was_clamped      = was_clamped;
    txn->verify_readback  = verify_readback;

    /* Single-outstanding-request constraint -- see file header. */
    if (g->pending_attr != 0u && !g->pending_ready) {
        txn->state = HAMFLY_SETTINGS_RMW_ERR_BUSY;
        return txn->state;
    }

    hamfly_settings_wire_ensure_installed();

    txn->have_clock = (g->hal.get_tick_ms != NULL);
    txn->attempt    = 1u;

    if (arm_attempt(txn) != HAMFLY_OK) {
        txn->state = HAMFLY_SETTINGS_RMW_ERR_TX;
        return txn->state;
    }

    txn->state = HAMFLY_SETTINGS_RMW_READING;
    return txn->state;
}

/* Handle a reply that just arrived for the initial read: length-match
 * (Rule 3), capture the previous value, patch the target field only (the
 * read-modify part), set the QB type byte (Rule 4), and send the write. */
static void on_read_reply(hamfly_settings_rmw_t *txn, uint8_t avail)
{
    hamfly_gimbal_t *g = txn->g;
    const hamfly_attr_field_t *f = txn->field;

    if ((uint16_t)f->offset + (uint16_t)f->size > (uint16_t)avail || avail > sizeof(txn->buf)) {
        txn->state = HAMFLY_SETTINGS_RMW_ERR_TOO_SHORT;
        return;
    }

    memcpy(txn->buf, g->pending_payload, avail);
    txn->len = avail;

    /* Original value, for a caller-driven restore -- see file header on
     * why this is not assumed to be redundant with a power cycle. */
    (void)hamfly_settings_decode_field(f, txn->buf, txn->len, &txn->prev_physical);

    /* The read-modify-write: patch ONLY the target field; every other byte
     * is carried through untouched from what was just read. */
    hamfly_settings_store_raw(f, txn->buf, txn->len, txn->target_raw);
    if (!HAMFLY_ATTR_IS_QX(txn->attr)) {
        txn->buf[0] = 0x01u; /* Rule 4: QB WRITE_ABS type byte; QX has none */
    }

    const hamfly_result_t wr = HAMFLY_ATTR_IS_QX(txn->attr)
        ? hamfly_settings_send_qx_write(g, txn->attr, txn->buf, txn->len)
        : hamfly_settings_send_qb_write(g, txn->attr, txn->buf, txn->len);
    if (wr != HAMFLY_OK) {
        txn->state = HAMFLY_SETTINGS_RMW_ERR_TX;
        return;
    }

    if (!txn->verify_readback) {
        txn->state = HAMFLY_SETTINGS_RMW_OK;
        return;
    }

    /* Issue a fresh read of the same attribute to confirm the write. */
    txn->attempt = 1u;
    if (arm_attempt(txn) != HAMFLY_OK) {
        /* The WRITE already went out; only the verify read failed to
         * transmit. Surface as unverified, not as a write failure. */
        txn->state = HAMFLY_SETTINGS_RMW_ERR_UNVERIFIED;
        return;
    }
    txn->state = HAMFLY_SETTINGS_RMW_VERIFYING;
}

/* Handle a reply that just arrived for the post-write verify read: decode
 * the field and compare its raw bytes to what was intended. */
static void on_verify_reply(hamfly_settings_rmw_t *txn, uint8_t avail)
{
    const hamfly_gimbal_t *g = txn->g;
    const hamfly_attr_field_t *f = txn->field;

    float readback;
    if (!hamfly_settings_decode_field(f, g->pending_payload, avail, &readback)) {
        txn->state = HAMFLY_SETTINGS_RMW_ERR_UNVERIFIED;
        return;
    }
    int32_t rb_raw;
    float   dummy_p;
    bool    dummy_c;
    if (!hamfly_settings_encode_field(f, readback, &rb_raw, &dummy_p, &dummy_c)) {
        txn->state = HAMFLY_SETTINGS_RMW_ERR_UNVERIFIED;
        return;
    }
    txn->state = (rb_raw == txn->target_raw)
        ? HAMFLY_SETTINGS_RMW_OK
        : HAMFLY_SETTINGS_RMW_ERR_UNVERIFIED;
}

hamfly_settings_rmw_state_t hamfly_settings_write_poll(hamfly_settings_rmw_t *txn)
{
    if (!txn) return HAMFLY_SETTINGS_RMW_ERR_ARG;

    if (txn->state != HAMFLY_SETTINGS_RMW_READING &&
        txn->state != HAMFLY_SETTINGS_RMW_VERIFYING) {
        return txn->state; /* already terminal; repeated polls are a no-op */
    }

    hamfly_gimbal_t *g = txn->g;
    const bool is_verify = (txn->state == HAMFLY_SETTINGS_RMW_VERIFYING);

    if (g->pending_ready && g->pending_response_attr == txn->attr) {
        const uint8_t avail = available_payload_len(txn);
        /* Consume the slot immediately so a later, unrelated reply can
         * never be mistaken for this transaction's. */
        g->pending_attr  = 0u;
        g->pending_ready = false;

        if (is_verify) on_verify_reply(txn, avail);
        else            on_read_reply(txn, avail);
        return txn->state;
    }

    /* No reply this tick -- advance the retry/timeout budget. Never blocks:
     * at most one frame is sent per poll() call. */
    bool expired;
    if (txn->have_clock) {
        const uint32_t now = g->hal.get_tick_ms(g->hal.ctx);
        expired = (int32_t)(now - txn->deadline_ms) >= 0;
    } else {
        if (txn->polls_left > 0u) txn->polls_left--;
        expired = (txn->polls_left == 0u);
    }
    if (!expired) return txn->state;

    if (txn->attempt < HAMFLY_SETTINGS_RMW_READ_TRIES) {
        /* Rule 1: retry. */
        txn->attempt++;
        if (arm_attempt(txn) != HAMFLY_OK) {
            /* is_verify: the write already succeeded; only the confirmation
             * read's retry failed to transmit -- surface as unverified, not
             * as a write failure, for the same reason as the immediate-arm
             * case in on_read_reply(). !is_verify: still in the pre-write
             * read phase, so no write was ever sent -- fail closed as TX. */
            txn->state = is_verify ? HAMFLY_SETTINGS_RMW_ERR_UNVERIFIED
                                    : HAMFLY_SETTINGS_RMW_ERR_TX;
        }
        return txn->state;
    }

    /* Rule 2: refuse. For the initial read this means no write was ever
     * sent -- fail closed. For the verify read the write already happened;
     * this only means it could not be confirmed. */
    txn->state = is_verify ? HAMFLY_SETTINGS_RMW_ERR_UNVERIFIED
                            : HAMFLY_SETTINGS_RMW_ERR_NO_READ;
    return txn->state;
}

bool hamfly_settings_write_done(const hamfly_settings_rmw_t *txn)
{
    if (!txn) return true;
    return txn->state != HAMFLY_SETTINGS_RMW_READING &&
           txn->state != HAMFLY_SETTINGS_RMW_VERIFYING &&
           txn->state != HAMFLY_SETTINGS_RMW_IDLE;
}

/* ==== from hamfly_settings_read.c ==== */

/* ----------------------------------------------------------------------
 * Shift derivation -- pure, table-only. See file header for the rule.
 * ---------------------------------------------------------------------- */
uint8_t hamfly_settings_qx_shift(uint16_t attr)
{
    uint8_t min_offset = 0xFFu;
    bool    found = false;

    for (uint16_t i = 0u; i < HAMFLY_ATTR_FIELD_COUNT; ++i) {
        if (hamfly_attr_fields[i].attr != attr) continue;
        found = true;
        if (hamfly_attr_fields[i].offset < min_offset) {
            min_offset = hamfly_attr_fields[i].offset;
        }
    }

    if (!found) return 0u;               /* nothing to shift either way */
    return (min_offset >= 1u) ? 1u : 0u;
}

/* Start (or restart, on a Rule-1 retry) one attempt: send the read for
 * txn->attr via whichever sender txn->used_qx selected (frozen at start(),
 * see header), and (re)arm the per-attempt budget. */
static hamfly_result_t read_arm_attempt(hamfly_settings_read_t *txn)
{
    hamfly_gimbal_t *g = txn->g;
    const hamfly_result_t r = txn->used_qx
        ? hamfly_request_attr(g, txn->attr)
        : hamfly_settings_send_qb_read(g, txn->attr);

    txn->polls_left  = HAMFLY_SETTINGS_READ_POLL_BUDGET;
    txn->deadline_ms = txn->have_clock
        ? (g->hal.get_tick_ms(g->hal.ctx) + HAMFLY_SETTINGS_READ_TIMEOUT_MS)
        : 0u;
    return r;
}

/* Bytes actually usable at g->pending_payload for this reply. QB replies
 * need the same off-by-one correction hamfly_settings_rmw.c's
 * read_available_payload_len() applies (hamfly_qb_parse_header() advances past
 * the attr byte, but hamfly_pump() snapshots pending_payload_len from the
 * pre-advance length) -- that artifact is a property of the QB legacy
 * parse path itself, not of any particular attribute, so it applies
 * whenever txn->used_qx is false. QX replies (used_qx true, whether attr is
 * naturally QX or is a low attr read via HAMFLY_FRAMING_QX_READS) need no
 * such correction: QX_ParseHeader() already leaves pending_payload_len as
 * exactly the payload length. */
static uint8_t read_available_payload_len(const hamfly_settings_read_t *txn)
{
    const hamfly_gimbal_t *g = txn->g;
    if (txn->used_qx) {
        return g->pending_payload_len;
    }
    return (g->pending_payload_len > 0u) ? (uint8_t)(g->pending_payload_len - 1u) : 0u;
}

hamfly_settings_read_state_t hamfly_settings_read_start(
    hamfly_settings_read_t *txn, hamfly_gimbal_t *g, uint16_t attr)
{
    if (!txn) return HAMFLY_SETTINGS_READ_ERR_ARG;
    memset(txn, 0, sizeof(*txn));

    if (!g) {
        txn->state = HAMFLY_SETTINGS_READ_ERR_ARG;
        return txn->state;
    }
    txn->g    = g;
    txn->attr = attr;

    /* Single-outstanding-request constraint -- see hamfly_core_gimbal.h.
     * Refuse rather than stomp a transaction (settings or telemetry)
     * already in flight on the one shared slot. */
    if (hamfly_request_busy(g)) {
        txn->state = HAMFLY_SETTINGS_READ_ERR_BUSY;
        return txn->state;
    }

    const bool attr_is_qx = HAMFLY_ATTR_IS_QX(attr);
    /* attr > 255 has no QB framing to begin with -- always QX. attr <= 255
     * follows the pinned policy. */
    txn->used_qx = attr_is_qx || (hamfly_settings_get_framing() == HAMFLY_FRAMING_QX_READS);
    /* Shift only ever applies to a low attr actually read over QX -- a
     * naturally-QX attribute (attr > 255) was never QB-framed in the first
     * place, so there is no leading byte to compensate for. */
    txn->shift = (txn->used_qx && !attr_is_qx) ? hamfly_settings_qx_shift(attr) : 0u;

    hamfly_settings_wire_ensure_installed();

    txn->have_clock = (g->hal.get_tick_ms != NULL);
    txn->attempt    = 1u;

    if (read_arm_attempt(txn) != HAMFLY_OK) {
        txn->state = HAMFLY_SETTINGS_READ_ERR_TX;
        return txn->state;
    }

    txn->state = HAMFLY_SETTINGS_READ_READING;
    return txn->state;
}

hamfly_settings_read_state_t hamfly_settings_read_poll(hamfly_settings_read_t *txn)
{
    if (!txn) return HAMFLY_SETTINGS_READ_ERR_ARG;
    if (txn->state != HAMFLY_SETTINGS_READ_READING) {
        return txn->state; /* already terminal (or never started); no-op */
    }

    hamfly_gimbal_t *g = txn->g;

    if (g->pending_ready && g->pending_response_attr == txn->attr) {
        const uint8_t avail = read_available_payload_len(txn);
        const uint8_t cap   = (uint8_t)(sizeof(txn->buf) - txn->shift);

        if (avail > cap) {
            hamfly_request_release(g);
            txn->state = HAMFLY_SETTINGS_READ_ERR_TOO_SHORT;
            return txn->state;
        }

        if (txn->shift) txn->buf[0] = 0x00u; /* filler byte -- see file header */
        memcpy(&txn->buf[txn->shift], g->pending_payload, avail);
        txn->len = (uint8_t)(txn->shift + avail);

        /* Free the shared slot now that its payload has been copied out --
         * a later, unrelated reply can never be mistaken for this one. */
        hamfly_request_release(g);

        txn->state = HAMFLY_SETTINGS_READ_OK;
        return txn->state;
    }

    /* No reply this tick -- advance the retry/timeout budget. Never blocks:
     * at most one frame is sent per poll() call. */
    bool expired;
    if (txn->have_clock) {
        const uint32_t now = g->hal.get_tick_ms(g->hal.ctx);
        expired = (int32_t)(now - txn->deadline_ms) >= 0;
    } else {
        if (txn->polls_left > 0u) txn->polls_left--;
        expired = (txn->polls_left == 0u);
    }
    if (!expired) return txn->state;

    if (txn->attempt < HAMFLY_SETTINGS_READ_TRIES) {
        /* Rule 1: retry. */
        txn->attempt++;
        if (read_arm_attempt(txn) != HAMFLY_OK) {
            txn->state = HAMFLY_SETTINGS_READ_ERR_TX;
        }
        return txn->state;
    }

    /* Rule 2 analog: refuse. Fail closed -- no partial/garbage payload is
     * ever reported as OK. */
    txn->state = HAMFLY_SETTINGS_READ_ERR_NO_READ;
    return txn->state;
}

bool hamfly_settings_read_done(const hamfly_settings_read_t *txn)
{
    if (!txn) return true;
    return txn->state != HAMFLY_SETTINGS_READ_READING &&
           txn->state != HAMFLY_SETTINGS_READ_IDLE;
}

/* ----------------------------------------------------------------------
 * UI-oriented helpers -- all read-only views over a completed (state ==
 * OK) transaction's buffer. See hamfly_settings_read.h for the contract.
 * ---------------------------------------------------------------------- */

bool hamfly_settings_read_payload(const hamfly_settings_read_t *txn,
                                   const uint8_t **out_payload, uint8_t *out_len)
{
    if (!txn || txn->state != HAMFLY_SETTINGS_READ_OK) return false;
    if (out_payload) *out_payload = txn->buf;
    if (out_len)     *out_len     = txn->len;
    return true;
}

uint8_t hamfly_settings_read_field_count(const hamfly_settings_read_t *txn)
{
    if (!txn || txn->state != HAMFLY_SETTINGS_READ_OK) return 0u;
    return hamfly_settings_field_count(txn->attr);
}

bool hamfly_settings_read_field_at(const hamfly_settings_read_t *txn, uint8_t index,
                                    const hamfly_attr_field_t **out_field,
                                    float *out_physical)
{
    if (!txn || txn->state != HAMFLY_SETTINGS_READ_OK) return false;

    const hamfly_attr_field_t *f = hamfly_settings_field_at(txn->attr, index);
    if (!f) return false;

    float v;
    if (!hamfly_settings_decode_field(f, txn->buf, txn->len, &v)) return false;

    if (out_field)    *out_field    = f;
    if (out_physical) *out_physical = v;
    return true;
}

bool hamfly_settings_read_get(const hamfly_settings_read_t *txn, const char *name,
                               float *out_physical)
{
    if (!txn || txn->state != HAMFLY_SETTINGS_READ_OK) return false;
    return hamfly_settings_get(txn->attr, name, txn->buf, txn->len, out_physical);
}
