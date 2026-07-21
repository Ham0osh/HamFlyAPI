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
 * See hamfly_settings_wire.h for why this module exists and how it wires
 * itself into the existing (unmodified) QX_RxMsg() dispatch without editing
 * hamfly_qx_protocol.c or hamfly_core_gimbal.c.
 *
 * NOT build-verified -- no C compiler was available when this was written.
 */

#include "hamfly_settings_wire.h"

#include <string.h>

#include "hamfly_qx_protocol.h"

/* ============================================================
 * QB legacy RX header parser -- installed into QX_ParseHeader_Legacy.
 * ============================================================ */

/* QB DATA layout on this wire (build/re-notes/hamfly_test_vectors.md secs
 * 1-3): DATA = [attr_id_byte, payload...]. A read's DATA is the attr byte
 * alone (no payload at all); a write's or reply's DATA carries the
 * attribute's own struct payload after it, whose first byte is that
 * struct's "type" marker (0x00 CURVAL / 0x01 WRITE_ABS) -- that convention
 * belongs to the attribute's field layout (hamfly_attr_table.h), not to
 * this outer framing, so it is not modelled here.
 *
 * This node is exclusively the QX_DEV_ID_MOVI_API_CONTROLLER *client*; nothing
 * in this API registers it as a QB *server*. Every QB frame it ever
 * receives is therefore a reply to its own read or write, so Header.Type
 * is unconditionally treated as CURVAL (matches how QX_Cli_Rx_CurVal()
 * dispatches it downstream). */
static void hamfly_qb_parse_header(QX_Msg_t *Msg_p)
{
    Msg_p->MsgBuf_p = Msg_p->MsgBufAtt_p;

    /* QX_CommsPorts[].RxMsg persists across frames -- these fields are only
     * ever set by QX_ParseHeader() for a QX frame's option byte, so a QB
     * frame following a QX frame could otherwise inherit stale state that
     * gates real branches inside QX_RxMsg() (the CRC32 block; FF_Ext skip).
     * QB carries neither, so both are unconditionally false here. */
    Msg_p->Header.AddCRC32 = 0u;
    Msg_p->Header.FF_Ext   = 0u;
    Msg_p->Header.Type     = QX_MSG_TYPE_CURVAL;

    if (Msg_p->Header.MsgLength < 1u) {
        /* Malformed/empty DATA: no attribute byte to read. Leave Attrib at
         * 0 (never a valid settings attribute) so nothing spuriously
         * matches a caller's pending_attr. */
        Msg_p->Header.Attrib = 0u;
        return;
    }

    Msg_p->Header.Attrib = (uint32_t)(*Msg_p->MsgBuf_p & 0x7Fu);
    Msg_p->MsgBuf_p++;
    /* QX_RxMsg() sets BufPayloadStart_p = MsgBuf_p right after this call
     * returns, i.e. to the byte immediately after the attr byte -- the
     * struct payload (type byte + fields), matching the RMW engine's
     * expectations exactly. */
}

static uint8_t s_qb_legacy_installed = 0u;

void hamfly_settings_wire_ensure_installed(void)
{
    if (s_qb_legacy_installed) return;
    QX_ParseHeader_Legacy = hamfly_qb_parse_header;
    s_qb_legacy_installed = 1u;
}

/* ============================================================
 * Pure frame builders
 * ============================================================ */

uint8_t hamfly_qb_build_read(uint16_t attr, uint8_t *out_buf, uint8_t out_max)
{
    if (HAMFLY_ATTR_IS_QX(attr) || !out_buf || out_max < 6u) return 0u;

    out_buf[0] = (uint8_t)'Q';
    out_buf[1] = (uint8_t)'B';
    out_buf[2] = 0x00u;                 /* DATA length hi -- always 1 byte of DATA */
    out_buf[3] = 0x01u;                 /* DATA length lo */
    out_buf[4] = (uint8_t)(attr & 0x7Fu);
    uint16_t sum = out_buf[4];
    out_buf[5] = (uint8_t)(0xFFu - (sum & 0xFFu));
    return 6u;
}

uint8_t hamfly_qb_build_write(uint16_t attr, const uint8_t *payload, uint8_t payload_len,
                               uint8_t *out_buf, uint8_t out_max)
{
    if (HAMFLY_ATTR_IS_QX(attr) || !payload || !out_buf) return 0u;
    if (payload_len == 0u || payload_len > QX_MAX_PAYLOAD_LEN) return 0u;

    const uint16_t data_len  = (uint16_t)payload_len + 1u;   /* attr byte + payload */
    const uint16_t frame_len = data_len + 5u;                /* Q,B,lenHi,lenLo,...,chk */
    if (frame_len > (uint16_t)out_max) return 0u;

    out_buf[0] = (uint8_t)'Q';
    out_buf[1] = (uint8_t)'B';
    out_buf[2] = (uint8_t)(data_len >> 8);
    out_buf[3] = (uint8_t)(data_len & 0xFFu);
    out_buf[4] = (uint8_t)((attr & 0x7Fu) | 0x80u);          /* Rule 4: write flag bit */
    memcpy(&out_buf[5], payload, payload_len);

    uint16_t sum = out_buf[4];
    for (uint8_t i = 0u; i < payload_len; ++i) sum += payload[i];
    out_buf[5 + payload_len] = (uint8_t)(0xFFu - (sum & 0xFFu));

    return (uint8_t)frame_len;
}

/* Mirrors QX_AddExtdValToBuf()'s algorithm exactly (hamfly_qx_protocol.c,
 * static -- reimplemented here rather than exposed there, per this task's
 * "new files only" scope). Full 4-byte/28-bit range, unlike
 * hamfly_write_attr_u8()'s 2-byte/14-bit-only path (review Finding 9). */
static uint8_t qx_varint_encode(uint32_t val, uint8_t *out)
{
    uint8_t n = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        const uint8_t this_chunk = (uint8_t)((val >> (7u * i)) & 0x7Fu);
        const uint8_t next_chunk = (uint8_t)((val >> (7u * (i + 1u))) & 0x7Fu);
        if (next_chunk > 0u) {
            out[n++] = (uint8_t)(this_chunk | 0x80u);
        } else {
            out[n++] = this_chunk;
            break;
        }
    }
    return n;
}

uint8_t hamfly_qx_build_write(uint16_t attr, uint8_t gimbal_id,
                               const uint8_t *payload, uint8_t payload_len,
                               uint8_t *out_buf, uint8_t out_max)
{
    if (!HAMFLY_ATTR_IS_QX(attr) || !payload || !out_buf) return 0u;
    if (payload_len == 0u || payload_len > QX_MAX_PAYLOAD_LEN) return 0u;

    /* attrib varint(<=4) + option(1) + src(1) + tgt(1) + trid(1) + rrid(1) + payload */
    uint8_t body[QX_MAX_PAYLOAD_LEN + 9u];
    uint8_t idx = 0u;

    idx = (uint8_t)(idx + qx_varint_encode((uint32_t)attr, &body[idx]));
    body[idx++] = 0x02u;        /* option byte: Type = WRITE_ABS(2), all other bits 0 */
    body[idx++] = 0x0Au;        /* SOURCE = QX_DEV_ID_MOVI_API_CONTROLLER */
    body[idx++] = gimbal_id;    /* TARGET */
    body[idx++] = 0x00u;        /* TRID -- broadcast */
    body[idx++] = 0x00u;        /* RRID -- broadcast */
    memcpy(&body[idx], payload, payload_len);
    idx = (uint8_t)(idx + payload_len);

    /* Single-byte QX length only (F5: legally carries up to 127); no
     * settings write needs the 2-byte extended form, so refuse rather than
     * guess at it. */
    if (idx > 127u) return 0u;

    const uint16_t total = (uint16_t)idx + 4u;   /* Q,X,len,body...,chk */
    if (total > (uint16_t)out_max) return 0u;

    out_buf[0] = (uint8_t)'Q';
    out_buf[1] = (uint8_t)'X';
    out_buf[2] = idx;   /* high bit clear -- single-byte length form */
    memcpy(&out_buf[3], body, idx);

    uint16_t sum = 0u;
    for (uint8_t i = 0u; i < idx; ++i) sum += body[i];
    out_buf[3 + idx] = (uint8_t)(0xFFu - (sum & 0xFFu));

    return (uint8_t)total;
}

/* ============================================================
 * Senders
 * ============================================================ */

hamfly_result_t hamfly_settings_send_raw(hamfly_gimbal_t *g, const uint8_t *buf, uint8_t len)
{
    if (!g || !g->hal.uart_putc || !buf) return HAMFLY_ERR_UART;

    uint32_t sent = 0u;
    for (uint8_t i = 0u; i < len; ++i) {
        g->hal.uart_putc(g->hal.ctx, buf[i]);
        sent++;
    }
    g->statistics.tx_packets++;
    g->statistics.tx_bytes += sent;

    return (g->statistics.uart_err_flags != 0u) ? HAMFLY_ERR_UART : HAMFLY_OK;
}

hamfly_result_t hamfly_settings_send_qb_read(hamfly_gimbal_t *g, uint16_t attr)
{
    hamfly_settings_wire_ensure_installed();
    if (!g) return HAMFLY_ERR_UART;
    if (HAMFLY_ATTR_IS_QX(attr)) return HAMFLY_ERR_ENCODE;

    uint8_t frame[6];
    const uint8_t flen = hamfly_qb_build_read(attr, frame, (uint8_t)sizeof frame);
    if (flen == 0u) return HAMFLY_ERR_ENCODE;

    const hamfly_result_t r = hamfly_settings_send_raw(g, frame, flen);

    g->pending_attr    = attr;
    g->pending_sent_ms = 0u;   /* caller/engine tracks its own timing budget */
    g->pending_ready   = false;

    return r;
}

hamfly_result_t hamfly_settings_send_qb_write(hamfly_gimbal_t *g, uint16_t attr,
                                               const uint8_t *payload, uint8_t payload_len)
{
    hamfly_settings_wire_ensure_installed();
    if (!g) return HAMFLY_ERR_UART;

    uint8_t frame[QX_MAX_PAYLOAD_LEN + 6u];
    const uint8_t flen = hamfly_qb_build_write(attr, payload, payload_len,
                                                frame, (uint8_t)sizeof frame);
    if (flen == 0u) return HAMFLY_ERR_ENCODE;

    return hamfly_settings_send_raw(g, frame, flen);
}

hamfly_result_t hamfly_settings_send_qx_write(hamfly_gimbal_t *g, uint16_t attr,
                                               const uint8_t *payload, uint8_t payload_len)
{
    hamfly_settings_wire_ensure_installed();
    if (!g) return HAMFLY_ERR_UART;

    /* Worst case: 4-byte attrib varint + 5 header bytes + 64-byte payload +
     * ('Q','X',len,checksum) = 77. Sized with margin; no real attribute
     * today needs more than ~25. */
    uint8_t frame[QX_MAX_PAYLOAD_LEN + 16u];
    const uint8_t flen = hamfly_qx_build_write(attr, g->gimbal_id, payload, payload_len,
                                                frame, (uint8_t)sizeof frame);
    if (flen == 0u) return HAMFLY_ERR_ENCODE;

    return hamfly_settings_send_raw(g, frame, flen);
}
