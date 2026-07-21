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
 * Raw QB (attr <= 255) and QX (attr > 255) frame builders for GCU settings,
 * plus the piece of wiring that makes QB replies visible to hamfly_pump().
 *
 * WHY THIS MODULE EXISTS INSTEAD OF REUSING THE QX CLIENT PATH
 * --------------------------------------------------------------------------
 * hamfly_request_attr() / QX_SendPacket_Cli_Read() / QX_SendPacket_Cli_WriteABS()
 * (hamfly_core_gimbal.c, hamfly_qx_protocol.c) always build the QX-style
 * frame (varint attribute, option byte, source/target/trid/rrid addressing)
 * regardless of attribute id, because QX_Msg_t.Legacy_Header defaults to 0
 * in QX_InitMsg() and nothing in this codebase ever sets it to 1. The
 * "Legacy" (QB) build/parse hooks -- QX_BuildHeader_Legacy /
 * QX_ParseHeader_Legacy -- exist as extern function pointers in
 * hamfly_qx_protocol.h but are DEFINED AS NULL in hamfly_qx_protocol.c and
 * are never assigned anywhere in src/ or platform/ (confirmed by grep).
 *
 * Consequence, confirmed by hand-tracing the wire format against
 * build/re-notes/hamfly_test_vectors.md: for attr <= 255 (essentially every
 * settings attribute in hamfly_attr_table.h), hamfly_request_attr() sends
 * the WRONG frame shape -- "QX ...varint...addressing..." instead of the
 * "QB 00 01 <attr>" the real gimbal expects (see hamfly_test_vectors.md
 * sec 1: attr 83 read is 6 bytes total, no varint, no addressing at all).
 * And even if it sent the right bytes, the RX side could not have told the
 * caller: QX_RxMsg() branches on Legacy_Header and, for a QB reply
 * (Legacy_Header=1, set correctly by the byte-level state machine on 'B'),
 * calls QX_ParseHeader_Legacy -- NULL -- and returns immediately, WITHOUT
 * setting Header.Attrib or BufPayloadStart_p. hamfly_pump() then reads
 * stale/leftover values for both, so a QB reply is invisible to the
 * pending_attr/pending_payload capture mechanism no matter what was sent.
 *
 * This is a pre-existing gap in hamfly_core_gimbal.c / hamfly_qx_protocol.c
 * (files this task may not edit), not something introduced by v2 GCU
 * settings work. See the "Files not touched" note in the delivery report
 * for the exact one-line fix if the QX_SendPacket_Cli_* path is ever wired
 * up for Legacy framing directly.
 *
 * THE FIX USED HERE, WITHOUT EDITING ANY EXISTING FILE
 * --------------------------------------------------------------------------
 * TX: hand-roll the complete QB/QX frame in a local buffer and hand it to
 *     hal.uart_putc() byte by byte -- exactly the pattern
 *     hamfly_write_attr_u8() already uses for its one QX case. No need for
 *     QX_TxMsg_Setup()/QX_TxMsg_Finish() (both `static`, not reachable from
 *     outside hamfly_qx_protocol.c anyway), so no Legacy_Header flag on any
 *     QX_Msg_t needs to exist for the send direction.
 * RX: QX_BuildHeader_Legacy / QX_ParseHeader_Legacy are ordinary mutable
 *     `extern` function pointers with external linkage -- any translation
 *     unit may assign them. hamfly_settings_wire_ensure_installed() points
 *     QX_ParseHeader_Legacy at hamfly_qb_parse_header() (this file), which
 *     teaches QX_RxMsg() to set Header.Attrib/Type from a QB DATA block the
 *     same way QX_ParseHeader() does for QX frames. After that one
 *     assignment, hamfly_pump()'s EXISTING, UNMODIFIED pending_attr /
 *     pending_ready / pending_payload capture logic works correctly for QB
 *     replies too -- zero changes needed to hamfly_core_gimbal.c.
 *     QX_BuildHeader_Legacy is deliberately left NULL: nothing in this
 *     module calls QX_TxMsg_Setup(), so it is never reached.
 *
 * NOT build-verified -- no C compiler was available when this was written.
 */

#ifndef HAMFLY_SETTINGS_WIRE_H
#define HAMFLY_SETTINGS_WIRE_H

#include <stdint.h>

#include "hamfly_core_gimbal.h"
#include "hamfly_attr_table.h"

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

#endif /* HAMFLY_SETTINGS_WIRE_H */
