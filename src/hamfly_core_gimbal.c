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
 * Gimbal control functions.
 */


#include "hamfly_core_gimbal.h"
#include "hamfly_core_telemetry.h"
#include "hamfly_qx_protocol.h"
#include "hamfly_qx_app.h"
#include <string.h>

#define HAMFLY_QX_PORT  (QX_COMMS_PORT_UART)

// ============================================================================
// Internal helpers: load/save between hamfly_control_t and FreeflyAPI.control
// ============================================================================

// Copy the control struct into the existing FreeflyAPI.control struct for Tx.
static void load_freefly_control(const hamfly_control_t *ctl)
{
    FreeflyAPI.control.pan.value   = ctl->pan;
    FreeflyAPI.control.tilt.value  = ctl->tilt;
    FreeflyAPI.control.roll.value  = ctl->roll;
    FreeflyAPI.control.pan.type    = (ff_api_control_type_e)ctl->pan_mode;
    FreeflyAPI.control.tilt.type   = (ff_api_control_type_e)ctl->tilt_mode;
    FreeflyAPI.control.roll.type   = (ff_api_control_type_e)ctl->roll_mode;
    FreeflyAPI.control.gimbal_kill = ctl->kill;
}

// Copy the QX287 telemetry from FreeflyAPI.status into hamfly_telemetry_t.
static void copy_qx287_to_telemetry(hamfly_telemetry_t *tel)
{
    // 287 is already parsed into FreeflyAPI.status by the QX callback.
    // hamfly_decode_qx287 reads it into hamfly_telemetry_t.
    // TODO: Investigate removing this wrapper and use decode directly.
    hamfly_decode_qx287(NULL, 0u, tel);
}

// Drain the txbuf and send bytes into the HAL.
static uint32_t drain_txbuf(hamfly_gimbal_t *g)
{
    uint8_t  byte;
    uint32_t sent = 0u;
    while (hamfly_txbuf_remove(&g->txbuf, &byte)) {
        g->hal.uart_putc(g->hal.ctx, byte);
        sent++;
    }
    return sent;
}

// ============================================================================
// Lifecycle helpers
// ============================================================================

// Initialize gimbal struct with HAL, empty buffers and stats.
void hamfly_init(hamfly_gimbal_t *g, const hamfly_hal_t *hal)
{
    memset(g, 0, sizeof(*g));
    g->hal       = *hal;
    g->gimbal_id = QX_DEV_ID_GIMBAL;
    hamfly_rb_init(&g->rxbuf);
    hamfly_rb_init(&g->txbuf);

    // Register as active instance for QX callbacks, if user wants to implement
    // multiple gimbals in the future make this a list or map instead of a
    // single pointer.
    qx_active_gimbal_ptr = (void *)g;

    // Start the FreeflyAPI 
    // Registers the QX callback that updates FreeflyAPI.status
    // and also initializes FreeflyAPI.control to zeros/DEFER.
    FreeflyAPI.begin();
}

// Reset gimbal struct to empty buffers and stats, but keep HAL and gimbal_id.
void hamfly_reset(hamfly_gimbal_t *g)
{
    if (!g) return;
    hamfly_rb_clear(&g->rxbuf);
    hamfly_rb_clear(&g->txbuf);
    memset(&g->telemetry,  0, sizeof(g->telemetry));
    memset(&g->statistics, 0, sizeof(g->statistics));
    g->pending_attr          = 0u;
    g->pending_sent_ms       = 0u;
    g->pending_ready         = false;
    g->pending_response_attr = 0u;
    g->pending_response_len  = 0u;
    g->pending_payload_len   = 0u;
}

// Clear just the statistics. Used so far for clearing test modes.
void hamfly_clear_statistics(hamfly_gimbal_t *g)
{
    if (!g) return;
    memset(&g->statistics, 0, sizeof(g->statistics));
}

// ============================================================================
// ISR Helpers
// ============================================================================

// ISR calls to push Rx bytes into the ring buffer.
void hamfly_on_rx_byte(hamfly_gimbal_t *g, uint8_t b)
{
    if (!g) return;
    (void)hamfly_rb_push(&g->rxbuf, b);
    g->statistics.rb_drops = g->rxbuf.drops;
}

// ISR calls to set UART error flags.
void hamfly_on_uart_err_flags(hamfly_gimbal_t *g, uint8_t err_mask)
{
    if (!g) return;
    g->statistics.uart_err_flags |= err_mask;
}

// ============================================================================
// Hamfly Pump: parse incoming bytes into telemetry and pending attr responses.
// ============================================================================
void hamfly_pump(hamfly_gimbal_t *g)
{
    if (!g) return;

    uint8_t  b;
    // Snapshot in number of checksum failures so we can keep a running count.
    uint32_t chkfail_before = QX_CommsPorts[HAMFLY_QX_PORT].ChkSumFail_cnt;

    // While there are bytes in rxbuf, pop into address &b
    while (hamfly_rb_pop(&g->rxbuf, &b)) {
        // Feed byte b into QX state machine until packet complete.
        g->statistics.rx_bytes++;
        int got_packet = QX_StreamRxCharSM(HAMFLY_QX_PORT, (unsigned char)b);
        
        if (!got_packet) continue;  // Stops here unless complete packet.
        
        g->statistics.rx_packets++;  // Count a full packet.

        // Grab the packet data out of the QX state machine for parsing.
        QX_CommsPort_t *port = &QX_CommsPorts[HAMFLY_QX_PORT];
        uint16_t rxattr = (uint16_t)port->RxMsg.Header.Attrib;
        uint16_t rxlen  = (uint16_t)port->RxMsg.Header.MsgLength;
        const uint8_t *pay = port->RxMsg.BufPayloadStart_p;

        // QX callback automatically decodes any QX287 telemetry in case.
        // Take a moment to sync the 287 fields in case anything updated.
        copy_qx287_to_telemetry(&g->telemetry);

        // Switch case for telemetry attributes I have decoded.
        // Each attr needs an appropriate decode function to read the raw bytes
        // into hamfly_telemetry_t.
        // TODO: Implement doc of current progress sniffing and decoding telem.
        if (pay) {
            switch (rxattr) {
                case 4u:  hamfly_decode_gps      (pay, rxlen, &g->telemetry);
                break;
                case 3u:  hamfly_decode_baro     (pay, rxlen, &g->telemetry);
                break;
                case 22u: hamfly_decode_attitude (pay, rxlen, &g->telemetry);
                break;
                case 1u:  hamfly_decode_sysstat  (pay, rxlen, &g->telemetry);
                break;
                case 12u: hamfly_decode_mag      (pay, rxlen, &g->telemetry);
                break;
                default: 
                break;
            }
        }

        // IF there is a pending attr AND it matches this packet's attr
        // THEN capture this payload into the pending slot and mark ready.
        if (g->pending_attr != 0u && rxattr == g->pending_attr) {
            // Populate pending response fields for debug.
            g->pending_response_attr = rxattr;
            g->pending_response_len  = rxlen;
            g->pending_ready         = true;

            // Defensive programming, may be overkill.
            // We assume buffers are max 32 bytes according to the FreeflyAPI
            // docs, I have given 64 bytes in case, but if we somehow go beyond
            // it clips the length.
            uint8_t snap_len = (rxlen < (uint16_t)sizeof(g->pending_payload))
                               ? (uint8_t)rxlen
                               : (uint8_t)sizeof(g->pending_payload);
            // Check the payload is not null to avoid dereferencing a null 
            // pointer if valid copies max snap_len bytes into the 
            // pending_payload.
            if (pay) memcpy(g->pending_payload, pay, snap_len);
            else     memset(g->pending_payload, 0,   snap_len);
            g->pending_payload_len = snap_len;
        }
    }

    // Count number of checksum fails that have accumulated.
    uint32_t chkfail_after = QX_CommsPorts[HAMFLY_QX_PORT].ChkSumFail_cnt;
    if (chkfail_after > chkfail_before)
        g->statistics.rx_bad_checksum += (chkfail_after - chkfail_before);
    // Count any drops from the ring buffer as well.
    g->statistics.rb_drops = g->rxbuf.drops;
}

// ============================================================================
// Hamfly Send Control: FreeFlyAPI QX277 send wrapper.
// ============================================================================
hamfly_result_t hamfly_send_control(hamfly_gimbal_t *g,
                                     const hamfly_control_t *ctl)
{
    if (!g || !g->hal.uart_putc) return HAMFLY_ERR_UART;
    // Load control frame from gimbal struct
    load_freefly_control(ctl);
    g->ctl = *ctl;
    FreeflyAPI.send();

    uint32_t sent = drain_rb(g);
    g->statistics.tx_packets++;
    g->statistics.tx_bytes += sent;

    return (g->statistics.uart_err_flags != 0u) ? HAMFLY_ERR_UART : HAMFLY_OK;
}

// ============================================================================
// Hamfly Kill: Safely kill the gimbal.
// ============================================================================
void hamfly_kill(hamfly_gimbal_t *g)
{
    if (!g) return;
    // Enable kill flag and send.
    hamfly_control_t k = g->ctl;
    k.kill = 1u;
    (void)hamfly_send_control(g, &k);
}

// ============================================================================
// Hamfly Request Attribute: Request read of an attr ID.
// ============================================================================
hamfly_result_t hamfly_request_attr(hamfly_gimbal_t *g, uint16_t attr_id)
{
    // This function uses UART (nut just memory manipulation) so needs check.
    if (!g || !g->hal.uart_putc) return HAMFLY_ERR_UART;

    // Load safe defaults into QX packet.
    QX_TxMsgOptions_t opts;
    QX_InitTxOptions(&opts);
    opts.Target_Addr = (QX_DevId_e)g->gimbal_id;  // Set device ID.
    // Send the packet into the QX state machine to build the packet which
    // gets pushed into the txbuf by the QX callback.
    QX_SendPacket_Cli_Read(&QX_Clients[0], (uint32_t)attr_id,
                            HAMFLY_QX_PORT, opts);
    
    // Drain the txbuf into the HAL and count bytes sent.
    uint32_t sent = drain_rb(g);

    // Set pending atts for bookkeeping and wait for response.
    g->pending_attr         = attr_id;
    g->pending_sent_ms      = 0u;   /* caller sets this from their tick */
    g->pending_ready        = false;

    // Update stats.
    g->statistics.tx_packets++;
    g->statistics.tx_bytes += sent;

    // Return UART error if any occurred during this process.
    return (g->statistics.uart_err_flags != 0u) ? HAMFLY_ERR_UART : HAMFLY_OK;
}

// ============================================================================
// Hamfly Capture Attribute Response: Request read of an attr ID + Raw bytes.
// Helper for testing or logging exact bytes.
// This is otherwise the same as hamfly_request_attr, for normal use keep with
// hamfly_request_attr and use the parsed telemetry in hamfly_telemetry_t.
// ============================================================================
hamfly_result_t hamfly_request_attr_capture(hamfly_gimbal_t *g,
                                              uint16_t attr_id,
                                              uint8_t *user_txbuf,
                                              uint8_t  user_txbuf_max,
                                              uint8_t *tx_len_out)
{
    // This function uses UART (nut just memory manipulation) so needs check.
    if (!g || !g->hal.uart_putc) return HAMFLY_ERR_UART;

    // Load safe defaults into QX packet.
    QX_TxMsgOptions_t opts;
    QX_InitTxOptions(&opts);
    opts.Target_Addr = (QX_DevId_e)g->gimbal_id;  // Set device ID.
    // Send the packet into the QX state machine to build the packet which
    // gets pushed into the txbuf by the QX callback.
    QX_SendPacket_Cli_Read(&QX_Clients[0], (uint32_t)attr_id,
                            HAMFLY_QX_PORT, opts);

    uint8_t  byte;
    uint32_t sent = 0u;
    uint8_t  cap  = 0u;
    // Drain the txbuf one by one and parse into user_txbuf.
    while (hamfly_rb_remove(&g->txbuf, &byte)) {
        g->hal.uart_putc(g->hal.ctx, byte);
        if (user_txbuf && cap < user_txbuf_max) user_txbuf[cap++] = byte;
        sent++;
    }
    if (tx_len_out) *tx_len_out = cap;

    // Set pending atts for bookkeeping and wait for response.
    g->pending_attr    = attr_id;
    g->pending_sent_ms = 0u;
    g->pending_ready   = false;

    // Update stats.
    g->statistics.tx_packets++;
    g->statistics.tx_bytes += sent;

    // Return UART error if any occurred during this process.
    return (g->statistics.uart_err_flags != 0u) ? HAMFLY_ERR_UART : HAMFLY_OK;
}

// ============================================================================
// Hamfly Write Attribute U8: Request write of an attr ID with a single byte.
// Handles single and two-byte varints for the attr ID by bypassing the QX
// state machine and building the packet directly into the txbuf.
// This is an extension, not part of the FreeflyAPI.
//
// Example usecases:
//  - Request write of 0x01 to 382 to reset the heading.
//  - Request write of a two-byte varint attr like 0x80 (set gimbal ID).
//  
// Issues:
//  - Many of these are unconfirmed and only exist due to BLE sniffing.
//  - Some appear to need more bytes, so moving to an array of bytes is needed. 
// ============================================================================
hamfly_result_t hamfly_write_attr_u8(hamfly_gimbal_t *g,
                                      uint16_t attr_id,
                                      uint8_t  value)
{
    if (!g || !g->hal.uart_putc) return HAMFLY_ERR_UART;

    uint8_t pkt[13];
    uint8_t idx = 0u, i;

    // Begin packet.
    pkt[idx++] = 0x51u;  // Q
    pkt[idx++] = 0x58u;  // X

    // Set attr ID as varint.
    if (attr_id < 128u) {
        pkt[idx++] = 0x07u;
        pkt[idx++] = (uint8_t)attr_id;
    } else {
        pkt[idx++] = 0x08u;
        pkt[idx++] = (uint8_t)((attr_id & 0x7Fu) | 0x80u);
        pkt[idx++] = (uint8_t)(attr_id >> 7u);
    }
    pkt[idx++] = 0x02u;  // Options byte -> write with no response
    pkt[idx++] = 0x0Au;  // SOURCE -> QX_DEV_ID_MOVI_API_CONTROLLER
    pkt[idx++] = (uint8_t)g->gimbal_id;
    pkt[idx++] = 0x00u;  // Transmit request ID (TRID)
    pkt[idx++] = 0x00u;  // Receive request ID (RRID)
    pkt[idx++] = value;  // Byte you want to write

    // Append the checksum byte;
    // The 8-bit inverted sum of all bytes after the header.
    uint16_t sum = 0u;
    for (i = 3u; i < idx; i++) sum += pkt[i];
    pkt[idx++] = (uint8_t)(255u - (sum & 0xFFu));

    // Send packet to the UART.
    uint32_t sent = 0u;
    for (i = 0u; i < idx; i++) {
        g->hal.uart_putc(g->hal.ctx, pkt[i]);
        sent++;
    }

    // Update stats.
    g->statistics.tx_packets++;
    g->statistics.tx_bytes += sent;

    // Return UART error if any occurred during this process.
    return (g->statistics.uart_err_flags != 0u) ? HAMFLY_ERR_UART : HAMFLY_OK;
}

// ============================================================================
// Accessors for telemetry and statistics.
// ============================================================================

// Get a copy of the telemetry struct.
void hamfly_get_telemetry(hamfly_gimbal_t *g, hamfly_telemetry_t *out)
{
    if (!g || !out) return;
    *out = g->telemetry;
}

// Get a copy of the statistics struct with the latest ring buffer drops.
void hamfly_get_statistics(hamfly_gimbal_t *g, hamfly_statistics_t *out)
{
    if (!g || !out) return;
    g->statistics.rb_drops = g->rxbuf.drops;
    *out = g->statistics;
}
