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
#include "hamfly.h"
#include <string.h>
#include <math.h>

#define HAMFLY_QX_PORT  (QX_COMMS_PORT_UART)

// ============================================================================
// Internal helpers: load/save between hamfly_control_t and FreeflyAPI.control
// ============================================================================

// Clamp a normalized control input to the ±1.0 domain the QX wire format
// expects. AddFloatAsSignedShort() applies no clamp of its own, so |v| > 1.0
// overflows the int16 and wraps: 1.5f becomes -16386, i.e. near-full-scale in
// the OPPOSITE direction. Saturating turns a dangerous reversal into a benign
// full-scale command. Enforced here in the wrapper layer so the QX protocol
// core stays byte-faithful to the Freefly reference.
static float hamfly_clamp_unit(float v)
{
    if (v >  1.0f) return  1.0f;
    if (v < -1.0f) return -1.0f;
    return v;
}

// Convert a normalized control float to the exact int16 word placed on the QX
// wire. Mirrors AddFloatAsSignedShort() in hamfly_qx_protocol.c (scale 32767,
// round-half-away-from-zero, plain cast) and applies the same ±1.0 clamp that
// load_freefly_control() applies on the send path, so the value returned here
// is the number the gimbal actually receives, not a re-scale.
static int16_t hamfly_float_to_wire_ss(float v)
{
    v = hamfly_clamp_unit(v);
    return (int16_t)((v * 32767.0f) + 0.5f * ((0.0f < v) - (v < 0.0f)));
}

// Copy the control struct into the existing FreeflyAPI.control struct for Tx.
static void load_freefly_control(const hamfly_control_t *ctl)
{
    FreeflyAPI.control.pan.value   = hamfly_clamp_unit(ctl->pan);
    FreeflyAPI.control.tilt.value  = hamfly_clamp_unit(ctl->tilt);
    FreeflyAPI.control.roll.value  = hamfly_clamp_unit(ctl->roll);
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
static uint32_t drain_rb(hamfly_gimbal_t *g)
{
    uint8_t  byte;
    uint32_t sent = 0u;
    while (hamfly_rb_pop(&g->txbuf, &byte)) {
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
    g->statistics.rxbuf_drops = g->rxbuf.drops;
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

        // Frame-complete hook: fires once per checksum-valid QX frame, in main-
        // loop (pump) context — NOT an ISR. Default is a weak no-op; the MCU
        // integration overrides it (e.g. toggle DPIN_MOVI_FRAME on a scope).
        hamfly_on_frame_complete(g);

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
                case 48u: hamfly_decode_sysecho  (pay, rxlen, &g->telemetry);
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
    g->statistics.rxbuf_drops = g->rxbuf.drops;
}

// ============================================================================
// Hamfly Send Control: FreeFlyAPI QX277 send wrapper.
// ============================================================================
hamfly_result_t hamfly_send_control(hamfly_gimbal_t *g,
                                     const hamfly_control_t *ctl)
{
    if (!g || !g->hal.uart_putc) return HAMFLY_ERR_UART;
    if (!ctl) return HAMFLY_ERR_ENCODE;

    /* v2 BREAKING CHANGE: `enable` is now honoured as a send-gate. In v1 the
     * field was documented as an explicit opt-in but never read, so a caller
     * that left enable=0 still commanded the gimbal. Integrators migrating
     * from v1 must set enable=1. See README "Migrating from v1". */
    if (!ctl->enable) return HAMFLY_ERR_BAD_STATE;

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
hamfly_result_t hamfly_kill(hamfly_gimbal_t *g)
{
    if (!g) return HAMFLY_ERR_UART;
    // Enable kill flag and send.
    hamfly_control_t k = g->ctl;
    k.kill = 1u;
    /* Kill is an explicit operator safety action, so it must bypass the
     * `enable` send-gate: g->ctl carries the LAST sent control, which on a
     * fresh gimbal (or an app that never opted in) has enable=0 and would
     * otherwise cause the stop command to be silently dropped. The gate
     * exists to prevent accidental motion, never to block a stop. */
    k.enable = 1u;
    /* v2: status is returned, not discarded. A stop that failed to reach the
     * gimbal must be visible to the caller. */
    return hamfly_send_control(g, &k);
}

// ============================================================================
// Shared request-slot arbitration. See hamfly_core_gimbal.h for the contract.
// There is one slot; a second request in flight overwrites the first and one
// of the two replies is silently discarded.
// ============================================================================
bool hamfly_request_busy(const hamfly_gimbal_t *g)
{
    /* Claimed from the moment a request is sent until the consumer releases
     * it. Deliberately NOT `pending_attr && !pending_ready`: that would report
     * "free" in the window between a reply landing and the owner consuming it,
     * which is exactly when a competing poll would overwrite the payload. */
    return (g != NULL) && (g->pending_attr != 0u);
}

void hamfly_request_release(hamfly_gimbal_t *g)
{
    if (!g) return;
    g->pending_attr        = 0u;
    g->pending_ready       = false;
    g->pending_payload_len = 0u;
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
    while (hamfly_rb_pop(&g->txbuf, &byte)) {
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

    /* The two-byte varint path below emits exactly two bytes and a fixed body
     * length of 0x08, so it is only correct for IDs that fit in 14 bits. An
     * attr_id > 0x3FFF needs three varint bytes; (uint8_t)(attr_id >> 7) would
     * drop the high bits and the declared length would be wrong. */
    if (attr_id > 0x3FFFu) return HAMFLY_ERR_ENCODE;

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
    g->statistics.rxbuf_drops = g->rxbuf.drops;
    *out = g->statistics;
}

// Expose the exact int16 pan/tilt/roll words the LAST hamfly_send_control()
// placed on the QX277 wire. Lets a logger record the transmitted word instead of
// re-scaling a float and hoping the rounding matches. Reads g->ctl (updated on
// every send); any NULL out-pointer is skipped. Per-axis, so wire byte order is
// irrelevant. See docs/2026-07-08_encoding_timing_report.md (encoding fixes).
void hamfly_get_control_wire_i16(const hamfly_gimbal_t *g,
                                 int16_t *pan, int16_t *tilt, int16_t *roll)
{
    if (!g) return;
    if (pan)  *pan  = hamfly_float_to_wire_ss(g->ctl.pan);
    if (tilt) *tilt = hamfly_float_to_wire_ss(g->ctl.tilt);
    if (roll) *roll = hamfly_float_to_wire_ss(g->ctl.roll);
}

// Weak default frame-complete hook. Called once per checksum-valid QX frame from
// hamfly_pump(). No-op unless the application provides a strong override of the
// same signature (the linker prefers the strong symbol). Intended for the MCU to
// mark Movi frame arrival on a debug pin (DPIN_MOVI_FRAME).
#if defined(__GNUC__)
__attribute__((weak))
#endif
void hamfly_on_frame_complete(hamfly_gimbal_t *g)
{
    (void)g;
}

// ============================================================================
// Hamfly Control Init: safe inert default for a new control struct.
// ============================================================================
void hamfly_control_init(hamfly_control_t *c)
{
    if (!c) return;
    memset(c, 0, sizeof(*c));
    /* All axes DEFER, enable=0 (inert until app opts in), kill=0. */
    c->pan_mode  = HAMFLY_DEFER;
    c->tilt_mode = HAMFLY_DEFER;
    c->roll_mode = HAMFLY_DEFER;
}

// ============================================================================
// Hamfly Is Killed: report last commanded kill state.
// ============================================================================
uint8_t hamfly_is_killed(const hamfly_gimbal_t *g)
{
    if (!g) return 0u;
    return g->ctl.kill ? 1u : 0u;
}

// ============================================================================
// Hamfly Home: Reset heading reference and baro home point.
// ============================================================================
hamfly_result_t hamfly_home(hamfly_gimbal_t *g)
{
    /* Attr 382 resets both:
     *  - compass heading reference (North zeroing — unconfirmed)
     *  - baro home reference (baro_alt_m reads ~0 after reset)
     * GPS reference is unaffected (GPS is always absolute). */
    return hamfly_write_attr_u8(g, HAMFLY_ATTR_HEADING_RESET, 0x01u);
}

// ============================================================================
// Hamfly Compass Cal Start: Stub — attr not yet confirmed.
// ============================================================================
hamfly_result_t hamfly_compass_cal_start(hamfly_gimbal_t *g)
{
    /* TODO: compass calibration attr/command not yet confirmed.
     * Steps to determine:
     *  1. Initiate calibration from iOS app while sniffing serial.
     *  2. Identify the attr write that triggers the sequence.
     *  3. Confirm with flag_compass_error clearing in sysstat.
     * Implement once attr is confirmed. */
    (void)g;
    return HAMFLY_ERR_BAD_STATE;
}

// ============================================================================
// Hamfly Calc GPS Pointing: Flat-earth ENU pointing from platform to target.
// ============================================================================
// Uses double-precision intermediates for lat/lon conversion.
// Flat-earth approximation valid for distances < ~10 km.
// ============================================================================
hamfly_result_t hamfly_calc_gps_pointing(
    const hamfly_gps_coord_t *platform,
    const hamfly_gps_coord_t *target,
    float                     heading_offset_deg,
    hamfly_pointing_t        *out,
    hamfly_control_t         *ctl_out)
{
    if (!platform || !target || !out) return HAMFLY_ERR_ENCODE;

    /* Convert int32 lat/lon to radians (double precision).
     * 1e-7 deg/count * pi/180 = pi/1.8e9                    */
    static const double DEG2RAD = 3.14159265358979323846 / 180.0;
    static const double R_EARTH = 6371000.0;  /* metres */

    double lat0_rad = (double)platform->lat_e7 * 1e-7 * DEG2RAD;
    double dLat_deg = (double)(target->lat_e7 - platform->lat_e7) * 1e-7;
    double dLon_deg = (double)(target->lon_e7 - platform->lon_e7) * 1e-7;

    /* ENU components (metres). */
    double E   = R_EARTH * dLon_deg * DEG2RAD * cos(lat0_rad);
    double N   = R_EARTH * dLat_deg * DEG2RAD;
    double U   = (double)(target->alt_baro_m - platform->alt_baro_m);

    /* Horizontal distance. */
    double d2d = sqrt(E * E + N * N);

    /* Degenerate: platform and target are within 0.1 m horizontally. */
    if (d2d < 0.1) return HAMFLY_ERR_ENCODE;

    /* Azimuth: atan2(E, N) gives bearing clockwise from North. */
    double az_rad = atan2(E, N);
    double az_deg = az_rad / DEG2RAD;
    if (az_deg < 0.0) az_deg += 360.0;

    /* Elevation: angle above horizon. */
    double el_deg = atan2(U, d2d) / DEG2RAD;

    /* Populate output. */
    out->azimuth_deg   = (float)az_deg;
    out->elevation_deg = (float)el_deg;
    out->distance_m    = (float)d2d;

    /* Optionally fill a control packet ready for hamfly_send_control(). */
    if (ctl_out) {
        double pan_cmd = az_deg - (double)heading_offset_deg;
        /* Normalise pan to [0, 360). */
        while (pan_cmd <    0.0) pan_cmd += 360.0;
        while (pan_cmd >= 360.0) pan_cmd -= 360.0;

        ctl_out->pan       = (float)pan_cmd;
        ctl_out->tilt      = (float)el_deg;
        ctl_out->roll      = 0.0f;
        ctl_out->pan_mode  = HAMFLY_ABSOLUTE;
        ctl_out->tilt_mode = HAMFLY_ABSOLUTE;
        ctl_out->roll_mode = HAMFLY_DEFER;
        ctl_out->enable    = 1u;
        ctl_out->kill      = 0u;
    }

    return HAMFLY_OK;
}