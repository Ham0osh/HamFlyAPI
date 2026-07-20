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
 * Gimbal control function and type definitions.
 * This is a part of the Hamfly extension of the FreeflyAPI.
 * This is the main user-facing API header for controlling the gimbal and
 * requesting telemetry.
 */

#ifndef HAMFLY_CORE_GIMBAL_H
#define HAMFLY_CORE_GIMBAL_H

#include <stdint.h>
#include <stdbool.h>

#include "hamfly_core_hal.h"        // HAL definition.
#include "hamfly_core_control.h"    // Control struct.
#include "hamfly_core_telemetry.h"  // For telemetry struct and accessors.
#include "hamfly_comm_rb.h"         // ISR-safe ring buffer for Rx and Tx.

// Return types of Hamfly API functions and their meanings.
typedef enum {
    HAMFLY_OK            = 0,
    HAMFLY_ERR_UART      = 1,  /* UART transmission failure.        */
    HAMFLY_ERR_ENCODE    = 2,  /* Packet build failure.             */
    HAMFLY_ERR_BUSY      = 3,  /* Pending request or tx busy.       */
    HAMFLY_ERR_NO_GPS    = 4,  /* GPS not locked — cannot point.    */
    HAMFLY_ERR_BAD_STATE = 5   /* Invalid state for this call.      */
} hamfly_result_t;

// Struct accumulating debug statistics about communications.
typedef struct {
    uint32_t tx_packets;
    uint32_t tx_bytes;
    uint32_t rx_packets;
    uint32_t rx_bytes;
    uint32_t rx_bad_checksum;
    uint16_t rxbuf_drops;
    uint8_t  uart_err_flags;
} hamfly_statistics_t;

// Main gimbal struct (Like the "Python Class" for the API).
// Instantiated by the user and passed to all API functions for this gimbal.
// Contains the HAL, telemetry, control, and communication state.
// Multiple gimbals can be supported by instantiating multiple hamfly_gimbal_t's
// with different HAL instances and gimbal_id's.
typedef struct {
    hamfly_hal_t        hal;
    hamfly_control_t    ctl;
    hamfly_telemetry_t  telemetry;
    hamfly_statistics_t statistics;
    hamfly_rb_t         rxbuf;  // Rx on ISR push
    hamfly_rb_t         txbuf;  // Tx on main loop push (user programmed)

    // The GCU address (Default = QX_DEV_ID_GIMBAL = 2).
    // TODO: set per-instance for multi-gimbal.
    uint8_t gimbal_id;

    // Handles pending transactions for request_attr and write_attr_u8.
    // (not included in QX277/QX283).
    uint16_t pending_attr;           // 0 if no pending request.
    uint32_t pending_sent_ms;        // Timestamp of request sent.
    bool     pending_ready;          // Flag set true if matching attr pumped.
    uint16_t pending_response_attr;  // Attr of received response for debug.
    uint16_t pending_response_len;   // Response packet MsgLen.
    uint8_t  pending_payload[64];    // Raw received bytes. Defines max capture
    //  size for request_attr_capture, which is larger than typical FreeflyAPI
    // 32 byte max for safety.
    uint8_t  pending_payload_len;    // Number of bytes in pending_payload.

} hamfly_gimbal_t;

// Create and clean up a gimbal struct.
// Call hamfly_init() once with a pointer to a hamfly_gimbal_t and a
// HAL instance to initialize the struct.
void hamfly_init            (hamfly_gimbal_t *g, const hamfly_hal_t *hal);
void hamfly_reset           (hamfly_gimbal_t *g);
void hamfly_clear_statistics(hamfly_gimbal_t *g);

// Used by ISR to push received bytes into the gimbal's ring buffer.
void hamfly_on_rx_byte        (hamfly_gimbal_t *g, uint8_t b);
void hamfly_on_uart_err_flags (hamfly_gimbal_t *g, uint8_t err_mask);

// Usefull user functions to...
//  - Pump the ring buffer and parse packets into the gimbal struct.
//  - Send control packets built by the user.
//  - Send a kill command to stop the gimbal immediately.
void            hamfly_pump         (hamfly_gimbal_t *g);
/* Returns HAMFLY_ERR_BAD_STATE if ctl->enable == 0 (v2 send-gate, see
 * hamfly_control_init), HAMFLY_ERR_ENCODE if ctl is NULL, HAMFLY_ERR_UART on
 * a UART fault. pan/tilt/roll are clamped to ±1.0 before serialisation. */
hamfly_result_t hamfly_send_control (hamfly_gimbal_t *g,
                                     const hamfly_control_t *ctl);
/* Emergency stop. Bypasses the `enable` send-gate by design. */
void            hamfly_kill         (hamfly_gimbal_t *g);

// And extensions from Hamfly to...
// - Request telemetry attributes by ID and respond into the gimbal struct.
hamfly_result_t hamfly_request_attr        (hamfly_gimbal_t *g,
                                            uint16_t attr_id);
hamfly_result_t hamfly_request_attr_capture(hamfly_gimbal_t *g,
                                            uint16_t attr_id,
                                            uint8_t *tx_buf, 
                                            uint8_t tx_buf_max, 
                                            uint8_t *tx_len_out);
hamfly_result_t hamfly_write_attr_u8       (hamfly_gimbal_t *g, 
                                            uint16_t attr_id, 
                                            uint8_t value);

/* Accessors */
void hamfly_get_telemetry  (hamfly_gimbal_t *g, hamfly_telemetry_t  *out);
void hamfly_get_statistics (hamfly_gimbal_t *g, hamfly_statistics_t *out);

/* Exact int16 pan/tilt/roll words the last hamfly_send_control() put on the
 * QX277 wire (mirrors AddFloatAsSignedShort: scale 32767, round-half-away, no
 * clamp). Reads g->ctl. NULL out-pointers are skipped. Lets a logger record the
 * transmitted word rather than re-scaling a float. */
void hamfly_get_control_wire_i16(const hamfly_gimbal_t *g,
                                 int16_t *pan, int16_t *tilt, int16_t *roll);

/* Frame-complete hook. hamfly_pump() calls this once per checksum-valid QX
 * frame, in main-loop context (NOT an ISR). The library ships a weak no-op;
 * define a strong override with this exact signature to be notified (e.g. to
 * toggle a debug pin, DPIN_MOVI_FRAME, on Movi telemetry frame arrival). */
void hamfly_on_frame_complete(hamfly_gimbal_t *g);

/* Utility helpers */

/* Zero-initialise a control struct to a safe inert state:
 * all axes DEFER, enable=0, kill=0.
 * Using enable=0 (not 1) so the app explicitly opts in before sending.
 * v2: this opt-in is ENFORCED — hamfly_send_control() rejects a struct with
 * enable=0 and returns HAMFLY_ERR_BAD_STATE. In v1 the field was documented
 * but never read. Set enable=1 when you are ready to command the gimbal. */
void hamfly_control_init(hamfly_control_t *c);

/* Returns 1 if the last commanded packet had kill=1, else 0.
 * Reads g->ctl which is updated on every hamfly_send_control(). */
uint8_t hamfly_is_killed(const hamfly_gimbal_t *g);

/* Reset gimbal heading reference and baro home point via attr 382.
 * GPS reference is unaffected (GPS is always absolute).
 * After this call baro_alt_m will read ~0 once telemetry refreshes. */
hamfly_result_t hamfly_home(hamfly_gimbal_t *g);

/* Initiate compass calibration sequence.
 * STUB — attr/command not yet confirmed from serial capture.
 * Always returns HAMFLY_ERR_BAD_STATE until implemented. */
hamfly_result_t hamfly_compass_cal_start(hamfly_gimbal_t *g);

/* Calculate pointing angles from platform to target GPS coord.
 * platform:           caller's current position (from telemetry).
 * target:             where to point.
 * heading_offset_deg: gimbal pan=0 offset from geographic North.
 *   Pass 0.0f if attr 382 zeros to North (unconfirmed).
 *   Pass compass heading at time of attr 382 call if it zeros to
 *   current heading.
 * out:     filled with azimuth, elevation, distance.
 * ctl_out: if not NULL, filled with ABSOLUTE pan/tilt control
 *          packet ready for hamfly_send_control(). roll = 0.
 * Pure math — does not query gimbal state internally.
 * Returns HAMFLY_ERR_ENCODE if geometry is degenerate (<0.1 m). */
hamfly_result_t hamfly_calc_gps_pointing(
    const hamfly_gps_coord_t *platform,
    const hamfly_gps_coord_t *target,
    float                     heading_offset_deg,
    hamfly_pointing_t        *out,
    hamfly_control_t         *ctl_out);

#endif /* HAMFLY_CORE_GIMBAL_H */
