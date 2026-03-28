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
hamfly_result_t hamfly_send_control (hamfly_gimbal_t *g,
                                     const hamfly_control_t *ctl);
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

#endif /* HAMFLY_CORE_GIMBAL_H */
