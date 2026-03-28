/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright 2026 Hamish Johnson, Quantum Information Systems Lab, SFU Physics
 * Author: Hamish Johnson
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
 * Typical usage:
 * 
 * #include "hamfly.h"
 * 
 * static hamfly_gimbal_t g_gimbal;
 * static const hamfly_hal_t HAL = hamfly_psoc5_hal(NULL);
 * hamfly_init(&g_gimbal, &HAL);
 * 
 * // PSoC Specific initiate ISR for Rx for a UART UDB called "UART_MOVI".
 * hamfly_on_rx_byte(&g_gimbal, UART_MOVI_RXDATA_REG);
 * 
 * // Within the main loop you can pump into the gimbal struct
 * hamfly_pump(&g_gimbal);
 * // and also send a control packet that you built:
 * hamfly_send_control(&g_gimbal, &ctl);
 * 
 * // You can also optionally request other telemetry attributes:
 * hamfly_request_attr(&g_gimbal, 22u);
 * g_gimbal.pending_sent_ms = now_ms;
 * // Later when response received into the struct:
 * if (g_gimbal.pending_ready) { ... }
 * 
 */

#ifndef HAMFLY_H
#define HAMFLY_H

#include "hamfly_core_hal.h"
#include "hamfly_core_control.h"
#include "hamfly_core_telemetry.h"
#include "hamfly_core_gimbal.h"

/* ============================================================
 * Status flag bit positions in sysstat_status_flags
 * (attr 1 off 5-6). Confirmed by cross-referencing:
 *   indoor  0x0065 — no GPS lock
 *   outdoor 0x006C — GPS locked
 * bits 5,6 always set in normal operation (boot OK + running).
 * ============================================================ */
#define HAMFLY_FLAG_COMPASS_ERROR  (1u << 0)
#define HAMFLY_FLAG_GPS_LOS        (1u << 1)
#define HAMFLY_FLAG_RADIO_LOS      (1u << 2)
#define HAMFLY_FLAG_GPS_LOCKED     (1u << 3)

/* GPS quality thresholds for safe absolute pointing. */
#define HAMFLY_GPS_MIN_HACC_M      5.0f  /* max HACC in metres  */
#define HAMFLY_GPS_MIN_SATS        6u    /* min satellite count */

/* ============================================================
 * Attribute IDs
 * ============================================================ */
#define HAMFLY_ATTR_SYSSTAT        1u
#define HAMFLY_ATTR_PLATFORM_ATT   2u
#define HAMFLY_ATTR_BARO           3u
#define HAMFLY_ATTR_GPS            4u
#define HAMFLY_ATTR_MAG            12u
#define HAMFLY_ATTR_EULER_ATT      22u
#define HAMFLY_ATTR_SYSTEM_ECHO    48u
#define HAMFLY_ATTR_QX277_CONTROL  277u
#define HAMFLY_ATTR_QX287_STATUS   287u
#define HAMFLY_ATTR_HEADING_RESET  382u

#endif /* HAMFLY_H */
