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
 * Gimbal control types and send function declarations.
 * This is a part of the Hamfly extension of the FreeflyAPI.
 * Control type enum matches FreeflyAPI ff_api_control_type_e in qx_app.h.
 */

#ifndef HAMFLY_CORE_CONTROL_H
#define HAMFLY_CORE_CONTROL_H

#include <stdint.h>

// Must match ff_api_control_type_e in qx_app.h for posterity.
typedef enum {
    HAMFLY_DEFER             = 0,
    HAMFLY_RATE              = 1,
    HAMFLY_ABSOLUTE          = 2,
    /* Absolute with MōVI Pro window/smoothing (Majestic Mode).
     * Window and smoothing parameters are configured in the MōVI Pro app;
     * they are not carried in this packet. On the wire this is the 2-bit
     * per-axis control-type field taking value 0b11.
     * Use this instead of HAMFLY_ABSOLUTE when the active stabilization
     * loop hunts around a hard setpoint — it trades pointing stiffness
     * for steady-state smoothness. */
    HAMFLY_ABSOLUTE_MAJESTIC = 3
} hamfly_control_mode_t;

/* Full-scale angular rate, in mrad/s, that a normalized RATE command of 1.0
 * (the wire max, int16 +32767) commands the gimbal to slew at. Needed to turn a
 * logged normalized/int16 RATE word into a physical rate for the control report.
 *
 * UNMEASURED — left at 0 as an explicit "undefined" sentinel. Do NOT invent a
 * value: an assumed scale would silently mis-calibrate every logged rate.
 * TODO: measure via experiment T1 in docs/2026-07-08_encoding_timing_report.md
 *       (command a known normalized rate, time/scope the gimbal sweep, back out
 *       mrad/s), then replace the 0 here with the measured constant. */
#define HAMFLY_RATE_FULLSCALE_MRAD_S 0  /* 0 == undefined (see TODO / T1) */

// Control packet struct. Gets built by the user and sent to the gimbal on
// hamfly_send_control() in hamfly_core_control.c.
typedef struct {
    float                 pan;
    float                 tilt;
    float                 roll;
    hamfly_control_mode_t pan_mode;
    hamfly_control_mode_t tilt_mode;
    hamfly_control_mode_t roll_mode;
    uint8_t               enable;
    uint8_t               kill;
} hamfly_control_t;

/* GPS coordinate for pointing calculations.
 * Uses int32 lat/lon (not float) to preserve ~1m precision.
 * alt_baro_m is baro-relative (from baro_alt_m, NOT gps_raw_alt). */
typedef struct {
    int32_t lat_e7;     /* latitude  * 1e7 (degrees) */
    int32_t lon_e7;     /* longitude * 1e7 (degrees) */
    float   alt_baro_m; /* baro-relative altitude in metres */
} hamfly_gps_coord_t;

/* Result of a pointing calculation. */
typedef struct {
    float azimuth_deg;   /* bearing to target: 0=N 90=E 180=S 270=W */
    float elevation_deg; /* angle above horizon; negative = below    */
    float distance_m;    /* horizontal (2D) ground distance          */
} hamfly_pointing_t;

#endif /* HAMFLY_CORE_CONTROL_H */
