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
 * Decoded telemetry from all confirmed attrs.
 * Decode functions are public, can be called directly on a raw payload buffer 
 * for testing without hardware.
 */

#ifndef HAMFLY_CORE_TELEMETRY_H
#define HAMFLY_CORE_TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>

// Struct to hold latest telemetry values.
typedef struct {
    // Attr 287: QX status push (arrives with every QX277)
    bool    valid;
    float   battery_left_v;
    float   battery_right_v;
    float   gimbal_r;               /* quaternion RIJK */
    float   gimbal_i;
    float   gimbal_j;
    float   gimbal_k;
    uint8_t gimbal_status1;
    uint8_t gimbal_status2;

    // Attr 4: GPS
    bool     gps_valid;
    /* Populated from sysstat flag_gps_locked, not from attr 4. */
    bool     gps_locked;
    /* Float fields for display and logging only.
     * Use int32 gps_raw_* fields for any pointing math. */
    float    gps_lat_deg;
    float    gps_lon_deg;
    float    gps_alt_m;
    float    gps_ground_spd_ms;
    /* valid only when moving >0.5 m/s */
    float    gps_heading_deg;
    float    gps_hacc_m;
    float    gps_vacc_m;
    float    gps_sacc_ms;
    /* Primary int32 fields — use these for pointing math. */
    int32_t  gps_raw_lat;        /* /1e7 deg */
    int32_t  gps_raw_lon;        /* /1e7 deg */
    /* WGS-84 ellipsoid height in mm. NOT mean sea level.
     * NOT reliable on cold start — GPS may take minutes to
     * converge. Do not use for absolute altitude in pointing.
     * Use baro_alt_m for relative altitude instead. */
    int32_t  gps_raw_alt;
    int16_t  gps_raw_spd;        /* /100 m/s */
    /* /100 deg; valid only when moving >0.5 m/s */
    int16_t  gps_raw_hdg;
    uint16_t gps_raw_hacc;       /* /100 m */
    uint16_t gps_raw_vacc;       /* /100 m */
    uint16_t gps_raw_sacc;       /* /100 m/s */

    // Attr 3: Barometric altitude
    bool    baro_valid;
    /* Altitude relative to home point at last attr 382 reset.
     * NOT absolute altitude. Zero at boot or after reset.
     * Positive = above home, negative = below home. */
    float   baro_alt_m;
    /* Scale has been changed /100 -> /1000 based on confirmed data
     * (-33 m/s at rest was implausible; -3.3 m/s is plausible).
     * Awaiting lift test to fully verify. */
    float   baro_roc_ms;

    // Attr 22: Gimbal Euler (LE, must be requested).
    // off 0: state byte (0x00 or 0xFF) — not decoded.
    // Angles confirmed vs iOS (pitch ±0.11°, yaw ±1.23° drift).
    bool     att_valid;
    uint32_t att_timestamp_ms;
    float    att_pitch_deg;       // off  1 /100 — confirmed
    float    att_yaw_deg;         // off  3 /100 — confirmed
    float    att_roll_deg;        // off  5 /100 — confirmed
    /* Rates updated to off 9-14 /100 dps per confirmed iOS layout.
     * Previous decode was off 13-18 at /10.
     * Rate values pending verification against iOS Charts screen. */
    float    att_pitch_rate_dps;  // off  9 /100 dps
    float    att_yaw_rate_dps;    // off 11 /100 dps
    float    att_roll_rate_dps;   // off 13 /100 dps
    /* Off 7-8 purpose is unconfirmed. Previously labelled
     * roll_err /10, but confirmed layout puts rates at off 9,
     * conflicting. Retained for capture. pitch/yaw err zeroed. */
    float    att_roll_err_deg;    // off 7 /10 — unconfirmed
    float    att_pitch_err_deg;   // unconfirmed — zeroed in decode
    float    att_yaw_err_deg;     // unconfirmed — zeroed in decode

    // Attr 1: System status
    bool    sysstat_valid;
    /* Decoded status flags — see HAMFLY_FLAG_* in hamfly.h.
     * Confirmed: indoor 0x0065 (no GPS), outdoor 0x006C (locked).
     * bits 5,6 always set in normal operation. */
    bool    flag_compass_error;   /* bit 0 — compass not calibrated */
    bool    flag_gps_los;         /* bit 1 — no GPS signal          */
    bool    flag_radio_los;       /* bit 2 — no radio connected     */
    bool    flag_gps_locked;      /* bit 3 — GPS has valid fix      */
    float   sysstat_batt_v;        // off 0  int16BE /100 V 
    uint8_t sysstat_gps_sats;      // off 2  uint8
    float   sysstat_temp_c;        // off 3  uint8   /3   C  
    uint8_t sysstat_cpu_pct;       // off 4  uint8        %  
    int16_t sysstat_status_flags;  // off 5  int16BE 
    int16_t sysstat_time_s;        // off 7  int16BE      s 
    float   sysstat_pressure_mb;   // off 9  int16BE /10  mb 
    int16_t sysstat_imu_rate;      // off 11 int16BE
    int16_t sysstat_cam_status;    // off 13 int16BE
    int16_t sysstat_imu_latency;   // off 16 int16BE      us
    float   sysstat_batt_a;        // off 18 int16BE /100 A
    int16_t sysstat_charge_mah;    // off 20 int16BE      mAh

    // Attr 12: Magnetometer
    bool    mag_valid;
    float   mag_x;            // off 1  /1000
    float   mag_y;            // off 3  /1000
    float   mag_z;            // off 5  /1000
    float   mag_magnitude;    // off 7  /100 
    float   mag_freq;         // off 9  /100 
    float   mag_off_x;        // off 11 /1000
    float   mag_off_y;        // off 13 /1000
    float   mag_off_z;        // off 15 /1000
    float   mag_declination;  // off 17 /10

    /* Attr 48: System echo (composite read-only status block).
     * Single read gives GPS lock state, rough position confirmation,
     * and current pressure in one round trip. Good boot health check.
     * Generic access: hamfly_request_attr(g, HAMFLY_ATTR_SYSTEM_ECHO).
     * off 0-7: GPS position bytes (mirrors attr 4 lon/lat).
     * off 8:   status byte: 0x09 = locked, 0xCB = no lock.
     * off 9-10: pressure /10 mb (mirrors attr 1 off 9-10).
     * off 11-12: IMU rate (mirrors attr 1 off 11-12).
     * off 13:  always 0x00. */
    bool    sysecho_valid;
    bool    sysecho_gps_locked;   /* from off 8 status byte */
    float   sysecho_pressure_mb;  /* off 9-10 /10 mb        */
    int16_t sysecho_imu_rate;     /* off 11-12              */

} hamfly_telemetry_t;

/* Convert the quaternion in telemetry (RIJK from attr 287) to Euler angles.
 * Axis order: Pan (yaw), Tilt (pitch), Roll — FreeflyAPI convention, ZYX.
 * Fills only the non-NULL output pointers. Clamps asinf argument to ±1
 * (gimbal-lock boundary) before conversion.
 * Returns 1 and fills outputs (degrees) if telemetry valid, else 0. */
uint8_t hamfly_telemetry_to_euler(const hamfly_telemetry_t *t,
                                  float *pan_deg,
                                  float *tilt_deg,
                                  float *roll_deg);

// Public decoder functions used within switch statement in hamfly_pump().
// Safe to call on example byte arrays while offline, or use for live testing.
void hamfly_decode_qx287    (const uint8_t *p, uint16_t plen, 
                             hamfly_telemetry_t *dst);
void hamfly_decode_gps      (const uint8_t *p, uint16_t plen,
                             hamfly_telemetry_t *dst);
void hamfly_decode_baro     (const uint8_t *p, uint16_t plen,
                             hamfly_telemetry_t *dst);
void hamfly_decode_attitude (const uint8_t *p, uint16_t plen,
                             hamfly_telemetry_t *dst);
void hamfly_decode_sysstat  (const uint8_t *p, uint16_t plen,
                             hamfly_telemetry_t *dst);
void hamfly_decode_mag      (const uint8_t *p, uint16_t plen,
                             hamfly_telemetry_t *dst);
void hamfly_decode_sysecho  (const uint8_t *p, uint16_t plen,
                             hamfly_telemetry_t *dst);

#endif /* HAMFLY_CORE_TELEMETRY_H */
