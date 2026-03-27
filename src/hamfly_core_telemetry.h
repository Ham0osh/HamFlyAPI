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
    bool    gps_valid;
    float   gps_lat_deg;
    float   gps_lon_deg;
    float   gps_alt_m;
    float   gps_ground_spd_ms;
    float   gps_heading_deg;
    float   gps_hacc_m;
    float   gps_vacc_m;
    float   gps_sacc_ms;
    int32_t gps_raw_lat;  // kept: float loses precision at 1e7
    int32_t gps_raw_lon;
    int32_t gps_raw_alt;
    int16_t gps_raw_spd;
    int16_t gps_raw_hdg;
    int16_t gps_raw_hacc;
    int16_t gps_raw_vacc;
    int16_t gps_raw_sacc;

    // Attr 3: Barometric altitude
    bool    baro_valid;
    float   baro_alt_m;
    float   baro_roc_ms;

    // Attr 22: Gimbal Euler (LE, must be requested)
    bool    att_valid;
    float   att_pitch_deg;       // off 1 /100
    float   att_yaw_deg;         // off 3 /100
    float   att_roll_deg;        // off 5 /100
    float   att_roll_err_deg;    // off 7 /10
    float   att_pitch_err_deg;   // off 9 /10
    float   att_yaw_err_deg;     // off 11 /10
    float   att_roll_rate_dps;   // off 13 /10
    float   att_pitch_rate_dps;  // off 15 /10
    float   att_yaw_rate_dps;    // off 17 /10

    // Attr 1: System status
    bool    sysstat_valid;
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

} hamfly_telemetry_t;

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

#endif /* HAMFLY_CORE_TELEMETRY_H */
