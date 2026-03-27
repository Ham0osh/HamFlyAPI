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
 */

#include "hamfly_core_telemetry.h"
#include "hamfly_qx_protocol.h"
#include <string.h>
#include <stdint.h>

// ============================================================================
// Helpers: Big and Little Endian readers.
// Used for attr 1, 4, 287 which are big-endian with no sub-index byte.
// Attr 22 is little-endian with a 0x00 sub-index at offset 0, so use rd_le16.
// ============================================================================

// Read big-endian int32 from buffer.
static int32_t rd_be32(const uint8_t *p)
{
    return (int32_t)(  ((uint32_t)p[0] << 24)
                     | ((uint32_t)p[1] << 16)
                     | ((uint32_t)p[2] <<  8)
                     |  (uint32_t)p[3]);
}

// Read big-endian int16 from buffer.
static int16_t rd_be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

// Read little-endian int16 from buffer.
static int16_t rd_le16(const uint8_t *p)
{
    int16_t v;
    memcpy(&v, p, 2);
    return v;
}

// ============================================================================
// Hamfly Decode QX287
// ============================================================================
// Main telemetry parsing function. Reads from FreeFlyAPI.status which is
// populated by the QX parser callback. Wrapping FreeflyAPI directly.
void hamfly_decode_qx287(const uint8_t *p, uint16_t plen,
                          hamfly_telemetry_t *dst)
{
    (void)p; (void)plen;

    // Copy fields into the telemetry struct.
    dst->battery_left_v   = FreeflyAPI.status.battery_v_left;
    dst->battery_right_v  = FreeflyAPI.status.battery_v_right;
    dst->gimbal_r         = FreeflyAPI.status.gimbal_r;
    dst->gimbal_i         = FreeflyAPI.status.gimbal_i;
    dst->gimbal_j         = FreeflyAPI.status.gimbal_j;
    dst->gimbal_k         = FreeflyAPI.status.gimbal_k;
    dst->gimbal_status1   = FreeflyAPI.status.gimbal_Status1;
    dst->gimbal_status2   = FreeflyAPI.status.gimbal_Status2;
    dst->valid            = true;
}

// ============================================================================
// Hamfly Decode GPS [Hamfly Specific] -- attr 4
// ============================================================================
// Reads the raw bytes of the GPS payload and into telemetry struct fields:
// 
// Big-endian, no sub-index byte, 22 byte payload.
// off  0-3  BE int32: longitude    /1e7  deg
// off  4-7  BE int32: latitude     /1e7  deg
// off  8-11 BE int32: altitude     /1000 m
// off 12-13 BE int16: ground speed /100  m/s
// off 14-15 BE int16: heading      /10   deg
// off 16-17 BE int16: HACC         /100  m
// off 18-19 BE int16: VACC         /100  m
// off 20-21 BE int16: SACC         /100  m/s
// ============================================================================
void hamfly_decode_gps(const uint8_t *p, uint16_t plen,
                        hamfly_telemetry_t *dst)
{
    if (plen < 22u) return;
    // Decode to int.
    int32_t lon_raw  = rd_be32(p +  0);
    int32_t lat_raw  = rd_be32(p +  4);
    int32_t alt_raw  = rd_be32(p +  8);
    int16_t spd_raw  = rd_be16(p + 12);
    int16_t hdg_raw  = rd_be16(p + 14);
    int16_t hacc_raw = rd_be16(p + 16);
    int16_t vacc_raw = rd_be16(p + 18);
    int16_t sacc_raw = rd_be16(p + 20);
    // Save raw for debug.
    dst->gps_raw_lon  = lon_raw;
    dst->gps_raw_lat  = lat_raw;
    dst->gps_raw_alt  = alt_raw;
    dst->gps_raw_spd  = spd_raw;
    dst->gps_raw_hdg  = hdg_raw;
    dst->gps_raw_hacc = hacc_raw;
    dst->gps_raw_vacc = vacc_raw;
    dst->gps_raw_sacc = sacc_raw;
    // Decode and save.
    dst->gps_lon_deg       = (float)lon_raw  / 1e7f;
    dst->gps_lat_deg       = (float)lat_raw  / 1e7f;
    dst->gps_alt_m         = (float)alt_raw  / 1000.0f;
    dst->gps_ground_spd_ms = (float)spd_raw  / 100.0f;
    dst->gps_heading_deg   = (float)hdg_raw  / 10.0f;
    dst->gps_hacc_m        = (float)hacc_raw / 100.0f;
    dst->gps_vacc_m        = (float)vacc_raw / 100.0f;
    dst->gps_sacc_ms       = (float)sacc_raw / 100.0f;
    dst->gps_valid         = true;
}

// ============================================================================
// Hamfly Decode Barometer [Hamfly Specific] -- attr 3
// ============================================================================
// Reads the raw bytes of the Barometer payload and into telemetry fields:
// 
// Little-endian, sub-index byte at off 0 (ignored), 5 byte payload.
// off 1-2  LE int16: altitude      /10  m
// off 3-4  LE int16: rate of climb /100 m/s
// ============================================================================
void hamfly_decode_baro(const uint8_t *p, uint16_t plen,
                         hamfly_telemetry_t *dst)
{
    if (plen < 5u) return;

    // Decode and populate.
    dst->baro_alt_m  = (float)rd_le16(p + 1) / 10.0f;
    dst->baro_roc_ms = (float)rd_le16(p + 3) / 100.0f;
    dst->baro_valid  = true;
}

// ============================================================================
// Hamfly Decode Attitude [Hamfly Specific] -- attr 22
// ============================================================================
// Reads the raw bytes of the Attitude payload and into telemetry fields:
// 
// Little-endian, sub-index byte at off 0 (ignored), 19 byte payload.
// off 1-2  LE int16: Pitch       /100 deg
// off 3-4  LE int16: Yaw         /100 deg
// off 5-6  LE int16: Roll        /100 deg
// off 7-8  LE int16: Roll error  /10  deg
// off 9-10 LE int16: Pitch error /10  deg
// off 11-12 LE int16: Yaw error  /10  deg
// off 13-14 LE int16: Roll rate  /10  dps
// off 15-16 LE int16: Pitch rate /10  dps
// off 17-18 LE int16: Yaw rate   /10  dps
// ============================================================================
void hamfly_decode_attitude(const uint8_t *p, uint16_t plen,
                              hamfly_telemetry_t *dst)
{
    if (plen < 19u) return;

    // Decode and populate.
    dst->att_pitch_deg      = (float)rd_le16(p +  1) / 100.0f;
    dst->att_yaw_deg        = (float)rd_le16(p +  3) / 100.0f;
    dst->att_roll_deg       = (float)rd_le16(p +  5) / 100.0f;
    dst->att_roll_err_deg   = (float)rd_le16(p +  7) / 10.0f;
    dst->att_pitch_err_deg  = (float)rd_le16(p +  9) / 10.0f;
    dst->att_yaw_err_deg    = (float)rd_le16(p + 11) / 10.0f;
    dst->att_roll_rate_dps  = (float)rd_le16(p + 13) / 10.0f;
    dst->att_pitch_rate_dps = (float)rd_le16(p + 15) / 10.0f;
    dst->att_yaw_rate_dps   = (float)rd_le16(p + 17) / 10.0f;
    dst->att_valid          = true;
}

// ============================================================================
// Hamfly Decode System Status [Hamfly Specific] -- attr 1
// ============================================================================
// Reads the raw bytes of the System Status payload and into telemetry fields:
// 
// Big-endian, no sub-index byte, 22 byte payload.
// off 0-1   BE int16:  battery voltage /100 V
// off 2     uint8:     GPS satellites
// off 3     uint8:     temperature     /3   C
// off 4     uint8:     CPU         %        %
// off 5-6   BE int16:  status flags
// off 7-8   BE int16:  time                 s
// off 9-10  BE int16:  pressure        /10  mb
// off 11-12 BE int16:  IMU rate
// off 13-14 BE int16:  camera status
// off 15    uint8:     reserved
// off 16-17 BE int16:  IMU latency          us
// off 18-19 BE int16:  battery current /100 A
// off 20-21 BE int16:  charge used          mAh
// ============================================================================
void hamfly_decode_sysstat(const uint8_t *p, uint16_t plen,
                             hamfly_telemetry_t *dst)
{
    if (plen < 22u) return;

    // Decode and populate.
    dst->sysstat_batt_v       = (float)rd_be16(p +  0) / 100.0f;
    dst->sysstat_gps_sats     = p[2];
    dst->sysstat_temp_c       = (float)p[3] / 3.0f;
    dst->sysstat_cpu_pct      = p[4];
    dst->sysstat_status_flags = rd_be16(p +  5);
    dst->sysstat_time_s       = rd_be16(p +  7);
    dst->sysstat_pressure_mb  = (float)rd_be16(p +  9) / 10.0f;
    dst->sysstat_imu_rate     = rd_be16(p + 11);
    dst->sysstat_cam_status   = rd_be16(p + 13);
    /* off 15: reserved, skip */
    dst->sysstat_imu_latency  = rd_be16(p + 16);
    dst->sysstat_batt_a       = (float)rd_be16(p + 18) / 100.0f;
    dst->sysstat_charge_mah   = rd_be16(p + 20);
    dst->sysstat_valid        = true;
}

// ============================================================================
// Hamfly Decode Magnetometer [Hamfly Specific] -- attr 12
// ============================================================================
// Reads the raw bytes of the Magnetometer payload and into telemetry fields:
// 
// Little-endian, sub-index byte at off 0 (ignored), 19 byte payload.
// off 1-2   LE int16:  X           /1000
// off 3-4   LE int16:  Y           /1000
// off 5-6   LE int16:  Z           /1000
// off 7-8   LE int16:  magnitude   /100
// off 9-10  LE int16:  frequency   /100
// off 11-12 LE int16:  offset X    /1000
// off 13-14 LE int16:  offset Y    /1000
// off 15-16 LE int16:  offset Z    /1000
// off 17-18 LE int16:  declination /10
// ============================================================================
void hamfly_decode_mag(const uint8_t *p, uint16_t plen,
                        hamfly_telemetry_t *dst)
{
    if (plen < 19u) return;

    // Decode and populate.
    dst->mag_x           = (float)rd_le16(p +  1) / 1000.0f;
    dst->mag_y           = (float)rd_le16(p +  3) / 1000.0f;
    dst->mag_z           = (float)rd_le16(p +  5) / 1000.0f;
    dst->mag_magnitude   = (float)rd_le16(p +  7) / 100.0f;
    dst->mag_freq        = (float)rd_le16(p +  9) / 100.0f;
    dst->mag_off_x       = (float)rd_le16(p + 11) / 1000.0f;
    dst->mag_off_y       = (float)rd_le16(p + 13) / 1000.0f;
    dst->mag_off_z       = (float)rd_le16(p + 15) / 1000.0f;
    dst->mag_declination = (float)rd_le16(p + 17) / 10.0f;
    dst->mag_valid       = true;
}
