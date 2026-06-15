/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright 2026 Quantum Information Systems Lab, SFU Physics
 *
 * Based on the Freefly QX Protocol (Copyright 2017 Freefly Systems),
 * originally licensed under the Apache License, Version 2.0.
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
 * Directly copied from the FreeflyAPI QX app.c/h file, refactored with Claude
 * Haiku into two files for posterity: hamfly_qx_protocol and hamfly_qx_app.
 *
 * Edits:
 *  - 'extern void *qx_active_gimbal_ptr' added.
 *  - 'QX_ParsePacket_Cli_MoVI_Ctrl_CB' removed.
 */

#ifndef HAMFLY_QX_APP_H
#define HAMFLY_QX_APP_H
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * Port and instance counts  (QX_App_Config)
 * ============================================================ */
#define QX_NUM_SRV  1
#define QX_NUM_CLI  1

typedef enum {
    QX_COMMS_PORT_UART = 0,
    QX_NUM_OF_PORTS
} QX_Comms_Port_e;

/* ============================================================
 * FreeflyAPI control types
 * Must stay in sync with hamfly_control_mode_t values.
 * ============================================================ */
typedef enum {
    DEFER             = 0,
    RATE              = 1,
    ABSOLUTE          = 2,
    ABSOLUTE_MAJESTIC = 3   /* absolute + MōVI Pro window/smoothing */
} ff_api_control_type_e;

/* ============================================================
 * Gimbal control flag bit definitions  (QX277 payload byte 13)
 * ============================================================ */
#define INPUT_CONTROL_RZ_DEFER    0x00
#define INPUT_CONTROL_RZ_RATE     0x01
#define INPUT_CONTROL_RZ_ABS      0x02
#define INPUT_CONTROL_RZ_ABS_MAJ  0x03
#define INPUT_CONTROL_RY_DEFER    0x00
#define INPUT_CONTROL_RY_RATE     0x04
#define INPUT_CONTROL_RY_ABS      0x08
#define INPUT_CONTROL_RY_ABS_MAJ  0x0C
#define INPUT_CONTROL_RX_DEFER    0x00
#define INPUT_CONTROL_RX_RATE     0x10
#define INPUT_CONTROL_RX_ABS      0x20
#define INPUT_CONTROL_RX_ABS_MAJ  0x30
#define INPUT_CONTROL_QUATERNION  0x80
#define INPUT_CONTROL_KILL        0x40

/* Lens control flag bit definitions */
#define INPUT_CONTROL_LZ_DEFER  0x00
#define INPUT_CONTROL_LZ_RATE   0x01
#define INPUT_CONTROL_LZ_ABS    0x02
#define INPUT_CONTROL_LI_DEFER  0x00
#define INPUT_CONTROL_LI_RATE   0x04
#define INPUT_CONTROL_LI_ABS    0x08
#define INPUT_CONTROL_LF_DEFER  0x00
#define INPUT_CONTROL_LF_RATE   0x10
#define INPUT_CONTROL_LF_ABS    0x20
#define INPUT_CONTROL_REC       0x40

/* ============================================================
 * FreeflyAPI structs (unchanged from original)
 * ============================================================ */
typedef struct {
    float                 value;
    ff_api_control_type_e type;
} ff_api_realtime_control_t;

typedef enum {
    Lens_AxisState_Disabled             = 0,
    Lens_AxisState_Reset                = 1,
    Lens_AxisState_Faulted              = 2,
    Lens_AxisState_Move_to_Command      = 3,
    Lens_AxisState_Calibrated           = 4,
    Lens_AxisState_Uncalibrated         = 5,
    Lens_AxisState_Man_Cal_Set_Max      = 6,
    Lens_AxisState_Man_Cal_Set_Min      = 7,
    Lens_AxisState_Auto_Cal_SensingTorque=8,
    Lens_AxisState_Auto_Cal_Set_Max     = 9,
    Lens_AxisState_Auto_Cal_Set_Min     = 10
} Lens_Axis_State_General_e;

typedef struct {
    uint8_t                  gimbal_kill;
    uint8_t                  gimbal_position_type_quaternions;
    ff_api_realtime_control_t pan;
    ff_api_realtime_control_t tilt;
    ff_api_realtime_control_t roll;
    ff_api_realtime_control_t focus;
    ff_api_realtime_control_t iris;
    ff_api_realtime_control_t zoom;
    uint8_t                  fiz_clearFaults_all_flag;
    uint8_t                  fiz_autoCalStart_all_flag;
    uint8_t                  fiz_record_button_flag;
    uint8_t                  fiz_setSubRangeLim_F_flag;
    uint8_t                  fiz_setSubRangeLim_I_flag;
    uint8_t                  fiz_setSubRangeLim_Z_flag;
} ff_api_gimbal_and_fiz_control_t;

typedef struct {
    float                    battery_v_left;
    float                    battery_v_right;
    uint8_t                  gimbal_Status1;
    uint8_t                  gimbal_Status2;
    float                    gimbal_i;
    float                    gimbal_j;
    float                    gimbal_k;
    float                    gimbal_r;
    uint16_t                 focus_position;
    uint16_t                 iris_position;
    uint16_t                 zoom_position;
    Lens_Axis_State_General_e focus_state;
    Lens_Axis_State_General_e iris_state;
    Lens_Axis_State_General_e zoom_state;
    uint8_t                  fiz_status;
    uint8_t                  focus_range_limits_active;
    uint8_t                  iris_range_limits_active;
    uint8_t                  zoom_range_limits_active;
    uint8_t                  camera_status;
    uint8_t                  camera_recording;
} ff_api_status_t;

typedef struct {
    void (*begin)(void);
    void (*send) (void);
    void (*recv) (void);
    ff_api_gimbal_and_fiz_control_t control;
    ff_api_status_t                 status;
} ff_api_t;

/* ============================================================
 * Globals and callbacks
 * ============================================================ */
extern ff_api_t FreeflyAPI;

/*
 * Active gimbal instance pointer.
 * Set by hamfly_init() so QX_SendMsg2CommsPort_CB can reach
 * the correct hamfly_txbuf without a global simple_buffer.
 * For a second gimbal, extend to an array indexed by port.
 */

extern void *qx_active_gimbal_ptr;

/* Parser callback -- implemented in qx_app.c */

#endif /* HAMFLY_QX_APP_H */
