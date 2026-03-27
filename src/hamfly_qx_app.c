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
 *  - 'QX_SendMsg2CommsPort_CB' edited to write to 'qx_active_gimbal->txbuf'.
 *  - 'qx_active_gimbal_ptr' and '#define qx_active_gimbal' added for the above.
 *  - Updated includes to consolidated files.
 */

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "hamfly_qx_app.h"
#include "hamfly_qx_protocol.h"
#include "hamfly_core_gimbal.h"
#include "hamfly_comm_txbuf.h"

/* ============================================================
 * Active gimbal instance pointer
 * Set by hamfly_init(). QX_SendMsg2CommsPort_CB writes into
 * this instance's txbuf instead of the global simple_buffer.
 * For a second gimbal, extend to an array indexed by port.
 * ============================================================ */
void *qx_active_gimbal_ptr = NULL;
#define qx_active_gimbal ((hamfly_gimbal_t *)qx_active_gimbal_ptr)

/* ============================================================
 * FreeflyAPI global instance
 * ============================================================ */
static uint8_t *QX_ParsePacket_Cli_MoVI_Ctrl_CB(QX_Msg_t *Msg_p);
static void ff_api_init(void);
static void ff_api_send(void);

ff_api_t FreeflyAPI = {
    .begin   = &ff_api_init,
    .send    = &ff_api_send,
    .control = {0},
    .status  = {0}
};

/* ============================================================
 * ff_api_init -- unchanged from original
 * ============================================================ */
static void ff_api_init(void)
{
    QX_InitCli(&QX_Clients[0], QX_DEV_ID_MOVI_API_CONTROLLER,
               QX_ParsePacket_Cli_MoVI_Ctrl_CB);

    FreeflyAPI.control.pan.type   = DEFER;
    FreeflyAPI.control.tilt.type  = DEFER;
    FreeflyAPI.control.roll.type  = DEFER;
    FreeflyAPI.control.focus.type = DEFER;
    FreeflyAPI.control.iris.type  = DEFER;
    FreeflyAPI.control.zoom.type  = DEFER;
    FreeflyAPI.control.fiz_autoCalStart_all_flag  = 0;
    FreeflyAPI.control.fiz_clearFaults_all_flag   = 0;
    FreeflyAPI.control.fiz_record_button_flag     = 0;
    FreeflyAPI.control.fiz_setSubRangeLim_F_flag  = 0;
    FreeflyAPI.control.fiz_setSubRangeLim_I_flag  = 0;
    FreeflyAPI.control.fiz_setSubRangeLim_Z_flag  = 0;
}

/* ============================================================
 * ff_api_send -- unchanged from original
 * ============================================================ */
static void ff_api_send(void)
{
    QX_TxMsgOptions_t options;
    QX_InitTxOptions(&options);
    options.Target_Addr    = QX_DEV_ID_GIMBAL;
    options.TransReq_Addr  = QX_DEV_ID_BROADCAST;
    options.RespReq_Addr   = QX_DEV_ID_BROADCAST;
    QX_SendPacket_Cli_WriteABS(&QX_Clients[0], 277, QX_COMMS_PORT_UART, options);
}

/* ============================================================
 * Parser callback -- unchanged from original
 * ============================================================ */
static uint8_t *QX_ParsePacket_Cli_MoVI_Ctrl_CB(QX_Msg_t *Msg_p)
{
    uint8_t  temp_UC_buffer[10];
    int16_t  temp_SS_buffer[10];
    uint16_t temp_US_buffer[10];

    switch (Msg_p->Parse_Type) {
        case QX_PARSE_TYPE_WRITE_REL_SEND: QX_Parser_SetDir_Read();     break;
        case QX_PARSE_TYPE_WRITE_ABS_SEND: QX_Parser_SetDir_Read();     break;
        case QX_PARSE_TYPE_CURVAL_RECV:    QX_Parser_SetDir_WriteAbs(); break;
        default: break;
    }

    QX_Parser_SetMsgPtr(Msg_p->BufPayloadStart_p);

    switch (Msg_p->Header.Attrib) {

        case 277:
            if (Msg_p->Parse_Type != QX_PARSE_TYPE_WRITE_ABS_SEND) break;

            PARSE_UC_AS_UC(temp_UC_buffer, 3, 0xFF, 0);

            {
                uint8_t pan_input_type;
                switch (FreeflyAPI.control.pan.type) {
                    case DEFER:    pan_input_type = INPUT_CONTROL_RZ_DEFER; break;
                    case RATE:     pan_input_type = INPUT_CONTROL_RZ_RATE;  break;
                    case ABSOLUTE: pan_input_type = INPUT_CONTROL_RZ_ABS;   break;
                    default:       pan_input_type = INPUT_CONTROL_RZ_DEFER; break;
                }
                uint8_t tilt_input_type;
                switch (FreeflyAPI.control.tilt.type) {
                    case DEFER:    tilt_input_type = INPUT_CONTROL_RY_DEFER; break;
                    case RATE:     tilt_input_type = INPUT_CONTROL_RY_RATE;  break;
                    case ABSOLUTE: tilt_input_type = INPUT_CONTROL_RY_ABS;   break;
                    default:       tilt_input_type = INPUT_CONTROL_RY_DEFER; break;
                }
                uint8_t roll_input_type;
                switch (FreeflyAPI.control.roll.type) {
                    case DEFER:    roll_input_type = INPUT_CONTROL_RX_DEFER; break;
                    case RATE:     roll_input_type = INPUT_CONTROL_RX_RATE;  break;
                    case ABSOLUTE: roll_input_type = INPUT_CONTROL_RX_ABS;   break;
                    default:       roll_input_type = INPUT_CONTROL_RX_DEFER; break;
                }

                temp_UC_buffer[0] = pan_input_type | tilt_input_type | roll_input_type;

                if (FreeflyAPI.control.gimbal_position_type_quaternions)
                    temp_UC_buffer[0] |=  INPUT_CONTROL_QUATERNION;
                else
                    temp_UC_buffer[0] &= ~INPUT_CONTROL_QUATERNION;

                if (FreeflyAPI.control.gimbal_kill)
                    temp_UC_buffer[0] |=  INPUT_CONTROL_KILL;
                else
                    temp_UC_buffer[0] &= ~INPUT_CONTROL_KILL;
            }

            PARSE_UC_AS_UC(temp_UC_buffer, 1, 0xFF, 0);

            PARSE_FL_AS_SS(&FreeflyAPI.control.roll.value,  1, 32767.0f, -32767.0f, 32767.0f);
            PARSE_FL_AS_SS(&FreeflyAPI.control.tilt.value,  1, 32767.0f, -32767.0f, 32767.0f);
            PARSE_FL_AS_SS(&FreeflyAPI.control.pan.value,   1, 32767.0f, -32767.0f, 32767.0f);
            temp_SS_buffer[0] = 0;
            PARSE_SS_AS_SS(temp_SS_buffer, 1, 0xFFFF, 0);

            {
                uint8_t focus_input_type;
                switch (FreeflyAPI.control.focus.type) {
                    case DEFER:    focus_input_type = INPUT_CONTROL_LF_DEFER; break;
                    case RATE:     focus_input_type = INPUT_CONTROL_LF_RATE;  break;
                    case ABSOLUTE: focus_input_type = INPUT_CONTROL_LF_ABS;   break;
                    default:       focus_input_type = INPUT_CONTROL_LF_DEFER; break;
                }
                uint8_t iris_input_type;
                switch (FreeflyAPI.control.iris.type) {
                    case DEFER:    iris_input_type = INPUT_CONTROL_LI_DEFER; break;
                    case RATE:     iris_input_type = INPUT_CONTROL_LI_RATE;  break;
                    case ABSOLUTE: iris_input_type = INPUT_CONTROL_LI_ABS;   break;
                    default:       iris_input_type = INPUT_CONTROL_LI_DEFER; break;
                }
                uint8_t zoom_input_type;
                switch (FreeflyAPI.control.zoom.type) {
                    case DEFER:    zoom_input_type = INPUT_CONTROL_LZ_DEFER; break;
                    case RATE:     zoom_input_type = INPUT_CONTROL_LZ_RATE;  break;
                    case ABSOLUTE: zoom_input_type = INPUT_CONTROL_LZ_ABS;   break;
                    default:       zoom_input_type = INPUT_CONTROL_LZ_DEFER; break;
                }
                temp_UC_buffer[0] = focus_input_type | iris_input_type | zoom_input_type;
            }

            PARSE_UC_AS_UC(temp_UC_buffer, 1, 0xFF, 0);

            temp_US_buffer[0] = (uint16_t)(FreeflyAPI.control.focus.value * (0xFFFF / 2) + (0xFFFF / 2));
            temp_US_buffer[1] = (uint16_t)(FreeflyAPI.control.iris.value  * (0xFFFF / 2) + (0xFFFF / 2));
            temp_US_buffer[2] = (uint16_t)(FreeflyAPI.control.zoom.value  * (0xFFFF / 2) + (0xFFFF / 2));
            PARSE_US_AS_US(temp_US_buffer, 3, 0xFFFF, 0);

            PARSE_BITS_AS_UC(&FreeflyAPI.control.fiz_clearFaults_all_flag,  0, 1);
            PARSE_BITS_AS_UC(&FreeflyAPI.control.fiz_autoCalStart_all_flag, 1, 1);
            PARSE_BITS_AS_UC(&FreeflyAPI.control.fiz_record_button_flag,    4, 1);
            PARSE_BITS_AS_UC(&FreeflyAPI.control.fiz_setSubRangeLim_F_flag, 5, 1);
            PARSE_BITS_AS_UC(&FreeflyAPI.control.fiz_setSubRangeLim_I_flag, 6, 1);
            PARSE_BITS_AS_UC(&FreeflyAPI.control.fiz_setSubRangeLim_Z_flag, 7, 1);
            QX_Parser_AdvMsgPtr();

            temp_UC_buffer[0] = 0;
            PARSE_UC_AS_UC(temp_UC_buffer, 1, 0xFF, 0);
            break;

        case 287:
            if (Msg_p->Parse_Type != QX_PARSE_TYPE_CURVAL_RECV) break;

            PARSE_UC_AS_UC(temp_UC_buffer, 3, 0xFF, 0);

            PARSE_UC_AS_UC(temp_UC_buffer, 2, 0xFF, 0);
            FreeflyAPI.status.battery_v_left  = ((float)temp_UC_buffer[0] * 0.1f) + 10.0f;
            FreeflyAPI.status.battery_v_right = ((float)temp_UC_buffer[1] * 0.1f) + 10.0f;
            if (FreeflyAPI.status.battery_v_left  == 10.0f) FreeflyAPI.status.battery_v_left  = 0;
            if (FreeflyAPI.status.battery_v_right == 10.0f) FreeflyAPI.status.battery_v_right = 0;

            PARSE_UC_AS_UC(&FreeflyAPI.status.gimbal_Status1, 1, 0xFF, 0);
            PARSE_UC_AS_UC(&FreeflyAPI.status.gimbal_Status2, 1, 0xFF, 0);

            PARSE_FL_AS_SS(&FreeflyAPI.status.gimbal_r, 1, 32767.0f, -32767.0f, 32767.0f);
            PARSE_FL_AS_SS(&FreeflyAPI.status.gimbal_i, 1, 32767.0f, -32767.0f, 32767.0f);
            PARSE_FL_AS_SS(&FreeflyAPI.status.gimbal_j, 1, 32767.0f, -32767.0f, 32767.0f);
            PARSE_FL_AS_SS(&FreeflyAPI.status.gimbal_k, 1, 32767.0f, -32767.0f, 32767.0f);

            PARSE_UC_AS_UC(temp_UC_buffer, 2, 0xFF, 0);

            PARSE_BITS_AS_UC(&FreeflyAPI.status.focus_range_limits_active, 0, 1);
            PARSE_BITS_AS_UC(&FreeflyAPI.status.iris_range_limits_active,  1, 1);
            PARSE_BITS_AS_UC(&FreeflyAPI.status.zoom_range_limits_active,  2, 1);
            QX_Parser_AdvMsgPtr();

            PARSE_BITS_AS_UC(&FreeflyAPI.status.camera_recording, 0, 1);
            QX_Parser_AdvMsgPtr();

            PARSE_UC_AS_UC((uint8_t *)&FreeflyAPI.status.focus_state, 1, 0xFF, 0);
            PARSE_UC_AS_UC((uint8_t *)&FreeflyAPI.status.iris_state,  1, 0xFF, 0);
            PARSE_UC_AS_UC((uint8_t *)&FreeflyAPI.status.zoom_state,  1, 0xFF, 0);

            PARSE_US_AS_US(&FreeflyAPI.status.focus_position, 1, 0xFFFF, 0);
            PARSE_US_AS_US(&FreeflyAPI.status.iris_position,  1, 0xFFFF, 0);
            PARSE_US_AS_US(&FreeflyAPI.status.zoom_position,  1, 0xFFFF, 0);
            break;

        default:
            Msg_p->AttNotHandled = 1;
            break;
    }

    return (uint8_t *)QX_Parser_GetMsgPtr();
}

/* ============================================================
 * QX_SendMsg2CommsPort_CB
 * Key change from original: writes into qx_active_gimbal->txbuf
 * instead of the global simple_buffer singleton.
 * ============================================================ */
void QX_SendMsg2CommsPort_CB(QX_Msg_t *TxMsg_p)
{
    int i;
    if (!qx_active_gimbal) return;

    switch (TxMsg_p->CommPort) {
        case QX_COMMS_PORT_UART:
            TxMsg_p->MsgBuf_p = TxMsg_p->MsgBufStart_p;
            for (i = 0; i < TxMsg_p->MsgBuf_MsgLen; i++)
                hamfly_txbuf_add(&qx_active_gimbal->txbuf, *TxMsg_p->MsgBuf_p++);
            break;
        default:
            break;
    }
}

void QX_FwdMsg_CB(QX_Msg_t *TxMsg_p) { (void)TxMsg_p; }

/* ============================================================
 * QX_GetTicks_ms
 * Wired to the active gimbal's HAL get_tick_ms if provided,
 * otherwise returns 0 (matches original behaviour).
 * ============================================================ */
uint32_t QX_GetTicks_ms(void)
{
    if (qx_active_gimbal && qx_active_gimbal->hal.get_tick_ms)
        return qx_active_gimbal->hal.get_tick_ms(qx_active_gimbal->hal.ctx);
    return 0u;
}
