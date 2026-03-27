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
 * Directly copied from the FreeflyAPI QX protocol.c/h, refactored with Claude
 * Haiku into two files for posterity: hamfly_qx_protocol and hamfly_qx_app.
 *
 * Edits:
 *  - Updated includes to consolidated files.
 */

#include "hamfly_qx_protocol.h"
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

/* ============================================================
 * Globals
 * ============================================================ */
QX_Server_t    QX_Servers[QX_NUM_SRV];
QX_Client_t    QX_Clients[QX_NUM_CLI];
QX_CommsPort_t QX_CommsPorts[QX_NUM_OF_PORTS];

void (*QX_BuildHeader_Legacy)(QX_Msg_t *Msg_p) = NULL;
void (*QX_ParseHeader_Legacy)(QX_Msg_t *Msg_p) = NULL;

/* QX_Parsing_Functions globals */
volatile uint8_t *msgPtr;
QB_Parser_Dir_e   rw;
static QB_Parser_Dir_e rw_orig;

/* ============================================================
 * Private prototypes
 * ============================================================ */
static QX_Stat_e QX_InitMsg          (QX_Msg_t *Msg_p);
static QX_Stat_e QX_RxMsg            (QX_Msg_t *RxMsg_p);
static QX_Stat_e QX_TxMsg_Setup      (QX_Msg_t *TxMsg_p);
static QX_Stat_e QX_TxMsg_Finish     (QX_Msg_t *TxMsg_p);
static void      QX_Srv_Rx_Read      (QX_Msg_t *RxMsg_p);
static void      QX_Srv_Rx_Write     (QX_Msg_t *RxMsg_p);
static void      QX_Cli_Rx_CurVal    (QX_Msg_t *RxMsg_p);
static void      QX_BuildHeader      (QX_Msg_t *Msg_p);
static void      QX_ParseHeader      (QX_Msg_t *Msg_p);
static void      QX_AddExtdValToBuf  (uint8_t **p, uint32_t Val);
static uint32_t  QX_GetExtdValFromBuf(uint8_t **p);
static uint8_t   QX_Calc8bChecksum   (uint8_t *buf_p, uint32_t len);
static uint32_t  QX_compute_crc32    (uint8_t data);

/* ============================================================
 * Internal parser macros (QX_Parsing_Functions private)
 * ============================================================ */
#define BITFIELD_MASK_1  0x01
#define BITFIELD_MASK_2  0x03
#define BITFIELD_MASK_3  0x07
#define BITFIELD_MASK_4  0x0F
#define BITFIELD_MASK_5  0x1F
#define BITFIELD_MASK_6  0x3F
#define BITFIELD_MASK_7  0x7F
#define BITFIELD_MASK_8  0xFF

#define ADDSL(v)   { *msgPtr++ = (uint8_t)((v) >> 24); *msgPtr++ = (uint8_t)((v) >> 16); *msgPtr++ = (uint8_t)((v) >> 8); *msgPtr++ = (uint8_t)(v); }
#define ADDSS(v)   { *msgPtr++ = (uint8_t)((v) >> 8); *msgPtr++ = (uint8_t)(v); }
#define ADDUS(v)   { *msgPtr++ = (uint8_t)((v) >> 8); *msgPtr++ = (uint8_t)(v); }
#define ADDCHAR(v) { *msgPtr++ = (uint8_t)(v); }

#define GETUC  ((uint8_t)*msgPtr++)
#define GETSC  ((int8_t)*msgPtr++)
#define GETSS  ((int16_t)((*msgPtr) << 8) | (int16_t)((*(msgPtr + 1)))); msgPtr += 2;
#define GETUS  ((uint16_t)((*msgPtr) << 8) | (uint16_t)((*(msgPtr + 1)))); msgPtr += 2;
#define GETSL  ((int32_t)((*msgPtr) << 24) | (int32_t)((*(msgPtr+1)) << 16) | (int32_t)((*(msgPtr+2)) << 8) | (int32_t)(*(msgPtr+3))); msgPtr += 4;

/* ============================================================
 * QX_Parsing_Functions implementations (verbatim)
 * ============================================================ */

void QX_Parser_SetMsgPtr(uint8_t *p)         { msgPtr = p; }
void QX_Parser_AdvMsgPtr(void)               { msgPtr++; }
volatile uint8_t *QX_Parser_GetMsgPtr(void)  { return msgPtr; }
void QX_Parser_SetDir_Read(void)             { rw = QB_Parser_Dir_Read; }
void QX_Parser_SetDir_WriteRel(void)         { rw = QB_Parser_Dir_WriteDel; }
void QX_Parser_SetDir_WriteAbs(void)         { rw = QB_Parser_Dir_WriteAbs; }
QB_Parser_Dir_e QX_Parser_GetDir(void)       { return rw; }

void QX_Parser_Dir_ForceWriteAbs_Set(void) {
    rw_orig = rw;
    if (rw == QB_Parser_Dir_WriteDel) rw = QB_Parser_Dir_WriteAbs;
}
void QX_Parser_Dir_ForceWriteAbs_Reset(void) { rw = rw_orig; }

void AddFloatAsSignedLong(float *v, uint32_t n, float scaleto) {
    while (n--) { int32_t value = (int32_t)(((*v)*scaleto) + 0.5f*((0<*v)-(*v<0))); ADDSL(value); v++; }
}
void AddFloatAsSignedShort(float *v, uint32_t n, float scaleto) {
    while (n--) { int16_t value = (int16_t)(((*v)*scaleto) + 0.5f*((0<*v)-(*v<0))); ADDSS(value); v++; }
}
void AddFloatAsSignedChar(float *v, uint32_t n, float scaleto) {
    while (n--) { int8_t value = (int8_t)(((*v)*scaleto) + 0.5f*((0<*v)-(*v<0))); ADDCHAR(value); v++; }
}
void AddFloatAsUnsignedChar(float *v, uint32_t n, float scaleto) {
    while (n--) { uint8_t value = (uint8_t)(((*v)*scaleto) + 0.5f*((0<*v)-(*v<0))); ADDCHAR(value); v++; }
}
void AddFloatAsUnsignedShort(float *v, uint32_t n, float scaleto) {
    while (n--) { uint16_t value = (uint16_t)(((*v)*scaleto) + 0.5f*((0<*v)-(*v<0))); ADDUS(value); v++; }
}

void GetFloatAsSignedLong(float *v, uint32_t n, float max, float min, float scalefrom) {
    if (rw == QB_Parser_Dir_WriteDel) {
        while (n--) { float r = (float)GETSL; r *= scalefrom; *v += r;
            if (max<*v)*v=max; if (*v<min)*v=min; if (isnan(*v))*v=0.0f; v++; }
    } else {
        while (n--) { float r = (float)GETSL; r *= scalefrom; *v = r;
            if (max<*v)*v=max; if (*v<min)*v=min; if (isnan(*v))*v=0.0f; v++; }
    }
}
void GetFloatAsSignedShort(float *v, uint32_t n, float max, float min, float scalefrom) {
    if (rw == QB_Parser_Dir_WriteDel) {
        while (n--) { float r = (float)GETSS; r *= scalefrom; *v += r;
            if (max<*v)*v=max; if (*v<min)*v=min; if (isnan(*v))*v=0.0f; v++; }
    } else {
        while (n--) { float r = (float)GETSS; r *= scalefrom; *v = r;
            if (max<*v)*v=max; if (*v<min)*v=min; if (isnan(*v))*v=0.0f; v++; }
    }
}
void GetFloatAsSignedChar(float *v, uint32_t n, float max, float min, float scalefrom) {
    if (rw == QB_Parser_Dir_WriteDel) {
        while (n--) { float r = (float)GETSC; r *= scalefrom; *v += r;
            if (max<*v)*v=max; if (*v<min)*v=min; if (isnan(*v))*v=0.0f; v++; }
    } else {
        while (n--) { float r = (float)GETSC; r *= scalefrom; *v = r;
            if (max<*v)*v=max; if (*v<min)*v=min; if (isnan(*v))*v=0.0f; v++; }
    }
}
void GetFloatAsUnsignedChar(float *v, uint32_t n, float max, float min, float scalefrom) {
    if (rw == QB_Parser_Dir_WriteDel) {
        while (n--) { float r = (float)GETSC; r *= scalefrom; *v += r;
            if (max<*v)*v=max; if (*v<min)*v=min; if (isnan(*v))*v=0.0f; v++; }
    } else {
        while (n--) { float r = (float)GETUC; r *= scalefrom; *v = r;
            if (max<*v)*v=max; if (*v<min)*v=min; if (isnan(*v))*v=0.0f; v++; }
    }
}
void GetFloatAsUnsignedShort(float *v, uint32_t n, float max, float min, float scalefrom) {
    if (rw == QB_Parser_Dir_WriteDel) {
        while (n--) { float r = (float)GETUS; r *= scalefrom; *v += r;
            if (max<*v)*v=max; if (*v<min)*v=min; if (isnan(*v))*v=0.0f; v++; }
    } else {
        while (n--) { float r = (float)GETUS; r *= scalefrom; *v = r;
            if (max<*v)*v=max; if (*v<min)*v=min; if (isnan(*v))*v=0.0f; v++; }
    }
}

void AddSignedLongAsSignedLong(int32_t *v, uint32_t n) {
    while (n--) { int32_t val = *v; ADDSL(val); v++; }
}
void AddSignedLongAsSignedShort(int32_t *v, uint32_t n) {
    while (n--) { int16_t val = (int16_t)*v; ADDSS(val); v++; }
}
void AddSignedLongAsSignedChar(int32_t *v, uint32_t n) {
    while (n--) { int8_t val = (int8_t)*v; ADDCHAR(val); v++; }
}
void AddSignedLongAsUnsignedChar(int32_t *v, uint32_t n) {
    while (n--) { uint8_t val = (uint8_t)*v; ADDCHAR(val); v++; }
}
void GetSignedLongAsSignedLong(int32_t *v, uint32_t n, int32_t max, int32_t min) {
    if (rw == QB_Parser_Dir_WriteDel) {
        while (n--) { int32_t r = GETSL; *v += r; if (max<*v)*v=max; if (*v<min)*v=min; v++; }
    } else {
        while (n--) { int32_t r = GETSL; *v  = r; if (max<*v)*v=max; if (*v<min)*v=min; v++; }
    }
}
void GetSignedLongAsSignedShort(int32_t *v, uint32_t n, int32_t max, int32_t min) {
    if (rw == QB_Parser_Dir_WriteDel) {
        while (n--) { int32_t r = (int32_t)(int16_t)GETSS; *v += r; if (max<*v)*v=max; if (*v<min)*v=min; v++; }
    } else {
        while (n--) { int32_t r = (int32_t)(int16_t)GETSS; *v  = r; if (max<*v)*v=max; if (*v<min)*v=min; v++; }
    }
}
void GetSignedLongAsSignedChar(int32_t *v, uint32_t n, int32_t max, int32_t min) {
    if (rw == QB_Parser_Dir_WriteDel) {
        while (n--) { int32_t r = (int32_t)GETSC; *v += r; if (max<*v)*v=max; if (*v<min)*v=min; v++; }
    } else {
        while (n--) { int32_t r = (int32_t)GETSC; *v  = r; if (max<*v)*v=max; if (*v<min)*v=min; v++; }
    }
}
void GetSignedLongAsUnsignedChar(int32_t *v, uint32_t n, int32_t max, int32_t min) {
    if (rw == QB_Parser_Dir_WriteDel) {
        while (n--) { int32_t r = (int32_t)GETSC; *v += r; if (max<*v)*v=max; if (*v<min)*v=min; v++; }
    } else {
        while (n--) { int32_t r = (int32_t)GETUC; *v  = r; if (max<*v)*v=max; if (*v<min)*v=min; v++; }
    }
}

void AddSignedShortAsSignedShort(int16_t *v, uint32_t n) {
    while (n--) { int16_t val = *v; ADDSS(val); v++; }
}
void AddSignedShortAsSignedChar(int16_t *v, uint32_t n) {
    while (n--) { int8_t val = (int8_t)*v; ADDCHAR(val); v++; }
}
void AddSignedShortAsUnsignedChar(int16_t *v, uint32_t n) {
    while (n--) { uint8_t val = (uint8_t)*v; ADDCHAR(val); v++; }
}
void GetSignedShortAsSignedShort(int16_t *v, uint32_t n, float max, float min) {
    if (rw == QB_Parser_Dir_WriteDel) {
        while (n--) { int16_t r = GETSS; *v += r; if (max<*v)*v=(int16_t)max; if (*v<min)*v=(int16_t)min; v++; }
    } else {
        while (n--) { int16_t r = GETSS; *v  = r; if (max<*v)*v=(int16_t)max; if (*v<min)*v=(int16_t)min; v++; }
    }
}
void GetSignedShortAsSignedChar(int16_t *v, uint32_t n, float max, float min) {
    if (rw == QB_Parser_Dir_WriteDel) {
        while (n--) { int16_t r = (int16_t)GETSC; *v += r; if (max<*v)*v=(int16_t)max; if (*v<min)*v=(int16_t)min; v++; }
    } else {
        while (n--) { int16_t r = (int16_t)GETSC; *v  = r; if (max<*v)*v=(int16_t)max; if (*v<min)*v=(int16_t)min; v++; }
    }
}
void GetSignedShortAsUnsignedChar(int16_t *v, uint32_t n, int16_t max, int16_t min) {
    if (rw == QB_Parser_Dir_WriteDel) {
        while (n--) { int16_t r = (int16_t)GETSC; *v += r; if (max<*v)*v=max; if (*v<min)*v=min; v++; }
    } else {
        while (n--) { int16_t r = (int16_t)GETUC; *v  = r; if (max<*v)*v=max; if (*v<min)*v=min; v++; }
    }
}

void AddSignedCharAsSignedChar(int8_t *v, uint32_t n) {
    while (n--) { ADDCHAR(*v); v++; }
}
void GetSignedCharAsSignedChar(int8_t *v, uint32_t n, int8_t max, int8_t min) {
    if (rw == QB_Parser_Dir_WriteDel) {
        while (n--) { int8_t r = GETSC; int32_t t = *v; t += r;
            if (max<t) t=max; if (t<min) t=min; *v = (int8_t)t; v++; }
    } else {
        while (n--) { int8_t r = GETSC; *v = r;
            if (max<*v)*v=max; if (*v<min)*v=min; v++; }
    }
}

void AddUnsignedCharAsUnsignedChar(uint8_t *v, uint32_t n) {
    while (n--) { ADDCHAR(*v); v++; }
}
void GetUnsignedCharAsUnsignedChar(uint8_t *v, uint32_t n, uint8_t max, uint8_t min) {
    if (rw == QB_Parser_Dir_WriteDel) {
        while (n--) { int8_t r = GETSC; int32_t t = *v; t += r;
            if (max<t) t=max; if (t<min) t=min; *v = (uint8_t)t; v++; }
    } else {
        while (n--) { uint8_t r = GETUC; *v = r;
            if (max<*v)*v=max; if (*v<min)*v=min; v++; }
    }
}

void AddUnsignedShortAsUnsignedShort(uint16_t *v, uint32_t n) {
    while (n--) { ADDUS(*v); v++; }
}
void GetUnsignedShortAsUnsignedShort(uint16_t *v, uint32_t n, uint16_t max, uint16_t min) {
    if (rw == QB_Parser_Dir_WriteDel) {
        while (n--) { uint16_t r = GETUS; *v += r; if (max<*v)*v=max; if (*v<min)*v=min; v++; }
    } else {
        while (n--) { uint16_t r = GETUS; *v  = r; if (max<*v)*v=max; if (*v<min)*v=min; v++; }
    }
}

void AddBitsAsByte(uint8_t *v, uint8_t start_bit, uint8_t n_bits) {
    uint8_t temp = *msgPtr, mask = 0;
    switch (n_bits) {
        case 1: mask=BITFIELD_MASK_1; break; case 2: mask=BITFIELD_MASK_2; break;
        case 3: mask=BITFIELD_MASK_3; break; case 4: mask=BITFIELD_MASK_4; break;
        case 5: mask=BITFIELD_MASK_5; break; case 6: mask=BITFIELD_MASK_6; break;
        case 7: mask=BITFIELD_MASK_7; break; case 8: mask=BITFIELD_MASK_8; break;
        default: break;
    }
    temp &= ~(mask << start_bit);
    temp |= ((*v & mask) << start_bit);
    *msgPtr = temp;
}

void GetBitsAsByte(uint8_t *v, uint8_t start_bit, uint8_t n_bits) {
    uint8_t mask = 0;
    switch (n_bits) {
        case 1: mask=BITFIELD_MASK_1; break; case 2: mask=BITFIELD_MASK_2; break;
        case 3: mask=BITFIELD_MASK_3; break; case 4: mask=BITFIELD_MASK_4; break;
        case 5: mask=BITFIELD_MASK_5; break; case 6: mask=BITFIELD_MASK_6; break;
        case 7: mask=BITFIELD_MASK_7; break; case 8: mask=BITFIELD_MASK_8; break;
        default: break;
    }
    if (rw == QB_Parser_Dir_WriteDel) {
        uint8_t xor_mask = (*msgPtr >> start_bit) & mask;
        *v ^= xor_mask;
    } else {
        *v = (*msgPtr >> start_bit) & mask;
    }
}

/* ============================================================
 * QX_Protocol implementations (verbatim from QX_Protocol.c)
 * ============================================================ */

static QX_Stat_e QX_InitMsg(QX_Msg_t *Msg_p)
{
    memset(Msg_p, 0, sizeof(QX_Msg_t));
    Msg_p->BufPayloadStart_p = NULL;
    Msg_p->MsgBufAtt_p       = NULL;
    Msg_p->MsgBuf_p          = NULL;
    return QX_STAT_OK;
}

static QX_Stat_e QX_RxMsg(QX_Msg_t *RxMsg_p)
{
    RxMsg_p->MsgBuf_p      = &RxMsg_p->MsgBuf[0];
    RxMsg_p->MsgBufStart_p = &RxMsg_p->MsgBuf[0];
    RxMsg_p->AttNotHandled = 0;

    if (RxMsg_p->Legacy_Header) {
        if (QX_ParseHeader_Legacy != NULL) QX_ParseHeader_Legacy(RxMsg_p);
        else return QX_STAT_ERROR;
    } else {
        QX_ParseHeader(RxMsg_p);
    }

    RxMsg_p->BufPayloadStart_p = RxMsg_p->MsgBuf_p;

    if (RxMsg_p->Header.AddCRC32) {
        uint32_t crc32_result = QX_accumulate_crc32(0xFFFFFFFF,
            RxMsg_p->MsgBuf, RxMsg_p->MsgBuf_MsgLen - 5);
        uint32_t crc32_msg = 0;
        memcpy(&crc32_msg, &RxMsg_p->MsgBuf[RxMsg_p->MsgBuf_MsgLen - 5], sizeof(crc32_msg));
        if (crc32_result != crc32_msg) return QX_STAT_ERROR_RXMSG_CRC32_FAIL;
    }

    QX_FwdMsg_CB(RxMsg_p);

    switch (RxMsg_p->Header.Type) {
        case QX_MSG_TYPE_READ:      QX_Srv_Rx_Read(RxMsg_p);    break;
        case QX_MSG_TYPE_WRITE_ABS:
        case QX_MSG_TYPE_WRITE_REL: QX_Srv_Rx_Write(RxMsg_p);  break;
        case QX_MSG_TYPE_CURVAL:    QX_Cli_Rx_CurVal(RxMsg_p);  break;
        default: return QX_STAT_ERROR_MSG_TYPE_NOT_SUPPORTED;
    }
    return QX_STAT_OK;
}

static void QX_Srv_Rx_Read(QX_Msg_t *RxMsg_p)
{
    int srv_i;
    for (srv_i = 0; srv_i < QX_NUM_SRV; srv_i++) {
        if ((RxMsg_p->Header.Target_Addr == QX_Servers[srv_i].Address) ||
            (RxMsg_p->Header.Target_Addr == QX_DEV_ID_BROADCAST)) {
            QX_TxMsgOptions_t options;
            QX_InitTxOptions(&options);
            options.use_CRC32           = RxMsg_p->Header.AddCRC32;
            options.FF_Ext              = RxMsg_p->Header.FF_Ext;
            options.Remove_Addr_Fields  = RxMsg_p->Header.Remove_Addr_Fields;
            options.Remove_Req_Fields   = 1;
            options.Target_Addr         = RxMsg_p->Header.Source_Addr;
            QX_SendPacket_Srv_CurVal(&QX_Servers[srv_i],
                RxMsg_p->Header.Attrib, RxMsg_p->CommPort, options);
        }
    }
}

static void QX_Srv_Rx_Write(QX_Msg_t *RxMsg_p)
{
    int srv_i;
    if      (RxMsg_p->Header.Type == QX_MSG_TYPE_WRITE_REL) RxMsg_p->Parse_Type = QX_PARSE_TYPE_WRITE_REL_RECV;
    else if (RxMsg_p->Header.Type == QX_MSG_TYPE_WRITE_ABS) RxMsg_p->Parse_Type = QX_PARSE_TYPE_WRITE_ABS_RECV;

    for (srv_i = 0; srv_i < QX_NUM_SRV; srv_i++) {
        if ((RxMsg_p->Header.Target_Addr == QX_Servers[srv_i].Address) ||
            (RxMsg_p->Header.Target_Addr == QX_DEV_ID_BROADCAST)) {
            QX_Servers[srv_i].Parser_CB(RxMsg_p);
            if (RxMsg_p->DisableStdResponse == 0) {
                QX_TxMsgOptions_t options;
                QX_InitTxOptions(&options);
                options.use_CRC32          = RxMsg_p->Header.AddCRC32;
                options.FF_Ext             = RxMsg_p->Header.FF_Ext;
                options.Remove_Addr_Fields = RxMsg_p->Header.Remove_Addr_Fields;
                options.Remove_Req_Fields  = 1;
                options.Target_Addr        = RxMsg_p->Header.Source_Addr;
                QX_SendPacket_Srv_CurVal(&QX_Servers[srv_i],
                    RxMsg_p->Header.Attrib, RxMsg_p->CommPort, options);
            }
        }
    }
}

static void QX_Cli_Rx_CurVal(QX_Msg_t *RxMsg_p)
{
    int cli_i;
    RxMsg_p->Parse_Type = QX_PARSE_TYPE_CURVAL_RECV;
    for (cli_i = 0; cli_i < QX_NUM_CLI; cli_i++)
        QX_Clients[cli_i].Parser_CB(RxMsg_p);
}

static QX_Stat_e QX_TxMsg_Setup(QX_Msg_t *TxMsg_p)
{
    TxMsg_p->MsgBufAtt_p = &TxMsg_p->MsgBuf[4];
    TxMsg_p->MsgBuf_p    = TxMsg_p->MsgBufAtt_p;
    if (TxMsg_p->Legacy_Header) {
        if (QX_BuildHeader_Legacy != NULL) QX_BuildHeader_Legacy(TxMsg_p);
        else return QX_STAT_ERROR;
    } else {
        QX_BuildHeader(TxMsg_p);
    }
    return QX_STAT_OK;
}

static QX_Stat_e QX_TxMsg_Finish(QX_Msg_t *TxMsg_p)
{
    uint8_t QX_use2ByteLen = 0;
    if (TxMsg_p->AttNotHandled == 1) return QX_STAT_ERROR_ATT_NOT_HANDLED;

    TxMsg_p->Header.MsgLength = (uint16_t)(TxMsg_p->MsgBuf_p - TxMsg_p->MsgBufAtt_p);

    if (TxMsg_p->Legacy_Header || (TxMsg_p->Header.MsgLength > 100)) {
        TxMsg_p->MsgBufStart_p = &TxMsg_p->MsgBuf[0];
        QX_use2ByteLen = 1;
    } else {
        TxMsg_p->MsgBufStart_p = &TxMsg_p->MsgBuf[1];
    }

    TxMsg_p->MsgBuf_MsgLen = (uint16_t)(TxMsg_p->MsgBuf_p - TxMsg_p->MsgBufStart_p);

    if (TxMsg_p->Header.AddCRC32) {
        uint32_t bytes_to_add = 4 - ((TxMsg_p->MsgBuf_MsgLen) % 4);
        memset(TxMsg_p->MsgBuf_p, 0, bytes_to_add);
        TxMsg_p->MsgBuf_p      += bytes_to_add;
        TxMsg_p->MsgBuf_MsgLen += (uint16_t)bytes_to_add;
        TxMsg_p->Header.MsgLength = (uint16_t)(TxMsg_p->MsgBuf_p - TxMsg_p->MsgBufAtt_p) + 4;
    }

    if (TxMsg_p->Legacy_Header) {
        TxMsg_p->MsgBuf[0] = 'Q';
        TxMsg_p->MsgBuf[1] = 'B';
        TxMsg_p->MsgBuf[2] = (TxMsg_p->Header.MsgLength >> 8) & 0xFF;
        TxMsg_p->MsgBuf[3] =  TxMsg_p->Header.MsgLength & 0xFF;
    } else {
        if (QX_use2ByteLen) {
            TxMsg_p->MsgBuf[0] = 'Q';
            TxMsg_p->MsgBuf[1] = 'X';
            TxMsg_p->MsgBuf[2] = (uint8_t)((TxMsg_p->Header.MsgLength & 0x7F) | 0x80);
            TxMsg_p->MsgBuf[3] = (uint8_t)((TxMsg_p->Header.MsgLength >> 7) & 0x7F);
        } else {
            TxMsg_p->MsgBuf[1] = 'Q';
            TxMsg_p->MsgBuf[2] = 'X';
            TxMsg_p->MsgBuf[3] = (uint8_t)(TxMsg_p->Header.MsgLength & 0x7F);
        }
    }

    if (TxMsg_p->Header.AddCRC32) {
        uint32_t crc = QX_accumulate_crc32(0xFFFFFFFF,
            TxMsg_p->MsgBufStart_p, TxMsg_p->MsgBuf_MsgLen);
        memcpy(TxMsg_p->MsgBuf_p, &crc, sizeof(crc));
        TxMsg_p->MsgBuf_p      += 4;
        TxMsg_p->MsgBuf_MsgLen  = (uint16_t)(TxMsg_p->MsgBuf_p - TxMsg_p->MsgBufStart_p);
    }

    uint8_t chksum = QX_Calc8bChecksum(TxMsg_p->MsgBufAtt_p, TxMsg_p->Header.MsgLength);
    *TxMsg_p->MsgBuf_p++ = 0xFF - chksum;
    TxMsg_p->MsgBuf_MsgLen++;

    QX_SendMsg2CommsPort_CB(TxMsg_p);
    return QX_STAT_OK;
}

static void QX_BuildHeader(QX_Msg_t *Msg_p)
{
    Msg_p->MsgBuf_p = Msg_p->MsgBufAtt_p;
    QX_AddExtdValToBuf(&Msg_p->MsgBuf_p, Msg_p->Header.Attrib);
    Msg_p->Header.AddOptionByte1 = 0;
    if (Msg_p->Header.AddCRC32) Msg_p->Header.AddOptionByte1 = 1;

    volatile uint8_t OptionByte = 0;
    OptionByte |= (Msg_p->Header.Type & 0xF);
    OptionByte |= (Msg_p->Header.FF_Ext            & 0x01) << 4;
    OptionByte |= (Msg_p->Header.Remove_Req_Fields  & 0x01) << 5;
    OptionByte |= (Msg_p->Header.Remove_Addr_Fields & 0x01) << 6;
    OptionByte |= (Msg_p->Header.AddOptionByte1    & 0x01) << 7;
    *Msg_p->MsgBuf_p++ = OptionByte;

    if (Msg_p->Header.AddOptionByte1) {
        OptionByte = 0;
        OptionByte |= (Msg_p->Header.AddCRC32 & 0x01) << 0;
        *Msg_p->MsgBuf_p++ = OptionByte;
    }

    if (Msg_p->Header.Remove_Addr_Fields == 0) {
        QX_AddExtdValToBuf(&Msg_p->MsgBuf_p, Msg_p->Header.Source_Addr);
        QX_AddExtdValToBuf(&Msg_p->MsgBuf_p, Msg_p->Header.Target_Addr);
    } else {
        Msg_p->Header.Source_Addr = QX_DEV_ID_BROADCAST;
        Msg_p->Header.Target_Addr = QX_DEV_ID_BROADCAST;
    }

    if (Msg_p->Header.Remove_Req_Fields == 0) {
        QX_AddExtdValToBuf(&Msg_p->MsgBuf_p, Msg_p->Header.TransReq_Addr);
        QX_AddExtdValToBuf(&Msg_p->MsgBuf_p, Msg_p->Header.RespReq_Addr);
    } else {
        Msg_p->Header.TransReq_Addr = QX_DEV_ID_BROADCAST;
        Msg_p->Header.RespReq_Addr  = QX_DEV_ID_BROADCAST;
    }

    Msg_p->BufPayloadStart_p = Msg_p->MsgBuf_p;
}

static void QX_ParseHeader(QX_Msg_t *Msg_p)
{
    volatile uint8_t OptionByte;
    Msg_p->MsgBuf_p = Msg_p->MsgBufAtt_p;

    Msg_p->Header.Attrib = QX_GetExtdValFromBuf(&Msg_p->MsgBuf_p);

    OptionByte = *Msg_p->MsgBuf_p++;
    Msg_p->Header.Type               = (QX_Msg_Type_e)(OptionByte & 0xF);
    Msg_p->Header.FF_Ext             = (OptionByte >> 4) & 0x1;
    Msg_p->Header.Remove_Req_Fields  = (OptionByte >> 5) & 0x1;
    Msg_p->Header.Remove_Addr_Fields = (OptionByte >> 6) & 0x1;
    Msg_p->Header.AddOptionByte1     = (OptionByte >> 7) & 0x1;

    if (Msg_p->Header.AddOptionByte1) {
        OptionByte = *Msg_p->MsgBuf_p++;
        Msg_p->Header.AddCRC32 = (OptionByte >> 0) & 0x1;
    }

    if (Msg_p->Header.Remove_Addr_Fields == 0) {
        Msg_p->Header.Source_Addr = (QX_DevId_e)(QX_GetExtdValFromBuf(&Msg_p->MsgBuf_p));
        Msg_p->Header.Target_Addr = (QX_DevId_e)(QX_GetExtdValFromBuf(&Msg_p->MsgBuf_p));
    }

    if (Msg_p->Header.Remove_Req_Fields == 0) {
        Msg_p->Header.TransReq_Addr = (QX_DevId_e)(QX_GetExtdValFromBuf(&Msg_p->MsgBuf_p));
        Msg_p->Header.RespReq_Addr  = (QX_DevId_e)(QX_GetExtdValFromBuf(&Msg_p->MsgBuf_p));
    }

    if (Msg_p->Header.FF_Ext) Msg_p->MsgBuf_p += 2;
}

static uint8_t QX_Calc8bChecksum(uint8_t *buf_p, uint32_t len)
{
    uint32_t j;
    uint8_t checksum = 0;
    for (j = 0; j < len; j++) checksum += *buf_p++;
    return checksum;
}

static void QX_AddExtdValToBuf(uint8_t **p, uint32_t Val)
{
    int n;
    for (n = 0; n < 4; n++) {
        uint8_t this_chunk = (uint8_t)((Val >> (7*n)) & 0x7F);
        uint8_t next_chunk = (uint8_t)((Val >> (7*(n+1))) & 0x7F);
        if (next_chunk > 0) { **p = this_chunk | 0x80; (*p)++; }
        else                { **p = this_chunk;         (*p)++; break; }
    }
}

static uint32_t QX_GetExtdValFromBuf(uint8_t **p)
{
    uint32_t Val = 0;
    int n;
    for (n = 0; n < 4; n++) {
        Val = (((uint32_t)(**p & 0x7F)) << (7*n)) | Val;
        if (**p & 0x80) (*p)++;
        else            { (*p)++; break; }
    }
    return Val;
}

static uint32_t QX_compute_crc32(uint8_t data)
{
    uint8_t i;
    uint32_t crc = (uint32_t)data << 24;
    for (i = 0; i < 8; i++) {
        if ((crc & 0x80000000) != 0) crc = (crc << 1) ^ 0x04C11DB7;
        else                          crc = (crc << 1);
    }
    return crc;
}

uint32_t QX_accumulate_crc32(uint32_t initial, const uint8_t *data, uint32_t size)
{
    uint32_t i, final = initial;
    for (i = 0; i < size; i++)
        final = QX_compute_crc32((uint8_t)(final >> 24) ^ data[i]) ^ (final << 8);
    return final;
}

/* ============================================================
 * Public functions
 * ============================================================ */

void QX_InitSrv(QX_Server_t *QX_Server, QX_DevId_e Address, uint8_t *(*Parser_CB)(QX_Msg_t *))
{
    QX_Server->Parser_CB = Parser_CB;
    QX_Server->Address   = Address;
}

void QX_InitCli(QX_Client_t *QX_Client, QX_DevId_e Address, uint8_t *(*Parser_CB)(QX_Msg_t *))
{
    QX_Client->Parser_CB = Parser_CB;
    QX_Client->Address   = Address;
}

void QX_InitTxOptions(QX_TxMsgOptions_t *options)
{
    options->Remove_Addr_Fields = 0;
    options->Remove_Req_Fields  = 0;
    options->RespReq_Addr       = QX_DEV_ID_BROADCAST;
    options->Target_Addr        = QX_DEV_ID_BROADCAST;
    options->TransReq_Addr      = QX_DEV_ID_BROADCAST;
    options->FF_Ext             = 0;
    options->use_CRC32          = 0;
}

void QX_Disable_Default_Response(QX_Msg_t *Msg_p) { Msg_p->DisableStdResponse = 1; }

void QX_Connection_Status_Update(QX_Comms_Port_e port)
{
    QX_CommsPorts[port].Timeout_Cntr =
        QX_GetTicks_ms() - QX_CommsPorts[port].last_rx_msg_time;
    if (QX_CommsPorts[port].Timeout_Cntr > QX_PORT_TIMEOUT_MSEC)
        QX_CommsPorts[port].Connected = 0;
}

QX_Stat_e QX_SendPacket_Srv_CurVal(QX_Server_t *Srv_p, uint32_t Attrib,
    QX_Comms_Port_e CommPort, QX_TxMsgOptions_t options)
{
    QX_Msg_t TxMsg; QX_InitMsg(&TxMsg);
    TxMsg.CommPort              = CommPort;
    TxMsg.Header.Attrib         = Attrib;
    TxMsg.Header.Type           = QX_MSG_TYPE_CURVAL;
    TxMsg.Header.Target_Addr    = options.Target_Addr;
    TxMsg.Header.TransReq_Addr  = options.TransReq_Addr;
    TxMsg.Header.RespReq_Addr   = options.RespReq_Addr;
    TxMsg.Header.Source_Addr    = Srv_p->Address;
    TxMsg.Header.FF_Ext         = options.FF_Ext;
    TxMsg.Header.AddCRC32       = options.use_CRC32;
    QX_TxMsg_Setup(&TxMsg);
    TxMsg.Parse_Type = QX_PARSE_TYPE_CURVAL_SEND;
    TxMsg.MsgBuf_p   = Srv_p->Parser_CB(&TxMsg);
    return QX_TxMsg_Finish(&TxMsg);
}

QX_Stat_e QX_SendPacket_Cli_Read(QX_Client_t *Cli_p, uint32_t Attrib,
    QX_Comms_Port_e CommPort, QX_TxMsgOptions_t options)
{
    QX_Msg_t TxMsg; QX_InitMsg(&TxMsg);
    TxMsg.CommPort             = CommPort;
    TxMsg.Header.Attrib        = Attrib;
    TxMsg.Header.Type          = QX_MSG_TYPE_READ;
    TxMsg.Header.Target_Addr   = options.Target_Addr;
    TxMsg.Header.TransReq_Addr = QX_DEV_ID_BROADCAST;
    TxMsg.Header.RespReq_Addr  = options.RespReq_Addr;
    TxMsg.Header.Source_Addr   = Cli_p->Address;
    TxMsg.Header.FF_Ext        = options.FF_Ext;
    TxMsg.Header.AddCRC32      = options.use_CRC32;
    QX_TxMsg_Setup(&TxMsg);
    return QX_TxMsg_Finish(&TxMsg);
}

QX_Stat_e QX_SendPacket_Cli_WriteABS(QX_Client_t *Cli_p, uint32_t Attrib,
    QX_Comms_Port_e CommPort, QX_TxMsgOptions_t options)
{
    QX_Msg_t TxMsg; QX_InitMsg(&TxMsg);
    TxMsg.CommPort             = CommPort;
    TxMsg.Header.Attrib        = Attrib;
    TxMsg.Header.Type          = QX_MSG_TYPE_WRITE_ABS;
    TxMsg.Header.Target_Addr   = options.Target_Addr;
    TxMsg.Header.TransReq_Addr = options.TransReq_Addr;
    TxMsg.Header.RespReq_Addr  = options.RespReq_Addr;
    TxMsg.Header.Source_Addr   = Cli_p->Address;
    TxMsg.Header.FF_Ext        = options.FF_Ext;
    TxMsg.Header.AddCRC32      = options.use_CRC32;
    QX_TxMsg_Setup(&TxMsg);
    TxMsg.Parse_Type = QX_PARSE_TYPE_WRITE_ABS_SEND;
    TxMsg.MsgBuf_p   = Cli_p->Parser_CB(&TxMsg);
    return QX_TxMsg_Finish(&TxMsg);
}

QX_Stat_e QX_SendPacket_Cli_WriteREL(QX_Client_t *Cli_p, uint32_t Attrib,
    QX_Comms_Port_e CommPort, QX_TxMsgOptions_t options)
{
    QX_Msg_t TxMsg; QX_InitMsg(&TxMsg);
    TxMsg.CommPort             = CommPort;
    TxMsg.Header.Attrib        = Attrib;
    TxMsg.Header.Type          = QX_MSG_TYPE_WRITE_REL;
    TxMsg.Header.Target_Addr   = options.Target_Addr;
    TxMsg.Header.TransReq_Addr = options.TransReq_Addr;
    TxMsg.Header.RespReq_Addr  = options.RespReq_Addr;
    TxMsg.Header.Source_Addr   = Cli_p->Address;
    TxMsg.Header.FF_Ext        = options.FF_Ext;
    TxMsg.Header.AddCRC32      = options.use_CRC32;
    QX_TxMsg_Setup(&TxMsg);
    TxMsg.Parse_Type = QX_PARSE_TYPE_WRITE_REL_SEND;
    TxMsg.MsgBuf_p   = Cli_p->Parser_CB(&TxMsg);
    return QX_TxMsg_Finish(&TxMsg);
}

QX_Stat_e QX_SendPacket_Control(QX_Client_t *Cli_p, uint32_t Attrib,
    QX_Comms_Port_e CommPort, QX_TxMsgOptions_t options)
{
    return QX_SendPacket_Cli_WriteABS(Cli_p, Attrib, CommPort, options);
}

uint8_t QX_StreamRxCharSM(QX_Comms_Port_e port, unsigned char rxbyte)
{
    uint8_t chksum;
    switch (QX_CommsPorts[port].RxState) {

        case QX_RX_STATE_START_WAIT:
            if (rxbyte == 'Q') {
                QX_CommsPorts[port].RxCntr          = 1;
                QX_CommsPorts[port].RxMsg.MsgBuf[0] = rxbyte;
                QX_CommsPorts[port].RxMsg.MsgBuf_p  = &QX_CommsPorts[port].RxMsg.MsgBuf[1];
                QX_CommsPorts[port].RxState         = QX_RX_STATE_GET_PROTOCOL_VER;
            } else {
                QX_CommsPorts[port].non_Q_cnt++;
            }
            break;

        case QX_RX_STATE_GET_PROTOCOL_VER:
            *QX_CommsPorts[port].RxMsg.MsgBuf_p++ = rxbyte;
            QX_CommsPorts[port].RxCntr++;
            if      (rxbyte == 'X') { QX_CommsPorts[port].RxMsg.Legacy_Header = 0; QX_CommsPorts[port].RxState = QX_RX_STATE_GET_QX_LEN0; }
            else if (rxbyte == 'B') { QX_CommsPorts[port].RxMsg.Legacy_Header = 1; QX_CommsPorts[port].RxState = QX_RX_STATE_GET_QB_LEN0; }
            else                    { QX_CommsPorts[port].RxState = QX_RX_STATE_START_WAIT; }
            break;

        case QX_RX_STATE_GET_QX_LEN0:
            *QX_CommsPorts[port].RxMsg.MsgBuf_p++ = rxbyte;
            QX_CommsPorts[port].RxCntr++;
            if (rxbyte & 0x80) {
                QX_CommsPorts[port].RxMsg.Header.MsgLength = (uint16_t)(rxbyte & ~0x80);
                QX_CommsPorts[port].RxState = QX_RX_STATE_GET_QX_LEN1;
            } else {
                QX_CommsPorts[port].RxMsg.Header.MsgLength  = (uint16_t)rxbyte;
                QX_CommsPorts[port].RxMsg.MsgBufAtt_p       = QX_CommsPorts[port].RxMsg.MsgBuf_p;
                QX_CommsPorts[port].RxMsg.RunningChecksum    = 0;
                QX_CommsPorts[port].RxState = QX_RX_STATE_GET_DATA;
                if (QX_MAX_PAYLOAD_LEN < QX_CommsPorts[port].RxMsg.Header.MsgLength)
                    QX_CommsPorts[port].RxState = QX_RX_STATE_START_WAIT;
            }
            break;

        case QX_RX_STATE_GET_QX_LEN1:
            *QX_CommsPorts[port].RxMsg.MsgBuf_p++ = rxbyte;
            QX_CommsPorts[port].RxCntr++;
            QX_CommsPorts[port].RxMsg.MsgBufAtt_p = QX_CommsPorts[port].RxMsg.MsgBuf_p;
            if (rxbyte & 0x80) {
                QX_CommsPorts[port].RxState = QX_RX_STATE_START_WAIT;
            } else {
                QX_CommsPorts[port].RxMsg.Header.MsgLength |= (((uint16_t)(rxbyte & ~0x80)) << 7);
                QX_CommsPorts[port].RxMsg.RunningChecksum    = 0;
                QX_CommsPorts[port].RxState = QX_RX_STATE_GET_DATA;
            }
            if (QX_MAX_PAYLOAD_LEN < QX_CommsPorts[port].RxMsg.Header.MsgLength)
                QX_CommsPorts[port].RxState = QX_RX_STATE_START_WAIT;
            break;

        case QX_RX_STATE_GET_QB_LEN0:
            *QX_CommsPorts[port].RxMsg.MsgBuf_p++ = rxbyte;
            QX_CommsPorts[port].RxCntr++;
            QX_CommsPorts[port].RxMsg.Header.MsgLength = (uint16_t)rxbyte << 8;
            QX_CommsPorts[port].RxState = QX_RX_STATE_GET_QB_LEN1;
            break;

        case QX_RX_STATE_GET_QB_LEN1:
            *QX_CommsPorts[port].RxMsg.MsgBuf_p++ = rxbyte;
            QX_CommsPorts[port].RxCntr++;
            QX_CommsPorts[port].RxMsg.MsgBufAtt_p       = QX_CommsPorts[port].RxMsg.MsgBuf_p;
            QX_CommsPorts[port].RxMsg.Header.MsgLength  |= (uint16_t)rxbyte;
            QX_CommsPorts[port].RxMsg.RunningChecksum    = 0;
            QX_CommsPorts[port].RxState = QX_RX_STATE_GET_DATA;
            if (QX_MAX_PAYLOAD_LEN < QX_CommsPorts[port].RxMsg.Header.MsgLength)
                QX_CommsPorts[port].RxState = QX_RX_STATE_START_WAIT;
            break;

        case QX_RX_STATE_GET_DATA:
            *QX_CommsPorts[port].RxMsg.MsgBuf_p++ = rxbyte;
            QX_CommsPorts[port].RxCntr++;
            QX_CommsPorts[port].RxMsg.RunningChecksum += rxbyte;
            if (QX_CommsPorts[port].RxMsg.Header.MsgLength <=
                QX_CommsPorts[port].RxCntr -
                (QX_CommsPorts[port].RxMsg.MsgBufAtt_p - &QX_CommsPorts[port].RxMsg.MsgBuf[0]))
                QX_CommsPorts[port].RxState = QX_RX_STATE_GET_CHKSUM;
            break;

        case QX_RX_STATE_GET_CHKSUM:
            *QX_CommsPorts[port].RxMsg.MsgBuf_p++ = rxbyte;
            QX_CommsPorts[port].RxCntr++;
            chksum = rxbyte;
            QX_CommsPorts[port].RxState      = QX_RX_STATE_START_WAIT;
            QX_CommsPorts[port].RxMsg.CommPort = port;
            if ((chksum + QX_CommsPorts[port].RxMsg.RunningChecksum) == 0xFF) {
                QX_CommsPorts[port].Timeout_Cntr       = 0;
                QX_CommsPorts[port].last_rx_msg_time   = QX_GetTicks_ms();
                QX_CommsPorts[port].Connected          = 1;
                QX_CommsPorts[port].RxMsg.MsgBuf_MsgLen = QX_CommsPorts[port].RxCntr;
                QX_RxMsg(&QX_CommsPorts[port].RxMsg);
                return 1;
            } else {
                QX_CommsPorts[port].ChkSumFail_cnt++;
            }
            break;
    }
    return 0;
}
