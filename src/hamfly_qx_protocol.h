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
 *
 * v2 divergences from the Freefly original (2026-07-20):
 *  - F4 : added QX_ParserCtx_t (per-message parser state) and removed the
 *         `extern QB_Parser_Dir_e rw;` global. Every QX_Parser_* prototype,
 *         every Add and Get prototype and all 14 PARSE_x_AS_x macros gained a
 *         leading QX_ParserCtx_t* parameter. Macro NAMES are unchanged; only
 *         their arity grew, so a missed call site is a compile error rather
 *         than a silent bug. The public Parser_CB function-pointer type is
 *         deliberately UNCHANGED -- the context never crosses that boundary.
 *
 * Rationale: build/review/findings.md. NOT build-verified.
 */

#ifndef HAMFLY_QX_PROTOCOL_H
#define HAMFLY_QX_PROTOCOL_H

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "hamfly_qx_app.h"   /* supplies QX_App_Config types and QX_NUM_SRV/CLI */

/* ============================================================
 * Definitions
 * ============================================================ */
#define QX_MAX_MSG_LEN          (QX_MAX_OUTER_FRAME_LEN + QX_MAX_PAYLOAD_LEN)
#define QX_MAX_OUTER_FRAME_LEN  5
#define QX_MAX_PAYLOAD_LEN      64
#define QX_PORT_TIMEOUT_MSEC    2000

/* ============================================================
 * Enumerations
 * ============================================================ */
typedef enum {
    QX_RX_STATE_START_WAIT = 0,
    QX_RX_STATE_GET_PROTOCOL_VER,
    QX_RX_STATE_GET_QX_LEN0,
    QX_RX_STATE_GET_QX_LEN1,
    QX_RX_STATE_GET_QB_LEN0,
    QX_RX_STATE_GET_QB_LEN1,
    QX_RX_STATE_GET_DATA,
    QX_RX_STATE_GET_CHKSUM
} QX_Rx_State_e;

typedef enum {
    QX_DEV_ID_BROADCAST             = 0,
    QX_DEV_ID_WEDGE_LENS_CONTROLLER = 1,
    QX_DEV_ID_GIMBAL                = 2,
    QX_DEV_ID_GIMBAL_INT_FIZ        = 3,
    QX_DEV_ID_MOVI_API_CONTROLLER   = 10
} QX_DevId_e;

typedef enum {
    QX_MSG_TYPE_CURVAL    = 0,
    QX_MSG_TYPE_READ      = 1,
    QX_MSG_TYPE_WRITE_ABS = 2,
    QX_MSG_TYPE_WRITE_REL = 3
} QX_Msg_Type_e;

typedef enum {
    QX_PARSE_TYPE_CURVAL_SEND     = 0,
    QX_PARSE_TYPE_CURVAL_RECV     = 1,
    QX_PARSE_TYPE_WRITE_ABS_SEND  = 2,
    QX_PARSE_TYPE_WRITE_ABS_RECV  = 3,
    QX_PARSE_TYPE_WRITE_REL_SEND  = 4,
    QX_PARSE_TYPE_WRITE_REL_RECV  = 5
} QX_Parse_Type_e;

typedef enum {
    QX_STAT_OK                          = 0,
    QX_STAT_ERROR                       = 1,
    QX_STAT_ERROR_MSG_TYPE_NOT_SUPPORTED= 2,
    QX_STAT_ERROR_EXTENSION_FAILED      = 3,
    QX_STAT_ERROR_INVALID_EXTENSION     = 4,
    QX_STAT_ERROR_MSG_LENGTH_INVALID    = 5,
    QX_STAT_ERROR_KEY_REQUIRED          = 6,
    QX_STAT_ERROR_RXMSG_CRC32_FAIL      = 7,
    QX_STAT_ERROR_ATT_NOT_HANDLED       = 8
} QX_Stat_e;

/* ============================================================
 * Structs
 * ============================================================ */
typedef struct {
    uint8_t    FF_Ext;
    uint8_t    use_CRC32;
    uint8_t    Remove_Addr_Fields;
    uint8_t    Remove_Req_Fields;
    QX_DevId_e Target_Addr;
    QX_DevId_e TransReq_Addr;
    QX_DevId_e RespReq_Addr;
} QX_TxMsgOptions_t;

typedef struct {
    uint16_t      MsgLength;
    uint32_t      Attrib;
    QX_Msg_Type_e Type;
    uint8_t       AddOptionByte1;
    uint8_t       AddCRC32;
    uint8_t       FF_Ext;
    uint8_t       FF_Ext_R0;
    uint8_t       FF_Ext_R1;
    uint8_t       Remove_Addr_Fields;
    uint8_t       Remove_Req_Fields;
    QX_DevId_e    Source_Addr;
    QX_DevId_e    Target_Addr;
    QX_DevId_e    TransReq_Addr;
    QX_DevId_e    RespReq_Addr;
} QX_MsgHeader_t;

typedef struct {
    QX_Parse_Type_e  Parse_Type;
    uint8_t          DisableStdResponse;
    uint8_t          RunningChecksum;
    uint8_t          CRC32_Checksum;
    uint8_t          AttNotHandled;
    uint8_t          Legacy_Header;
    QX_Comms_Port_e  CommPort;
    QX_MsgHeader_t   Header;
    uint8_t          MsgBuf[QX_MAX_MSG_LEN];
    uint16_t         MsgBuf_MsgLen;
    uint8_t         *MsgBufStart_p;
    uint8_t         *MsgBufAtt_p;
    uint8_t         *BufPayloadStart_p;
    uint8_t         *MsgBuf_p;
} QX_Msg_t;

typedef struct {
    QX_Rx_State_e RxState;
    uint16_t      RxCntr;
    QX_Msg_t      RxMsg;
    uint32_t      Timeout_Cntr;
    uint8_t       Connected;
    uint32_t      ChkSumFail_cnt;
    uint32_t      non_Q_cnt;
    uint32_t      last_rx_msg_time;
} QX_CommsPort_t;

typedef struct {
    QX_DevId_e  Address;
    uint8_t    *(*Parser_CB)(QX_Msg_t *);
} QX_Server_t;

typedef struct {
    QX_DevId_e  Address;
    uint8_t    *(*Parser_CB)(QX_Msg_t *);
} QX_Client_t;

/* ============================================================
 * Globals (defined in qx_protocol.c)
 * ============================================================ */
extern QX_CommsPort_t QX_CommsPorts[QX_NUM_OF_PORTS];
extern QX_Server_t    QX_Servers[QX_NUM_SRV];
extern QX_Client_t    QX_Clients[QX_NUM_CLI];

/* ============================================================
 * Functions
 * ============================================================ */
void      QX_InitSrv        (QX_Server_t *, QX_DevId_e, uint8_t *(*)(QX_Msg_t *));
void      QX_InitCli        (QX_Client_t *, QX_DevId_e, uint8_t *(*)(QX_Msg_t *));
uint8_t   QX_StreamRxCharSM (QX_Comms_Port_e port, unsigned char rxbyte);
void      QX_InitTxOptions  (QX_TxMsgOptions_t *options);
QX_Stat_e QX_SendPacket_Srv_CurVal   (QX_Server_t *, uint32_t, QX_Comms_Port_e, QX_TxMsgOptions_t);
QX_Stat_e QX_SendPacket_Cli_Read     (QX_Client_t *, uint32_t, QX_Comms_Port_e, QX_TxMsgOptions_t);
QX_Stat_e QX_SendPacket_Cli_WriteABS (QX_Client_t *, uint32_t, QX_Comms_Port_e, QX_TxMsgOptions_t);
QX_Stat_e QX_SendPacket_Cli_WriteREL (QX_Client_t *, uint32_t, QX_Comms_Port_e, QX_TxMsgOptions_t);
QX_Stat_e QX_SendPacket_Control      (QX_Client_t *, uint32_t, QX_Comms_Port_e, QX_TxMsgOptions_t);
void      QX_Disable_Default_Response(QX_Msg_t *);
void      QX_Connection_Status_Update(QX_Comms_Port_e port);

/* Application callbacks -- implemented in qx_app.c */
extern void     QX_SendMsg2CommsPort_CB(QX_Msg_t *);
extern void     QX_FwdMsg_CB           (QX_Msg_t *);
extern uint32_t QX_GetTicks_ms         (void);
extern uint32_t QX_accumulate_crc32    (uint32_t, const uint8_t *, uint32_t);

extern void (*QX_BuildHeader_Legacy)(QX_Msg_t *);
extern void (*QX_ParseHeader_Legacy)(QX_Msg_t *);

/* ============================================================
 * Parsing macros (from QX_Parsing_Functions.h)
 * ============================================================ */
typedef enum {
    QB_Parser_Dir_Read,
    QB_Parser_Dir_WriteDel,
    QB_Parser_Dir_WriteAbs
} QB_Parser_Dir_e;

/* Per-message parser context. Replaces the old file-scope rw/msgPtr/rw_orig
 * globals (F4). One instance is stack-allocated by whichever function drives
 * a single QX message through the Add and Get primitives below -- currently
 * only QX_ParsePacket_Cli_MoVI_Ctrl_CB in hamfly_qx_app.c. Zero-init is a
 * valid starting state (rw == QB_Parser_Dir_Read). Never share one instance
 * across two in-flight messages / call stacks. */
typedef struct {
    volatile uint8_t *msgPtr;
    QB_Parser_Dir_e   rw;
    QB_Parser_Dir_e   rw_orig;
} QX_ParserCtx_t;

void QX_Parser_SetMsgPtr           (QX_ParserCtx_t *ctx, uint8_t *p);
void QX_Parser_AdvMsgPtr           (QX_ParserCtx_t *ctx);
volatile uint8_t *QX_Parser_GetMsgPtr(QX_ParserCtx_t *ctx);
void QX_Parser_SetDir_Read         (QX_ParserCtx_t *ctx);
void QX_Parser_SetDir_WriteRel     (QX_ParserCtx_t *ctx);
void QX_Parser_SetDir_WriteAbs     (QX_ParserCtx_t *ctx);
QB_Parser_Dir_e QX_Parser_GetDir   (QX_ParserCtx_t *ctx);
void QX_Parser_Dir_ForceWriteAbs_Set  (QX_ParserCtx_t *ctx);
void QX_Parser_Dir_ForceWriteAbs_Reset(QX_ParserCtx_t *ctx);

/* Float parsers */
void AddFloatAsSignedLong    (QX_ParserCtx_t *, float *, uint32_t, float);
void AddFloatAsSignedShort   (QX_ParserCtx_t *, float *, uint32_t, float);
void AddFloatAsSignedChar    (QX_ParserCtx_t *, float *, uint32_t, float);
void AddFloatAsUnsignedChar  (QX_ParserCtx_t *, float *, uint32_t, float);
void AddFloatAsUnsignedShort (QX_ParserCtx_t *, float *, uint32_t, float);
void GetFloatAsSignedLong    (QX_ParserCtx_t *, float *, uint32_t, float, float, float);
void GetFloatAsSignedShort   (QX_ParserCtx_t *, float *, uint32_t, float, float, float);
void GetFloatAsSignedChar    (QX_ParserCtx_t *, float *, uint32_t, float, float, float);
void GetFloatAsUnsignedChar  (QX_ParserCtx_t *, float *, uint32_t, float, float, float);
void GetFloatAsUnsignedShort (QX_ParserCtx_t *, float *, uint32_t, float, float, float);

/* Signed long parsers */
void AddSignedLongAsSignedLong  (QX_ParserCtx_t *, int32_t *, uint32_t);
void AddSignedLongAsSignedShort (QX_ParserCtx_t *, int32_t *, uint32_t);
void AddSignedLongAsSignedChar  (QX_ParserCtx_t *, int32_t *, uint32_t);
void AddSignedLongAsUnsignedChar(QX_ParserCtx_t *, int32_t *, uint32_t);
void GetSignedLongAsSignedLong  (QX_ParserCtx_t *, int32_t *, uint32_t, int32_t, int32_t);
void GetSignedLongAsSignedShort (QX_ParserCtx_t *, int32_t *, uint32_t, int32_t, int32_t);
void GetSignedLongAsSignedChar  (QX_ParserCtx_t *, int32_t *, uint32_t, int32_t, int32_t);
void GetSignedLongAsUnsignedChar(QX_ParserCtx_t *, int32_t *, uint32_t, int32_t, int32_t);

/* Signed short parsers */
void AddSignedShortAsSignedShort (QX_ParserCtx_t *, int16_t *, uint32_t);
void AddSignedShortAsSignedChar  (QX_ParserCtx_t *, int16_t *, uint32_t);
void AddSignedShortAsUnsignedChar(QX_ParserCtx_t *, int16_t *, uint32_t);
void GetSignedShortAsSignedShort (QX_ParserCtx_t *, int16_t *, uint32_t, float, float);
void GetSignedShortAsSignedChar  (QX_ParserCtx_t *, int16_t *, uint32_t, float, float);
void GetSignedShortAsUnsignedChar(QX_ParserCtx_t *, int16_t *, uint32_t, int16_t, int16_t);

/* Signed/unsigned char parsers */
void AddSignedCharAsSignedChar    (QX_ParserCtx_t *, int8_t  *, uint32_t);
void GetSignedCharAsSignedChar    (QX_ParserCtx_t *, int8_t  *, uint32_t, int8_t,  int8_t);
void AddUnsignedCharAsUnsignedChar(QX_ParserCtx_t *, uint8_t *, uint32_t);
void GetUnsignedCharAsUnsignedChar(QX_ParserCtx_t *, uint8_t *, uint32_t, uint8_t, uint8_t);

/* Unsigned short parsers */
void AddUnsignedShortAsUnsignedShort(QX_ParserCtx_t *, uint16_t *, uint32_t);
void GetUnsignedShortAsUnsignedShort(QX_ParserCtx_t *, uint16_t *, uint32_t, uint16_t, uint16_t);

/* Bit field parsers */
void AddBitsAsByte(QX_ParserCtx_t *, uint8_t *, uint8_t start_bit, uint8_t n_bits);
void GetBitsAsByte(QX_ParserCtx_t *, uint8_t *, uint8_t start_bit, uint8_t n_bits);

/* Macros -- all now take an explicit ctx as the first argument. Every call
 * site must be updated (compile error otherwise: wrong macro arg count),
 * which makes a missed migration site unrepresentable rather than a latent
 * bug. */
#define PARSE_FL_AS_SL(ctx,v,n,mx,mn,sc) \
    if((ctx)->rw==QB_Parser_Dir_Read){AddFloatAsSignedLong(ctx,v,n,sc);}else{GetFloatAsSignedLong(ctx,v,n,mx,mn,1.0f/sc);}
#define PARSE_FL_AS_SS(ctx,v,n,mx,mn,sc) \
    if((ctx)->rw==QB_Parser_Dir_Read){AddFloatAsSignedShort(ctx,v,n,sc);}else{GetFloatAsSignedShort(ctx,v,n,mx,mn,1.0f/sc);}
#define PARSE_FL_AS_SC(ctx,v,n,mx,mn,sc) \
    if((ctx)->rw==QB_Parser_Dir_Read){AddFloatAsSignedChar(ctx,v,n,sc);}else{GetFloatAsSignedChar(ctx,v,n,mx,mn,1.0f/sc);}
#define PARSE_FL_AS_UC(ctx,v,n,mx,mn,sc) \
    if((ctx)->rw==QB_Parser_Dir_Read){AddFloatAsUnsignedChar(ctx,v,n,sc);}else{GetFloatAsUnsignedChar(ctx,v,n,mx,mn,1.0f/sc);}
#define PARSE_FL_AS_US(ctx,v,n,mx,mn,sc) \
    if((ctx)->rw==QB_Parser_Dir_Read){AddFloatAsUnsignedShort(ctx,v,n,sc);}else{GetFloatAsUnsignedShort(ctx,v,n,mx,mn,1.0f/sc);}
#define PARSE_SL_AS_SL(ctx,v,n,mx,mn) \
    if((ctx)->rw==QB_Parser_Dir_Read){AddSignedLongAsSignedLong(ctx,(int32_t*)v,n);}else{GetSignedLongAsSignedLong(ctx,(int32_t*)v,n,mx,mn);}
#define PARSE_SL_AS_SS(ctx,v,n,mx,mn) \
    if((ctx)->rw==QB_Parser_Dir_Read){AddSignedLongAsSignedShort(ctx,(int32_t*)v,n);}else{GetSignedLongAsSignedShort(ctx,(int32_t*)v,n,mx,mn);}
#define PARSE_SL_AS_SC(ctx,v,n,mx,mn) \
    if((ctx)->rw==QB_Parser_Dir_Read){AddSignedLongAsSignedChar(ctx,(int32_t*)v,n);}else{GetSignedLongAsSignedChar(ctx,(int32_t*)v,n,mx,mn);}
#define PARSE_SL_AS_UC(ctx,v,n,mx,mn) \
    if((ctx)->rw==QB_Parser_Dir_Read){AddSignedLongAsUnsignedChar(ctx,(int32_t*)v,n);}else{GetSignedLongAsUnsignedChar(ctx,(int32_t*)v,n,mx,mn);}
#define PARSE_SS_AS_SS(ctx,v,n,mx,mn) \
    if((ctx)->rw==QB_Parser_Dir_Read){AddSignedShortAsSignedShort(ctx,v,n);}else{GetSignedShortAsSignedShort(ctx,v,n,mx,mn);}
#define PARSE_SS_AS_SC(ctx,v,n,mx,mn) \
    if((ctx)->rw==QB_Parser_Dir_Read){AddSignedShortAsSignedChar(ctx,v,n);}else{GetSignedShortAsSignedChar(ctx,v,n,mx,mn);}
#define PARSE_SS_AS_UC(ctx,v,n,mx,mn) \
    if((ctx)->rw==QB_Parser_Dir_Read){AddSignedShortAsUnsignedChar(ctx,v,n);}else{GetSignedShortAsUnsignedChar(ctx,v,n,mx,mn);}
#define PARSE_SC_AS_SC(ctx,v,n,mx,mn) \
    if((ctx)->rw==QB_Parser_Dir_Read){AddSignedCharAsSignedChar(ctx,v,n);}else{GetSignedCharAsSignedChar(ctx,v,n,mx,mn);}
#define PARSE_UC_AS_UC(ctx,v,n,mx,mn) \
    if((ctx)->rw==QB_Parser_Dir_Read){AddUnsignedCharAsUnsignedChar(ctx,v,n);}else{GetUnsignedCharAsUnsignedChar(ctx,v,n,mx,mn);}
#define PARSE_US_AS_US(ctx,v,n,mx,mn) \
    if((ctx)->rw==QB_Parser_Dir_Read){AddUnsignedShortAsUnsignedShort(ctx,v,n);}else{GetUnsignedShortAsUnsignedShort(ctx,v,n,mx,mn);}
#define PARSE_BITS_AS_UC(ctx,v,sb,nb) \
    if((ctx)->rw==QB_Parser_Dir_Read){AddBitsAsByte(ctx,v,sb,nb);}else{GetBitsAsByte(ctx,v,sb,nb);}

#endif /* HAMFLY_QX_PROTOCOL_H */
