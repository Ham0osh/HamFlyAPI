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

extern QB_Parser_Dir_e rw;

void QX_Parser_SetMsgPtr           (uint8_t *p);
void QX_Parser_AdvMsgPtr           (void);
volatile uint8_t *QX_Parser_GetMsgPtr(void);
void QX_Parser_SetDir_Read         (void);
void QX_Parser_SetDir_WriteRel     (void);
void QX_Parser_SetDir_WriteAbs     (void);
QB_Parser_Dir_e QX_Parser_GetDir   (void);
void QX_Parser_Dir_ForceWriteAbs_Set  (void);
void QX_Parser_Dir_ForceWriteAbs_Reset(void);

/* Float parsers */
void AddFloatAsSignedLong    (float *, uint32_t, float);
void AddFloatAsSignedShort   (float *, uint32_t, float);
void AddFloatAsSignedChar    (float *, uint32_t, float);
void AddFloatAsUnsignedChar  (float *, uint32_t, float);
void AddFloatAsUnsignedShort (float *, uint32_t, float);
void GetFloatAsSignedLong    (float *, uint32_t, float, float, float);
void GetFloatAsSignedShort   (float *, uint32_t, float, float, float);
void GetFloatAsSignedChar    (float *, uint32_t, float, float, float);
void GetFloatAsUnsignedChar  (float *, uint32_t, float, float, float);
void GetFloatAsUnsignedShort (float *, uint32_t, float, float, float);

/* Signed long parsers */
void AddSignedLongAsSignedLong  (int32_t *, uint32_t);
void AddSignedLongAsSignedShort (int32_t *, uint32_t);
void AddSignedLongAsSignedChar  (int32_t *, uint32_t);
void AddSignedLongAsUnsignedChar(int32_t *, uint32_t);
void GetSignedLongAsSignedLong  (int32_t *, uint32_t, int32_t, int32_t);
void GetSignedLongAsSignedShort (int32_t *, uint32_t, int32_t, int32_t);
void GetSignedLongAsSignedChar  (int32_t *, uint32_t, int32_t, int32_t);
void GetSignedLongAsUnsignedChar(int32_t *, uint32_t, int32_t, int32_t);

/* Signed short parsers */
void AddSignedShortAsSignedShort (int16_t *, uint32_t);
void AddSignedShortAsSignedChar  (int16_t *, uint32_t);
void AddSignedShortAsUnsignedChar(int16_t *, uint32_t);
void GetSignedShortAsSignedShort (int16_t *, uint32_t, float, float);
void GetSignedShortAsSignedChar  (int16_t *, uint32_t, float, float);
void GetSignedShortAsUnsignedChar(int16_t *, uint32_t, int16_t, int16_t);

/* Signed/unsigned char parsers */
void AddSignedCharAsSignedChar    (int8_t  *, uint32_t);
void GetSignedCharAsSignedChar    (int8_t  *, uint32_t, int8_t,  int8_t);
void AddUnsignedCharAsUnsignedChar(uint8_t *, uint32_t);
void GetUnsignedCharAsUnsignedChar(uint8_t *, uint32_t, uint8_t, uint8_t);

/* Unsigned short parsers */
void AddUnsignedShortAsUnsignedShort(uint16_t *, uint32_t);
void GetUnsignedShortAsUnsignedShort(uint16_t *, uint32_t, uint16_t, uint16_t);

/* Bit field parsers */
void AddBitsAsByte(uint8_t *, uint8_t start_bit, uint8_t n_bits);
void GetBitsAsByte(uint8_t *, uint8_t start_bit, uint8_t n_bits);

/* Macros */
#define PARSE_FL_AS_SL(v,n,mx,mn,sc) \
    if(rw==QB_Parser_Dir_Read){AddFloatAsSignedLong(v,n,sc);}else{GetFloatAsSignedLong(v,n,mx,mn,1.0f/sc);}
#define PARSE_FL_AS_SS(v,n,mx,mn,sc) \
    if(rw==QB_Parser_Dir_Read){AddFloatAsSignedShort(v,n,sc);}else{GetFloatAsSignedShort(v,n,mx,mn,1.0f/sc);}
#define PARSE_FL_AS_SC(v,n,mx,mn,sc) \
    if(rw==QB_Parser_Dir_Read){AddFloatAsSignedChar(v,n,sc);}else{GetFloatAsSignedChar(v,n,mx,mn,1.0f/sc);}
#define PARSE_FL_AS_UC(v,n,mx,mn,sc) \
    if(rw==QB_Parser_Dir_Read){AddFloatAsUnsignedChar(v,n,sc);}else{GetFloatAsUnsignedChar(v,n,mx,mn,1.0f/sc);}
#define PARSE_FL_AS_US(v,n,mx,mn,sc) \
    if(rw==QB_Parser_Dir_Read){AddFloatAsUnsignedShort(v,n,sc);}else{GetFloatAsUnsignedShort(v,n,mx,mn,1.0f/sc);}
#define PARSE_SL_AS_SL(v,n,mx,mn) \
    if(rw==QB_Parser_Dir_Read){AddSignedLongAsSignedLong((int32_t*)v,n);}else{GetSignedLongAsSignedLong((int32_t*)v,n,mx,mn);}
#define PARSE_SL_AS_SS(v,n,mx,mn) \
    if(rw==QB_Parser_Dir_Read){AddSignedLongAsSignedShort((int32_t*)v,n);}else{GetSignedLongAsSignedShort((int32_t*)v,n,mx,mn);}
#define PARSE_SL_AS_SC(v,n,mx,mn) \
    if(rw==QB_Parser_Dir_Read){AddSignedLongAsSignedChar((int32_t*)v,n);}else{GetSignedLongAsSignedChar((int32_t*)v,n,mx,mn);}
#define PARSE_SL_AS_UC(v,n,mx,mn) \
    if(rw==QB_Parser_Dir_Read){AddSignedLongAsUnsignedChar((int32_t*)v,n);}else{GetSignedLongAsUnsignedChar((int32_t*)v,n,mx,mn);}
#define PARSE_SS_AS_SS(v,n,mx,mn) \
    if(rw==QB_Parser_Dir_Read){AddSignedShortAsSignedShort(v,n);}else{GetSignedShortAsSignedShort(v,n,mx,mn);}
#define PARSE_SS_AS_SC(v,n,mx,mn) \
    if(rw==QB_Parser_Dir_Read){AddSignedShortAsSignedChar(v,n);}else{GetSignedShortAsSignedChar(v,n,mx,mn);}
#define PARSE_SS_AS_UC(v,n,mx,mn) \
    if(rw==QB_Parser_Dir_Read){AddSignedShortAsUnsignedChar(v,n);}else{GetSignedShortAsUnsignedChar(v,n,mx,mn);}
#define PARSE_SC_AS_SC(v,n,mx,mn) \
    if(rw==QB_Parser_Dir_Read){AddSignedCharAsSignedChar(v,n);}else{GetSignedCharAsSignedChar(v,n,mx,mn);}
#define PARSE_UC_AS_UC(v,n,mx,mn) \
    if(rw==QB_Parser_Dir_Read){AddUnsignedCharAsUnsignedChar(v,n);}else{GetUnsignedCharAsUnsignedChar(v,n,mx,mn);}
#define PARSE_US_AS_US(v,n,mx,mn) \
    if(rw==QB_Parser_Dir_Read){AddUnsignedShortAsUnsignedShort(v,n);}else{GetUnsignedShortAsUnsignedShort(v,n,mx,mn);}
#define PARSE_BITS_AS_UC(v,sb,nb) \
    if(rw==QB_Parser_Dir_Read){AddBitsAsByte(v,sb,nb);}else{GetBitsAsByte(v,sb,nb);}

#endif /* HAMFLY_QX_PROTOCOL_H */
