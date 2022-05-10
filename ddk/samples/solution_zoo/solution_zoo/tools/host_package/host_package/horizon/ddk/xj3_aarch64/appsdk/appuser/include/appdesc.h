/***************************************************************************
 *   Copyright (C) 2019 by horizon.                                        *
 *   zhaoqing.su@horizon.ai                                                *
 *                                                                         *
 *   appdesc header file.                                                  *
 *                                                                         *
 ***************************************************************************/
#ifndef APPDESC_H
#define APPDESC_H

#ifdef __CPLUSPLUS
extern "C" {
#endif

#include <errno.h>
#include <fcntl.h>
#include <linux/netlink.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#ifdef __CPLUSPLUS
}
#endif

#ifdef APPDESC_GLOBALS
#define APPDESC_EXT
#else
#define APPDESC_EXT extern
#endif

#define TX_TO_CLIENT_NUM_MAX 2000
typedef struct Send_to_Client_Type_ {
  uint32_t wDataLength;
  uint8_t Data[TX_TO_CLIENT_NUM_MAX];
} Tx_to_Client_Type;
APPDESC_EXT Tx_to_Client_Type Tx_to_Client_Map_Type;
APPDESC_EXT Tx_to_Client_Type Prd_Tx_DtcSts_to_Client;

typedef enum {
  kRxState_Idle = 0,
  kRxState_ApplInformed,
  kRxState_FrameReceived,
  kRxState_WaitCF,
  kRxState_WaitFC,
  kRxState_WaitForFCConfIsr,
  kRxState_Error
} tRxStateEngine;
#define RX_FROM_CLIENT_NUM_MAX 30
typedef struct Rx_From_Client_Type_ {
  tRxStateEngine engine;
  uint8_t msgID;
  uint32_t wDataLength;
  uint8_t Data[RX_FROM_CLIENT_NUM_MAX];
} Rx_From_Client_Type;
APPDESC_EXT Rx_From_Client_Type Rx_From_Client_Map_Type;
typedef enum ReqCmd_OpFnsh_Sts_ {
  FnshSts_Idle = 0,
  FnshSts_WrOp,
  FnshSts_RdOp,
  FnshSts_Oping,
  FnshSts_Fail,
  FnshSts_Ok
} ReqCmd_OpFnsh_Sts;
APPDESC_EXT ReqCmd_OpFnsh_Sts ClearDtc_Fnsh_Sts;
typedef enum Resp_WrRes_Sts_ {
  WrRes_NonOp = 0,
  WrRes_Oping,
  WrRes_OK,
  WrRes_Fail,
} Resp_WrRes_Sts;

typedef enum AppDesc_stEEPBlckIdx_ {
  AppDesc_NULLIdx = 0,
  AppDesc_VehConfigIdx,
  AppDesc_EndIdx
} AppDesc_stEEPBlckIdx;
typedef struct AppDesc_stILWithEEP_ {
  uint8_t prmVehConfig[10];  // F1A1
  ReqCmd_OpFnsh_Sts WrParm_Fnsh_Sts[AppDesc_EndIdx];
  Resp_WrRes_Sts WrParm_Res[AppDesc_EndIdx];
} AppDesc_stILWithEEP;
APPDESC_EXT AppDesc_stILWithEEP AppDesc_stgILWithEEP;

typedef enum {
  Fault_Status_Mask_Null = 0,
  Fault_Status_Mask_His,
  Fault_Status_Mask_Cur,
  Fault_Status_Mask_HisAndCur,
  Fault_Status_Mask_End
} Fault_Status_Mask_Tbl;

APPDESC_EXT void AppDesc_init(void);

/* Read DID service */
APPDESC_EXT void AppDesc_Read_DID0100_J2FaultSts_Summary(void);
APPDESC_EXT void AppDesc_Read_DIDF200_J2FaultRes_BitMask(void);

/* write DID service */
APPDESC_EXT void AppDesc_Write_DIDF1A1(void);

/* clear DTC service */
APPDESC_EXT void AppDescClearDTCInformation(uint8_t clear_type);

/* read DTC service */
APPDESC_EXT void AppDescReportNumberOfDTCByStatusMask(void);
APPDESC_EXT void AppDescReportDTCByStatusMask(void);
APPDESC_EXT void AppDescReportDTCSnapshotRecordByDTCNumber(void);
APPDESC_EXT void AppDescReportDTCExtendedDataRecordByDTCNumber(void);
APPDESC_EXT void AppDescReportSupportedDTC(void);
APPDESC_EXT void AppDesc_Task_Management(void);
APPDESC_EXT void App_Prd_Read_J2FaultRes_BitMask(Tx_to_Client_Type *dtc_sts);
#endif