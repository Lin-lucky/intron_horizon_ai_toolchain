/***************************************************************************
 *   Copyright (C) 2019 by horizon.                                        *
 *   zhaoqing.su@horizon.ai                                                *
 *                                                                         *
 *   desc header file.                                                     *
 *                                                                         *
 ***************************************************************************/
#ifndef DESC_H
#define DESC_H

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

#ifdef DESC_GLOBALS
#define DESC_EXT
#else
#define DESC_EXT extern
#endif

typedef void (*DescFuncType)(void);

#define DESC_ECU_HARD_REST 0x01
/* define Clear DTC Information DTC group */
#define DESC_CLEAR_ALL_DTC_INFORMATION (0xFFFF)
#define DESC_CLEAR_HIS_DTC_INFORMATION (0xFFFF)

#define DESC_NEGATIVE_RESPONSE_SERVICE_ID (0x7f)

#define kDescNrcGeneralReject (0x10)
#define kDescNrcServiceNotSupported (0x11)
#define kDescNrcSubfunctionNotSupported (0x12)
#define kDescNrcIncorrectMessageLengthOrInvalidFormat (0x13)
#define kDescNrcBusyRepeatRequest (0x21)
#define kDescNrcConditionsNotCorrect (0x22)
#define kDescNrcRequestSequenceError (0x24)
#define kDescNrcRequestOutOfRange (0x31)
#define kDescNrcAccessDenied (0x33)
#define kDescNrcInvalidKey (0x35)
#define kDescNrcExceedNumOfAttempts (0x36)
#define kDescNrcTimeDelayNotExpired (0x37)
#define kDescNrcUploadDownloadNotAccepted (0x70)
#define kDescNrcTransferDataSuspended (0x71)
#define kDescNrcGeneralProgrammingFailure (0x72)
#define kDescNrcWrongBlockSequenceCounter (0x73)
#define kDescNrcResponsePending (0x78)
#define kDescNrcSubfunctionNotSupportedInActiveSession (0x7E)
#define kDescNrcServiceNotSupportedInActiveSession (0x7F)
#define kDescNrcRpmTooHigh (0x81)
#define kDescNrcRpmTooLow (0x82)
#define kDescNrcEngineIsRunning (0x83)
#define kDescNrcEngineIsNotRunning (0x84)
#define kDescNrcEngineRunTimeTooLow (0x85)
#define kDescNrcTemperatureTooHigh (0x86)
#define kDescNrcTemperatureTooLow (0x87)
#define kDescNrcVehicleSpeedTooHigh (0x88)
#define kDescNrcVehicleSpeedTooLow (0x89)
#define kDescNrcThrottleSpeedTooHigh (0x8A)
#define kDescNrcThrottleSpeedTooLow (0x8B)
#define kDescNrcTransmissionRangeInNeutral (0x8C)
#define kDescNrcTransmissionRangeInGears (0x8D)
#define kDescNrcBrakeSwitchNotClosed (0x8F)
#define kDescNrcShifterLeverNotInPark (0x90)
#define kDescNrcTorqueConverterClutchLocked (0x91)
#define kDescNrcVoltageTooHigh (0x92)
#define kDescNrcVoltageTooLow (0x93)

#define DTCStatusAvailabilityMask 0x7B /*support DTC status bit 0,1,3,4,5,6*/
#define DTCFormatIdentifierISO15031_64DTCFormat 0
#define DTCFormatIdentifierISO14229_1DTCFormat 1

typedef struct Desc_stLngTmRspInfo_ {
  uint8_t u1RspSID;
  uint8_t u1DID0;
  uint8_t u1DID1;
} Desc_stLngTmRspInfo;
DESC_EXT Desc_stLngTmRspInfo Desc_stgLngTmRspInfo;

DESC_EXT void Desc_init(void);
DESC_EXT void DescSendPosResp(void);
DESC_EXT void DescSendNegResp(uint8_t cRespCode);
DESC_EXT void DescEcuRest(void);
DESC_EXT void DescReadDataByIdentifier(void);
DESC_EXT void DescWriteDataByIdentifier(void);
DESC_EXT void DescClearDiagnosticInformation(void);
DESC_EXT void DescClearDiagnosticHisInformation(void);
DESC_EXT void DescReadDTCInformation(void);
DESC_EXT void Desc_Task_Management(void);
#endif