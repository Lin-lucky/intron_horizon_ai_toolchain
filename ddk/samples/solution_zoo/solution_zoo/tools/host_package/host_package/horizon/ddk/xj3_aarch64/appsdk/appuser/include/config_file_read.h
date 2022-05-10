/***************************************************************************
 *   Copyright (C) 2019 by horizon.                                        *
 *   zhaoqing.su@horizon.ai                                                *
 *                                                                         *
 *   config_file_read header file.                                         *
 *                                                                         *
 ***************************************************************************/
#ifndef CONFIG_FILE_READ_H
#define CONFIG_FILE_READ_H

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

#include <map>

#ifdef CONFIG_FILE_READ_GLOBALS
#define CONFIG_FILE_READ_EXT
#else
#define CONFIG_FILE_READ_EXT extern
#endif

#define DTC_ERROR_SOC_WORKSTATUS 1U
#define ENV_DATA_MAX_NUM_MALLOC 0x8000  // 0x100000
#define BYTE_TO_BIT_NUM 8               // 1bit to 1dtc

#define BITMASK_BYTE_NUM_DEFAULT 2
#define SNAPSHOT_DATA_LEN_DEFAULT 20
#define SELFTEST_SNAPSHOT_DATA_LEN_DEFAULT 32768
#define STABLE_TIME_DEFAUL 100  // 100ms

#define KERNEL_DTC_ID_BASE_DEFAULT 0x0001
#define KERNEL_MODULE_ID_MIN_DEFAULT 0x0001
#define KERNEL_MODULE_ID_MAX_DEFAULT 0x7FFF

#define MODEL_DTC_ID_BASE_DEFUALT 0x1001
#define MODEL_MODULE_ID_MIN_DEFAULT 0x8001
#define MODEL_MODULE_ID_MAX_DEFAULT 0x8FFF

#define PERCEPTION_DTC_ID_BASE_DEFAULT 0x2001
#define PERCEPTION_MODULE_ID_MIN_DEFAULT 0x9001
#define PERCEPTION_MODULE_ID_MAX_DEFAULT 0x9FFF

#define ADAS_DTC_ID_BASE_DEFAULT 0x3001
#define ADAS_MODULE_ID_MIN_DEFAULT 0xA001
#define ADAS_MODULE_ID_MAX_DEFAULT 0xAFFF

#define AP_DTC_ID_BASE_DEFAUTL 0x4001
#define AP_MODULE_ID_MIN_DEFAULT 0xB001
#define AP_MODULE_ID_MAX_DEFAULT 0xBFFF

#define GENERAL_DTC_ID_BASE_DEFAULT 0xE001
#define GENERAL_MODULE_ID_MIN_DEFAULT 0xF001
#define GENERAL_MODULE_ID_MAX_DEFAULT 0xFFFE

#define SELFTEST_DTC_ID_BASE_DEFAULT 0xF001
#define SELFTEST_MODULE_ID_MIN_DEFAULT 0xFFFF
#define SELFTEST_MODULE_ID_MAX_DEFAULT 0xFFFF

#define SELFTEST_KERNEL_DTC_ID_DEFAULT 0xF002
#define SELFTEST_APP_DTC_ID_DEFAULT 0xF003

#define BITMASK_REPORT_ID SELFTEST_DTC_ID_BASE_DEFAULT
#define KERNEL_SELFTEST_ID (SELFTEST_DTC_ID_BASE_DEFAULT + 1)
#define APP_SELFTEST_ID (SELFTEST_DTC_ID_BASE_DEFAULT + 2)

#define ENV_ID_OCCUP_BYTE_NUM 2
#define ENV_LEN_OCCUP_BYTE_NUM 2
#define STS_LEN_OCCUP_BYTE_NUM 1
#define NUM_OF_ENV_ID_OCCUP_BYTE_NUM 1

typedef struct Gen_Dtc_id_Type_ {
  uint16_t dtc_id;
} Gen_Dtc_id_Type;
typedef std::map<uint16_t, Gen_Dtc_id_Type> GEN_DTC_ID_MAP;
CONFIG_FILE_READ_EXT GEN_DTC_ID_MAP diag_gen_dtc_id_map;

typedef struct DTC_Status_Type_ {
  uint16_t dtc_id_base;
  uint8_t dtc_sts_len;
  uint8_t *his_dtc_status;
  uint8_t *cur_dtc_status;
} DTC_Status_Type;
typedef std::map<uint16_t, DTC_Status_Type> DTC_STATUS_MAP;
CONFIG_FILE_READ_EXT DTC_STATUS_MAP diag_dtc_sts_map;

typedef struct stdtc_list_struct_ {
  uint16_t module_id;
  uint16_t event_id;
  uint16_t dtc_id;
  uint16_t dtc_report_byte_index;
  uint16_t dtc_report_bit_index;
  char const *dtc_description;
  uint8_t dtc_status;
  char const *filename;
  uint32_t snapshotlen;
  uint32_t toclientlen;
  uint16_t stable_time;
} stdtc_list_struct;
typedef std::map<uint16_t, stdtc_list_struct> DTC_LIST_MAP;
CONFIG_FILE_READ_EXT DTC_LIST_MAP diag_dtc_list_map;

typedef struct diag_dtc_info_ {
  uint16_t dtc_id;
  uint16_t dtc_count;
  uint8_t dtc_trigged;
  uint8_t dtc_avliable;
  uint8_t dtc_env_wr_en;
  uint16_t tmcnt;
  uint16_t stable_time;
  uint8_t env_data_gen_timing;
  uint32_t ErrLen;
  uint8_t *pErr;  // when error, env data
  uint32_t BeforSucessLen;
  uint8_t *pBeforSucess;  // when chg to success, env data
  uint32_t SucessLen;
  uint8_t *pSucess;  // save general env data
} diag_dtc_info;
typedef std::map<uint16_t, diag_dtc_info> DTC_INFO_MAP;
CONFIG_FILE_READ_EXT DTC_INFO_MAP diag_dtc_info_map;

typedef struct dtc_base_info_ {
  uint16_t kernel_dtc_id_base;
  uint16_t kernel_module_id_max;
  uint16_t kernel_module_id_min;

  uint16_t model_dtc_id_base;
  uint16_t model_module_id_max;
  uint16_t model_module_id_min;

  uint16_t perception_dtc_id_base;
  uint16_t perception_module_id_max;
  uint16_t perception_module_id_min;

  uint16_t adas_dtc_id_base;
  uint16_t adas_module_id_max;
  uint16_t adas_module_id_min;

  uint16_t ap_dtc_id_base;
  uint16_t ap_module_id_max;
  uint16_t ap_module_id_min;

  uint16_t general_dtc_id_base;
  uint16_t general_module_id_max;
  uint16_t general_module_id_min;

  uint16_t selftest_dtc_id_base;
  uint16_t selftest_module_id_max;
  uint16_t selftest_module_id_min;

  uint16_t dtc_sts_len_byte_num_total;
  uint16_t dtc_sts_byte_num_total;

  uint16_t dtc_env_len_byte_num_total;
  uint16_t dtc_env_data_byte_num_total;
  uint16_t dtc_env_id_byte_num_total;
} dtc_base_info;
CONFIG_FILE_READ_EXT dtc_base_info diag_dtc_base_info;

typedef struct diagnose_info_base_ {
  char const *name;
  char const *version;
} diagnose_info_base;
CONFIG_FILE_READ_EXT diagnose_info_base base_diagnose_info;
CONFIG_FILE_READ_EXT uint8_t DTC_LIST_NUM_MAX;

CONFIG_FILE_READ_EXT int read_json_file(void);
#endif
