/***************************************************************************
 *   Copyright (C) 2019 by horizon.                                        *
 *   zhaoqing.su@horizon.ai                                                *
 *                                                                         *
 *   dtc_to_file header file.                                              *
 *                                                                         *
 ***************************************************************************/
#ifndef DTC_TO_FILE_H
#define DTC_TO_FILE_H

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

#ifdef DTC_TO_FILE_GLOBALS
#define DTC_TO_FILE_EXT
#else
#define DTC_TO_FILE_EXT extern
#endif
typedef enum { time_node_init = 0, time_node_noinit = 1 } read_dtc_time_node;

DTC_TO_FILE_EXT int dtc_to_file_init(void);
DTC_TO_FILE_EXT int write_dtc_env_to_file(uint16_t dtc_id, uint8_t dtc_type,
                                          uint8_t *envbuf, uint32_t envlen);
DTC_TO_FILE_EXT int write_dtc_status_to_file(void);
DTC_TO_FILE_EXT int read_dtc_env_from_file(uint16_t dtc_id, uint8_t dtc_type,
                                           uint32_t dtc_env_len);
DTC_TO_FILE_EXT int read_dtc_status_from_file(read_dtc_time_node time_node);
DTC_TO_FILE_EXT int dtc_to_file_clear(uint8_t type);

#endif
