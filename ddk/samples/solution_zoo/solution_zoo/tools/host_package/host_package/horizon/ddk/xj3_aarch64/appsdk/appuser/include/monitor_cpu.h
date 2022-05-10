/***************************************************************************
 *   Copyright (C) 2019 by horizon.                                        *
 *   zhaoqing.su@horizon.ai                                                *
 *                                                                         *
 *   monitor cpu header file.                                                  *
 *                                                                         *
 ***************************************************************************/
#ifndef INCLUDE_MONITOR_CPU_H_
#define INCLUDE_MONITOR_CPU_H_

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

#ifdef MONITOR_CPU_GLOBALS
#define MONITOR_CPU_EXT
#else
#define MONITOR_CPU_EXT extern
#endif

MONITOR_CPU_EXT void monitor_j2cpu_init(void);
MONITOR_CPU_EXT void monitor_j2cpu_task(void);
#endif  // INCLUDE_MONITOR_CPU_H_
