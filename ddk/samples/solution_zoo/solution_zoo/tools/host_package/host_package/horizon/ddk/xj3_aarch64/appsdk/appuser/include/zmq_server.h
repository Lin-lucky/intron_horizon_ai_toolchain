/***************************************************************************
 *   Copyright (C) 2019 by horizon.                                        *
 *   zhaoqing.su@horizon.ai                                                *
 *                                                                         *
 *   zmq_server header file.                                               *
 *                                                                         *
 ***************************************************************************/
#ifndef ZMQ_SERVER_H
#define ZMQ_SERVER_H

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

#ifdef ZMQ_SERVER_GLOBALS
#define ZMQ_SERVER_EXT
#else
#define ZMQ_SERVER_EXT extern
#endif

#define REQ_FROM_CLIENT_NUM_MAX 30
typedef struct {
  uint8_t req_from_client[REQ_FROM_CLIENT_NUM_MAX];
  uint8_t *resp_to_client;
} zmq_il_data;
ZMQ_SERVER_EXT zmq_il_data zmq_il_msg;

typedef enum {
  zmq_service_sts_dis = 0,
  zmq_service_sts_en,
  zmq_service_sts_end
} enzmq_service_sts;
ZMQ_SERVER_EXT enzmq_service_sts zmq_service_sts;

ZMQ_SERVER_EXT int zmq_server_init(void);
ZMQ_SERVER_EXT void *thread_zmq_server_process(void *arg);
#endif