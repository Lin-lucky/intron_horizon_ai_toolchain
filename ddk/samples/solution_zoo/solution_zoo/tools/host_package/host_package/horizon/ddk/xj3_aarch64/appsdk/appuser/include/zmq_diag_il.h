/***************************************************************************
 *   Copyright (C) 2019 by horizon.                                        *
 *   zhaoqing.su@horizon.ai                                                *
 *                                                                         *
 *   zmq_diag_il header file.                                              *
 *                                                                         *
 ***************************************************************************/
#ifndef INCLUDE_ZMQ_DIAG_IL_H_
#define INCLUDE_ZMQ_DIAG_IL_H_

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

#ifdef ZMQ_DIAG_IL_GLOBALS
#define ZMQ_DIAG_IL_EXT
#else
#define ZMQ_DIAG_IL_EXT extern
#endif

typedef enum {
  diag_zmq_resp_null,
  diag_zmq_resp_req_err,
  diag_zmq_resp_uds_oping,
  diag_zmq_resp_uds_res,
  diag_zmq_resp_end
} diag_zmq_resp_type;
ZMQ_DIAG_IL_EXT diag_zmq_resp_type diag_zmq_resp_map_type;

#define DIAG_REQ_HEAD_ERR_RESP 0x7F
#define MESSAGE_ID_SET 0x0505
#define SPI_PROTOCOL_ID_SET 0x00535049
#define ETH_PROTOCOL_ID_SET 0x00535049
#define UART_PROTOCOL_ID_SET 0x00535049
#define REQ_COMM_ID_SET 0x0202
#define RESP_COMM_ID_SET 0x0302

typedef struct {
  uint8_t msgid[2];  // msgid[0]---high byte

  uint8_t msglen[4];  // msglen[0]---high byte

  uint8_t protocol_id[4];  // protocol_id[0]---high byte

  uint8_t time_stamp[8];

  uint8_t comm_id[2];  // comm_id[0]---high byte

  uint8_t comm_len[4];  // comm_len[0]---high byte
} __attribute__((packed)) zmq_diag_il_head;

typedef struct {
  zmq_diag_il_head head;
  uint8_t *data;
} __attribute__((packed)) zmq_diag_il_pack;
ZMQ_DIAG_IL_EXT zmq_diag_il_pack zmq_diag_req;
ZMQ_DIAG_IL_EXT zmq_diag_il_pack zmq_diag_resp;

ZMQ_DIAG_IL_EXT int zmq_diag_req_from_client_process(void);
ZMQ_DIAG_IL_EXT uint32_t
zmq_diag_resp_to_client_process(diag_zmq_resp_type tdiag_resp_type);
#endif  // INCLUDE_ZMQ_DIAG_IL_H_
