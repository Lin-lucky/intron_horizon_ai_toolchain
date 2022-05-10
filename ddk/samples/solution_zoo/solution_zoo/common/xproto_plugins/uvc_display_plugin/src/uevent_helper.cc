/*  Copyright 2019 Horizon Robotics, Inc.
SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * uevent_helper.c
 *  uevent helper function to handle dwc3 controller unbind/bind case.
 *  eg.
 *  1. sdb board micro usb cable hotplug will trigger usb mode switch
 *     default(micro usb plug out):	host mode
 *     micro usb plug in:		device mode
 *
 *  2. xj3 board with otg cable hotplug
 *     default: device mode
 *     otg cable plug in: host mode
 *
 *
 * Contact: jianghe xu<jianghe.xu@horizon.ai>
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/netlink.h>

#include "uevent_helper.h"

int open_uevent_socket()
{
  struct sockaddr_nl addr;
  int sz = 64 * 1024;
  int s;

  memset(&addr, 0, sizeof(addr));
  addr.nl_family = AF_NETLINK;
  addr.nl_pid = getpid();
  addr.nl_groups = 0xffffffff;

  s = socket(PF_NETLINK, SOCK_DGRAM, NETLINK_KOBJECT_UEVENT);
  if (s < 0)
    return -1;

  setsockopt(s, SOL_SOCKET, SO_RCVBUFFORCE, &sz, sizeof(sz));

  if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    close(s);
    return -1;
  }

  return s;
}

void parse_uevent(const char *msg, struct uevent *uevent)
{
  uevent->action = "";
  uevent->path = "";
  uevent->subsystem = "";
  uevent->firmware = "";
  uevent->major = -1;
  uevent->minor = -1;
  uevent->devname = "";
  uevent->devtype = "";

  while (*msg) {
    // printf("%s\n", msg);
    if (!strncmp(msg, "ACTION=", 7)) {
      msg += 7;
      uevent->action = msg;
    } else if (!strncmp(msg, "DEVPATH=", 8)) {
    msg += 8;
      uevent->path = msg;
    } else if (!strncmp(msg, "SUBSYSTEM=", 10)) {
      msg += 10;
      uevent->subsystem = msg;
    } else if (!strncmp(msg, "FIRMWARE=", 9)) {
      msg += 9;
      uevent->firmware = msg;
    } else if (!strncmp(msg, "MAJOR=", 6)) {
      msg += 6;
      uevent->major = atoi(msg);
      } else if (!strncmp(msg, "MINOR=", 6)) {
      msg += 6;
      uevent->minor = atoi(msg);
    } else if (!strncmp(msg, "DEVNAME=", 8)) {
      msg += 8;
      uevent->devname = msg;
    } else if (!strncmp(msg, "DEVTYPE=", 8)) {
      msg += 8;
      uevent->devtype = msg;
    }
    while (*msg++) {
    }
  }

  if (access("/tmp/uvc_uevent_print", F_OK) == 0) {
    printf("event { action = '%s', path = '%s', subsystem = '%s', "
        "firmware = '%s', major = %d, minor = %d , "
        "devname = '%s', devtype = '%s'}\n",
        uevent->action, uevent->path, uevent->subsystem,
        uevent->firmware, uevent->major, uevent->minor,
        uevent->devname, uevent->devtype);
      }
}

void close_uevent_socket(int sock)
{
  if (sock)
    close(sock);

  return;
}
