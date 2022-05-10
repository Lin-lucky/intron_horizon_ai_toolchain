/* Copyright 2019 Horizon Robotics, Inc.
SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * uevent_helper.h
 * uevent helper function to handle dwc3 controller unbind/bind case.
 * eg.
 *    1. sdb board micro usb cable hotplug will trigger usb mode switch
 *      default(micro usb plug out):	host mode
 *      micro usb plug in:		device mode
 *
 *    2. xj3 board with otg cable hotplug
 *      default: device mode
 *      otg cable plug in: host mode
 *
 *
 * Contact: jianghe xu<jianghe.xu@horizon.ai>
 */

#ifndef __UEVENT_HELPER_H__
#define __UEVENT_HELPER_H__

struct uevent {
  const char *action;
  const char *path;
  const char *subsystem;
  const char *firmware;
  int major;
  int minor;
  const char *devname;
  const char *devtype;
};

int open_uevent_socket();
void close_uevent_socket(int sock);
void parse_uevent(const char *msg, struct uevent *uevent);

#endif  /* __UEVENT_HELPER_H__ */
