/**
 * Copyright (c) 2020, Inc.
 * All rights reserved.
 * @Author:
 * @Mail:
 */

#ifndef _INCLUDE_RTSPCLIENT_HEVC_H_
#define _INCLUDE_RTSPCLIENT_HEVC_H_

#include <stdio.h>
#include <string.h>

typedef unsigned int uint32;
typedef signed char int8;
typedef unsigned char uint8;

struct vc_params_t {
  int width, height;
  uint32 profile, level;
  uint32 nal_length_size;
  void clear() { memset(this, 0, sizeof(*this)); }
};

bool parse_sps(unsigned char *data, int size, vc_params_t &params);

#endif  // _INCLUDE_RTSPCLIENT_HEVC_H_
