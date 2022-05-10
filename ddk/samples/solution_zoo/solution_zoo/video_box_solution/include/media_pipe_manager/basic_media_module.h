/**
 * Copyright (c) 2020, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */
#ifndef INCLUDE_BASICMEDIAMODULE_H_
#define INCLUDE_BASICMEDIAMODULE_H_

#include <cstdint>
#include <cstring>

#include "vio/hb_vio_interface.h"

namespace solution {
namespace video_box {

typedef enum {
  RTSP_Payload_NONE,
  RTSP_Payload_H264,
  RTSP_Payload_H265,
  RTSP_Payload_PCMU,
  RTSP_Payload_PCMA,
  RTSP_Payload_AAC,
} RTSPPayloadType;

struct PipeModuleInfo {
  void *attr;
  uint32_t input_width;
  uint32_t input_height;
  uint32_t output_width;
  uint32_t output_height;
  uint32_t frame_depth;
  uint16_t input_encode_type;
};

class BasicMediaModule {
 public:
  virtual int Init(uint32_t group_id, const PipeModuleInfo *module_info) = 0;
  virtual int Start() = 0;
  virtual int Input(void *data) = 0;
  virtual int Output(void **data) = 0;
  virtual int OutputBufferFree(void *data) = 0;
  virtual int Stop() = 0;
  virtual int DeInit() = 0;
  BasicMediaModule(){};
  ~BasicMediaModule(){};
};

}  // namespace vision
}  // namespace horizon

#endif  // INCLUDE_BASICMEDIAMODULE_H_