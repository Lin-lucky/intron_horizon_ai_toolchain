/**
 * Copyright (c) 2020, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */
#ifndef INCLUDE_VDECMODULE_H_
#define INCLUDE_VDECMODULE_H_

#include <mutex>
#include <vector>

#include "media_pipe_manager/basic_media_module.h"
#include "hb_comm_vdec.h"

namespace solution {
namespace video_box {
class VdecModule : public BasicMediaModule {
public:
  VdecModule();
  ~VdecModule();
  virtual int Init(uint32_t group_id,
                   const PipeModuleInfo *module_info) override;
  virtual int Start() override;
  virtual int Input(void *data) override;
  virtual int Output(void **data) override;
  virtual int OutputBufferFree(void *data) override;
  virtual int Stop() override;
  virtual int DeInit() override;

private:
  static std::once_flag flag_;
  uint32_t group_id_;
  uint32_t timeout_;

  uint32_t frameDepth_;
  uint32_t buffer_index_;
  std::vector<VIDEO_FRAME_S> buffers_;

  int in_fps_ = 0;
  int out_fps_ = 0;
  std::chrono::high_resolution_clock::time_point in_start_tp_;
  std::chrono::high_resolution_clock::time_point out_start_tp_;
};

}  // namespace vision
}  // namespace horizon
#endif  // INCLUDE_VDECMODULE_H_