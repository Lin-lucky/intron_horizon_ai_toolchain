/**
 * Copyright (c) 2020, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */
#ifndef INCLUDE_VPSMODULE_H_
#define INCLUDE_VPSMODULE_H_

#include <vector>

#include "hb_vio_interface.h"
#include "media_pipe_manager/basic_media_module.h"

namespace solution {
namespace video_box {
class VpsModule : public BasicMediaModule {
public:
  VpsModule();
  ~VpsModule();
  virtual int Init(uint32_t group_id,
                   const PipeModuleInfo *module_info) override;
  virtual int Start() override;
  virtual int Input(void *data) override;
  virtual int Output(void **data) override;
  virtual int OutputBufferFree(void *data) override;
  virtual int Stop() override;
  virtual int DeInit() override;

private:
  uint32_t group_id_;
  uint32_t timeout_;
  uint32_t frameDepth_;
  uint32_t buffer_index_;
  std::vector<pym_buffer_t> buffers_;
};

}  // namespace vision
}  // namespace horizon
#endif  // INCLUDE_VPSMODULE_H_