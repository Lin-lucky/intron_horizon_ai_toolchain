/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     first_num_best header
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.16
 * @date      2019.05.22
 */

#ifndef COMMON_XSTREAM_METHODS_SNAPSHOT_METHOD_\
INCLUDE_SNAPSHOT_METHOD_STRATEGY_CROP_H_
#define COMMON_XSTREAM_METHODS_SNAPSHOT_METHOD_\
INCLUDE_SNAPSHOT_METHOD_STRATEGY_CROP_H_

#include <list>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "snapshot_method/snapshot_method.h"
#include "snapshot_method/snapshot_utils.h"
#include "snapshot_method/snapshot_data_type.h"
#include "xstream/vision_type.h"

namespace xstream {

class Crop : public SnapShot {
 public:
  int Init(std::shared_ptr<SnapShotParam> config) override;

  std::vector<BaseDataPtr> ProcessFrame(const std::vector<BaseDataPtr> &in,
                                        const InputParamPtr &param) override;

  void Finalize() override;

  int UpdateParameter(const std::string &content) override;

 private:
  std::vector<BaseDataPtr> CropAndPost(const std::vector<BaseDataPtr> &in);
};
}  // namespace xstream

#endif  // COMMON_XSTREAM_METHODS_SNAPSHOT_METHOD_INCLUDE_SNAPSHOT_METHOD_STRATEGY_CROP_H_
