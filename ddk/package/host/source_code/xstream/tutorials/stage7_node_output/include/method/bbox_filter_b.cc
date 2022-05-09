/**
 * @copyright Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @file      bbox_filter_b.cc
 * @brief     BBoxFilterB class implementation
 * @author    Qingpeng Liu (qingpeng.liu@horizon.ai)
 * @version   0.0.0.1
 * @date      2020-01-03
 */

#include "method/bbox_filter_b.h"
#include <cassert>
#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <thread>
#include "xstream/xstream_world.h"
#include "method/bbox.h"

namespace xstream {

int BBoxFilterB::Init(const std::string &config_file_path) {
  std::cout << "BBoxFilterB::Init " << config_file_path << std::endl;
  area_threshold_ = 100;

  std::cout << "BBoxFilterB::Init area_thres:" << area_threshold_ << std::endl;
  return 0;
}

int BBoxFilterB::UpdateParameter(InputParamPtr ptr) {
  return 0;
}

InputParamPtr BBoxFilterB::GetParameter() const {
  return InputParamPtr();
}

std::vector<BaseDataPtr> BBoxFilterB::DoProcess(
    const std::vector<BaseDataPtr> &input,
    const InputParamPtr &param) {

  std::cout << "BBoxFilterB::DoProcess begin" << input.size() << std::endl;
  std::vector<BaseDataPtr> output;
  // one batch
  for (size_t j = 0; j < input.size(); j++) {
    output.push_back(std::make_shared<BaseDataVector>());
    if (input[j]->state_ == DataState::INVALID) {
      // std::cout << "input slot " << j << " is invalid" << std::endl;
      continue;
    }
    auto in_rects = std::static_pointer_cast<BaseDataVector>(input[j]);
    auto out_rects = std::static_pointer_cast<BaseDataVector>(output[j]);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cout << "BBoxFilterB::DoProcessing end" << std::endl;
  return output;
}

void BBoxFilterB::Finalize() {
  std::cout << "BBoxFilterB::Finalize" << std::endl;
}

std::string BBoxFilterB::GetVersion() const { return "test_only"; }

}  // namespace xstream
