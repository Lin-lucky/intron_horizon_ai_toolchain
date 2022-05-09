/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @file MultiSourceTestMethod.cpp
 * @brief
 * @author guoqian.sun
 * @email guoqian.sun@horizon.ai
 * @date 2019/12/04
 */
#include "multisource_test_method.h"

#include <cassert>
#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <thread>
#include <vector>

#if defined(HR_POSIX)
#include <pthread.h>
#endif
#include "hobotlog/hobotlog.hpp"
#include "hobotxstream/data_types/bbox.h"
#include "json/json.h"
#include "xstream/profiler.h"
#include "xstream/xstream_data.h"

namespace xstream {
// C++ 11 要求静态成员变量在使用之前必须声明。
std::atomic_ulong MultiSourceTestMethod::instance_count_;
MethodInfo MultiSourceTestMethod::methodinfo_;

std::vector<std::vector<BaseDataPtr>> MultiSourceTestMethod::DoProcess(
    const std::vector<std::vector<BaseDataPtr>> &input,
    const std::vector<InputParamPtr> &param) {
  size_t threadhash = std::hash<std::thread::id>()(std::this_thread::get_id());
  std::vector<std::vector<BaseDataPtr>> output;
  output.resize(input.size());
  // input data can be a batch of frame.
  // first layer of vector is the frame batch
  for (size_t batch_idx = 0; batch_idx < input.size(); batch_idx++) {
    auto &input_slots = input[batch_idx];
    auto &output_slots = output[batch_idx];
    output_slots.resize(input_slots.size());
    // size_t frm_id = 0;
    // size_t src_id = 0;
    for (size_t slot_idx = 0; slot_idx < input_slots.size(); slot_idx++) {
      if (input_slots[slot_idx]->state_ == DataState::INVALID) {
        std::cout << "input slot " << slot_idx << " is invalid" << std::endl;
        continue;
      }
      auto in_slot =
          std::static_pointer_cast<MulSrcTestInput>(input_slots[slot_idx]);
      auto out_slot = std::make_shared<MulSrcTestOutput>();

      out_slot->frame_id_ = in_slot->frame_id_;
      out_slot->source_id_ = in_slot->source_id_;

      out_slot->thread_hash_ = threadhash;
      out_slot->method_id_ = method_id_;
      output_slots[slot_idx] = out_slot;
    }
    // std::cout << "Test Method: data from source-" <<src_id << " frame-"
    // << frm_id << " runs on method-" << method_id_ << " via thread-"
    // << std::hex << threadhash << std::endl;
  }
  return output;
}
void MultiSourceTestMethod::SetMethodInfo(const MethodInfo &methodinfo) {
  methodinfo_ = methodinfo;
}

}  // namespace xstream
