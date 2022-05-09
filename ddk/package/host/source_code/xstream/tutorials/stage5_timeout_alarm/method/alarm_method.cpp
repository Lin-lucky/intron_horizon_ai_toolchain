/**
 * @copyright Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @file      alarm_method.cpp
 * @brief     Timeout_Alarm class implementation
 * @author    Ronghui Zhang (ronghui.zhang@horizon.ai)
 * @version   0.0.0.1
 * @date      2020-10-26
 */

#include <cassert>
#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <thread>
#include <ctime>
#include "bbox.h"
#include "xstream/xstream_world.h"
#include "method/alarm_method.h"

#define MIN_VALUE 3
#define MAX_VALUE 10

namespace xstream {

int TimeoutAlarm::Init(const std::string &config_file_path) {
  std::cout << "TimeoutAlarm::Init " << config_file_path << std::endl;
  return 0;
}

std::vector<BaseDataPtr> TimeoutAlarm::DoProcess(
    const std::vector<BaseDataPtr> &input,
    const InputParamPtr &param) {
  std::cout << "TimeoutAlarm::DoProcess" << std::endl;
  unsigned int seed = time(0);
  int cost = rand_r(&seed) % (MAX_VALUE - MIN_VALUE + 1) + MIN_VALUE;
  std::vector<BaseDataPtr> output;
  // one batch
  for (size_t j = 0; j < input.size(); j++) {
    output.push_back(std::make_shared<BaseDataVector>());
    if (input[j]->state_ == DataState::INVALID) {
      std::cout << "input slot " << j << " is invalid" << std::endl;
      continue;
    }
    auto in_rects = std::static_pointer_cast<BaseDataVector>(input[j]);
    auto out_rects = std::static_pointer_cast<BaseDataVector>(output[j]);
    for (auto &in_rect : in_rects->datas_) {
      // passthrough data
      out_rects->datas_.push_back(in_rect);
    }
  }
  std::cout << "sleep " << cost << " seconds" << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(cost));
  return output;
}

int TimeoutAlarm::UpdateParameter(InputParamPtr ptr) {
  return 0;
}

InputParamPtr TimeoutAlarm::GetParameter() const {
  return InputParamPtr();
}

void TimeoutAlarm::Finalize() {
  std::cout << "TimeoutAlarm::Finalize" << std::endl;
}

std::string TimeoutAlarm::GetVersion() const {
  return "TimeoutAlarm test";
}

}  // namespace xstream
