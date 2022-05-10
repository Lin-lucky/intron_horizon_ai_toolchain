 /*
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @brief     LowPassFilter Method
 * @author    ronghui.zhang
 * @email     ronghui.zhang@horizon.ai
 * @version   0.0.0.1
 * @date      2020.11.02
 */

#ifndef INCLUDE_LOWPASSFILTERMETHOD_LOWPASSFILTERMETHOD_H_
#define INCLUDE_LOWPASSFILTERMETHOD_LOWPASSFILTERMETHOD_H_

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <set>

#include "xstream/simple_method.h"
#include "lowpass_filter.h"
#include "oneeuro_filter.h"

namespace xstream {
struct pair_filter {
  pair_filter(double freq,
    double mincutoff = 1.0, double beta_ = 1.0, double dcutoff = 0.3) {
    filter_x = std::make_shared<OneEuroFilter>(freq, mincutoff, beta_, dcutoff);
    filter_y = std::make_shared<OneEuroFilter>(freq, mincutoff, beta_, dcutoff);
  }
  std::shared_ptr<OneEuroFilter> filter_x;
  std::shared_ptr<OneEuroFilter> filter_y;
};

struct filtersVector {
  std::vector<std::shared_ptr<pair_filter>> filter_vector;
};

class LowPassFilterMethod : public SimpleMethod {
 public:
  int Init(const std::string &config_file_path) override;

  std::vector<BaseDataPtr> DoProcess(
      const std::vector<BaseDataPtr> &input,
      const xstream::InputParamPtr &param) override;

  void Finalize() override;

  int UpdateParameter(InputParamPtr ptr) override;

  InputParamPtr GetParameter() const override;

  std::string GetVersion() const override { return "0.0.1"; }

  MethodInfo GetMethodInfo() override {
    MethodInfo method_info;
    method_info.is_thread_safe_ = true;
    return method_info;
  };

 private:
  void RunSingleFrame(
    const std::vector<BaseDataPtr> &frame_input,
    std::vector<BaseDataPtr> &frame_output);

  std::set<int> id_set_;
  std::map<int, filtersVector> kps_filter_pairs_;
  std::map<int, filtersVector> box_filter_pairs_;

  double dcutoff_ = 0.3;
  double beta_ = 1.0;
  double mincutoff_ = 0.3;
  double freq_ = 120;
};

}  // namespace xstream

#endif  // INCLUDE_MERGEMETHOD_MERGEMETHOD_H_
