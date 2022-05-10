/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved
 * @Author  jianglei.huang
 * @version 0.0.1
 * @date  2019.10.16
 */

#ifndef MATCH_STRATEGY_CONFIG_PARAMS_TYPE_HPP_
#define MATCH_STRATEGY_CONFIG_PARAMS_TYPE_HPP_

#include <memory>

namespace hobot {
namespace vehicle_match_strategy {

struct Params {
  int img_width = 1920;
  int img_height = 1080;
  int img_cells = 2;
  int bbox_cells = 4;
  float overlap_th = 0.90;
  int min_statis_frames = 500;
  int work_frames = 2500;
  float max_pair_dis = 8.f;
  int min_stat_value = 5;
};

typedef std::shared_ptr<Params> ParamsPtr;

}   // namespace vehicle_match_strategy
}   // namespace hobot

#endif  // MATCH_STRATEGY_CONFIG_PARAMS_TYPE_HPP_
