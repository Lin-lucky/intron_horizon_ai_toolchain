/*
 * Copyright (c) 2019 Horizon Robotics. All rights reserved
 * @Author  jianglei.huang
 * @version 0.0.1
 * @date  2019.10.16
 */

#include "match_strategy/pairs_match_api.hpp"
#include "match_strategy/pairs_match.hpp"

namespace hobot {
namespace vehicle_match_strategy {

std::shared_ptr<PairsMatchAPI> PairsMatchAPI::NewPairsMatchAPI(
    const std::string &config_file) {
  return std::make_shared<PairsMatch>(config_file);
}

std::shared_ptr<PairsMatchAPI> PairsMatchAPI::NewPairsMatchAPI(
    const Params &params) {
  return std::make_shared<PairsMatch>(params);
}

}   // namespace vehicle_match_strategy
}   // namespace hobot
