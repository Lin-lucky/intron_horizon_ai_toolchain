/*
 * Copyright (c) 2019 Horizon Robotics. All rights reserved
 * @Author  jianglei.huang
 * @version 0.0.1
 * @date  2019.10.22
 */

#ifndef MATCH_STRATEGY_PARSE_CONFIG_FROM_JSON_HPP_
#define MATCH_STRATEGY_PARSE_CONFIG_FROM_JSON_HPP_

#include <fstream>
#include <string>
#include "json/json.h"
#include "match_utility/parse_json.hpp"
#include "match_strategy/config_params_type.hpp"

namespace hobot {
namespace vehicle_match_strategy {

class ParseJsonMatch : protected ParseJson {
 public:
  ParseJsonMatch() = default;
  explicit ParseJsonMatch(const std::string &config_file) {
    Init(config_file);
  }

 public:
  Params params_;

 private:
  /*
   * @biref Actually init class members by parsing config.
   * Must be realized in child class.
   * @param ifs, config file input stream
   */
  void DoParsing(std::ifstream &is) override;

  /*
   * @biref Overloaded function.
   * Actually init class members by parsing config.
   * Must be realized in child class.
   * @param ifs, string input stream.
   */
  void DoParsing(std::istringstream &is) override {};
};

}   // namespace vehicle_match_strategy
}   // namespace hobot

#endif  // MATCH_STRATEGY_PARSE_CONFIG_FROM_JSON_HPP_
