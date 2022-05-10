/*
 * Copyright (c) 2019 Horizon Robotics. All rights reserved
 * @Author  jianglei.huang
 * @version 0.0.1
 * @date  2019.10.16
 */

#ifndef MATCH_STRATEGY_PAIRS_MATCH_API_HPP_
#define MATCH_STRATEGY_PAIRS_MATCH_API_HPP_

#include <memory>
#include <string>
#include "match_utility/data_type.hpp"
#include "match_strategy/config_params_type.hpp"

namespace hobot {
namespace vehicle_match_strategy {

/*
 * @brief Vehicle and Plate mating strategy API.
 */
class PairsMatchAPI {
 public:
  PairsMatchAPI() = default;
  virtual ~PairsMatchAPI() = default;

  /*
   * @brief Create a api
   * @params config file path
   */
  static std::shared_ptr<PairsMatchAPI> NewPairsMatchAPI(
      const std::string &config_file);

  /*
   * @brief Create a api
   * @params algorithm config parameters.
   */
  static std::shared_ptr<PairsMatchAPI> NewPairsMatchAPI(
      const Params &params);

  /*
   * @brief Main process function.
   * @params vehicle_list: pointer to a vector of VehicleType.
   * @params plate_list: pointer to a vector of PlateType.
   * @return the matched plate will be writed to corresponding
   * vehicle.
   */
  virtual void Process(VehicleListPtr &vehicle_list,
                       const PlateListPtr &plate_list) = 0;
};

}   // namespace vehicle_match_strategy
}   // namespace hobot

#endif  // MATCH_STRATEGY_PAIRS_MATCH_API_HPP_
