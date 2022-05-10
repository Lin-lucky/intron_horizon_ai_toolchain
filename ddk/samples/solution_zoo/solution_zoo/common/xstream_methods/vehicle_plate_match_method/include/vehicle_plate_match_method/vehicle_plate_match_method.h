/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     BBoxFilter Method
 * @author    shuhuan.sun
 * @email     shuhuan.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.23
 */

#ifndef METHOD_VEHICLE_PLATE_MATCH_METHOD_H_
#define METHOD_VEHICLE_PLATE_MATCH_METHOD_H_

#include <atomic>
#include <string>
#include <vector>
#include <memory>
#include "xstream/simple_method.h"
#include "xstream/vision_type.h"

#include "match_utility/data_type.hpp"
#include "match_utility/utils.hpp"
#include "match_strategy/config_params_type.hpp"
#include "match_strategy/pairs_match_api.hpp"

namespace xstream {

class VehiclePlateMatchMethod : public SimpleMethod {
 public:
  int Init(const std::string &config_file_path) override;

  std::vector<BaseDataPtr> DoProcess(
      const std::vector<BaseDataPtr> &input,
      const xstream::InputParamPtr &param) override;

  void Finalize() override;

  int UpdateParameter(InputParamPtr ptr) override;

  InputParamPtr GetParameter() const override;

  std::string GetVersion() const override;

 private:
  std::shared_ptr<hobot::vehicle_match_strategy::PairsMatchAPI> matcher;
  bool using_fake_vehicle_lmks_;
  bool using_fake_plate_lmks_;
  bool using_fake_plate_color_;
  bool using_fake_plate_type_;
};

}  // namespace xstream


#endif  // METHOD_VEHICLE_MATCH_H_
