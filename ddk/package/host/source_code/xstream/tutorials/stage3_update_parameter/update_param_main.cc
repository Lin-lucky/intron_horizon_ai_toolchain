/**
 * @copyright Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @file      main.cc
 * @brief     main function
 * @author    Qingpeng Liu (qingpeng.liu@horizon.ai)
 * @version   0.0.0.1
 * @date      2020-01-09
 */

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <map>
#include "xstream/xstream_world.h"
#include "method/filter_param.h"
#include "method/bbox.h"

using xstream::BaseData;
using xstream::BaseDataPtr;
using xstream::BaseDataVector;
using xstream::InputData;
using xstream::InputDataPtr;

// state integer to string
std::map<int, std::string> integalstate2str{{0, "VALID"}, {1, "FILTERED"},
                                            {2, "INVISIBLE"},
                                            {3, "DISAPPEARED"}, {4, "INVALID"}};

namespace stage3_update_param {

class Callback {
 public:
  void OnOutput(xstream::OutputDataPtr output) {
    using xstream::BaseDataVector;

    for (auto data : output->datas_) {
      if (data->error_code_ < 0) {
        std::cout << "data error: " << data->error_code_ << std::endl;
        continue;
      }
      BaseDataVector *pdata = reinterpret_cast<BaseDataVector *>(data.get());
      for (size_t i = 0; i < pdata->datas_.size(); ++i) {
        auto xstream_box =
            std::static_pointer_cast<xstream::BBox>(
                pdata->datas_[i]);
        auto state_int = static_cast<int>(xstream_box->state_);
        std::string state_str = integalstate2str[state_int];
        std::cout << pdata->name_ << " [" << i << "]: "
                  << state_str << std::endl;
      }
    }
    std::cout << "================================" << std::endl;
  }
};
}   // namespace stage3_update_param

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::cout << "Usage : excutable config_file" << std::endl;
    std::cout << "Example : ./stage3_update_parameter_example "
              << "./filter_workflow.json" << std::endl;
    return -1;
  }

  std::cout << "================================" << std::endl;
  std::cout << "    Update Parameter Example    " << std::endl;
  std::cout << "  BBox Filter A Threshold: 45   " << std::endl;
  std::cout << "  BBox Filter B Threshold: 150  " << std::endl;
  std::cout << "================================" << std::endl;
  std::cout << std::endl << std::endl;

  stage3_update_param::Callback callback;

  auto config = argv[1];
  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();
  flow->SetConfig("config_file", config);
  flow->Init();

  std::cout << std::endl;
  std::cout << "================================" << std::endl;

  for (int i = 1; i <= 10; ++i) {
    InputDataPtr inputdata(new InputData());
    BaseDataVector *data(new BaseDataVector);
    xstream::BBox *bbox(new xstream::BBox(0, 20, i, 50));
    std::cout << "input bbox: [0, 20, " << i << ", 50]" << std::endl;
    bbox->type_ = "BBox";

    data->datas_.push_back(BaseDataPtr(bbox));
    data->name_ = "in_bbox";

    inputdata->datas_.push_back(BaseDataPtr(data));

    auto output = flow->SyncPredict(inputdata);
    callback.OnOutput(output);

    // update parameter using member variable
    if (i == 4) {
      std::string unique_name("BBoxFilter_A");
      auto ptr = std::make_shared<xstream::FilterParam>(unique_name);
      std::cout << "===== "
                << "update BBoxFilter_A parameter, change threshold to: "
                << 180.0 << " =====" << std::endl;
      ptr->SetThreshold(180.0);
      flow->UpdateConfig(ptr->unique_name_, ptr);
    }

    // udpate parameter using json string
    if (i == 7) {
      std::string unique_name("BBoxFilter_A");
      std::string cfg("{\"threshold\":240.0}");
      auto ptr = std::make_shared<xstream::FilterParam>(unique_name, cfg);
      ptr->is_json_format_ = true;
      std::cout << "===== "
                << "update BBoxFilter_A parameter, change threshold to: "
                << 240.0 << " =====" << std::endl;
      flow->UpdateConfig(ptr->unique_name_, ptr);
    }
  }

  delete flow;

  std::cout << "================================" << std::endl;

  return 0;
}
