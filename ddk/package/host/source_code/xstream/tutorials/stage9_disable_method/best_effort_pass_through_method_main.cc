/**
* Copyright (c) 2020 Horizon Robotics. All rights reserved.
* @file best_effort_pass_through_method_main.cc
* @brief  example for making method instance pass through with best effort
* @author zhe.sun
* @date 2020/10/28
*/

#include <iostream>
#include "xstream/xstream_world.h"
#include "method/bbox.h"

using xstream::BaseData;
using xstream::BaseDataPtr;
using xstream::BaseDataVector;
using xstream::InputData;
using xstream::InputDataPtr;
using xstream::InputParam;
using xstream::InputParamPtr;

namespace BestEffortPassThoughPredefinedMethodTutorials {
class Callback {
 public:
  void OnCallback(xstream::OutputDataPtr output) {
    using xstream::BaseDataVector;
    std::cout << "============Output Call Back============" << std::endl;
    std::cout << "—seq: " << output->sequence_id_ << std::endl;
    std::cout << "—output_type: " << output->output_type_ << std::endl;
    std::cout << "—error_code: " << output->error_code_ << std::endl;
    std::cout << "—error_detail_: " << output->error_detail_ << std::endl;
    std::cout << "—datas_ size: " << output->datas_.size() << std::endl;
    for (auto data : output->datas_) {
      std::cout<< "——output data " << data->name_ << " state:"
      << static_cast<std::underlying_type<xstream::DataState>::type>(
      data->state_) << std::endl;
      auto out_rects = std::static_pointer_cast<BaseDataVector>(data);
      if (out_rects->state_ == xstream::DataState::VALID) {
        for (auto out_rect : out_rects->datas_) {
          auto bbox = std::static_pointer_cast<xstream::BBox>(out_rect);
          std::cout << "——output data: " << *bbox.get() << std::endl;
        }
      }
    }
    std::cout << "============Output Call Back End============" << std::endl;
  }
};
}  // namespace BestEffortPassThoughPredefinedMethodTutorials

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::cout << "Usage : ./best_effort_pass_though_method_example config"
      << std::endl;
    std::cout << "Example : ./best_effort_pass_though_method_example "
      "./config/control_workflow_best_effort_pass_through.json" << std::endl;
    return -1;
  }

  auto config = argv[1];
  // 1 Create xstream sdk object
  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();

  BestEffortPassThoughPredefinedMethodTutorials::Callback callback;
  flow->SetConfig("config_file", config);

  flow->Init();

  // 2 prepare input data
  InputDataPtr inputdata(new InputData());
  BaseDataVector *data1(new BaseDataVector);
  BaseDataVector *data2(new BaseDataVector);
  xstream::BBoxPtr bbox1 = std::make_shared<xstream::BBox>(0, 0, 10, 20, 0.1);
  xstream::BBoxPtr bbox2 = std::make_shared<xstream::BBox>(0, 0, 7, 7, 0.8);
  xstream::BBoxPtr bbox3 = std::make_shared<xstream::BBox>(0, 0, 6, 8, 0.7);
  xstream::BBoxPtr bbox4 = std::make_shared<xstream::BBox>(0, 0, 8, 8, 0.9);
  data1->name_ = "face_head_box";   // corresponding the inputs in workflow
  data2->name_ = "body_hand_box";   // corresponding the inputs in workflow
  data1->datas_.push_back(bbox1);
  data1->datas_.push_back(bbox2);
  data2->datas_.push_back(bbox3);
  data2->datas_.push_back(bbox4);
  inputdata->datas_.push_back(BaseDataPtr(data1));
  inputdata->datas_.push_back(BaseDataPtr(data2));

  // 3 common output
  auto out = flow->SyncPredict(inputdata);
  callback.OnCallback(out);

  // 4 set best pass through output
  std::cout << "------------best effort pass through output----------"
    << std::endl;
  inputdata->params_.clear();
  xstream::InputParamPtr
    b_effort_pass_through(
      new xstream::DisableParam(
        "bbox_area_filter",
        xstream::DisableParam::Mode::BestEffortPassThrough));
  inputdata->params_.push_back(b_effort_pass_through);
  out = flow->SyncPredict(inputdata);
  callback.OnCallback(out);
  // destruct xstream sdk object
  delete flow;
  return 0;
}
