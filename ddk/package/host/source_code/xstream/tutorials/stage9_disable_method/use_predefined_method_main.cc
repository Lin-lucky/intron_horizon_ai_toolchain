/**
* Copyright (c) 2020 Horizon Robotics. All rights reserved.
* @file use_pre_defined_method_main.cc
* @brief  example for making method instance disable
* @author zhe.sun
* @date 2020/10/27
*/

#include <iostream>
#include "xstream/xstream_world.h"
#include "method/bbox.h"

using xstream::InputParam;
using xstream::InputParamPtr;
using xstream::BaseData;
using xstream::BaseDataPtr;
using xstream::BaseDataVector;
using xstream::InputData;
using xstream::InputDataPtr;

namespace UsePredefinedMethodTutorials {
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
}  // namespace UsePredefinedMethodTutorials

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::cout << "Usage : ./use_predefined_method_example config" << std::endl;
    std::cout << "Example : ./use_predefined_method_example "
      "./config/control_workflow.json" << std::endl;
    return -1;
  }

  auto config = argv[1];
  // 1 Create xstream sdk object
  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();

  UsePredefinedMethodTutorials::Callback callback;
  flow->SetConfig("config_file", config);
  flow->Init();

  // 2 prepare input data
  InputDataPtr inputdata(new InputData());
  BaseDataVector *data(new BaseDataVector);
  xstream::BBoxPtr bbox1 = std::make_shared<xstream::BBox>(0, 0, 10, 20, 0.1);
  xstream::BBoxPtr bbox2 = std::make_shared<xstream::BBox>(0, 0, 4, 5, 0.8);
  xstream::BBoxPtr bbox3 = std::make_shared<xstream::BBox>(0, 0, 6, 8, 0.7);
  xstream::BBoxPtr bbox4 = std::make_shared<xstream::BBox>(0, 0, 8, 8, 0.9);
  data->name_ = "face_head_box";   // corresponding the inputs in workflow
  data->datas_.push_back(bbox1);
  data->datas_.push_back(bbox2);
  data->datas_.push_back(bbox3);
  data->datas_.push_back(bbox4);
  inputdata->datas_.push_back(BaseDataPtr(data));

  // 3 use pre-defined input data
  std::cout << "------------ pre-defined output----------" << std::endl;
  inputdata->params_.clear();
  xstream::DisableParamPtr
      pre_define(
        new xstream::DisableParam(
          "bbox_area_filter",
          xstream::DisableParam::Mode::UsePreDefine));

  // 4 preprea new input data
  BaseDataVector *pre_data(new BaseDataVector);
  xstream::BBoxPtr pre_bbox =
      std::make_shared<xstream::BBox>(0, 0, 10, 10, 1.0);
  pre_data->datas_.push_back(pre_bbox);
  pre_define->pre_datas_.emplace_back(BaseDataPtr(pre_data));

  inputdata->params_.push_back(pre_define);
  auto out = flow->SyncPredict(inputdata);
  callback.OnCallback(out);

  // destruct xstream sdk object
  delete flow;
  return 0;
}
