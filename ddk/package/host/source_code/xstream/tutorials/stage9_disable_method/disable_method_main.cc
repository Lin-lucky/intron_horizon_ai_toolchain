/**
* @copyright Copyright (c) 2020 Horizon Robotics. All rights reserved.
* @file      disable_method_main.cc
* @brief     example for making method instance disable
* @author    zhe.sun
* @date      2020/10/27
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

namespace DisableMethodTutorials {
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
}  // namespace DisableMethodTutorials

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::cout << "Usage : ./disable_method_example config" << std::endl;
    std::cout << "Example : ./disable_method_example"
              << " ./config/control_workflow.json" << std::endl;
    return -1;
  }
  auto config = argv[1];
  // Create xstream sdk object
  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();

  DisableMethodTutorials::Callback callback;
  std::cout << "config_file :" << config << std::endl;
  flow->SetConfig("config_file", config);
  flow->Init();

  // prepare input data
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
  // 1 common predict
  auto out = flow->SyncPredict(inputdata);
  callback.OnCallback(out);

  // 2 set DisableParam Invalid for "bbox_area_filter"
  std::cout << "------------ invalid output------------" << std::endl;
  xstream::InputParamPtr invalidFilter3(
    new xstream::DisableParam("bbox_area_filter",
                              xstream::DisableParam::Mode::Invalid));
  // add invalid parameter to inputdata params_
  inputdata->params_.push_back(invalidFilter3);
  // start synchronous predict, and then get output
  out = flow->SyncPredict(inputdata);
  callback.OnCallback(out);


  // 3 set DisableParam Invalid output "bbox_length_filter"
  std::cout << "------------ invalid output------------" << std::endl;
  xstream::InputParamPtr invalidFilter2(
    new xstream::DisableParam("bbox_length_filter",
                              xstream::DisableParam::Mode::Invalid));
  // add invalid parameter to inputdata params_
  inputdata->params_.clear();
  inputdata->params_.push_back(invalidFilter2);
  // start synchronous predict, and then get output
  out = flow->SyncPredict(inputdata);
  callback.OnCallback(out);
  // destruct xstream sdk object
  delete flow;
  return 0;
}
