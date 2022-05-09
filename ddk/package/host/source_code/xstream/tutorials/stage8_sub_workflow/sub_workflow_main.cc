/**
 * @copyright Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @file      sub_workflow_main.cc
 * @brief     example for making method instance disable
 * @author    xudong.du
 * @date      2020/10/30
 */
#include <fstream>
#include <iostream>

#include "xstream/xstream_world.h"
#include "method/bbox.h"
#include "method/gray_image.h"

using xstream::BaseData;
using xstream::BaseDataPtr;
using xstream::BaseDataVector;
using xstream::InputData;
using xstream::InputDataPtr;
using xstream::InputParam;
using xstream::InputParamPtr;

namespace SubWorkflowTutorials {
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
      std::cout << "——output data " << data->name_ << " state:"
                << static_cast<std::underlying_type<xstream::DataState>::type>(
                       data->state_)
                << std::endl;
      if (data->name_ == "output_box") {
        auto out_rects = std::static_pointer_cast<BaseDataVector>(data);
        if (out_rects->state_ == xstream::DataState::VALID) {
          for (auto out_rect : out_rects->datas_) {
            auto bbox = std::static_pointer_cast<xstream::BBox>(out_rect);
            std::cout << "——output box: " << *bbox.get() << std::endl;
          }
        }
      }
      if (data->name_ == "box_shape") {
        auto out_shape_results = std::static_pointer_cast<BaseDataVector>(data);
        if (out_shape_results->state_ == xstream::DataState::VALID) {
          for (auto out_shape_result : out_shape_results->datas_) {
            auto shape_result =
                std::static_pointer_cast<xstream::BBoxClassifyResult>(
                    out_shape_result);
            std::cout << "——output shape: " << *shape_result.get() << std::endl;
          }
        }
      }
    }
    std::cout << "============Output Call Back End============" << std::endl;
  }
};
}  // namespace SubWorkflowTutorials

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::cout << "Usage : ./stage8_sub_workflow config" << std::endl;
    std::cout << "Example : ./stage8_sub_workflow"
              << " ./config/sub_workflow_config.json" << std::endl;
    return -1;
  }
  auto config_file = argv[1];
  // Create xstream sdk object
  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();

  SubWorkflowTutorials::Callback callback;
  std::cout << "config_file :" << config_file << std::endl;
  flow->SetConfig("config_file", config_file);
  auto ret = flow->Init();
  std::cout << "flow init ret = " << ret << std::endl;

  // prepare input data
  InputDataPtr inputdata(new InputData());
  BaseDataVector *data(new BaseDataVector);
  xstream::GrayImagePtr gray_image = std::make_shared<xstream::GrayImage>();
  gray_image->width = 1920;
  gray_image->height = 1080;
  data->name_ = "image";  // corresponding the inputs in workflow
  data->datas_.push_back(gray_image);
  inputdata->datas_.push_back(BaseDataPtr(data));
  // predict
  std::cout << "start SyncPredict.." << std::endl;
  auto out = flow->SyncPredict(inputdata);
  callback.OnCallback(out);
  // destruct xstream sdk object
  delete flow;
  return 0;
}
