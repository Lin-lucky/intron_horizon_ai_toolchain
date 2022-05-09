/**
* @copyright Copyright (c) 2020 Horizon Robotics. All rights reserved.
* @file      bbox_filter_main.cc
* @brief     simple example of method
* @author    zhe.sun
* @date      2020/10/30
*/

#include <unistd.h>
#include <iostream>
#include <string>
#include "xstream/xstream_world.h"
#include "method/bbox.h"

using xstream::InputParam;
using xstream::InputParamPtr;
using xstream::BaseData;
using xstream::BaseDataPtr;
using xstream::BaseDataVector;
using xstream::InputData;
using xstream::InputDataPtr;

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

int main(int argc, char const *argv[]) {
  if (argc < 3) {
    std::cout << "Usage : ./stage1_bbox_filter_example config async/sync"
              << std::endl;
    std::cout << "Example : ./stage1_bbox_filter_example"
              << " ./config/bbox_workflow.json async/sync" << std::endl;
    return -1;
  }
  auto config = argv[1];
  std::string running_mode = argv[2];
  std::cout << "config_file :" << config << std::endl;

  // Create xstream sdk object
  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();
  // Set config_file
  flow->SetConfig("config_file", config);

  // Set CallBack Func For Async Mode
  Callback callback;
  flow->SetCallback(
      std::bind(&Callback::OnCallback, &callback, std::placeholders::_1));
  // Init
  flow->Init();

  // Prepare input data
  InputDataPtr inputdata(new InputData());
  BaseDataVector *data(new BaseDataVector);
  xstream::BBoxPtr bbox1 = std::make_shared<xstream::BBox>(0, 0, 10, 20, 0.1);
  xstream::BBoxPtr bbox2 = std::make_shared<xstream::BBox>(0, 0, 4, 5, 0.3);
  xstream::BBoxPtr bbox3 = std::make_shared<xstream::BBox>(0, 0, 6, 8, 0.7);
  xstream::BBoxPtr bbox4 = std::make_shared<xstream::BBox>(0, 0, 8, 8, 0.9);
  data->name_ = "face_head_box";   // corresponding the inputs in workflow
  data->datas_.push_back(bbox1);
  data->datas_.push_back(bbox2);
  data->datas_.push_back(bbox3);
  data->datas_.push_back(bbox4);
  inputdata->datas_.push_back(BaseDataPtr(data));

  // predict according to running mode
  if (running_mode == "sync") {
    // sync mode
    auto out = flow->SyncPredict(inputdata);
    callback.OnCallback(out);
  } else if (running_mode == "async") {
    // async mode
    flow->AsyncPredict(inputdata);
  }
  sleep(1);

  // destruct xstream sdk object
  delete flow;
  return 0;
}
