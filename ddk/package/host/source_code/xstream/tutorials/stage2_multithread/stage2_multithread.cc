/**
* @copyright Copyright (c) 2020 Horizon Robotics. All rights reserved.
* @file      stage2_multithread.cc
* @brief     simple example of multithread
* @author    zhe.sun
* @date      2020/10/31
*/

#include <unistd.h>
#include <iostream>
#include <string>
#include <random>
#include "xstream/xstream_world.h"
#include "method/value.h"

using xstream::InputParam;
using xstream::InputParamPtr;
using xstream::BaseData;
using xstream::BaseDataPtr;
using xstream::BaseDataVector;
using xstream::InputData;
using xstream::InputDataPtr;
using xstream::FloatValue;
using xstream::FloatValuePtr;

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
      auto out_data = std::static_pointer_cast<FloatValue>(data);
      std::cout << "——output data: " << out_data->value_ << std::endl;
    }
    std::cout << "============Output Call Back End============" << std::endl;
  }
};

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::cout << "Usage : ./stage2_multithread_example config"
              << std::endl;
    std::cout << "Example : ./stage2_multithread_example"
              << " ./config/multithread_workflow.json" << std::endl;
    return -1;
  }
  auto config = argv[1];
  std::cout << "config_file :" << config << std::endl;

  // Create xstream sdk object
  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();
  // Set config_file
  flow->SetConfig("config_file", config);
  // Init
  flow->Init();
  // Set CallBack Func For Async Mode
  Callback callback;
  flow->SetCallback(
      std::bind(&Callback::OnCallback, &callback, std::placeholders::_1));

  for (int i = 0; i < 2; i++) {
    // Prepare input data
    InputDataPtr inputdata(new InputData());
    BaseDataVector *data(new BaseDataVector);
    for (int i = 0; i < 10; i++) {
      std::random_device rd;
      std::default_random_engine eng(rd());
      std::uniform_real_distribution<float> distr(0, 10);

      FloatValuePtr float_value = std::make_shared<FloatValue>(distr(eng));
      data->datas_.push_back(float_value);
    }
    data->name_ = "input_array";   // corresponding the inputs in workflow
    inputdata->datas_.push_back(BaseDataPtr(data));

    flow->AsyncPredict(inputdata);
  }

  sleep(2);
  // destruct xstream sdk object
  delete flow;
  return 0;
}
