/**
 * @copyright Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @file      main.cc
 * @brief     main function
 * @author    Ronghui Zhang (ronghui.zhang@horizon.ai)
 * @version   0.0.0.1
 * @date      2020-10-26
 */

#include <chrono>
#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include "method/bbox.h"
#include "xstream/xstream_world.h"
#include "method/alarm_method.h"

using std::chrono::milliseconds;

int main(int argc, char const *argv[]) {
  using xstream::BaseData;
  using xstream::BaseDataPtr;
  using xstream::BaseDataVector;
  using xstream::InputData;
  using xstream::InputDataPtr;
  if (argc < 2) {
    std::cout <<
    "Usage : ./stage5_timeout_alarm_sync_example work_flow_config_file"
              << std::endl;
    std::cout <<
    "Example : ./stage5_timeout_alarm_sync_example ./workflow.json"
              << std::endl;
    return -1;
  }

  std::string workflow_config = argv[1];
  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();

  int monitor_interval = 5;
  // set monitor_interval
  flow->SetConfig("time_monitor", std::to_string(monitor_interval));
  flow->SetConfig("config_file", workflow_config);
  flow->Init();

  std::cout << "***********************" << std::endl
            << "testing synchronous function" << std::endl
            << "***********************" << std::endl;
  for (int i = 0; i < 10; i++) {
    InputDataPtr inputdata(new InputData());
    BaseDataVector *data(new BaseDataVector);
    xstream::BBoxPtr bbox1 =
    std::make_shared<xstream::BBox>(0, 0, 10, 20, 0.1);
    xstream::BBoxPtr bbox2 = std::make_shared<xstream::BBox>(0, 0, 7, 7, 0.8);
    data->name_ = "in_bbox";   // corresponding the inputs in workflow
    data->datas_.push_back(bbox1);
    data->datas_.push_back(bbox2);
    inputdata->datas_.push_back(BaseDataPtr(data));

    auto out = flow->SyncPredict(inputdata);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  delete flow;
  return 0;
}
