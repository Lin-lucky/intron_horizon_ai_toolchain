/*
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: main.cc
 * @Brief: 
 * @Author: ruoting.ding 
 * @Date: 2020-04-17 16:30:22 
 * @Last Modified by: qingpeng.liu
 * @Last Modified time: 2020-11-02 17:52:09
 */

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "method/callback.h"
#include "xstream/xstream_world.h"
#include "method/bbox.h"

int main(int argc, char const *argv[]) {
  using Stage7Async::Callback;
  using xstream::BaseData;
  using xstream::BaseDataPtr;
  using xstream::BaseDataVector;
  using xstream::InputData;
  using xstream::InputDataPtr;

  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();

  flow->SetConfig("config_file", "./config/filter.json");

  Callback callback;
  flow->SetCallback(
      std::bind(&Callback::OnCallback, &callback, std::placeholders::_1));
  flow->Init();
  std::cout << "========Init Finish==============" << std::endl;
  flow->SetCallback(
      std::bind(&Callback::OnCallbackBBoxFilterA,
       &callback, std::placeholders::_1), "BBoxFilter_1");

  float x1{0};   // BBox(框)的左上角横坐标
  float y1{20};  // BBox(框)的左上角纵坐标
  float x2{0};   // BBox(框)的右上角横坐标
  float y2{50};  // BBox(框)的右上角纵坐标

  std::shared_ptr<xstream::BBox> bbox = std::make_shared<xstream::BBox>(
    x1, y1, x2, y2);
  bbox->type_ = "BBox";
  std::shared_ptr<BaseDataVector> data = std::make_shared<BaseDataVector>();
  data->datas_.push_back(BaseDataPtr(bbox));
  data->name_ = "head_box";

  InputDataPtr inputdata = std::make_shared<InputData>();
  inputdata->datas_.push_back(BaseDataPtr(data));

  flow->AsyncPredict(inputdata);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  delete flow;

  return 0;
}
