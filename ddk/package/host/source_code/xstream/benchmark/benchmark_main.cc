/**
* @copyright Copyright (c) 2020 Horizon Robotics. All rights reserved.
* @file      benchmark_main.cc
* @brief     benchmark_main of xstream
* @author    zhe.sun
* @date      2020/11/5
*/

#include <signal.h>
#include <iostream>
#include <chrono>
#include <mutex>
#include <thread>
#include <condition_variable>
#include "xstream/xstream_world.h"
#include "method/passthrough_method.h"

using xstream::BaseData;
using xstream::BaseDataPtr;
using xstream::InputData;
using xstream::InputDataPtr;

// FPS统计函数
void FrameFPS(xstream::OutputDataPtr output);

// ctrl + C 退出
static bool exit_ = false;
static void signal_handle(int param) {
  std::cout << "recv signal " << param << ", stop" << std::endl;
  exit_ = true;
}

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::cout << "Usage : ./benchmark_main config"
              << std::endl;
    std::cout << "Example : ./benchmark_main"
              << " ./config/benchmark.json" << std::endl;
    return -1;
  }

  signal(SIGINT, signal_handle);
  signal(SIGSEGV, signal_handle);

  auto config = argv[1];
  std::cout << "config_file :" << config << std::endl;

  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();
  flow->SetConfig("config_file", config);
  flow->SetCallback(std::bind(&FrameFPS, std::placeholders::_1));
  flow->Init();

  while (!exit_) {
    InputDataPtr inputdata(new InputData());
    BaseDataPtr data(new BaseData());
    data->name_ = "input";   // corresponding the inputs in workflow
    inputdata->datas_.push_back(data);

    // async mode
    flow->AsyncPredict(inputdata);
    std::this_thread::sleep_for(std::chrono::microseconds(500));
  }

  // destruct xstream sdk object
  delete flow;
  return 0;
}


void FrameFPS(xstream::OutputDataPtr output) {
  static auto last_time = std::chrono::system_clock::now();
  static int fps = 0;
  static int frameCount = 0;

  frameCount++;

  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now() - last_time);
  if (duration.count() > 1000) {
    fps = frameCount;
    frameCount = 0;
    last_time = std::chrono::system_clock::now();
    std::cout << "fps = " << fps << std::endl;
  }
}
