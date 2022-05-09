/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: ronghui zhang
 * @Mail: zhangronghui@horizon.ai
 * @Date: 2019-11-30 01:15:22
 * @Version: v0.0.1
 * @Brief: test thread safe
 */

#include <gtest/gtest.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <thread>
#include <fstream>

#include "hobotlog/hobotlog.hpp"
#include "xstream/xstream_error.h"
#include "xstream/xstream_sdk.h"
#include "hobotxstream/xstream_config.h"

TEST(ConfigTest, input) {
  LOGD << "test input" << std::endl;
  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();
  flow->SetConfig("config_file", "./test/configs/config_input.json");
  EXPECT_EQ(xstream::INPUT_UNFEED_ERROR, flow->Init());
}

TEST(ConfigTest, flow_input) {
  LOGD << "test flow input" << std::endl;
  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();
  flow->SetConfig("config_file", "./test/configs/config_flow_input.json");
  EXPECT_EQ(xstream::INPUT_UNFEED_ERROR, flow->Init());
}

TEST(ConfigTest, input_error) {
  LOGD << "test input error" << std::endl;
  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();
  flow->SetConfig("config_file", "./test/configs/config_input_error.json");
  EXPECT_EQ(xstream::INPUT_UNFEED_ERROR, flow->Init());
}

TEST(ConfigTest, nodeName) {
  LOGD << "test nodeName" << std::endl;
  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();
  flow->SetConfig("config_file", "./test/configs/config_nodeName.json");
  EXPECT_EQ(xstream::NODE_NAME_ERROR, flow->Init());
}

TEST(ConfigTest, isCircle) {
  LOGD << "test circle" << std::endl;
  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();
  flow->SetConfig("config_file", "./test/configs/config_circle.json");
  EXPECT_EQ(xstream::WORKFLOW_CIRCLE_ERROR, flow->Init());
}

TEST(ConfigTest, isRepeatedOutput) {
  LOGD << "test repeated output" << std::endl;
  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();
  flow->SetConfig("config_file", "./test/configs/config_repeated_output.json");
  EXPECT_EQ(xstream::OUTPUT_REPEATED_ERROR, flow->Init());
}

TEST(ConfigTest, isOK) {
  LOGD << "test correct workflow" << std::endl;
  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();
  flow->SetConfig("config_file", "./test/configs/config_ok.json");
  EXPECT_EQ(xstream::CONFIG_OK, flow->Init());
}

TEST(ConfigTest, jsonStringOK) {
  LOGD << "test json string workflow" << std::endl;
  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();
  std::ifstream in("./test/configs/config_ok.json");
  std::string config_string;
  config_string.assign(std::istreambuf_iterator<char>(in),
                      std::istreambuf_iterator<char>());
  LOGD << "config_string is:"
       << config_string;
  flow->SetConfig("config_string", config_string.c_str());
  EXPECT_EQ(xstream::CONFIG_OK, flow->Init());
}
