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

#include <iostream>

#include "hobotlog/hobotlog.hpp"
#include "hobotxstream/xstream_config.h"
#include "xstream/xstream_error.h"
#include "xstream/xstream_sdk.h"

TEST(MethodInitTest, ConfigPath) {
  LOGD << "test method init by config path" << std::endl;
  auto config = std::make_shared<xstream::XStreamConfig>();
  EXPECT_EQ(0, config->LoadFile("./test/configs/config_init_path.json"));

  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();
  flow->SetConfig("config_file", "./test/configs/config_init_path.json");
  EXPECT_EQ(0, flow->Init());
}

TEST(MethodInitTest, ConfigContent) {
  auto config = std::make_shared<xstream::XStreamConfig>();
  EXPECT_EQ(0,
            config->LoadFile("./test/configs/config_init_config_content.json"));
  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();
  flow->SetConfig("config_file",
                  "./test/configs/config_init_config_content.json");
  EXPECT_EQ(0, flow->Init());
}

TEST(MethodInitTest, ConfigIncludeContent) {
  auto config = std::make_shared<xstream::XStreamConfig>();
  EXPECT_EQ(0, config->LoadFile("./test/configs/config_init_path_v1.1.json"));
  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();
  flow->SetConfig("config_file", "./test/configs/config_init_path_v1.1.json");
  EXPECT_EQ(0, flow->Init());
}
