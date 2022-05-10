/*
 * @Description: UT
 * @Author: fei.cheng@horizon.ai
 * @Date: 2019-11-05 19:40:56
 * @LastEditors: shiyu.fu@horizon.ai
 * @LastEditTime: 2020-08-17 11:51:27
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */
#include <sys/utsname.h>

#include <memory>

#include "gtest/gtest.h"
#include "hobotlog/hobotlog.hpp"
#include "reorder_message/reorder_message.h"
#include "reorder_plugin/reorder_plugin.h"
#include "smart_message/smart_message.h"
#include "xproto/msg_type/vio_message.h"

using xproto::ReorderPlugin;
using ReorderPluginPtr = std::shared_ptr<ReorderPlugin>;
using xproto::CustomReorderMessage;

class ReorderPluginTest : public ::testing::Test {
 protected:
  void SetUp() override { reorderplugin = std::make_shared<ReorderPlugin>(); }
  void TearDown() override {}

  ReorderPluginPtr reorderplugin = nullptr;
};

TEST_F(ReorderPluginTest, API) {
  int ret;
  SetLogLevel(INFO);

  if (reorderplugin == NULL) {
    std::cout << "failed to create reorderplugin" << std::endl;
    return;
  }

  ret = reorderplugin->Init();
  EXPECT_EQ(ret, 0);

  ret = reorderplugin->Start();
  EXPECT_EQ(ret, 0);

  sleep(1);

  ret = reorderplugin->Stop();
  EXPECT_EQ(ret, 0);

  ret = reorderplugin->DeInit();
  EXPECT_EQ(ret, 0);
}
