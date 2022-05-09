/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: thread_manager_test.cpp
 * @Brief: unit test of the ThreadManager
 * @Author: qingpeng.liu
 * @Email: qingpeng.liu@horizon.ai
 * @Date: 2020-12-22 21:44:05
 * @Last Modified by: qingpeng.liu
 * @Last Modified time: 2020-12-22 22:04:08
 */


#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <vector>


#include "hobotlog/hobotlog.hpp"
#include "common/xthread.h"
#include "hobotxstream/method_manager.h"

namespace xstream_xthread_pool {

void function_test(void*) {  }

TEST(Interface, DelOneThread) {
  const std::string unique_name = "ut_xthread_pool_ptr";

  std::shared_ptr<xstream::XThread> xthread_ptr =
    std::make_shared<xstream::XThread>(100, INT_MAX);

  xthread_ptr->SetAffinity(0);
  auto ret = xthread_ptr->GetThreadIdx();
  EXPECT_EQ(100u, ret);
  xthread_ptr->ClearTasks();
}

}  // namespace xstream_xthread_pool
