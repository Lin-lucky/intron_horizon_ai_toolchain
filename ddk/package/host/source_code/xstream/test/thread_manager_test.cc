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
#include "common/thread_manager.h"

namespace xstream_thread_manager {

TEST(Interface, DelThread) {
  std::shared_ptr<xstream::ThreadManager> thread_manager_ptr =
    std::make_shared<xstream::ThreadManager>();
  std::vector<uint32_t> thread_idxes;
  thread_idxes.push_back(1);
  thread_idxes.push_back(2);
  thread_idxes.push_back(3);
  auto ret_threads = thread_manager_ptr->CreateThreads(thread_idxes);
  EXPECT_EQ(3u, ret_threads.size());
  auto ret_get = thread_manager_ptr->GetThread(456);
  EXPECT_EQ(nullptr, ret_get);
  ret_get = thread_manager_ptr->GetThread(3);
  EXPECT_NE(nullptr, ret_get);
  auto ret_del = thread_manager_ptr->DelThread(1);
  EXPECT_EQ(true, ret_del);
  ret_del = thread_manager_ptr->DelThread(2);
  EXPECT_EQ(true, ret_del);
  ret_del = thread_manager_ptr->DelThread(3);
  EXPECT_EQ(true, ret_del);
}

}  // namespace xstream_thread_manager
