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
#include "common/xthread_pool.h"
#include "hobotxstream/method_manager.h"

namespace xstream_xthread_pool {

void function_test(void*) {  }

int XThreadPoolKeyMatching(const void *key,
  const std::vector<void *> &contexts) {
    return 0;
  }

TEST(Interface, DelOneThread) {
  const std::string unique_name = "ut_xthread_pool_ptr";

  xstream::XThreadRawPtr xthread_raw_ptr0 = new xstream::XThread(100, INT_MAX);
  xstream::XThreadRawPtr xthread_raw_ptr1 = new xstream::XThread(200, INT_MAX);
  xstream::XThreadRawPtr xthread_raw_ptr2 = new xstream::XThread(300, INT_MAX);
  std::vector<xstream::XThreadRawPtr> xthread_v;
  xthread_v.push_back(xthread_raw_ptr0);
  xthread_v.push_back(xthread_raw_ptr1);
  xthread_v.push_back(xthread_raw_ptr2);

  xstream::WrapperFunctionTask prepare = function_test;
  std::vector<xstream::WrapperFunctionTask> prepares;
  prepares.push_back(prepare);
  prepares.push_back(prepare);
  prepares.push_back(prepare);
  xstream::MethodManagerContextPtr context =
      std::make_shared<xstream::MethodManagerContext>();
  context->state_ = xstream::MethodManagerContextState::INITIALIZED;

  std::vector<xstream::MethodManagerContextPtr> contexts;
  contexts.resize(3);
  std::vector<void *> params;
  params.resize(3);
  for (size_t i = 0; i < 3; ++i) {
    contexts[i] = std::make_shared<xstream::MethodManagerContext>();
    contexts[i]->state_ = xstream::MethodManagerContextState::INITIALIZED;;
    params[i] = contexts[i].get();
  }
  std::shared_ptr<xstream::XThreadPool> xthread_pool_ptr =
      std::make_shared<xstream::XThreadPool>(unique_name,
      xthread_v, prepares, params);

  xstream::XThreadRawPtr xthread_raw_ptr = new xstream::XThread(1234, INT_MAX);
  xthread_pool_ptr->AddOneThread(xthread_raw_ptr, prepare,
    reinterpret_cast<void*>(&context));
  xthread_pool_ptr->SetAffinity(0);
  xthread_pool_ptr->GetThreadIdx();
  xthread_pool_ptr->GetThreads();
  xthread_pool_ptr->Pause();
  xthread_pool_ptr->Resume();

  const void *key = static_cast<const void*>(unique_name.c_str());
  auto ret = xthread_pool_ptr->PostTimerTask(
    prepare, std::chrono::milliseconds(10), key);
  EXPECT_EQ(0, ret);
  xstream::XThreadPool::PostStrategy stgy;
  ret = xthread_pool_ptr->PostTimerTask(
    prepare, std::chrono::milliseconds(10), key);
  EXPECT_EQ(0, ret);
  stgy = xstream::XThreadPool::PostStrategy::KEY_MATCHING;
  ret = xthread_pool_ptr->SetPostStrategy(stgy);
  EXPECT_EQ(true, ret);
  ret = xthread_pool_ptr->PostTimerTask(
    prepare, std::chrono::milliseconds(10), key);
  EXPECT_EQ(-1, ret);

  stgy = xstream::XThreadPool::PostStrategy::KEY_MATCHING;
  ret = xthread_pool_ptr->SetPostStrategy(stgy);
  EXPECT_EQ(true, ret);
  xthread_pool_ptr->SetKeyMatchingFunc(XThreadPoolKeyMatching);
  ret = xthread_pool_ptr->PostTimerTask(
    prepare, std::chrono::milliseconds(10), key);
  EXPECT_EQ(0, ret);

  xthread_pool_ptr->DelOneThread();
  xthread_pool_ptr->ClearTasks();
}

}  // namespace xstream_xthread_pool
