/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong, chuanyi.yang
 * @Mail: songshan.gong@horizon.ai
 * @Mail: chuanyi.yang@horizon.ai
 * @Date: 2019-10-10 05:34:51
 * @Version: v0.0.1
 * @Brief: xstream engine implementation.
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-11-13 18:27:39
 */

#if defined(HR_POSIX)
#include <pthread.h>
#endif
#include <cstring>
#include <unordered_map>

#include "common/xthread.h"
#include "common/thread_manager.h"
#include "hobotlog/hobotlog.hpp"

namespace xstream {

uint ThreadManager::instance_num_ = 0;

ThreadManager::~ThreadManager() {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  for (auto it = threads_.begin(); it != threads_.end(); it++) {
    auto pthr = it->second;
    delete pthr;
  }
  threads_.clear();
}

XThreadRawPtr ThreadManager::CreateThread(
    uint32_t thread_idx, std::string name) {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  if (threads_.find(thread_idx) == threads_.end()) {
    threads_[thread_idx] = new XThread(thread_idx);
    threads_[thread_idx]->SetThreadName(name);
  }
  return threads_[thread_idx];
}

std::vector<XThreadRawPtr> ThreadManager::CreateThreads(
    std::vector<uint32_t> thread_idxes) {
  std::vector<XThreadRawPtr> ret_threads;
  std::lock_guard<std::mutex> lck(thread_mutex_);
  for (auto thread_idx : thread_idxes) {
    if (threads_.find(thread_idx) == threads_.end()) {
      threads_[thread_idx] = new XThread(thread_idx);
    }
    ret_threads.push_back(threads_[thread_idx]);
  }

  return ret_threads;
}

XThreadRawPtr ThreadManager::GetThread(uint32_t thread_idx) {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  if (threads_.find(thread_idx) != threads_.end()) {
    return threads_[thread_idx];
  }
  return nullptr;
}

bool ThreadManager::DelThread(uint32_t thread_idx) {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  if (threads_.find(thread_idx) != threads_.end()) {
    auto thread = threads_[thread_idx];
    threads_.erase(thread_idx);
    delete thread;
  }
  return true;
}
}  // namespace xstream
