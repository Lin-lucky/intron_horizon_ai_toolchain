/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong,chuanyi.yang
 * @Mail: songshan.gong@horizon.ai
 * @Mail: chuanyi.yang@horizon.ai
 * @Date: 2019-10-09 19:55:46
 * @Version: v0.0.1
 * @Brief: xstream exec engine declarition.
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-11-13 05:19:25
 */

#ifndef COMMON_THREAD_MANAGER_H_
#define COMMON_THREAD_MANAGER_H_

#include <limits.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "xthread.h"
#include "hobotlog/hobotlog.hpp"

namespace xstream {

// user thread idx: 0~999;
// reserved thread_idx: 1000 ~1099
// auto-produce thread_idx:1100~INT_MAX
#define XSTREAM_SCHEDULE_THREAD_UPPER_IDX 1000
#define XSTREAM_SCHEDULE_THREAD_DOWN_IDX 1001
#define XSTREAM_AUTO_PRODUCE_THREAD_IDX_BASE 1100

// 按需分配线程资源，并通过数字索引分配的线程;
class ThreadManager {
 public:
  static ThreadManager *Instance() { return new ThreadManager(); }
  static uint instance_num_;
  uint id_;
  ThreadManager() {
    id_ = instance_num_;
    instance_num_++;
  }

  ~ThreadManager();

  XThreadRawPtr CreateThread(uint32_t thread_idx, std::string name = "");

  XThreadRawPtr CreateAutoThread(std::string thread_name = "") {
    std::lock_guard<std::mutex> lck(thread_mutex_);
    uint32_t thread_idx =
        XSTREAM_AUTO_PRODUCE_THREAD_IDX_BASE + auto_thread_cnt_;
    auto_thread_cnt_++;
    HOBOT_CHECK(threads_.find(thread_idx) == threads_.end())
        << "Anyone set auto-produce thread idx:" << thread_idx;
    threads_[thread_idx] = new XThread(thread_idx);
    threads_[thread_idx]->SetThreadName(thread_name);
    return threads_[thread_idx];
  }

  // 每个线程一个context，每个线程一个唯一索引idx
  std::vector<XThreadRawPtr> CreateThreads(std::vector<uint32_t> thread_idxes);

  // 根据thread_idx获取线程，如果尚未分配则返回NULL;
  XThreadRawPtr GetThread(uint32_t thread_idx);

  std::vector<XThreadRawPtr> GetThreads() const {
    std::vector<XThreadRawPtr> ret;
    for (auto it = threads_.begin(); it != threads_.end(); ++it) {
      ret.push_back(it->second);
    }
    return ret;
  }

  // 删除对应线程，并释放资源，要保证调用这个之前，先调用对应线程的Stop操作。
  bool DelThread(uint32_t thread_idx);

 private:
  ThreadManager(const ThreadManager &other) = delete;
  ThreadManager(ThreadManager &&other) = delete;

  std::unordered_map<uint32_t, XThreadRawPtr> threads_;
  mutable std::mutex thread_mutex_;

  int auto_thread_cnt_{0};
};

}  // namespace xstream

#endif  // COMMON_THREAD_MANAGER_H_
