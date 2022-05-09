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

#include "common/xthread_pool.h"
#include "hobotlog/hobotlog.hpp"

namespace xstream {

XThreadPool::XThreadPool(const std::string &unique_name,
                         const std::vector<XThreadRawPtr> &ths,
                         const std::vector<WrapperFunctionTask> &prepares,
                         const std::vector<void *> &contexts) {
  unique_name_ = unique_name;
  threads_ = ths;
  prepares_ = prepares;
  contexts_ = contexts;
  find_keymatching_index_ =
      std::bind(&XThreadPool::DefaultKeyMatching, this, std::placeholders::_1,
                std::placeholders::_2);

  HOBOT_CHECK((ths.size() > 0) && (ths.size() == prepares.size()) &&
              (ths.size() == contexts.size()));
  // cast prepares functions firstly.
  for (size_t i = 0; i < threads_.size(); i++) {
    if (prepares[i]) {
      auto task = std::bind(prepares[i], contexts[i]);
      threads_[i]->PostAsyncTask(unique_name_, task);
    }
  }
}

XThreadPool::~XThreadPool() { Stop(); }

void XThreadPool::AddOneThread(const XThreadRawPtr th,
                               const WrapperFunctionTask &prepare,
                               void *context) {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  threads_.push_back(th);
  prepares_.push_back(prepare);
  contexts_.push_back(context);

  if (prepare) {
    th->PostAsyncTask(unique_name_, std::bind(prepare, context));
  }
}

int XThreadPool::GetSelectThreadIdx(const void *key, int *select_th_idx) {
  switch (stgy_) {
    case PostStrategy::ROUND_BIN: {
      if (cur_post_pos_ >= threads_.size()) {
        cur_post_pos_ = 0;
      }
      *select_th_idx = cur_post_pos_++;
      break;
    }
    case PostStrategy::KEY_MATCHING: {
      if (key == nullptr) {
        // return error, can not post the task since no key is inputed
        LOGE << "ThreadPool: key is Null in KEY_MATCHING mode ";
        return -1;
      }
      *select_th_idx = find_keymatching_index_(key, contexts_);
      if (*select_th_idx < 0) {
        LOGE << "ThreadPool: Can not found the key matching thread";
        return -1;
      }
      break;
    }
    default: {
      // not support other post strategy currently.
      HOBOT_CHECK(false) << "Doesn't support this " << (uint16_t)stgy_
                         << " post strategy currently";
    }
  }
  return 0;
}

int XThreadPool::PostAsyncTaskInternal(const FunctionTask &task,
                                       const void *key) {
  if (stop_) return 0;
  int selected_thr_idx = 0;

  if (GetSelectThreadIdx(key, &selected_thr_idx) < 0) {
    return -1;
  }

  threads_[selected_thr_idx]->PostAsyncTask(unique_name_, task);
  return 0;
}

XThreadRawPtr XThreadPool::DelOneThread() {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  if (threads_.size() <= 1) return nullptr;
  auto rm_thr = threads_.back();
  threads_.pop_back();
  contexts_.pop_back();
  prepares_.pop_back();
  std::list<std::shared_ptr<Task>> rm_list;
  rm_thr->ClearSpecificTasks(unique_name_, &rm_list);
  for (auto task : rm_list) {
    PostAsyncTaskInternal(task->func_);
  }
  return rm_thr;
}

bool XThreadPool::SetPostStrategy(XThreadPool::PostStrategy stgy) {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  stgy_ = stgy;

  return true;
}

int XThreadPool::PostTimerTask(const WrapperFunctionTask &task,
                               std::chrono::milliseconds timeout,
                               const void *key) {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  if (stop_) return 0;
  int selected_thr_idx = 0;

  if (GetSelectThreadIdx(key, &selected_thr_idx) < 0) {
    return -1;
  }

  threads_[selected_thr_idx]->PostTimerTask(
      unique_name_, std::bind(task, contexts_[selected_thr_idx]), timeout);
  return 0;
}

int XThreadPool::PostAsyncTask(const WrapperFunctionTask &task,
                               const void *key) {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  if (stop_) return 0;
  int selected_thr_idx = 0;
  if (GetSelectThreadIdx(key, &selected_thr_idx) < 0) {
    return -1;
  }
  threads_[selected_thr_idx]->PostAsyncTask(
      unique_name_, std::bind(task, contexts_[selected_thr_idx]));
  return 0;
}

int XThreadPool::Pause() {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  pause_ = true;

  return 0;
}

int XThreadPool::Resume() {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  pause_ = false;
  return 0;
}

int XThreadPool::Stop() {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  if (!stop_) {
    stop_ = true;
    for (auto thr : threads_) {
#if 1
      thr->Pause();
      thr->ClearSpecificTasks(unique_name_);
      thr->Resume();
#else
      // 假设所有node同时停止。
      thr->Stop();
#endif
    }
  }
  return 0;
}

void XThreadPool::ClearTasks() {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  for (auto thr : threads_) {
    thr->Pause();
    thr->ClearSpecificTasks(unique_name_);
    thr->Resume();
  }
}

bool XThreadPool::SetAffinity(int core_id) {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  for (auto thr : threads_) {
    thr->SetAffinity(core_id);
  }
  return true;
}

bool XThreadPool::SetPriority(const std::string &policy, int priority) {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  for (auto thr : threads_) {
    thr->SetPriority(policy, priority);
  }
  return true;
}

std::vector<uint32_t> XThreadPool::GetThreadIdx() const {
  std::vector<uint32_t> ret_vec;
  std::lock_guard<std::mutex> lck(thread_mutex_);
  for (auto thr : threads_) {
    ret_vec.push_back(thr->GetThreadIdx());
  }
  return ret_vec;
}

std::vector<XThreadRawPtr> XThreadPool::GetThreads() const {
  std::lock_guard<std::mutex> lck(thread_mutex_);
  return threads_;
}

}  // namespace xstream
