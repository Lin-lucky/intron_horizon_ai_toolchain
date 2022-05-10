/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @file
 * @brief
 * @author
 * @email
 * @date
 */

#include "roi_zoom_plugin/thread_pool.h"

#include <iostream>

namespace xproto {

ThreadPool::ThreadPool() {
  stop_ = false;
  thread_pool_.clear();
}

ThreadPool::~ThreadPool() {
  stop_ = false;
  thread_pool_.clear();
}

bool ThreadPool::Start(int thread_count) {
  stop_ = false;
  std::lock_guard<std::mutex> lck(mutex_thread_);
  for (int i = 0; i < thread_count; i++) {
    thread_pool_.emplace_back(PThread(
        MAKE_SAFEPTR<std::thread>(std::bind(&ThreadPool::ProcessTask, this))));
  }

  return true;
}

void ThreadPool::Stop() {
  stop_ = true;
  condition_.notify_all();
  std::lock_guard<std::mutex> lck(mutex_thread_);
  for (auto &thread : thread_pool_) {
    thread->join();
  }
  thread_pool_.clear();
}

std::vector<std::thread::id> ThreadPool::GetAllThreadId() {
  std::vector<std::thread::id> ids;
  std::lock_guard<std::mutex> lck(mutex_thread_);
  for (auto &thread : thread_pool_) {
    ids.emplace_back(thread->get_id());
  }
  return ids;
}

void ThreadPool::ProcessTask() {
  PTask task;
  do {
    {
      std::unique_lock<std::mutex> lck(mutex_);
      if (task_list_.size() <= 0) {
        condition_.wait(lck);
      }

      if (task_list_.size() <= 0) continue;
      task = task_list_.front();
      task_list_.pop_front();
    }

    task->func();
    condition_.notify_one();
  } while (!stop_);
}

void ThreadPool::AddTask(const TaskFunction &fun) {
  auto task = PTask(MAKE_SAFEPTR<Task>(fun));
  {
    std::lock_guard<std::mutex> lck(mutex_);
    task_list_.push_back(task);
  }
  condition_.notify_one();  // wake up one thread to process task
}

}  // namespace xproto
