/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include <functional>
#include <memory>
#include "hobotlog/hobotlog.hpp"
#include "utils/executor.h"

namespace xproto {

std::once_flag Executor::flag_;
std::shared_ptr<Executor> Executor::worker_;

std::shared_ptr<Executor> Executor::GetInstance(const int exe_cnt) {
  if (!worker_) {
    std::call_once(flag_,
        [&]() {
        worker_ = std::make_shared<Executor>(exe_cnt);
        });
  }
  return worker_;
}

Executor::Executor(const int exe_cnt) {
  stop_ = false;
  pause_ = false;
  thread_count_ = exe_cnt;
  for (auto i = 0; i < thread_count_; ++i) {
    threads_.emplace_back(
        std::make_shared<std::thread>(std::bind(&Executor::Run, this)));
  }
}

Executor::~Executor() {
#if 0
  stop_ = true;
  condition_.notify_all();  // wake worker thread(s)
  LOGD << "Before join";
  for (auto i = 0; i < thread_count_; ++i) {
    threads_[i]->join();
  }
  LOGD << "After join";
#else
  Stop();
#endif
}

int Executor::Pause() {
  LOGW << "pause Executor";
  pause_ = true;
  return 0;
}
int Executor::Stop() {
  if (!stop_) {
    {
      std::lock_guard<std::mutex> lck(task_queue_mutex_);
      stop_ = true;
    }
    condition_.notify_all();
  }

  for (auto i = 0; i < thread_count_; ++i) {
    threads_[i]->join();
  }
  LOGD << "After join";
  return 0;
}

int Executor::Resume() {
  pause_ = false;
  condition_.notify_all();
  return 0;
}

std::future<bool> Executor::AddTask(exe_func func) {
  auto task = std::make_shared<Task>();
  task->func_ = func;
  task->p_ = std::make_shared<std::promise<bool>>();
  {
    std::lock_guard<std::mutex> lck(task_queue_mutex_);
    task_queue_.push_back(task);
    LOGI << "task_queue_size: " << task_queue_.size()
      << " thread_count: " << thread_count_;
    HOBOT_CHECK(task_queue_.size() <= static_cast<unsigned int>(thread_count_));
  }
  condition_.notify_one();  // wake worker thread(s)
  return task->p_->get_future();
}

void Executor::Run() {
  while (!stop_) {
    std::shared_ptr<Task> task;
    {
      std::unique_lock<std::mutex> lck(task_queue_mutex_);
      while ((task_queue_.size() <= 0 || pause_) && !stop_) {
        if (task_queue_.size() == 0) {
          LOGW << "task_queue_ is empty";
        } else {
          LOGW << "pause executor";
        }
        condition_.wait(lck);
      }
      if (stop_ || task_queue_.size() <= 0) {
        continue;
      }
      task = task_queue_.front();
      task_queue_.pop_front();
    }
    /// do job, func will exit auto because of the status of
    /// is_running
    task->func_();
    task->p_->set_value(true);
    LOGD << "Finish a job";
  }
}

}  // namespace xproto
