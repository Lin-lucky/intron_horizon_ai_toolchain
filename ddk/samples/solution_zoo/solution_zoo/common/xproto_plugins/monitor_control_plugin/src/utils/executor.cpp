/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @file      executor.cp
 * @brief
 * @author    ruoting.ding
 * @email     ruoting.ding@horizon.ai
 * @date      2019.08.10
 */
#include <memory>
#include <functional>
#include "hobotlog/hobotlog.hpp"
#include "utils/executor.hpp"

namespace xproto {

std::once_flag Executor::flag_;
std::shared_ptr<Executor> Executor::worker_;

std::shared_ptr<Executor> Executor::GetInstance() {
  if (!worker_) {
    std::call_once(flag_, [&]() { worker_ = std::make_shared<Executor>(); });
  }
  return worker_;
}

Executor::Executor() {
  stop_ = false;
  pause_ = false;
  for (auto i = 0; i < thread_count_; ++i) {
    threads_.emplace_back(
        std::make_shared<std::thread>(std::bind(&Executor::Run, this)));
  }
}

Executor::~Executor() {
  stop_ = true;
  condition_.notify_all();  // wake worker thread(s)
  for (auto i = 0; i < thread_count_; ++i) {
    threads_[i]->join();
  }
}

int Executor::Pause() {
  LOGD << "pause Executor";
  pause_ = true;
  return 0;
}
int Executor::Stop() {
  stop_ = true;
  return 0;
}

int Executor::Resume() {
  pause_ = false;
  condition_.notify_all();
  return 0;
}

std::future<bool> Executor::AddTask(exe_func func) {
  Task t;
  t.func_ = func;
  t.p_ = std::make_shared<std::promise<bool>>();
  task_queue_.push_back(t);
  HOBOT_CHECK(static_cast<int>(task_queue_.size()) <= thread_count_);
  condition_.notify_one();  // wake worker thread(s)
  return t.p_->get_future();
}

void Executor::Run() {
  Task task;
  do {
    {
      std::unique_lock<std::mutex> lck(task_queue_mutex_);
      while ((task_queue_.size() <= 0 || pause_) && !stop_) {
        if (task_queue_.size() == 0) {
          LOGD << "task_queue_ is empty";
        } else {
          LOGD << "pause executor";
        }
        condition_.wait(lck);
      }
      if (task_queue_.size() > 0) {
        task = task_queue_.pop();
      }
    }
    if (stop_) {
      LOGD << "stop executor";
      return;
    }
    /// do job, func will exit auto because of the status of
    /// is_running
    auto ret = task.func_();
    task.p_->set_value(ret);
    LOGD << "Finish a job";
  } while (!stop_);
}

}  // namespace xproto
