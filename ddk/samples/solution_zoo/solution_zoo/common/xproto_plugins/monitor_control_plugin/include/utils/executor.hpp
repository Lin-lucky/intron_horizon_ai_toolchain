/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @file      executor.h
 * @brief
 * @author    ruoting.ding
 * @email     ruoting.ding@horizon.ai
 * @date      2019.08.10
 */

#ifndef HORIZON_EXECUTOR_HPP_
#define HORIZON_EXECUTOR_HPP_

#include <signal.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <string.h>
#include <sched.h>
#include <future>
#include <vector>
#include <memory>
#include "blocking_queue.hpp"

namespace xproto {

class Executor {
 public:
  /// such as InputProducer::Run
  using exe_func = std::function<int()>;
  static std::shared_ptr<Executor> GetInstance();
  Executor();
  ~Executor();
  void Run();
  std::future<bool> AddTask(exe_func);
  int Pause();
  int Resume();
  int Stop();

 private:
  struct Task {
    std::shared_ptr<std::promise<bool>> p_;
    exe_func func_;
  };
  std::atomic_bool stop_;
  std::atomic_bool pause_;
  std::condition_variable condition_;
  horizon::vision::BlockingQueue<Task> task_queue_;
  using ThreadPtr = std::shared_ptr<std::thread>;
  std::vector<ThreadPtr> threads_;
  mutable std::mutex task_queue_mutex_;
  static std::once_flag flag_;
  static std::shared_ptr<Executor> worker_;
  int thread_count_ = 6;
};

}  // namespace xproto

#endif  // HORIZON_INPUT_PRODUCER_HPP_
