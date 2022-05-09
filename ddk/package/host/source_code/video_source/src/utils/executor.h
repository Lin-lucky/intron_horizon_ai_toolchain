/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef VIDEO_SOURCE_UTILS_EXECUTOR_H_
#define VIDEO_SOURCE_UTILS_EXECUTOR_H_

#include <future>
#include <vector>
#include <list>
#include <memory>

namespace xproto {

class Executor {
 public:
  /// such as InputProducer::Run
  using exe_func = std::function<int()>;
  static std::shared_ptr<Executor> GetInstance(const int exe_cnt);
  explicit Executor(const int exe_cnt);
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
  typedef std::list<std::shared_ptr<Task> > TaskContainer;
  std::atomic_bool stop_;
  std::atomic_bool pause_;
  std::condition_variable condition_;
  TaskContainer task_queue_;
  using ThreadPtr = std::shared_ptr<std::thread>;
  std::vector<ThreadPtr> threads_;
  mutable std::mutex task_queue_mutex_;
  static std::once_flag flag_;
  static std::shared_ptr<Executor> worker_;
  int thread_count_ = 4;
};

}  // namespace xproto

#endif  // VIDEO_SOURCE_UTILS_EXECUTOR_H_
