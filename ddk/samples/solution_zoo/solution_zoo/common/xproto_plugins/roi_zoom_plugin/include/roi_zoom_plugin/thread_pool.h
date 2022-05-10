/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @file
 * @brief
 * @author
 * @email
 * @date
 */

#ifndef INCLUDE_ROI_ZOOM_PLUGIN_THREADPOOL_H_
#define INCLUDE_ROI_ZOOM_PLUGIN_THREADPOOL_H_

#include <atomic>
#include <condition_variable>
#include <list>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <functional>

namespace xproto {

typedef std::function<void()> TaskFunction;
struct Task {
  TaskFunction func;
  explicit Task(const TaskFunction& _task) : func(_task) {}
};

typedef std::shared_ptr<Task> PTask;

class ThreadPool;
typedef std::shared_ptr<ThreadPool> PThreadPool;

#define MAX_TASK_NUM 10
#define SAFEPTR std::shared_ptr
#define MAKE_SAFEPTR std::make_shared

class ThreadPool {
 public:
  ThreadPool();
  ~ThreadPool();

 public:
  // Starts the execution of the thread pool.
  /*virtual*/ bool Start(int thread_count);

  // stop thread pool,  waits until all threads are joined.
  /*virtual*/ void Stop();

  /*virtual*/ std::vector<std::thread::id> GetAllThreadId();

  int GetTaskMaxNum() { return MAX_TASK_NUM; }
  int GetTaskNum() {
    std::lock_guard<std::mutex> lck(mutex_);
    return (int)task_list_.size();  // NOLINT
  }

  void AddTask(const TaskFunction& task);

 protected:
  void ProcessTask();

 protected:
  typedef std::list<PTask> TaskList;
  typedef std::shared_ptr<std::thread> PThread;

  TaskList task_list_;
  std::vector<PThread> thread_pool_;

  mutable std::mutex mutex_;
  mutable std::mutex mutex_thread_;
  std::condition_variable condition_;

  std::atomic<bool> stop_;

  int idelThread_num_;
};
typedef std::shared_ptr<ThreadPool> PThreadPool;

}  // namespace xproto

#endif  // INCLUDE_ROI_ZOOM_PLUGIN_THREADPOOL_H_
