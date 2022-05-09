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

#ifndef COMMON_XTHREAD_H_
#define COMMON_XTHREAD_H_

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
#include "hobotlog/hobotlog.hpp"

namespace xstream {

#if defined(HR_POSIX) && defined(HR_LINUX)
enum class LinuxThreadPriority {
  LINUX_SCHED_NORMAL,
  LINUX_SCHED_BATCH,
  LINUX_SCHED_FIFO,
  LINUX_SCHED_RR,
  LINUX_SCHED_IDLE
};
#endif

typedef std::function<void()> FunctionTask;

struct Task {
  std::string post_from_;
  FunctionTask func_;

  explicit Task(const std::string &post_from, const FunctionTask &task)
      : post_from_(post_from), func_(task) {}
};

class XThread {
 public:
  explicit XThread(uint32_t thread_idx, int max_task_count = INT_MAX);

  virtual ~XThread();

  void SetMaxTaskCount(int max_task_count) { max_task_count_ = max_task_count; }

  int PostTimerTask(const std::string &post_from, const FunctionTask &task,
                    std::chrono::milliseconds timeout);

  int PostAsyncTask(const std::string &post_from, const FunctionTask &task);

  int Pause();

  int Resume();

  int Stop();

  // Clear task with key @post_from.
  void ClearSpecificTasks(const std::string &post_from,
                          std::list<std::shared_ptr<Task>> *removed = nullptr);

  // Clear all tasks.
  void ClearTasks(std::list<std::shared_ptr<Task>> *removed = nullptr);

  bool SetAffinity(int core_id);

  bool SetPriority(const std::string &policy, int priority);

  int SetThreadName(std::string name);

  std::string GetThreadName();

  uint32_t GetThreadIdx() const;

  std::thread::id GetThreadId() const;

 private:
  XThread() = delete;
  void ExecLoop();
  uint32_t thread_idx_;
  std::string thread_name_ = "";
  std::shared_ptr<std::thread> thread_;

  int TaskCount() { return task_queue_.size(); }

  typedef std::list<std::shared_ptr<Task>> TaskContainer;
  TaskContainer task_queue_;
  uint32_t max_task_count_;
  mutable std::mutex task_queue_mutex_;
  std::condition_variable condition_;

  std::atomic<bool> stop_{false};
  std::atomic<int> pause_{0};
};
typedef XThread *XThreadRawPtr;

}  // namespace xstream

#endif  // COMMON_XTHREAD_H_
