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
#include "hobotlog/hobotlog.hpp"

namespace xstream {

#if defined(HR_LINUX) && defined(HR_POSIX)
static std::unordered_map<std::string, LinuxThreadPriority> g_str2priority = {
    {"SCHED_NORMAL", LinuxThreadPriority::LINUX_SCHED_NORMAL},
    {"SCHED_OTHER", LinuxThreadPriority::LINUX_SCHED_NORMAL},
    {"SCHED_BATCH", LinuxThreadPriority::LINUX_SCHED_BATCH},
    {"SCHED_RR", LinuxThreadPriority::LINUX_SCHED_RR},
    {"SCHED_FIFO", LinuxThreadPriority::LINUX_SCHED_FIFO},
    {"SCHED_IDLE", LinuxThreadPriority::LINUX_SCHED_IDLE}};
#endif

XThread::XThread(uint32_t thread_idx, int max_task_count) {
  thread_idx_ = thread_idx;
  max_task_count_ = max_task_count;

  thread_ =
      std::shared_ptr<std::thread>(new std::thread(&XThread::ExecLoop, this));
}

XThread::~XThread() { Stop(); }

int XThread::PostTimerTask(const std::string &post_from,
                           const FunctionTask &task,
                           std::chrono::milliseconds timeout) {
  // not implement
  return 0;
}

int XThread::PostAsyncTask(const std::string &post_from,
                           const FunctionTask &functask) {
  if (stop_) return 0;
  {
    {
      std::lock_guard<std::mutex> lck(task_queue_mutex_);
      if (task_queue_.size() >= max_task_count_) {
        return -1;
      }
      auto task = std::shared_ptr<Task>(new Task(post_from, functask));
      task_queue_.push_back(task);
    }
    condition_.notify_one();
  }
  return 0;
}

int XThread::Pause() {
  pause_++;
  return 0;
}

int XThread::Resume() {
  pause_--;
  condition_.notify_one();
  return 0;
}

int XThread::Stop() {
  if (!stop_) {
    {
      {
        std::lock_guard<std::mutex> lck(task_queue_mutex_);
        stop_ = true;
      }
      condition_.notify_one();
    }
    if (thread_) {
      thread_->join();
    }
  }
  return 0;
}

void XThread::ClearSpecificTasks(const std::string &post_from,
                                 std::list<std::shared_ptr<Task>> *removed) {
  std::lock_guard<std::mutex> lck(task_queue_mutex_);
  TaskContainer target_list;
  for (auto it = task_queue_.begin(); it != task_queue_.end();) {
    if ((*it)->post_from_ == post_from) {
      target_list.push_back(*it);
      it = task_queue_.erase(it);
    } else {
      it++;
    }
  }
  if (removed) {
    *removed = target_list;
  }
}

void XThread::ClearTasks(std::list<std::shared_ptr<Task>> *removed) {
  std::lock_guard<std::mutex> lck(task_queue_mutex_);
  TaskContainer target_list;
  task_queue_.swap(target_list);
  if (removed) {
    *removed = target_list;
  }
}

bool XThread::SetAffinity(int core_id) {
  // not implement
  return true;
}

bool XThread::SetPriority(const std::string &policy, int priority) {
#if defined(HR_POSIX) && defined(HR_LINUX)
  if (g_str2priority.find(policy) == g_str2priority.end()) {
    return false;
  }

  sched_param sch;
  int old_policy;
  pthread_getschedparam(thread_->native_handle(), &old_policy, &sch);
  auto enum_policy = g_str2priority.find(policy)->second;
  switch (enum_policy) {
    case LinuxThreadPriority::LINUX_SCHED_NORMAL: {
      LOGW << policy << " cannot set priority";
      if (pthread_setschedparam(thread_->native_handle(), SCHED_OTHER, &sch)) {
        LOGE << "Failed to setschedparam: " << std::strerror(errno);
      }
      break;
    }
    case LinuxThreadPriority::LINUX_SCHED_BATCH: {
      LOGW << policy << " cannot set priority";
      if (pthread_setschedparam(thread_->native_handle(), SCHED_BATCH, &sch)) {
        LOGE << "Failed to setschedparam: " << std::strerror(errno);
      }
      break;
    }
    case LinuxThreadPriority::LINUX_SCHED_IDLE: {
      LOGW << policy << " cannot set priority";
      if (pthread_setschedparam(thread_->native_handle(), SCHED_IDLE, &sch)) {
        LOGE << "Failed to setschedparam: " << std::strerror(errno);
      }
      break;
    }
    case LinuxThreadPriority::LINUX_SCHED_FIFO: {
      sch.sched_priority = priority;
      if (pthread_setschedparam(thread_->native_handle(), SCHED_FIFO, &sch)) {
        LOGE << "Failed to setschedparam: " << std::strerror(errno);
      }
      break;
    }
    case LinuxThreadPriority::LINUX_SCHED_RR: {
      sch.sched_priority = priority;
      if (pthread_setschedparam(thread_->native_handle(), SCHED_RR, &sch)) {
        LOGE << "Failed to setschedparam: " << std::strerror(errno);
      }
      break;
    }
    default: {
      LOGW << "NOT support sched policy:" << policy;
      break;
    }
  }

#endif
  return true;
}

int XThread::SetThreadName(std::string name) {
  thread_name_ = name;
  return 0;
}

std::string XThread::GetThreadName() {
  return thread_name_;
}

uint32_t XThread::GetThreadIdx() const { return thread_idx_; }

std::thread::id XThread::GetThreadId() const {return thread_->get_id();}

void XThread::ExecLoop() {
  do {
    std::shared_ptr<Task> tsk;
    {
      std::unique_lock<std::mutex> lck(task_queue_mutex_);
      condition_.wait(
          lck, [&]() { return (task_queue_.size() > 0 && !pause_) || stop_; });
      if (stop_) {
        return;
      }

      tsk = task_queue_.front();
      task_queue_.pop_front();
    }
    tsk->func_();
  } while (!stop_);
}

}  // namespace xstream
