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

#ifndef COMMON_XTHREAD_POOL_H_
#define COMMON_XTHREAD_POOL_H_

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

typedef std::function<void(void *)> WrapperFunctionTask;

typedef std::function<int(const void *, const std::vector<void *> &)>
    ThreadPoolKeyMatchingFunc;

class XThreadPool {
 public:
  /**
   * @brief Construct a new Thread Pool object
   *
   * @param unique_name 线程池的唯一名字，
   *                    clear task时用于区分是否是该线程池发射的task
   * @param ths 需要添加进线程池的线程
   * @param prepares 每个线程执行正式任务之前需要执行的操作
   * @param contexts 每个线程对应的context信息。
   */
  explicit XThreadPool(const std::string &unique_name,
                       const std::vector<XThreadRawPtr> &ths,
                       const std::vector<WrapperFunctionTask> &prepares,
                       const std::vector<void *> &contexts);

  virtual ~XThreadPool();

  // 动态添加线程
  void AddOneThread(const XThreadRawPtr th, const WrapperFunctionTask &prepare,
                    void *context);

  // 动态删除线程，只是从线程池中删掉，不释放线程资源。
  // 会自动调用ThreadStop，并把该线程剩余负载迁移到其他线程;
  // 如果当前线程池中只有一个线程，删除失败，返回NULL;
  XThreadRawPtr DelOneThread();

  enum class PostStrategy {
    ROUND_BIN = 0,
    KEY_MATCHING = 1,
    // TODO(songshan.gong): support other post strategy.
  };

  bool SetPostStrategy(PostStrategy stgy = PostStrategy::ROUND_BIN);

  bool SetKeyMatchingFunc(const ThreadPoolKeyMatchingFunc &key_matching_func) {
    if (key_matching_func == nullptr) return false;
    find_keymatching_index_ = key_matching_func;
    return true;
  }

  int PostTimerTask(const WrapperFunctionTask &task,
                    std::chrono::milliseconds timeout,
                    const void *key = nullptr);

  int PostAsyncTask(const WrapperFunctionTask &task, const void *key = nullptr);

  int Pause();

  int Resume();

  int Stop();

  void ClearTasks();

  bool SetAffinity(int core_id);

  bool SetPriority(const std::string &policy, int priority);

  std::vector<uint32_t> GetThreadIdx() const;

  std::vector<XThreadRawPtr> GetThreads() const;

 private:
  XThreadPool() = delete;
  int PostAsyncTaskInternal(const FunctionTask &task,
                            const void *key = nullptr);

  int DefaultKeyMatching(const void *key, const std::vector<void *> &contexts) {
    return -1;
  }

  int GetSelectThreadIdx(const void *key, int *select_th_idx);

  std::vector<XThreadRawPtr> threads_;
  mutable std::mutex thread_mutex_;
  std::string unique_name_;

  std::vector<WrapperFunctionTask> prepares_;
  std::vector<void *> contexts_;
  ThreadPoolKeyMatchingFunc find_keymatching_index_;

  PostStrategy stgy_{PostStrategy::ROUND_BIN};
  uint32_t cur_post_pos_{0};

  std::atomic<bool> stop_{false};
  std::atomic<bool> pause_{false};
};

typedef std::shared_ptr<XThreadPool> XThreadPoolPtr;

}  // namespace xstream

#endif  // COMMON_XTHREAD_POOL_H_
