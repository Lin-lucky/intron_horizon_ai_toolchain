/*!
 * Copyright (c) 2017 by Contributors
 * \file blocking_queue.hpp
 * \brief
 * \author liangzhujin
 */

#ifndef HORIZON_VISION_BLOCKING_QUEUE_HPP_
#define HORIZON_VISION_BLOCKING_QUEUE_HPP_

#include <condition_variable>
#include <chrono>
#include <deque>
#include <string>
#include <mutex>

namespace horizon {
namespace vision {

template<typename T>
class BlockingQueue {
 public:
  BlockingQueue() {}

  void push_front(const T &t) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      queue_.push_front(t);
    }
    condition_.notify_one();
  }

  void push_back(const T &t) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      queue_.push_back(t);
    }
    condition_.notify_one();
  }

  void push(const T &t) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      queue_.push_back(t);
    }
    condition_.notify_one();
  }

  bool try_pop(T *t, const std::chrono::microseconds &timeout =
                         std::chrono::microseconds(0)) {
    std::unique_lock<std::mutex> lock(mutex_);

    if (!condition_.wait_for(lock, timeout,
                             [this] { return !queue_.empty(); })) {
      return false;
    }

    *t = queue_.front();
    queue_.pop_front();
    return true;
  }

  // This logs a message if the threads needs to be blocked
  // useful for detecting e.g. when data feeding is too slow
  T pop() {
    std::unique_lock<std::mutex> lock(mutex_);

    condition_.wait(lock, [this] { return !queue_.empty(); });

    T t = queue_.front();
    queue_.pop_front();
    return t;
  }

  T pop_back() {
    std::unique_lock<std::mutex> lock(mutex_);
    int wait_count = 0;
    condition_.wait(lock, [this] { return !queue_.empty(); });

    T t = queue_.back();
    queue_.clear();
    return t;
  }

  bool try_peek(T *t) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (queue_.empty()) {
      return false;
    }

    *t = queue_.front();
    return true;
  }

  // Return element without removing it
  T peek() {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this] { return !queue_.empty(); });

    return queue_.front();
  }

  size_t size() {
    std::lock_guard<std::mutex> lock(mutex_);
    const size_t ret = queue_.size();
    return ret;
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.clear();
  }

 protected:
  std::deque<T> queue_;

  std::mutex mutex_;
  std::condition_variable condition_;

 private:
  BlockingQueue(const BlockingQueue &);
  BlockingQueue &operator=(const BlockingQueue &);
};

}  // namespace vision
}  // namespace horizon
#endif  // HORIZON_VISION_BLOCKING_QUEUE_HPP_
