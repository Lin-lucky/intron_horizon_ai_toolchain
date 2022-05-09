//
// Created by Minghao Wang on 17-9-26.
// Copyright (c) 2017 Horizon Robotics. All rights reserved.
//

#ifndef XPROTO_SRC_INCLUDE_XPROTO_UTILS_RINGQUEUE_H_
#define XPROTO_SRC_INCLUDE_XPROTO_UTILS_RINGQUEUE_H_
#include <stdio.h>
#include <stdlib.h>

#include <condition_variable>
#include <deque>
#include <functional>
#include <iostream>
#include <mutex>
#include <utility>

#include "xproto/utils/singleton.h"

namespace xproto {

template <typename Dtype>
class RingQueue {
 public:
  using ReleaseDataFunc = std::function<void(Dtype& elem)>;
  RingQueue() { stop_ = false; }
  int Init(typename std::deque<Dtype>::size_type size,
           ReleaseDataFunc release_func) {
    if (!size_) {
      size_ = size;
    }
    release_func_ = release_func;
    return 0;
  }

  void UnInit() {
    stop_ = true;
    cv_.notify_all();
    Clear();
  }

  void Push(const Dtype& elem) {
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (size_ <= 0 || stop_) {
        return;
      }
      if (que_.size() >= size_) {
        auto e = que_.front();
        if (release_func_) {
          release_func_(e);
        }
        que_.pop_front();
      }
      que_.push_back(elem);
    }
    cv_.notify_all();
  }

  void PushMove(const Dtype& elem) {
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (size_ <= 0 || stop_) {
        return;
      }
      if (que_.size() >= size_) {
        auto e = que_.front();
        if (release_func_) {
          release_func_(e);
        }
        que_.pop_front();
      }
      que_.push_back(std::move(elem));
    }
    cv_.notify_all();
  }

  bool Pop(Dtype& elem) {
    std::unique_lock<std::mutex> lock(mtx_);
    cv_.wait(lock, [this] { return (!que_.empty() || stop_); });
    if (stop_) {
      return false;
    }
    auto e = que_.front();
    que_.pop_front();
    elem = std::move(e);
    return true;
  }

  bool Empty() {
    std::lock_guard<std::mutex> l(mtx_);
    return que_.empty();
  }

  bool IsValid() {
    std::lock_guard<std::mutex> l(mtx_);
    // may push twice at one time
    return (size_ > 0 && que_.size() < size_);
  }

  void Clear() {
    std::lock_guard<std::mutex> l(mtx_);
    while (!que_.empty()) {
      auto elem = que_.front();
      if (release_func_) {
        release_func_(elem);
      }
      que_.pop_front();
    }
  }

 private:
  typename std::deque<Dtype>::size_type size_ = 0;
  std::deque<Dtype> que_;
  std::mutex mtx_;
  std::condition_variable cv_;
  ReleaseDataFunc release_func_;
  std::atomic<bool> stop_;
};
}  // namespace xproto
#endif  // XPROTO_SRC_INCLUDE_XPROTO_UTILS_RINGQUEUE_H_
