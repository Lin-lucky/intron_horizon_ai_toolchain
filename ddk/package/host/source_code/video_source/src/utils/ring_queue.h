/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef VIDEO_SOURCE_COMMON_RING_QUEUE_H_
#define VIDEO_SOURCE_COMMON_RING_QUEUE_H_
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include "hobotlog/hobotlog.hpp"

namespace videosource {

template <typename Dtype>
class RingQueue {
 public:
  using ReleaseDataFunc = std::function<void(Dtype& elem)>;
  RingQueue() {}
  int Init(typename std::deque<Dtype>::size_type size,
           ReleaseDataFunc release_func) {
    if (!size_) {
      LOGD << "Try to construct empty RingQueue";
      size_ = size;
    }
    release_func_ = release_func;
    return 0;
  }

  void Push(const Dtype& elem) {
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (size_ <= 0) {
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

  bool Pop(Dtype &video_buffer) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!que_.empty()) {
      video_buffer = que_.front();
      que_.pop_front();
    } else {
      return false;
    }
    return true;
  }

  bool Peek(Dtype &video_buffer) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!que_.empty()) {
      video_buffer = que_.front();
    } else {
      return false;
    }
    return true;
  }

  bool Empty() {
    std::lock_guard<std::mutex> l(mtx_);
    return que_.empty();
  }

  bool IsValid() {
    std::lock_guard<std::mutex> l(mtx_);
    return (size_ > 0 && que_.size() < size_);
  }

  bool IsFull() {
    std::lock_guard<std::mutex> l(mtx_);
    return (que_.size() >= size_);
  }

  int Size() {
    std::lock_guard<std::mutex> l(mtx_);
    return que_.size();
  }

  bool DeleteOne() {
    if (size_ <= 0) {
      return false;
    }
    auto elem = que_.front();
    if (release_func_) {
      release_func_(elem);
    }
    que_.pop_front();
    return true;
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
};

}  // namespace videosource
#endif  // VIDEO_SOURCE_COMMON_RING_QUEUE_H_
