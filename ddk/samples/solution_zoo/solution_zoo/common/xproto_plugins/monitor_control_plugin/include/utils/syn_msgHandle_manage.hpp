/*
 * @Description:
 * @Author: xx@horizon.ai
 * @Date: 2020-06-22 16:17:25
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */
#include <unistd.h>
#include <iostream>
#include <string>
#include <memory>
#include <future>
#include <utility>
#include <unordered_map>
#include "hobotlog/hobotlog.hpp"

#ifndef INCLUDE_UTILS_SYN_MSGHANDLE_MANAGE_HPP_
#define INCLUDE_UTILS_SYN_MSGHANDLE_MANAGE_HPP_

template <class SynMsgType>
class SynMsgHandleManage {
  /*
   * usage:
   * 1. GenMsgId() AddPromise() GetFuture()
   * 2. now is block-state and wait response
   * 3. SetPromise() if get response
   * 4. DelPromise()
   * */

 public:
  static SynMsgHandleManage& GetInstance() {
    static SynMsgHandleManage instance;
    return instance;
  }

  uint64_t GenMsgId() {
    std::lock_guard<std::mutex> lock(mtx_);
    return ++msg_id_;
  }

  int AddPromise(uint64_t msg_id) {
    std::unique_lock<std::mutex> lock(mtx_);
    msg_time_[msg_id] =
            std::chrono::high_resolution_clock::now();

    if (promise_.find(msg_id) != promise_.end()) {
      lock.unlock();
      LOGE << "msg already exist id:" << msg_id;
      return -1;
    }
    promise_.insert(std::make_pair(msg_id, std::promise<SynMsgType>()));
    auto promise_size = promise_.size();
    lock.unlock();
    LOGI << "promise size:" << promise_size;
    return 0;
  }

  int DelPromise(uint64_t msg_id) {
    {
      std::unique_lock<std::mutex> lock(mtx_);
      if (msg_time_.find(msg_id) == msg_time_.end()) {
        lock.unlock();
        LOGE << "msg not exist id:" << msg_id;
        return -1;
      } else {
        std::chrono::duration<float, std::milli> dur =
                std::chrono::high_resolution_clock::now() - msg_time_[msg_id];
        LOGD << "msg_id:" << msg_id << "  dur:" << dur.count() << " ms";
      }
      msg_time_.erase(msg_id);
      lock.unlock();
    }

    LOGD << "del msg_id:" << msg_id;
    std::unique_lock<std::mutex> lock(mtx_);
    if (promise_.find(msg_id) == promise_.end()) {
      lock.unlock();
      LOGE << "msg not exist id:" << msg_id;
      return -1;
    }
    promise_.erase(msg_id);
    lock.unlock();
    return 0;
  }

  int SetPromise(uint64_t msg_id, SynMsgType val) {
    std::unique_lock<std::mutex> lock(mtx_);
    if (promise_.find(msg_id) == promise_.end()) {
      lock.unlock();
      LOGE << "msg not exist id:" << msg_id;
      return -1;
    }
    promise_.at(msg_id).set_value(val);
    lock.unlock();
    return 0;
  }

  int GetFuture(uint64_t msg_id, SynMsgType& val, int timeout_ms = -1) {
    std::unique_lock<std::mutex> lock(mtx_);
    if (promise_.find(msg_id) == promise_.end()) {
      lock.unlock();
      LOGE << "msg not exist id:" << msg_id;
      return -1;
    }
    auto& prom = promise_.at(msg_id);
    lock.unlock();

    auto&& fut = prom.get_future();
    if (timeout_ms > 0) {
      if (std::future_status::ready ==
              fut.wait_for(std::chrono::milliseconds(timeout_ms))) {
        val = fut.get();
      } else {
        LOGE << "wait msg response timeout id:" << msg_id;
        return -1;
      }
    } else {
      val = fut.get();
    }
    return 0;
  }

 private:
  SynMsgHandleManage() {}
  ~SynMsgHandleManage() {}

 private:
  std::unordered_map<uint64_t, std::promise<SynMsgType>> promise_;
  std::unordered_map<uint64_t,
          std::chrono::high_resolution_clock::time_point> msg_time_;

  uint64_t msg_id_ = 0;
  std::mutex mtx_;
};

#endif  // INCLUDE_UTILS_SYN_MSGHANDLE_MANAGE_HPP_
