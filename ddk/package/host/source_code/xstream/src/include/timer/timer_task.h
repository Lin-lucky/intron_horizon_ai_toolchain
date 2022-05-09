/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @brief     timer functions
 * @author    wenhao.zou
 * @email     wenhao.zou@horizon.ai
 * @version   0.0.0.1
 * @date      2020.02.22
 */

#ifndef XSTREAM_FRAMEWORK_INCLUDE_TIMER_TIMER_TASK_H_
#define XSTREAM_FRAMEWORK_INCLUDE_TIMER_TIMER_TASK_H_

#include <signal.h>
#include <functional>
#include <list>
#include <mutex>
#include <thread>
#include <vector>
#include <memory>
#include <map>
#include "common/rw_mutex.h"


namespace xstream {

class TimerManager;

class TimerTask {
 public:
  enum TimerType { ONCE, CIRCLE };

  explicit TimerTask(TimerManager &manager);

  ~TimerTask();

  void Start(std::function<void(void*)> fun, unsigned interval,
             void *userdata_,
             TimerType timeType = ONCE);

  void Stop();

  typedef timer_t TimerTaskID;
  TimerTaskID TimerID();

 private:
  void OnTimer(uint64_t now);
  friend class TimerManager;
  TimerManager &manager_;
  TimerType timer_type_;
  std::function<void(void*)> callback_fun_;
  uint32_t interval_;
  uint64_t expires_;
  int vecIndex_;
  std::list<TimerTask *>::iterator itr_;
  TimerTaskID timer_id_;
  const clockid_t clock_id_;
  const int timer_signo_;
  void *userdata_;
  bool is_stop;
};
}  // namespace xstream
#endif  // XSTREAM_FRAMEWORK_INCLUDE_TIMER_TIMER_TASK_H_
