/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @brief     timer functions
 * @author    wenhao.zou
 * @email     wenhao.zou@horizon.ai
 * @version   0.0.0.1
 * @date      2020.02.22
 */

#ifndef XSTREAM_FRAMEWORK_INCLUDE_TIMER_TIMER_H_
#define XSTREAM_FRAMEWORK_INCLUDE_TIMER_TIMER_H_

#include <signal.h>
#include <functional>
#include <list>
#include <mutex>
#include <thread>
#include <vector>
#include <memory>
#include <map>
#include "common/rw_mutex.h"
#include "timer_manager.h"
#include "timer_task.h"

namespace xstream {
#ifdef __USE_ISOC99
  static const int64_t kNumMillisecsPerSec = int_least64_t(1000);
  static const int64_t kNumMicrosecsPerSec = int_least64_t(1000000);
  static const int64_t kNumNanosecsPerSec = int_least64_t(1000000000);
#else
  static const int64_t kNumMillisecsPerSec = INT64_C(1000);
  static const int64_t kNumMicrosecsPerSec = INT64_C(1000000);
  static const int64_t kNumNanosecsPerSec = INT64_C(1000000000);
#endif
  static const int64_t kNumMicrosecsPerMillisec =
      kNumMicrosecsPerSec / kNumMillisecsPerSec;
  static const int64_t kNumNanosecsPerMillisec =
      kNumNanosecsPerSec / kNumMillisecsPerSec;
  static const int64_t kNumNanosecsPerMicrosec =
      kNumNanosecsPerSec / kNumMicrosecsPerSec;

class Timer {
 public:
  static Timer *Instance();

  void *AddTimer(std::function<void(void*)> callback, uint32_t interval,
                void* userdata,
                TimerTask::TimerType timeType = TimerTask::ONCE);

  void RemoveTimer(void *&ptr);

  static void TimerSignHandler(int sig, siginfo_t *si, void *uc);

  void ProcessSignHandler(siginfo_t *si);

 private:
  explicit Timer(int singno = SIGRTMIN);

  static Timer *inst_;

 private:
  bool is_stop_;
  TimerManager timer_manager_;
  static std::mutex mutex_;
  std::thread *thread_ptr_;
  int timer_signo_;
};
}  // namespace xstream
#endif  // XSTREAM_FRAMEWORK_INCLUDE_TIMER_TIMER_H_
