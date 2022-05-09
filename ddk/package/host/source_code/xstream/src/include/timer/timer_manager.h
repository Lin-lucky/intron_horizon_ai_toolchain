/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @brief     timer functions
 * @author    wenhao.zou
 * @email     wenhao.zou@horizon.ai
 * @version   0.0.0.1
 * @date      2020.02.22
 */

#ifndef XSTREAM_FRAMEWORK_INCLUDE_TIMER_TIMER_MANAGER_H_
#define XSTREAM_FRAMEWORK_INCLUDE_TIMER_TIMER_MANAGER_H_

#include <signal.h>
#include <functional>
#include <list>
#include <mutex>
#include <thread>
#include <vector>
#include <memory>
#include <map>
#include "common/rw_mutex.h"
#include "timer_task.h"

namespace xstream {

class TimerTask;

class TimerManager {
 public:
  explicit TimerManager(int singno);

  static uint64_t GetCurrentMillisecs();

  void DetectTimers();

  int ProcessSignHandler(siginfo_t *si);

 private:
  friend class TimerTask;

  void AddTimer(std::shared_ptr<TimerTask> timertask);

  void RemoveTimer(TimerTask* timertask);

  int Cascade(int offset, int index);

 protected:
  int timer_signo_;

 private:
//  typedef std::list<TimerTask *> TimeList;
//  std::vector<TimeList> tvec_;
//  uint64_t check_time_;

  mutable RWLock timer_tb_mutex_;

  std::map<TimerTask::TimerTaskID, std::shared_ptr<TimerTask>> timer_tb_;
};

}  // namespace xstream
#endif  // XSTREAM_FRAMEWORK_INCLUDE_TIMER_TIMER_MANAGER_H_
