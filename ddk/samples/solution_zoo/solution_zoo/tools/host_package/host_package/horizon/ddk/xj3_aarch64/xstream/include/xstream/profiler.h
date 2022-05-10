/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @file profiler.h
 * @brief
 * @author ruoting.ding
 * @email ruoting.ding@horizon.ai
 * @date 2019/4/15
 */

#ifndef XSTREAM_PROFILER_H_
#define XSTREAM_PROFILER_H_

#include <atomic>
#include <chrono>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "xstream/version.h"

namespace xstream {
inline std::int64_t getMicroSecond() {
  auto time_now = std::chrono::system_clock::now();
  auto duration_in_ms = std::chrono::duration_cast<std::chrono::microseconds>(
      time_now.time_since_epoch());
  return duration_in_ms.count();
}

struct XSTREAM_EXPORT FpsStatisticInfo {
  /// FPS start time
  std::int64_t pre_time_;
  /// current total count
  std::atomic_int cnt_;

  FpsStatisticInfo() { cnt_ = 0; }
};

struct XSTREAM_EXPORT TimeStatisticInfo {
  /// the interval of Statistical result output
  static int cycle_ms_;
  /// process start time
  std::int64_t pre_time_;

  TimeStatisticInfo() { pre_time_ = 0; }
};

class ProfilerScope;
class Profiler;
typedef std::shared_ptr<Profiler> ProfilerPtr;

class XSTREAM_EXPORT Profiler : public std::enable_shared_from_this<Profiler> {
 public:
  Profiler() = default;
  ~Profiler();

  enum class State { kNotRunning = 0, kRunning };

  static ProfilerPtr Get();
  static ProfilerPtr Release() {
    if (instance_) {
      instance_ = nullptr;
    }
    return instance_;
  }
  inline bool IsRunning() const { return state_ == State::kRunning; }

  void Log(const std::stringstream &ss);

  inline void Start() { SetState(State::kRunning); }
  inline void Stop() { SetState(State::kNotRunning); }

  bool SetOutputFile(const std::string &file);

  void SetIntervalForTimeStat(int cycle_ms_);

  enum class Type { kFps, kProcessTime };
  std::unique_ptr<ProfilerScope> CreateFpsScope(const std::string &name);
  std::unique_ptr<ProfilerScope> CreateTimeScope(const std::string &name,
                                                 int sequence_id);

  std::string name_ = "";
  // thread_id to thread_name
  static std::map<std::thread::id, std::string> thread_id2name_;

 private:
  Profiler(const Profiler &) = delete;
  void SetState(Profiler::State state);
  static ProfilerPtr instance_;
  State state_ = State::kNotRunning;

  std::unordered_map<std::string, std::shared_ptr<FpsStatisticInfo>> fps_stats_;
  std::unordered_map<std::string, std::shared_ptr<TimeStatisticInfo>>
      time_stats_;

  std::fstream foi_;
  std::mutex lock_;
};

class XSTREAM_EXPORT ProfilerScope {
 public:
  ProfilerScope(ProfilerPtr profiler, const std::string &name)
      : profiler_(profiler), name_(name) {}

  virtual ~ProfilerScope() {}

  enum ProFilerType { FPS = 0, TIME };

 protected:
  ProfilerPtr profiler_;
  std::string name_;

  ProFilerType tag_;
};

class XSTREAM_EXPORT ScopeTime : public ProfilerScope {
 public:
  ScopeTime(ProfilerPtr profiler, const std::string &name,
            std::shared_ptr<TimeStatisticInfo> stat, int64_t sequence_id = -1)
      : ProfilerScope(profiler, name), stat_(stat) {
    tag_ = ProFilerType::TIME;
    sequence_id_ = sequence_id;
    if (stat_->pre_time_ == 0) {
      stat_->pre_time_ = getMicroSecond();
    }
  }

  ~ScopeTime() {
    auto cur_time = getMicroSecond();
    auto diff = cur_time - stat_->pre_time_;
    if (diff > stat_->cycle_ms_) {
      std::stringstream ss;
      ss << name_ << " | ";
      if (sequence_id_ < 0) {  // method内部
        auto id = std::this_thread::get_id();
        auto iter = Profiler::thread_id2name_.find(id);
        if (iter != Profiler::thread_id2name_.end()) {
          ss << Profiler::thread_id2name_[id];
        } else {
          ss << std::this_thread::get_id();
        }
      } else {
        ss << "sequencec_id_" << sequence_id_;
      }
      ss << " | "
         << "X"
         << " | " << std::to_string(stat_->pre_time_) << " | "
         << std::to_string(diff) << "\n";
      profiler_->Log(ss);
    }
    stat_->pre_time_ = 0;
  }

 private:
  int64_t sequence_id_;
  std::shared_ptr<TimeStatisticInfo> stat_;
};

class XSTREAM_EXPORT ScopeFps : public ProfilerScope {
 public:
  ScopeFps(ProfilerPtr profiler, const std::string &name,
           std::shared_ptr<FpsStatisticInfo> stat)
      : ProfilerScope(profiler, name), stat_(stat) {
    tag_ = ProFilerType::FPS;
    if (stat_->cnt_ == 0) {
      stat_->pre_time_ = getMicroSecond();
    }
    stat_->cnt_++;
  }

  ~ScopeFps() {
    auto cur_time = getMicroSecond();
    auto diff = cur_time - stat_->pre_time_;
    if (diff > 1000000) {
      std::stringstream ss;
      ss << name_ << " | " << std::this_thread::get_id() << " | "
         << "C"
         << " | " << std::to_string(stat_->pre_time_) << " | "
         << stat_->cnt_ / (diff / 1000.0 / 1000) << "\n";
      ss << name_ << " | " << std::this_thread::get_id() << " | "
         << "C"
         << " | " << std::to_string(cur_time) << " | "
         << "0"
         << "\n";
      profiler_->Log(ss);
      stat_->cnt_ = 0;
    }
  }

 private:
  std::shared_ptr<FpsStatisticInfo> stat_;
};

#define RUN_FPS_PROFILER_WITH_PROFILER(profiler, name) \
  std::unique_ptr<xstream::ProfilerScope> scope_fps;            \
  if (profiler->IsRunning()) {                         \
    scope_fps = profiler->CreateFpsScope(name);        \
  }

#define RUN_TIME_PROFILER_WITH_PROFILER(profiler, name, sequence_id) \
  std::unique_ptr<xstream::ProfilerScope> scope_time;                         \
  if (profiler->IsRunning()) {                                       \
    scope_time = profiler->CreateTimeScope(name, sequence_id);       \
  }

#define RUN_FPS_PROFILER(name) \
  RUN_FPS_PROFILER_WITH_PROFILER(xstream::Profiler::Get(), name)

#define RUN_PROCESS_TIME_PROFILER(name) \
  RUN_TIME_PROFILER_WITH_PROFILER(xstream::Profiler::Get(), name, -1)

}  // namespace xstream
#endif  // XSTREAM_PROFILER_H_
