/**
 * Copyright (c) 2020, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: xue.liang
 * @Mail: xue.liang@horizon.ai
 * @Date: 2020-12-15 20:38:52
 * @Version: v0.0.1
 * @Brief: roi_zoom_plugin impl based on xpp.
 * @Last Modified by: xue.liang
 * @Last Modified time: 2020-12-16 22:41:30
 */

#ifndef INCLUDE_ROI_ZOOM_PLUGIN_ROI_ZOOM_PROCESSOR_H_
#define INCLUDE_ROI_ZOOM_PLUGIN_ROI_ZOOM_PROCESSOR_H_
#include <future>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "blocking_queue/blocking_queue.hpp"
#include "smart_message/smart_message.h"
#include "roi_zoom_plugin/thread_pool.h"
#include "roi_zoom_plugin/roi_zoom_common.h"
#include "roi_zoom_plugin/roi_calc.h"
#include "roi_zoom_plugin/vot_module.h"
#include "roi_zoom_plugin/vps_module.h"

namespace xproto {
class RoiProcessor {
 public:
  static std::shared_ptr<RoiProcessor>& Instance() {
    static std::shared_ptr<RoiProcessor> processor;
    static std::once_flag init_flag;
    std::call_once(init_flag, []() {
      processor = std::shared_ptr<RoiProcessor>(new RoiProcessor());
    });
    return processor;
  }

  ~RoiProcessor() = default;
  int SetConfig(RoiCalcConfig& config) {
    return roi_calc_.UpdateConfig(config);
  }
  int Init(const bool enable_vot,
           const bool enable_intelligent_tracking = false);
  int Start();
  int Stop();
  int Input(std::shared_ptr<VideoRoiData> data);
  int DeInit();

 private:
  RoiProcessor();
  int Process();
  int ProcessData(std::shared_ptr<VideoRoiData> data);
  int ProcessRoi(std::shared_ptr<VideoRoiData> data,
                 std::shared_ptr<VideoRoiData> out_data, RoiInfo& info,
                 RoiInfo* tmp_ino = nullptr, const bool track_second = false,
                 const bool send_video = true);

  int GetFrameData(const bool view_data = true,
                   const bool track_second = false);

 private:
  std::atomic<bool> is_running_;
  bool enable_vot_ = false;
  bool enable_intelligent_tracking_ = false;
  // hobot::vision::BlockingQueue<std::shared_ptr<VideoRoiData>> video_list_;
  std::condition_variable condition_;
  std::map<uint64_t, std::shared_ptr<VideoRoiData>> video_list_;
  std::mutex mut_cache_;
  uint64_t cache_len_limit_ = 40;

  std::shared_ptr<std::thread> roi_thread_;
  std::shared_ptr<VotModule> votmodule_;
  std::shared_ptr<VpsModule> vpsmodule_;
  SmartCalcRoi roi_calc_;

  PThreadPool thread_pool_;
};

}  // namespace xproto
#endif  // INCLUDE_ROI_ZOOM_PLUGIN_ROI_ZOOM_PROCESSOR_H_
