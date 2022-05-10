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

#include "roi_zoom_plugin/roi_zoom_processor.h"
#include <string>
#include "hobotlog/hobotlog.hpp"
#include "smart_message/smart_message.h"
namespace xproto {

using xproto::message::SmartMessage;

RoiProcessor::RoiProcessor() {
  votmodule_ = std::make_shared<VotModule>();
  vpsmodule_ = std::make_shared<VpsModule>();
  is_running_ = false;
}

int RoiProcessor::Init(const bool enable_vot,
                       const bool enable_intelligent_tracking) {
  enable_vot_ = enable_vot;
  enable_intelligent_tracking_ = enable_intelligent_tracking;
  if (enable_vot_) {
    votmodule_->Init();
  }
  vpsmodule_->Init();
  return 0;
}

int RoiProcessor::Start() {
  if (is_running_) return 0;

  if (enable_vot_) {
    votmodule_->Start();
  }
  vpsmodule_->Start();
  is_running_ = true;
  roi_thread_ = std::make_shared<std::thread>(&RoiProcessor::Process, this);
  thread_pool_ = std::make_shared<ThreadPool>();
  if (!thread_pool_->Start(3)) {
    LOGE << "RoiProcessor start thread pool fail";
    return -1;
  }

  return 0;
}

int RoiProcessor::Stop() {
  is_running_ = false;
  LOGI << "usb roi RoiProcessor Stop begin";
  thread_pool_->Stop();
  condition_.notify_all();
  if (roi_thread_->joinable()) {
    roi_thread_->join();
  }

  LOGI << "roi feed thread stop done";
  vpsmodule_->Stop();
  if (enable_vot_) {
    LOGI << "roi vot stop";
    votmodule_->Stop();
  }

  video_list_.clear();
  LOGI << "usb roi RoiProcessor Stop end !!!";
  return 0;
}

int RoiProcessor::DeInit() {
  LOGI << "usb roi RoiProcessor DeInit begin";
  vpsmodule_->DeInit();
  if (enable_vot_) {
    votmodule_->DeInit();
  }
  LOGI << "usb roi RoiProcessor DeInit end !!!";
  return 0;
}

int RoiProcessor::Input(std::shared_ptr<VideoRoiData> video_data) {
  if (!is_running_) return 0;
  if (video_data->y_virtual_addr == NULL ||
      video_data->uv_virtual_addr == NULL) {
    return 0;
  }
  if (video_data->channel == 0) {
    votmodule_->Input(video_data);
    return 0;
  }

  std::lock_guard<std::mutex> lg(mut_cache_);
  uint64_t frame_id = video_data->frame_id;
  video_list_[frame_id] = video_data;
  condition_.notify_one();
  LOGV << "cache_vio_smart_ size:" << video_list_.size();
  return 0;
}

int RoiProcessor::Process() {
  while (is_running_) {
    std::map<uint64_t, std::shared_ptr<VideoRoiData> >::iterator front;
    std::shared_ptr<VideoRoiData> video_data = nullptr;

    std::unique_lock<std::mutex> lg(mut_cache_);
    condition_.wait(lg,
                    [this]() { return !is_running_ || !video_list_.empty(); });

    if (!is_running_) {
      video_list_.clear();
      break;
    }

    if (video_list_.empty()) {
      continue;
    }
    front = video_list_.begin();
    {
      video_data = front->second;
      video_list_.erase(front);
      lg.unlock();
      ProcessData(video_data);
    }

    if (video_list_.size() >= cache_len_limit_) {
      // exception occurred
      lg.lock();
      front = video_list_.begin();
      video_data = front->second;
      video_list_.erase(front);
      lg.unlock();
    }
  }

  return 0;
}

int RoiProcessor::ProcessData(std::shared_ptr<VideoRoiData> data) {
  if (data->channel == 0 && enable_vot_) {
    votmodule_->Input(data);
    return 0;
  }

  std::vector<HorizonVisionBBox> body_boxs;
  std::vector<HorizonVisionBBox> hand_boxs;

  for (uint32_t i = 0; i < data->smart_frame->smart_data_list_num; ++i) {
    const auto &s_data = data->smart_frame->smart_data_list[i];
    if (s_data.face) {
      const auto &rect = s_data.face->face_rect;
      if (rect.score > 0.98) body_boxs.push_back(rect);
    }

    if (s_data.hand && enable_intelligent_tracking_) {
      const auto &rect = s_data.hand->hand_rect;
      // if (s_data.hand->hand_gesture == 4) {
        if (rect.score > 0.92) hand_boxs.push_back(rect);
      // }
    }
  }

  if (body_boxs.empty()) {
    LOGD << " Body box size is 0 !!!";
    // return 0;
  }

  auto roi_view = roi_calc_.GetViewRoiBox(body_boxs);
  LOGD << roi_view.x << ' ' << roi_view.y << ' ' << roi_view.width << ' '
       << roi_view.height;
  vpsmodule_->UpdateViewRoi(roi_view);
  vpsmodule_->Input(data);
  thread_pool_->AddTask(
      std::bind(&RoiProcessor::GetFrameData, this, true, false));

  if (!enable_intelligent_tracking_) return 0;

  // if (hand_boxs.empty())
  //   return 0;

  auto roi_track = roi_calc_.GetTrackRoiBox(hand_boxs, body_boxs);
  LOGD << roi_track.size();
  int size = roi_track.size();
  if (size == 0) {
    LOGW << "roi track size is 0 !!!";
    return 0;
  }

  RoiInfo *info_tmp = nullptr;
  if (roi_track[0].size() > 1) {
    info_tmp = &roi_track[0][1];
  }

  vpsmodule_->UpdateTrackRoi(roi_track[0][0], info_tmp);
  thread_pool_->AddTask(
      std::bind(&RoiProcessor::GetFrameData, this, false, false));

  if (size > 1) {
    RoiInfo *info_tmp = nullptr;
    if (roi_track[1].size() > 1) {
      info_tmp = &roi_track[1][1];
    }
    vpsmodule_->UpdateTrackRoi(roi_track[1][0], info_tmp, true);
    thread_pool_->AddTask(
        std::bind(&RoiProcessor::GetFrameData, this, false, true));
  }
  return 0;
}

int RoiProcessor::ProcessRoi(std::shared_ptr<VideoRoiData> data,
                             std::shared_ptr<VideoRoiData> out_data,
                             RoiInfo &info, RoiInfo *tmp_ino,
                             const bool track_second, const bool send_video) {
  int nRet = vpsmodule_->Process(data, out_data, info, tmp_ino, track_second,
                                 send_video);
  if (nRet == 0 && enable_vot_) {
    votmodule_->Input(out_data);
  }

  return 0;
}

int RoiProcessor::GetFrameData(const bool view_data, const bool track_second) {
  std::shared_ptr<VideoRoiData> roi_data = std::make_shared<VideoRoiData>();
  int nRet = 0;
  if (view_data) {
    roi_data->channel = 1;
    nRet = vpsmodule_->OutputViewData(roi_data);
    if (nRet == 0 && enable_vot_) {
      votmodule_->Input(roi_data);
    }
    return 0;
  }

  roi_data->channel = track_second ? 3 : 2;
  nRet = vpsmodule_->OutputTrackData(roi_data, track_second);
  if (nRet == 0 && enable_vot_) {
    votmodule_->Input(roi_data);
  }
  return 0;
}
}  // namespace xproto
