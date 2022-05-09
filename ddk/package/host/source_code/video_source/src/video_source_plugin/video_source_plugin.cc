/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include "video_source_plugin/video_source_plugin.h"
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <map>
#include <algorithm>
#include "video_source_plugin/video_source_config.h"
#include "video_source_plugin/video_source_produce.h"
#include "video_source_plugin/video_source_message.h"
#include "utils/blocking_queue.h"
#include "hobotlog/hobotlog.hpp"

XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_IMAGE_MESSAGE)
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_DROP_MESSAGE)
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_DROP_IMAGE_MESSAGE)
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_MULTI_IMAGE_MESSAGE)
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_FACE_PIC_IMAGE_MESSAGE)
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_INFO_IMAGE_MESSAGE)

namespace xproto {

VideoSourcePlugin::VideoSourcePlugin(const std::string &path) {
  config_ = GetConfigFromFile(path);
  HOBOT_CHECK(config_);
  GetSubConfigs();
}

void VideoSourcePlugin::GetJ3devConfigs(
    ProduceConfig &produce_cfg,
    const std::shared_ptr<VideoSourceConfig> &config_json) {
  std::vector<int> channel_id_list;
  if (config_json == nullptr) {
    LOGE << "config_json is nullptr";
    return;
  }

  if (config_json->HasMember("data_source_num")) {
    data_source_num_ = config_json->GetIntValue("data_source_num");
  }
  if (config_json->HasMember("channel_id")) {
    channel_id_list = config_json->GetIntArrayItem("channel_id");
  }
  int channel_num = static_cast<int>(channel_id_list.size());
  HOBOT_CHECK(data_source_num_ == channel_num);
  if (config_json->HasMember("video_source_file")) {
    produce_cfg.cfg_file = config_json->GetStringValue("video_source_file");
  }
  for (size_t index = 0; index < channel_id_list.size(); index++) {
    auto cfg = produce_cfg;
    auto channel_id = channel_id_list[index];
    cfg.channel_id = channel_id;
    chn_id_2_index_[channel_id] = index;
    produce_cfg_list_.push_back(cfg);
  }
  for (auto cfg : produce_cfg_list_) {
    LOGI << " channel_id: " << cfg.channel_id
      << "  data_source_num: " << data_source_num_
      << "  board_name:" << cfg.board_name
      << "  produce_name:" << cfg.produce_name
      << "  max_vio_buffer:" << cfg.max_vio_buffer
      << "  is_msg_package:" << cfg.is_msg_package
      << "  is_msg_package:" << cfg.is_msg_order
      << "  max_ts_compare_value:" << cfg.max_ts_cmp
      << "  video_source_file:" << cfg.cfg_file;
  }
}

void VideoSourcePlugin::GetSubConfigs() {
  int chret = 0;
  int config_index = 0;

  ProduceConfig produce_cfg;
  if (config_->HasMember("config_index")) {
    config_index = config_->GetIntValue("config_index");
  }
  if (config_->HasMember("board_name")) {
    produce_cfg.board_name = config_->GetStringValue("board_name");
  }

  std::string config_index_name = "config_" + std::to_string(config_index);
  auto config_json = config_->GetSubConfig(config_index_name);
  HOBOT_CHECK(config_json);

  // parse produce config
  if (config_json->HasMember("produce_name")) {
    produce_cfg.produce_name = config_json->GetStringValue("produce_name");
  }
  if (config_json->HasMember("data_type")) {
    produce_cfg.data_type = config_json->GetStringValue("data_type");
  }
  if (config_json->HasMember("max_vio_buffer")) {
    produce_cfg.max_vio_buffer = config_json->GetIntValue("max_vio_buffer");
  }
  if (config_json->HasMember("is_msg_package")) {
    produce_cfg.is_msg_package = config_json->GetIntValue("is_msg_package");
    is_sync_mode_ = produce_cfg.is_msg_package;
  }
  if (config_json->HasMember("is_msg_order")) {
    produce_cfg.is_msg_order = config_json->GetIntValue("is_msg_order");
    is_order_mode_ = produce_cfg.is_msg_order;
  }
  if (config_json->HasMember("max_ts_compare")) {
    produce_cfg.max_ts_cmp = config_json->GetIntValue("max_ts_compare");
    max_ts_cmp_ = produce_cfg.max_ts_cmp;
  }

  // judge board name
  if (produce_cfg.board_name == "j3dev") {
    LOGI << "enter get j3dev configs";
    GetJ3devConfigs(produce_cfg, config_json);
    return;
  } else if (produce_cfg.board_name == "x3dev") {
  } else {
    LOGE << "Unsupport board_name: " << produce_cfg.board_name;
    return;
  }

  // parse data type config
  std::shared_ptr<VideoSourceConfig> type_json;
  if (produce_cfg.data_type == "mono") {
    type_json = config_json->GetSubConfig("mono");
    HOBOT_CHECK(type_json);
    if (type_json->HasMember("channel_id")) {
      produce_cfg.channel_id = type_json->GetIntValue("channel_id");
    }
    if (type_json->HasMember("video_source_file")) {
      produce_cfg.cfg_file = type_json->GetStringValue("video_source_file");
    }
    produce_cfg_list_.push_back(produce_cfg);
    data_source_num_ = 1;
  } else if (produce_cfg.data_type == "dual") {
    type_json = config_json->GetSubConfig("dual");
    HOBOT_CHECK(type_json);
    std::vector<int> channel_id_list;
    std::vector<std::string> cfg_files;
    if (type_json->HasMember("channel_id")) {
      channel_id_list = type_json->GetIntArrayItem("channel_id");
    }
    if (type_json->HasMember("video_source_file")) {
      cfg_files = type_json->GetStringArrayItem("video_source_file");
    }
    HOBOT_CHECK(channel_id_list.size() == cfg_files.size());
    data_source_num_ = static_cast<int>(cfg_files.size());
    HOBOT_CHECK(data_source_num_ > 1);
    for (int i = 0; i < data_source_num_; i++) {
      auto cfg = produce_cfg;
      cfg.channel_id = channel_id_list[i];
      cfg.cfg_file = cfg_files[i];
      produce_cfg_list_.push_back(cfg);
    }
  } else if (produce_cfg.data_type == "multi_mix") {
    auto js = config_json->GetJson();
    auto value_js = js["multi_mix"];
    data_source_num_ = static_cast<int>(value_js.size());
    for (int i = 0; i < data_source_num_; i++) {
      auto value = value_js[i];
      auto cfg = produce_cfg;
      cfg.produce_name = value["produce_name"].asString();
      cfg.max_vio_buffer = value["max_vio_buffer"].asInt();
      cfg.channel_id = value["channel_id"].asInt();
      cfg.cfg_file = value["video_source_file"].asString();
      produce_cfg_list_.push_back(cfg);
    }
  } else {
    LOGE << "unsupported data type: " << produce_cfg.data_type;
    chret = -1;
  }
  HOBOT_CHECK(chret == 0);

  LOGI << "config_index: " << config_index;
  for (int i = 0; i < data_source_num_; i++) {
    auto &cfg = produce_cfg_list_[i];
    LOGI << "produce_cfg index:" << i
      << "  board_name:" << cfg.board_name
      << "  produce_name:" << cfg.produce_name
      << "  data_type:" << cfg.data_type
      << "  max_vio_buffer:" << cfg.max_vio_buffer
      << "  is_msg_package:" << cfg.is_msg_package
      << "  max_ts_compare_value:" << cfg.max_ts_cmp
      << "  channel_id:" << cfg.channel_id
      << "  video_source_file:" << cfg.cfg_file;
  }
}

void VideoSourcePlugin::SetLoggingLevel(const std::string &log_level) {
  for (int i = 0; i < data_source_num_; i++) {
    auto &cfg = produce_cfg_list_[i];
    cfg.log_level = log_level;
  }
}

int VideoSourcePlugin::Init() {
  int ret = -1;
  if (is_inited_)
    return 0;
  for (int i = 0; i < data_source_num_; i++) {
    auto &cfg = produce_cfg_list_[i];
    std::string produce_name = cfg.produce_name;
    int channel_id = cfg.channel_id;
    std::string cfg_file = cfg.cfg_file;
    auto handle = VideoSourceProduce::CreateVideoSourceProduce(produce_name,
        channel_id, cfg_file);
    HOBOT_CHECK(handle);
    ret = handle->SetConfig(cfg);
    HOBOT_CHECK(ret == 0);
    handle->SetConfigNum(data_source_num_);
    produce_handles_.push_back(handle);
  }
  // 调用父类初始化成员函数注册信息
  XPluginAsync::Init();
  is_inited_ = true;
  return 0;
}

int VideoSourcePlugin::DeInit() {
  produce_handles_.clear();
  is_inited_ = false;
  XPluginAsync::DeInit();
  return 0;
}

VideoSourcePlugin::~VideoSourcePlugin() {}

Listener VideoSourcePlugin::SetListenerFunc() {
  if (data_source_num_ > 1) {
    img_msg_queue_list_.resize(data_source_num_);
    for (int i = 0; i < data_source_num_; i++) {
      channel_status_[i] = 1;
      img_msg_queue_list_[i] =
        std::make_shared<BlockingQueue<std::shared_ptr<VioMessage>>>();
    }
  }
  auto send_frame = [&](const std::shared_ptr<VioMessage> input) {
    std::lock_guard<std::mutex> lk(pym_img_mutex_);
    if (!input) {
      LOGE << "VioMessage is NULL, return";
      return -1;
    }
    // single vio message
    if (data_source_num_ <= 1 || is_order_mode_ == false) {
      PushMsg(input);
      return 0;
    }
    // multi vio message
    if (is_running_ == false) {
      ClearAllQueue();
      return 0;
    }
    {
      int channel_id = input->channel_;
      int chn_index = chn_id_2_index_[channel_id];
      size_t max_msg_sync_size = produce_cfg_list_[chn_index].max_vio_buffer;
      while (img_msg_queue_list_[chn_index]->size() >= max_msg_sync_size) {
        img_msg_queue_list_[chn_index]->pop();
      }
      LOGD << "channel_id: " << channel_id << " push viomessage";
      img_msg_queue_list_[chn_index]->push(input);
    }
    if (!is_sync_mode_) {
      SyncPymImage();
    } else {
      SyncPymImages();
    }
    return 0;
  };

  return send_frame;
}

void VideoSourcePlugin::SyncPymImages() {
  std::vector<std::pair<uint64_t, std::shared_ptr<VioMessage>>>
    send_vio_msg_list;

  // check all channel have image message
  for (int i = 0; i < data_source_num_; i++) {
    auto chn_msg_queue = img_msg_queue_list_[i];
    if (chn_msg_queue->size()) {
      continue;
    } else {
      return;
    }
  }

  for (int i = 0; i < data_source_num_; i++) {
    auto chn_msg_queue = img_msg_queue_list_[i];
    auto &msg_list = chn_msg_queue->get_data();
    // debug print all vio message in BlockingQueue
    for (auto it = msg_list.begin(); it != msg_list.end();) {
      std::shared_ptr<VioMessage> vio_msg = nullptr;
      vio_msg = *it;
      auto channel_id = vio_msg->channel_;
      auto frame_id = vio_msg->sequence_id_;
      auto time_stamp = vio_msg->time_stamp_;
      LOGD << "SyncPymImages index: " << i
        << " channel_id: " << channel_id
        << " frame_id: " << frame_id
        << " time_stamp: " << time_stamp;
      it++;
    }
    // pop first vio message in BlockingQueue
    std::shared_ptr<VioMessage> vio_msg = nullptr;
    vio_msg = chn_msg_queue->pop();
    auto time_stamp = vio_msg->time_stamp_;
    HOBOT_CHECK(vio_msg);
    send_vio_msg_list.push_back(std::make_pair(time_stamp, vio_msg));
  }

  if (max_ts_cmp_) {
    // timestamp compare in different channel
    // time_stamp sort
    auto vio_msg_list_sort(send_vio_msg_list);
    std::sort(vio_msg_list_sort.begin(), vio_msg_list_sort.end(),
        cmp_time_stamp);
    auto max_index = data_source_num_ - 1;
    auto max_time_diff = vio_msg_list_sort[0].first -
      vio_msg_list_sort[max_index].first;
    if (max_time_diff > max_ts_cmp_) {
      LOGW << "sync pym images failed, msg drop..."
        << " max_time_diff: " << max_time_diff
        << " max_ts_compare_value: " << max_ts_cmp_;
      return;
    }
    LOGD << "sync pym images success, msg send..."
      << " max_time_diff: " << max_time_diff
      << " max_ts_compare_value: " << max_ts_cmp_;
  }
  std::shared_ptr<MultiVioMessage> multi_vio_msg(new MultiVioMessage());
  for (int i = 0; i < data_source_num_; i++) {
    auto vio_msg = send_vio_msg_list[i].second;
    multi_vio_msg->multi_vio_img_.push_back(vio_msg);
  }
  PushMsg(multi_vio_msg);
}

void VideoSourcePlugin::SyncPymImage() {
  bool reset = true;
  for (auto itr = channel_status_.begin(); itr != channel_status_.end();
       ++itr) {
    int channel_id = itr->first;
    int send_status = itr->second;
    int chn_index = chn_id_2_index_[channel_id];
    if (send_status) {
      /**
       * 1. if send status is true, mean this channel msg will be sent out soon.
       * 2. after a channel send a msg, send status of this channel will be close
       *    until other all channel send ok which will trigger reset flag be set as
       *    true status.
       * 3. generally channel msg order is [0,1,2,3] or [1,0,3,2] and so on,
       *    all channel order is not repeat.
       * 4. if order is [0,1,1,2], send will be waiting until all
       *    send channel is not same, then reset flag be set true
       */
      reset = false;
      if (img_msg_queue_list_[chn_index]->size() > 0) {
        auto vio_msg = img_msg_queue_list_[chn_index]->pop();
        PushMsg(vio_msg);
        LOGD << "send vio message, channel_id:" << channel_id
             << " frame_id:" << vio_msg->sequence_id_
             << " timestamp:" << vio_msg->time_stamp_;
        itr->second = 0;  // this channel send status close
      }
    }
  }
  if (reset) {
    for (auto itr = channel_status_.begin(); itr != channel_status_.end();
         ++itr) {
      itr->second = 1;  // all channel send status open
      int channel_id = itr->first;
      int chn_index = chn_id_2_index_[channel_id];
      if (img_msg_queue_list_[chn_index]->size() > 0) {
        auto vio_msg = img_msg_queue_list_[chn_index]->pop();
        PushMsg(vio_msg);
        LOGD << "send vio message, channel_id:" << channel_id
             << " frame_id:" << vio_msg->sequence_id_
             << " timestamp:" << vio_msg->time_stamp_;
        itr->second = 0;  // this channel send status close
      }
    }
  }
  for (auto itr = channel_status_.begin(); itr != channel_status_.end();
         ++itr) {
    int channel_id = itr->first;
    int channel_status = itr->second;
    LOGD << "channel_id:" << channel_id << ", status:" << channel_status;
  }
}

int VideoSourcePlugin::Start() {
  int ret;
  LOGI << "enter video source pluin start...";
  if (is_running_) return 0;
  is_running_ = true;

  auto send_frame = SetListenerFunc();
  // all produce init
  for (auto handle : produce_handles_) {
    handle->SetListener(send_frame);
    ret = handle->Init();
    if (ret < 0) {
      LOGF << "VideoSourceProduce init failed, err: " << ret << std::endl;
      return -1;
    }
  }
  // all produce start
  for (auto handle : produce_handles_) {
    ret = handle->Start();
    if (ret < 0) {
      LOGF << "VideoSourceProduce start failed, err: " << ret << std::endl;
      return -1;
    }
  }

  return 0;
}

int VideoSourcePlugin::Stop() {
  LOGW << "Enter VideoSourcePlugin stop, produce_handles_num: "
    << produce_handles_.size();
  if (!is_running_) return 0;
  is_running_ = false;
  // all produce stop
  for (auto handle : produce_handles_) {
    handle->Stop();
  }
  // all produce deinit
  for (auto handle : produce_handles_) {
    handle->DeInit();
  }
  return 0;
}

int VideoSourcePlugin::CamStart() {
  int ret;
  if (is_running_) return 0;
  is_running_ = true;

  auto send_frame = SetListenerFunc();
  for (auto handle : produce_handles_) {
    if (handle->GetSourceType() == kHorizonVideoSourceTypeCam) {
      handle->SetListener(send_frame);
      ret = handle->Start();
      if (ret < 0) {
        LOGF << "VideoSourcePlugin start failed, err: " << ret << std::endl;
        return -1;
      }
    }
  }
  return 0;
}

int VideoSourcePlugin::CamStop() {
  if (!is_running_) return 0;
  is_running_ = false;
  for (auto handle : produce_handles_) {
    if (handle->GetSourceType() == kHorizonVideoSourceTypeCam) {
      handle->Stop();
    }
  }
  return 0;
}

int VideoSourcePlugin::FbStart() {
  int ret;
  if (is_running_) return 0;
  is_running_ = true;

  auto send_frame = SetListenerFunc();
  for (auto handle : produce_handles_) {
    if (handle->GetSourceType() == kHorizonVideoSourceTypeFb) {
      handle->SetListener(send_frame);
      ret = handle->Start();
      if (ret < 0) {
        LOGF << "VideoSourcePlugin start failed, err: " << ret << std::endl;
        return -1;
      }
    }
  }
  return 0;
}

int VideoSourcePlugin::FbStop() {
  if (!is_running_) return 0;
  is_running_ = false;
  for (auto handle : produce_handles_) {
    if (handle->GetSourceType() == kHorizonVideoSourceTypeFb) {
      handle->Stop();
    }
  }
  return 0;
}

std::shared_ptr<VideoSourceConfig> VideoSourcePlugin::GetConfigFromFile(
    const std::string &path) {
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    LOGF << "Open config file " << path << " failed";
    return nullptr;
  }
  LOGI << "Open config file " << path << " success";
  std::stringstream ss;
  ss << ifs.rdbuf();
  ifs.close();
  std::string content = ss.str();
  Json::Value value;
  Json::CharReaderBuilder builder;
  builder["collectComments"] = false;
  JSONCPP_STRING error;
  std::shared_ptr<Json::CharReader> reader(builder.newCharReader());
  try {
    bool ret = reader->parse(content.c_str(), content.c_str() + content.size(),
                             &value, &error);
    if (ret) {
      auto config = std::shared_ptr<VideoSourceConfig>(
          new VideoSourceConfig(value));
      return config;
    } else {
      LOGE << "reader parse failed, ret: " << ret;
      return nullptr;
    }
  } catch (std::exception &e) {
    LOGE << "catch exception...";
    return nullptr;
  }
}

void VideoSourcePlugin::ClearAllQueue() {
  img_msg_queue_list_.clear();
}

}  // namespace xproto
