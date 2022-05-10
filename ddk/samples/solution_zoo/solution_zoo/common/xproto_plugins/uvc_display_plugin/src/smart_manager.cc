/*!
 * Copyright (c) 2020-present, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     smart_manager.cpp
 * \Author   zhuo.wang
 * \Version  1.0.0.0
 * \Date     2021.1.26
 * \Brief    implement of api file
 */
#include "smart_manager.h"

#include <ctype.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>
#include <chrono>
#include <fstream>
#include <string>
#include <utility>

#include "hobotlog/hobotlog.hpp"
#ifdef USE_MC
#include "transport_message/monitor_control_message.h"
#endif
#include "smart_message/smart_message.h"
#include "xproto/message/msg_registry.h"
#include "xproto/msg_type/protobuf/x3.pb.h"
#include "xproto/msg_type/vio_message.h"
#include "xstream/xstream_data.h"

namespace xproto {
using xproto::XPluginErrorCode;
using xproto::message::SmartMessage;
#ifdef USE_MC
using xproto::message::MonitorControlMessage;
#endif

SmartManager::SmartManager(std::string config_path) {
  config_path_ = config_path;
  LOGI << "SmartManager smart config file path:" << config_path_;
}

int SmartManager::Init() {
  LOGI << "SmartManager Init";
  // config_
  if (config_path_ != "") {
    std::ifstream ifs(config_path_);
    if (!ifs.is_open()) {
      LOGF << "open config file " << config_path_ << " failed";
      return -1;
    }
    Json::CharReaderBuilder builder;
    std::string err_json;
    try {
      bool ret = Json::parseFromStream(builder, ifs, &config_, &err_json);
      if (!ret) {
        LOGF << "invalid config file " << config_path_;
        return -1;
      }
    } catch (std::exception &e) {
      LOGF << "exception while parse config file " << config_path_ << ", "
           << e.what();
      return -1;
    }
    // smart_type_
    if (config_.isMember("smart_type")) {
      smart_type_ = static_cast<SmartType>(config_["smart_type"].asInt());
    }

    if (config_.isMember("ap_mode")) {
      ap_mode_ = config_["ap_mode"].asBool();
    }
  }

  serialize_thread_.CreatThread(1);
  auto print_timestamp_str = getenv("uvc_print_timestamp");
  if (print_timestamp_str && !strcmp(print_timestamp_str, "ON")) {
    print_timestamp_ = true;
  }
  return 0;
}

int SmartManager::DeInit() {
  LOGI << "SmartManager DeInit";
  return 0;
}

void SmartManager::RecvThread() {
  LOGD << "start SmartManager RecvThread";
  while (!stop_flag_) {
    std::string recvstr;
    int recv_size = Recv(recvstr);
    if (recv_size > 0) {
      LOGD << "Receive pb info data from ap size: " << recv_size;
      pb_ap2cp_info_queue_.push(recvstr);
    }
  }
}

void SmartManager::SendThread() {
  LOGD << "start SmartManager SendThread";
  if (ap_mode_) {
    while (!stop_flag_) {
      {
        // 需要从pb_buffer中获取一个结果返回
        std::unique_lock<std::mutex> lck(queue_lock_);
        static uint64_t last_send_id = 0;
        bool wait_ret =
            condition_.wait_for(lck, std::chrono::milliseconds(100), [&]() {
              return (smart_msg_queue_.size() > 0 &&
                      ((smart_msg_queue_.top().first <= last_send_id + 1) ||
                       smart_msg_queue_.size() >= queue_max_size_)) ||
                     (pb_cp2ap_info_queue_.size() > 0);
            });
        if (!wait_ret)
          continue;
        size_t queue_len = smart_msg_queue_.size();
        if (queue_len > 0) {
          auto msg_pair = smart_msg_queue_.top();
          auto send_msg = [&, this]() {
            last_send_id = msg_pair.first;
            LOGD << "send frame " << msg_pair.first;
            std::string pb_string = std::move(msg_pair.second);
            smart_msg_queue_.pop();
            lck.unlock();

            auto send_start = std::chrono::high_resolution_clock::now();
            if (Send(pb_string) <= 0) {
              LOGE << "Send error!";
            } else {
              LOGD << "Send end";
            }
            auto send_end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<float, std::milli> cost =
                send_end - send_start;
            LOGI << "send pb cost ms:" << cost.count();
          };
          //  此时发送的是智能帧或drop帧
          if (last_send_id + 1 == msg_pair.first) {
            send_msg();
          } else if (msg_pair.first == 0
                     //  此时是发送AP查询的底库的CURD相关信息
                     || last_send_id + 1 > msg_pair.first) {
            //  此时发送的是抓拍帧或第一帧智能帧
            auto last_send_id_cache = last_send_id;
            send_msg();
            last_send_id = last_send_id_cache;
          } else if (queue_len >= queue_max_size_) {
            // exception occurred
            auto lost_frame_id = last_send_id;
            send_msg();
            LOGW << "frame id: " << lost_frame_id << " lost";
          }
        }
      }

      // send smart first, in order to avoid receiving smart data
      // after recved stop cmd response in ap
      // send info data
      while (pb_cp2ap_info_queue_.size() > 0) {
        std::string pb_string;
        if (pb_cp2ap_info_queue_.try_pop(&pb_string,
                                         std::chrono::microseconds(1000))) {
          // 将pb string 发送给ap
          if (Send(pb_string) <= 0) {
            LOGE << "Send error!";
          } else {
            LOGD << "Send end";
          }
        }
      }
    }
  }
}

int SmartManager::Start() {
  if (thread_ == nullptr) {
    thread_ = std::make_shared<std::thread>(&SmartManager::SendThread, this);
  }
  if (ap_mode_ && recv_thread_ == nullptr) {
    recv_thread_ =
        std::make_shared<std::thread>(&SmartManager::RecvThread, this);
  }
  return 0;
}

int SmartManager::Stop() {
  LOGI << "SmartManager Stop";
  stop_flag_ = true;
  if (thread_ != nullptr) {
    thread_->join();
    thread_ = nullptr;
  }
  if (recv_thread_ != nullptr) {
    recv_thread_->join();
    recv_thread_ = nullptr;
  }
  {
    std::lock_guard<std::mutex> lck(queue_lock_);
    while (!smart_msg_queue_.empty()) {
      smart_msg_queue_.pop();
    }
  }
  LOGI << "SmartManager Stop Done";
  return 0;
}

int SmartManager::FeedInfo(const XProtoMessagePtr &msg) {
#ifdef USE_MC
  // MonitorControlMessage
  auto mc_msg = std::static_pointer_cast<XProtoMessage>(msg);
  if (!mc_msg.get()) {
    LOGE << "msg is null";
    return -1;
  }
  std::string protocol = mc_msg->Serialize();
  // pb入队
  LOGD << "info data to queue, size: " << protocol.size();
  pb_cp2ap_info_queue_.push(std::move(protocol));
  condition_.notify_one();
#endif
  return 0;
}

int SmartManager::FeedSmart(XProtoMessagePtr msg, int ori_image_width,
                            int ori_image_height, int dst_image_width,
                            int dst_image_height) {
  auto smart_msg = std::static_pointer_cast<SmartMessage>(msg);
  // convert pb2string
  if (!smart_msg.get()) {
    LOGE << "msg is null";
    return -1;
  }

  if (serialize_thread_.GetTaskNum() > 5) {
    LOGW << "Serialize Thread task num more than 5: "
         << serialize_thread_.GetTaskNum();
  }
  serialize_thread_.PostTask(
      std::bind(&SmartManager::Serialize, this, smart_msg, ori_image_width,
                ori_image_height, dst_image_width, dst_image_height));
  return 0;
}

int SmartManager::Serialize(SmartMessagePtr smart_msg, int ori_image_width,
                            int ori_image_height, int dst_image_width,
                            int dst_image_height) {
  std::string protocol;
  uint64_t timestamp = 0;
  switch ((SmartType)smart_type_) {
  case SmartType::SMART_FACE:
  case SmartType::SMART_BODY: {
    auto msg = dynamic_cast<SmartMessage *>(smart_msg.get());
    if (msg) {
      msg->SetExpansionRatio(
          config_["matting_trimapfree_expansion_ratio"].isNumeric() ?
          config_["matting_trimapfree_expansion_ratio"].asFloat() : 0);
      msg->SetAPMode(ap_mode_);
      protocol = msg->Serialize(ori_image_width, ori_image_height,
                                dst_image_width, dst_image_height);
      timestamp = msg->time_stamp_;
    }
    break;
  }
  case SmartType::SMART_VEHICLE: {
    auto msg = dynamic_cast<SmartMessage *>(smart_msg.get());
    if (msg) {
      protocol = msg->Serialize(ori_image_width, ori_image_height,
                                dst_image_width, dst_image_height);
      timestamp = msg->time_stamp_;
    }
    break;
  }
  default:
    LOGE << "not support smart_type";
    return -1;
  }
  // pb入队
  LOGD << "smart data to queue";
  {
    std::lock_guard<std::mutex> lck(queue_lock_);
    smart_msg_queue_.emplace(std::make_pair(timestamp, std::move(protocol)));
    LOGD << "smart_msg_queue_ size:" << smart_msg_queue_.size();
    if (smart_msg_queue_.size() > queue_max_size_) {
      LOGW << "smart_msg_queue_.size() is larger than MAX_SIZE: "
           << queue_max_size_;
      smart_msg_queue_.pop();
    }
  }
  condition_.notify_one();
  if (print_timestamp_) {
    LOGW << "SmartManager::Serialize timestamp:" << timestamp;
  }
  return 0;
}

int SmartManager::FeedDropSmart(uint64_t frame_id) {
  LOGD << "feed drop frame " << frame_id;
  serialize_thread_.PostTask(
      std::bind(&SmartManager::SerializeDropFrame, this, frame_id));
  return 0;
}

//  it is better SerializeDropFrame in smartmessage rather in hid_manager.
int SmartManager::SerializeDropFrame(uint64_t frame_id) {
  x3::MessagePack pack;
  pack.set_flow_(x3::MessagePack_Flow::MessagePack_Flow_CP2AP);
  pack.set_type_(x3::MessagePack_Type::MessagePack_Type_kXPlugin);
  auto add_frame = pack.mutable_addition_()->mutable_frame_();
  add_frame->set_timestamp_(frame_id);
  add_frame->set_sequence_id_(frame_id);
  add_frame->set_frame_type_(x3::Frame_FrameType_DropFrame);

  auto protocol = pack.SerializeAsString();
  {
    std::lock_guard<std::mutex> lck(queue_lock_);
    smart_msg_queue_.emplace(std::make_pair(frame_id, std::move(protocol)));
    LOGD << "smart_msg_queue_ size:" << smart_msg_queue_.size();
    if (smart_msg_queue_.size() > queue_max_size_) {
      LOGW << "smart_msg_queue_.size() is larger than MAX_SIZE: "
           << queue_max_size_;
      smart_msg_queue_.pop();
    }
  }
  condition_.notify_one();
  if (print_timestamp_) {
    LOGW << "SmartManager::SerializeDropFrame timestamp:" << frame_id;
  }

  return 0;
}
}  // namespace xproto
