/*
 * Copyright (c) 2020-present, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     smart_manager.h
 * \Author   xudong.du
 * \Version  1.0.0.0
 * \Date     2020/9/27
 * \Brief    implement of api header
 */
#ifndef INCLUDE_UVCPLUGIN_SMARTMANAGER_H_
#define INCLUDE_UVCPLUGIN_SMARTMANAGER_H_
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <vector>
#include <utility>
#include <deque>

#include "blocking_queue.hpp"
#include "thread_pool.h"
#include "xproto/message/flowmsg.h"
#include "xproto/plugin/xpluginasync.h"
#include "smart_message/smart_message.h"
#include "json/json.h"

namespace xproto {
using xproto::XProtoMessagePtr;
using xproto::message::SmartMessage;
using SmartMessagePtr = std::shared_ptr<SmartMessage>;

class SmartManager {
 public:
  SmartManager() = default;
  explicit SmartManager(std::string config_path);
  ~SmartManager() = default;
  virtual int Init();
  virtual int DeInit();
  int Start();
  int Stop();

  int FeedSmart(XProtoMessagePtr msg, int ori_image_width, int ori_image_height,
                int dst_image_width, int dst_image_height);
  int FeedDropSmart(uint64_t frame_id);
  int FeedInfo(const XProtoMessagePtr &msg);

  bool IsApMode() { return ap_mode_; }
  bool SetApMode(bool set) { return ap_mode_ = set; }
  horizon::vision::BlockingQueue<std::string> pb_ap2cp_info_queue_;

  bool stop_flag_ = false;

 private:
  virtual int Recv(std::string &recvstr) = 0;
  virtual int Send(const std::string &sendstr) = 0;

  void SendThread();
  void RecvThread();

  int Serialize(SmartMessagePtr smart_msg, int ori_image_width,
                int ori_image_height, int dst_image_width,
                int dst_image_height);
  int SerializeDropFrame(uint64_t frame_id);

  bool ap_mode_ = false;
  std::string config_path_;
  Json::Value config_;
  std::shared_ptr<std::thread> thread_ = nullptr;
  std::shared_ptr<std::thread> recv_thread_ = nullptr;
  enum SmartType { SMART_FACE, SMART_BODY, SMART_VEHICLE };
  SmartType smart_type_ = SMART_BODY;

  std::mutex queue_lock_;
  const unsigned int queue_max_size_ = 50;

  using SmartMsg = std::pair<uint64_t, std::string>;
  struct compare {
    bool operator()(SmartMsg msg1, SmartMsg msg2) {
      return msg1.first > msg2.first;
    }
  };
  std::priority_queue<SmartMsg, std::deque<SmartMsg>, compare> smart_msg_queue_;
  std::condition_variable condition_;
  horizon::vision::BlockingQueue<std::string> pb_cp2ap_info_queue_;

  horizon::vision::CThreadPool serialize_thread_;
  bool print_timestamp_ = false;
};
}  // namespace xproto

#endif  // INCLUDE_UVCPLUGIN_SMARTMANAGER_H_
