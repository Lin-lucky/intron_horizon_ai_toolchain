/*
 * Copyright (c) 2020-present, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     hid_manager.h
 * \Author   zhe.sun
 * \Version  1.0.0.0
 * \Date     2020/6/10
 * \Brief    implement of api header
 */
#ifndef INCLUDE_UVCPLUGIN_HIDMANAGER_H_
#define INCLUDE_UVCPLUGIN_HIDMANAGER_H_
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <deque>
#include <vector>
#include <utility>

#include "smart_manager.h"
#include "json/json.h"

namespace xproto {
#define HID_MAX_PACKET_SIZE (1024)
#define HID_BUFFER_SIZE (10*1024*1024)  // 10M bytes

class HidManager : public SmartManager {
 public:
  HidManager() = delete;
  explicit HidManager(std::string config_path);
  ~HidManager();

  int Init() override;
  int DeInit() override;

 private:
  int Recv(std::string &recvstr) override;
  int Send(const std::string &sendstr) override;

  std::string config_path_;
  Json::Value config_;

  std::string hid_file_ = "/dev/hidg0";
  int hid_file_handle_ = -1;
  char *recv_data_ = nullptr;
};
}  // namespace xproto
#endif  // INCLUDE_UVCPLUGIN_HIDMANAGER_H_
