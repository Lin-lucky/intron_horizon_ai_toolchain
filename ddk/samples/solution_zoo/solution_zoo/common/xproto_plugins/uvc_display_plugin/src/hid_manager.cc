/*!
 * Copyright (c) 2020-present, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     hid_manager.cpp
 * \Author   zhe.sun
 * \Version  1.0.0.0
 * \Date     2020.6.10
 * \Brief    implement of api file
 */
#include "hid_manager.h"

#include <ctype.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>
#include <string.h>
#include <limits.h>
#include <chrono>
#include <fstream>
#include <string>
#include <utility>

#include "hobotlog/hobotlog.hpp"

namespace xproto {

HidManager::HidManager(std::string config_path) : SmartManager(config_path) {
  config_path_ = config_path;
  LOGI << "HidManager smart config file path:" << config_path_;
}

HidManager::~HidManager() {
  if (recv_data_) {
    delete[] recv_data_;
    recv_data_ = nullptr;
  }
}

int HidManager::Init() {
  LOGI << "HidManager Init";
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

    // hid_file_
    if (config_.isMember("hid_file")) {
      hid_file_ = config_["hid_file"].asString();
    }
  }

  // hid_file_handle_
  hid_file_handle_ = open(hid_file_.c_str(), O_RDWR, 0666);
  if (hid_file_handle_ < 0) {
    LOGE << "open hid device file fail: " << strerror(errno);
    return -1;
  }
  LOGD << "Hid open hid_file_handle";

  recv_data_ = new char[HID_BUFFER_SIZE];
  return SmartManager::Init();
}

int HidManager::DeInit() {
  close(hid_file_handle_);
  if (recv_data_) {
    delete[] recv_data_;
    recv_data_ = nullptr;
  }

  return SmartManager::DeInit();
}

int HidManager::Recv(std::string &recvstr) {
  int recv_size = 0;
  int ret = 0;
  bool is_readsize = false;
  int target_size = INT_MAX - sizeof(int);
  fd_set rset;      // 创建文件描述符的聚合变量
  timeval timeout;  // select timeout
  while (!stop_flag_) {
    if (recv_size == target_size + static_cast<int>(sizeof(int)))
      return recv_size;
    else if (recv_size > target_size + static_cast<int>(sizeof(int)))
      HOBOT_CHECK(false) << "recv_size is larger than target";
    FD_ZERO(&rset);                   // 文件描述符聚合变量清0
    FD_SET(hid_file_handle_, &rset);  // 添加文件描述符
    timeout.tv_sec = 0;
    timeout.tv_usec = 100 * 1000;
    ret = select(hid_file_handle_ + 1, &rset, NULL, NULL, &timeout);
    if (ret == 0) {
      break;
    } else if (ret < 0) {
      LOGE << "Hid select: read request error, ret: " << ret;
      return ret;
    } else if (!FD_ISSET(hid_file_handle_, &rset)) {
      LOGE << "FD_ISSET is no";
      continue;
    }
    int remain = target_size + static_cast<int>(sizeof(int)) - recv_size;
    remain = remain >= HID_MAX_PACKET_SIZE ? HID_MAX_PACKET_SIZE : remain;
    ret = read(hid_file_handle_, recv_data_ + recv_size, remain);
    if (ret < 0) {
      LOGE << "Hid read hid_file_handle error, hid_file_handle_: "
           << hid_file_handle_ << " ret: " << ret;
      return ret;
    } else if (ret == 0) {
      continue;
    } else {
      recv_size += ret;
      if (!is_readsize && recv_size >= static_cast<int>(sizeof(int))) {
        is_readsize = true;
        target_size = *reinterpret_cast<int *>(recv_data_);
        LOGI << "recv_size : " << recv_size;
        LOGI << "target_size: " << target_size;
      }
    }
  }
  recvstr = std::string(recv_data_ + sizeof(int), recv_size - sizeof(int));
  return recv_size;
}

int HidManager::Send(const std::string &sendstr) {
  LOGI << "Start send smart data...  len:" << sendstr.length();
  int buffer_size_src = sendstr.length();
  char *str_src = const_cast<char *>(sendstr.c_str());

  int buffer_size = buffer_size_src + sizeof(int);
  char *buffer = new char[buffer_size];
  if (buffer == nullptr) {
    LOGE << "send error: null data!";
    return -1;
  }
  // add size
  memmove(buffer, &buffer_size, sizeof(int));
  memmove(buffer + sizeof(int), str_src, buffer_size_src);
  int ret = 0;
  char *buffer_offset = buffer;
  int remainding_size = buffer_size;

  fd_set wset;      // 创建文件描述符的聚合变量
  timeval timeout;  // select timeout
  while (remainding_size > 0 && !stop_flag_) {
    FD_ZERO(&wset);                   // 文件描述符聚合变量清0
    FD_SET(hid_file_handle_, &wset);  // 添加文件描述符
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    int retv = select(hid_file_handle_ + 1, NULL, &wset, NULL, &timeout);
    if (retv == 0) {
      LOGE << "Hid select: send data time out";
      continue;
    } else if (retv < 0) {
      LOGE << "Hid select: send data error, ret: " << retv;
      return -1;
    }
    if (remainding_size >= 1024) {
      LOGD << "Send 1024 bytes data...";
      ret = write(hid_file_handle_, buffer_offset, 1024);
      LOGD << "Send 1024 bytes data end";
    } else {
      LOGD << "Send " << remainding_size << " bytes data...";
      ret = write(hid_file_handle_, buffer_offset, remainding_size);
      LOGD << "Send " << remainding_size << " bytes data end";
    }
    if (ret < 0) {
      LOGF << "send package error: " << strerror(errno) << "; ret: " << ret;
      delete[] buffer;
      return -1;
    }
    remainding_size = remainding_size - ret;
    buffer_offset = buffer_offset + ret;
    if (remainding_size < 0) {
      LOGF << "send package error: " << strerror(errno) << "; ret: " << ret;
      delete[] buffer;
      return -1;
    }
  }
  delete[] buffer;
  return buffer_size_src;
}
}  // namespace xproto
