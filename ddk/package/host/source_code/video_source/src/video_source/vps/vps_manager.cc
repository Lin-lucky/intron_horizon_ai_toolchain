/**
 *  Copyright (C) 2021 Horizon Robotics Inc.
 *  All rights reserved.
 *  @Author: yong.wu
 *  @Email: <yong.wu@horizon.ai>
 *  @Date: 2021-04-12
 *  @Version: v0.0.1
 *  @Brief: implemenation of vps manager.
 */
#include "video_source/vps/vps_manager.h"
#include <cstddef>
#include <memory>
#include <stdio.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <queue>
#include <utility>
#include "hobotlog/hobotlog.hpp"
#include "video_source/vps/vps_data_type.h"
#include "video_source/vps/x3/vps_module_hapi.h"
#include "video_source/vps/j3/vps_module_vapi.h"
#include "vps_manager.h"

namespace videosource {

int VpsManager::SetPlatformType(const std::string &input_platform_type) {
  platform_type_ = input_platform_type;
  return 0;
}

int VpsManager::FindGroupId(const int &input_channel_id,
      int &output_group_id) {
  size_t i = 0;
  LOGD << "enter FindGroupId, chn_id_map_list size: "
    << chn_id_map_list_.size();
  for (i = 0; i < chn_id_map_list_.size(); i++) {
    auto chn_id = chn_id_map_list_[i].first;
    if (input_channel_id == chn_id) {
      output_group_id = chn_id_map_list_[i].second;
      LOGI << "find channel_id: " << input_channel_id
        << " map to group_id: " << output_group_id;
      break;
    }
  }
  if (i == chn_id_map_list_.size()) {
    LOGE << "can not find channel_id: " << input_channel_id
      << " map group_id";
    return -1;
  }
  return 0;
}

int VpsManager::CheckChnIdValue(const int &input_channel_id) {
  for (size_t i = 0; i < chn_id_map_list_.size(); i++) {
    auto chn_id = chn_id_map_list_[i].first;
    if (input_channel_id == chn_id) {
      LOGE << "channel_id: " << input_channel_id
        << " has been register!";
      return -1;
    }
  }
  return 0;
}

int VpsManager::GetGroupId(const int &input_channel_id,
    int &output_group_id) {
  std::lock_guard<std::mutex> lg(mutex_);
  int ret = -1;
  int group_id = input_channel_id;

  ret = CheckChnIdValue(input_channel_id);
  if (ret) {
    LOGE << "check channel id failed, ret: " << ret;
    return ret;
  }
  if (platform_type_ == "x3") {
    ret = CreateGroupIdFifo();
    if (ret) {
      LOGE << "create group id fifo failed, ret: " << ret;
      return ret;
    }
    ret = GetGroupIdFromFifo(input_channel_id, group_id);
    if (ret) {
      LOGE << "get group id from fifo failed, ret: " << ret;
      return ret;
    }
  }
  output_group_id = group_id;
  LOGW << "get group_id: " << output_group_id
    << " from grp_id_fifo success...";
  auto chn_id_map = std::make_pair(input_channel_id, output_group_id);
  chn_id_map_list_.push_back(chn_id_map);
  return 0;
}

int VpsManager::FreeGroupId(const int &input_group_id) {
  int ret = -1;
  size_t chn_list_num = chn_id_map_list_.size();

  for (size_t i = 0; i < chn_list_num; i++) {
    auto chn_id = chn_id_map_list_[i].first;
    auto group_id = chn_id_map_list_[i].second;
    if (input_group_id == group_id) {
      LOGI << "find channel_id: " << chn_id
        << " group_id: " << group_id
        << " need free to fifo";
      chn_id_map_list_.erase(chn_id_map_list_.begin() + i);
      break;
    }
  }
  if (chn_list_num == chn_id_map_list_.size()) {
    LOGE << "can not find input_group_id: " << input_group_id
      << " free group id to fifo failed!";
    return -1;
  }
  if (platform_type_ == "x3") {
    ret = FreeGroupIdToFifo(input_group_id);
    if (ret) {
      LOGE << "free group id to fifo failed, ret: " << ret;
      return ret;
    }
  }
  LOGW << "free group id: " << input_group_id << " to fifo success...";
  return 0;
}

int VpsManager::CreateVpsModule(const int &input_channel_id,
    int &output_group_id,
    std::shared_ptr<VpsModule> &output_module) {
  std::lock_guard<std::mutex> lg(mutex_);
  int ret = -1;
  int group_id = input_channel_id;

  ret = CheckChnIdValue(input_channel_id);
  if (ret) {
    LOGE << "check channel id failed, ret: " << ret;
    return ret;
  }
  if (platform_type_ == "x3") {
    ret = CreateGroupIdFifo();
    if (ret) {
      LOGE << "create group id fifo failed, ret: " << ret;
      return ret;
    }
    ret = GetGroupIdFromFifo(input_channel_id, group_id);
    if (ret) {
      LOGE << "get group id from fifo failed, ret: " << ret;
      return ret;
    }
  }
  output_group_id = group_id;
  auto chn_id_map = std::make_pair(input_channel_id, output_group_id);
  chn_id_map_list_.push_back(chn_id_map);
  if (platform_type_ == "x3") {
    output_module =
      std::make_shared<VpsModuleHapi>(input_channel_id, output_group_id);
  } else if (platform_type_ == "j3") {
    output_module =
      std::make_shared<J3VpsModuleVapi>(input_channel_id, output_group_id);
  } else {
    LOGE << "Unsupport input platform type: " << platform_type_;
    return -1;
  }
  return 0;
}

int VpsManager::DestroyVpsModule(
    const std::shared_ptr<VpsModule> &input_module) {
  std::lock_guard<std::mutex> lg(mutex_);
  int ret = -1;

  if (input_module == nullptr) {
    LOGE << "input module is nullptr";
    return -1;
  }
  int group_id = input_module->GetGroupId();
  ret = FreeGroupId(group_id);
  if (ret) {
    LOGE << "free group id to fifo failed, ret: " << ret;
    return ret;
  }
  return 0;
}

int VpsManager::GetGroupIdFromFifo(const int &input_channel_id,
    int &output_group_id) {
  int ret, recv;
  GroupIdStatus grp_id = { 0 };
  int fifo_fd = grp_fifo_fd_;

  for (int try_time = 0; try_time < 4; try_time++) {
    ret = read(fifo_fd, &grp_id, sizeof(GroupIdStatus));
    if (ret <= 0) {
      LOGE << "read fifo failed, ret: " << ret
        << " fifo_fd: " << fifo_fd
        << " errno: " << errno;
      perror("read vps group id fifo");
      usleep(5000);  // 5ms
    } else {
      break;
    }
  }
  if (ret <= 0) {
    LOGE << "get group_id failed, ret: " << ret;
    return -1;
  }

  if (input_channel_id < 0) {
    // from behind to front find for image source
    for (recv = GRP_FIFO_NUM - 1; recv >= 0; recv--) {
      if (grp_id.status[recv] == 255) {
        break;
      }
    }
    if (recv == -1) {
      LOGE << "all id has been used, get group_id failed";
      return -1;
    }
  } else {
    // from front to behind find for video source
    for (recv = 0; recv < GRP_FIFO_NUM; recv++) {
      if (grp_id.status[recv] == 255) {
        break;
      }
    }
    if (recv == GRP_FIFO_NUM) {
      LOGE << "all id has been used, get group_id failed";
      return -1;
    }
  }
  grp_id.status[recv] = 1;  // used
  ret = write(fifo_fd, &grp_id, sizeof(GroupIdStatus));
  if (ret != sizeof(GroupIdStatus)) {
    LOGE << "write input group id to fifo failed, ret: " << ret
      << " fifo_fd: " << fifo_fd
      << " errno: " << errno;
    return -1;
  }

  LOGI << "get group_id: " << recv
    << " from group fifo success...";
  output_group_id = recv;
  return 0;
}

int VpsManager::FreeGroupIdToFifo(int input_group_id) {
  int ret = -1;
  int value = 255;
  ret = WriteGroupIdToFifo(input_group_id, value);
  if (ret) {
    LOGE << "write group id to fifo failed, ret: " << ret;
    return ret;
  }
  ret = DestroyGroupIdFifo();
  if (ret) {
    LOGE << "destroy group id fifo failed, ret: " << ret;
    return ret;
  }
  LOGI << "free group_id: " << input_group_id
    << " to group fifo success...";
  return 0;
}

int VpsManager::WriteGroupIdToFifo(int input_group_id, int value) {
  int ret = -1;
  GroupIdStatus grp_id = { 0 };
  int fifo_fd = grp_fifo_fd_;

  if (input_group_id < 0 || input_group_id >= GRP_FIFO_NUM) {
    LOGE << "input group id is invalid, id: " << input_group_id;
    return -1;
  }

  for (int try_time = 0; try_time < 4; try_time++) {
    ret = read(fifo_fd, &grp_id, sizeof(GroupIdStatus));
    if (ret <= 0) {
      LOGE << "read fifo failed, ret: " << ret
        << " fifo_fd: " << fifo_fd
        << " errno: " << errno;
      usleep(5000);  // 5ms
    } else {
      break;
    }
  }
  if (ret <= 0) {
    LOGE << "get group_id failed, ret: " << ret;
    return -1;
  }

  grp_id.status[input_group_id] = value;  // not used
  ret = write(fifo_fd, &grp_id, sizeof(GroupIdStatus));
  if (ret != sizeof(GroupIdStatus)) {
    LOGE << "write input group id to fifo failed, ret: " << ret
      << " fifo_fd: " << fifo_fd
      << " errno: " << errno;
    return -1;
  }
  return 0;
}

int VpsManager::CreateGroupIdFifo() {
  int ret = -1;
  if (m_ref_cnt_++ == 0 && init_flag_ == false) {
    ret = CreateSharedFifo(GRP_FIFO_NAME, grp_fifo_fd_);
    if (ret) {
      LOGE << "create group shared fifo failed, ret: " << ret;
      return ret;
    }
    init_flag_ = true;
  }
  LOGI << "create group id fifo, ref_cnt: " << m_ref_cnt_
    << " grp_fifo_fd: " << grp_fifo_fd_;
  return 0;
}

int VpsManager::DestroyGroupIdFifo() {
  int ret = -1;
  m_ref_cnt_--;
  if (m_ref_cnt_ <= 0 && init_flag_) {
    ret = DestroySharedFifo(GRP_FIFO_NAME, grp_fifo_fd_);
    if (ret) {
      LOGE << "destroy group shared fifo failed, ret: " << ret;
      return ret;
    }
    init_flag_ = false;
  }
  LOGI << "destroy group id fifo , ref_cnt: " << m_ref_cnt_;
  return 0;
}

int VpsManager::TryTestFifo(const std::string &input_fifo_name) {
  int ret = -1;
  int fifo_fd;
  GroupIdStatus grp_id = { 0 };

  ret = open(input_fifo_name.c_str(), O_RDWR | O_NONBLOCK);
  if (ret < 0) {
    perror("open vps manager fifo");
    goto err;
  }
  fifo_fd = ret;

  ret = read(fifo_fd, &grp_id, sizeof(GroupIdStatus));
  if (ret <= 0) {
    perror("read vps manager fifo");
    goto err;
  }
  // write callback data to fifo
  ret = write(fifo_fd, &grp_id, sizeof(GroupIdStatus));
  if (ret != sizeof(GroupIdStatus)) {
    perror("write vps manager fifo");
    goto err;
  }
  return 0;

err:
  unlink(input_fifo_name.c_str());
  return -1;
}

int VpsManager::CreateSharedFifo(const std::string &input_fifo_name,
    int &output_fifo_fd) {
  int ret = -1;
  int try_time = 0;
  int fd_tmp = -1;

  do {
    ret = mkfifo(input_fifo_name.c_str(), 0666);
    if (ret == 0) {
      is_main_process_ = true;
      LOGW << "create vps manager fifo success, this is main process";
      break;
    } else {
      // maybe fifo has been created
      perror("mkfifo vps manager");
      LOGE << "create vps manager fifo failed, ret: " << ret
        << " fifo maybe has been created";
      ret = TryTestFifo(input_fifo_name);
      if (ret) {
        continue;  // last same process exception quit
      } else {
        break;  // other process normal open fifo
      }
    }
  } while (try_time++ < 3);

  try_time = 0;
  do {
    ret = open(input_fifo_name.c_str(), O_RDWR | O_NONBLOCK);
    LOGD << "open input fifo, fd: " << ret << " index: " << try_time;
    if (ret < 0) {
      LOGE << "open input fifo failed, fd: " << ret << " index: " << try_time;
      perror("open");
    } else if (ret == 0) {
      // pass fd is 0 beause ipu hal has bug
      fd_tmp = ret;
      LOGI << "pass fd value is equal to 0";
    } else {
      LOGI << "vps manager get correct fd: " << ret;
      break;
    }
  } while (try_time++ < 3);

  if (fd_tmp == 0) {
    LOGD << "close fd: " << fd_tmp;
    close(fd_tmp);
  }
  if (ret <= 0) {
    LOGE << "open input fifo failed, ret: " << ret;
    unlink(input_fifo_name.c_str());
    return -1;
  }
  output_fifo_fd = ret;

  if (is_main_process_ == false) {
    LOGI << "this process is not main process"
      << " input_fifo_name: " << input_fifo_name;
    return 0;  // other process normal open fifo
  }

  GroupIdStatus grp_id = { 0 };
  for (int i = 0; i < GRP_FIFO_NUM; i++) {
    grp_id.status[i] = 255;  // 255 is free, 1 is used
  }
  ret = write(output_fifo_fd, &grp_id, sizeof(GroupIdStatus));
  if (ret != sizeof(GroupIdStatus)) {
    LOGE << "write input group id to fifo failed, ret: " << ret
      << " fifo_fd: " << output_fifo_fd
      << " errno: " << errno;
    return -1;
  } else {
    LOGW << "write input group id to fifo success, write_bytes_num: " << ret
      << " fifo_fd: " << output_fifo_fd;
  }
  return 0;
}

int VpsManager::DestroySharedFifo(const std::string &input_fifo_name,
    const int &input_fifo_fd) {
  int ret = -1;

  if (input_fifo_fd > 0) {
    ret = close(input_fifo_fd);
    if (ret) {
      LOGE << "close fifo_fd: " << input_fifo_fd << " failed,"
        << " ret:" << ret << " errno: "<< errno;
      return ret;
    } else {
      LOGD << "close fifo_fd: " << input_fifo_fd << " success...";
    }
  }
  if (is_main_process_ == true) {
    m_ref_cnt_ = 0;
    init_flag_ = false;
    is_main_process_ = false;
    grp_fifo_fd_ = -1;
    ret = unlink(input_fifo_name.c_str());
    if (ret) {
      LOGE << "unlink fifo name: " << input_fifo_name << " failed,"
        << " ret:" << ret << " errno: "<< errno;
      return ret;
    } else {
      LOGI << "unlink fifo name: " << input_fifo_name << " success...";
    }
  }
  return 0;
}

}  // namespace videosource
