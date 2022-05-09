/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef VIDEO_SOURCE_VPS_VPS_MANAGER_H_
#define VIDEO_SOURCE_VPS_VPS_MANAGER_H_
#include "video_source/vps/vps_module.h"
#include <string>
#include <memory>
#include <mutex>
#include <vector>
#include <utility>

#define GRP_FIFO_NAME  "grp_id_fifo"
#define GRP_FIFO_NUM    (8)

namespace videosource {

#define IN
#define OUT
#define INOUT

struct GroupIdStatus {
  int status[GRP_FIFO_NUM];
};

class VpsManager {
 public:
  static VpsManager &Get() {
    static VpsManager inst;
    return inst;
  }
  ~VpsManager() {}
  int SetPlatformType(IN const std::string &input_platform_type);
  int CreateVpsModule(IN const int &input_channel_id,
      OUT int &output_group_id,
      OUT std::shared_ptr<VpsModule> &output_module);
  int DestroyVpsModule(IN const std::shared_ptr<VpsModule> &input_module);
  int GetGroupId(IN const int &intput_channel_id,
      OUT int &output_group_id);
  int FreeGroupId(IN const int &input_group_id);
  int FindGroupId(IN const int &input_channel_id,
      OUT int &output_group_id);

 protected:
  int GetGroupIdFromFifo(const int &input_channel_id, int &output_group_id);
  int FreeGroupIdToFifo(int input_group_id);
  int WriteGroupIdToFifo(int input_group_id, int value);
  int CreateGroupIdFifo();
  int DestroyGroupIdFifo();
  int CreateSharedFifo(const std::string &input_fifo_name,
      int &output_fifo_fd);
  int DestroySharedFifo(const std::string &input_fifo_name,
      const int &input_fifo_fd);
  int CheckChnIdValue(const int &input_channel_id);
  int TryTestFifo(const std::string &input_fifo_name);

 private:
  std::mutex mutex_;
  int m_ref_cnt_ = 0;
  bool init_flag_ = false;
  bool is_main_process_ = false;
  bool vin_vps_bind_ = false;
  int grp_fifo_fd_ = -1;
  std::string platform_type_ = "x3";
  std::vector<std::pair<int, int>> chn_id_map_list_;
};

}  // namespace videosource
#endif  // VIDEO_SOURCE_VPS_VPS_MANAGER_H_
