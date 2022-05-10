/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */
#ifndef INCLUDE_RTSPCLIENT_SPSINFOMGR_H_
#define INCLUDE_RTSPCLIENT_SPSINFOMGR_H_

#include <string.h>

#include <mutex>
#include <string>
namespace solution {
namespace video_box {
class SPSInfoMgr {
 public:
  static SPSInfoMgr &GetInstance();
  ~SPSInfoMgr() = default;

 public:
  int Init();

  int AnalyticsSps(const unsigned char *spsinfo, const int sps_len, int &width,
                   int &height, const std::string type_name = "H264");

 private:
  int AnalyticsSps_H264(const unsigned char *spsinfo, const int sps_len,
                        int &width, int &height);

  int AnalyticsSps_H265(const unsigned char *spsinfo, const int sps_len,
                        int &width, int &height);

 private:
  SPSInfoMgr();
  SPSInfoMgr(const SPSInfoMgr &);
  SPSInfoMgr &operator=(const SPSInfoMgr &);

  static SPSInfoMgr *instance_;
  std::mutex manager_mutex_;
  bool initialized_;
};
}  // namespace video_box
}  // namespace solution

#endif  // INCLUDE_RTSPCLIENT_SPSINFOMGR_H_
