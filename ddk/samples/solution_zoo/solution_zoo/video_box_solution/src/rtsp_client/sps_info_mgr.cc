/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */
#include "rtsp_client/sps_info_mgr.h"

#include "hobotlog/hobotlog.hpp"
extern "C" {
#include "rtsp_client/sps_pps.h"
}
#include "rtsp_client/hevc_sps.h"

namespace solution {
namespace video_box {
SPSInfoMgr* SPSInfoMgr::instance_ = NULL;

SPSInfoMgr::SPSInfoMgr() : initialized_(false) {}

SPSInfoMgr& SPSInfoMgr::GetInstance() {
  static std::once_flag flag;
  std::call_once(flag, [&]() { instance_ = new SPSInfoMgr(); });
  return *instance_;
}
int SPSInfoMgr::Init() { return 0; }

int SPSInfoMgr::AnalyticsSps(const unsigned char* spsinfo, const int sps_len,
                             int& width, int& height,
                             const std::string type_name) {
  if (type_name.find("H264") != std::string::npos) {
    return AnalyticsSps_H264(spsinfo, sps_len, width, height);
  } else {
    return AnalyticsSps_H265(spsinfo, sps_len, width, height);
  }
}

int SPSInfoMgr::AnalyticsSps_H264(const unsigned char* spsinfo,
                                  const int sps_len, int& width, int& height) {
  std::lock_guard<std::mutex> lock(manager_mutex_);
  struct get_bit_context objBit;
  memset(&objBit, 0, sizeof(objBit));
  objBit.buf = (uint8_t*)spsinfo + 1;  // NOLINT
  objBit.buf_size = sps_len;
  struct SPS objSps;
  memset(&objSps, 0, sizeof(objSps));
  int nRet = h264dec_seq_parameter_set(&objBit, &objSps);
  if (nRet != 0) {
    LOGE << "spsinfomgr call h264dec_seq_parameter_set fail, return:" << nRet;
    return nRet;
  }

  width = h264_get_width(&objSps);
  height = h264_get_height(&objSps);
  LOGW << "sps anlytics get width:" << width << " height:" << height;
  return 0;
}

int SPSInfoMgr::AnalyticsSps_H265(const unsigned char* spsinfo,
                                  const int sps_len, int& width, int& height) {
  std::lock_guard<std::mutex> lock(manager_mutex_);
  vc_params_t params = {0};
  bool bRet = parse_sps((unsigned char*)spsinfo, sps_len, params);  // NOLINT
  if (!bRet) {
    LOGE << "h265 parse_sps fail";
    return -1;
  }

  width = params.width;
  height = params.height;
  return 0;
}

}  // namespace vision
}  // namespace horizon
