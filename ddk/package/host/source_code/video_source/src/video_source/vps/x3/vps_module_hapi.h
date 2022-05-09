/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef VIDEO_SOURCE_VPS_VPS_MODULE_HAPI_H_
#define VIDEO_SOURCE_VPS_VPS_MODULE_HAPI_H_
#include <chrono>
#include <semaphore.h>
#include <string>
#include <vector>
#include <memory>
#include <queue>
#include <thread>
#include "video_source/vps/vps_module.h"
#include "video_source/vps/vps_data_type.h"
#include "video_source/video_source_type.h"
#include "utils/json_cfg_wrapper.h"

namespace videosource {

class VpsModuleHapi : public VpsModule {
 public:
  explicit VpsModuleHapi(const int &channel_id, const int &group_id)
    : VpsModule(channel_id, group_id) {}
  ~VpsModuleHapi() {}

  // derived class override
  // static file config init vpsmodule
  int Init(const std::vector<std::string> &config_file_list) override;
  // dynamic config init vpsmodule
  int Init(std::shared_ptr<VpsConfig> &vps_cfg) override;
  int DeInit() override;
  int Start()  override;
  int Stop()  override;
  int GetFrame(int vps_chn, void *output_nv12) override;
  int FreeFrame(int vps_chn, void *input_nv12) override;
  int GetPyramidFrame(int vps_chn, void *output_src, void *output_pym) override;
  int FreePyramidFrame(int vps_chn, void *input_src, void *input_pym) override;
  int GetImageBuffer(std::shared_ptr<ImageFrame> &output_buf_info) override;
  int RunVpsProcess(void *input_nv12) override;
  int SendFrame(void *input_nv12) override;

  int GetVpsIpuChn(std::vector<int> &ipu_chn_list) override;
  int GetVpsPymChn(std::vector<int> &pym_chn_list) override;
  int GetConfig(std::shared_ptr<void> &output_cfg) override;
  void SetConfig(std::shared_ptr<void> &input_cfg) override;

 protected:
  int VpsLoadConfig(const std::string &input_config_file);
  int VpsInit();
  int VpsDeInit();
  int VpsStart();
  int VpsStop();
  int SetGdcInfo(VpsGdcType type);
  int GetGdcInfo(void *buf);
  int VpsCreateGetDataThread();
  int VpsDestoryGetDataThread();

 private:
  int HbChnInfoInit();
  int HbCheckConfig(std::shared_ptr<VpsConfig> &vps_cfg);
  int HbGetVpsFrameDepth();
  int HbVpsCreateGrp(int group_id, int grp_w, int grp_h, int grp_depth);
  int HbGetGdcData(const char *gdc_name, char **gdc_data, int *gdc_len);
  int HbSetGdcInfo(int gdc_idx, int group_id, const VpsGdcInfo &info);
  void HbGetPymDataThread();
  void HbGetIpuDataThread();

 private:
  bool init_flag_ = false;
  bool start_flag_ = false;
  std::shared_ptr<VpsConfig> vps_cfg_ = nullptr;
  int gdc_group_id_[MAX_GDC_NUM] = {-1, -1};
  std::shared_ptr<std::thread> pym_thread_ = nullptr;
  std::shared_ptr<std::thread> ipu_thread_ = nullptr;
  int pym_start_flag_ = false;
  int ipu_start_flag_ = false;
  bool vp_init_flag_ = false;
  std::chrono::system_clock::time_point pym_start_time_;
  std::chrono::system_clock::time_point ipu_start_time_;
  int vps_dump_num_ = 0;
};

}  // namespace videosource
#endif  // VIDEO_SOURCE_VPS_VPS_MODULE_HAPI_H_
