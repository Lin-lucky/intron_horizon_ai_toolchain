/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef VIDEO_SOURCE_VPS_J3_VPS_MODULE_VAPI_H_
#define VIDEO_SOURCE_VPS_J3_VPS_MODULE_VAPI_H_
#include <chrono>
#include <cstdint>
#include <semaphore.h>
#include <string>
#include <vector>
#include <memory>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>
#include "video_source/vps/vps_module.h"
#include "video_source/video_source_type.h"
#include "utils/json_cfg_wrapper.h"
#include "utils/ring_queue.h"

namespace videosource {

class J3VpsModuleVapi : public VpsModule {
 public:
  explicit J3VpsModuleVapi(const int &channel_id, const int &group_id)
    : VpsModule(channel_id, group_id) {}
  ~J3VpsModuleVapi() {}

  // derived class override
  // static file config init vpsmodule
  int Init(const std::shared_ptr<JsonConfigWrapper> &json_cfg) override;
  int DeInit() override;
  int Start()  override;
  int Stop()  override;
  int GetPyramidFrame(int vps_chn, void *output_src, void *output_pym) override;
  int FreePyramidFrame(int vps_chn, void *input_src, void *input_pym) override;
  int GetVpsPymChn(std::vector<int> &pym_chn_list) override;

 private:
  int LoadConfig(const std::shared_ptr<JsonConfigWrapper> &vps_cfg_json);
  int VpsCreateGetDataThread();
  int VpsDestoryGetDataThread();
  void HbGetPymDataThread();
  int CheckVioConfig(const std::string &vio_config_file);
  int CheckIpuConfig(const std::shared_ptr<JsonConfigWrapper> &ipu_json);
  int CheckPymConfig(const std::shared_ptr<JsonConfigWrapper> &pym_json);
  int ConvertIpuChn2Type(const int &ipu_chn);
  void UpdateSourceChannel();
  static int GetVpsTaskNum() { return task_num_; }
  static void AddVpsTask() { task_num_++; }
  static void FreeVpsTask() { task_num_--; }

 private:
  bool init_flag_ = false;
  bool start_flag_ = false;
  std::shared_ptr<std::thread> pym_thread_ = nullptr;
  int pym_start_flag_ = false;
  std::chrono::system_clock::time_point pym_start_time_;
  std::vector<int> gdc_en_list_;
  std::vector<int> bind_source_chn_list_;
  std::vector<int> ipu_chn_list_;
  std::string vio_cfg_file_;
  int pym_chn_ = 2;  // hard code to ds2
  int dump_count_ = 0;
  static std::once_flag vps_init_flag_;
  static std::atomic_int task_num_;
  VIO_DATA_TYPE_E ipu_data_type_ = HB_VIO_IPU_DS2_DATA;  // default ds2 output
  int this_src_chn_ = -1;
  int this_ipu_chn_ = -1;
  static std::vector<std::vector<int>> vps_ipu_valid_index_;

  // ipu info
  struct IpuCfgInfo {
#define MAX_IPU_DS_NUM 5
    int source_sel;
    bool ds2_to_ddr_en;
    struct IpuScaleInfo {
      bool roi_en;
      int roi_x;
      int roi_y;
      int roi_w;
      int roi_h;
      bool scale_en;
      int scale_w;
      int scale_h;
      int buf_num;
    };
    IpuScaleInfo ds_info[MAX_IPU_DS_NUM];
    IpuScaleInfo us_info;
  };
  IpuCfgInfo ipu_cfg_ = { 0 };
  static std::vector<IpuCfgInfo> ipu_cfg_list_;

  // pym info
  struct PymCfgInfo {
#define MAX_PYM_DS_NUM 24
#define MAX_PYM_US_NUM 6
    int img_src_sel;	//ipu otf in mode : 1,  ddr in mode : 0,
    int frame_id;
    int src_w;
    int src_h;
    int ds_layer_en;
    int ds_uv_bypass;
    int us_layer_en;
    int us_uv_bypass;
    int ddr_in_buf_num;
    int output_buf_num;
    int timeout;
    struct PymScaleInfo {
      uint16_t factor;
      uint16_t roi_x;
      uint16_t roi_y;
      uint16_t roi_w;
      uint16_t roi_h;
    };
    PymScaleInfo ds_info[MAX_PYM_DS_NUM];
    PymScaleInfo us_info[MAX_PYM_US_NUM];
  } pym_cfg_t;
  PymCfgInfo pym_cfg_ = { 0 };
  static std::vector<PymCfgInfo> pym_cfg_list_;
};

}  // namespace videosource
#endif  // VIDEO_SOURCE_VPS_J3_VPS_MODULE_VAPI_H_
