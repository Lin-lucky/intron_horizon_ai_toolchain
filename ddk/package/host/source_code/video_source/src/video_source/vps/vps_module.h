/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef VIDEO_SOURCE_VPS_VPS_MODULE_H_
#define VIDEO_SOURCE_VPS_VPS_MODULE_H_
#include <semaphore.h>
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <functional>
#include "video_source/video_source_type.h"
#include "video_source/vps/vps_data_type.h"
#include "utils/json_cfg_wrapper.h"
#include "utils/ring_queue.h"

namespace videosource {

using FrameIdCb = std::function<int(const int &frame_id)>;

class VpsModule {
 public:
  explicit VpsModule(const int &channel_id, const int &group_id)
    : channel_id_(channel_id), group_id_(group_id) {}
  ~VpsModule() {}

  // static file config init vpsmodule
  virtual int Init(
      const std::vector<std::string> &config_file_list) { return 0; }
  virtual int Init(
      const std::shared_ptr<JsonConfigWrapper> &json_cfg) { return 0; }
#ifdef X3
  // dynamic config init vpsmodule
  virtual int Init(std::shared_ptr<VpsConfig> &vps_cfg) { return 0; }
#endif
  virtual int DeInit() { return 0; }
  virtual int Start() { return 0; }
  virtual int Stop() { return 0; }
  virtual int GetGroupId() { return group_id_; }
  virtual int LoadConfig(const std::string &input_cfg_file) { return 0; }
  // get image buffer info, including physical address and virtual address
  virtual int GetImageBuffer(
      std::shared_ptr<ImageFrame> &output_buf_info) { return 0; }
  // run vps process for static image
  virtual int RunVpsProcess(void *input_nv12) { return 0; }
  // get source image ouput by vps module
  virtual int GetFrame(int vps_chn, void *output_nv12) { return 0; }
  // free source image by vps module
  virtual int FreeFrame(int vps_chn, void *input_nv12) { return 0; }
  // get pyramid image ouput by vps module
  virtual int GetPyramidFrame(int vps_chn,
      void *output_src, void *output_pym) { return 0; }
  // free pyramid image by vps module
  virtual int FreePyramidFrame(int vps_chn,
      void *input_src, void *input_pym) { return 0; }
  // send a image to vps module
  virtual int SendFrame(void *input_nv12) { return 0; }

  virtual int GetVpsIpuChn(std::vector<int> &ipu_chn_list) { return 0; }
  virtual int GetVpsPymChn(std::vector<int> &pym_chn_list) { return 0; }
  virtual int GetConfig(std::shared_ptr<void> &output_cfg) { return 0; }
  virtual void SetConfig(std::shared_ptr<void> &input_cfg) {}

  virtual void* CreateSrcAddrInfo();
  virtual void* CreatePymAddrInfo();
  virtual void* CreateSrcContext(int chn, void* buf);
  int GetSrcContext(void* ctx, int &chn, void* &vio_buf);
  virtual int ConvertSrcInfo(void *src_buf,
      std::shared_ptr<ImageFrame> src_img);
  virtual int ConvertPymInfo(void *pym_buf,
      std::shared_ptr<PyramidFrame> pym_img);
  virtual int ConvertVioBufInfo(std::shared_ptr<ImageFrame> src_img, void *dst);
  virtual void SetVpsInputSize(const int &width, const int &height) {
    vps_input_width_ = width;
    vps_input_height_ = height;
  }
  virtual void GetVpsInputSize(int &width, int &height) {
    width = vps_input_width_;
    height = vps_input_height_;
  }
  virtual bool GetVpsSyncStatus() { return vps_sync_state_; }
  virtual void SetVpsSyncStatus(const bool &value) { vps_sync_state_ = value; }
  virtual void SetFrameIdCb(
      const FrameIdCb &cb_func) { cb_func_ = cb_func; }

 protected:
  virtual int LoadConfigFile(const std::string &input_cfg_file,
      std::shared_ptr<JsonConfigWrapper> &output_json_cfg);
  void GetDumpNum(const std::string &input, bool &dump_en, int &dump_num);
  int HbDumpToFile2Plane(char *filename, char *src_buf,
      char *src_buf1, unsigned int size, unsigned int size1,
      int width, int height, int stride);
  int HbDumpPymData(int grp_id, int pym_chn, pym_buffer_t *out_pym_buf);
  int HbDumpPymLayerData(int grp_id, int pym_chn, int layer,
      pym_buffer_t *out_pym_buf);
  int HbDumpIpuData(int grp_id, int ipu_chn, hb_vio_buffer_t *out_ipu_buf);
  int HbCheckPymData(VpsPymInfo &pym_info, const int &ds_layer_num);
  int HbTimeoutWait(sem_t &sem, const uint64_t &timeout_ms);
  int HbManagerPymBuffer(int max_buf_num);
  void HbAllocPymBuffer();
  void HbFreePymBuffer();
  void HbPrintPymInfo(const VpsPymInfo &curr_pym_info,
      const VpsPymInfo &last_pym_info);

 protected:
  int channel_id_ = -1;
  int group_id_ = 0;
  int vps_input_width_ = 0;
  int vps_input_height_ = 0;
  sem_t vps_sync_sem_;
  bool vps_sync_state_ = false;
  FrameIdCb cb_func_;
  sem_t pym_sem_;
  sem_t ipu_sem_;
  std::shared_ptr<RingQueue<VpsPymInfo>> pym_rq_ = nullptr;
  std::shared_ptr<RingQueue<VpsIpuInfo>> ipu_rq_ = nullptr;
  VpsPymInfo last_pym_info_ = { 0 };

 private:
  std::mutex mutex_;
  int dump_index_ = 0;
  int consumed_pym_buffers_ = 0;
};

}  // namespace videosource
#endif  // VIDEO_SOURCE_VPS_VPS_MODULE_H_
