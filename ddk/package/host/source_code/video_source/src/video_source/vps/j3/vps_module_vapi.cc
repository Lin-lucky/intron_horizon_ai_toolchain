/**
 *  Copyright (C) 2021 Horizon Robotics Inc.
 *  All rights reserved.
 *  @Author: yong.wu
 *  @Email: <yong.wu@horizon.ai>
 *  @Date: 2021-04-16
 *  @Version: v0.0.1
 *  @Brief: implemenation of video process system.
 */

#include "video_source/vps/j3/vps_module_vapi.h"
#include <cstdint>
#include <memory>
#include <string>
#include "video_source/vps/vps_data_type.h"
#include "hobotlog/hobotlog.hpp"
#include "utils/log.h"
#include "vps_module_vapi.h"

namespace videosource {

std::once_flag J3VpsModuleVapi::vps_init_flag_;
std::atomic_int J3VpsModuleVapi::task_num_{0};
std::vector<J3VpsModuleVapi::IpuCfgInfo> J3VpsModuleVapi::ipu_cfg_list_;
std::vector<J3VpsModuleVapi::PymCfgInfo> J3VpsModuleVapi::pym_cfg_list_;
std::vector<std::vector<int>> J3VpsModuleVapi::vps_ipu_valid_index_;

int J3VpsModuleVapi::LoadConfig(
    const std::shared_ptr<JsonConfigWrapper> &vps_cfg_json) {
  if (vps_cfg_json == nullptr) {
    LOGE << "vps_cfg_json is nullptr";
    return -1;
  }
  if (vps_cfg_json->HasMember("vio_cfg_file")) {
    vio_cfg_file_ = vps_cfg_json->GetSTDStringValue("vio_cfg_file");
    LOGI << "vio_cfg_file: " << vio_cfg_file_;
  }
  if (vps_cfg_json->HasMember("gdc_en")) {
    gdc_en_list_ = vps_cfg_json->GetIntArray("gdc_en");
    for (size_t chn_index = 0; chn_index < gdc_en_list_.size(); chn_index++) {
      LOGI << "channel_id: " << chn_index
        << " gdc_en: " << gdc_en_list_[chn_index];
    }
  }
  if (vps_cfg_json->HasMember("bind_source_chn")) {
    bind_source_chn_list_ = vps_cfg_json->GetIntArray("bind_source_chn");
    for (size_t chn_index = 0; chn_index < bind_source_chn_list_.size();
        chn_index++) {
      LOGI << "channel_id: " << chn_index
        << " bind_source_chn: " << bind_source_chn_list_[chn_index];
    }
  }
  if (vps_cfg_json->HasMember("ipu_chn")) {
    ipu_chn_list_ = vps_cfg_json->GetIntArray("ipu_chn");
    for (size_t chn_index = 0; chn_index < ipu_chn_list_.size();
        chn_index++) {
      LOGI << "channel_id: " << chn_index
        << " ipu_chn: " << ipu_chn_list_[chn_index];
    }
  }
  return 0;
}

int J3VpsModuleVapi::Init(
    const std::shared_ptr<JsonConfigWrapper> &json_cfg) {
  int ret = -1;

  if (init_flag_ == true) {
    LOGE << "J3VpsModuleVapi has been init, init_flag: " << init_flag_
      << " group_id: " << group_id_;
    return 0;
  }

  ret = LoadConfig(json_cfg);
  if (ret) {
    LOGE << "J3VpsModuleVapi Init failed!";
    return ret;
  }

  AddVpsTask();
  std::call_once(vps_init_flag_,
      [&] () {
      // vio init all device
      LOGI << "Enter j3 hb_vio_init , group_id: " << group_id_;
      ret = CheckVioConfig(vio_cfg_file_);
      HOBOT_CHECK(ret == 0) << "check vio config failed, ret: " << ret;
      auto start_time = std::chrono::system_clock::now();
      ret = hb_vio_init(vio_cfg_file_.c_str());
      auto curr_time = std::chrono::system_clock::now();
      auto cost_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          curr_time - start_time).count();
      HOBOT_CHECK(ret == 0) << "hb_vio_init failed, ret: " << ret;
      LOGI << "j3 hb_vio_init cost_time: " << cost_time << "ms";
    });

  UpdateSourceChannel();
  sem_init(&pym_sem_, 0, 0);
  init_flag_ = true;
  LOGI << "J3VpsModuleVapi init success," << " group_id: " << group_id_;
  return 0;
}

int J3VpsModuleVapi::DeInit() {
  int ret = -1;
  if (init_flag_ == false) {
    LOGE << "J3VpsModuleVapi has not init, init_flag: " << init_flag_
      << " group_id: " << group_id_;
    return -1;
  }

  FreeVpsTask();
  if (GetVpsTaskNum() == 0) {
    // vio deinit all pipeline
    ret = hb_vio_deinit();
    HOBOT_CHECK(ret == 0) << "hb_vio_deinit failed, ret: " << ret;
  }

  sem_destroy(&pym_sem_);
  init_flag_ = false;
  LOGI << "J3VpsModuleVapi deinit success, group_id: " << group_id_;
  return 0;
}

int J3VpsModuleVapi::Start() {
  int ret = -1;

  if (group_id_ != this_src_chn_) {
    LOGW << "this pipe_id: " << group_id_
      << " bind to other channel: " << this_src_chn_;
  }
  ret = hb_vio_start_pipeline(this_src_chn_);
  HOBOT_CHECK(ret == 0) << "hb_vio_deinit failed, ret: " << ret
    << " group_id: " << group_id_
    << " this_src_chn: " << this_src_chn_;
  ret = VpsCreateGetDataThread();
  if (ret) {
    LOGE << "Vps create pym thread failed!!!";
    return ret;
  }
  start_flag_ = true;
  LOGI << "J3VpsModuleVapi start success, "
    << " group_id: " << group_id_
    << " this_src_chn: " << this_src_chn_;
  return 0;
}

int J3VpsModuleVapi::Stop() {
  int ret = -1;

  start_flag_ = false;
  ret = VpsDestoryGetDataThread();
  if (ret) {
    LOGE << "Vps destory pym thread failed!!!";
    return ret;
  }
  ret = hb_vio_stop_pipeline(this_src_chn_);
  if (group_id_ == this_src_chn_) {
    HOBOT_CHECK(ret == 0) << "hb_vio_stop_pipeline failed, ret: " << ret
      << " group_id: " << group_id_
      << " this_src_chn: " << this_src_chn_;
  }
  LOGI << "J3VpsModuleVapi stop success, "
    << " group_id: " << group_id_
    << " this_src_chn: " << this_src_chn_;
  return 0;
}

int J3VpsModuleVapi::VpsCreateGetDataThread() {
  LOGI << "Enter VpsCreateGetDataThread,group_id: " << group_id_
    << " pym_start_flag: " << pym_start_flag_;

  if (pym_start_flag_ == false && pym_thread_ == nullptr) {
    pym_thread_ = std::make_shared<std::thread>(
        &J3VpsModuleVapi::HbGetPymDataThread, this);
    pym_start_flag_ = true;
  }
  return 0;
}

int J3VpsModuleVapi::VpsDestoryGetDataThread() {
  LOGI << "Enter VpsDestoryGetDataThread,group_id: " << group_id_
    << " pym_start_flag: " << pym_start_flag_;

  // destory pym thread
  if (pym_start_flag_ == true && pym_thread_ != nullptr) {
    pym_start_flag_ = false;
    pym_thread_->join();
    pym_thread_ = nullptr;
  }
  LOGI << "Quit VpsDestoryDataThread, group_id: " << group_id_;
  return 0;
}

int J3VpsModuleVapi::GetVpsPymChn(std::vector<int> &pym_chn_list) {
  if (init_flag_ == false) {
    LOGE << "vps module has not init";
    return -1;
  }
  int chn_index = static_cast<int>(ipu_data_type_);  // use ipu index
  std::vector<int> pym_valid_chn;
  pym_valid_chn.push_back(chn_index);
  pym_chn_list = pym_valid_chn;
  return 0;
}

int J3VpsModuleVapi::GetPyramidFrame(int vps_chn,
    void *output_src, void *output_pym) {
  int ret = -1;
  int timeout = 2000;  // 2000ms
  VpsPymInfo pym_info = { 0 };

  if (output_src == nullptr || output_pym == nullptr) {
    LOGE << "get pyramid frame failed,"
      << " output_src or output_pym is nullptr"
      << " output_src: " << output_src
      << " output_pym: " << output_pym;
    return -1;
  }
  pym_buffer_t *pym_buf = reinterpret_cast<pym_buffer_t *>(output_pym);
  hb_vio_buffer_t *src_buf = reinterpret_cast<hb_vio_buffer_t *>(output_src);

  ret = HbTimeoutWait(pym_sem_, timeout);
  if (ret < 0) {
    LOGE << "Wait Pym Data Timeout!!!"
      << " group_id: " << group_id_;
    return ret;
  }
  if (pym_rq_ == nullptr) {
    LOGE << "pym ring_queue is nullptr";
    return -1;
  }
  auto rv = pym_rq_->Pop(pym_info);
  if (!rv) {
    LOGE << "No Pym info in RingQueue!"
      << " group_id: " << group_id_;
    return -1;
  }
  *pym_buf = pym_info.pym_buf;
  *src_buf = pym_info.src_buf;
  HbAllocPymBuffer();
  // dump pramid frame
  {
    bool dump_en = false;
    int dump_num = 0;
    GetDumpNum("./vps_output.txt", dump_en, dump_num);
    if (dump_en == true && (dump_count_++ < dump_num || dump_num < 0)) {
      // dump all pym layer
      HbDumpPymData(this_src_chn_, this_ipu_chn_, pym_buf);
    }
  }
  return 0;
}

int J3VpsModuleVapi::FreePyramidFrame(
    int vps_chn, void *input_src, void *input_pym) {
  int ret = -1;
  int this_src_chn = this_src_chn_;
  if (input_src == nullptr || input_pym == nullptr) {
    LOGE << "free pyramid frame failed,"
      << " input_src or input_pym is nullptr"
      << " input_src: " << input_src
      << " input_pym: " << input_pym;
    return -1;
  }
  hb_vio_buffer_t *src_buf = reinterpret_cast<hb_vio_buffer_t*>(input_src);
  ret = hb_vio_free_ipubuf(this_src_chn, src_buf);
  if (ret) {
    LOGE << "hb_vio_free_ipubuf failed, ret: " << ret
      << " group_id: " << group_id_
      << " this_src_chn: " << this_src_chn;
    return ret;
  }
  ret = hb_vio_free_pymbuf(this_src_chn, HB_VIO_PYM_DATA, input_pym);
  if (ret) {
    LOGE << "hb_vio_free_pymbuf failed, ret: " << ret
      << " group_id: " << group_id_
      << " this_src_chn: " << this_src_chn;
    return ret;
  }
  HbFreePymBuffer();
  return 0;
}

void J3VpsModuleVapi::UpdateSourceChannel() {
  int ret = -1;
  size_t group_id = static_cast<size_t>(group_id_);

  if ((bind_source_chn_list_.size() == 0)
      || ((bind_source_chn_list_.size() > group_id)
       && (bind_source_chn_list_[group_id] < 0))) {
    this_src_chn_ = group_id;
    HOBOT_CHECK(vps_ipu_valid_index_.size() > group_id);
    auto ipu_valid_index = vps_ipu_valid_index_[group_id];
    HOBOT_CHECK(ipu_valid_index.size() > 0);
    this_ipu_chn_ = ipu_valid_index[0];
  } else {   // use bind source
    HOBOT_CHECK(bind_source_chn_list_.size() > group_id);
    HOBOT_CHECK(ipu_chn_list_.size() > group_id);
    this_src_chn_ = bind_source_chn_list_[group_id];
    this_ipu_chn_ = ipu_chn_list_[group_id];
  }
  // update ipu_cfg and pym_cfg
  LOGI << "ipu cfg list size: " << ipu_cfg_list_.size();
  LOGI << "pym cfg list size: " << pym_cfg_list_.size();
  HOBOT_CHECK(ipu_cfg_list_.size() > group_id);
  HOBOT_CHECK(pym_cfg_list_.size() > group_id);
  ipu_cfg_ = ipu_cfg_list_[this_src_chn_];
  pym_cfg_ = pym_cfg_list_[this_src_chn_];
  // convert ipu chn to type
  ret = ConvertIpuChn2Type(this_ipu_chn_);
  HOBOT_CHECK(ret == 0);
  LOGW << "group_id: " << group_id
    << " this_src_chn: " << this_src_chn_
    << " this_ipu_chn: " << this_ipu_chn_
    << " ipu_data_type: " << ipu_data_type_;
  HOBOT_CHECK(pym_cfg_.img_src_sel == 0)
    << "pym img_src_sel: " << pym_cfg_.img_src_sel
    << " must set 0(pym-offline mode)";
}

void J3VpsModuleVapi::HbGetPymDataThread() {
  // start get pym data
  int ret = -1;
  int index = 0;
  VpsPymInfo pym_info;
  int vio_fps = 0;
  int this_src_chn = this_src_chn_;
  int pym_chn = this_ipu_chn_;
  int frame_depth = pym_cfg_.output_buf_num;
  int max_pym_buffer = frame_depth - 1;
  size_t group_id = static_cast<size_t>(group_id_);
  LOGI << "Enter Get PYM Data Thread, pipe_id: " << group_id
    << " pym_start_flag: " << pym_start_flag_
    << " max_pym_buffer is: " << max_pym_buffer;
  pym_rq_ = std::make_shared<RingQueue<VpsPymInfo>>();
  pym_rq_->Init(max_pym_buffer, [](VpsPymInfo &elem) {
    int ret;
    ret = hb_vio_free_ipubuf(elem.grp_id, &elem.src_buf);
    if (ret) {
      LOGE << "hb_vio_free_ipubuf error, ret: " << ret;
    }
    ret = hb_vio_free_pymbuf(elem.grp_id, HB_VIO_PYM_DATA, &elem.pym_buf);
    if (ret) {
      LOGE << "hb_vio_free_pymbuf error, ret: " << ret;
    }
  });

  pym_start_time_ = std::chrono::system_clock::now();
  while (pym_start_flag_) {
    memset(&pym_info, 0, sizeof(VpsPymInfo));

    ret = HbManagerPymBuffer(max_pym_buffer);
    if (ret) {
      LOGE  << "pipe_id: " << group_id
        << " this_src_chn: " << this_src_chn
        << " HbManagerPymBuffer error, ret: " << ret;
      continue;
    }
    // 1. get ipu buffer
    ret = hb_vio_get_data(this_src_chn, ipu_data_type_, &pym_info.src_buf);
    if (ret) {
      if (pym_start_flag_) {
        LOGE << "pipe_id: " << group_id
          << " hb_vio_get_data, ipu_data_type: " << ipu_data_type_
          << " error, ret: " << ret
          << " this_src_chn: " << this_src_chn;
      }
      continue;
    }
    // 2. send ipu image to pym
    ret = hb_vio_run_pym(this_src_chn, &pym_info.src_buf);
    if (ret) {
      if (pym_start_flag_) {
        LOGE << "pipe_id: " << group_id
          << " hb_vio_run_pym error, ret: " << ret
          << " this_src_chn: " << this_src_chn;
      }
      hb_vio_free_ipubuf(this_src_chn, &pym_info.src_buf);
      continue;
    }
    // 3. get pym image
    ret = hb_vio_get_data(this_src_chn, HB_VIO_PYM_DATA, &pym_info.pym_buf);
    if (ret) {
      if (pym_start_flag_) {
        LOGE << "pipe_id: " << group_id
          << " hb_vio_get_data(pym) error, ret: " << ret
          << " this_src_chn: " << this_src_chn;
      }
      hb_vio_free_ipubuf(this_src_chn, &pym_info.src_buf);
      continue;
    }

    pym_info.grp_id = this_src_chn;
    pym_info.pym_chn = pym_chn;
    pym_info.buf_index = (index++) % frame_depth;

    /* calculate vio fps */
    {
      vio_fps++;
      auto curr_time = std::chrono::system_clock::now();
      auto cost_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            curr_time - pym_start_time_);
      if (cost_time.count() >= 1000) {
        LOGW << "group_id:" << group_id << " vio_fps:" << vio_fps;
        pym_start_time_ = std::chrono::system_clock::now();
        vio_fps = 0;
      }
    }

    // print pym info
    HbPrintPymInfo(pym_info, last_pym_info_);
    // store last pym info
    last_pym_info_ = pym_info;
    pym_rq_->Push(pym_info);
    sem_post(&pym_sem_);
  }
  pym_rq_->Clear();
  LOGD << "Quit get pym data thread, group_id: " << group_id;
}

int J3VpsModuleVapi::ConvertIpuChn2Type(const int &ipu_chn) {
  switch (ipu_chn) {
    case 0:
      ipu_data_type_ = HB_VIO_IPU_DS0_DATA;
      break;
    case 1:
      ipu_data_type_ = HB_VIO_IPU_DS1_DATA;
      break;
    case 2:
      ipu_data_type_ = HB_VIO_IPU_DS2_DATA;
      if (ipu_cfg_.ds2_to_ddr_en == false) {
        LOGE << "ds2_to_ddr_en is false, must set true";
        return -1;
      }
      break;
    case 3:
      ipu_data_type_ = HB_VIO_IPU_DS3_DATA;
      break;
    case 4:
      ipu_data_type_ = HB_VIO_IPU_DS4_DATA;
      break;
    case 5:
      ipu_data_type_ = HB_VIO_IPU_US_DATA;
      break;
    default:
      LOGE << "Unsupport ipu_chn: " << ipu_chn;
      return -1;
  }
  return 0;
}

int J3VpsModuleVapi::CheckIpuConfig(
    const std::shared_ptr<JsonConfigWrapper> &ipu_json) {
  if (ipu_json == nullptr) {
    LOGE << "ipu_json is nullptr";
    return -1;
  }
  std::vector<int> ipu_valid_index;
  // 1. check ipu config
  memset(&ipu_cfg_, 0, sizeof(IpuCfgInfo));
  auto ipu_config_json = ipu_json->GetSubConfig("ipu_config");
  HOBOT_CHECK(ipu_config_json);
  ipu_cfg_.source_sel = ipu_config_json->GetIntValue("source_sel");
  ipu_cfg_.ds2_to_ddr_en = ipu_config_json->GetBoolValue("ds2_to_ddr_en");
  LOGI << "ipu souce sel: " << ipu_cfg_.source_sel;
  LOGI << "ipu ds2_to_ddr_en: " << ipu_cfg_.ds2_to_ddr_en;
  // 2. check ipu downscale and upscale config
  auto cfg_size_json = ipu_json->GetSubConfig("cfg_size");
  HOBOT_CHECK(cfg_size_json);
  auto ipu_ds_config_json_list = cfg_size_json->GetSubConfigList("ipu_ds_config");
  HOBOT_CHECK(ipu_ds_config_json_list.size() > 0);
  auto ipu_us_config_json = cfg_size_json->GetSubConfig("ipu_us_config");
  HOBOT_CHECK(ipu_us_config_json);
  for (size_t i = 0; i < ipu_ds_config_json_list.size(); i++) {
    auto ipu_ds_config_json = ipu_ds_config_json_list[i];
    std::string roi_en_name = "ds" + std::to_string(i) + "_roi_en";
    std::string roi_x_name = "ds" + std::to_string(i) + "_roi_start_x";
    std::string roi_y_name = "ds" + std::to_string(i) + "_roi_start_y";
    std::string roi_w_name = "ds" + std::to_string(i) + "_roi_start_w";
    std::string roi_h_name = "ds" + std::to_string(i) + "_roi_start_h";
    std::string scale_en_name = "downscale_ds" + std::to_string(i) + "_en";
    std::string scale_w_name = "ds" + std::to_string(i) + "_tag_width";
    std::string scale_h_name = "ds" + std::to_string(i) + "_tag_height";
    std::string buf_num_name = "ds" + std::to_string(i) + "_buf_num";
    ipu_cfg_.ds_info[i].roi_en =
      ipu_ds_config_json->GetBoolValue(roi_en_name);
    ipu_cfg_.ds_info[i].roi_x =
      ipu_ds_config_json->GetIntValue(roi_x_name);
    ipu_cfg_.ds_info[i].roi_y =
      ipu_ds_config_json->GetIntValue(roi_y_name);
    ipu_cfg_.ds_info[i].roi_w =
      ipu_ds_config_json->GetIntValue(roi_w_name);
    ipu_cfg_.ds_info[i].roi_h =
      ipu_ds_config_json->GetIntValue(roi_h_name);
    ipu_cfg_.ds_info[i].scale_en =
      ipu_ds_config_json->GetBoolValue(scale_en_name);
    ipu_cfg_.ds_info[i].scale_w =
      ipu_ds_config_json->GetIntValue(scale_w_name);
    ipu_cfg_.ds_info[i].scale_h =
      ipu_ds_config_json->GetIntValue(scale_h_name);
    ipu_cfg_.ds_info[i].buf_num =
      ipu_ds_config_json->GetIntValue(buf_num_name);
    LOGI << "ipu ds " << i << " info:"
      << " roi_en: " << ipu_cfg_.ds_info[i].roi_en
      << " roi_x: " << ipu_cfg_.ds_info[i].roi_x
      << " roi_y: " << ipu_cfg_.ds_info[i].roi_y
      << " roi_w: " << ipu_cfg_.ds_info[i].roi_w
      << " roi_h: " << ipu_cfg_.ds_info[i].roi_h
      << " scale_en: " << ipu_cfg_.ds_info[i].scale_en
      << " scale_w: " << ipu_cfg_.ds_info[i].scale_w
      << " scale_h: " << ipu_cfg_.ds_info[i].scale_h
      << " buf_num: " << ipu_cfg_.ds_info[i].buf_num;
    if (ipu_cfg_.ds_info[i].scale_en == true ||
        ipu_cfg_.ds_info[i].roi_en == true) {
      ipu_valid_index.push_back(i);
    }
  }
  {
    std::string roi_en_name = "upscale_roi_en";
    std::string roi_x_name = "us_roi_start_x";
    std::string roi_y_name = "us_roi_start_y";
    std::string roi_w_name = "us_roi_width";
    std::string roi_h_name = "us_roi_height";
    std::string scale_en_name = "upscale_us_en";
    std::string scale_w_name = "us_tag_width";
    std::string scale_h_name = "us_tag_height";
    std::string buf_num_name = "us_buf_num";
    ipu_cfg_.us_info.roi_en =
      ipu_us_config_json->GetBoolValue(roi_en_name);
    ipu_cfg_.us_info.roi_x =
      ipu_us_config_json->GetIntValue(roi_x_name);
    ipu_cfg_.us_info.roi_y =
      ipu_us_config_json->GetIntValue(roi_y_name);
    ipu_cfg_.us_info.roi_w =
      ipu_us_config_json->GetIntValue(roi_w_name);
    ipu_cfg_.us_info.roi_h =
      ipu_us_config_json->GetIntValue(roi_h_name);
    ipu_cfg_.us_info.scale_en =
      ipu_us_config_json->GetBoolValue(scale_en_name);
    ipu_cfg_.us_info.scale_w =
      ipu_us_config_json->GetIntValue(scale_w_name);
    ipu_cfg_.us_info.scale_h =
      ipu_us_config_json->GetIntValue(scale_h_name);
    ipu_cfg_.us_info.buf_num =
      ipu_us_config_json->GetIntValue(buf_num_name);
    LOGI << "ipu us info:"
      << " roi_en: " << ipu_cfg_.us_info.roi_en
      << " roi_x: " << ipu_cfg_.us_info.roi_x
      << " roi_y: " << ipu_cfg_.us_info.roi_y
      << " roi_w: " << ipu_cfg_.us_info.roi_w
      << " roi_h: " << ipu_cfg_.us_info.roi_h
      << " scale_en: " << ipu_cfg_.us_info.scale_en
      << " scale_w: " << ipu_cfg_.us_info.scale_w
      << " scale_h: " << ipu_cfg_.us_info.scale_h
      << " buf_num: " << ipu_cfg_.us_info.buf_num;
    if (ipu_cfg_.us_info.scale_en == true ||
        ipu_cfg_.us_info.roi_en == true) {
      int us_index = 5;  // ds5 is upscale
      ipu_valid_index.push_back(us_index);
    }
  }
  vps_ipu_valid_index_.push_back(ipu_valid_index);
  ipu_cfg_list_.push_back(ipu_cfg_);
  return 0;
}

int J3VpsModuleVapi::CheckPymConfig(
    const std::shared_ptr<JsonConfigWrapper> &pym_json) {
  if (pym_json == nullptr) {
    LOGE << "pym_json is nullptr";
    return -1;
  }
  memset(&pym_cfg_, 0, sizeof(PymCfgInfo));
  auto pym_ctrl_config_json = pym_json->GetSubConfig("pym_ctrl_config");
  HOBOT_CHECK(pym_ctrl_config_json);
  pym_cfg_.img_src_sel = pym_ctrl_config_json->GetIntValue("img_src_sel");
  pym_cfg_.frame_id = pym_ctrl_config_json->GetIntValue("frame_id");
  pym_cfg_.src_w = pym_ctrl_config_json->GetIntValue("src_w");
  pym_cfg_.src_h = pym_ctrl_config_json->GetIntValue("src_h");
  pym_cfg_.ds_layer_en = pym_ctrl_config_json->GetIntValue("ds_layer_en");
  pym_cfg_.ds_uv_bypass = pym_ctrl_config_json->GetIntValue("ds_uv_bypass");
  pym_cfg_.us_layer_en = pym_ctrl_config_json->GetIntValue("us_layer_en");
  pym_cfg_.us_uv_bypass = pym_ctrl_config_json->GetIntValue("us_uv_bypass");
  pym_cfg_.ddr_in_buf_num = pym_ctrl_config_json->GetIntValue("ddr_in_buf_num");
  pym_cfg_.output_buf_num = pym_ctrl_config_json->GetIntValue("output_buf_num");
  pym_cfg_.timeout = pym_ctrl_config_json->GetIntValue("timeout");
  LOGI << "pym config: "
    << " img_src_sel: " << pym_cfg_.img_src_sel
    << " frame_id: " << pym_cfg_.frame_id
    << " src_w" << pym_cfg_.src_w
    << " src_h: " << pym_cfg_.src_h
    << " ds_layer_en: " << pym_cfg_.ds_layer_en
    << " ds_uv_bypass: " << pym_cfg_.ds_uv_bypass
    << " us_layer_en: " << pym_cfg_.us_layer_en
    << " us_uv_bypass: " << pym_cfg_.us_uv_bypass
    << " ddr_in_buf_num: " << pym_cfg_.ddr_in_buf_num
    << " output_buf_num: " << pym_cfg_.output_buf_num
    << " timeout: " << pym_cfg_.timeout;

  auto pym_ds_config_json = pym_json->GetSubConfig("pym_ds_config");
  for (int ds_idx = 0; ds_idx < MAX_PYM_DS_NUM; ds_idx++) {
    if (ds_idx % 4 == 0) continue;
    std::string factor_name = "factor_" + std::to_string(ds_idx);
    std::string roi_x_name = "roi_x_" + std::to_string(ds_idx);
    std::string roi_y_name = "roi_y_" + std::to_string(ds_idx);
    std::string roi_w_name = "roi_w_" + std::to_string(ds_idx);
    std::string roi_h_name = "roi_h_" + std::to_string(ds_idx);
    pym_cfg_.ds_info[ds_idx].factor =
      pym_ds_config_json->GetIntValue(factor_name);
    pym_cfg_.ds_info[ds_idx].roi_x =
      pym_ds_config_json->GetIntValue(roi_x_name);
    pym_cfg_.ds_info[ds_idx].roi_y =
      pym_ds_config_json->GetIntValue(roi_y_name);
    pym_cfg_.ds_info[ds_idx].roi_w =
      pym_ds_config_json->GetIntValue(roi_w_name);
    pym_cfg_.ds_info[ds_idx].roi_h =
      pym_ds_config_json->GetIntValue(roi_h_name);
    LOGI << "ds_pym_layer: "<< ds_idx << " ds_roi_x: "
      << pym_cfg_.ds_info[ds_idx].roi_x;
    LOGI << "ds_pym_layer: "<< ds_idx << " ds_roi_y: "
      << pym_cfg_.ds_info[ds_idx].roi_y;
    LOGI << "ds_pym_layer: "<< ds_idx << " ds_roi_width: "
      << pym_cfg_.ds_info[ds_idx].roi_w;
    LOGI << "ds_pym_layer: "<< ds_idx << " ds_roi_height: "
      << pym_cfg_.ds_info[ds_idx].roi_h;
    LOGI << "ds_pym_layer: "<< ds_idx << " ds_factor: "
      << pym_cfg_.ds_info[ds_idx].factor;
  }

  auto pym_us_config_json = pym_json->GetSubConfig("pym_us_config");
  for (int us_idx = 0; us_idx < MAX_PYM_US_NUM; us_idx++) {
    std::string factor_name = "factor_" + std::to_string(us_idx);
    std::string roi_x_name = "roi_x_" + std::to_string(us_idx);
    std::string roi_y_name = "roi_y_" + std::to_string(us_idx);
    std::string roi_w_name = "roi_w_" + std::to_string(us_idx);
    std::string roi_h_name = "roi_h_" + std::to_string(us_idx);
    pym_cfg_.us_info[us_idx].factor =
      pym_us_config_json->GetIntValue(factor_name);
    pym_cfg_.us_info[us_idx].roi_x =
      pym_us_config_json->GetIntValue(roi_x_name);
    pym_cfg_.us_info[us_idx].roi_y =
      pym_us_config_json->GetIntValue(roi_y_name);
    pym_cfg_.us_info[us_idx].roi_w =
      pym_us_config_json->GetIntValue(roi_w_name);
    pym_cfg_.us_info[us_idx].roi_h =
      pym_us_config_json->GetIntValue(roi_h_name);
    LOGI << "us_pym_layer: "<< us_idx << " us_roi_x: "
      << pym_cfg_.us_info[us_idx].roi_x;
    LOGI << "us_pym_layer: "<< us_idx << " us_roi_y: "
      << pym_cfg_.us_info[us_idx].roi_y;
    LOGI << "us_pym_layer: "<< us_idx << " us_roi_width: "
      << pym_cfg_.us_info[us_idx].roi_w;
    LOGI << "us_pym_layer: "<< us_idx << " us_roi_height: "
      << pym_cfg_.us_info[us_idx].roi_h;
    LOGI << "us_pym_layer: "<< us_idx << " us_factor: "
      << pym_cfg_.us_info[us_idx].factor;
  }
  pym_cfg_list_.push_back(pym_cfg_);
  return 0;
}

int J3VpsModuleVapi::CheckVioConfig(const std::string &vio_config_file) {
  int ret = -1;
  std::shared_ptr<JsonConfigWrapper> vio_cfg_json = nullptr;

  ret = LoadConfigFile(vio_config_file, vio_cfg_json);
  if (ret) {
    LOGE << "load vio config file failed, ret: " << ret;
    return ret;
  }

  std::vector<std::shared_ptr<JsonConfigWrapper>> pipe_json_list;
  for (int i = 0; i < MAX_PIPE_NUM; i++) {
    std::string pipe_name = "pipeline" + std::to_string(i);
    LOGI << "check vio config, pipe_name: " << pipe_name;
    auto pipe_json = vio_cfg_json->GetSubConfig(pipe_name);
    if (pipe_json) {
      pipe_json_list.push_back(pipe_json);
    }
  }

  for (auto pipe_json : pipe_json_list) {
    // auto sif_json = pipe_json->GetSubConfig("sif");
    // auto isp_json = pipe_json->GetSubConfig("isp");
    // auto dwe_json = pipe_json->GetSubConfig("dwe");
    auto ipu_json = pipe_json->GetSubConfig("ipu");
    auto pym_json = pipe_json->GetSubConfig("pym");
    ret = CheckIpuConfig(ipu_json);
    HOBOT_CHECK(ret == 0)  << " check ipu config failed";
    ret = CheckPymConfig(pym_json);
    HOBOT_CHECK(ret == 0)  << " check pym config failed";
  }
  return 0;
}

}  // namespace videosource
