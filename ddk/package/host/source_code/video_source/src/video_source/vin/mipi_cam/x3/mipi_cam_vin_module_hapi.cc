/**
 * Copyright (c) 2021, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: yong.wu
 * @Mail: yong.wu@horizon.ai
 * @Date: 2021-04-16
 * @Version: v0.0.1
 * @Brief: mipi camera vin module for video source system.
 */
#include "video_source/vin/mipi_cam/x3/mipi_cam_vin_module_hapi.h"
#include <cstdint>
#include <dlfcn.h>
#include <string.h>
#include <string>
#include "video_source/vin/mipi_cam/x3/mipi_cam_vin_params.h"
#include "video_source/vps/vps_manager.h"
#include "hobotlog/hobotlog.hpp"

namespace videosource {

int MipiCamVinModule::Init(const std::string &config_file) {
  int ret = -1;

  ret = LoadConfig(config_file);
  if (ret) {
    LOGE << "MipiCamVinModule LoadConfig failed!";
    return ret;
  }

  int group_id = group_id_;
  int bind_chn_id = vin_cfg_.bind_chn_id;
  LOGI << "Enter vinmodule init, group_id: " << group_id_;
  // 1. if bind_chn_id is greater 0, not need VinInit and CamInit
  if (bind_chn_id >= 0) {
    VpsManager &manager = VpsManager::Get();
    ret = manager.FindGroupId(bind_chn_id, bind_group_id_);
    HOBOT_CHECK(ret == 0) << "bind_chn_id has not register...";
    bind_chn_id_ = bind_chn_id;
    LOGI << "Init MipiCamVinModule, the source is other vin module."
      << " channel_id: " << channel_id_
      << " group_id: " << group_id
      << " bind_chn_id: " << bind_chn_id
      << " bind_group_id: " << bind_group_id_;
    return 0;
  }
  // 2. owner vin module
  ret = VinInit();
  if (ret) {
    LOGE << "MipiCamVinModule VinInit failed!";
    return ret;
  }
  ret = CamInit();
  if (ret) {
    LOGE << "MipiCamVinModule CamInit failed!";
    return ret;
  }
  return 0;
}

int MipiCamVinModule::DeInit() {
  int ret;
  LOGI << "Enter MipiCamVinModule deinit, group_id: " << group_id_;
  int group_id = group_id_;
  int bind_chn_id = vin_cfg_.bind_chn_id;

  // 1. bind other vin module
  if (bind_chn_id >= 0) {
    LOGI << "DeInit MipiCamVinModule, the source is other vin module."
      << " channel_id: " << channel_id_
      << " group_id: " << group_id
      << " bind_chn_id: " << bind_chn_id
      << " bind_group_id: " << bind_group_id_;
    return 0;
  }

  // 2. owner vin module
  ret = VinDeInit();
  if (ret) {
    LOGE << "MipiCamVinModule VinDeInit failed!";
    return ret;
  }
  ret = CamDeInit();
  if (ret) {
    LOGE << "MipiCamVinModule CamDeInit failed!";
    return ret;
  }

  LOGD << "MipiCamVinModule deinit success...";
  return 0;
}

int MipiCamVinModule::HbCheckDataOutputMode() {
  bool vin_out_en = vin_out_en_;
  int group_id = group_id_;
  int bind_chn_id = vin_cfg_.bind_chn_id;
  int vin_vps_mode = vin_cfg_.vin_vps_mode;

  VinBufferConfig vin_buf_cfg = { 0 };
  this->GetVinBuf(vin_buf_cfg);
  vin_buf_cfg.max_queue_len = vin_frame_depth_;
  if (vin_vps_mode == VIN_ONLINE_VPS_OFFLINE
      || vin_vps_mode == VIN_OFFLINE_VPS_OFFINE
      || vin_vps_mode == VIN_SIF_ONLINE_DDR_ISP_DDR_VPS_ONLINE) {
    src_type_ = kISP_BUF_SOURCE;
    vin_buf_cfg.format =
      HorizonVisionPixelFormat::kHorizonVisionPixelFormatNV12;
  } else if (vin_vps_mode == VIN_OFFLINE_VPS_ONLINE
      || vin_vps_mode == VIN_OFFLINE_VPS_OFFINE
      || vin_vps_mode == VIN_SIF_OFFLINE_ISP_OFFLINE_VPS_ONLINE
      || vin_vps_mode == VIN_SIF_OFFLINE_VPS_OFFLINE
      || vin_vps_mode == VIN_SIF_OFFLINE) {
    src_type_ = kSIF_BUF_SOURCE;
  } else {
    LOGE << "UnSupport vin_vps_mode: " << vin_vps_mode
      << " in HbGetDataThread";
    return -1;
  }
  if (vin_out_en) {  // vin output enable
    vin_buf_cfg.use_vb = true;  // call InputRawData
  } else {  // vin output disable
    vin_buf_cfg.use_vb = false;  // call InputRawDataWithoutVb
    /**
     * if usb vb is true, vinmodule will been used,
     * otherwise vinmodule will not alloc ion memory
     */
    vin_start_flag_ = true;
  }
  vin_buf_cfg.decode_en = false;
  this->SetVinBuf(vin_buf_cfg, false);

  if (bind_chn_id < 0 && group_id >= 0) {
    src_group_id_ = group_id;  // get sif or isp data from ddr
  } else if (bind_chn_id >= 0) {
    src_group_id_ = bind_group_id_;   // multi-channel splice mode
  } else {
    LOGE << "UnSupport get data mode, "
      << " vin_out_en: " << vin_out_en
      << " vin_vps_mode: " << vin_vps_mode
      << " channel_id: " << channel_id_
      << " group_id: " << group_id
      << " bind_chn_id: " << bind_chn_id
      << " bind_group_id: " << bind_group_id_;
    return -1;
  }

  LOGI << "check data output success, "
    << " vin_out_en: " << vin_out_en
    << " vin_vps_mode: " << vin_vps_mode
    << " channel_id: " << channel_id_
    << " group_id: " << group_id
    << " bind_chn_id: " << bind_chn_id
    << " bind_group_id: " << bind_group_id_;
  return 0;
}

int MipiCamVinModule::VinCreateDataThread() {
  int ret = -1;
  LOGI << "Enter VinCreateDataThread, group_id: " << group_id_;

  if (data_thread_flag_ == true) {
    LOGE << "vin get data thread has been create";
    return -1;
  }

  ret = HbCheckDataOutputMode();
  if (ret) {
    LOGE << "vin module check data out mode failed, ret: " << ret;
    return ret;
  }

  if (data_thread_ == nullptr) {
    data_thread_ = std::make_shared<std::thread>(
        &MipiCamVinModule::HbGetDataThread, this);
    LOGI << "create data_thread: " << data_thread_;
  }

  data_thread_flag_ = true;
  return 0;
}

int MipiCamVinModule::VinDestoryDataThread() {
  int ret = -1;
  LOGI << "Enter VinDestoryDataThread, group_id: " << group_id_;
  if (data_thread_flag_ == false) {
    LOGW << "data thread has not create";
    return 0;
  }

  if (data_thread_ != nullptr) {
    is_running_ = false;
    data_thread_->join();
  }

  ret = VinModule::VinStop();
  if (ret) {
    LOGE << "vin module stop failed, ret: " << ret;
    return ret;
  }

  ret = VinModule::VinDeInit();
  if (ret) {
    LOGE << "vin module deinit failed, ret: " << ret;
    return ret;
  }

  data_thread_flag_ = false;
  LOGI << "Quit VinDestoryDataThread, group_id: " << group_id_;
  return 0;
}

int MipiCamVinModule::Start() {
  int ret;
  bool vin_out_en = vin_out_en_;
  int bind_chn_id = vin_cfg_.bind_chn_id;
  LOGI << "Enter vinmodule start, group_id: " << group_id_
    << " vin_out_en: " << vin_out_en
    << " bind_chn_id: " << bind_chn_id;

  // 1.bind other vin module
  if (bind_chn_id >= 0) {
    ret = VinCreateDataThread();
    if (ret) {
      LOGE << "MipiCamVinModule create data thread failed!!!";
      return ret;
    }
    return 0;
  }

  // 2.owner vin module
  ret = VinStart();
  if (ret) {
    LOGE << "MipiCamVinModule VinStart failed, group_id: " << group_id_;
    return ret;
  }
  ret = CamStart();
  if (ret) {
    LOGE << "MipiCamVinModule CamStart failed, group_id: " << group_id_;
    return ret;
  }
  if (vin_out_en == true) {
    ret = VinCreateDataThread();
    if (ret) {
      LOGE << "MipiCamVinModule create data thread failed!!!";
      return ret;
    }
  } else {
    vin_start_flag_ = true;  // mipi-cam vin->vps auto bind
  }
  return 0;
}

int MipiCamVinModule::Stop() {
  int ret;
  bool vin_out_en = vin_out_en_;
  int bind_chn_id = vin_cfg_.bind_chn_id;
  LOGI << "Enter vinmodule stop, group_id: " << group_id_
    << " vin_out_en: " << vin_out_en
    << " bind_chn_id: " << bind_chn_id;

  // 1. bind other vin module
  if (bind_chn_id >= 0) {
    ret = VinDestoryDataThread();
    if (ret) {
      LOGE << "MipiCamVinModule destroy data thread failed!"
        << " group_id: " << group_id_;
      return ret;
    }
    return 0;
  }

  // 2. owner vin module
  ret = CamStop();
  if (ret) {
    LOGE << "MipiCamVinModule CamStop failed, group_id: " << group_id_;
    return ret;
  }
  ret = VinStop();
  if (ret) {
    LOGE << "MipiCamVinModule VinStop failed, group_id: " << group_id_;
    return ret;
  }

  if (vin_out_en == true) {
    ret = VinDestoryDataThread();
    if (ret) {
      LOGE << "MipiCamVinModule destroy data thread failed!"
        << " group_id: " << group_id_;
      return ret;
    }
  }
  return 0;
}

int MipiCamVinModule::HbSensorPlgInit() {
  std::string sensor_plugin_path = vin_cfg_.sensor_info.sensor_plugin_path;
  std::string sensor_plugin_name = vin_cfg_.sensor_info.sensor_plugin_name;
  int sensor_plugin_type = vin_cfg_.sensor_info.sensor_plugin_type;
  int need_clk = vin_cfg_.sensor_info.need_clk;

  if (sensor_plugin_name.empty() || sensor_plugin_path.empty()) {
    LOGE << "sensor plugin name or path is nullptr";
    return -1;
  }
  std::string name_buff = sensor_plugin_path + "lib" +
    sensor_plugin_name + ".so";
  const char *c_name_buff = name_buff.c_str();
  sensor_plg_cfg_.sensor_fd = dlopen(c_name_buff, RTLD_LAZY);
  if (sensor_plg_cfg_.sensor_fd == nullptr) {
    LOGE << "dlopen plugin sensor: " << name_buff << " error: "
      << dlerror();
    return -1;
  }
  sensor_plg_cfg_.sensor_ops =
    dlsym(sensor_plg_cfg_.sensor_fd, sensor_plugin_name.c_str());
  if (sensor_plg_cfg_.sensor_ops == nullptr) {
    LOGE << "dlsym plugin sensor: " << name_buff << " error: "
      << dlerror();
    return -1;
  }
  sensor_plg_cfg_.sensor_name = sensor_plugin_name.c_str();
  sensor_plg_cfg_.sensor_type = sensor_plugin_type;
  sensor_plg_cfg_.need_clk = need_clk;

  return 0;
}

int MipiCamVinModule::HbSensorPlgDeInit() {
  if (sensor_plg_cfg_.sensor_fd) {
    dlclose(sensor_plg_cfg_.sensor_fd);
  }
  return 0;
}

int MipiCamVinModule::CamInit() {
  LOGI << "enter MipiCamVinModule CamInit...";
  int ret = -1;
  int group_id = group_id_;
  int sensor_id = vin_cfg_.sensor_info.sensor_id;
  int sensor_port = vin_cfg_.sensor_info.sensor_port;
  int i2c_bus = vin_cfg_.sensor_info.i2c_bus;
  int need_clk = vin_cfg_.sensor_info.need_clk;
  int serdes_idx = vin_cfg_.sensor_info.serdes_index;
  int serdes_port = vin_cfg_.sensor_info.serdes_port;
  int mipi_idx = vin_cfg_.mipi_info.mipi_index;

  if (need_clk == 1) {
    ret = HbEnableSensorClk(mipi_idx);
    if (ret) {
    LOGE << "hb enable sensor clk, ret: " << ret
      << " group_id: " << group_id
      << " sensor_id: " << sensor_id
      << " mipi_idx: " << mipi_idx;
      return ret;
    }
  }
  ret = HbSensorInit(group_id, sensor_id, i2c_bus, sensor_port,
      mipi_idx, serdes_idx, serdes_port);
  if (ret < 0) {
    LOGE << "hb sensor init error, ret: " << ret
      << " group_id: " << group_id
      << " sensor_id: " << sensor_id
      << " mipi_idx: " << mipi_idx;
    return ret;
  }
  ret = HbMipiInit(sensor_id, mipi_idx);
  if (ret < 0) {
    LOGE << "hb mipi init error, ret: " << ret
      << " group_id: " << group_id
      << " sensor_id: " << sensor_id
      << " mipi_idx: " << mipi_idx;
    return ret;
  }

  cam_init_flag_ = true;
  return 0;
}

int MipiCamVinModule::CamDeInit() {
  int ret = -1;
  int group_id = group_id_;
  int need_clk = vin_cfg_.sensor_info.need_clk;
  int mipi_idx = vin_cfg_.mipi_info.mipi_index;

  ret = HbMipiDeInit(mipi_idx);
  if (ret < 0) {
    LOGE << "hb mipi deinit error, ret: " << ret
      << " group_id: " << group_id << " mipi_idx: " << mipi_idx;
    return ret;
  }
  ret = HbSensorDeInit(group_id);
  if (ret < 0) {
    LOGE << "hb sensor deinit error, ret: " << ret
      << " group_id: " << group_id << " mipi_idx: " << mipi_idx;
    return ret;
  }
  if (need_clk == 1) {
    ret = HbDisableSensorClk(mipi_idx);
    if (ret) {
    LOGE << "hb disable sensor clock error, ret: " << ret
      << " group_id: " << group_id << " mipi_idx: " << mipi_idx;
      return ret;
    }
  }

  return 0;
}

int MipiCamVinModule::CamStart() {
  int ret = -1;
  int group_id = group_id_;
  int mipi_idx = vin_cfg_.mipi_info.mipi_index;

  LOGD << "Enter cam start, group_id: " << group_id
    << " mipi_idx: " << mipi_idx;
  ret = HbSensorStart(group_id);
  if (ret < 0) {
    LOGE << "hb sensor start error, ret: " << ret
      << " group_id: " << group_id << " mipi_idx: " << mipi_idx;
    return ret;
  }

  ret = HbMipiStart(mipi_idx);
  if (ret < 0) {
    LOGE << "hb mipi start error, ret: " << ret
      << " group_id: " << group_id << " mipi_idx: " << mipi_idx;
    return ret;
  }

  /**
   * write mirror register maybe behind the cam start,
   * otherwise write maybe not effective
   */
  int mirror_en = vin_cfg_.sensor_info.mirror_en;
  if (mirror_en) {
    ret = HbVinSetImageMirror();
    if (ret) {
      LOGE << "MipiCamVinModule HbVinSetImageMirror failed, group_id: "
        << group_id_;
      return ret;
    }
  }
  return 0;
}

int MipiCamVinModule::CamStop() {
  int ret = -1;
  int group_id = group_id_;
  int mipi_idx = vin_cfg_.mipi_info.mipi_index;

  LOGD << "Enter sensor stop, group_id: " << group_id;
  ret = HbSensorStop(group_id);
  if (ret < 0) {
    LOGE << "hb sensor stop error, ret: " << ret
      << " group_id: " << group_id << " mipi_idx: " << mipi_idx;
    return ret;
  }
  LOGD << "Enter mipi stop, mipi_idx: " << mipi_idx;
  ret = HbMipiStop(mipi_idx);
  if (ret < 0) {
    LOGE << "hb mipi stop error, ret: " << ret
      << " group_id: " << group_id << " mipi_idx: " << mipi_idx;
    return ret;
  }

  LOGD << "MipiCamVinModule CamStop success...";
  return 0;
}

int MipiCamVinModule::HbSensorInit(int dev_id, int sensor_id, int bus, int port,
    int mipi_idx, int sedres_index, int sedres_port) {
  int ret = -1;
  int extra_mode = vin_cfg_.sensor_info.extra_mode;
  MIPI_SENSOR_INFO_S *snsinfo = NULL;

  snsinfo = &cam_params_.sensor_info;
  memset(snsinfo, 0, sizeof(MIPI_SENSOR_INFO_S));
  if (sensor_plg_en_ == true) {
    SnsPlgModule *plg_handle =
      reinterpret_cast<SnsPlgModule *>(sensor_plg_cfg_.sensor_ops);
    ret = plg_handle->get_sns_attr(&sensor_plg_cfg_, snsinfo);
    if (ret) {
      LOGE << "sensor plugin get sensor attr falied, ret: " << ret;
      return ret;
    }
  } else {
    HbMipiGetSnsAttrBySns(static_cast<MipiSensorType>(sensor_id),
        snsinfo);
  }

  if (sensor_id == kOV10635_30FPS_720p_960_YUV_LINE_CONCATENATED) {
    HB_MIPI_SetExtraMode(snsinfo, extra_mode);
  }
  HB_MIPI_SetBus(snsinfo, bus);
  HB_MIPI_SetPort(snsinfo, port);
  HB_MIPI_SensorBindSerdes(snsinfo, sedres_index, sedres_port);
  HB_MIPI_SensorBindMipi(snsinfo, mipi_idx);
  snsinfo->sensorInfo.gpio_num = vin_cfg_.sensor_info.gpio_num;
  snsinfo->sensorInfo.gpio_pin[0] = vin_cfg_.sensor_info.gpio_pin;
  snsinfo->sensorInfo.gpio_level[0] = vin_cfg_.sensor_info.gpio_level;

  HbPrintSensorInfo(snsinfo);
  ret = HB_MIPI_InitSensor(dev_id, snsinfo);
  if (ret < 0) {
    LOGE << "hb mipi init sensor error, ret: " << ret
      << " dev_id: " << dev_id;
    return ret;
  }
  LOGD << "hb sensor init success..."
    << " group_id: " << group_id_
    << " dev_id: " << dev_id
    << " sensor_id: " << sensor_id
    << " bus_bum: " << bus
    << " sensor_port: " << port;

  return 0;
}

int MipiCamVinModule::HbSensorDeInit(int dev_id) {
  int ret = -1;

  ret = HB_MIPI_DeinitSensor(dev_id);
  if (ret < 0) {
    LOGE << "hb mipi deinit sensor error, ret: " << ret
      << " dev_id: " << dev_id;
    return ret;
  }

  return 0;
}

int MipiCamVinModule::HbSensorStart(int dev_id) {
  int ret = -1;

  ret = HB_MIPI_ResetSensor(dev_id);
  if (ret < 0) {
    LOGE << "hb mipi reset sensor, ret: " << ret
      << " dev_id: " << dev_id;
    return ret;
  }

  return 0;
}

int MipiCamVinModule::HbSensorStop(int dev_id) {
  int ret = -1;

  ret = HB_MIPI_UnresetSensor(dev_id);
  if (ret < 0) {
    LOGE << "hb mipi unreset sensor, ret: " << ret
      << " dev_id: " << dev_id;
    return ret;
  }

  return 0;
}

int MipiCamVinModule::HbEnableSensorClk(int mipi_idx) {
  int ret = -1;

  ret = HB_MIPI_EnableSensorClock(mipi_idx);
  if (ret < 0) {
    LOGE << "hb mipi enable sensor error, ret: " << ret
      << " mipi_idx: " << mipi_idx;
    return ret;
  }

  return 0;
}

int MipiCamVinModule::HbDisableSensorClk(int mipi_idx) {
  int ret = -1;

  ret = HB_MIPI_DisableSensorClock(mipi_idx);
  if (ret < 0) {
    LOGE << "hb mipi disable sensor error, ret: " << ret
      << " mipi_idx: " << mipi_idx;
    return ret;
  }

  return 0;
}

int MipiCamVinModule::HbMipiInit(int sensor_id, int mipi_idx) {
  int ret = -1;
  MIPI_ATTR_S *mipi_attr = NULL;

  mipi_attr = &cam_params_.mipi_info;
  memset(mipi_attr, 0, sizeof(MIPI_ATTR_S));

  if (sensor_plg_en_ == true) {
    SnsPlgModule *plg_handle =
      reinterpret_cast<SnsPlgModule *>(sensor_plg_cfg_.sensor_ops);
    ret = plg_handle->get_mipi_attr(&sensor_plg_cfg_, mipi_attr);
    if (ret) {
      LOGE << "sensor plugin get mipi attr falied, ret: " << ret;
      return ret;
    }
  } else {
    HbMipiGetMipiAttrBySns(static_cast<MipiSensorType>(sensor_id),
        mipi_attr);
  }

  ret = HB_MIPI_SetMipiAttr(mipi_idx, mipi_attr);
  if (ret < 0) {
    LOGE << "hb mipi set mipi attr error, ret: " << ret
      << " sensor_id:" << sensor_id << " mipi_idx:"  << mipi_idx;
    return ret;
  }
  LOGD << "hb mipi init success..."
    << " group_id: " << group_id_
    << " sensor_id: " << sensor_id
    << " mipi_idx: " << mipi_idx;

  return 0;
}

int MipiCamVinModule::HbMipiDeInit(int mipi_idx) {
  int ret = -1;

  ret = HB_MIPI_Clear(mipi_idx);
  if (ret < 0) {
    LOGE << "hb mipi clear error, ret: " << ret
      << " mipi_idx: " << mipi_idx;
    return ret;
  }
  LOGD << "hb mipi deinit success...";

  return 0;
}

int MipiCamVinModule::HbMipiStart(int mipi_idx) {
  int ret = -1;

  ret = HB_MIPI_ResetMipi(mipi_idx);
  if (ret < 0) {
    LOGE << "hb mipi reset mipi error, ret: " << ret
      << " mipi_idx: " << mipi_idx;
    return ret;
  }

  return 0;
}

int MipiCamVinModule::HbMipiStop(int mipi_idx) {
  int ret = -1;
  ret = HB_MIPI_UnresetMipi(mipi_idx);
  if (ret < 0) {
    LOGE << "HB_MIPI_UnresetMipi error, ret: " << ret
      << " mipi_idx: " << mipi_idx;
    return ret;
  }

  return 0;
}

int MipiCamVinModule::VinStart() {
  int ret = -1;
  int group_id = group_id_;
  LOGD << "Enter vinmodule start, group_id:" << group_id;

  ret = HB_VIN_EnableChn(group_id, 0);  // dwe start
  if (ret < 0) {
    LOGE << "HB_VIN_EnableChn error, ret: " << ret
      << " group_id: " << group_id;
    return ret;
  }

  ret = HB_VIN_StartPipe(group_id);  // isp start
  if (ret < 0) {
    LOGE << "HB_VIN_StartPipe error, ret: " << ret
      << " group_id: " << group_id;
    return ret;
  }
  ret = HB_VIN_EnableDev(group_id);  // sif start && start thread
  if (ret < 0) {
    LOGE << "HB_VIN_EnableDev error, ret: " << ret
      << " group_id: " << group_id;
    return ret;
  }

  return 0;
}

int MipiCamVinModule::VinStop() {
  int group_id = group_id_;
  LOGD << "Enter MipiCamVinModule VinStop";

  HB_VIN_DisableDev(group_id);     // thread stop && sif stop
  HB_VIN_StopPipe(group_id);       // isp stop
  HB_VIN_DisableChn(group_id, 1);  // dwe stop

  return 0;
}

int MipiCamVinModule::VinDeInit() {
  LOGD << "enter MipiCamVinModule VinDeInit...";
  int group_id = group_id_;

  HB_VIN_DestroyDev(group_id);     // sif deinit && destroy
  HB_VIN_DestroyChn(group_id, 1);  // dwe deinit
  HB_VIN_DestroyPipe(group_id);    // isp deinit && destroy

  if (sensor_plg_en_ == true) {
    HbSensorPlgDeInit();
    sensor_plg_en_ = false;
  }
  return 0;
}

void MipiCamVinModule::HbDisCropSet(uint32_t group_id, uint32_t event,
    VIN_DIS_MV_INFO_S *data, void *userdata) {
  LOGD << "data gmvX: "    << data->gmvX;
  LOGD << "data gmvY: "    << data->gmvY;
  LOGD << "data xUpdate: " << data->xUpdate;
  LOGD << "data yUpdate: " << data->yUpdate;

  return;
}

int MipiCamVinModule::VinInit() {
  int ret = -1;
  int vin_vps_mode = vin_cfg_.vin_vps_mode;
  int sensor_id = vin_cfg_.sensor_info.sensor_id;
  int mipi_idx = vin_cfg_.mipi_info.mipi_index;
  int vc_idx = vin_cfg_.mipi_info.vc_index;
  int dol2_vc_idx = vin_cfg_.mipi_info.dol2_vc_index;
  int group_id = group_id_;
  int sensor_plugin_en = vin_cfg_.sensor_info.sensor_plugin_en;

  LOGI << "Enter VinInit: "
    << " group_id: " << group_id
    << " sensor_plugin_en: " << sensor_plugin_en;
  if (sensor_plugin_en == 1) {
    ret = HbSensorPlgInit();
    if (ret) {
      LOGE << "MipiCamVinModule Sensor plugin init failed!";
      return ret;
    }
    sensor_plg_en_ = true;
  }

  VIN_DEV_ATTR_S *devinfo = NULL;
  VIN_PIPE_ATTR_S *pipeinfo = NULL;
  VIN_DIS_ATTR_S *disinfo = NULL;
  VIN_LDC_ATTR_S *ldcinfo = NULL;
  VIN_DEV_ATTR_EX_S *devexinfo = NULL;
  VIN_DIS_CALLBACK_S pstDISCallback;
  pstDISCallback.VIN_DIS_DATA_CB = HbDisCropSet;

  LOGD << "Enter Vin init, group_id: " << group_id;
  devinfo = &vin_params_.dev_info;
  devexinfo = &vin_params_.devex_info;
  pipeinfo = &vin_params_.pipe_info;
  disinfo = &vin_params_.dis_info;
  ldcinfo = &vin_params_.ldc_info;

  memset(devinfo, 0, sizeof(VIN_DEV_ATTR_S));
  memset(devexinfo, 0, sizeof(VIN_DEV_ATTR_EX_S));
  memset(pipeinfo, 0, sizeof(VIN_PIPE_ATTR_S));
  memset(disinfo, 0, sizeof(VIN_DIS_ATTR_S));
  memset(ldcinfo, 0, sizeof(VIN_LDC_ATTR_S));

  /* get default vin params */
  if (sensor_plg_en_ == true) {
    SnsPlgModule *plg_handle =
      reinterpret_cast<SnsPlgModule *>(sensor_plg_cfg_.sensor_ops);
    ret = plg_handle->get_vin_dev_attr(&sensor_plg_cfg_, devinfo);
    if (ret) {
      LOGE << "sensor plugin get vin dev attr falied, ret: " << ret;
      return ret;
    }
    ret = plg_handle->get_vin_dev_attr_ex(&sensor_plg_cfg_, devexinfo);
    if (ret) {
      LOGE << "sensor plugin get vin dev ex attr falied, ret: " << ret;
      return ret;
    }
    ret = plg_handle->get_vin_pipe_attr(&sensor_plg_cfg_, pipeinfo);
    if (ret) {
      LOGE << "sensor plugin get vin pipe attr falied, ret: " << ret;
      return ret;
    }
    ret = plg_handle->get_dis_attr(&sensor_plg_cfg_, disinfo);
    if (ret) {
      LOGE << "sensor plugin get vin dis attr falied, ret: " << ret;
      return ret;
    }
    ret = plg_handle->get_ldc_attr(&sensor_plg_cfg_, ldcinfo);
    if (ret) {
      LOGE << "sensor plugin get vin ldc attr falied, ret: " << ret;
      return ret;
    }
  } else {
    HbVinGetDevAttrBySns(static_cast<MipiSensorType>(sensor_id), devinfo);
    HbVinGetDevAttrExBySns(static_cast<MipiSensorType>(sensor_id),
        devexinfo);
    HbVinGetPipeAttrBySns(static_cast<MipiSensorType>(sensor_id), pipeinfo);
    HbVinGetDisAttrBySns(static_cast<MipiSensorType>(sensor_id), disinfo);
    HbVinGetLdcAttrBySns(static_cast<MipiSensorType>(sensor_id), ldcinfo);
  }
  /* set vin params */
  HbVinSetConfig();
  /* print sensor params */
  HbPrintSensorDevInfo(devinfo);

  // update max width and height
  VinBufferConfig vin_buf_cfg = { 0 };
  this->GetVinBuf(vin_buf_cfg);
  vin_buf_cfg.raw_pixel_len = devinfo->stSize.pix_length;
  // format is equal to 0, raw8~raw16
  // pix_length, 0:8 、1:10 、2:12、3:14 、4:16 5:20
  if (devinfo->stSize.format == 0) {
    vin_buf_cfg.format =
      HorizonVisionPixelFormat::kHorizonVisionPixelFormatRaw;
  } else {
    vin_buf_cfg.format =
      HorizonVisionPixelFormat::kHorizonVisionPixelFormatNV12;
  }
  this->SetVinBuf(vin_buf_cfg, false);

  int devinfo_image_width = devinfo->stSize.width;
  int devinfo_image_height = devinfo->stSize.height;
  int pipeinfo_image_width = pipeinfo->stSize.width;
  int pipeinfo_image_height = pipeinfo->stSize.height;

  if (vin_out_image_width_ != pipeinfo_image_width
      || vin_out_image_height_ != pipeinfo_image_height) {
    if (pipeinfo_image_width && pipeinfo_image_height) {
      // use isp ouput size priority
      vin_out_image_width_ = pipeinfo_image_width;
      vin_out_image_height_ = pipeinfo_image_height;
    } else {
      // use sif output size
      vin_out_image_width_ = devinfo_image_width;
      vin_out_image_height_ = devinfo_image_height;
    }
  }

  HbPrintSensorPipeInfo(pipeinfo);
  ret = HB_SYS_SetVINVPSMode(group_id,
      static_cast<SYS_VIN_VPS_MODE_E>(vin_vps_mode));
  if (ret < 0) {
    LOGE << "HB_SYS_SetVINVPSMode error, ret: " << ret;
    return ret;
  }
  ret = HB_VIN_CreatePipe(group_id, pipeinfo);  // isp init
  if (ret < 0) {
    LOGE << "HB_VIN_CreatePipe error, ret: " << ret;
    return ret;
  }
  ret = HB_VIN_SetMipiBindDev(group_id, mipi_idx);
  if (ret < 0) {
    LOGE << "HB_VIN_SetMipiBindDev error, ret: " << ret;
    return ret;
  }
  ret = HB_VIN_SetDevVCNumber(group_id, vc_idx);
  if (ret < 0) {
    LOGE << "HB_VIN_SetDevVCNumber error, ret: " << ret;
    return ret;
  }
  if (sensor_id == kIMX327_30FPS_2228P_RAW12_DOL2 ||
      sensor_id == kOS8A10_30FPS_2160P_RAW10_DOL2) {
    ret = HB_VIN_AddDevVCNumber(group_id, dol2_vc_idx);
    if (ret < 0) {
      LOGE << "HB_VIN_AddDevVCNumber error, ret: " << ret;
      return ret;
    }
  }
  ret = HB_VIN_SetDevAttr(group_id, devinfo);  // sif init
  if (ret < 0) {
      LOGE << "HB_VIN_SetDevAttr error, ret: " << ret;
    return ret;
  }
  if (vin_cfg_.sif_info.need_md) {
    ret = HB_VIN_SetDevAttrEx(group_id, devexinfo);
    if (ret < 0) {
      LOGE << "HB_VIN_SetDevAttrEx error, ret: " << ret;
      return ret;
    }
  }
  ret = HB_VIN_SetPipeAttr(group_id, pipeinfo);  // isp init
  if (ret < 0) {
      LOGE << "HB_VIN_SetPipeAttr error, ret: " << ret;
    goto pipe_err;
  }
  ret = HB_VIN_SetChnDISAttr(group_id, 1, disinfo);  //  dis init
  if (ret < 0) {
      LOGE << "HB_VIN_SetChnDISAttr error, ret: " << ret;
    goto pipe_err;
  }
  if (vin_cfg_.dwe_info.dis_en) {
    HB_VIN_RegisterDisCallback(group_id, &pstDISCallback);
  }
  ret = HB_VIN_SetChnLDCAttr(group_id, 1, ldcinfo);  //  ldc init
  if (ret < 0) {
      LOGE << "HB_VIN_SetChnLDCAttr error, ret: " << ret;
    goto pipe_err;
  }
  ret = HB_VIN_SetChnAttr(group_id, 1);  //  dwe init
  if (ret < 0) {
      LOGE << "HB_VIN_SetChnAttr error, ret: " << ret;
    goto pipe_err;
  }
  ret = HB_VIN_SetDevBindPipe(group_id, group_id);  //  bind init
  if (ret < 0) {
      LOGE << "HB_VIN_SetDevBindPipe error, ret: " << ret;
    goto chn_err;
  }

  vin_init_flag_ = true;
  return 0;

chn_err:
  HB_VIN_DestroyPipe(group_id);  // isp && dwe deinit
pipe_err:
  HB_VIN_DestroyDev(group_id);   // sif deinit

  return ret;
}

int MipiCamVinModule::HbMipiGetSnsAttrBySns(MipiSensorType sensor_type,
    MIPI_SENSOR_INFO_S *pst_sns_attr) {

  switch (sensor_type) {
    case kIMX327_30FPS_1952P_RAW12_LINEAR:
      memcpy(pst_sns_attr,
          &SENSOR_4LANE_IMX327_30FPS_12BIT_LINEAR_INFO,
          sizeof(MIPI_SENSOR_INFO_S));
      break;
    case kIMX327_30FPS_2228P_RAW12_DOL2:
      memcpy(pst_sns_attr,
          &SENSOR_4LANE_IMX327_30FPS_12BIT_DOL2_INFO,
          sizeof(MIPI_SENSOR_INFO_S));
      break;
    case kAR0233_30FPS_1080P_RAW12_954_PWL:
      memcpy(pst_sns_attr,
          &SENSOR_4LANE_AR0233_30FPS_12BIT_1080P_954_INFO,
          sizeof(MIPI_SENSOR_INFO_S));
      break;
    case kAR0233_30FPS_1080P_RAW12_960_PWL:
      memcpy(pst_sns_attr,
          &SENSOR_4LANE_AR0233_30FPS_12BIT_1080P_960_INFO,
          sizeof(MIPI_SENSOR_INFO_S));
      break;
    case kOS8A10_30FPS_2160P_RAW10_LINEAR:
      memcpy(pst_sns_attr,
          &SENSOR_OS8A10_30FPS_10BIT_LINEAR_INFO,
          sizeof(MIPI_SENSOR_INFO_S));
      break;
    case kOS8A10_30FPS_2160P_RAW10_DOL2:
      memcpy(pst_sns_attr,
          &SENSOR_OS8A10_30FPS_10BIT_DOL2_INFO,
          sizeof(MIPI_SENSOR_INFO_S));
      break;
    case kOV10635_30FPS_720p_954_YUV:
      memcpy(pst_sns_attr,
          &SENSOR_2LANE_OV10635_30FPS_YUV_720P_954_INFO,
          sizeof(MIPI_SENSOR_INFO_S));
      break;
    case kOV10635_30FPS_720p_960_YUV:
    case kOV10635_30FPS_720p_960_YUV_LINE_CONCATENATED:
      memcpy(pst_sns_attr,
          &SENSOR_2LANE_OV10635_30FPS_YUV_720P_960_INFO,
          sizeof(MIPI_SENSOR_INFO_S));
      break;
    case kSIF_TEST_PATTERN_1080P:
    case kSIF_TEST_PATTERN_4K:
    case kSIF_TEST_PATTERN_YUV_720P:
    case kSIF_TEST_PATTERN_12M_RAW12:
      memcpy(pst_sns_attr,
          &SENSOR_TESTPATTERN_INFO,
          sizeof(MIPI_SENSOR_INFO_S));
      break;
    case kS5KGM1SP_30FPS_4000x3000_RAW10:
      memcpy(pst_sns_attr,
          &SENSOR_S5KGM1SP_30FPS_10BIT_LINEAR_INFO,
          sizeof(MIPI_SENSOR_INFO_S));
      break;
    case kF37_30FPS_1080P_RAW10_LINEAR:
      memcpy(pst_sns_attr,
          &SENSOR_F37_30FPS_10BIT_LINEAR_INFO,
          sizeof(MIPI_SENSOR_INFO_S));
      break;
    case kSC8238_30FPS_2160P_RAW10_LINEAR:
      memcpy(pst_sns_attr,
             &SENSOR_SC8238_30FPS_10BIT_LINEAR_INFO,
             sizeof(MIPI_SENSOR_INFO_S));
      break;
    case kSC8238_30FPS_2160P_RAW10_DOL2:
      memcpy(pst_sns_attr,
             &SENSOR_SC8238_30FPS_10BIT_DOL2_INFO,
             sizeof(MIPI_SENSOR_INFO_S));
      break;
    case kIMX586_30FPS_2160P_RAW10_LINEAR:
      memcpy(pst_sns_attr,
          &SENSOR_IMX586_30FPS_10BIT_LINEAR_INFO,
          sizeof(MIPI_SENSOR_INFO_S));
      break;
    case kIMX586_30FPS_2160P_RAW10_DOL2:
      memcpy(pst_sns_attr,
          &SENSOR_IMX586_30FPS_10BIT_DOL2_INFO,
          sizeof(MIPI_SENSOR_INFO_S));
      break;
    default:
      LOGE << "not support sensor type sensor_type" << sensor_type;
      break;
  }
  LOGD << "Get sensor attr success...";
  return 0;
}

int MipiCamVinModule::HbMipiGetMipiAttrBySns(MipiSensorType sensor_type,
    MIPI_ATTR_S *pst_mipi_attr) {
  int need_clk;

  need_clk = vin_cfg_.sensor_info.need_clk;
  switch (sensor_type) {
    case kIMX327_30FPS_1952P_RAW12_LINEAR:
      if (need_clk == 1) {
        memcpy(pst_mipi_attr,
            &MIPI_4LANE_SENSOR_IMX327_30FPS_12BIT_NORMAL_SENSOR_CLK_ATTR,
            sizeof(MIPI_ATTR_S));
      } else {
        memcpy(pst_mipi_attr,
            &MIPI_4LANE_SENSOR_IMX327_30FPS_12BIT_NORMAL_ATTR,
            sizeof(MIPI_ATTR_S));
      }
      break;
    case kIMX327_30FPS_2228P_RAW12_DOL2:
      memcpy(pst_mipi_attr,
          &MIPI_4LANE_SENSOR_IMX327_30FPS_12BIT_DOL2_ATTR,
          sizeof(MIPI_ATTR_S));
      break;
    case kAR0233_30FPS_1080P_RAW12_954_PWL:
      memcpy(pst_mipi_attr,
          &MIPI_4LANE_SENSOR_AR0233_30FPS_12BIT_1080P_954_ATTR,
          sizeof(MIPI_ATTR_S));
      break;
    case kAR0233_30FPS_1080P_RAW12_960_PWL:
      memcpy(pst_mipi_attr,
          &MIPI_4LANE_SENSOR_AR0233_30FPS_12BIT_1080P_960_ATTR,
          sizeof(MIPI_ATTR_S));
      break;
    case kOS8A10_30FPS_2160P_RAW10_LINEAR:
      if (need_clk == 1) {
        memcpy(pst_mipi_attr,
            &MIPI_SENSOR_OS8A10_30FPS_10BIT_LINEAR_SENSOR_CLK_ATTR,
            sizeof(MIPI_ATTR_S));
      } else {
        memcpy(pst_mipi_attr,
            &MIPI_SENSOR_OS8A10_30FPS_10BIT_LINEAR_ATTR, sizeof(MIPI_ATTR_S));
      }
      break;
    case kOS8A10_30FPS_2160P_RAW10_DOL2:
      memcpy(pst_mipi_attr,
          &MIPI_SENSOR_OS8A10_30FPS_10BIT_DOL2_ATTR,
          sizeof(MIPI_ATTR_S));
      break;
    case kOV10635_30FPS_720p_954_YUV:
      memcpy(pst_mipi_attr,
          &MIPI_2LANE_OV10635_30FPS_YUV_720P_954_ATTR,
          sizeof(MIPI_ATTR_S));
      break;
    case kOV10635_30FPS_720p_960_YUV:
      memcpy(pst_mipi_attr,
          &MIPI_2LANE_OV10635_30FPS_YUV_720P_960_ATTR,
          sizeof(MIPI_ATTR_S));
      break;
    case kOV10635_30FPS_720p_960_YUV_LINE_CONCATENATED:
      memcpy(pst_mipi_attr,
          &MIPI_2LANE_OV10635_30FPS_YUV_LINE_CONCATE_720P_960_ATTR,
          sizeof(MIPI_ATTR_S));
      break;
    case kS5KGM1SP_30FPS_4000x3000_RAW10:
      memcpy(pst_mipi_attr,
          &MIPI_SENSOR_S5KGM1SP_30FPS_10BIT_LINEAR_ATTR,
          sizeof(MIPI_ATTR_S));
      break;
    case kF37_30FPS_1080P_RAW10_LINEAR:
      if (need_clk == 1) {
        memcpy(pst_mipi_attr,
            &MIPI_SENSOR_F37_30FPS_10BIT_LINEAR_SENSOR_CLK_ATTR,
            sizeof(MIPI_ATTR_S));
      } else {
        memcpy(pst_mipi_attr,
            &MIPI_SENSOR_F37_30FPS_10BIT_LINEAR_ATTR, sizeof(MIPI_ATTR_S));
      }
      break;
    case kSC8238_30FPS_2160P_RAW10_LINEAR:
      if (need_clk == 1) {
        memcpy(pst_mipi_attr,
               &MIPI_SENSOR_SC8238_30FPS_10BIT_LINEAR_SENSOR_CLK_ATTR,
               sizeof(MIPI_ATTR_S));
      } else {
        memcpy(pst_mipi_attr,
               &MIPI_SENSOR_SC8238_30FPS_10BIT_LINEAR_ATTR,
               sizeof(MIPI_ATTR_S));
      }
      break;
    case kSC8238_30FPS_2160P_RAW10_DOL2:
      memcpy(pst_mipi_attr,
             &MIPI_SENSOR_SC8238_30FPS_10BIT_DOL2_ATTR,
             sizeof(MIPI_ATTR_S));
      break;
    case kIMX586_30FPS_2160P_RAW10_LINEAR:
      if (need_clk == 1) {
        memcpy(pst_mipi_attr,
            &MIPI_SENSOR_IMX586_30FPS_10BIT_LINEAR_SENSOR_CLK_ATTR,
            sizeof(MIPI_ATTR_S));
      } else {
        memcpy(pst_mipi_attr,
            &MIPI_SENSOR_IMX586_30FPS_10BIT_LINEAR_ATTR, sizeof(MIPI_ATTR_S));
      }
      break;
    case kIMX586_30FPS_2160P_RAW10_DOL2:
      if (need_clk == 1) {
        memcpy(pst_mipi_attr,
            &MIPI_SENSOR_IMX586_30FPS_10BIT_DOL2_SENSOR_CLK_ATTR,
            sizeof(MIPI_ATTR_S));
      } else {
        memcpy(pst_mipi_attr,
            &MIPI_SENSOR_IMX586_30FPS_10BIT_DOL2_ATTR, sizeof(MIPI_ATTR_S));
      }
      break;
    default:
      LOGE << "not support sensor type";
      break;
  }
  LOGD << "get mipi host attr success...";

  return 0;
}

int MipiCamVinModule::HbVinGetDevAttrBySns(MipiSensorType sensor_type,
    VIN_DEV_ATTR_S *pstDevAttr) {

  switch (sensor_type) {
    case kIMX327_30FPS_1952P_RAW12_LINEAR:
      memcpy(pstDevAttr, &DEV_ATTR_IMX327_LINEAR_BASE,
          sizeof(VIN_DEV_ATTR_S));
      break;
    case kIMX327_30FPS_2228P_RAW12_DOL2:
      memcpy(pstDevAttr, &DEV_ATTR_IMX327_DOL2_BASE,
          sizeof(VIN_DEV_ATTR_S));
      break;
    case kAR0233_30FPS_1080P_RAW12_954_PWL:
    case kAR0233_30FPS_1080P_RAW12_960_PWL:
      memcpy(pstDevAttr, &DEV_ATTR_AR0233_1080P_BASE,
          sizeof(VIN_DEV_ATTR_S));
      break;
    case kOS8A10_30FPS_2160P_RAW10_LINEAR:
      memcpy(pstDevAttr, &DEV_ATTR_OS8A10_LINEAR_BASE,
          sizeof(VIN_DEV_ATTR_S));
      break;
    case kOS8A10_30FPS_2160P_RAW10_DOL2:
      memcpy(pstDevAttr, &DEV_ATTR_OS8A10_DOL2_BASE,
          sizeof(VIN_DEV_ATTR_S));
      break;
    case kOV10635_30FPS_720p_954_YUV:
    case kOV10635_30FPS_720p_960_YUV:
      memcpy(pstDevAttr, &DEV_ATTR_OV10635_YUV_BASE,
          sizeof(VIN_DEV_ATTR_S));
      break;
    case kOV10635_30FPS_720p_960_YUV_LINE_CONCATENATED:
      memcpy(pstDevAttr, &DEV_ATTR_OV10635_YUV_LINE_CONCATE_BASE,
          sizeof(VIN_DEV_ATTR_S));
      break;
    case kSIF_TEST_PATTERN_1080P:
      memcpy(pstDevAttr, &DEV_ATTR_TEST_PATTERN_1080P_BASE,
          sizeof(VIN_DEV_ATTR_S));
      break;
    case kSIF_TEST_PATTERN_4K:
      memcpy(pstDevAttr, &DEV_ATTR_TEST_PATTERN_4K_BASE,
          sizeof(VIN_DEV_ATTR_S));
      break;
    case kFEED_BACK_RAW12_1952P:
      memcpy(pstDevAttr, &DEV_ATTR_FEED_BACK_1097P_BASE,
          sizeof(VIN_DEV_ATTR_S));
      break;
    case kSIF_TEST_PATTERN_YUV_720P:
      memcpy(pstDevAttr, &DEV_ATTR_TEST_PATTERN_YUV422_BASE,
          sizeof(VIN_DEV_ATTR_S));
      break;
    case kSIF_TEST_PATTERN_12M_RAW12:
      memcpy(pstDevAttr, &DEV_ATTR_TEST_PATTERN_12M_BASE,
          sizeof(VIN_DEV_ATTR_S));
      break;
    case kS5KGM1SP_30FPS_4000x3000_RAW10:
      memcpy(pstDevAttr, &DEV_ATTR_S5KGM1SP_LINEAR_BASE,
          sizeof(VIN_DEV_ATTR_S));
      break;
    case kF37_30FPS_1080P_RAW10_LINEAR:
      memcpy(pstDevAttr, &DEV_ATTR_F37_LINEAR_BASE,
          sizeof(VIN_DEV_ATTR_S));
      break;
    case kSC8238_30FPS_2160P_RAW10_LINEAR:
      memcpy(pstDevAttr, &DEV_ATTR_SC8238_LINEAR_BASE,
             sizeof(VIN_DEV_ATTR_S));
      break;
    case kIMX586_30FPS_2160P_RAW10_LINEAR:
      memcpy(pstDevAttr, &DEV_ATTR_IMX586_LINEAR_BASE,
             sizeof(VIN_DEV_ATTR_S));
      break;
    case kIMX586_30FPS_2160P_RAW10_DOL2:
      memcpy(pstDevAttr, &DEV_ATTR_IMX586_DOL2_BASE,
             sizeof(VIN_DEV_ATTR_S));
      break;
    default:
      LOGE << "not support sensor type";
      break;
  }
  LOGD << "get vin dev attr success...";

  return 0;
}

int MipiCamVinModule::HbVinGetDevAttrExBySns(MipiSensorType sensor_type,
    VIN_DEV_ATTR_EX_S *pstDevAttrEx) {

  switch (sensor_type) {
    case kIMX327_30FPS_1952P_RAW12_LINEAR:
      memcpy(pstDevAttrEx, &DEV_ATTR_IMX327_MD_BASE,
          sizeof(VIN_DEV_ATTR_EX_S));
      break;
    case kOV10635_30FPS_720p_960_YUV:
    case kOV10635_30FPS_720p_954_YUV:
      memcpy(pstDevAttrEx, &DEV_ATTR_OV10635_MD_BASE,
          sizeof(VIN_DEV_ATTR_EX_S));
      break;
    default:
      break;
  }
  LOGD << "Get vin dev external attr success...";
  return 0;
}

int MipiCamVinModule::HbVinGetPipeAttrBySns(MipiSensorType sensor_type,
    VIN_PIPE_ATTR_S *pstPipeAttr) {

  switch (sensor_type) {
    case kIMX327_30FPS_1952P_RAW12_LINEAR:
    case kFEED_BACK_RAW12_1952P:
      memcpy(pstPipeAttr, &PIPE_ATTR_IMX327_LINEAR_BASE,
          sizeof(VIN_PIPE_ATTR_S));
      break;
    case kIMX327_30FPS_2228P_RAW12_DOL2:
      memcpy(pstPipeAttr, &PIPE_ATTR_IMX327_DOL2_BASE,
          sizeof(VIN_PIPE_ATTR_S));
      break;
    case kAR0233_30FPS_1080P_RAW12_954_PWL:
    case kAR0233_30FPS_1080P_RAW12_960_PWL:
      memcpy(pstPipeAttr, &PIPE_ATTR_AR0233_1080P_BASE,
          sizeof(VIN_PIPE_ATTR_S));
      break;
    case kOS8A10_30FPS_2160P_RAW10_LINEAR:
      memcpy(pstPipeAttr, &PIPE_ATTR_OS8A10_LINEAR_BASE,
          sizeof(VIN_PIPE_ATTR_S));
      break;
    case kOS8A10_30FPS_2160P_RAW10_DOL2:
      memcpy(pstPipeAttr, &PIPE_ATTR_OS8A10_DOL2_BASE,
          sizeof(VIN_PIPE_ATTR_S));
      break;
    case kOV10635_30FPS_720p_954_YUV:
    case kOV10635_30FPS_720p_960_YUV:
    case kSIF_TEST_PATTERN_YUV_720P:
      memcpy(pstPipeAttr, &PIPE_ATTR_OV10635_YUV_BASE,
          sizeof(VIN_PIPE_ATTR_S));
      break;
    case kOV10635_30FPS_720p_960_YUV_LINE_CONCATENATED:
      memcpy(pstPipeAttr, &VIN_ATTR_OV10635_YUV_LINE_CONCATE_BASE ,
          sizeof(VIN_PIPE_ATTR_S));
      break;
    case kSIF_TEST_PATTERN_1080P:
      memcpy(pstPipeAttr, &PIPE_ATTR_TEST_PATTERN_1080P_BASE,
          sizeof(VIN_PIPE_ATTR_S));
      break;
    case kSIF_TEST_PATTERN_4K:
      memcpy(pstPipeAttr, &PIPE_ATTR_TEST_PATTERN_4K_BASE,
          sizeof(VIN_PIPE_ATTR_S));
      break;
    case kSIF_TEST_PATTERN_12M_RAW12:
      memcpy(pstPipeAttr, &PIPE_ATTR_TEST_PATTERN_12M_BASE,
          sizeof(VIN_PIPE_ATTR_S));
      break;
    case kS5KGM1SP_30FPS_4000x3000_RAW10:
      memcpy(pstPipeAttr, &PIPE_ATTR_S5KGM1SP_LINEAR_BASE,
          sizeof(VIN_PIPE_ATTR_S));
      break;
    case kF37_30FPS_1080P_RAW10_LINEAR:
      memcpy(pstPipeAttr, &PIPE_ATTR_F37_LINEAR_BASE,
          sizeof(VIN_PIPE_ATTR_S));
      break;
    case kSC8238_30FPS_2160P_RAW10_LINEAR:
      memcpy(pstPipeAttr, &PIPE_ATTR_SC8238_LINEAR_BASE,
             sizeof(VIN_PIPE_ATTR_S));
      break;
    case kIMX586_30FPS_2160P_RAW10_LINEAR:
      memcpy(pstPipeAttr, &PIPE_ATTR_IMX586_LINEAR_BASE,
             sizeof(VIN_PIPE_ATTR_S));
      break;
    case kIMX586_30FPS_2160P_RAW10_DOL2:
      memcpy(pstPipeAttr, &PIPE_ATTR_IMX586_DOL2_BASE,
             sizeof(VIN_PIPE_ATTR_S));
      break;
    default:
      LOGE << "not support sensor type";
      break;
  }
  LOGD << "Get vin pipe attr success...";

  return 0;
}

int MipiCamVinModule::HbVinGetDisAttrBySns(MipiSensorType sensor_type,
    VIN_DIS_ATTR_S *pstDisAttr) {

  switch (sensor_type) {
    case kIMX327_30FPS_1952P_RAW12_LINEAR:
    case kIMX327_30FPS_2228P_RAW12_DOL2:
    case kAR0233_30FPS_1080P_RAW12_954_PWL:
    case kAR0233_30FPS_1080P_RAW12_960_PWL:
    case kSIF_TEST_PATTERN_1080P:
    case kSIF_TEST_PATTERN_4K:
    case kFEED_BACK_RAW12_1952P:
      memcpy(pstDisAttr, &DIS_ATTR_BASE, sizeof(VIN_DIS_ATTR_S));
      break;
    case kOS8A10_30FPS_2160P_RAW10_LINEAR:
    case kOS8A10_30FPS_2160P_RAW10_DOL2:
      memcpy(pstDisAttr, &DIS_ATTR_OS8A10_BASE, sizeof(VIN_DIS_ATTR_S));
      break;
    case kOV10635_30FPS_720p_954_YUV:
    case kOV10635_30FPS_720p_960_YUV:
    case kOV10635_30FPS_720p_960_YUV_LINE_CONCATENATED:
    case kSIF_TEST_PATTERN_YUV_720P:
      memcpy(pstDisAttr, &DIS_ATTR_OV10635_BASE, sizeof(VIN_DIS_ATTR_S));
      break;
    case kSIF_TEST_PATTERN_12M_RAW12:
    case kS5KGM1SP_30FPS_4000x3000_RAW10:
      memcpy(pstDisAttr, &DIS_ATTR_12M_BASE, sizeof(VIN_DIS_ATTR_S));
      break;
    case kSC8238_30FPS_2160P_RAW10_LINEAR:
      memcpy(pstDisAttr, &DIS_ATTR_SC8238_BASE, sizeof(VIN_DIS_ATTR_S));
      break;
    case kIMX586_30FPS_2160P_RAW10_LINEAR:
    case kIMX586_30FPS_2160P_RAW10_DOL2:
      memcpy(pstDisAttr, &DIS_ATTR_IMX586_BASE, sizeof(VIN_DIS_ATTR_S));
      break;
    case kF37_30FPS_1080P_RAW10_LINEAR:
      memcpy(pstDisAttr, &DIS_ATTR_F37_BASE, sizeof(VIN_DIS_ATTR_S));
      break;
    default:
      LOGE << "not support sensor type";
      break;
  }
  LOGD << "Get vin dis attr success...";

  return 0;
}

int MipiCamVinModule::HbVinGetLdcAttrBySns(MipiSensorType sensor_type,
    VIN_LDC_ATTR_S *pstLdcAttr) {

  switch (sensor_type) {
    case kIMX327_30FPS_1952P_RAW12_LINEAR:
    case kIMX327_30FPS_2228P_RAW12_DOL2:
    case kAR0233_30FPS_1080P_RAW12_954_PWL:
    case kAR0233_30FPS_1080P_RAW12_960_PWL:
    case kSIF_TEST_PATTERN_1080P:
    case kSIF_TEST_PATTERN_4K:
    case kFEED_BACK_RAW12_1952P:
      memcpy(pstLdcAttr, &LDC_ATTR_BASE, sizeof(VIN_LDC_ATTR_S));
      break;
    case kOS8A10_30FPS_2160P_RAW10_LINEAR:
    case kOS8A10_30FPS_2160P_RAW10_DOL2:
      memcpy(pstLdcAttr, &LDC_ATTR_OS8A10_BASE, sizeof(VIN_LDC_ATTR_S));
      break;
    case kOV10635_30FPS_720p_954_YUV:
    case kOV10635_30FPS_720p_960_YUV:
    case kOV10635_30FPS_720p_960_YUV_LINE_CONCATENATED:
    case kSIF_TEST_PATTERN_YUV_720P:
      memcpy(pstLdcAttr, &LDC_ATTR_OV10635_BASE, sizeof(VIN_LDC_ATTR_S));
      break;
    case kSIF_TEST_PATTERN_12M_RAW12:
    case kS5KGM1SP_30FPS_4000x3000_RAW10:
      memcpy(pstLdcAttr, &LDC_ATTR_12M_BASE, sizeof(VIN_LDC_ATTR_S));
      break;
    case kSC8238_30FPS_2160P_RAW10_LINEAR:
      memcpy(pstLdcAttr, &LDC_ATTR_SC8238_BASE, sizeof(VIN_LDC_ATTR_S));
      break;
    case kIMX586_30FPS_2160P_RAW10_LINEAR:
    case kIMX586_30FPS_2160P_RAW10_DOL2:
      memcpy(pstLdcAttr, &LDC_ATTR_IMX586_BASE, sizeof(VIN_LDC_ATTR_S));
      break;
    case kF37_30FPS_1080P_RAW10_LINEAR:
      memcpy(pstLdcAttr, &LDC_ATTR_F37_BASE, sizeof(VIN_LDC_ATTR_S));
      break;
    default:
      LOGE << "not support sensor type";
      break;
  }
  LOGD << "Get vin ldc attr success...";

  return 0;
}

int MipiCamVinModule::HbTimeCostMs(struct timeval *start, struct timeval *end) {
  int time_ms = -1;
  time_ms = ((end->tv_sec * 1000 + end->tv_usec / 1000) -
      (start->tv_sec * 1000 + start->tv_usec / 1000));
  LOGD << "time cost " << time_ms << "ms";
  return time_ms;
}

void MipiCamVinModule::HbPrintSensorDevInfo(VIN_DEV_ATTR_S *devinfo) {
  LOGI << "sensor format: " << devinfo->stSize.format;
  LOGI << "sensor width: " << devinfo->stSize.width;
  LOGI << "sensor height: " << devinfo->stSize.height;
  LOGI << "sensor pix_length: " << devinfo->stSize.pix_length;
  LOGI << "sensor mipi attr enable_frame_id: "
    << devinfo->mipiAttr.enable_frame_id;
  LOGI << "sensor mipi attr enable_mux_out: "
    << devinfo->mipiAttr.enable_mux_out;
  LOGI << "sensor mipi attr set_init_frame_id: "
    << devinfo->mipiAttr.set_init_frame_id;
  LOGI << "sensor mipi attr ipu_channels: "
    << devinfo->mipiAttr.ipi_channels;
  LOGI << "sensor mipi attr enable_line_shift: "
    << devinfo->mipiAttr.enable_line_shift;
  LOGI << "sensor mipi attr enable_id_decoder: "
    << devinfo->mipiAttr.enable_id_decoder;
  LOGI << "sensor mipi attr set_bypass_channels: "
    << devinfo->mipiAttr.set_bypass_channels;
  LOGI << "sensor mipi attr enable_bypass: "
    << devinfo->mipiAttr.enable_bypass;
  LOGI << "sensor mipi attr set_line_shift_count: "
    << devinfo->mipiAttr.set_line_shift_count;
  LOGI << "sensor mipi attr enable_pattern: "
    << devinfo->mipiAttr.enable_pattern;

  LOGI << "sensor out ddr attr stride: "
    << devinfo->outDdrAttr.stride;
  LOGI << "sensor out ddr attr buffer_num: "
    << devinfo->outDdrAttr.buffer_num;

  return;
}

void MipiCamVinModule::HbPrintSensorPipeInfo(VIN_PIPE_ATTR_S *pipeinfo) {
  LOGI << "isp_out ddr_out_buf_num: " << pipeinfo->ddrOutBufNum;
  LOGI << "isp_out width: " << pipeinfo->stSize.width;
  LOGI << "isp_out height: " << pipeinfo->stSize.height;
  LOGI << "isp_out sensor_mode: " << pipeinfo->snsMode;
  LOGI << "isp_out format: " << pipeinfo->stSize.format;

  return;
}

void MipiCamVinModule::HbPrintSensorInfo(MIPI_SENSOR_INFO_S *snsinfo) {
  LOGI << "bus_bum: " << snsinfo->sensorInfo.bus_num;
  LOGI << "bus_type: " << snsinfo->sensorInfo.bus_type;
  LOGI << "reg_width: " << snsinfo->sensorInfo.reg_width;
  LOGI << "sensor_name: " << snsinfo->sensorInfo.sensor_name;
  LOGI << "sensor_mode: " << snsinfo->sensorInfo.sensor_mode;
  LOGI << "sensor_addr: " << snsinfo->sensorInfo.sensor_addr;
  LOGI << "serial_addr: " << snsinfo->sensorInfo.serial_addr;
  LOGI << "resolution: " << snsinfo->sensorInfo.resolution;
  LOGI << "gpio_num: " << snsinfo->sensorInfo.gpio_num;
  LOGI << "gpio_pin: " << snsinfo->sensorInfo.gpio_pin[0];
  LOGI << "gpio_level: " << snsinfo->sensorInfo.gpio_level[0];

  return;
}

void MipiCamVinModule::HbVinSetConfig() {
  int temper_mode, isp_out_buf_num, isp_3a_en;
  int ldc_en, dis_en, mirror_en;
  int cfa_pattern;

  /* 1. get vin config */
  temper_mode = vin_cfg_.isp_info.temper_mode;
  isp_out_buf_num = vin_cfg_.isp_info.isp_out_buf_num;
  isp_3a_en = vin_cfg_.isp_info.isp_3a_en;
  cfa_pattern = vin_cfg_.isp_info.cfa_pattern;
  ldc_en = vin_cfg_.dwe_info.ldc_en;
  dis_en = vin_cfg_.dwe_info.dis_en;
  mirror_en = vin_cfg_.sensor_info.mirror_en;
  /* 2. set vin config to vin_params */
  vin_params_.pipe_info.temperMode = temper_mode;
  vin_params_.pipe_info.ddrOutBufNum = isp_out_buf_num;
  vin_params_.pipe_info.ispAlgoState = isp_3a_en;
  if (mirror_en) {
    vin_params_.pipe_info.cfaPattern =
      static_cast<VIN_PIPE_CFA_PATTERN_E>(cfa_pattern);
  }
  vin_params_.ldc_info.ldcEnable = ldc_en;
  vin_params_.dis_info.disPath.rg_dis_enable = dis_en;
  LOGI << "vin set config success, "
    << " temper_mode: " << temper_mode
    << " isp_out_buf_num: " << isp_out_buf_num
    << " isp_3a_en: " << isp_3a_en
    << " cfa_pattern: " << cfa_pattern
    << " ldc_en: " << ldc_en
    << " dis_en: " << dis_en
    << " mirror_en: " << mirror_en;
  return;
}

int MipiCamVinModule::HbVinSetImageMirror() {
  MipiSensorType sensor_type =
    static_cast<MipiSensorType>(vin_cfg_.sensor_info.sensor_id);
  std::string cmd_set_mirror{"i2ctransfer -y -f " +
      std::to_string(vin_cfg_.sensor_info.i2c_bus) + " "};
  std::string sensor_set_data{""};
  switch (sensor_type) {
    case kOS8A10_30FPS_2160P_RAW10_LINEAR:
    case kOS8A10_30FPS_2160P_RAW10_DOL2:
      sensor_set_data = "w4@0x36 0x38 0x21 0x04 0x44";
      break;
    case kF37_30FPS_1080P_RAW10_LINEAR:
      sensor_set_data = "w2@0x40 0x12 0x00";
      break;
    case kIMX586_30FPS_2160P_RAW10_LINEAR:
    case kIMX586_30FPS_2160P_RAW10_DOL2:
      sensor_set_data = "w3@0x1a 0x01 0x01 0x01";
      break;
    default:
      break;
  }
  if (!sensor_set_data.empty()) {
    cmd_set_mirror += sensor_set_data;
    LOGW << "cmd_set_mirror: " << cmd_set_mirror
      << ", sensor_set_data: " << sensor_set_data;
    system(cmd_set_mirror.data());
  }

  return 0;
}

int MipiCamVinModule::SetVinBindVps(const int &group_id) {
  int ret = -1;
  struct HB_SYS_MOD_S src_mod, dst_mod;
  int vin_vps_mode = vin_cfg_.vin_vps_mode;

  src_mod.enModId = HB_ID_VIN;
  src_mod.s32DevId = group_id_;
  if (vin_vps_mode == VIN_ONLINE_VPS_ONLINE ||
      vin_vps_mode == VIN_OFFLINE_VPS_ONLINE ||
      vin_vps_mode == VIN_SIF_ONLINE_DDR_ISP_ONLINE_VPS_ONLINE ||
      vin_vps_mode == VIN_SIF_OFFLINE_ISP_OFFLINE_VPS_ONLINE ||
      vin_vps_mode == VIN_FEEDBACK_ISP_ONLINE_VPS_ONLINE ||
      vin_vps_mode == VIN_SIF_VPS_ONLINE)
    src_mod.s32ChnId = 1;
  else
    src_mod.s32ChnId = 0;
  dst_mod.enModId = HB_ID_VPS;
  dst_mod.s32DevId = group_id;
  dst_mod.s32ChnId = 0;
  ret = HB_SYS_Bind(&src_mod, &dst_mod);
  if (ret != 0) {
    LOGE << "HB_SYS_Bind failed, ret: " << ret
      << " vin_vps_mode: " << vin_vps_mode
      << " group_id: " << group_id_;
    return ret;
  }

  LOGI << "mipi camera set vin bind vps success..."
    << " vin_vps_mode: " << vin_vps_mode
    << " group_id: " << group_id_;
  return 0;
}

void MipiCamVinModule::HbPrintConfig() {
  LOGI << "Enter print mipi camera vin module config";
  LOGI << "------------sensor info---------------";
  LOGI << "bind_chn_id: " << vin_cfg_.bind_chn_id;
  LOGI << "vin_vps_mode: " << vin_cfg_.vin_vps_mode;
  LOGI << "sensor_id: " << vin_cfg_.sensor_info.sensor_id;
  LOGI << "sensor_plugin_en: " << vin_cfg_.sensor_info.sensor_plugin_en;
  LOGI << "sensor_plugin_path: " << vin_cfg_.sensor_info.sensor_plugin_path;
  LOGI << "sensor_plugin_name: " << vin_cfg_.sensor_info.sensor_plugin_name;
  LOGI << "sensor_plugin_type: " << vin_cfg_.sensor_info.sensor_plugin_type;
  LOGI << "sensor_port: " << vin_cfg_.sensor_info.sensor_port;
  LOGI << "i2c_bus: " << vin_cfg_.sensor_info.i2c_bus;
  LOGI << "need_clk: " << vin_cfg_.sensor_info.need_clk;
  LOGI << "serdes_index: " << vin_cfg_.sensor_info.serdes_index;
  LOGI << "serdes_port: " << vin_cfg_.sensor_info.serdes_port;
  LOGI << "extra_mode: " << vin_cfg_.sensor_info.extra_mode;
  LOGI << "gpio_num: " << vin_cfg_.sensor_info.gpio_num;
  LOGI << "gpio_pin: " << vin_cfg_.sensor_info.gpio_pin;
  LOGI << "gpio_level: " << vin_cfg_.sensor_info.gpio_level;

  LOGI << "------------mipi info--------------";
  LOGI << "mipi_index: " << vin_cfg_.mipi_info.mipi_index;
  LOGI << "vc_index: " << vin_cfg_.mipi_info.vc_index;
  LOGI << "dol2_vc_index: " << vin_cfg_.mipi_info.dol2_vc_index;

  LOGI << "------------sif info---------------";
  LOGI << "need_md: " << vin_cfg_.sif_info.need_md;
  LOGI << "sif_out_buf_num: " << vin_cfg_.sif_info.sif_out_buf_num;

  LOGI << "------------isp info---------------";
  LOGI << "temper_mode: " << vin_cfg_.isp_info.temper_mode;
  LOGI << "isp_out_buf_num: " << vin_cfg_.isp_info.isp_out_buf_num;
  LOGI << "isp_3a_en: " << vin_cfg_.isp_info.isp_3a_en;

  LOGI << "------------dwe info---------------";
  LOGI << "ldc_en: " << vin_cfg_.dwe_info.ldc_en;
  LOGI << "dis_en: " << vin_cfg_.dwe_info.dis_en;
}

int MipiCamVinModule::LoadConfig(const std::string &config_file) {
  int ret = -1;
  std::shared_ptr<JsonConfigWrapper> json_cfg = nullptr;

  ret = VinModule::LoadConfigFile(config_file, json_cfg);
  if (ret) {
    LOGE << "load config file failed, ret: " << ret;
    return ret;
  }

  vin_cfg_.bind_chn_id = json_cfg->GetIntValue("bind_chn_id");
  vin_cfg_.vin_vps_mode = json_cfg->GetIntValue("vin_vps_mode");

  // 1. parse sensor config info
  auto json_cfg_sensor = json_cfg->GetSubConfig("sensor");
  if (json_cfg_sensor == nullptr) {
    LOGE << "json cfg sensor parameter is not exit!";
    return -1;
  }
  vin_cfg_.sensor_info.sensor_id =
    json_cfg_sensor->GetIntValue("sensor_id");
  vin_cfg_.sensor_info.sensor_plugin_en =
    json_cfg_sensor->GetIntValue("sensor_plugin_en");
  vin_cfg_.sensor_info.sensor_plugin_path =
    json_cfg_sensor->GetSTDStringValue("sensor_plugin_path");
  vin_cfg_.sensor_info.sensor_plugin_name =
    json_cfg_sensor->GetSTDStringValue("sensor_plugin_name");

  std::string sensor_plugin_type =
    json_cfg_sensor->GetSTDStringValue("sensor_plugin_type");
  if (sensor_plugin_type == "linear") {
    vin_cfg_.sensor_info.sensor_plugin_type = kLINEAR_SENSOR_TYPE;
  } else if  (sensor_plugin_type == "dol2") {
    vin_cfg_.sensor_info.sensor_plugin_type = kDOL2_SENSOR_TYPE;
  } else if  (sensor_plugin_type == "dol3") {
    vin_cfg_.sensor_info.sensor_plugin_type = kDOL3_SENSOR_TYPE;
  } else {
    vin_cfg_.sensor_info.sensor_plugin_type = kSENSOR_TYPE_INVALID;
  }
  vin_cfg_.sensor_info.sensor_port =
    json_cfg_sensor->GetIntValue("sensor_port");
  vin_cfg_.sensor_info.i2c_bus =
    json_cfg_sensor->GetIntValue("i2c_bus");
  vin_cfg_.sensor_info.need_clk =
    json_cfg_sensor->GetIntValue("need_clk");
  vin_cfg_.sensor_info.mirror_en =
    json_cfg_sensor->GetIntValue("mirror_en");
  vin_cfg_.sensor_info.serdes_index =
    json_cfg_sensor->GetIntValue("serdes_index");
  vin_cfg_.sensor_info.serdes_port =
    json_cfg_sensor->GetIntValue("serdes_port");
  vin_cfg_.sensor_info.extra_mode =
    json_cfg_sensor->GetIntValue("extra_mode");
  vin_cfg_.sensor_info.gpio_num =
    json_cfg_sensor->GetIntValue("gpio_num");
  vin_cfg_.sensor_info.gpio_pin =
    json_cfg_sensor->GetIntValue("gpio_pin");
  vin_cfg_.sensor_info.gpio_level =
    json_cfg_sensor->GetIntValue("gpio_level");

  // 2. parse mipi config info
  auto json_cfg_mipi = json_cfg->GetSubConfig("mipi");
  if (json_cfg_mipi == nullptr) {
    LOGE << "json cfg mipi parameter is not exit!";
    return -1;
  } else {
    vin_cfg_.mipi_info.mipi_index =
      json_cfg_mipi->GetIntValue("host_index");
    vin_cfg_.mipi_info.vc_index =
      json_cfg_mipi->GetIntValue("vc_index");
    vin_cfg_.mipi_info.dol2_vc_index =
      json_cfg_mipi->GetIntValue("dol2_vc_index");
  }

  // 3. parse sif config info
  auto json_cfg_sif = json_cfg->GetSubConfig("sif");
  if (json_cfg_sif == nullptr) {
    LOGW << "json cfg sif parameter is not exit!";
  } else {
    vin_cfg_.sif_info.need_md =
      json_cfg_sif->GetIntValue("need_md");
    vin_cfg_.sif_info.sif_out_buf_num =
      json_cfg_sif->GetIntValue("sif_out_buf_num");
  }

  // 4. parse isp config info
  auto json_cfg_isp = json_cfg->GetSubConfig("isp");
  if (json_cfg_isp == nullptr) {
    LOGW << "json cfg isp parameter is not exit!";
  } else {
    vin_cfg_.isp_info.temper_mode =
      json_cfg_isp->GetIntValue("temper_mode");
    vin_cfg_.isp_info.isp_out_buf_num =
      json_cfg_isp->GetIntValue("isp_out_buf_num");
    vin_cfg_.isp_info.isp_3a_en =
      json_cfg_isp->GetIntValue("isp_3a_en");
    vin_cfg_.isp_info.cfa_pattern =
      json_cfg_isp->GetIntValue("cfa_pattern_type");
  }

  // 5. parse dwe config info
  auto json_cfg_dwe = json_cfg->GetSubConfig("dwe");
  if (json_cfg_dwe == nullptr) {
    LOGW << "json cfg dwe parameter is not exit!";
  } else {
    auto json_cfg_ldc = json_cfg_dwe->GetSubConfig("ldc");
    if (json_cfg_ldc == nullptr) {
      LOGW << "json cfg ldc parameter is not exit!";
    } else {
      vin_cfg_.dwe_info.ldc_en =
        json_cfg_ldc->GetIntValue("ldc_en");
    }
    auto json_cfg_dis = json_cfg_dwe->GetSubConfig("dis");
    if (json_cfg_dis == nullptr) {
      LOGW << "json cfg dis parameter is not exit!";
    } else {
      vin_cfg_.dwe_info.dis_en =
        json_cfg_dis->GetIntValue("dis_en");
    }
  }
  HbPrintConfig();

  return 0;
}

void MipiCamVinModule::HbGetDataThread() {
  // start get other camera channel data as input
  int ret = -1;
  int group_id = group_id_;
  VinSourceType src_type = src_type_;
  int src_group_id = src_group_id_;
  int timeout = 2000;
  int src_chn = 0;  // sif_to_ddr or isp_to_ddr, chn is 0
  hb_vio_buffer_t vio_buf;
  int vio_buf_width, vio_buf_height;

  is_running_ = true;
  LOGI << "Enter Get Data Thread, group_id: " << group_id
    << " start_flag: " << is_running_
    << " src_type: " << src_type
    << " src_group_id: " << src_group_id;
  while (is_running_) {
    memset(&vio_buf, 0, sizeof(hb_vio_buffer_t));

    // 1.get inner sif source data
    if (src_type == kSIF_BUF_SOURCE) {
      ret = HB_VIN_GetDevFrame(src_group_id, src_chn, &vio_buf, timeout);
      if (ret) {
        LOGE << "group_id: " << group_id
          << " src_group_id: " << src_group_id
          << " HB_VIN_GetDevFrame error, ret: " << ret;
        continue;
      }
    } else if (src_type == kISP_BUF_SOURCE) {
      ret = HB_VIN_GetChnFrame(src_group_id, src_chn, &vio_buf, timeout);
      if (ret) {
        LOGE << "group_id: " << group_id
          << " src_group_id: " << src_group_id
          << " HB_VIN_GetChnFrame error, ret: " << ret;
        continue;
      }
    } else {
      LOGE << "error source type!!!";
      is_running_  = false;
      continue;
    }

    vio_buf_width = vio_buf.img_addr.width;
    vio_buf_height = vio_buf.img_addr.height;
    HOBOT_CHECK(vio_buf_width > 0);
    HOBOT_CHECK(vio_buf_height > 0);

    // 2. update max width and height
    VinBufferConfig vin_buf_cfg = { 0 };
    this->GetVinBuf(vin_buf_cfg);
    if (vin_image_width_ != vio_buf_width
        || vin_image_height_ != vio_buf_height) {
      vin_image_width_ = vio_buf_width;
      vin_image_height_ = vio_buf_height;
      vin_buf_cfg.max_width = ALIGN(vin_image_width_, 16);
      vin_buf_cfg.max_height = ALIGN(vin_image_height_, 16);
      LOGD << "vin_buf max_width: " << vin_buf_cfg.max_width
        << " vin buf max_height: " << vin_buf_cfg.max_height
        << " vio buf width: " << vio_buf_width
        << " vio buf heigh: " << vio_buf_height;
      this->SetVinBuf(vin_buf_cfg, true);
    }

    // 3.send inner source data
    VinFrame frame = { 0 };
    frame.format = vin_buf_cfg.format;
    frame.frame_id = vio_buf.img_info.frame_id;
    frame.plane_count = vio_buf.img_info.planeCount;
    frame.vaddr[0] = vio_buf.img_addr.addr[0];
    frame.vaddr[1] = vio_buf.img_addr.addr[1];
    frame.vaddr[2] = vio_buf.img_addr.addr[2];
    frame.paddr[0] = vio_buf.img_addr.paddr[0];
    frame.paddr[1] = vio_buf.img_addr.paddr[1];
    frame.paddr[2] = vio_buf.img_addr.paddr[2];
    auto image_w = vio_buf.img_addr.width;
    auto image_h = vio_buf.img_addr.height;
    auto image_s = vio_buf.img_addr.stride_size;
    frame.width = static_cast<int>(image_w);
    frame.height = static_cast<int>(image_h);
    frame.stride = static_cast<int>(image_s);
    if (frame.plane_count == 1) {
      frame.plane_size[0] = vio_buf.img_info.size[0];
      frame.size = frame.plane_size[0];
    } else if (frame.plane_count == 2) {
      frame.plane_size[0] = vio_buf.img_info.size[0];
      frame.plane_size[1] = vio_buf.img_info.size[1];
      frame.size = frame.plane_size[0] + frame.plane_size[1];
    } else if (frame.plane_count == 3) {
      frame.plane_size[0] = vio_buf.img_info.size[0];
      frame.plane_size[1] = vio_buf.img_info.size[1];
      frame.plane_size[2] = vio_buf.img_info.size[2];
      frame.size = frame.plane_size[0] + frame.plane_size[1] +
        frame.plane_size[2];
    } else {
      LOGE << "Unsupport frame plane_count: " << frame.plane_count;
      continue;
    }
    LOGD << "mipi camera input data,"
      << " frame_id:" << frame.frame_id
      << " format: " << frame.format
      << " plane_count: " << frame.plane_count
      << " image_w: " << image_w
      << " image_h: " << image_h
      << " image_s: " << image_s
      << " frame_size: " << frame.size
      << " size[0]: " << vio_buf.img_info.size[0]
      << " size[1]: " << vio_buf.img_info.size[1]
      << " size[2]: " << vio_buf.img_info.size[2]
      << " vaddr[0]: " << frame.vaddr[0]
      << " vaddr[1]: " << frame.vaddr[1]
      << " vaddr[2]: " << frame.vaddr[2];

    InputData(frame);

    // 4.free inner source data
    if (src_type == kSIF_BUF_SOURCE) {
      ret = HB_VIN_ReleaseDevFrame(src_group_id, src_chn, &vio_buf);
      if (ret) {
        LOGE << "group_id: " << group_id
          << " src_group_id: " << src_group_id
          << " HB_VIN_ReleaseDevFrame error, ret: " << ret;
        continue;
      }
    } else if (src_type == kISP_BUF_SOURCE) {
      ret = HB_VIN_ReleaseChnFrame(src_group_id, src_chn, &vio_buf);
      if (ret) {
        LOGE << "group_id: " << group_id
          << " src_group_id: " << src_group_id
          << " HB_VIN_ReleaseChnFrame error, ret: " << ret;
        continue;
      }
    } else {
      LOGE << "error source type!!!";
      is_running_  = false;
      continue;
    }
  }

  LOGI << "Quit vin get data thread, group_id: " << group_id;
}

}  // namespace videosource
