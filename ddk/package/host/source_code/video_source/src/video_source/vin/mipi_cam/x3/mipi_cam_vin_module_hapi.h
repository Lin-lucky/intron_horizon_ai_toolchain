/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef VIDEO_SOURCE_VIN_MIPI_CAM_VIN_MODULE_HAPI_H_
#define VIDEO_SOURCE_VIN_MIPI_CAM_VIN_MODULE_HAPI_H_
#include <string>
#include <memory>
#include "video_source/vin/vin_module.h"
#include "video_source/vin/mipi_cam/x3/mipi_cam_data_type.h"
#include "video_source/vin/mipi_cam/x3/sensor_plugin.h"

namespace videosource {

class MipiCamVinModule : public VinModule {
 public:
  MipiCamVinModule() = delete;
  explicit MipiCamVinModule(const int &channel_id, const int &group_id)
    : VinModule("mipi_camera_vin_module", channel_id, group_id) {}
  ~MipiCamVinModule() {}

  int Init(const std::string &config_file) override;
  int DeInit() override;
  int Start() override;
  int Stop() override;
  int SetVinBindVps(const int &group_id) override;

 protected:
  int LoadConfig(const std::string &config_file);
  int CamInit();
  int CamDeInit();
  int CamStart();
  int CamStop();
  int VinInit();
  int VinDeInit();
  int VinStart();
  int VinStop();
  int VinCreateDataThread();
  int VinDestoryDataThread();

 private:
  int HbMipiGetSnsAttrBySns(MipiSensorType sensor_type,
      MIPI_SENSOR_INFO_S *pst_sns_attr);
  int HbMipiGetMipiAttrBySns(MipiSensorType sensor_type,
      MIPI_ATTR_S *pst_mipi_attr);
  int HbVinGetDevAttrBySns(MipiSensorType sensor_type,
      VIN_DEV_ATTR_S *pstDevAttr);
  int HbVinGetDevAttrExBySns(MipiSensorType sensor_type,
      VIN_DEV_ATTR_EX_S *pstDevAttrEx);
  int HbVinGetPipeAttrBySns(MipiSensorType sensor_type,
      VIN_PIPE_ATTR_S *pstPipeAttr);
  int HbVinGetDisAttrBySns(MipiSensorType sensor_type,
      VIN_DIS_ATTR_S *pstDisAttr);
  int HbVinGetLdcAttrBySns(MipiSensorType sensor_type,
      VIN_LDC_ATTR_S *pstLdcAttr);
  int HbTimeCostMs(struct timeval *start, struct timeval *end);
  void HbPrintSensorDevInfo(VIN_DEV_ATTR_S *devinfo);
  void HbPrintSensorPipeInfo(VIN_PIPE_ATTR_S *pipeinfo);
  void HbPrintSensorInfo(MIPI_SENSOR_INFO_S *snsinfo);
  void HbVinSetConfig();
  int HbSensorInit(int dev_id, int sensor_id, int bus, int port,
      int mipi_idx, int sedres_index, int sedres_port);
  int HbSensorDeInit(int group_id);
  int HbSensorStart(int group_id);
  int HbSensorStop(int group_id);
  int HbEnableSensorClk(int mipi_idx);
  int HbDisableSensorClk(int mipi_idx);
  int HbMipiInit(int sensor_id, int mipi_idx);
  int HbMipiDeInit(int mipi_idx);
  int HbMipiStart(int mipi_idx);
  int HbMipiStop(int mipi_idx);
  int HbSensorPlgInit();
  int HbSensorPlgDeInit();
  static void HbDisCropSet(uint32_t group_id, uint32_t event,
      VIN_DIS_MV_INFO_S *data, void *userdata);
  void HbPrintConfig();
  void HbGetDataThread();
  int HbCheckDataOutputMode();
  int HbVinSetImageMirror();

 private:
  VinMipiCamConfig vin_cfg_ = { 0 };
  VinParams vin_params_ = { 0 };
  CamParams cam_params_ = { 0 };
  bool vin_init_flag_ = false;
  bool cam_init_flag_ = false;
  bool sensor_plg_en_ = false;
  SnsPlgCtrlInfo sensor_plg_cfg_ = { 0 };
  bool data_thread_flag_ = false;
  bool is_running_ = false;
  std::shared_ptr<std::thread> data_thread_ = nullptr;
  VinSourceType src_type_ = kVin_SOURCE_INVALID;
  int src_group_id_ = -1;
  int bind_group_id_ = -1;
};

}  // namespace videosource
#endif  // VIDEO_SOURCE_VIN_MIPI_CAM_VIN_MODULE_HAPI_H_
