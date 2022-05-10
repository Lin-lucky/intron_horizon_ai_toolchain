/**
 * Copyright (c) 2020, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: xue.liang
 * @Mail: xue.liang@horizon.ai
 * @Date: 2020-12-15 20:38:52
 * @Version: v0.0.1
 * @Brief: roi_zoom_plugin impl based on xpp.
 * @Last Modified by: xue.liang
 * @Last Modified time: 2020-12-16 22:41:30
 */

#ifndef INCLUDE_ROI_ZOOM_PLUGIN_ROI_ZOOM_PLUGIN_H_
#define INCLUDE_ROI_ZOOM_PLUGIN_ROI_ZOOM_PLUGIN_H_

#include <future>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "hobotlog/hobotlog.hpp"
#include "json/json.h"
#include "roi_message/roi_message.h"
#include "singleton/singleton.h"
#include "smart_message/smart_message.h"
#include "xproto/msg_type/protobuf/x3.pb.h"
#include "xproto/xproto_world.h"
#include "xstream/xstream_world.h"

namespace xproto {

using xstream::InputDataPtr;
using xstream::OutputDataPtr;
using xstream::XStreamSDK;

class RoiZoomPlugin : public XPluginAsync {
 public:
  RoiZoomPlugin() = default;
  explicit RoiZoomPlugin(const std::string& config_file);

  ~RoiZoomPlugin() = default;
  int Init() override;
  int DeInit() override;
  int Start() override;
  int Stop() override;

 private:
  int StartPlugin(uint64_t msg_id);
  int StopPlugin(uint64_t msg_id);
  int OnGetSmarterResult(const XProtoMessagePtr& msg);
  int OnGetVioResult(const XProtoMessagePtr& msg);
  int ParseConfig();
  void ConvertSmartToRoiData(xstream::OutputDataPtr xstream_out,
                             const uint64_t frame_id, const uint64_t pts);

 private:
  std::atomic<bool> is_running_;
  std::string config_file_;
  Json::Value config_;
  bool auto_start_ = false;
  bool enable_vot_ = false;
  bool enable_intelligent_tracking_ = false;
  bool is_init_ = false;

  enum SmartType { SMART_FACE, SMART_BODY, SMART_VEHICLE };
  SmartType smart_type_ = SMART_BODY;
};

}  // namespace xproto
#endif  // INCLUDE_ROI_ZOOM_PLUGIN_ROI_ZOOM_PLUGIN_H_
