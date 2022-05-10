/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2019-08-04 02:41:22
 * @Version: v0.0.1
 * @Brief: smartplugin declaration
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-09-30 00:45:01
 */

#ifndef INCLUDE_SMARTPLUGIN_SMARTPLUGIN_H_
#define INCLUDE_SMARTPLUGIN_SMARTPLUGIN_H_

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "smart_message/smart_message.h"
#include "smart_plugin/runtime_monitor.h"
#include "smart_plugin/traffic_info.h"
#include "venc_module.h"
#include "video_box_common.h"
#include "video_processor.h"
#include "vot_module.h"
#include "xproto/message/flowmsg.h"
#include "xproto/plugin/xpluginasync.h"
#include "xstream/xstream_world.h"
#include "xware_plugin/json_config_wrapper.h"
namespace solution {
namespace video_box {

using xproto::XPluginAsync;
using xproto::XProtoMessage;
using xproto::XProtoMessagePtr;

using xproto::message::SmartMessage;
using xstream::InputDataPtr;
using xstream::OutputDataPtr;
using xstream::XStreamSDK;

class JsonConfigWrapper;

class SmartPlugin : public XPluginAsync {
 public:
  SmartPlugin() = default;
  explicit SmartPlugin(const std::string &smart_config_file);

  void SetConfig(const std::string &config_file) {
    smart_config_file_ = config_file;
  }

  ~SmartPlugin() = default;
  int Init() override;
  int Start() override;
  int Stop() override;

 private:
  int Feed(XProtoMessagePtr msg);
  void OnCallback(xstream::OutputDataPtr out);
  int FeedPic(XProtoMessagePtr msg);
  void OnCallbackPic(xstream::OutputDataPtr out);
  void OnCallbackFeature(xstream::OutputDataPtr out);
  int FeedRecog(XProtoMessagePtr msg);

  void ParseConfig();
  void GetRtspConfigFromFile(const std::string &path);
  void GetDisplayConfigFromFile(const std::string &path);

  std::vector<std::shared_ptr<XStreamSDK>> sdk_;
  std::shared_ptr<XStreamSDK> pic_sdk_;
  std::shared_ptr<XStreamSDK> feature_sdk_;

  std::string smart_config_file_;
  std::string rtsp_config_file_;
  std::string display_config_file_;
  std::shared_ptr<RuntimeMonitor> monitor_;
  std::shared_ptr<JsonConfigWrapper> config_;
  std::string xstream_workflow_cfg_file_;
  std::string xstream_workflow_cfg_pic_file_;
  std::string xstream_workflow_cfg_feature_file_;
  bool enable_profile_{false};
  std::string profile_log_file_;
  bool result_to_json{false};
  Json::Value root;
  std::shared_ptr<solution::video_box::VideoProcessor> video_processor_;
  bool running_venc_1080p_ = false;
  bool running_venc_720p_ = false;
  bool running_vot_ = true;
  bool encode_smart_ = true;
  bool enable_recog_{false};

  int channel_num_ = 0;
  int display_mode_ = 0;

  solution::video_box::smart_vo_cfg_t smart_vo_cfg_;

  // for test fps
  static void ComputeFpsThread(void *param);
  uint64_t smartframe_ = 0;
  std::thread read_thread_;
  bool running_;
  std::unordered_map<uint64_t,
                     std::unordered_map<uint64_t, std::shared_ptr<RecogResult>>>
      recog_cache_;
  std::mutex cache_mtx_;
  bool run_smart_ = true;
};

}  // namespace video_box
}  // namespace solution

#endif  // INCLUDE_SMARTPLUGIN_SMARTPLUGIN_H_
