/*
 * @Description: implement of visualplugin
 * @Author: GYW
 * @Date: 2020-03-30
 * @Copyright 2020 Horizon Robotics, Inc.
 */
#ifndef INCLUDE_VISUALPLUGIN_VISUALPLUGIN_H_
#define INCLUDE_VISUALPLUGIN_VISUALPLUGIN_H_

#include <turbojpeg.h>

#include <memory>
#include <mutex>
#include <vector>
#include <string>
#include <thread>

#include "visual_plugin/convert.h"
#include "visual_plugin/horizon_server_api.h"
#include "visual_plugin/visual_config.h"
#include "xproto/message/flowmsg.h"
#include "xproto/plugin/xpluginasync.h"

namespace xproto {

using xproto::XProtoMessagePtr;

class VisualPlugin : public xproto::XPluginAsync {
 public:
  VisualPlugin() = delete;
  explicit VisualPlugin(const std::string &config_file);
  ~VisualPlugin() override;
  int Init() override;
  int Start() override;
  int Stop() override;
  std::string desc() const { return "visualplugin"; }

 private:
  enum InputType {
    VT_VIDEO,  // VioMessage
    VT_SMART   // SmartMessage
  };

  struct VisualInput {
    InputType type;
    XProtoMessagePtr frame_msg;
  };

  int Reset();
  void PushFrame(InputType type, XProtoMessagePtr frame_msg);

  int EncodeJPG(tjhandle handle_tj, const unsigned char *yuv_buf,  // NOLINT
                int width,                                         // NOLINT
                int height, unsigned char **jpeg_buf,              // NOLINT
                unsigned long *jpeg_size,                          // NOLINT
                int quality = 50);                                 // NOLINT

  // encode thread
  int EncodeThread();

  int FeedVideo(XProtoMessagePtr msg);
  int FeedSmart(XProtoMessagePtr msg);

 private:
  std::string config_file_;
  std::shared_ptr<VisualConfig> config_;

  const uint8_t cache_size_ = 25;  // max input cache size
  bool stop_flag_;
  bool fifo_open_flag_;
  std::mutex map_mutex_;
  std::vector<VisualInput *> input_frames_;
  std::shared_ptr<std::thread> worker_;
  SERVER_PARAM_S server_param_;
  std::vector<uchar> h264_sps_frame_;
};

}  // namespace xproto
#endif
