/*
 * @Description: implement of websocketconfig
 * @Author: ronghui.zhang
 * @Date: 2020-05-14
 * @Copyright 2020 Horizon Robotics, Inc.
 */
#ifndef XPROTO_WEB_DISPLAY_PLUGIN_INCLUDE_CONFIG_H_
#define XPROTO_WEB_DISPLAY_PLUGIN_INCLUDE_CONFIG_H_

#include <string>
#include <mutex>
#include <memory>
#include "json/json.h"

namespace xproto {
class RTSPServerPlugin;

enum VideoType {H264, H265};

class RTSPPluginConfig {
  friend class RTSPServerPlugin;

 public:
  RTSPPluginConfig() = delete;
  explicit RTSPPluginConfig(const std::string &path);
  bool LoadConfig();
  std::string GetValue(const std::string &key);
  Json::Value GetJson() const;

 private:
  bool CheckConfig();
  std::string path_;
  Json::Value json_;
  std::mutex mutex_;

  int enable_smart_;
  uint8_t layer_;
  VideoType video_type_;
  uint32_t image_width_, image_height_;
  uint32_t data_buf_size_, packet_size_;

  int frame_buf_depth_ = 0;
  int rotation_;
  int mirror_;
  int use_vb_ = 0;
  int is_cbr_ = 1;
  int bitrate_ = 6000;
  int debug_encode_cost_ = 0;
  int debug_dump_stream_ = 0;
  std::string rtsp_server_config_;
};

}  // namespace xproto
#endif  // XPROTO_WEB_DISPLAY_PLUGIN_INCLUDE_CONFIG_H_
