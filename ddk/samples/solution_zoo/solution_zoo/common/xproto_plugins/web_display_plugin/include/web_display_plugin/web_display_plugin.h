/*!
 * Copyright (c) 2016-present, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     websocketplugin.h
 * \Author   ronghui.zhang
 * \Version  1.0.0.0
 * \Date     2020/5/12
 * \Brief    implement of api header
 */
#ifndef INCLUDE_WEBSOCKETPLUGIN_WEBSOCKETPLUGIN_H_
#define INCLUDE_WEBSOCKETPLUGIN_WEBSOCKETPLUGIN_H_
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "thread_pool/thread_pool.h"
#include "xproto/msg_type/protobuf/x3.pb.h"
#include "xproto/msg_type/smart_legible_message.h"
#include "xproto/xproto_world.h"

namespace xproto {
using horizon::vision::CThreadPool;
using xproto::XPluginAsync;
using xproto::XProtoMessagePtr;
using xproto::message::SmartLegibleMessage;
using xproto::message::SmartLegibleMessagePtr;
struct compare_msg {
  bool operator()(const SmartLegibleMessagePtr m1,
                  const SmartLegibleMessagePtr m2) {
    return (m1->time_stamp_ > m2->time_stamp_);
  }
};
class UwsServer;
class WebDisplayConfig;
class WebDisplayPlugin : public XPluginAsync {
 public:
  WebDisplayPlugin() = delete;
  explicit WebDisplayPlugin(std::string config_path);
  ~WebDisplayPlugin() override;
  int Init() override;
  int Start() override;
  int Stop() override;
  std::string desc() const { return "WebDisplayPlugin"; }

 private:
  // 这里需要与SmartPlugin产生的感知消息匹配
  // 一般情况，若SmartPlugin的派生类产生新的消息类型
  // (继承CustomSmartMessage),则需要继承WebDisplayPlugin，
  // 重写GetSmartMessageType接口
  virtual std::string GetSmartMessageType() {
    // 当前解决方案默认使用TYPE_SMART_LEGIBLE_MESSAGE
    return TYPE_SMART_LEGIBLE_MESSAGE;
  }
  int FeedSmart(XProtoMessagePtr msg);
  int SendSmartMessage(XProtoMessagePtr msg);

  void ParseConfig();
  int Reset();
  void map_smart_proc();

 private:
  std::shared_ptr<UwsServer> uws_server_;
  std::string config_file_;
  std::shared_ptr<WebDisplayConfig> config_;
  std::shared_ptr<std::thread> worker_;
  std::mutex map_smart_mutex_;
  bool map_stop_ = false;
  std::condition_variable map_smart_condition_;
  const uint8_t cache_size_ = 25;  // max input cache size
  std::priority_queue<SmartLegibleMessagePtr,
                      std::vector<SmartLegibleMessagePtr>, compare_msg>
      x3_smart_msg_;
  int origin_image_width_ = 1920;  // update by FeedVideo
  int origin_image_height_ = 1080;
  int dst_image_width_ = 1920;  // update by FeedVideo
  int dst_image_height_ = 1080;
  std::mutex smart_mutex_;
  bool smart_stop_flag_;
  CThreadPool data_send_thread_;
};

}  // namespace xproto

#endif  // INCLUDE_WEBSOCKETPLUGIN_WEBSOCKETPLUGIN_H_
