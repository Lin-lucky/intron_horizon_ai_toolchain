/*
 * @Description: implement of reorder_plugin.h
 * @Author: shiyu.fu@horizon.ai
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */
#ifndef INCLUDE_REORDER_PLUGIN_REORDER_PLUGIN_H_
#define INCLUDE_REORDER_PLUGIN_REORDER_PLUGIN_H_

#include <atomic>
#include <chrono>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "blocking_queue/blocking_queue.hpp"
#include "reorder_message/reorder_message.h"
#include "smart_message/smart_message.h"
#include "transport_message/hbipc_message.h"
#include "xproto/msg_type//vio_message.h"
#include "xproto/xproto_world.h"

using xproto::message::SmartMessage;
using xproto::message::VioMessage;
using xproto::message::ReorderMessage;
using xproto::message::ReorderMsgPtr;

namespace xproto {

struct CustomReorderMessage : ReorderMessage {
  explicit CustomReorderMessage(XProtoMessagePtr msg, std::string type,
                                 uint64_t id) {
    type_ = TYPE_REORDER_MESSAGE;
    actual_msg_ = msg;
    actual_type_ = type;
    frame_id_ = id;
  }
  std::string Serialize() override;
};

struct compare{
  bool operator()(ReorderMsgPtr msg1, ReorderMsgPtr msg2){
    return msg1->frame_id_ > msg2->frame_id_;
  }
};

class ReorderPlugin : public XPluginAsync {
 public:
  ReorderPlugin() = default;
  explicit ReorderPlugin(std::string cfg_file);
  ~ReorderPlugin();

 public:
  /* xproto框架接口的封装函数 */
  // 初始化plugin
  int Init() override;
  // 反初始化plugin
  int Deinit();
  // 开启plugin服务
  int Start() override;
  // 关闭plugin服务
  int Stop() override;
  // 返回plugin的名称
  std::string desc() const { return "ReorderPlugin"; }

 private:
  int OnGetAppResult(const XProtoMessagePtr msg);
  XProtoMessagePtr Reorder(const ReorderMsgPtr msg);

 private:
  std::shared_ptr<std::thread> thread_;
  std::atomic<bool> is_stop_;
  std::priority_queue<ReorderMsgPtr,
                      std::vector<ReorderMsgPtr>, compare> msg_queue_;
  uint64_t target_frame_id_ = 0;
  int need_reorder_;
};

}  // namespace xproto

#endif  // INCLUDE_REORDER_PLUGIN_REORDER_PLUGIN_H_
