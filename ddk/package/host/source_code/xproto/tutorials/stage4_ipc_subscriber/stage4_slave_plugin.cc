/*!
 * -------------------------------------------
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     stage4_slave_plugin.cpp
 * \Author xudong.du
 * \Mail     xudong.du@horizon.ai
 * \Version  1.0.0.0
 * \Date     2019-07-30
 * \Brief    Sample custom plugin
 * \DO NOT MODIFY THIS COMMENT, \
 * \WHICH IS AUTO GENERATED BY EDITOR
 * -------------------------------------------
 */

#include <signal.h>
#include <unistd.h>

#include <iostream>
#include <sstream>
#include <thread>

#include "hobotlog/hobotlog.hpp"
#include "xproto/msg_type/control_message.h"
#include "xproto/msg_type/smart_legible_message.h"
#include "xproto/session/xsession.h"
#include "xproto/xproto_world.h"

/*
 * 框架包含三部分：总线、消息、插件。
 * 总线是系统框架，用户不用care;
 * 插件的定义：
 * 1.需要继承XPluginAsync，通常需要override的函数包括:
 *   Init()、Start()、Stop()、Desc();
 * 2.插件可能会生产消息或者向总线注册监听某类消息：
 *   如果生产消息需要调用PushMsg()将消息发送到总线分发;
 *   如果监听消息，需要实现消息处理函数，并在Init函数中
 *   注册需要监听的消息类型，并绑定对应的消息处理函数，
 *   同时在Init函数返回前调用父plugin的Init方法，
 *   通常是XPluginAsync::Init()。
 * 消息的声明与定义：
 * 1.使用宏XPLUGIN_REGISTER_MSG_TYPE,自定义消息类型，每个消息名字唯一；
 * 2.定义新的Message需要继承XProtoMessage;
 * 3.需要监听消息的插件需要：
 *   a.实现消息处理函数；
 *   b.覆盖Init函数，在其中完成监听消息注册，并绑定对应的消息处理函数，
 *     及其他初始化工作，同时在函数返回前需要调用父plugin的Init方法，
 *     通常是XPluginAsync::Init()。
 */

using xproto::XPluginAsync;
using xproto::XProtoMessage;
using xproto::XProtoMessagePtr;

using std::chrono::milliseconds;
using std::chrono::seconds;
using xproto::message::ControlMessage;
using xproto::message::ControlMessagePtr;
using xproto::message::SmartLegibleMessage;
using xproto::message::SmartLegibleMessagePtr;

XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_SMART_LEGIBLE_MESSAGE);
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_CONTROL_MESSAGE);

class ExampleSlavePlugin : public xproto::XPluginAsync {
 public:
  explicit ExampleSlavePlugin(std::string config_path) {
    LOGD << "ExampleSlavePlugin config_path = " << config_path;
  }
  ~ExampleSlavePlugin() {}
  int Init() override {
    LOGI << "ExampleSlavePlugin Init.";
    RegisterMsg(
        TYPE_SMART_LEGIBLE_MESSAGE,
        std::bind(&ExampleSlavePlugin::FeedSmart, this, std::placeholders::_1));
    return 0;
  }
  int DeInit() override {
    LOGD << "ExampleSlavePlugin::DeInit.";
    return 0;
  }
  int Start() override {
    LOGD << "ExampleSlavePlugin::Start.";
    return 0;
  }
  int Stop() override {
    LOGD << "ExampleSlavePlugin::Stop.";
    return 0;
  }

  std::string desc() const { return "ExampleSlavePlugin"; }

 private:
  void CreateControlMessage(uint64_t id) {
    static int cc = 0;
    auto control_msg = std::make_shared<ControlMessage>();
    //  构造一个ControlMessage
    control_msg->time_stamp_ = cc++;
    control_msg->message_.cmd_id_ = id;
    control_msg->message_.type_ = "reply";
    control_msg->message_.value_ = "i have got one SmartLegibleMessage";
    PushMsg(control_msg);
    LOGD << "Slave push control msg to.";
  }
  int FeedSmart(XProtoMessagePtr msg) {
    LOGD << "ExampleSlavePlugin::FeedSmart.";
    if (nullptr == msg) {
      return -1;
    }
    auto smart_data = std::static_pointer_cast<SmartLegibleMessage>(msg);
    LOGD << "******ExampleSlavePlugin::FeedSmart******";
    LOGD << "time_stamp: " << smart_data->time_stamp_;
    std::cout << "frame_id： " << smart_data->frame_id_ << std::endl;
    if (smart_data->background_img_) {
      LOGD << "img: size = " << smart_data->background_img_->DataSize()
           << ", uv size = " << smart_data->background_img_->DataUVSize()
           << ", width = " << smart_data->background_img_->Width()
           << ", hegith = " << smart_data->background_img_->Height()
           << ", type = " << smart_data->img_serialize_type_;
      if (dump_count_++ < 5) {
        std::string dump_filename = "test_pic_";
        if (smart_data->img_serialize_type_ ==
            xproto::message::kSmartImageTypeNv12) {
          dump_filename = dump_filename + std::to_string(dump_count_) + ".yuv";
        } else if (smart_data->img_serialize_type_ ==
                   xproto::message::kSmartImageTypeJpg) {
          dump_filename = dump_filename + std::to_string(dump_count_) + ".jpg";
        }
        SaveImageFile(
            reinterpret_cast<uint8_t *>(smart_data->background_img_->Data()),
            smart_data->background_img_->DataSize(), dump_filename);
      }
    }
    for (auto t : smart_data->smart_data_.targets_) {
      LOGD << "track_id: " << t->track_id_ << ", ";
      for (auto b : t->boxs_) {
        LOGW << *b;
      }
      for (auto attr : t->attributes_) {
        LOGW << *attr;
      }
      for (auto lmk : t->lmks_) {
        LOGW << *lmk;
      }
    }
    //  收到一帧有效message, 向数据总线push一个ControlMessage
    CreateControlMessage(smart_data->frame_id_);
    LOGD << "******ExampleSlavePlugin::FeedSmart end******";
    return 0;
  }

  int SaveImageFile(uint8_t *addr, int len, std::string name) {
    FILE *fd = fopen(name.c_str(), "wb+");
    if (!fd) {
      LOGE << "open file name:%s failure!";
      return -1;
    }
    LOGD << "filename = " << name << ", len = " << len
         << ", addr = " << reinterpret_cast<int64_t>(addr);

    fwrite(addr, sizeof(char), len, fd);
    fflush(fd);
    fclose(fd);
    return 0;
  }
  int dump_count_ = 0;
};

int main(int argc, char **argv) {
  if (argc < 3) {
    std::cout
        << "Usage: stage4_ipc_subscriber ip port"
        << std::endl;
    return 0;
  }
  SetLogLevel(HOBOT_LOG_DEBUG);
  std::string ip = std::string(argv[1]);
  uint16_t port = atoi(argv[2]);
  std::cout << "ip = " << ip << ", prot = " << port << std::endl;

  xproto::XSession::Instance().ConnectTo(ip, port);

  std::string example_config = "example.json";

  auto example_plg = std::make_shared<ExampleSlavePlugin>(example_config);

  example_plg->Init();

  example_plg->Start();

  std::this_thread::sleep_for(seconds(20));

  example_plg->Stop();

  example_plg->DeInit();
  xproto::XSession::Instance().Reset();

  return 0;
}
