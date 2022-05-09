/*!
 * -------------------------------------------
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     sample_plugin.cpp
 * \Author Songshan Gong
 * \Mail     songshan.gong@horizon.ai
 * \Version  1.0.0.0
 * \Date     2019-07-30
 * \Brief    Sample custom plugin
 * \DO NOT MODIFY THIS COMMENT, \
 * \WHICH IS AUTO GENERATED BY EDITOR
 * -------------------------------------------
 */

#include <unistd.h>
#include <signal.h>
#include <sstream>
#include <iostream>
#include <thread>
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

#define TYPE_SAMPLE_MESSAGE "XPLUGIN_SAMPLE_MESSAGE"
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_SAMPLE_MESSAGE)

struct NumberProdMessage : XProtoMessage {
  float num_;
  explicit NumberProdMessage(float num) :num_(num) {
    type_ = TYPE_SAMPLE_MESSAGE;
  }
  std::string Serialize() override {
    std::ostringstream ss;
    ss << num_;
    return std::string(ss.str());
  }
};

class NumberProducerPlugin : public XPluginAsync {
 public:
  std::string desc() const {
    return "NumberProducerPlugin";
  }
  int Init() {
    total_cnt_ = 50;
    prd_thread_ = nullptr;
    return XPluginAsync::Init();
  }
  int Start() {
    std::cout << "total_cnt=" << total_cnt_ << std::endl;
    std::cout << desc() << " Start" << std::endl;
    prd_thread_ = new std::thread([&] (){
      for (uint32_t i = 0; i < total_cnt_ && !prd_stop_; i++) {
        auto np_msg = std::make_shared<NumberProdMessage>(5);
        PushMsg(np_msg);
        std::this_thread::sleep_for(milliseconds(40));
      }
    });
    return 0;
  }
  int Stop() {
    prd_stop_ = true;
    prd_thread_->join();
    if (prd_thread_) {
      delete prd_thread_;
    }
    std::cout << desc() << " Stop" << std::endl;
    return 0;
  }
  int DeInit() {
    return XPluginAsync::DeInit();
  }

 private:
  uint32_t total_cnt_;
  std::thread *prd_thread_;
  bool prd_stop_{false};
};

class SumConsumerPlugin : public XPluginAsync {
 public:
  int Init() override {
    sum_ = 0.f;
    RegisterMsg(TYPE_SAMPLE_MESSAGE, std::bind(&SumConsumerPlugin::Sum,
                                               this, std::placeholders::_1));
    return XPluginAsync::Init();
  }
  int Sum(XProtoMessagePtr msg) {
    auto np_msg = std::static_pointer_cast<NumberProdMessage>(msg);
    sum_ += np_msg->num_;
    std::cout << "curr sum:" << sum_ << std::endl;
    return sum_;
  }

  int Start() {
    std::cout << desc() << " Start" << std::endl;
    return 0;
  }
  int Stop() {
    std::cout << desc() << " Stop" << std::endl;
    return 0;
  }
  int DeInit() {
    UnRegisterMsg(TYPE_SAMPLE_MESSAGE);
    return XPluginAsync::DeInit();
  }
  std::string desc() const {
    return "SumConsumerPlugin";
  }

 private:
  float sum_;
};

int main() {
  auto np_plg = std::make_shared<NumberProducerPlugin>();
  auto sc_plg = std::make_shared<SumConsumerPlugin>();

  np_plg->Init();
  sc_plg->Init();

  np_plg->Start();
  sc_plg->Start();

  std::this_thread::sleep_for(seconds(3));

  np_plg->Stop();
  sc_plg->Stop();

  np_plg->DeInit();
  sc_plg->DeInit();

  return 0;
}
