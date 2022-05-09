/*
 * Copyright (c) 2020 Horizon Robotics
 * @brief     definition of plugins
 * @author    zhe.sun
 * @date      2020.10.4
 */

#ifndef TUTORIALS_STAGE2_INCLUDE_PLUGINS_H_
#define TUTORIALS_STAGE2_INCLUDE_PLUGINS_H_

#include <iostream>
#include <chrono>
#include <string>
#include <memory>
#include <thread>
#include "number_message.h"

using xproto::XPluginAsync;
using xproto::XProtoMessagePtr;
class NumberProducerPlugin : public XPluginAsync {
 public:
  std::string desc() const {
    return "NumberProducerPlugin";
  }
  int Init() {
    total_cnt_ = 10;
    prd_thread_ = nullptr;
    return XPluginAsync::Init();
  }
  int Start() {
    std::cout << "total_cnt=" << total_cnt_ << std::endl;
    std::cout << desc() << " Start" << std::endl;
    prd_thread_ = new std::thread([&] (){
      for (uint32_t i = 0; i < total_cnt_ && !prd_stop_; i++) {
        auto np_msg1 = std::make_shared<NumberProdMessage1>(i);
        // 向总线发送消息，若超出最大限制数量，则持续等待直到消息队列长度满足要求再发送
        PushMsg(np_msg1);
        std::cout << "PushMsg NumberProdMessage1: "
                  << i << " success" << std::endl;
      }
      std::this_thread::sleep_for(std::chrono::seconds(5));
      for (uint32_t i = 0; i < total_cnt_ && !prd_stop_; i++) {
        auto np_msg2 = std::make_shared<NumberProdMessage2>(i);
        // 向总线发送消息【可能失败】，若未超出最大限制数量，发送消息，返回成功；否则不再发送，返回失败
        int ret = TryPushMsg(np_msg2);
        if (ret == 0) {
          std::cout << "TryPushMsg NumberProdMessage2: "
                    << i << " success." << std::endl;
        } else {
          std::cout << "TryPushMsg NumberProdMessage2 "
                    << i << " fail." << std::endl;
          std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
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
    RegisterMsg(TYPE_NUMBER_MESSAGE1, std::bind(&SumConsumerPlugin::Sum1,
                                               this, std::placeholders::_1));
    RegisterMsg(TYPE_NUMBER_MESSAGE2, std::bind(&SumConsumerPlugin::Sum2,
                                               this, std::placeholders::_1));
    return XPluginAsync::Init();
  }
  int Sum1(XProtoMessagePtr msg) {
    auto np_msg = std::static_pointer_cast<NumberProdMessage1>(msg);
    sum_ += np_msg->num_;
    std::cout << "Consume NumberProdMessage1, curr sum:"
              << sum_ << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return sum_;
  }
  int Sum2(XProtoMessagePtr msg) {
    auto np_msg = std::static_pointer_cast<NumberProdMessage2>(msg);
    sum_ += np_msg->num_;
    std::cout << "Consume NumberProdMessage2, curr sum:"
              << sum_ << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
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
    return XPluginAsync::DeInit();
  }
  std::string desc() const {
    return "SumConsumerPlugin";
  }

 private:
  float sum_;
};

#endif  // TUTORIALS_STAGE2_INCLUDE_PLUGINS_H_
