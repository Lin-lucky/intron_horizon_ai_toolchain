/*
 * Copyright (c) 2020 Horizon Robotics
 * @brief     sample_plugin
 * @author    zhe.sun
 * @date      2020.10.4
 */

#include <thread>
#include <chrono>
#include "plugins.h"
int main() {
  auto np_plg = std::make_shared<NumberProducerPlugin>();
  auto sc_plg = std::make_shared<SumConsumerPlugin>();

  // 设置该插件的最大消息队列大小，默认30
  sc_plg->SetPluginMsgLimit(5);

  np_plg->Init();
  sc_plg->Init();

  np_plg->Start();
  sc_plg->Start();

  std::this_thread::sleep_for(std::chrono::seconds(16));

  np_plg->Stop();
  sc_plg->Stop();

  np_plg->DeInit();
  sc_plg->DeInit();

  return 0;
}
