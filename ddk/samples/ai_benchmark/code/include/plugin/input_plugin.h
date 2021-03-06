// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _PLUGIN_INPUT_PLUGIN_H_
#define _PLUGIN_INPUT_PLUGIN_H_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <string>
#include <thread>

#include "base_plugin.h"
#include "input/data_iterator.h"

/**
 * Plugin for input producer
 */
class InputProducerPlugin : public BasePlugin {
 public:
  InputProducerPlugin() = default;

  /**
   * Init input producer plugin from config file & config string
   * @param[in] config_file: config file path
   *        the config_file should be in the json format
   *        example:
   *        {
   *            "input_type": "camera", # one of [camera, image,
   *                                    # network,preprocessed_image] "limit":
   * 2,
   *            ...   # see `camera_data_iterator`
   *                  # `image_list_data_iterator`
   *                  # `network_data_iterator`
   *                  # `preprocessed_image_iterator` and so on
   *        }
   * @param[in] config_string: config string, same as config_file
   * @return 0 if success
   */
  int Init(std::string config_file, std::string config_string) override;

  void Produce();

  int Release(XProtoMessagePtr msg);

  bool IsRunning();

  int Start() override;

  int Stop() override;

  int LoadConfig(std::string &config_string);

  ~InputProducerPlugin() override;

 private:
  DataIterator *data_iterator_ = nullptr;
  std::shared_ptr<std::thread> produce_thread_;
  std::atomic<bool> stop_;
  volatile int produced_count_ = 0;
  volatile int released_count_ = 0;
  int limit_ = 3;

  std::condition_variable cv_;
  std::mutex m_;
};

#endif  // _PLUGIN_INPUT_PLUGIN_H_
