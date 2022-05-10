// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _PLUGIN_OUTPUT_PLUGIN_H_
#define _PLUGIN_OUTPUT_PLUGIN_H_

#include <string>
#include <unordered_map>

#include "base_plugin.h"
#include "output/output.h"

#define FPS_LOG "SHOW_FPS_LOG"

/**
 * Plugin for output consumer
 */
class OutputConsumerPlugin : public BasePlugin {
 public:
  OutputConsumerPlugin() = default;

  /**
   * Input output consumer plugin from config file & config string
   * @param[in] config_file: config file path
   *        the config file should be in the json format
   *        for example:
   *        {
   *            "output_type": "client", # one of [image, video, raw, client]
   *            ... # see  `client_output`
   *                # `image_list_output`
   *                # `raw_output`
   *                # `video_output` and so on
   *        }
   * @param[in] config_string: same as config file
   * @return 0 if success
   */
  int Init(std::string config_file, std::string config_string);

  int LoadConfig(std::string &config_string) override;

  int Send(XProtoMessagePtr msg);

  int Start() override;

  int Stop() override;

  ~OutputConsumerPlugin() override;

 private:
  OutputModule *output_module_ = nullptr;
  std::unordered_map<int, XProtoMessagePtr> cache_;
  int next_frame = 0;
  int32_t fps_status_ = 0;
};

#endif  // _PLUGIN_OUTPUT_PLUGIN_H_
