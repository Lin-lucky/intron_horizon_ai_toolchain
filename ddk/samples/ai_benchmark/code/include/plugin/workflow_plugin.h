// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _PLUGIN_WORKFLOW_PLUGIN_H_
#define _PLUGIN_WORKFLOW_PLUGIN_H_

#include <memory>
#include <string>

#include "base_plugin.h"

class WorkflowPlugin : public BasePlugin {
 public:
  WorkflowPlugin() = default;

  int Init(const std::string& config_file,
           const std::string& config_json_string);

  int Start() override;

  int FeedWorkflow(XProtoMessagePtr msg);

  int OnCallback(xstream::OutputDataPtr output_data);

  int Stop() override;

  ~WorkflowPlugin() override;

 private:
  std::shared_ptr<xstream::XStreamSDK> flow_;
};

#endif  // _PLUGIN_WORKFLOW_PLUGIN_H_
