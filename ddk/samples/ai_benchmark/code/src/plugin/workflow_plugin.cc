// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "plugin/workflow_plugin.h"

#include <iostream>

#include "glog/logging.h"
#include "rapidjson/document.h"

int WorkflowPlugin::Init(const std::string& config_file,
                         const std::string& config_json_string) {
  flow_.reset(xstream::XStreamSDK::CreateSDK());
  if (!config_json_string.empty()) {
    flow_->SetConfig("config_string", config_json_string);
  } else if (!config_file.empty()) {
    flow_->SetConfig("config_file", config_file);
  }
  flow_->SetConfig("profiler", "off");
  //  flow_->SetConfig("profiler_file", "profiler_file.log");
  if (flow_->Init() != 0) {
    VLOG(EXAMPLE_SYSTEM) << "XStreamSDK init failed";
    return -1;
  }
  RegisterMsg(
      TYPE_INPUT_MESSAGE,
      std::bind(&WorkflowPlugin::FeedWorkflow, this, std::placeholders::_1));

  flow_->SetCallback(
      std::bind(&WorkflowPlugin::OnCallback, this, std::placeholders::_1));

  VLOG(EXAMPLE_DETAIL) << "WorkflowPlugin inited.";
  return XPluginAsync::Init();
}

int WorkflowPlugin::Start() {
  VLOG(EXAMPLE_DETAIL) << "WorkflowPlugin start.";
  return 0;
}

int WorkflowPlugin::FeedWorkflow(XProtoMessagePtr msg) {
  auto input_msg = std::static_pointer_cast<InputMessage>(msg);
  auto image_tensor = input_msg->image_tensor;

  xstream::InputDataPtr input_data(new xstream::InputData());
  image_tensor->name_ = "input_data";
  input_data->datas_.push_back(xstream::BaseDataPtr(image_tensor));

  auto id = flow_->AsyncPredict(input_data);
  if (id < 0) {
    VLOG(EXAMPLE_SYSTEM) << "WorkFlow feed input data failed.";
    return -1;
  }
  VLOG(EXAMPLE_DEBUG) << "Workflow feed, id:" << id;
  return 0;
}

int WorkflowPlugin::OnCallback(xstream::OutputDataPtr output_data) {
  auto image_tensor =
      std::static_pointer_cast<ImageTensor>(output_data->datas_[0]);
  auto perception =
      std::static_pointer_cast<Perception>(output_data->datas_[1]);

  auto msg = std::make_shared<OutputMessage>(image_tensor, perception);
  PushMsg(msg);
  VLOG(EXAMPLE_DEBUG) << "WorkFlow finish." << std::endl;
  return 0;
}

int WorkflowPlugin::Stop() {
  VLOG(EXAMPLE_DETAIL) << "WorkflowPlugin stop.";
  return 0;
}

WorkflowPlugin::~WorkflowPlugin() {}
