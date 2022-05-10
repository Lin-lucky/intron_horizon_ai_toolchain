// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <fstream>

#include "gflags/gflags.h"
#include "glog/logging.h"
#include "input/data_iterator.h"
#include "plugin/input_plugin.h"
#include "plugin/output_plugin.h"
#include "plugin/workflow_plugin.h"
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/writer.h"

#define EMPTY ""

DEFINE_int32(log_level,
             google::FATAL,
             "Logging level (INFO=0, WARNING=1, ERROR=2, FATAL=3)");
DEFINE_string(config_file, EMPTY, "Json config file");

int main(int argc, char **argv) {
  // Parsing command line arguments
  gflags::SetUsageMessage(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::cout << gflags::GetArgv() << std::endl;

  // Init logging
  google::InitGoogleLogging("");
  google::SetStderrLogging(0);
  FLAGS_colorlogtostderr = true;
  google::SetVLOGLevel("*", FLAGS_log_level);
  FLAGS_max_log_size = 200;
  FLAGS_logbufsecs = 0;
  FLAGS_logtostderr = true;

  VLOG(EXAMPLE_SYSTEM) << "EXAMPLE_SYSTEM";
  VLOG(EXAMPLE_REPORT) << "EXAMPLE_REPORT";
  VLOG(EXAMPLE_DETAIL) << "EXAMPLE_DETAIL";
  VLOG(EXAMPLE_DEBUG) << "EXAMPLE_DEBUG";

  // Parsing config
  std::ifstream ifs(FLAGS_config_file);
  rapidjson::IStreamWrapper isw(ifs);
  rapidjson::Document document;
  document.ParseStream(isw);
  LOG_IF(FATAL, document.HasParseError()) << "Parsing config file failed";
  LOG_IF(
      FATAL,
      !(document.HasMember("input_config") && document.HasMember("workflow") &&
        document.HasMember("output_config")))
      << "Missing config for input/workflow/output";

  // Input plugin
  rapidjson::StringBuffer input_config_buf;
  rapidjson::Writer<rapidjson::StringBuffer> input_writer(input_config_buf);
  document["input_config"].Accept(input_writer);
  auto input_plg = std::make_shared<InputProducerPlugin>();
  int ret_code = input_plg->Init(EMPTY, input_config_buf.GetString());
  LOG_IF(FATAL, ret_code != 0) << "Input plugin init failed";

  // Workflow plugin
  rapidjson::StringBuffer workflow_config_buf;
  rapidjson::Writer<rapidjson::StringBuffer> workflow_writer(
      workflow_config_buf);
  document.Accept(workflow_writer);
  auto workflow_plg = std::make_shared<WorkflowPlugin>();
  ret_code = workflow_plg->Init(EMPTY, workflow_config_buf.GetString());
  LOG_IF(FATAL, ret_code != 0) << "Workflow init failed";
  //
  // Output plugin
  rapidjson::StringBuffer output_config_buf;
  rapidjson::Writer<rapidjson::StringBuffer> output_writer(output_config_buf);
  document["output_config"].Accept(output_writer);
  auto output_plg = std::make_shared<OutputConsumerPlugin>();
  ret_code = output_plg->Init(EMPTY, output_config_buf.GetString());
  LOG_IF(FATAL, ret_code != 0) << "Output plugin init failed";

  // Start
  ret_code = input_plg->Start();
  LOG_IF(FATAL, ret_code != 0) << "Input plugin start failed";
  ret_code = workflow_plg->Start();
  LOG_IF(FATAL, ret_code != 0) << "Workflow plugin start failed";
  ret_code = output_plg->Start();
  LOG_IF(FATAL, ret_code != 0) << "Output plugin start failed";

  while (true) {
    if (!input_plg->IsRunning()) {
      VLOG(EXAMPLE_REPORT) << "finished!";
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // Stop
  input_plg->Stop();
  workflow_plg->Stop();
  output_plg->Stop();
  return 0;
}
