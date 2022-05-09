/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: xudong.du
 * @Mail: xudong.du@horizon.ai
 * @Date: 2020-10-28
 * @Version: v0.0.0
 * @Brief: Test Multi source input
 */

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "json/json.h"
#include "method/multisource_method.h"
#include "xstream/xstream_world.h"

namespace Multisource {
class Callback {
 public:
  Callback() {}

  ~Callback() {}

  void OnCallback(xstream::OutputDataPtr output) {
    using xstream::MultiSourceOutput;
    using xstream::MultiSourceOutputPtr;
    auto out_test =
        std::static_pointer_cast<MultiSourceOutput>(output->datas_[0]);
    std::cout << "[Multisource]:source_id " << output->source_id_
              << " sum_result = " << out_test->sum_out << std::endl;
    resbuffer.push_back(out_test);
  }

  size_t GetResultNum() const { return resbuffer.size(); }

 private:
  std::vector<xstream::MultiSourceOutputPtr> resbuffer;
};

int AdjustConfigFile(const std::string &inconfig, const std::string &outconfig,
                     int32_t source_num, int32_t thread_num) {
  std::ifstream infile(inconfig);
  std::ofstream outfile(outconfig);
  Json::Value config;
  infile >> config;
  if (source_num <= 0) {
    config.removeMember("source_number");
  } else {
    config["source_number"] = source_num;
  }
  auto &workflow = config["workflow"];
  int node_size = workflow.size();
  for (int i = 0; i < node_size; ++i) {
    auto node = workflow[i];
    auto method_name = node["method_type"].asString();
    if (method_name == "MultiSourceTest") {
      workflow[i]["thread_count"] = thread_num > 0 ? thread_num : 1;
      break;
    }
  }
  outfile << config;
  return 0;
}

int DoTest(const std::string &config_template, const std::string &config_temp,
           const xstream::MethodInfo &methodinfo, int source_num,
           int thread_count) {
  using xstream::BaseDataPtr;
  using xstream::InputData;
  using xstream::InputDataPtr;
  using xstream::MultiSourceInput;

  AdjustConfigFile(config_template, config_temp, source_num, thread_count);

  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();
  Callback callback;
  source_num = std::max(source_num, 1);
  thread_count = std::max(thread_count, 1);

  flow->SetCallback(
      std::bind(&Callback::OnCallback, &callback, std::placeholders::_1));
  flow->SetConfig("config_file", config_temp);
  flow->Init();

  std::cout << "multisource Method Version : "
       << flow->GetVersion("multisource_node");

  InputDataPtr inputdata(new InputData());
  std::shared_ptr<MultiSourceInput> test_input;
  inputdata->datas_.resize(1);
  std::vector<size_t> source_ids;

  for (size_t source_id = 0; source_id < (size_t)source_num; source_id++) {
    source_ids.push_back(source_id);
  }
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::shuffle(source_ids.begin(), source_ids.end(),
               std::default_random_engine(seed));

  for (int num = 1; num <= 10; num++) {
    for (auto source_id : source_ids) {
      inputdata->source_id_ = source_id;
      test_input = std::make_shared<MultiSourceInput>();
      test_input->name_ = "test_input";
      test_input->sum_in = num;
      inputdata->datas_[0] = BaseDataPtr(test_input);
      flow->AsyncPredict(inputdata);
    }
  }

  // waiting for async function done
  while (callback.GetResultNum() != 10 * source_ids.size()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  delete flow;
  // 反初始化sdk
  return 0;
}

}  // namespace Multisource

static const char config_temp[] = "./temp.json";

int main(int argc, char const *argv[]) {
  using Multisource::DoTest;
  if (argc < 2) {
    std::cout << "Usage : ./stage4_multisource config" << std::endl;
    std::cout << "Example : ./stage4_multisource"
              << " ./config/multisource_test.json" << std::endl;
    return -1;
  }
  auto config_file = argv[1];

  xstream::MethodInfo methodinfo;
  methodinfo.is_need_reorder_ = true;
  methodinfo.is_src_ctx_dept_ = true;
  methodinfo.is_thread_safe_ = false;
  int sourcenum = 5;
  xstream::MultiSourceMethod::SetMethodInfo(methodinfo);

  DoTest(config_file, config_temp, methodinfo, sourcenum, -1);

  return 0;
}
