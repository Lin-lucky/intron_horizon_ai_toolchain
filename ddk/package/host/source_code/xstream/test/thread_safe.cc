/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: ronghui zhang
 * @Mail: zhangronghui@horizon.ai
 * @Date: 2019-11-30 01:15:22
 * @Version: v0.0.1
 * @Brief: test thread safe
 */

#include <gtest/gtest.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <thread>

#include "hobotlog/hobotlog.hpp"
#include "hobotxstream/data_types/filter_param.h"
#include "hobotxstream/data_types/orderdata.h"
#include "hobotxstream/xstream_config.h"
#include "method/bbox_filter.h"
#include "thread_safe_method.h"
#include "xstream/xstream_error.h"
#include "xstream/xstream_sdk.h"

namespace ThreadSafe {
clock_t begin_clock = 1;
std::set<int> used_sequence_id;
class Callback {
 public:
  Callback() {}

  ~Callback() {}

  void OnCallback(xstream::OutputDataPtr output) {
    std::cout << "**********OnCallBack*********" << std::endl;
    using xstream::BaseDataVector;
    if ((output->sequence_id_ == 99999) || (output->sequence_id_ % 1000 == 0)) {
      auto duration = clock() - begin_clock;
      std::cout << "seq: " << output->sequence_id_ << " duration:" << duration
                << std::endl;
    }

    std::cout << "======================" << std::endl;
    std::cout << "seq: " << output->sequence_id_ << std::endl;
    std::cout << "method_unique_name: " << output->unique_name_ << std::endl;
    std::cout << "error_code: " << output->error_code_ << std::endl;
    std::cout << "error_detail_: " << output->error_detail_ << std::endl;
    std::cout << "datas_ size: " << output->datas_.size() << std::endl;
    std::cout << "sequence_id_: " << output->sequence_id_ << std::endl;

    for (auto data : output->datas_) {
      if (data->error_code_ < 0) {
        std::cout << "data error: " << data->error_code_ << std::endl;
        continue;
      }
      std::cout << "data type_name : " << data->type_ << " " << data->name_
                << std::endl;
      BaseDataVector *pdata = reinterpret_cast<BaseDataVector *>(data.get());
      std::cout << "pdata size: " << pdata->datas_.size() << std::endl;
      for (uint i = 0; i < pdata->datas_.size(); i++) {
        auto safeData =
            std::static_pointer_cast<xstream::OrderData>(pdata->datas_[i]);
        std::set<int>::iterator it;
        for (it = used_sequence_id.begin(); it != used_sequence_id.end();
             it++) {
          EXPECT_NE(*it, (int)safeData->sequence_id);
        }
        std::cout << "used_sequence_id insert id = " << safeData->sequence_id
                  << std::endl;
        used_sequence_id.insert(safeData->sequence_id);
      }
    }
  }
};

int SafeTest(const std::string &config) {
  using xstream::BaseData;
  using xstream::BaseDataPtr;
  using xstream::BaseDataVector;
  using xstream::InputData;
  using xstream::InputDataPtr;

  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();
  Callback callback;
  flow->SetCallback(
      std::bind(&Callback::OnCallback, &callback, std::placeholders::_1));
  flow->SetConfig("config_file", config);
  flow->Init();
  std::cout << "data_test Method Version : "
            << flow->GetVersion("threadsafe_test1") << std::endl;

  xstream::InputParamPtr pass_through(new xstream::DisableParam(
      "passthroughMethod1", xstream::DisableParam::Mode::PassThrough));

  std::cout << "***********************" << std::endl
            << "testing aysnc function" << std::endl
            << "***********************" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  for (int i = 0; i < 10; i++) {
    InputDataPtr inputdata(new InputData());
    BaseDataVector *data(new BaseDataVector);
    xstream::OrderData *testData(new xstream::OrderData(i));
    testData->name_ = "input_test_data";
    data->name_ = "input_test_data";
    data->datas_.push_back(BaseDataPtr(testData));
    inputdata->datas_.push_back(BaseDataPtr(data));
    inputdata->params_.push_back(pass_through);
    flow->AsyncPredict(inputdata);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // waiting for async function done
  std::this_thread::sleep_for(std::chrono::seconds(5));
  delete flow;
  // 初始化sdk
  return 0;
}

}  // namespace ThreadSafe

TEST(ThreadSafe, compatiable) {
  ThreadSafe::SafeTest("./test/configs/threadsafe_test.json");
}
