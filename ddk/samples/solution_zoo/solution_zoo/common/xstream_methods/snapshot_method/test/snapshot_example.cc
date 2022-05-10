/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     SelectMethod Example
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2019.01.05
 */
#include <unistd.h>

#include <cassert>
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <thread>

#include "hobotlog/hobotlog.hpp"
#include "test_support.h"
#include "xstream/vision_type.h"
#include "xstream/xstream_world.h"

class Callback {
 public:
  Callback() = default;

  ~Callback() = default;

  void OnCallback(const xstream::OutputDataPtr &output) {
    using xstream::BaseDataVector;
    if (output->datas_.size() != 1) {
      std::cout << "output data size error" << std::endl;
    }
    auto &data = output->datas_[0];
    if (data->error_code_ < 0) {
      std::cout << "data error: " << data->error_code_ << std::endl;
    }
    auto *psnap_data = dynamic_cast<BaseDataVector *>(data.get());
    if (!psnap_data->datas_.empty()) {
      for (const auto &item : psnap_data->datas_) {
        assert("BaseDataVector" == item->type_);
        //  get the item score
        auto one_target_snapshot_info =
            std::static_pointer_cast<BaseDataVector>(item);

        for (auto &snapshot_info : one_target_snapshot_info->datas_) {
          auto one_snap =
              std::static_pointer_cast<xstream::XStreamSnapshotInfo>(
                  snapshot_info);
          WriteLog(one_snap);
          std::cout << "track_id:" << one_snap->value->track_id_ << " frame_id:"
                    << one_snap->value->origin_image_frame_->frame_id_
                    << " select_score:" << one_snap->value->select_value_;
          auto userdatas = one_snap->value->userdata_;
          for (size_t i = 0; i < userdatas.size(); i++) {
            auto &userdata = userdatas[i];
            std::cout << " userdata" << i << ":" << userdata->name_;
          }
          std::cout << std::endl;
        }
      }
    }
  }
};

int main(int argc, char const *argv[]) {
  SetLogLevel(HOBOT_LOG_INFO);

  // init the SDK
  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();
  Callback callback;
  flow->SetCallback(
      std::bind(&Callback::OnCallback, &callback, std::placeholders::_1));
  flow->SetConfig("config_file", "./config/snapshot.json");
  flow->Init();
  std::string algo_res = "./test/files/logdh5cdump.log";
  std::ifstream fin(algo_res.data(), std::ios::binary);
  if (fin.fail()) {
    std::cout << "Open track_result failed" << std::endl;
    return -1;
  }

  std::string smart_frame;

  using std::chrono::duration;
  using std::chrono::high_resolution_clock;

  std::string img_format;
  int count = 0;
  while (getline(fin, smart_frame)) {
    xstream::InputDataPtr inputdata(new xstream::InputData());
    auto start_time = high_resolution_clock::now();
    int ret = ConstructInput(smart_frame, "", inputdata, img_format, true);
    auto construct_end_time = high_resolution_clock::now();
    duration<double, std::milli> proc_cost = construct_end_time - start_time;
    std::cout << "construct cost(ms):" << proc_cost.count() << std::endl;
    if (ret == 0) {
      // sync SnapShot
      auto out = flow->SyncPredict(inputdata);

      auto predict_end_time = high_resolution_clock::now();
      proc_cost = predict_end_time - construct_end_time;
      std::cout << "predict cost(ms):" << proc_cost.count() << std::endl;
      callback.OnCallback(out);

      auto callback_end_time = high_resolution_clock::now();
      proc_cost = callback_end_time - predict_end_time;
      std::cout << "callback cost(ms):" << proc_cost.count() << std::endl;
    }

    auto end_time = high_resolution_clock::now();
    proc_cost = end_time - start_time;
    std::cout << "Process Frame cost(ms):" << proc_cost.count() << std::endl;

    count++;
  }

  delete flow;
  return 0;
}
