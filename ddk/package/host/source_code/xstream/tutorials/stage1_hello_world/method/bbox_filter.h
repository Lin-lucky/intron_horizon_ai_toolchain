/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @brief     BBoxFilter Method
 * @author    zhe.sun
 * @version   0.0.0.1
 * @date      2020.10.30
 */

#ifndef XSTREAM_FRAMEWORK_TUTORIALS_STAGE1_METHOD_BBOX_FILTER_H_
#define XSTREAM_FRAMEWORK_TUTORIALS_STAGE1_METHOD_BBOX_FILTER_H_

#include <string>
#include <iostream>
#include <memory>
#include <vector>
#include "method/bbox.h"
#include "xstream/xstream_world.h"

namespace xstream {

class BBoxFilter : public SimpleMethod {
 private:
  float score_threshold_ = 0.5;

 public:
  int Init(const std::string &config_file_path) override {
    std::cout << "BBoxFilter Init" << std::endl;
    return 0;
  }

  std::vector<BaseDataPtr> DoProcess(
      const std::vector<BaseDataPtr> &input,
      const xstream::InputParamPtr &param) override {
    std::cout << "BBoxScoreFilter::DoProcess " << input.size() << std::endl;
    std::vector<BaseDataPtr> output;

    // one frame
    for (size_t j = 0; j < input.size(); j++) {
      output.push_back(std::make_shared<BaseDataVector>());
      if (input[j]->state_ == DataState::INVALID) {
        std::cout << "input slot " << j << " is invalid" << std::endl;
        continue;
      }
      auto in_rects = std::static_pointer_cast<BaseDataVector>(input[j]);
      auto out_rects = std::static_pointer_cast<BaseDataVector>(output[j]);
      for (auto &in_rect : in_rects->datas_) {
        auto bbox = std::static_pointer_cast<BBox>(in_rect);
        if (bbox->score > score_threshold_) {
          out_rects->datas_.push_back(in_rect);
        } else {
          std::cout << "filter out: " << bbox->x1 << "," << bbox->y1 << ","
                    << bbox->x2 << "," << bbox->y2 << ", score: " << bbox->score
                    << std::endl;
        }
      }
    }
    return output;
  }

  void Finalize() override {
    std::cout << "BBoxFilter Finalize" << std::endl;
  }
};

}  // namespace xstream

#endif  // XSTREAM_FRAMEWORK_TUTORIALS_STAGE1_METHOD_BBOX_FILTER_H_
