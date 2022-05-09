/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     BBoxFilter Method
 * @author    xudong.du
 * @email     xudong.du@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.23
 */

#include "faster_detect.h"

#include <time.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <random>

#include "./bbox.h"
#include "./gray_image.h"
#include "xstream/xstream_world.h"

namespace xstream {

int FasterDetect::Init(const std::string &config_file_path) {
  std::cout << "FasterDetect::Init " << config_file_path << std::endl;
  return 0;
}

int FasterDetect::UpdateParameter(InputParamPtr ptr) { return 0; }

InputParamPtr FasterDetect::GetParameter() const {
  auto param = std::make_shared<FasterDetectParam>("");
  return param;
}

void FasterDetect::Finalize() {
  std::cout << "FasterDetect::Finalize" << std::endl;
}

std::string FasterDetect::GetVersion() const { return ""; }

std::vector<std::vector<BaseDataPtr>> FasterDetect::DoProcess(
    const std::vector<std::vector<BaseDataPtr>> &input,
    const std::vector<InputParamPtr> &param) {
  std::cout << "FasterDetect::DoProcess " << input.size();
  std::vector<std::vector<BaseDataPtr>> output;
  output.resize(input.size());  // batch size
  // one batch
  for (size_t i = 0; i < input.size(); ++i) {
    auto &in_batch_i = input[i];
    auto &out_batch_i = output[i];
    // one slot
    out_batch_i.push_back(std::make_shared<BaseDataVector>());
    auto in_imags = std::static_pointer_cast<BaseDataVector>(in_batch_i[0]);
    auto image =
        std::static_pointer_cast<xstream::GrayImage>(in_imags->datas_[0]);

    if (image->state_ == DataState::INVALID) {
      std::cout << "input slot 0 is invalid";
      continue;
    }
    auto out_rects = std::static_pointer_cast<BaseDataVector>(out_batch_i[0]);

    std::default_random_engine random(time(NULL));
    std::cout << "width = " << image->Width();
    std::uniform_int_distribution<int> point_width(0, image->Width());
    std::uniform_int_distribution<int> point_height(0, image->Height());
    for (int i = 0; i < 10; i++) {
      auto bbox = std::make_shared<BBox>();
      bbox->x1 = point_width(random);
      bbox->y1 = point_height(random);
      bbox->x2 = point_width(random);
      bbox->y2 = point_height(random);
      bbox->score = 0.99f;
      out_rects->datas_.push_back(bbox);
      std::cout << "box: " << bbox->x1 << ","
                << bbox->y1 << "," << bbox->x2 << ","
                << bbox->y2 << ", score: " << bbox->score;
    }
  }
  return output;
}

}  // namespace xstream
