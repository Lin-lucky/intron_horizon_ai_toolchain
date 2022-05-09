/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     BBoxFilter Method
 * @author    shuhuan.sun
 * @email     shuhuan.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.23
 */

#include "bbox_filter.h"

#include <fstream>
#include <iostream>
#include <memory>

#include "./bbox.h"
#include "./gray_image.h"
#include "xstream/xstream_world.h"

namespace xstream {

int BBoxFilter::Init(const std::string &config_file_path) {
  std::cout << "BBoxFilter::Init " << config_file_path << std::endl;
  return 0;
}

int BBoxFilter::UpdateParameter(InputParamPtr ptr) { return 0; }

InputParamPtr BBoxFilter::GetParameter() const {
  auto param = std::make_shared<BBoxFilterParam>("");
  return param;
}

void BBoxFilter::Finalize() {
  std::cout << "BBoxFilter::Finalize" << std::endl;
}

std::string BBoxFilter::GetVersion() const { return ""; }

std::vector<BaseDataPtr> BBoxLocationFilter::DoProcess(
    const std::vector<BaseDataPtr> &input,
    const InputParamPtr &param) {
  std::cout << "BBoxLocationFilter::DoProcess " << input.size() << std::endl;
  std::vector<BaseDataPtr> output;
  // one batch
  {
    output.push_back(std::make_shared<BaseDataVector>());
    auto in_imags = std::static_pointer_cast<BaseDataVector>(input[0]);
    auto image =
        std::static_pointer_cast<xstream::GrayImage>(in_imags->datas_[0]);
    if (image->state_ == DataState::INVALID) {
      std::cout << "input slot 0 is invalid" << std::endl;
      return output;
    }
    auto width = image->Width();
    auto height = image->Height();
    auto in_rects = std::static_pointer_cast<BaseDataVector>(input[1]);
    auto out_rects = std::static_pointer_cast<BaseDataVector>(output[0]);
    if (in_rects->state_ == DataState::INVALID) {
      std::cout << "input slot 1 is invalid" << std::endl;
      return output;
    }
    std::cout << "size = " << in_rects->datas_.size()
         << ", threshold = " << location_to_border_threshold_;
    for (auto &in_rect : in_rects->datas_) {
      auto bbox = std::static_pointer_cast<BBox>(in_rect);
      if (bbox->x1 >= location_to_border_threshold_ &&
          bbox->x1 <= (width - location_to_border_threshold_) &&
          bbox->x2 >= location_to_border_threshold_ &&
          bbox->x2 <= (width - location_to_border_threshold_) &&
          bbox->y1 >= location_to_border_threshold_ &&
          bbox->y1 <= (height - location_to_border_threshold_) &&
          bbox->y2 >= location_to_border_threshold_ &&
          bbox->y2 <= (height - location_to_border_threshold_)) {
        out_rects->datas_.push_back(in_rect);
      } else {
        std::cout << "BBoxLocationFilter filter out: " << bbox->x1 << ","
                  << bbox->y1 << "," << bbox->x2 << "," << bbox->y2
                  << ", location_to_border_threshold_: "
                  << location_to_border_threshold_ << std::endl;
      }
    }
  }
  return output;
}

std::vector<BaseDataPtr> BBoxAreaFilter::DoProcess(
    const std::vector<BaseDataPtr> &input,
    const InputParamPtr &param) {
  std::cout << "BBoxAreaFilter::DoProcess " << input.size() << std::endl;
  std::vector<BaseDataPtr> output;
  // one batch
  {
    output.push_back(std::make_shared<BaseDataVector>());

    auto in_rects = std::static_pointer_cast<BaseDataVector>(input[0]);
    auto out_rects = std::static_pointer_cast<BaseDataVector>(output[0]);
    if (in_rects->state_ == DataState::INVALID) {
      std::cout << "input slot 1 is invalid" << std::endl;
      return output;
    }
    std::cout << "rect size = " << in_rects->datas_.size();
    for (auto &in_rect : in_rects->datas_) {
      auto bbox = std::static_pointer_cast<BBox>(in_rect);
      if (bbox->Width() * bbox->Height() >= area_threshold_) {
        out_rects->datas_.push_back(in_rect);
      } else {
        std::cout << "BBoxAreaFilter filter out: "
             << bbox->x1 << "," << bbox->y1
             << "," << bbox->x2 << "," << bbox->y2
             << ", area_threshold_: " << area_threshold_;
      }
    }
  }
  return output;
}

std::vector<BaseDataPtr> BBoxShapeFilter::DoProcess(
    const std::vector<BaseDataPtr> &input,
    const InputParamPtr &param) {
  std::cout << "BBoxShapeFilter::DoProcess " << input.size() << std::endl;
  std::vector<BaseDataPtr> output;
  // one batch
  {
    output.push_back(std::make_shared<BaseDataVector>());
    output.push_back(std::make_shared<BaseDataVector>());

    auto in_rects = std::static_pointer_cast<BaseDataVector>(input[0]);
    auto out_rects = std::static_pointer_cast<BaseDataVector>(output[0]);
    auto out_classify_results =
        std::static_pointer_cast<BaseDataVector>(output[1]);
    if (in_rects->state_ == DataState::INVALID) {
      std::cout << "input slot 1 is invalid" << std::endl;
      return output;
    }
    std::cout << "rect size = " << in_rects->datas_.size();
    for (auto &in_rect : in_rects->datas_) {
      auto bbox = std::static_pointer_cast<BBox>(in_rect);
      auto one_classify_result = std::make_shared<BBoxClassifyResult>();
      if (bbox->Width() == bbox->Height()) {
        one_classify_result->result = BBoxClassifyResult::SHAPE_SQUARE;
        out_rects->datas_.push_back(in_rect);
      } else {
        one_classify_result->result = BBoxClassifyResult::SHAPE_RECT;
      }
      out_rects->datas_.push_back(in_rect);
      out_classify_results->datas_.push_back(one_classify_result);
    }
  }
  return output;
}

}  // namespace xstream
