/**
 * @copyright Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @file      b_box_filter.cc
 * @brief     BBoxFilter class implementation
 * @author    Qingpeng Liu (qingpeng.liu@horizon.ai)
 * @version   0.0.0.1
 * @date      2020-01-03
 */

#include "method/bbox_filter.h"
#include <cassert>
#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <thread>
#include "xstream/xstream_world.h"
#include "json/json.h"
#include "method/bbox.h"

namespace xstream {

int BBoxFilter::Init(const std::string &config_file_path) {
  std::ifstream infile(config_file_path);
  Json::Value cfg_jv;
  infile >> cfg_jv;
  infile.close();
  area_threshold_ = cfg_jv["threshold"].asFloat();
  std::cout << "BBoxFilter::Init area_thres:" << area_threshold_ << std::endl;
  return 0;
}

int BBoxFilter::UpdateParameter(InputParamPtr ptr) {
  auto real_ptr = dynamic_cast<xstream::FilterParam *>(ptr.get());
  if (real_ptr->is_json_format_) {
    std::string content = real_ptr->Format();
    Json::CharReaderBuilder builder;
    JSONCPP_STRING error;
    std::shared_ptr<Json::CharReader> json_reader(builder.newCharReader());
    Json::Value cfg_jv;
    try {
      bool ret = json_reader->parse(content.c_str(),
                                    content.c_str() + content.size(),
                                    &cfg_jv, &error);
      if (ret && cfg_jv.isObject()) {
        if (cfg_jv.isMember("threshold")) {
          auto val = cfg_jv["threshold"];
          area_threshold_ = val.asFloat();
        } else {
          return -1;
        }
      } else {
        return -1;
      }
    } catch (std::exception &e) {
      return -1;
    }
  } else {
    if (real_ptr->HasThreshold()) {
      area_threshold_ = real_ptr->GetThreshold();
    }
  }
  return 0;
}

InputParamPtr BBoxFilter::GetParameter() const {
  auto param = std::make_shared<FilterParam>("");
  param->SetThreshold(area_threshold_);
  return param;
}

std::vector<BaseDataPtr> BBoxFilter::DoProcess(
    const std::vector<BaseDataPtr> &input,
    const InputParamPtr &param) {
  std::vector<BaseDataPtr> output;
  output.resize(input.size());
  for (size_t j = 0; j < input.size(); ++j) {
    auto in_rects = std::static_pointer_cast<BaseDataVector>(input[j]);
    assert("BaseDataVector" == in_rects->type_);
    auto out_rects = std::make_shared<BaseDataVector>();
    output[j] = std::static_pointer_cast<BaseData>(out_rects);
    for (auto &in_rect : in_rects->datas_) {
      auto bbox = std::static_pointer_cast<xstream::BBox>(in_rect);
      // BBoxFilter_A and BBoxFilter_B using the same input
      // copy input data in case one method affects the other
      auto out_rect = BaseDataPtr(
          new xstream::BBox(bbox->x1, bbox->y1, bbox->x2, bbox->y2));
      out_rect->type_ = bbox->type_;
      // skip filtered data
      if (in_rect->state_ == DataState::FILTERED) {
        out_rects->datas_.push_back(out_rect);
        continue;
      }
      assert("BBox" == out_rect->type_);
      if (bbox->Width() * bbox->Height() > area_threshold_) {
        out_rects->datas_.push_back(out_rect);
      } else {
        out_rect->state_ = DataState::FILTERED;
        out_rects->datas_.push_back(out_rect);
      }
    }
  }
  return output;
}

void BBoxFilter::Finalize() {
  std::cout << "BBoxFilter::Finalize" << std::endl;
}

std::string BBoxFilter::GetVersion() const { return "test_only"; }

}  // namespace xstream
