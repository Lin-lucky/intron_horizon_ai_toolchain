/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of roi_process
 * @file   roi_process.cc
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */

#include "model_inference/preprocess/utils/roi_process.h"
#include "hobotlog/hobotlog.hpp"
#include <libyuv.h>

namespace inference {

// {
//     // 支持none；内置预定义Predefined;用户自定义的class_name
//     "class_name": "pyramid_roi_preprocess",
//     // 输入数据为金字塔，对于run_model模型从指定层来获取金字塔图像
//     "pyramid_layer": 4,
//     "config": {
//         // 内置预定义ROI预处理逻辑
//         // 支持的scale缩放类型，待细化，当前仅仅支持等比例缩放到2.2
//         "roi_process_pipeline": [
//           "norm_by_width_length(2.2, 1.2)"  // (expand_scale, aspect_ratio)
//         ]
//     }
// }

int RoiProcess::Init(const std::string &json_str) {
  // string转json
  Json::Reader Reader;
  Json::Value json_value;
  Reader.parse(json_str, json_value);

  int pipeline_size = json_value.size();
  HOBOT_CHECK(pipeline_size == 1);
  std::string process_desp = json_value[0].asString();
  // 操作类型
  std::string process = process_desp.substr(0,
      process_desp.find_first_of("("));
  auto iter = norm_method_map_.find(process);
  HOBOT_CHECK(iter != norm_method_map_.end()) << "not support " << process;
  roi_process_pipeline_ = iter->second;

  std::vector<float> res;
  auto check_str_valid_func = [](std::string str) {
    str.erase(0, str.find_first_not_of(" "));
    str.erase(str.find_last_not_of(" ") + 1);
    LOGD << "check str = \'" << str << "\'";
    if (str.empty()) {
      return false;
    }
    auto it = str.begin();
    while (it != str.end()) {
      if ((*it >= '0' && *it <= '9') || *it == '.') {
        it++;
        continue;
      } else {
        return false;
      }
    }
    return true;
  };

  // 操作参数
  int start_idx = process_desp.find_first_of("(");
  int end_idx = process_desp.find_first_of(")");
  if (start_idx > 0 && end_idx > start_idx) {
    std::string params = process_desp.substr(
        start_idx + 1, end_idx - start_idx - 1);
    std::istringstream iss(params);
    for (std::string item; getline(iss, item, ',');) {
      float f_item;
      if (false == check_str_valid_func(item)) {
        LOGF << "Invalid config param: \'" << item << "\'";
        return -1;
      }
      try {
        f_item = std::stof(item);
      } catch (std::exception& e) {
        LOGF << e.what();
        LOGF << "Invalid param " << res.size() << ": " << item;
        return -1;
      }
      res.push_back(f_item);
    }
  }

  if (res.size() > 0) {
    expand_scale_ = res[0];
  }
  if (res.size() > 1) {
    aspect_ratio_ = res[1];
  }
  return 0;
}

void RoiProcess::Execute(
    const std::vector<std::shared_ptr<xstream::BBox>> rois_in,
    const uint32_t total_width, const uint32_t total_height,
    std::vector<std::shared_ptr<xstream::BBox>> &rois_out) {
  int roi_size = rois_in.size();
  rois_out.resize(roi_size);
  for (int i = 0; i < roi_size; i++) {
    auto roi_in = rois_in[i];
    auto roi_out = std::make_shared<xstream::BBox>();
    rois_out[i] = roi_out;
    if (roi_in->state_ != xstream::DataState::VALID ||
        roi_process_pipeline_ == NONE) {
      *roi_out = *roi_in;
    } else {
      // do norm
      NormalizeRoi(roi_in.get(), roi_out.get(),
                   total_width, total_height);
    }
  }
}

int RoiProcess::NormalizeRoi(
    xstream::BBox *src, xstream::BBox *dst,
    uint32_t total_w, uint32_t total_h) {
  *dst = *src;
  float box_w = dst->x2_ - dst->x1_;
  float box_h = dst->y2_ - dst->y1_;
  float center_x = (dst->x1_ + dst->x2_) / 2.0f;
  float center_y = (dst->y1_ + dst->y2_) / 2.0f;
  float w_new = box_w;
  float h_new = box_h;

  switch (roi_process_pipeline_) {
    case NORM_BY_WIDTH_LENGTH: {
      w_new = box_w * expand_scale_;
      h_new = box_h + w_new - box_w;
      if (h_new <= 0) return -1;
    } break;
    case NORM_BY_WIDTH_RATIO:
    case NORM_BY_HEIGHT_RATIO:
    case NORM_BY_LSIDE_RATIO: {
      h_new = box_h * expand_scale_;
      w_new = box_w * expand_scale_;
    } break;
    case NORM_BY_HEIGHT_LENGTH: {
      h_new = box_h * expand_scale_;
      w_new = box_w + h_new - box_h;
      if (w_new <= 0) return -1;
    } break;
    case NORM_BY_LSIDE_LENGTH: {
      if (box_w > box_h) {
        w_new = box_w * expand_scale_;
        h_new = box_h + w_new - box_w;
        if (h_new <= 0) return -1;
      } else {
        h_new = box_h * expand_scale_;
        w_new = box_w + h_new - box_h;
        if (w_new <= 0) return -1;
      }
    } break;
    case NORM_BY_LSIDE_SQUARE: {
      if (box_w > box_h) {
        w_new = box_w * expand_scale_;
        h_new = w_new / aspect_ratio_;
      } else {
        h_new = box_h * expand_scale_;
        w_new = h_new * aspect_ratio_;
      }
    } break;
    case NORM_BY_DIAGONAL_SQUARE: {
      float diagonal = sqrt(pow(box_w, 2.0) + pow(box_h, 2.0));
      w_new = h_new = diagonal * expand_scale_;
    } break;
    case NORM_BY_WIDTH_SQUARE: {
        w_new = box_w * expand_scale_;
        h_new = w_new;
    } break;
    case NORM_BY_HEIGHT_SQUARE: {
        h_new = box_h * expand_scale_;
        w_new = h_new;
    } break;
    case NONE:
      break;
    default:
      return 0;
  }
  dst->x1_ = center_x - w_new / 2;
  dst->x2_ = center_x + w_new / 2;
  dst->y1_ = center_y - h_new / 2;
  dst->y2_ = center_y + h_new / 2;

  // 超出图像范围做截断
  dst->x1_ = std::max(0.0f, dst->x1_);
  dst->y1_ = std::max(0.0f, dst->y1_);
  dst->x2_ = std::min(static_cast<float>(total_w - 1), dst->x2_);
  dst->y2_ = std::min(static_cast<float>(total_h - 1), dst->y2_);
  LOGD << "norm roi[x1, y1, x2, y2]: [" << dst->x1_ << ", " << dst->y1_ << ", "
       << dst->x2_ << ", " << dst->y2_ << "]";
  return 0;
}

}  // namespace inference
