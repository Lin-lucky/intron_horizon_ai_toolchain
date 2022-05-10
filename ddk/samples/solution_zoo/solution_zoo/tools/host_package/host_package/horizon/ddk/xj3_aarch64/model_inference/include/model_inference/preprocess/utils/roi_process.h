/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of roi_process
 * @file   roi_process.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */
#ifndef ROI_PROCESS_H_
#define ROI_PROCESS_H_

#include <vector>
#include <string>
#include <map>
#include <memory>
#include "json/json.h"
#include "xstream/xstream_world.h"
#include "xstream/vision_type.h"

namespace inference {

class RoiProcess {
 public:
  enum RoiProcType {
    NONE,
    NORM_BY_WIDTH_LENGTH,
    NORM_BY_WIDTH_RATIO,
    NORM_BY_HEIGHT_LENGTH,
    NORM_BY_HEIGHT_RATIO,
    NORM_BY_LSIDE_LENGTH,
    NORM_BY_LSIDE_RATIO,
    NORM_BY_LSIDE_SQUARE,
    NORM_BY_DIAGONAL_SQUARE,
    NORM_BY_WIDTH_SQUARE,
    NORM_BY_HEIGHT_SQUARE
  };  // roi预处理类型: 参考CNNMethod::NormMethod
  const std::map<std::string, RoiProcType> norm_method_map_ = {
    {"norm_by_width_length", NORM_BY_WIDTH_LENGTH},
    {"norm_by_width_ratio", NORM_BY_WIDTH_RATIO},
    {"norm_by_height_length", NORM_BY_HEIGHT_LENGTH},
    {"norm_by_height_ratio", NORM_BY_HEIGHT_RATIO},
    {"norm_by_lside_length", NORM_BY_LSIDE_LENGTH},
    {"norm_by_lside_ratio", NORM_BY_LSIDE_RATIO},
    {"norm_by_lside_square", NORM_BY_LSIDE_SQUARE},
    {"norm_by_diagonal_square", NORM_BY_DIAGONAL_SQUARE},
    {"norm_by_width_square", NORM_BY_WIDTH_SQUARE},
    {"norm_by_height_square", NORM_BY_HEIGHT_SQUARE},
    {"norm_by_nothing", NONE}};

  RoiProcType roi_process_pipeline_ = NONE;

  float expand_scale_ = 1.0;  // 默认1表示不外扩
  float aspect_ratio_ = 0.0;  // 默认0表示保持roi原始宽高比

  // 加载配置文件信息，格式：
  //  {
  //     "roi_process_pipeline": [
  //         "norm_by_width_length(2.2, 1.2)"  // (expand_scale, aspect_ratio)
  //      ]
  //  }
  // json_value是array
  int Init(const std::string &json_str);

  void Execute(const std::vector<std::shared_ptr<xstream::BBox>> rois_in,
               const uint32_t total_width, const uint32_t total_height,
               std::vector<std::shared_ptr<xstream::BBox>> &rois_out);

  int NormalizeRoi(xstream::BBox *src, xstream::BBox *dst,
                   uint32_t total_w, uint32_t total_h);
};

}  // namespace inference

#endif  // ROI_PROCESS_H_
