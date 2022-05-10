/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief     contours_method
 * @author    hangjun.yang
 * @date      2021.05.15
 */

#include "contours_method/contours_method.h"

#include <string>
#include <vector>
#include <cmath>
#include <fstream>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "hobotlog/hobotlog.hpp"
#include "xstream/xstream_data.h"
#include "xstream/vision_type.h"
#include "json/json.h"

namespace xstream {

int ContoursMethod::Init(const std::string &config_file_path) {
  LOGI << "Init " << config_file_path << std::endl;
  std::ifstream config_ifs(config_file_path);
  if (!config_ifs.good()) {
    LOGF << "open config file failed.";
    return -1;
  }
  Json::Value config_jv;
  config_ifs >> config_jv;
  contours_max_ratio_ = config_jv["contours_max_ratio"].asFloat();
  return 0;
}

std::vector<BaseDataPtr> ContoursMethod::DoProcess(
    const std::vector<BaseDataPtr> &input,
    const xstream::InputParamPtr &param) {
  LOGD << "input's size: " << input.size();
  // input[0] is body_box
  // input[1] is body_mask
  // output[0] is body_contours

  std::vector<BaseDataPtr> output;
  output.resize(1);
  if (input.size() != 2) {
    LOGF << "contours method input size is error";
  }

  auto body_rois = std::static_pointer_cast<BaseDataVector>(input[0]);
  auto body_masks = std::static_pointer_cast<BaseDataVector>(input[1]);
  if (body_rois->datas_.size() != body_masks->datas_.size()) {
    LOGF << "contours method input body roi and mask size error";
  }

  auto output_contours = std::make_shared<BaseDataVector>();
  output[0] = output_contours;

  for (size_t idx = 0; idx < body_rois->datas_.size(); ++idx) {
    auto one_mask = std::static_pointer_cast<
            xstream::Segmentation>(body_masks->datas_[idx]);
    auto one_box =\
      std::static_pointer_cast<xstream::BBox>(body_rois->datas_[idx]);
    auto contours_points = std::make_shared<xstream::FloatPoints>();
    output_contours->datas_.push_back(contours_points);
    if (one_box->state_ != DataState::VALID
        || one_mask->state_ != DataState::VALID) {
      contours_points->state_ = DataState::INVALID;
      continue;
    }

    // body mask is square
    int mask_size = sqrt(one_mask->values_.size());
    cv::Mat mask_mat(mask_size, mask_size, CV_32F);
    for (int h = 0; h < mask_size; ++h) {
      float *ptr = mask_mat.ptr<float>(h);
      for (int w = 0; w < mask_size; ++w) {
        *(ptr + w) = (one_mask->values_)[h * mask_size + w];
      }
    }

    int x1 = one_box->x1_;
    int y1 = one_box->y1_;
    int x2 = one_box->x2_;
    int y2 = one_box->y2_;
    int width = x2 - x1;
    int height = y2 - y1;
    float ratio = 1;
    float w_ratio = static_cast<float>(width) / mask_size;
    float h_ratio = static_cast<float>(height) / mask_size;
    if (w_ratio >= 4 && h_ratio >= 4) {
      ratio = w_ratio < h_ratio ? w_ratio : h_ratio;
      if (ratio >= contours_max_ratio_) {
        // when the ratio is bigger than contours_max_ratio_,
        // then mask will enarge to max ratio
        // to reduce the amount of calculation
        ratio = contours_max_ratio_;
      }
      width = width / ratio;
      height = height / ratio;
    }
    cv::resize(mask_mat, mask_mat, cv::Size(width, height));
    cv::Mat mask_gray(height, width, CV_8UC1);
    mask_gray.setTo(0);
    std::vector<std::vector<cv::Point>> contours;
    for (int h = 0; h < height; ++h) {
      uchar *p_gray = mask_gray.ptr<uchar>(h);
      const float *p_mask = mask_mat.ptr<float>(h);
      for (int w = 0; w < width; ++w) {
        if (p_mask[w] > 0) {
          // the pixel is belongs to body
          p_gray[w] = 1;
        } else {
        }
      }
    }
    mask_mat.release();
    cv::findContours(mask_gray, contours, cv::noArray(), cv::RETR_CCOMP,
                  cv::CHAIN_APPROX_NONE);
    mask_gray.release();
    for (size_t i = 0; i < contours.size(); i++) {
      auto one_line = contours[i];
      for (size_t j = 0; j < one_line.size(); j++) {
        contours_points->values_.push_back(\
          xstream::Point(contours[i][j].x * ratio + x1,
                         contours[i][j].y * ratio + y1,
                         0));
      }
    }
    contours.clear();
    std::vector<std::vector<cv::Point>>(contours).swap(contours);
  }
  return output;
}

}  // namespace xstream
