/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: rect_pyd_verification.cpp
 * @Brief: conformance validation of rect+pyramid
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-05-22 14:27:05
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-05-22 15:18:10
 */

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "./plat_cnn.h"
#include "CNNMethod/Predictor/Predictor.h"
#include "CNNMethod/Predictor/PredictorFactory.h"
#include "CNNMethod/util/CNNMethodConfig.h"
#include "CNNMethod/util/CNNMethodData.h"
#include "CNNMethod/util/util.h"
#include "hobotlog/hobotlog.hpp"
#include "xstream/method.h"
#include "xstream/xstream_data.h"
#include "xstream/xstream_sdk.h"
#ifdef X2
#include "./vio_wrapper.h"
#endif
#ifdef X3
#include "./vio_wrapper_global.h"
#endif
#include "xstream/vision_type.h"

typedef std::shared_ptr<xstream::ImageFrame> ImageFramePtr;
void DumpInvalidCommon(const std::vector<std::vector<uint32_t>> &real_nhwc,
                       std::ostream &os);
void DumpValidCommon(std::vector<std::vector<int8_t>> &target_rlt,
                     const std::vector<std::vector<uint32_t>> &real_nhwc,
                     std::ostream &os);
void DumpLmk(std::vector<std::vector<int8_t>> &mxnet_outs,
             const xstream::BBox &box,
             const std::vector<std::vector<uint32_t>> nhwc,
             std::ostream &os);
void DumpPose3D(std::vector<int8_t> &mxnet_outs, std::ostream &os);

static void Usage() {
  std::cout << "./example ver_rect_pyd method_cfg_file hb_vio_cfg_file gt.txt "
               "out_file"
            << std::endl;
}

void DumpMxnetOutput(xstream::CNNMethodRunData *run_data,
                     std::ostream &os,
                     const std::string &post_fn,
                     const std::string &img_path) {
  auto &result = run_data->mxnet_output;
  auto &real_nhwc = run_data->real_nhwc;

  {
    auto &frame_rlt = result;
    for (size_t target_idx = 0; target_idx < frame_rlt.size(); target_idx++) {
      auto &target_rlt = frame_rlt[target_idx];
      if (target_rlt.size()) {
        os << img_path;
        if (post_fn == "lmk_pose") {
          auto boxes = std::static_pointer_cast<xstream::BaseDataVector>(
              (*(run_data->input))[0]);
          auto xstream_box = std::static_pointer_cast<
              xstream::XStreamData<xstream::BBox>>(
              boxes->datas_[target_idx]);
          DumpLmk(target_rlt, xstream_box->value, real_nhwc, os);
          DumpPose3D(target_rlt[3], os);
        } else {
          DumpValidCommon(target_rlt, real_nhwc, os);
        }
        os << std::endl;
      }
    }
  }
}
void DumpValidCommon(std::vector<std::vector<int8_t>> &target_rlt,
                     const std::vector<std::vector<uint32_t>> &real_nhwc,
                     std::ostream &os) {
  for (size_t layer_idx = 0; layer_idx < target_rlt.size(); layer_idx++) {
    auto &layer_nhwc = real_nhwc[layer_idx];
    int elem_num =
        layer_nhwc[0] * layer_nhwc[1] * layer_nhwc[2] * layer_nhwc[3];
    auto layer_rlt = reinterpret_cast<float *>(target_rlt[layer_idx].data());
    for (int elem_idx = 0; elem_idx < elem_num; elem_idx++) {
      os << " " << std::fixed << std::setprecision(5) << layer_rlt[elem_idx];
    }
  }
}

void DumpInvalidCommon(const std::vector<std::vector<uint32_t>> &real_nhwc,
                       std::ostream &os) {
  for (size_t layer_idx = 0; layer_idx < real_nhwc.size(); layer_idx++) {
    auto &layer_nhwc = real_nhwc[layer_idx];
    int elem_num =
        layer_nhwc[0] * layer_nhwc[1] * layer_nhwc[2] * layer_nhwc[3];
    for (int elem_idx = 0; elem_idx < elem_num; elem_idx++) {
      os << " " << std::fixed << std::setprecision(5) << -1.0f;
    }
  }
}

void DumpLmk(std::vector<std::vector<int8_t>> &mxnet_outs,
             const xstream::BBox &box,
             const std::vector<std::vector<uint32_t>> nhwc,
             std::ostream &os) {
  static const float SCORE_THRESH = 0.0;
  static const float REGRESSION_RADIUS = 3.0;
  static const float STRIDE = 4.0;
  static const float num = 1;
  static const float height_m = 16;
  static const float width_m = 16;

  auto fl_scores = reinterpret_cast<float *>(mxnet_outs[0].data());
  auto fl_coords = reinterpret_cast<float *>(mxnet_outs[1].data());
  std::vector<std::vector<float>> points_score;
  std::vector<std::vector<float>> points_x;
  std::vector<std::vector<float>> points_y;
  points_score.resize(5);
  points_x.resize(5);
  points_y.resize(5);

  // nhwc, 1x16x16x5, 1x16x16x10
  for (int n = 0; n < num; ++n) {          // n
    for (int i = 0; i < height_m; ++i) {   // h
      for (int j = 0; j < width_m; ++j) {  // w
        int index_score = n * nhwc[0][1] * nhwc[0][2] * nhwc[0][3]
                          + i * nhwc[0][2] * nhwc[0][3] + j * nhwc[0][3];
        int index_coords = n * nhwc[1][1] * nhwc[1][2] * nhwc[0][3]
                           + i * nhwc[1][2] * nhwc[1][3] + j * nhwc[1][3];
        for (int k = 0; k < 5; ++k) {  // c
          auto score = fl_scores[index_score + k];
          if (score > SCORE_THRESH) {
            points_score[k].push_back(score);
            float x =
                (j + 0.5 - fl_coords[index_coords + 2 * k] * REGRESSION_RADIUS)
                * STRIDE;
            float y =
                (i + 0.5
                 - fl_coords[index_coords + 2 * k + 1] * REGRESSION_RADIUS)
                * STRIDE;
            x = std::min(std::max(x, 0.0f), width_m * STRIDE);
            y = std::min(std::max(y, 0.0f), height_m * STRIDE);
            points_x[k].push_back(x);
            points_y[k].push_back(y);
          }
        }
      }
    }
  }
  os << " " << box.x1 << " " << box.y1 << " " << box.x2 << " " << box.y2;
  std::vector<int> scores(5);
  for (int i = 0; i < 5; ++i) {
    float x = xstream::Mean(points_x[i]);
    float y = xstream::Mean(points_y[i]);
    x = box.x1 + x / 64 * (box.x2 - box.x1);
    y = box.y1 + y / 64 * (box.y2 - box.y1);
    scores[i] = static_cast<float>(points_score[i].size()) > 0.000001 ? 1 : 0;
    os << " " << std::fixed << std::setprecision(5) << x << " " << y;
  }
  for (int i = 0; i < 5; i++) {
    os << " " << scores[i];
  }
  /* lmks1 post process */
  std::vector<float> lmks1(10);
  auto reg_coords = reinterpret_cast<float *>(mxnet_outs[2].data());
  for (int i = 0; i < 5; i++) {
    lmks1[i << 1] = box.x1 + reg_coords[i << 1] * (box.x2 - box.x1);
    lmks1[(i << 1) + 1] = box.y1 + reg_coords[(i << 1) + 1] * (box.y2 - box.y1);
  }
  for (int i = 0; i < 10; i++) {
    os << " " << std::fixed << std::setprecision(5) << lmks1[i];
  }
}

void DumpPose3D(std::vector<int8_t> &mxnet_outs, std::ostream &os) {
  auto mxnet_out = reinterpret_cast<float *>(mxnet_outs.data());
  float yaw = mxnet_out[0] * 90.0;
  float pitch = mxnet_out[1] * 90.0;
  float roll = mxnet_out[2] * 90.0;
  os << " " << yaw << " " << pitch << " " << roll;
}

int DoVerRectPyd(int argc, char **argv) {
  if (argc < 5) {
    Usage();
    return -1;
  }
  std::string cfg_path = argv[1];
  std::string post_fn;
  std::ifstream infile(cfg_path.c_str());
  std::stringstream buffer;
  buffer << infile.rdbuf();
  auto config = std::shared_ptr<xstream::CNNMethodConfig>(
      new xstream::CNNMethodConfig(buffer.str()));
  config->SetSTDStringValue("parent_path",
                            xstream::get_parent_path(cfg_path));

  xstream::Predictor *predictor =
      xstream::PredictorFactory::GetPredictor(xstream::InputType::RECT);
  predictor->Init(config);
  post_fn = config->GetSTDStringValue("post_fn");

  std::string fb_cfg = argv[2];
  HbVioFbWrapperGlobal fb_handle(fb_cfg);
  fb_handle.Init();

  std::string gt_list = argv[3];
  std::string input_image;
  float x1, y1, x2, y2;

  std::ifstream f_rect(gt_list);
  std::string gt_line;
  std::string output_file = argv[4];
  std::ofstream output(output_file, std::ios::out);
  while (getline(f_rect, gt_line)) {
    std::istringstream gt(gt_line);
    gt >> input_image;
    // pyramid
    uint32_t effective_w, effective_h;
    auto py_img = fb_handle.GetImgInfo(
        input_image, &effective_w, &effective_h);
    gt >> x1 >> y1 >> x2 >> y2;
    if (x1 < 0 || x2 >= 1920 || y1 < 0 || y2 >= 1080) continue;
    // rect
    std::vector<std::vector<float>> rois;
    rois.resize(1);
    rois[0].push_back(x1);
    rois[0].push_back(y1);
    rois[0].push_back(x2);
    rois[0].push_back(y2);

    xstream::CNNMethodRunData run_data;
    std::vector<xstream::BaseDataPtr> input;

    auto xstream_rois = std::make_shared<xstream::BaseDataVector>();
    for (auto &roi : rois) {
      auto xstream_roi = std::make_shared<xstream::BBox>();
      xstream_roi->x1_ = roi[0];
      xstream_roi->y1_ = roi[1];
      xstream_roi->x2_ = roi[2];
      xstream_roi->y2_ = roi[3];
      xstream_rois->datas_.push_back(xstream_roi);
    }
    input.push_back(
        std::static_pointer_cast<xstream::BaseData>(xstream_rois));
    input.push_back(std::static_pointer_cast<xstream::BaseData>(py_img));
    run_data.input = &input;
    predictor->Do(&run_data);
    DumpMxnetOutput(&run_data, output, post_fn, input_image);
    fb_handle.FreeImgInfo(py_img);
  }
  delete predictor;
  return 0;
}
