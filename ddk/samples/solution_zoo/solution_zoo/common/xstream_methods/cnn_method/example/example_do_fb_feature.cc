//
// Created by yaoyao.sun on 2019-04-29.
// Copyright (c) 2019 Horizon Robotics. All rights reserved.
//

#include <assert.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "cnn_method/cnn_method.h"
#include "cnn_method/util/util.h"
#include "hobotlog/hobotlog.hpp"
#include "hobotxsdk/xstream_sdk.h"
#include "image_utils.h"
#include "opencv2/opencv.hpp"
#include "xstream/vision_type.h"

typedef std::shared_ptr<xstream::ImageFrame> ImageFramePtr;

static void Usage() {
  std::cout << "./example do_fb_feature " <<
  "xstream_cfg_file img_lmk_list out_file\n";
}

using xstream::CNNMethod;

using xstream::BaseData;
using xstream::BaseDataPtr;
using xstream::BaseDataVector;
using xstream::InputData;
using xstream::InputDataPtr;

using xstream::BBox;
using xstream::RawDataImageFrame;
using xstream::Landmarks;
using xstream::Point;
using xstream::Points;
using xstream::SnapshotInfo;

template <typename DType>
struct LmkSnapInfo : SnapshotInfo<DType> {
  Points PointsToSnap(const Points &in) { return in; }
};

void PrintFaceFeature(const std::vector<xstream::BaseDataPtr> &result,
                      std::ostream &output) {
  auto face_feature =
      std::static_pointer_cast<xstream::BaseDataVector>(result[0]);
  auto target_size = face_feature->datas_.size();
  for (size_t target_idx = 0; target_idx < target_size; target_idx++) {
    auto features = std::static_pointer_cast<xstream::BaseDataVector>(
        face_feature->datas_[target_idx]);
    for (size_t snap_idx = 0; snap_idx < features->datas_.size(); snap_idx++) {
      auto feature = std::static_pointer_cast<xstream::FloatFeature>(
          features->datas_[target_idx]);
      static int feature_size = 128;
      if (feature->state_ != xstream::DataState::VALID) {
        for (int i = 0; i < feature_size; i++) {
          output << " " << std::fixed << std::setprecision(5) << -1.0f;
        }
      } else {
        for (int i = 0; i < feature_size; i++) {
          output << " " << std::fixed << std::setprecision(5)
                 << feature->value.values[i];
        }
      }
    }
  }
  output << std::endl;
}

int DoFbFeature(int argc, char **argv) {
  if (argc < 4) {
    Usage();
    return -1;
  }
  std::string cfg_file = argv[1];
  std::string img_list = argv[2];
  std::string output_file = argv[3];

  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();
  flow->SetConfig("config_file", cfg_file.c_str());
  flow->SetConfig("profiler", "on");
  flow->SetConfig("profiler_file", "./profiler.txt");
  flow->Init();

  std::ifstream ifs_img_list(img_list);
  if (!ifs_img_list.is_open()) {
    LOGD << "open image list file failed." << std::endl;
    return -1;
  }

  std::ofstream output(output_file, std::ios::out);

  std::string gt_data;
  std::string input_image;
  Landmarks landmark;
  landmark.values.resize(5);
  while (getline(ifs_img_list, gt_data)) {
    std::istringstream gt(gt_data);
    gt >> input_image;
    for (auto &point : landmark.values_) {
      gt >> point.x_ >> point.y_;
      LOGD << "x: " << point.x_ << " y: " << point.y_;
    }

    auto img_bgr = cv::imread(input_image);
    int width = img_bgr.cols;
    int height = img_bgr.rows;
    LOGD << "origin image size, width: " << img_bgr.cols
         << ", height: " << img_bgr.rows << std::endl;

    cv::Mat yuv_420(height * 3 / 2, width, CV_8UC1);
    cv::cvtColor(img_bgr, yuv_420, CV_BGR2YUV_I420);
    uint8_t *output_data = nullptr;
    int output_size, output_1_stride, output_2_stride;
    HobotXStreamConvertImage(yuv_420.data,
                          height * width * 3 / 2,
                          width, height,
                          width, width / 2,
                          IMAGE_TOOLS_RAW_YUV_I420,
                          IMAGE_TOOLS_RAW_YUV_NV12,
                          &output_data, &output_size,
                          &output_1_stride,
                          &output_2_stride);

    cv::Mat img_nv12 = cv::Mat(height * 3 / 2, width, CV_8UC1);
    memcpy(img_nv12.data, output_data, output_size);
    HobotXStreamFreeImage(output_data);

    auto face_img = std::make_shared<RawDataImageFrame>();

    face_img->img = img_nv12;
    face_img->pixel_format =
        HorizonVisionPixelFormat::kHorizonVisionPixelFormatRawNV12;

    auto snap_shot_info = std::make_shared<LmkSnapInfo<BaseDataPtr>>();
    snap_shot_info->userdata_.resize(2);
    snap_shot_info->userdata_[1] = landmark;
    snap_shot_info->snap_ = face_img;

    auto p_persons = std::make_shared<BaseDataVector>();
    auto p_one_person = std::make_shared<BaseDataVector>();
    p_persons->datas_.push_back(p_one_person);
    p_persons->name_ = "snap_list";
    p_one_person->datas_.push_back(snap_shot_info);

    xstream::InputDataPtr inputdata(new xstream::InputData());
    inputdata->datas_.push_back(
        std::static_pointer_cast<xstream::BaseData>(p_persons));
    auto out = flow->SyncPredict(inputdata);
    std::vector<std::string> dirs;
    xstream::split_string(input_image, dirs, "/");
    std::string track_id = dirs.size() >= 2 ? dirs[dirs.size() - 2] : dirs[0];
    output << track_id;
    PrintFaceFeature(out->datas_, output);
  }
  delete flow;
  return 0;
}
