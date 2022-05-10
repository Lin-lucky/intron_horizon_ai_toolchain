// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "output/image_list_output.h"

#include <fstream>

#include "glog/logging.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/opencv.hpp"
#include "rapidjson/document.h"
#include "utils/image_utils.h"

DEFINE_AND_REGISTER_OUTPUT(image, ImageListOutputModule)

int ImageListOutputModule::Init(std::string config_file,
                                std::string config_string) {
  int ret_code = OutputModule::Init(config_file, config_string);
  if (ret_code != 0) {
    return -1;
  }

  return 0;
}

int ImageListOutputModule::Init(rapidjson::Document &document) { return 0; }

void ImageListOutputModule::Write(ImageTensor *frame, Perception *perception) {
  cv::Mat mat;
  std::fstream output_dir_fst;
  output_dir_fst.open(image_output_dir_, std::ios::in);
  if (output_dir_fst && draw_perception(frame, perception, mat) == 0) {
    std::stringstream ss;
    ss << image_output_dir_ << "/" << frame->frame_id;
    if (frame->image_name.empty()) {
      ss << ".jpg";
    } else {
      ss << '_' << frame->image_name;
    }
    cv::imwrite(ss.str(), mat);
    image_counter_++;
  }
}

int ImageListOutputModule::LoadConfig(std::string &config_string) {
  rapidjson::Document document;
  document.Parse(config_string.data());

  if (document.HasParseError()) {
    VLOG(EXAMPLE_SYSTEM) << "Parsing config file failed";
    return -1;
  }

  if (document.HasMember("image_output_dir")) {
    image_output_dir_ = document["image_output_dir"].GetString();
  }

  return 0;
}
