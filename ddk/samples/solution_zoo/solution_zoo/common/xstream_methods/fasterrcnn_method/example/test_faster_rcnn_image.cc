/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @author    yaoyao.sun
 * @date      2019.04.12
 */

#include <assert.h>
#include <stdlib.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "fasterrcnn_method/dump.h"
#include "fasterrcnn_method/fasterrcnn_method.h"
#include "fasterrcnn_method/yuv_utils.h"
#include "image_utils.h"
#include "opencv2/opencv.hpp"
#include "xstream/vision_type.h"
#include "xstream/xstream_sdk.h"

using xstream::BaseData;
using xstream::BaseDataPtr;
using xstream::BaseDataVector;
using xstream::InputData;
using xstream::InputDataPtr;
using xstream::InputParamPtr;

using xstream::FasterRCNNMethod;
using xstream::ImageFrame;
using xstream::RawDataImageFrame;

static void Usage() {
  std::cout << "./FasterRCNNMethod_example faster_rcnn_image "
               "config_file img_list_file\n";
}

int TestFasterRCNNImage(int argc, char **argv) {
  if (argc < 3) {
    Usage();
    return -1;
  }
  std::string img_list = argv[2];

  FasterRCNNMethod faster_rcnn_method;
  std::string config_file = argv[1];
  faster_rcnn_method.Init(config_file);
  auto faster_rcnn_param =
    std::make_shared<xstream::FasterRCNNParam>("FasterRCNNMethod");
  faster_rcnn_param->max_face_count = 0;
  faster_rcnn_method.UpdateParameter(faster_rcnn_param);

  std::cout << "fasterrcnn init success." << std::endl;

  std::ifstream ifs(img_list);
  if (!ifs.is_open()) {
    std::cout << "open image list file failed." << std::endl;
    return -1;
  }
  static int64_t frame_id = 0;
  std::string input_image;
  while (getline(ifs, input_image)) {
    auto img_bgr = cv::imread(input_image);
    std::cout << "origin image size, width: " << img_bgr.cols
              << ", height: " << img_bgr.rows << std::endl;
    auto cv_image_frame_ptr = std::make_shared<xstream::RawDataImageFrame>();
    auto ret = HobotXStreamAllocImage(img_bgr.total() * img_bgr.step[1],
                                      &cv_image_frame_ptr->data_);
    if (ret != 0) {
      return -1;
    }
    memcpy(cv_image_frame_ptr->data_, img_bgr.data,
           img_bgr.total() * img_bgr.step[1]);
    cv_image_frame_ptr->width_ = img_bgr.cols;
    cv_image_frame_ptr->height_ = img_bgr.rows;
    cv_image_frame_ptr->pixel_format_ =
        xstream::HorizonVisionPixelFormat::kHorizonVisionPixelFormatRawBGR;

    std::vector<BaseDataPtr> input;
    xstream::InputParamPtr param;
    input.push_back(cv_image_frame_ptr);
    std::vector<BaseDataPtr> xstream_output =
        faster_rcnn_method.DoProcess(input, param);
    std::cout << "predict success: " << frame_id++ << std::endl;
    HobotXStreamFreeImage(cv_image_frame_ptr->data_);
  }
  faster_rcnn_method.Finalize();
  std::this_thread::sleep_for(std::chrono::seconds(10));
  return 0;
}
