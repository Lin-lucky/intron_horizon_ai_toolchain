//
// Created by yaoyao.sun on 2019-05-14.
// Copyright (c) 2019 Horizon Robotics. All rights reserved.
//

#include <gtest/gtest.h>
#include <stdlib.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "fasterrcnn_method/fasterrcnn_method.h"
#include "fasterrcnn_method/dump.h"
#include "fasterrcnn_method/yuv_utils.h"
#include "hobotlog/hobotlog.hpp"
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

TEST(FACE_DET_TEST, Basic) {
  SetLogLevel(HOBOT_LOG_DEBUG);
  FasterRCNNMethod faster_rcnn_method;
  std::string config_file = "./configs/multitask_config.json";
  faster_rcnn_method.Init(config_file);

  std::string img_list = "./test/data/image.list";
  std::ifstream ifs(img_list);
  ASSERT_TRUE(ifs.is_open());

  std::string input_image;
  while (getline(ifs, input_image)) {
    auto img_bgr = cv::imread(input_image);
    std::cout << "origin image size, width: " << img_bgr.cols
              << ", height: " << img_bgr.rows << std::endl;
    auto image_frame_ptr = std::make_shared<RawDataImageFrame>();

    auto ret = HobotXStreamAllocImage(img_bgr.total() * img_bgr.step[1],
                                      &image_frame_ptr->data_);
    ASSERT_EQ(ret, static_cast<std::size_t>(0));
    memcpy(image_frame_ptr->data_, img_bgr.data,
           img_bgr.total() * img_bgr.step[1]);
    image_frame_ptr->width_ = img_bgr.cols;
    image_frame_ptr->height_ = img_bgr.rows;
    image_frame_ptr->channel_ = 3;
    image_frame_ptr->pixel_format_ =
        xstream::HorizonVisionPixelFormat::kHorizonVisionPixelFormatRawBGR;

    std::vector<BaseDataPtr> input;
    xstream::InputParamPtr param;
    input.push_back(image_frame_ptr);
    std::vector<BaseDataPtr> xstream_output =
      faster_rcnn_method.DoProcess(input, param);
    auto faster_rcnn_out = xstream_output;
    auto rects = std::static_pointer_cast<BaseDataVector>(faster_rcnn_out[0]);
    ASSERT_TRUE(rects->datas_.size() == 3);     // NOLINT
  }

  faster_rcnn_method.Finalize();
}
