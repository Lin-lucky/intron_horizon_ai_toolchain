/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     FasterRCNN Method
 * @author    ruoting.ding
 * @email     ruoting.ding@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.21
 */

#ifndef INCLUDE_FASTERRCNNMETHOD_DUMP_H_
#define INCLUDE_FASTERRCNNMETHOD_DUMP_H_

#include <memory>
#include <string>
#include <vector>

#include "xstream/vision_type.h"
#include "opencv2/highgui/highgui.hpp"
#include "xstream/xstream_sdk.h"
#include "json/json.h"


using xstream::BaseDataPtr;
using xstream::BaseDataVector;

using xstream::BBox;

namespace faster_rcnn_method {

void ShowBBox(cv::Mat frame, const BBox &bbox,
              const cv::Scalar &color = cv::Scalar(0, 0, 255));

void DumpSkeleton(cv::Mat &frame,
                  const std::shared_ptr<BaseDataVector> &skeletons);

void DumpLandmark(cv::Mat &frame,
                  const std::shared_ptr<BaseDataVector> &landmarks);

void DumpMask(cv::Mat &frame, const std::shared_ptr<BaseDataVector> &masks,
              const std::shared_ptr<BaseDataVector> &body_boxes);

void DumpRects(cv::Mat &frame, const std::shared_ptr<BaseDataVector> &rects,
               int class_id);

void DumpRectsWithInfo(cv::Mat &frame,
                       const std::shared_ptr<BaseDataVector> &rects,
                       int class_id, std::string image_name);

void DumpPlateWithInfo(cv::Mat &frame,
                       const std::shared_ptr<BaseDataVector> &plate_color,
                       const std::shared_ptr<BaseDataVector> &plate_row,
                       int class_id, std::string image_name);

}  // namespace faster_rcnn_method

#endif  // INCLUDE_FASTERRCNNMETHOD_DUMP_H_
