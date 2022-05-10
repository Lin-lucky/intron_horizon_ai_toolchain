/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @file util.h
 * @brief image conversion to xstream struct declaration
 * @author ruoting.ding
 * @email ruoting.ding@horizon.ai
 * @date 2019/4/29
 */

#ifndef INCLUDE_HORIZON_VISION_UTIL_H_
#define INCLUDE_HORIZON_VISION_UTIL_H_
#include <cstdint>
#include <memory>

#include "smart_plugin/convert.h"
#include "vision/vision_type.h"
#include "vision/vision_type.hpp"
#include "xstream/xstream_world.h"

namespace solution {
namespace video_box {

using ImageFramePtr = std::shared_ptr<hobot::vision::ImageFrame>;
using XStreamImageFramePtr = solution::video_box::XStreamData<ImageFramePtr>;
using XStreamBaseDataVectorPtr = std::shared_ptr<xstream::BaseDataVector>;

HorizonVisionImage *ImageConversion(uint8_t *data, int data_size, int width,
                                    int height, int stride, int stride_uv,
                                    HorizonVisionPixelFormat format);

HorizonVisionImage *ImageConversion(const ImageFramePtr &cpp_img);
XStreamImageFramePtr *ImageConversion(const HorizonVisionImage &c_img);
XStreamImageFramePtr *ImageFrameConversion(const HorizonVisionImageFrame *);

}  // namespace video_box
}  // namespace solution

#endif  //  INCLUDE_HORIZON_VISION_UTIL_H_
