/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     provides xstream image Convertor
 * @author    hangjun.yang
 * @email     hangjun.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.10
 */
#ifndef INCLUDE_IMAGEUTILS_IMAGEUTILS_CONVERTOR_H_
#define INCLUDE_IMAGEUTILS_IMAGEUTILS_CONVERTOR_H_
#include "image_utils/image_utils/common.h"
#include "image_utils/image_utils/base.h"
namespace xstream {

class ImageConvertor : public ImageBase {
 public:
  bool Convert(const ImageToolsFormatData &input,
               const HobotXStreamImageToolsPixelFormat dst_fmt,
               ImageToolsFormatData &output);

 private:
  // convert all(except gray) to rgb/bgr
  bool ConvertToRGBOrBGR();

  // convert all(except gray) to i420
  bool ConvertToI420();

  // convert RGB/BGR to gray
  bool ConvertRGBorBGRToGray();

  // convert I420/NV12/NV21 to gray
  bool ConvertYUV420ToGray();

  // convert I420 to NV12/NV21
  bool ConvertI420ToNV();

  // convert NV12 to NV21  or convert NV21 to NV12
  bool ConvertBetweenNV();

  // convert RGB/BGR to NV12/nv21
  bool ConvertRGBOrBGRToNV();

  // convert nv12 to yuv444
  bool ConvertNV12ToYUV444();

 private:
  int width_;
  int height_;
};

}  // namespace xstream

#endif  // INCLUDE_IMAGEUTILS_IMAGEUTILS_CONVERTOR_H_
