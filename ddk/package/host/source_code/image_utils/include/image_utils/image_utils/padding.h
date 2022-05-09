/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     provides xstream image Padding definition
 * @author    hangjun.yang
 * @email     hangjun.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.15
 */

#ifndef INCLUDE_IMAGEUTILS_IMAGEUTILS_PADDING_H_
#define INCLUDE_IMAGEUTILS_IMAGEUTILS_PADDING_H_

#include "image_utils/image_utils/common.h"
#include "image_utils/image_utils/base.h"

namespace xstream {

class ImagePadding : public ImageBase {
 public:
  bool Pad(const ImageToolsFormatData &input,
           const int padding_left_width,
           const int padding_right_width,
           const int padding_top_height,
           const int padding_bottom_height,
           const uint8_t* padding_value,
           ImageToolsFormatData &output);

 private:
  // pad RGB/BGR/GRAY
  bool PadOnePlane();

  // pad I420
  bool PadI420();

  // pad nv12/nv21
  bool PadNV();

  // pad one row.
  void PadRow(uint8_t *data,
              const int element_size,
              const int width,
              const uint8_t *padding_value);

 private:
  HobotXStreamImageToolsResizeInfo resize_info_;
  int padding_left_width_;
  int padding_right_width_;
  int padding_top_height_;
  int padding_bottom_height_;
  const uint8_t* padding_value_;
};
}  // namespace xstream

#endif  // INCLUDE_IMAGEUTILS_IMAGEUTILS_PADDING_H_
