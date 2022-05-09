/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @brief     provides xstream image Rotater
 * @author    zhengzheng.ge
 * @email     zhengzheng.ge@horizon.ai
 * @version   0.0.0.1
 * @date      2019.11.15
 */

#ifndef INCLUDE_IMAGEUTILS_IMAGEUTILS_ROTATER_H_
#define INCLUDE_IMAGEUTILS_IMAGEUTILS_ROTATER_H_

#include <set>
#include "image_utils/image_utils/common.h"
#include "image_utils/image_utils/base.h"

namespace xstream {

class ImageRotater : public ImageBase {
 public:
  bool Rotate(const ImageToolsFormatData &input,
              const int degree,
              ImageToolsFormatData &output);
 private:
  bool RotateI420();
  bool RotateNV();
  bool RotateGray();
  bool RotateRGB();

 private:
  int degree_ = 0;
  static const std::set<int> degree_set_;
};
}  // namespace xstream

#endif  // INCLUDE_IMAGEUTILS_IMAGEUTILS_ROTATER_H_
