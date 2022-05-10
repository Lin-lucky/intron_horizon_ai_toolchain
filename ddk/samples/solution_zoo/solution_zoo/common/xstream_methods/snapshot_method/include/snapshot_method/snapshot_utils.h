/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     image_utils header
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.16
 * @date      2019.04.22
 */

#ifndef COMMON_XSTREAM_METHODS_SNAPSHOT_METHOD_\
INCLUDE_SNAPSHOT_METHOD_SNAPSHOT_UTILS_H_
#define COMMON_XSTREAM_METHODS_SNAPSHOT_METHOD_\
INCLUDE_SNAPSHOT_METHOD_SNAPSHOT_UTILS_H_
#include <memory>

#include "image_utils.h"
#include "xstream/vision_type.h"
#include "opencv2/opencv.hpp"

namespace xstream {

using xstream::BBox;
using xstream::Point;
using xstream::FloatPoints;
typedef std::shared_ptr<xstream::ImageFrame> ImageFramePtr;

enum class FotmatDataArrayType {
  kContinueType,  // for BGR/RGB/GRAY, and for Y U V store continue
  kSeperateType  // for Y U V store seperately, e.g. vio returned NV12
};

class ImageUtils {
 public:
  static BBox AdjustSnapRect(const uint32_t &frame_width,
                             const uint32_t &frame_height, const BBox &in_bbox,
                             const float &scale);

  static int Data2CVImage(const uint32_t &height, const uint32_t &width,
                          const HorizonVisionPixelFormat &format, uint8_t *data,
                          cv::Mat &cv_img);

  static ImageFramePtr DoFaceCrop(const ImageFramePtr &frame,
                                  const BBox &crop_rect,
                                  const uint32_t &output_width,
                                  const uint32_t &output_height,
                                  const bool &need_resize = true);
};
}  // namespace xstream

#endif  // COMMON_XSTREAM_METHODS_SNAPSHOT_METHOD_INCLUDE_SNAPSHOT_METHOD_SNAPSHOT_UTILS_H_
