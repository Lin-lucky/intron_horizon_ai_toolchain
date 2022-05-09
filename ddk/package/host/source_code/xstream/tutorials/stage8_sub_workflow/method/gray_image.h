/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @brief     bbox base data type only for example
 * @author    xudong.du
 * @email     xudong.du@horizon.ai
 * @version   0.0.0.1
 * @date      2020.10.29
 */

#ifndef XSTREAM_TUTORIALS_STAGE8_METHOD_GRAYIMAGE_H_
#define XSTREAM_TUTORIALS_STAGE8_METHOD_GRAYIMAGE_H_

#include <memory>
#include <string>
#include <vector>

#include "xstream/xstream_world.h"
namespace xstream {

struct GrayImage : public BaseData {
  inline GrayImage() {}
  inline GrayImage(int width_, int height_, std::vector<int8_t> image_data_) {
    width = width_;
    height = height_;
    image_data.assign(image_data_.begin(), image_data_.end());
  }
  inline int Width() const { return width; }
  inline int Height() const { return height; }

  inline friend std::ostream &operator<<(std::ostream &out,
                                         GrayImage &gray_image) {
    out << "( width: " << gray_image.width << " height: " << gray_image.height;
    return out;
  }

  inline friend std::ostream &operator<<(std::ostream &out,
                                         const GrayImage &gray_image) {
    out << "( width: " << gray_image.width << " height: " << gray_image.height;
    return out;
  }

  std::vector<int8_t> image_data;
  int width;
  int height;
};

typedef std::shared_ptr<GrayImage> GrayImagePtr;

}  // namespace xstream

#endif  // XSTREAM_TUTORIALS_STAGE8_METHOD_GRAYIMAGE_H_
