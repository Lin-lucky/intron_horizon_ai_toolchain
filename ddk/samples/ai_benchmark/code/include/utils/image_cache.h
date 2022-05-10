// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _UTILS_IMAGE_CACHE_H_
#define _UTILS_IMAGE_CACHE_H_
#include <map>
#include <string>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"

class ImageCache {
 public:
  static ImageCache *GetIns();

  void SetCacheSize(uint32_t size);

  int RealCacheSize();

  bool AddImage(std::string &file_name,
                uint8_t *buffer,
                int width,
                int height,
                int ori_width,
                int ori_height);

  bool FindImage(std::string &file_name,
                 uint8_t **buffer,
                 int &ori_width,
                 int &ori_height);

  ImageCache(const ImageCache &) = delete;

  void operator=(const ImageCache &) = delete;

  ImageCache(ImageCache &&) = delete;

  ~ImageCache() = default;

 private:
  struct ImageInfo {
    ImageInfo() = default;
    ImageInfo(
        uint8_t *image, int width, int height, int ori_width, int ori_height)
        : buffer_(image),
          width_(width),
          height_(height),
          ori_width_(ori_width),
          ori_height_(ori_height) {}
    uint8_t *buffer_;
    int width_;
    int height_;
    int ori_width_;
    int ori_height_;

    ~ImageInfo() = default;
  };
  ImageCache() {}

  std::map<std::string, ImageInfo> imageCache_;

  uint32_t cache_size_ = 10;
};

#endif  // _UTILS_IMAGE_CACHE_H_
