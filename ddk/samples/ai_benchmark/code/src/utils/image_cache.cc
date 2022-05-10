// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

/*
 *  Copyright (c) 2020 by Horizon
 * \file image_cache.cc
 * \brief
 */

#include "utils/image_cache.h"

ImageCache* ImageCache::GetIns() {
  static ImageCache* cache = nullptr;
  if (cache == nullptr) {
    cache = new ImageCache();
  }

  return cache;
}

void ImageCache::SetCacheSize(uint32_t size) {
  cache_size_ = size;
  auto real_size = imageCache_.size();

  // need remove some image
  if (cache_size_ < real_size) {
    auto rm_size = real_size - cache_size_;
    for (auto&& image : imageCache_) {
      imageCache_.erase(image.first);
    }
  }
}

int ImageCache::RealCacheSize() { return imageCache_.size(); }

bool ImageCache::AddImage(std::string& file_name,
                          uint8_t* image,
                          int width,
                          int height,
                          int ori_widht,
                          int ori_height) {
  auto real_size = imageCache_.size();
  if (real_size < cache_size_) {
    ImageInfo image_info(image, width, height, ori_widht, ori_height);
    imageCache_.insert(std::make_pair(file_name, image_info));
    return true;
  }

  return false;
}

bool ImageCache::FindImage(std::string& file_name,
                           uint8_t** buffer,
                           int& ori_width,
                           int& ori_height) {
  if (imageCache_.find(file_name) != imageCache_.end()) {
    *buffer = imageCache_[file_name].buffer_;
    ori_width = imageCache_[file_name].ori_width_;
    ori_height = imageCache_[file_name].ori_height_;
    return true;
  }

  return false;
}
