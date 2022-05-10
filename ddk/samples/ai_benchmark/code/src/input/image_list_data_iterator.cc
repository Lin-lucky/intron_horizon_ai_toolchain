// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "input/image_list_data_iterator.h"

#include <iostream>

#include "glog/logging.h"
#include "rapidjson/document.h"
#include "utils/data_transformer.h"
#include "utils/stop_watch.h"
#include "utils/tensor_utils.h"
#include "utils/utils.h"

DEFINE_AND_REGISTER_DATA_ITERATOR(image, ImageListDataIterator)

/**
 * parse image list from config file
 * @param[in] list_file image list config file
 * @param[out] files list of images
 * @return
 */
static bool ParseImageList(const std::string &list_file,
                           std::vector<std::string> &files) {
  std::ifstream lst_ifstream(list_file);
  if (!lst_ifstream) {
    VLOG(EXAMPLE_SYSTEM) << "open image list file : " << list_file
                         << " failed!";
    return false;
  }

  std::string line;
  while (std::getline(lst_ifstream, line)) {
    std::istringstream ss(line);
    std::string temp;
    std::getline(ss, temp, ' ');
    files.push_back(std::move(temp));
  }
  return true;
}

int ImageListDataIterator::Init(std::string config_file,
                                std::string config_string,
                                hbDNNTensorLayout tensor_layout) {
  int ret_code = DataIterator::Init(config_file, config_string, tensor_layout);
  if (ret_code != 0) {
    return -1;
  }
  if (data_type_ > HB_DNN_IMG_TYPE_BGR) {
    VLOG(EXAMPLE_SYSTEM) << "ImageListDataIterator only support image type, "
                            "range: [0, 5], given: "
                         << data_type_;
    return -1;
  }
  ifs_.open(image_list_file_, std::ios::in);
  if (!ifs_.is_open()) {
    VLOG(EXAMPLE_SYSTEM) << "Open " << image_list_file_ << " failed";
    return -1;
  }

  if (cache_able_) {
    hbDNNTensor tensor;
    prepare_image_tensor(height_, width_, data_type_, tensor_layout, &tensor);
    cache_ = ImageCache::GetIns();
    cache_->SetCacheSize(max_cache_size_);
    while (!ifs_.eof()) {
      std::string image_file;
      ifs_ >> image_file;
      if (image_file.empty()) continue;
      VLOG(EXAMPLE_DEBUG) << "cache the file " << image_file;
      read_image_cache(image_file, &tensor, cache_);
    }

    release_tensor(&tensor);

    // reopen the file
    if (ifs_.is_open()) {
      ifs_.close();
    }

    // clear the ifs
    ifs_.clear(std::ios::goodbit);

    // read again
    ifs_.open(image_list_file_, std::ios::in);
    if (!ifs_.is_open()) {
      VLOG(EXAMPLE_SYSTEM) << "Open " << image_list_file_ << " failed";
      return false;
    }
  }
  if (!ParseImageList(image_list_file_, image_files_)) {
    VLOG(EXAMPLE_SYSTEM) << "Parse image list error!";
  }

  return 0;
}

bool ImageListDataIterator::Next(ImageTensor *image_tensor) {
  std::string image_file;

  if (cache_able_) {
    if (send_index_ >= cache_->RealCacheSize()) {
      is_finish_ = true;
      return false;
    }
  } else {
    if (send_index_ >= image_files_.size()) {
      is_finish_ = true;
      return false;
    }
  }

  image_file = image_files_[send_index_++];

  //  loop send if configured
  if (loop_able_) {
    if (send_index_ >= image_files_.size()) {
      send_index_ = 0;
    }

    if (cache_able_ && send_index_ >= cache_->RealCacheSize()) {
      send_index_ = 0;
    }
  }

  if (image_file.empty()) {
    return false;
  }

  auto &tensor = image_tensor->tensor;
  image_tensor->ori_image_path = image_file;
  prepare_image_tensor(height_, width_, data_type_, tensor_layout_, &tensor);
  read_image_tensor(image_file, image_tensor, &tensor, transformers_, cache_);
  image_tensor->image_name = get_file_name(image_file);
  image_tensor->frame_id = NextFrameId();
  flush_tensor(&tensor);
  return true;
}

void ImageListDataIterator::Release(ImageTensor *image_tensor) {
  release_tensor(&image_tensor->tensor);
}

int ImageListDataIterator::LoadConfig(std::string &config_string) {
  rapidjson::Document document;
  document.Parse(config_string.data());

  if (document.HasParseError()) {
    VLOG(EXAMPLE_SYSTEM) << "Parsing config file failed";
    return -1;
  }

  if (document.HasMember("image_list_file")) {
    image_list_file_ = document["image_list_file"].GetString();
  }

  if (document.HasMember("width")) {
    width_ = document["width"].GetInt();
  }

  if (document.HasMember("height")) {
    height_ = document["height"].GetInt();
  }

  if (document.HasMember("need_pre_load")) {
    cache_able_ = document["need_pre_load"].GetBool();
  }

  if (document.HasMember("need_loop")) {
    loop_able_ = document["need_loop"].GetBool();
  }

  if (document.HasMember("max_cache")) {
    max_cache_size_ = document["max_cache"].GetInt();
  }

  if (document.HasMember("data_type")) {
    data_type_ = static_cast<hbDNNDataType>(document["data_type"].GetInt());
  }

  std::string transformers = "default_transformers";
  if (document.HasMember("transformers")) {
    transformers = document["transformers"].GetString();
  }
  transformers_ = get_transformers(transformers);
  if (!transformers_) {
    VLOG(EXAMPLE_SYSTEM) << "Get transformers: " << transformers << " failed";
    return -1;
  }

  return 0;
}

ImageListDataIterator::~ImageListDataIterator() {
  if (ifs_.is_open()) {
    ifs_.close();
  }
}

bool ImageListDataIterator::HasNext() { return !is_finish_; }
