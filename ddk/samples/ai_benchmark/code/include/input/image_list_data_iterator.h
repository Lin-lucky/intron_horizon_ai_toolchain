// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _INPUT_IMAGE_LIST_ITERATOR_H_
#define _INPUT_IMAGE_LIST_ITERATOR_H_

#include <string>
#include <vector>

#include "data_iterator.h"
#include "utils/data_transformer.h"
#include "utils/image_cache.h"

class ImageListDataIterator : public DataIterator {
 public:
  ImageListDataIterator() : DataIterator("image_list_data_iterator") {}

  /**
   * Init Image Data iterator from file
   * @param[in] config_file: config file
   *        the file content should be in the json format
   *        for example:
   *        {
   *            "image_list_file" : "image_list.txt" #  one image file per line
   *            "width": 214,
   *            "height": 214,
   *            "transformers": "default_transformers"
   *            "data_type": 2
   *        }
   * transformers options: [mobilenetv1_transformers,
   *                         mobilenetv2_transformers,
   *                         resnet18_transformers,
   *                         resnet50_transformers,
   *                         googlenet_transformers,
   *                         efficientnet_transformers
   *                         yolov2_transformers,
   *                         yolov3_transformers,
   *                         yolov5_transformers,
   *                         default_transformers], and default_transformers fo
   * default
   * @param[in] config_string: config string
   *        same as config_file
   * @return 0 if success
   */
  int Init(std::string config_file,
           std::string config_string,
           hbDNNTensorLayout tensor_layout) override;

  /**
   * Release image_tensor
   * @param[in] image_tensor: image tensor to be released
   */
  void Release(ImageTensor *image_tensor) override;

  /**
   * Next Image Data read from file system
   * @param[out] image_tensor: image tensor
   * @return 0 if success
   */
  bool Next(ImageTensor *image_tensor) override;

  /**
   * Check if has next image
   * @return 0 if finish
   */
  bool HasNext() override;

  ~ImageListDataIterator() override;

 private:
  int LoadConfig(std::string &config_string) override;

 private:
  std::string image_list_file_;
  std::vector<std::string> image_files_;
  std::ifstream ifs_;
  int width_;
  int height_;
  bool cache_able_ = false;
  bool loop_able_ = true;
  int max_cache_size_ = 10;
  ImageCache *cache_ = nullptr;
  int send_index_ = 0;
  transformers_func transformers_;
  hbDNNDataType data_type_ = HB_DNN_IMG_TYPE_YUV444;
};

#endif  // _INPUT_IMAGE_LIST_ITERATOR_H_
