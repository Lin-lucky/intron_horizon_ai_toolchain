// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _INPUT_PREPROCESSED_IMAGE_ITERATOR_H_
#define _INPUT_PREPROCESSED_IMAGE_ITERATOR_H_

#include <string>

#include "data_iterator.h"

class PreprocessedImageIterator : public DataIterator {
 public:
  PreprocessedImageIterator() : DataIterator("preprocessed_data_iterator") {}

  /**
   * Init Image Data iterator from file
   * @param[in] config_file: config file
   *        config file should be in the json format
   *        for example:
   *        {
   *            "image_list_file": "yuv_list.txt",
   *            "data_type": 2,
   *        }
   *        the `image_list_file` should be one data file per line
   *        and the data file name should be in for format of
   *        {name}_{org_h}_{org_w}_{result_h}_{result_w}.bin
   * @param[in] config_string: config string
   *        same as config file
   * @return 0 if success
   */
  int Init(std::string config_file,
           std::string config_string,
           hbDNNTensorLayout tensor_layout) override;

  /**
   * Next Image Data read from file system
   * @param[out] image_tensor: image tensor
   * @return 0 if success
   */
  bool Next(ImageTensor *image_tensor) override;

  /**
   * Release image_tensor
   * @param[in] image_tensor: image tensor to be released
   */
  void Release(ImageTensor *image_tensor) override;

  /**
   * Check if has next image
   * @return 0 if finish
   */
  bool HasNext() override;

  ~PreprocessedImageIterator() override;

 private:
  static void ParsePathParams(std::string &input_file,
                              std::string &image_name,
                              int &org_h,
                              int &org_w,
                              int &dst_h,
                              int &dst_w);

  int LoadConfig(std::string &config_string) override;

 private:
  std::string image_list_file_;
  std::ifstream ifs_;
  hbDNNDataType data_type_ = HB_DNN_IMG_TYPE_YUV444;
};

#endif  // _INPUT_PREPROCESSED_IMAGE_ITERATOR_H_
