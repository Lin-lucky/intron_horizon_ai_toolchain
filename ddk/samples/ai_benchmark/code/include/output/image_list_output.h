// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _IMAGE_LIST_OUTPUT_H_
#define _IMAGE_LIST_OUTPUT_H_

#include <string>

#include "base/perception_common.h"
#include "output.h"

class ImageListOutputModule : public OutputModule {
 public:
  ImageListOutputModule() : OutputModule("image_list_output") {}

  /**
   * Init ImageListOutputModule
   * @param[in] config_file: config file
   *        config file should be in the json format
   *        for example:
   *        {
   *            "image_output_dir": "image_out"
   *        }
   * @param[in] config_string: config string
   *        same as config file
   * @return 0 if success
   */
  int Init(std::string config_file, std::string config_string) override;

  int Init(rapidjson::Document &document) override;

  /**
   * Draw perception data on image the save it to file disk
   * @param[in] frame: frame info
   * @param[in] perception: perception data
   */
  void Write(ImageTensor *frame, Perception *perception) override;

  ~ImageListOutputModule() override = default;

 private:
  int LoadConfig(std::string &config_string) override;

 private:
  std::string image_output_dir_ = "output_images";
  int image_counter_ = 0;
};

#endif  // _IMAGE_LIST_OUTPUT_H_
