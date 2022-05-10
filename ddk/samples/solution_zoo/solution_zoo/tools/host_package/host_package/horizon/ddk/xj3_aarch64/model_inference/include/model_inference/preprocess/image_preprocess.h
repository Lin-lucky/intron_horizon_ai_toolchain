/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of image_preprocess
 * @file   image_preprocess.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */
#ifndef IMAGE_PREPROCESS_H_
#define IMAGE_PREPROCESS_H_

#include <vector>
#include <memory>
#include <string>
#include <cstring>
#include "model_inference/preprocess/preprocess.h"
#include "model_inference/preprocess/utils/image_process.h"
#include "model_inference/inference_engine.h"
#include "model_inference/inference_data.h"
#include "xstream/xstream_world.h"
#include "hobotlog/hobotlog.hpp"

namespace inference {

// 2、从内存读取一个原图，根据用户配置的image_process_pipeline对图像进行预处理
class ImageInputWithPreProcess : public PreProcess {
 private:
  ImageProcess img_proc_;

 public:
  explicit ImageInputWithPreProcess(Inferencer* infer) : PreProcess(infer) {}
  virtual ~ImageInputWithPreProcess() {}

  virtual int Init(const std::string &json_str) {
    // string转json
    Json::Reader Reader;
    Json::Value config;
    Reader.parse(json_str, config);
    if (!config["config"]["image_process_pipeline"].isNull()) {
      img_proc_.Init(
          config["config"]["image_process_pipeline"].toStyledString());
    }
    return 0;
  }

  virtual int Execute(const std::vector<xstream::BaseDataPtr> &input,
                      std::vector<std::shared_ptr<InferenceEngineTask>>& tasks);
};

}  // namespace inference

#endif  // IMAGE_PREPROCESS_H_
