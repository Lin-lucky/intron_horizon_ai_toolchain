/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of pyramid_preprocess
 * @file   pyramid_preprocess.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */
#ifndef PYRAMID_PREPROCESS_H_
#define PYRAMID_PREPROCESS_H_

#include <vector>
#include <string>
#include <memory>
#include "model_inference/preprocess/preprocess.h"
#include "model_inference/preprocess/utils/image_process.h"
#include "model_inference/inference_data.h"
#include "xstream/xstream_world.h"

namespace inference {

// 1、从金字塔一层输入，不需要额外分配input tensor空间
// 直接将金字塔的图层地址赋值给tensor
// 3、从金字塔选择一层，根据用户配置的image_process_pipeline对图像进行预处理
class PyramidPreProcess : public PreProcess {
 private:
  ImageProcess img_proc_;
  int pyramid_layer_ = 0;

 public:
  explicit PyramidPreProcess(Inferencer* infer) : PreProcess(infer) {}
  virtual ~PyramidPreProcess() {}

  virtual int Init(const std::string &json_str) {
    // string转json
    Json::Reader Reader;
    Json::Value config;
    Reader.parse(json_str, config);
    pyramid_layer_ = config["pyramid_layer"].asInt();
    if (!config["config"]["image_process_pipeline"].isNull()) {
      img_proc_.Init(
          config["config"]["image_process_pipeline"].toStyledString());
    }
    return 0;
  }

  virtual int Execute(const std::vector<xstream::BaseDataPtr> &input,
                      std::vector<std::shared_ptr<InferenceEngineTask>>& tasks);

 private:
  // Tensor需要在此申请
  int Execute(std::shared_ptr<xstream::PyramidImageFrame> pyramid,
              const int pyramid_level,
              std::vector<Tensor> &input_tensors);
};

}  // namespace inference

#endif  // PYRAMID_PREPROCESS_H_
