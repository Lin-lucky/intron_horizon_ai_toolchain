/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of pyramid_roi_preprocess
 * @file   pyramid_roi_preprocess.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */
#ifndef PYRAMID_ROI_RESIZER_PREPROCESS_H_
#define PYRAMID_ROI_RESIZER_PREPROCESS_H_

#include <vector>
#include <string>
#include <memory>
#include "model_inference/preprocess/preprocess.h"
#include "model_inference/preprocess/utils/roi_process.h"
#include "model_inference/preprocess/utils/image_process.h"
#include "model_inference/inference_data.h"
#include "xstream/xstream_world.h"

namespace inference {

// 4、输入整个金字塔，并根据用户配置的roi_process_pipeline对图像进行预处理
class PyramidRoiResizerPreProcess : public PreProcess {
 private:
  RoiProcess roi_proc_;

 public:
  explicit PyramidRoiResizerPreProcess(Inferencer* infer) : PreProcess(infer) {}
  virtual ~PyramidRoiResizerPreProcess() {}

  virtual int Init(const std::string &json_str) {
    // string转json
    Json::Reader Reader;
    Json::Value config;
    int ret = 0;
    Reader.parse(json_str, config);
    if (!config["config"]["roi_process_pipeline"].isNull()) {
      ret = roi_proc_.Init(
          config["config"]["roi_process_pipeline"].toStyledString());
      if (ret != 0) {
        return ret;
      }
    }
    return 0;
  }

  virtual int Execute(
      const std::vector<xstream::BaseDataPtr> &input,
      std::vector<std::shared_ptr<InferenceEngineTask>>& tasks);
};

}  // namespace inference

#endif  // PYRAMID_ROI_RESIZER_PREPROCESS_H_
