/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of pyramid_roi_bpu_preprocess
 * @file   pyramid_roi_bpu_preprocess.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.05.21
 */
#ifndef PYRAMID_ROI_BPU_PREPROCESS_H_
#define PYRAMID_ROI_BPU_PREPROCESS_H_

#include <vector>
#include <string>
#include <memory>
#include "bpu_predict_extension.h"
#include "model_inference/preprocess/preprocess.h"
#include "model_inference/preprocess/utils/roi_process.h"
#include "model_inference/inference_data.h"
#include "xstream/xstream_world.h"

namespace inference {

// 输入金字塔全图和一系列rois，根据用户配置的roi_process_pipeline对rois预处理
// 取roi对应的图像后再crop&&resize处理
class PyramidRoiBPUPreProcess : public PreProcess {
 private:
  RoiProcess roi_proc_;
  int bpu_core_ = 2;

 public:
  explicit PyramidRoiBPUPreProcess(Inferencer* infer) : PreProcess(infer) {}
  virtual ~PyramidRoiBPUPreProcess() {}

  virtual int Init(const std::string &json_str);

  virtual int Execute(
      const std::vector<xstream::BaseDataPtr> &input,
      std::vector<std::shared_ptr<InferenceEngineTask>>& tasks);

  void ConvertTensor2BPUTensor(
    const Tensor &tensor, BPU_TENSOR_S &bpu_tensor);
};

}  // namespace inference

#endif  // PYRAMID_ROI_BPU_PREPROCESS_H_
