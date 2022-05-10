/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of inference_task
 * @file   inference_task.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */
#ifndef INFERENCER_H_
#define INFERENCER_H_

#include <vector>
#include <string>
#include <memory>
#include "inference_engine.h"
#include "preprocess/preprocess.h"
#include "postprocess/postprocess.h"
#include "xstream/xstream_world.h"
#include "json/json.h"
#include "hobotlog/hobotlog.hpp"

namespace inference {

class Inferencer : public xstream::BaseData {
 public:
  int Init(const std::string &cfg_path);

  void Finalize();

  // 操作：预处理和推理
  // 一帧可能对应多个task
  std::vector<std::shared_ptr<InferenceEngineTask>> Predict(
      const std::vector<xstream::BaseDataPtr> &input);

  // 一帧可能对应多个task
  std::vector<xstream::BaseDataPtr> GetResult(
      std::vector<std::shared_ptr<InferenceEngineTask>> tasks);

  int Perf(const std::vector<xstream::BaseDataPtr> &input,
           int run_times, bool only_inference);

  // 模型结构信息, PreProcess需要
  std::vector<TensorProperties> input_model_info_;
  std::vector<TensorProperties> output_model_info_;

 private:
  std::string config_file_path_;  // 配置文件路径
  Json::Value config_;
  int core_id_ = 0;                   // default core bpu0
  bool run_model_with_bbox_ = false;  // default tensor
  bool is_packed_model_ = false;      // default single model
  std::string model_path_;
  std::string model_name_;

  std::string preproc_type_;
  std::string postproc_type_;
  std::shared_ptr<PreProcess> preproc_;
  std::shared_ptr<PostProcess> postproc_;

  void *model_handle_ = nullptr;
  void *model_file_handle_ = nullptr;

  bool convert_to_float_ = true;  // 默认后处理转换浮点
};

}  // namespace inference

#endif  // INFERENCER_H_
