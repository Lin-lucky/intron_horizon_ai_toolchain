/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of image_preprocess
 * @file   image_preprocess.cc
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */

#include "model_inference/preprocess/image_preprocess.h"
#include "model_inference/inferencer.h"
#include <cstring>

namespace inference {

int ImageInputWithPreProcess::Execute(
    const std::vector<xstream::BaseDataPtr> &input,
    std::vector<std::shared_ptr<InferenceEngineTask>>& tasks) {
  HOBOT_CHECK(input.size() == 1);  // image
  auto xstream_img = std::static_pointer_cast<xstream::ImageFrame>(input[0]);
  std::string img_type = xstream_img->type_;
  HOBOT_CHECK(img_type == "RawDataImageFrame") << "not support " << img_type;
  HOBOT_CHECK(xstream_img->pixel_format_ ==
              xstream::kHorizonVisionPixelFormatRawNV12)
      << "not support pixel_format: " << xstream_img->pixel_format_;

  auto image = std::static_pointer_cast<
      xstream::RawDataImageFrame>(xstream_img);

  // 构建一个Tensor task
  auto tensor_task = std::make_shared<TensorInferenceEngineTask>();
  tasks.push_back(tensor_task);

  // prepare sequence_id_
  tensor_task->sequence_id_ = image->frame_id_;

  // prepare tensor properties
  InferenceEngine::GetInstance()->PrepareModelTensorProperties(
      infer_->input_model_info_, tensor_task->input_tensors_);
  InferenceEngine::GetInstance()->PrepareModelTensorProperties(
      infer_->output_model_info_, tensor_task->output_tensors_);

  std::vector<Tensor> &input_tensors = tensor_task->input_tensors_;
  HOBOT_CHECK(input_tensors.size() == 1);
  Tensor &input_tensor = input_tensors[0];

  int ret = 0;
  // 进行预处理，并申请Tensor
  uint8_t *output = nullptr;
  int output_height = 0, output_width = 0;
  if (img_proc_.process_pipeline_.size() != 0) {
    const uint8_t* input_y = reinterpret_cast<uint8_t*>(image->Data());
    const uint8_t* input_uv = reinterpret_cast<uint8_t*>(image->DataUV());
    const int input_height = image->height_;
    const int input_width = image->width_;
    ret = img_proc_.Execute(input_y, input_uv, input_height, input_width,
                            &output, output_height, output_width);
    if (ret != 0) {
      LOGE << "Image Preprocess failed";
      return -1;
    }
  } else {
    output = reinterpret_cast<uint8_t*>(image->Data());
    output_height = image->height_, output_width = image->width_;
  }
  LOGD << "preprocess success";
  // 申请Input Tensor
  ret = InferenceEngine::GetInstance()->AllocModelTensor(input_tensors);
  if (ret != 0) {
    LOGE << "AllocModelTensor failed";
    return -1;
  }

  // 复制data：检查tensor类型是nv12，检查模型输入大小和data对应
  if (input_tensor.properties.tensor_type == IMG_TYPE_NV12) {
    HOBOT_CHECK(input_tensor.sys_mem[0].mem_size ==
        output_height * output_width * 3 >> 1);

     memcpy(input_tensor.sys_mem[0].vir_addr, output,
           input_tensor.sys_mem[0].mem_size);
  } else if (input_tensor.properties.tensor_type == IMG_TYPE_NV12_SEPARATE) {
    HOBOT_CHECK(input_tensor.sys_mem[0].mem_size ==
                output_height * output_width);
    HOBOT_CHECK(input_tensor.sys_mem[1].mem_size ==
                output_height * output_width >> 1);
    memcpy(input_tensor.sys_mem[0].vir_addr, output,
           input_tensor.sys_mem[0].mem_size);
    memcpy(input_tensor.sys_mem[1].vir_addr,
           output + input_tensor.sys_mem[0].mem_size,
           input_tensor.sys_mem[1].mem_size);
  } else {
    HOBOT_CHECK(0) << "model input data_type is not nv12, please check.";
  }

  if (img_proc_.process_pipeline_.size() != 0) {
    // 释放output
    img_proc_.Free(output);
  }

  // 申请Output Tensor
  ret = InferenceEngine::GetInstance()->AllocModelTensor(
      tensor_task->output_tensors_);
  if (ret != 0) {
    LOGE << "AllocModelTensor failed";
    // 释放input tensor
    InferenceEngine::GetInstance()->FreeTensor(tensor_task->input_tensors_);
    return -1;
  }
  return 0;
}

}  // namespace inference
