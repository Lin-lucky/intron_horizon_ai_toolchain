/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of pyramid_preprocess
 * @file   pyramid_preprocess.cc
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */

#include "model_inference/preprocess/pyramid_preprocess.h"
#include "model_inference/inferencer.h"
#include "string.h"
#include "model_inference/inference_engine.h"
#include "hobotlog/hobotlog.hpp"
#include "xstream/vision_type.h"

namespace inference {

int PyramidPreProcess::Execute(
    const std::vector<xstream::BaseDataPtr> &input,
    std::vector<std::shared_ptr<InferenceEngineTask>>& tasks) {
  HOBOT_CHECK(input.size() == 1);  // image
  auto xstream_img = std::static_pointer_cast<xstream::ImageFrame>(input[0]);

  std::string img_type = xstream_img->type_;
  HOBOT_CHECK(img_type == "PyramidImageFrame") << "not support " << img_type;

  auto pyramid_image = std::static_pointer_cast<
      xstream::PyramidImageFrame>(xstream_img);

  // 构建Tensor Task
  auto tensor_task = std::make_shared<TensorInferenceEngineTask>();
  tasks.push_back(tensor_task);

  // prepare sequence_id_
  tensor_task->sequence_id_ = pyramid_image->frame_id_;
  // prepare tensor properties
  InferenceEngine::GetInstance()->PrepareModelTensorProperties(
      infer_->input_model_info_, tensor_task->input_tensors_);
  InferenceEngine::GetInstance()->PrepareModelTensorProperties(
      infer_->output_model_info_, tensor_task->output_tensors_);

  int ret = Execute(pyramid_image,
                    pyramid_layer_,
                    tensor_task->input_tensors_);
  if (ret != 0) {
    LOGE << "Prepare Input Tensor failed";
    return -1;
  }

  // 申请output_tensors
  InferenceEngine::GetInstance()->AllocModelTensor(
      tensor_task->output_tensors_);
  return 0;
}

int PyramidPreProcess::Execute(
    std::shared_ptr<xstream::PyramidImageFrame> pyramid,
    const int pyramid_level,
    std::vector<Tensor> &input_tensors) {
  HOBOT_CHECK(input_tensors.size() == 1);
  Tensor &input_tensor = input_tensors[0];
  // 判断是否需要进行ImageProcess, 是则调用ImageProcess::Execute
  // 且需要申请Tensor空间
  if (img_proc_.process_pipeline_.size() == 0) {
    // 不需预处理,直接拷贝金字塔图像到Tensor
    input_tensor.sys_mem_type = MEM_TYPE_ASSIGNMENT;
    switch (input_tensor.properties.tensor_type) {
      case IMG_TYPE_NV12: {
        input_tensor.properties.tensor_type = IMG_TYPE_NV12_SEPARATE;
      }
      case IMG_TYPE_NV12_SEPARATE: {
        input_tensor.sys_mem[0].phy_addr =
            pyramid->img_.down_scale[pyramid_level].y_paddr;
        input_tensor.sys_mem[0].vir_addr = reinterpret_cast<void*>(
            pyramid->img_.down_scale[pyramid_level].y_vaddr);
        input_tensor.sys_mem[0].mem_size = pyramid->DataSize(pyramid_level);
        input_tensor.sys_mem[1].phy_addr =
            pyramid->img_.down_scale[pyramid_level].c_paddr;
        input_tensor.sys_mem[1].vir_addr = reinterpret_cast<void*>(
            pyramid->img_.down_scale[pyramid_level].c_vaddr);
        input_tensor.sys_mem[1].mem_size = pyramid->DataUVSize(pyramid_level);
        break;
      }
      default:
        LOGE << "not supporttensor_type: "
             << input_tensor.properties.tensor_type;
        return -1;
    }
  } else {
    // 需要预处理，并申请Tensor
    const uint8_t* input_y = reinterpret_cast<uint8_t*>(
        pyramid->img_.down_scale[pyramid_level].y_vaddr);
    const uint8_t* input_uv = reinterpret_cast<uint8_t*>(
        pyramid->img_.down_scale[pyramid_level].c_vaddr);
    const int input_height = pyramid->Height(pyramid_level);
    const int input_width = pyramid->Width(pyramid_level);
    // TODO(zhe.sun) stride
    uint8_t *output = nullptr;
    int output_height = 0, output_width = 0;
    int ret = img_proc_.Execute(input_y, input_uv, input_height, input_width,
                                &output, output_height, output_width);
    if (ret != 0) {
      LOGE << "Image Preprocess failed";
      return -1;
    }
    // 申请Tensor
    InferenceEngine::GetInstance()->AllocModelTensor(input_tensors);
    // 复制data：检查tensor类型是nv12，检查模型输入大小和data对应
    if (input_tensor.properties.tensor_type == IMG_TYPE_NV12) {
      HOBOT_CHECK(input_tensor.sys_mem[0].mem_size ==
                  output_height * output_width * 3 >> 1)
          << input_tensor.sys_mem[0].mem_size << " "
          << (output_height * output_width * 3 >> 1);
      memcpy(input_tensor.sys_mem[0].vir_addr, output,
             input_tensor.sys_mem[0].mem_size);
      // 不需flush，run model前进行flush
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
      // 不需flush，run model前进行flush
    } else {
      HOBOT_CHECK(0) << "model input data_type is not nv12, please check.";
    }

    // 释放output
    img_proc_.Free(output);
  }

  return 0;
}

}  // namespace inference
