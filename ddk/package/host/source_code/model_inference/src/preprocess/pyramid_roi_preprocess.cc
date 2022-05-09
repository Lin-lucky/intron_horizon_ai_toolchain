/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of pyramid_roi_preprocess
 * @file   pyramid_roi_preprocess.cc
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */

#include "model_inference/preprocess/pyramid_roi_preprocess.h"
#include <cstring>
#include "model_inference/inferencer.h"
#include "hobotlog/hobotlog.hpp"

namespace inference {

int PyramidRoiPreProcess::Execute(
    const std::vector<xstream::BaseDataPtr> &input,
    std::vector<std::shared_ptr<InferenceEngineTask>>& tasks) {
  LOGD << "PyramidRoiPreProcess Execute";
  HOBOT_CHECK(input.size() == 2);  // roi, image
  auto rois = std::static_pointer_cast<xstream::BaseDataVector>(input[0]);
  auto xstream_img = std::static_pointer_cast<xstream::ImageFrame>(input[1]);

  std::string img_type = xstream_img->type_;
  HOBOT_CHECK(img_type == "PyramidImageFrame") << "not support " << img_type;
  auto pyramid_image = std::static_pointer_cast<
      xstream::PyramidImageFrame>(xstream_img);

  uint32_t w = pyramid_image->Width(0);  // 原图大小
  uint32_t h = pyramid_image->Height(0);

  int box_num = rois->datas_.size();

  int h_idx, w_idx, c_idx;
  InferenceEngine::GetInstance()->GetHWCIndex(
      infer_->input_model_info_[0].tensor_type,
      infer_->input_model_info_[0].tensor_layout,
      &h_idx, &w_idx, &c_idx);
  uint32_t dst_h =
      infer_->input_model_info_[0].valid_shape.dimension_size[h_idx];
  uint32_t dst_w =
      infer_->input_model_info_[0].valid_shape.dimension_size[w_idx];
  LOGD << "model input h: " << dst_h;
  LOGD << "model input w: " << dst_w;

  // preprocess rois
  std::vector<std::shared_ptr<xstream::BBox>> input_roi(box_num);
  std::vector<std::shared_ptr<xstream::BBox>> preprocess_roi;
  for (int roi_idx = 0; roi_idx < box_num; roi_idx++) {
    input_roi[roi_idx] =
        std::static_pointer_cast<xstream::BBox>(rois->datas_[roi_idx]);
  }
  if (roi_proc_.roi_process_pipeline_ != NONE) {
    roi_proc_.Execute(
        input_roi, pyramid_image->Width(0), pyramid_image->Height(0),
        preprocess_roi);
  } else {
    preprocess_roi = input_roi;
  }

  // 每个box对应一个tensor task; TODO 框架需要兼容处理无效任务
  for (int roi_idx = 0; roi_idx < box_num; roi_idx++) {
    // 构建Tensor Task
    auto tensor_task = std::make_shared<TensorInferenceEngineTask>();
    tasks.push_back(tensor_task);
    // prepare sequence_id_
    tensor_task->sequence_id_ = pyramid_image->frame_id_;

    // invalid_box
    if (preprocess_roi[roi_idx]->state_ != xstream::DataState::VALID) {
      LOGD << "Box Invalid";
      continue;
    }

    // crop roi
    uint8_t *crop_output = nullptr;
    int crop_dst_h, crop_dst_w;
    int ret = 0;
    {
      // aligen even
      int crop_x1 = preprocess_roi[roi_idx]->x1_;
      int crop_y1 = preprocess_roi[roi_idx]->y1_;
      int crop_x2 = preprocess_roi[roi_idx]->x2_;
      int crop_y2 = preprocess_roi[roi_idx]->y2_;

      crop_dst_h = crop_y2 - crop_y1 + 1;
      crop_dst_w = crop_x2 - crop_x1 + 1;
      if (crop_dst_w & 1) {
        if (crop_x2 < (w - 1)) {
          ++crop_x2;
        } else if (crop_x1 > 0) {
          --crop_x1;
        }
        crop_dst_w++;
      }
      if (crop_dst_h & 1) {
        if (crop_y2 < (h - 1)) {
          ++crop_y2;
        } else if (crop_y1 > 0) {
          --crop_y1;
        }
        crop_dst_h++;
      }
      ret = img_proc_.Crop(
        reinterpret_cast<uint8_t *>(pyramid_image->Data(0)),
        reinterpret_cast<uint8_t *>(pyramid_image->DataUV(0)),
        h, w, pyramid_image->Stride(0),
        crop_x1, crop_y1, crop_x2, crop_y2,
        &crop_output);
      if (ret != 0) {
        LOGE << "Crop roi failed";
        continue;
      }
    }

    uint8_t *output = nullptr;
    int output_height, output_width;
    // 至少需要resize预处理
    HOBOT_CHECK(img_proc_.process_pipeline_.size() >= 0);
    ret = img_proc_.Execute(
        crop_output, crop_output + crop_dst_h * crop_dst_w,
        crop_dst_h, crop_dst_w, &output,
        output_height, output_width);
    img_proc_.Free(crop_output);
    if (ret != 0) {
      LOGE << "Image PreProcess failed";
      continue;
    }
    // 检查配置参数与模型对应
    HOBOT_CHECK(output_height == dst_h && output_width == dst_w);

    // prepare tensor properties
    InferenceEngine::GetInstance()->PrepareModelTensorProperties(
        infer_->input_model_info_, tensor_task->input_tensors_);
    InferenceEngine::GetInstance()->PrepareModelTensorProperties(
        infer_->output_model_info_, tensor_task->output_tensors_);
    // 申请Tensor
    InferenceEngine::GetInstance()->
        AllocModelTensor(tensor_task->input_tensors_);
    InferenceEngine::GetInstance()->
        AllocModelTensor(tensor_task->output_tensors_);
    HOBOT_CHECK(tensor_task->input_tensors_.size() == 1);
    auto &input_tensor = tensor_task->input_tensors_[0];

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
      HOBOT_CHECK(0) << "not support tensor_type: "
          << input_tensor.properties.tensor_type;
    }
    img_proc_.Free(output);
  }

  return 0;
}

}  // namespace inference
