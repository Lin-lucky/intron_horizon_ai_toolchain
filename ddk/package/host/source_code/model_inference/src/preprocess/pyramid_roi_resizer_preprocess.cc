/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of pyramid_roi_preprocess
 * @file   pyramid_roi_preprocess.cc
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */

#include "model_inference/preprocess/pyramid_roi_resizer_preprocess.h"
#include "model_inference/inferencer.h"
#include "hobotlog/hobotlog.hpp"

namespace inference {

int PyramidRoiResizerPreProcess::Execute(
    const std::vector<xstream::BaseDataPtr> &input,
    std::vector<std::shared_ptr<InferenceEngineTask>>& tasks) {
  HOBOT_CHECK(input.size() == 2);  // roi, image
  auto rois = std::static_pointer_cast<xstream::BaseDataVector>(input[0]);
  auto xstream_img = std::static_pointer_cast<xstream::ImageFrame>(input[1]);

  std::string img_type = xstream_img->type_;
  HOBOT_CHECK(img_type == "PyramidImageFrame") << "not support " << img_type;

  auto pyramid_image = std::static_pointer_cast<
      xstream::PyramidImageFrame>(xstream_img);

  // 构建Roi Task
  auto roi_task = std::make_shared<RoiInferenceEngineTask>();
  tasks.push_back(roi_task);

  // prepare sequence_id_
  roi_task->sequence_id_ = pyramid_image->frame_id_;

  // preprocess rois
  int box_num = rois->datas_.size();
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
  // 将BBox封装为InferBox
  int valid_box_num = 0;
  std::vector<InferBBox> &infer_rois = roi_task->roi_box_;
  infer_rois.resize(box_num);

  for (int roi_idx = 0; roi_idx < box_num; roi_idx++) {
    infer_rois[roi_idx].x1 = preprocess_roi[roi_idx]->x1_;
    infer_rois[roi_idx].y1 = preprocess_roi[roi_idx]->y1_;
    infer_rois[roi_idx].x2 = preprocess_roi[roi_idx]->x2_;
    infer_rois[roi_idx].y2 = preprocess_roi[roi_idx]->y2_;
    infer_rois[roi_idx].score = preprocess_roi[roi_idx]->score_;
    if (preprocess_roi[roi_idx]->state_ != xstream::DataState::VALID) {
      infer_rois[roi_idx].resizable = false;
    } else {
      infer_rois[roi_idx].resizable = true;
      valid_box_num++;
    }
  }

  // 无有效检测框,不需预测
  if (valid_box_num == 0) {
    LOGD << "no valid box";
    return -1;
  }
  // prepare input_tensor
  roi_task->input_tensors_.resize(1);  // 内置roi仅支持一层模型输入
  int pym_layer = std::end(pyramid_image->img_.down_scale) -
                  std::begin(pyramid_image->img_.down_scale);
  std::vector<Tensor> pyramid_tensors(pym_layer);
  // 将pyramid封装为Tensor数据结构
  for (int i = 0; i < pym_layer; i++) {
    auto &downscale_pym = pyramid_image->img_.down_scale[i];
    int height = downscale_pym.height;
    int width = downscale_pym.width;
    int step = downscale_pym.step;
    // width, height, step, y_paddr, c_paddr, y_vaddr, c_vaddr
    Tensor &pyramid_tensor = pyramid_tensors[i];
    pyramid_tensor.sys_mem_type = MEM_TYPE_ASSIGNMENT;
    pyramid_tensor.properties.tensor_layout = LAYOUT_NCHW;
    pyramid_tensor.properties.tensor_type = IMG_TYPE_NV12_SEPARATE;
    // data size
    pyramid_tensor.properties.aligned_shape.num_dimensions = 4;
    pyramid_tensor.properties.aligned_shape.dimension_size[0] = 1;  // n
    pyramid_tensor.properties.aligned_shape.dimension_size[1] = 3;  // c
    pyramid_tensor.properties.aligned_shape.dimension_size[2] = height;  // h
    pyramid_tensor.properties.aligned_shape.dimension_size[3] = step;  // step
    pyramid_tensor.properties.valid_shape.num_dimensions = 4;
    pyramid_tensor.properties.valid_shape.dimension_size[0] = 1;  // n
    pyramid_tensor.properties.valid_shape.dimension_size[1] = 3;  // c
    pyramid_tensor.properties.valid_shape.dimension_size[2] = height;  // h
    pyramid_tensor.properties.valid_shape.dimension_size[3] = width;  // w
    // data
    pyramid_tensor.sys_mem[0].phy_addr = downscale_pym.y_paddr;
    pyramid_tensor.sys_mem[0].vir_addr =
        reinterpret_cast<void*>(downscale_pym.y_vaddr);
    pyramid_tensor.sys_mem[0].mem_size = height * step;
    pyramid_tensor.sys_mem[1].phy_addr = downscale_pym.c_paddr;
    pyramid_tensor.sys_mem[1].vir_addr =
        reinterpret_cast<void*>(downscale_pym.c_vaddr);
    pyramid_tensor.sys_mem[1].mem_size = height * step >> 1;
  }

  // prepare output_tensor properties
  InferenceEngine::GetInstance()->PrepareModelTensorProperties(
      infer_->output_model_info_, roi_task->output_tensors_);
  int ret = InferenceEngine::GetInstance()->AllocModelTensor(
      roi_task->output_tensors_, true, valid_box_num);
  if (ret != 0) {
    LOGE << "Alloc Output Tensor failed";
    roi_task->output_tensors_.clear();
    roi_task->input_tensors_.clear();
    return -1;
  }

  // Tensor封装为需要送入模型的数据(BPU不需操作，DNN需根据roi选层)
  InferenceEngine::GetInstance()->PrepareRoiTensor(
      infer_, pyramid_tensors, infer_rois, roi_task->input_tensors_[0]);

  return 0;
}

}  // namespace inference
