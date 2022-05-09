/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of pyramid_roi_bpu_preprocess
 * @file   pyramid_roi_bpu_preprocess.cc
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */

#include "model_inference/preprocess/pyramid_roi_bpu_preprocess.h"
#include <cstring>
#include "bpu_predict_extension.h"
#include "model_inference/inferencer.h"
#include "hobotlog/hobotlog.hpp"

namespace inference {

static std::map<TensorLayout, BPU_LAYOUT_E> g_layout2bpu_table = {
      {LAYOUT_NONE, BPU_LAYOUT_NONE},
      {LAYOUT_NHWC, BPU_LAYOUT_NHWC},
      {LAYOUT_NCHW, BPU_LAYOUT_NCHW}};
static std::map<DataType, BPU_DATA_TYPE_E> g_datatype2bpu_table = {
      {IMG_TYPE_Y, BPU_TYPE_IMG_Y},
      {IMG_TYPE_NV12, BPU_TYPE_IMG_YUV_NV12},
      {IMG_TYPE_YUV444, BPU_TYPE_IMG_YUV444},
      {IMG_TYPE_BGR, BPU_TYPE_IMG_BGR},
      {IMG_TYPE_RGB, BPU_TYPE_IMG_RGB},
      {IMG_TYPE_NV12_SEPARATE, BPU_TYPE_IMG_NV12_SEPARATE},
      {TENSOR_TYPE_U8, BPU_TYPE_TENSOR_U8},
      {TENSOR_TYPE_S8, BPU_TYPE_TENSOR_S8},
      {TENSOR_TYPE_F32, BPU_TYPE_TENSOR_F32},
      {TENSOR_TYPE_S32, BPU_TYPE_TENSOR_S32},
      {TENSOR_TYPE_U32, BPU_TYPE_TENSOR_U32},
      {TENSOR_TYPE_S64, BPU_TYPE_TENSOR_S64},
      {TENSOR_TYPE_MAX, BPU_TYPE_MAX}};

int PyramidRoiBPUPreProcess::Init(const std::string &json_str) {
  // string转json
  Json::Reader Reader;
  Json::Value config;
  Reader.parse(json_str, config);
  int ret = 0;
  if (!config["config"]["roi_process_pipeline"].isNull()) {
    ret = roi_proc_.Init(
        config["config"]["roi_process_pipeline"].toStyledString());
    if (ret != 0) {
        LOGE << "RoiProcess Init Failed.";
        return ret;
    }
  }
  if (config["config"]["bpu_core"].isInt()) {
    bpu_core_ = config["config"]["bpu_core"].asInt();
    HOBOT_CHECK(bpu_core_ >= 0 && bpu_core_ <= 2);
  }
  return 0;
}

int PyramidRoiBPUPreProcess::Execute(
    const std::vector<xstream::BaseDataPtr> &input,
    std::vector<std::shared_ptr<InferenceEngineTask>>& tasks) {
  LOGD << "PyramidRoiBPUPreProcess Execute";
  HOBOT_CHECK(input.size() == 2);  // roi, image
  auto rois = std::static_pointer_cast<xstream::BaseDataVector>(input[0]);
  auto xstream_img = std::static_pointer_cast<xstream::ImageFrame>(input[1]);

  std::string img_type = xstream_img->type_;
  HOBOT_CHECK(img_type == "PyramidImageFrame") << "not support " << img_type;
  auto pyramid_image = std::static_pointer_cast<
      xstream::PyramidImageFrame>(xstream_img);

  uint32_t src_width = pyramid_image->Width(0);  // 原图大小
  uint32_t src_height = pyramid_image->Height(0);

  int box_num = rois->datas_.size();

  // 0. preprocess rois
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

  // 1. 准备原图数据，封装tensor
  Tensor nv12_tensor;
  nv12_tensor.properties.tensor_type = IMG_TYPE_NV12_SEPARATE;
  nv12_tensor.properties.tensor_layout = LAYOUT_NCHW;
  int h_idx, w_idx, c_idx;
  InferenceEngine::GetInstance()->GetHWCIndex(
      nv12_tensor.properties.tensor_type, nv12_tensor.properties.tensor_layout,
      &h_idx, &w_idx, &c_idx);
  {
    nv12_tensor.properties.valid_shape.num_dimensions = 4;
    nv12_tensor.properties.valid_shape.dimension_size[0] = 1;
    nv12_tensor.properties.valid_shape.dimension_size[h_idx] = src_height;
    nv12_tensor.properties.valid_shape.dimension_size[w_idx] = src_width;
    nv12_tensor.properties.valid_shape.dimension_size[c_idx] = 3;
    nv12_tensor.properties.aligned_shape = nv12_tensor.properties.valid_shape;

    // Copy y data to data0
    int pym_layer = 0;
    nv12_tensor.sys_mem[0].vir_addr = reinterpret_cast<void *>(
        pyramid_image->img_.down_scale[pym_layer].y_vaddr);
    nv12_tensor.sys_mem[0].phy_addr =
        pyramid_image->img_.down_scale[pym_layer].y_paddr;
    nv12_tensor.sys_mem[0].mem_size = src_height * src_width;
    // Copy uv data to data_ext
    nv12_tensor.sys_mem[1].vir_addr = reinterpret_cast<void *>(
        pyramid_image->img_.down_scale[pym_layer].c_vaddr);
    nv12_tensor.sys_mem[1].phy_addr =
        pyramid_image->img_.down_scale[pym_layer].c_paddr;
    nv12_tensor.sys_mem[1].mem_size = (src_height + 1) / 2 * src_width;
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

    int ret = 0;
    // prepare tensor properties
    InferenceEngine::GetInstance()->PrepareModelTensorProperties(
        infer_->input_model_info_, tensor_task->input_tensors_);
    InferenceEngine::GetInstance()->PrepareModelTensorProperties(
        infer_->output_model_info_, tensor_task->output_tensors_);
    HOBOT_CHECK(tensor_task->input_tensors_.size() == 1);
    auto &input_tensor = tensor_task->input_tensors_[0];
    // HB_BPU_cropAndResize only support nv12_seperate
    if (input_tensor.properties.tensor_type == IMG_TYPE_NV12) {
      input_tensor.properties.tensor_type = IMG_TYPE_NV12_SEPARATE;
    }

    // 申请Tensor
    InferenceEngine::GetInstance()->
        AllocModelTensor(tensor_task->input_tensors_);
    InferenceEngine::GetInstance()->
        AllocModelTensor(tensor_task->output_tensors_);

    InferBBox input_roi = {preprocess_roi[roi_idx]->x1_,
                           preprocess_roi[roi_idx]->y1_,
                           preprocess_roi[roi_idx]->x2_,
                           preprocess_roi[roi_idx]->y2_,
                           preprocess_roi[roi_idx]->score_,
                           preprocess_roi[roi_idx]->id_,
                           true};
    ResizeCtrlParam ctrl_param = {
        RESIZE_TYPE_BILINEAR, RESIZE_TYPE_IMG_YUV_NV12, bpu_core_, 0, nullptr};
    ret = InferenceEngine::GetInstance()->ResizeImage(nv12_tensor, input_tensor,
                                                      input_roi, ctrl_param);
    if (ret != 0) {
      LOGE << "Resize failed";
      InferenceEngine::GetInstance()->FreeTensor(tensor_task->input_tensors_);
      InferenceEngine::GetInstance()->FreeTensor(tensor_task->output_tensors_);
      continue;
    }
  }

  return 0;
}

void PyramidRoiBPUPreProcess::ConvertTensor2BPUTensor(
    const Tensor &tensor, BPU_TENSOR_S &bpu_tensor) {
  bpu_tensor.data_type = g_datatype2bpu_table[tensor.properties.tensor_type];
  bpu_tensor.data_shape.layout =
      g_layout2bpu_table[tensor.properties.tensor_layout];
  bpu_tensor.data_shape.ndim = tensor.properties.valid_shape.num_dimensions;
  for (int dim = 0; dim < bpu_tensor.data_shape.ndim; dim++) {
    bpu_tensor.data_shape.d[dim] =
        tensor.properties.valid_shape.dimension_size[dim];
  }

  bpu_tensor.aligned_shape.layout =
      g_layout2bpu_table[tensor.properties.tensor_layout];
  bpu_tensor.aligned_shape.ndim =
      tensor.properties.aligned_shape.num_dimensions;
  for (int dim = 0; dim < bpu_tensor.aligned_shape.ndim; dim++) {
    bpu_tensor.aligned_shape.d[dim] =
        tensor.properties.aligned_shape.dimension_size[dim];
  }
  switch (tensor.properties.tensor_type) {
  case IMG_TYPE_NV12:
  case TENSOR_TYPE_S8:
  case TENSOR_TYPE_U8:
  case TENSOR_TYPE_S16:
  case TENSOR_TYPE_U16:
  case TENSOR_TYPE_F32:
  case TENSOR_TYPE_S32:
  case TENSOR_TYPE_U32:
  case TENSOR_TYPE_F64:
  case TENSOR_TYPE_S64:
  case TENSOR_TYPE_U64:
    bpu_tensor.data.phyAddr = tensor.sys_mem[0].phy_addr;
    bpu_tensor.data.virAddr = tensor.sys_mem[0].vir_addr;
    bpu_tensor.data.memSize = tensor.sys_mem[0].mem_size;
    break;
  case IMG_TYPE_NV12_SEPARATE:
    bpu_tensor.data.phyAddr = tensor.sys_mem[0].phy_addr;
    bpu_tensor.data.virAddr = tensor.sys_mem[0].vir_addr;
    bpu_tensor.data.memSize = tensor.sys_mem[0].mem_size;
    bpu_tensor.data_ext.phyAddr = tensor.sys_mem[1].phy_addr;
    bpu_tensor.data_ext.virAddr = tensor.sys_mem[1].vir_addr;
    bpu_tensor.data_ext.memSize = tensor.sys_mem[1].mem_size;
    break;
  default:
    LOGE << "not support tensor_type: " << tensor.properties.tensor_type;
    break;
  }
}

}  // namespace inference
