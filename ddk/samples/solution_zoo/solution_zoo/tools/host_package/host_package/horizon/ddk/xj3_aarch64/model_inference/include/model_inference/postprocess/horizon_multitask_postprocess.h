

/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of multitask_postprocess
 * @file   multitask_postprocess.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.05.13
 */
#ifndef HORIZON_MULTITASK_POSTPROCESS_H_
#define HORIZON_MULTITASK_POSTPROCESS_H_

#include <vector>
#include <string>
#include <memory>
#include <map>
#include <unordered_map>
#include "model_inference/inference_task.h"
#include "model_inference/postprocess/postprocess.h"
#include "hobotlog/hobotlog.hpp"
#include "bpu_parse_utils_extension.h"
#include "xstream/vision_type.h"

namespace inference {

struct FasterRCNNOutMsg;
struct FasterRCNNBranchInfo;

class MultitaskPostProcess : public PostProcess {
 public:
  virtual int Init(const std::string &json_str);

  virtual int Execute(
      const std::vector<std::shared_ptr<InferenceEngineTask>> &tasks,
      std::vector<xstream::BaseDataPtr> *frame_result);

 private:
  // 根据task->output_tensors_获取模型信息
  int GetModelInfo(const std::shared_ptr<InferenceEngineTask> task);

  // 解析RppBox
  void GetRppRects(const Tensor &bpu_tensor,
                   const std::shared_ptr<InferenceEngineTask> task,
                   std::map<std::string, std::vector<xstream::BBox>> &boxes,
                   int index, const std::string &name,
                   const std::unordered_map<int, std::string> &labels);

  // 解析Kps
  void GetKps(std::vector<xstream::Landmarks> &kpss,
              void* output,
              const std::vector<xstream::BBox> &body_boxes);
  void GetKps2(std::vector<xstream::Landmarks> &kpss,
               void* kps_label_output, void* kps_offset_output,
               const std::vector<xstream::BBox> &body_boxes);

  // 解析mask
  void GetMask(std::vector<xstream::Segmentation> &masks, void* output,
               const std::vector<xstream::BBox> &body_boxes);

  // 解析reid
  void GetReid(std::vector<xstream::FloatFeature> &reids, void *output,
               const std::vector<xstream::BBox> &body_boxes);

  // 解析LMKS
  void GetLMKS(std::vector<xstream::Landmarks> &landmarks, void* output,
               const std::vector<xstream::BBox> &face_boxes);

  // 解析LMKS2
  void GetLMKS2(std::vector<xstream::Landmarks> &landmarks,
                void* lmks2_label_output,
                void* lmks2_offset_output,
                const std::vector<xstream::BBox> &face_boxes);
  // 解析LMKS1
  void GetLMKS1(std::vector<xstream::Landmarks> &landmarks,
                void* output,
                const std::vector<xstream::BBox> &face_boxes);

  // 解析Pose
  void GetPose(std::vector<xstream::Pose3D> &face_pose,
               void* output, const std::vector<xstream::BBox> &face_boxes);

  void GetPlateAttribute(
    std::vector<xstream::Attribute_<int>> *plates_attribute, void* output,
    uint32_t shift, std::shared_ptr<std::vector<int>> aligned_dim,
    const std::vector<xstream::BBox> &plate_boxes,
    int32_t attribute_num);

  // Tensor 2 BPUTensor
  void ConvertTensor2BPUTensor(
      const Tensor &tensor, BPU_TENSOR_S &bpu_tensor);

  // 坐标转换
  void CoordinateTransform(FasterRCNNOutMsg &det_result);

  // FasterRCNNOutMsg to xstream data
  void GetResultMsg(FasterRCNNOutMsg &det_result,
                    std::vector<xstream::BaseDataPtr> &frame_output);

  std::map<int, FasterRCNNBranchInfo> out_level2rcnn_branch_info_;
  std::vector<std::string> method_outs_;

  bool is_init_ = false;

  // 模型输出层
  int output_layer_;

  int model_input_width_;
  int model_input_height_;
  int src_image_width_;
  int src_image_height_;

  float kps_pos_distance_ = 0.1;
  int kps_feat_width_ = 16;
  int kps_feat_height_ = 16;
  int kps_points_number_ = 17;
  int kps_feat_stride_ = 16;
  float kps_anchor_param_ = 0.0;

  float lmk_pos_distance_ = 12;
  int lmk_feat_width_ = 8;
  int lmk_feat_height_ = 8;
  int lmk_feat_stride_ = 16;
  int lmk_points_number_ = 5;
  float lmk_anchor_param_ = 0.0;
  int face_pose_number_ = 3;
  float face_pv_thr_ = 0.0;

  int32_t plate_color_num_ = 6;
  int32_t plate_row_num_ = 2;

  uint32_t kps_shift_ = 0;
  uint32_t kps_label_shift_ = 0;
  uint32_t kps_offset_shift_ = 0;
  uint32_t mask_shift_ = 0;
  uint32_t reid_shift_ = 0;
  uint32_t lmks_shift_ = 0;
  uint32_t lmks2_label_shift_ = 0;
  uint32_t lmks2_offset_shift_ = 0;
  uint32_t lmks1_shift_ = 0;
  uint32_t face_pose_shift_ = 0;
  uint32_t plate_color_shift_ = 0;
  uint32_t plate_row_shift_ = 0;

  std::shared_ptr<std::vector<int>> aligned_reid_dim = nullptr;
  std::shared_ptr<std::vector<int>> aligned_mask_dim = nullptr;
  std::shared_ptr<std::vector<int>> aligned_kps_dim = nullptr;
  std::shared_ptr<std::vector<int>> aligned_kps_label_dim = nullptr;
  std::shared_ptr<std::vector<int>> aligned_kps_offset_dim = nullptr;
  std::shared_ptr<std::vector<int>> aligned_lmks_dim = nullptr;
  std::shared_ptr<std::vector<int>> aligned_lmks2_label_dim = nullptr;
  std::shared_ptr<std::vector<int>> aligned_lmks2_offset_dim = nullptr;
  std::shared_ptr<std::vector<int>> aligned_lmks1_dim = nullptr;
  std::shared_ptr<std::vector<int>> aligned_face_pose_dim = nullptr;
  std::shared_ptr<std::vector<int>> aligned_plate_color_dim = nullptr;
  std::shared_ptr<std::vector<int>> aligned_plate_row_dim = nullptr;

  int reid_element_type;
  int mask_element_type;
  int kps_element_type;
  int kps_label_element_type;
  int kps_offset_element_type;
  int lmk_element_type;
  int lmk1_element_type;
  int lmk2_label_element_type;
  int lmk2_offet_element_type;
  int face_pose_element_type;
  int plate_color_element_type;
  int plate_row_element_type;
};

enum class FasterRCNNBranchOutType {
  BBOX,
  KPS,
  MASK,
  REID,
  LMKS2_LABEL,
  LMKS2_OFFSET,
  LMKS1,
  POSE_3D,
  PLATE_COLOR,
  PLATE_ROW,
  KPS_LABEL,
  KPS_OFFSET,
  LMKS,
  INVALID
};

struct FasterRCNNBranchInfo {
  FasterRCNNBranchOutType type;
  std::string name;
  std::string box_name;
  std::unordered_map<int, std::string> labels;
};

// result
struct FasterRCNNOutMsg {
  std::map<std::string, std::vector<xstream::BBox>> boxes;
  std::map<std::string, std::vector<xstream::Landmarks>> landmarks;
  std::map<std::string, std::vector<xstream::FloatFeature>> features;
  std::map<std::string, std::vector<xstream::Segmentation>> segmentations;
  std::map<std::string, std::vector<xstream::Pose3D>> poses;
  std::map<std::string, std::vector<xstream::Attribute_<int>>> attributes;
};

}  // namespace inference

#endif  // HORIZON_MULTITASK_POSTPROCESS_H_
