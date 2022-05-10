/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: multitask_postprocess_method.cc
 * @Brief: definition of the MultitaskPostProcessMethod
 * @Author: zhe.sun
 * @Email: zhe.sun@horizon.ai
 * @Date: 2020-12-03 20:00:05
 * @Last Modified by: zhe.sun
 * @Last Modified time: 2020-12-03 21:23:08
 */

#include "multitask_postprocess_method/multitask_postprocess_method.h"
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>
#include "dnn_async_data.h"
#include "dnn_util.h"
#include "hobotlog/hobotlog.hpp"
#include "xstream/profiler.h"
#include "xstream/vision_type.h"

namespace xstream {

static std::map<std::string, BranchOutType> str2out_type = {
    {"bbox", BranchOutType::BBOX},
    {"kps", BranchOutType::KPS},
    {"mask", BranchOutType::MASK},
    {"reid", BranchOutType::REID},
    {"lmks2_label", BranchOutType::LMKS2_LABEL},
    {"lmks2_offset", BranchOutType::LMKS2_OFFSET},
    {"lmks1", BranchOutType::LMKS1},
    {"3d_pose", BranchOutType::POSE_3D},
    {"plate_color", BranchOutType::PLATE_COLOR},
    {"plate_row", BranchOutType::PLATE_ROW},
    {"kps_label", BranchOutType::KPS_LABEL},
    {"kps_offset", BranchOutType::KPS_OFFSET},
    {"lmks", BranchOutType::LMKS}};

int MultitaskPostProcessMethod::Init(const std::string &cfg_path) {
  LOGD << "MultitaskPostProcessMethod Init";
  DnnPostProcessMethod::Init(cfg_path);

  auto net_info = config_.GetSubConfig("net_info");

  std::vector<std::shared_ptr<Config>> model_out_sequence =
      net_info->GetSubConfigArray("model_out_sequence");

  LOGD << "rcnn branch out type: ";
  for (size_t i = 0; i < model_out_sequence.size(); ++i) {
    std::string type_str = model_out_sequence[i]->GetSTDStringValue("type");
    HOBOT_CHECK(!type_str.empty());
    LOGD << type_str;
    if (str2out_type.find(type_str) == str2out_type.end()) {
      out_level2branch_info_[i].type = BranchOutType::INVALID;
    } else {
      out_level2branch_info_[i].type = str2out_type[type_str];
      out_level2branch_info_[i].name =
          model_out_sequence[i]->GetSTDStringValue("name");
      out_level2branch_info_[i].box_name =
          model_out_sequence[i]->GetSTDStringValue("box_name");
      out_level2branch_info_[i].labels =
          model_out_sequence[i]->GetLabelsMap("labels");
    }
  }

  kps_pos_distance_ = net_info->GetFloatValue("kps_pos_distance", 0.1);
  kps_feat_width_ = net_info->GetIntValue("kps_feat_width", 16);
  kps_feat_height_ = net_info->GetIntValue("kps_feat_height", 16);
  kps_points_number_ = net_info->GetIntValue("kps_points_number", 17);
  kps_feat_stride_ = net_info->GetIntValue("kps_feat_stride", 16);
  kps_anchor_param_ = net_info->GetFloatValue("kps_anchor_param", 0.0);

  lmk_pos_distance_ = net_info->GetFloatValue("lmk_pos_distance", 12);
  lmk_feat_width_ = net_info->GetIntValue("lmk_feat_width", 8);
  lmk_feat_height_ = net_info->GetIntValue("lmk_feat_height", 8);
  lmk_feat_stride_ = net_info->GetIntValue("lmk_feat_stride", 16);
  lmk_points_number_ = net_info->GetIntValue("lmk_points_number", 5);
  lmk_anchor_param_ = net_info->GetFloatValue("lmk_anchor_param", 0.0);
  face_pose_number_ = net_info->GetIntValue("3d_pose_number", 3);
  face_pv_thr_ = net_info->GetFloatValue("face_pv_thr", 0.0);

  bbox_threshold_ = net_info->GetFloatValue("bbox_threshold", 0.0);

  plate_color_num_ = net_info->GetIntValue("plate_color_num", 0);
  plate_row_num_ = net_info->GetIntValue("plate_row_num", 0);

  method_outs_ = config_.GetSTDStringArray("method_outs");

  need_prepare_output_info_ = true;

  return 0;
}

int MultitaskPostProcessMethod::ParseDnnResult(
    DnnAsyncData &dnn_result, std::vector<BaseDataPtr> &frame_result) {
  LOGD << "MultitaskPostProcessMethod ParseDnnResult";

  frame_result.resize(method_outs_.size());

  if (need_prepare_output_info_) {
    int ret = HB_BPU_getHW(dnn_result.dnn_model->bpu_model.inputs[0].data_type,
                           &dnn_result.dnn_model->bpu_model.inputs[0].shape,
                           &model_input_height_, &model_input_width_);
    if (ret != 0) {
      LOGE << "Error getting model input size";
      return ret;
    }

    int output_num = dnn_result.dnn_model->bpu_model.output_num;
    model_output_info_.resize(output_num);
    ret = GetOutputInfo(dnn_result.dnn_model->bpu_model, model_output_info_);
    if (ret != 0) {
      LOGE << "Failed getting model output info";
      return ret;
    }
    need_prepare_output_info_ = false;
  }

  OutMsg det_results;
  ParseResults(dnn_result, det_results);
  CoordinateTransfrom(dnn_result, det_results, model_input_width_,
                      model_input_height_);

  std::map<std::string, std::shared_ptr<BaseDataVector>> xstream_det_result;
  Convert2FrameworkData(det_results, xstream_det_result);

  for (size_t i = 0; i < frame_result.size(); ++i) {
    if (xstream_det_result[method_outs_[i]]) {
      frame_result[i] = xstream_det_result[method_outs_[i]];
    }
  }

  return 0;
}

void MultitaskPostProcessMethod::ParseResults(DnnAsyncData &dnn_result,
                                              OutMsg &det_results) {
  auto &output_tensors = dnn_result.output_tensors[0];

  if (dnn_result.output_tensors.empty()) {
    LOGD << "failed to run model in predict method";
    return;
  }
  // flush cache
  for (int i = 0; i < dnn_result.dnn_model->bpu_model.output_num; ++i) {
    if (HB_SYS_isMemCachable(&(output_tensors[i].data))) {
      HB_SYS_flushMemCache(&(output_tensors[i].data),
                           HB_SYS_MEM_CACHE_INVALIDATE);
    }
  }

  auto bpu_model = dnn_result.dnn_model->bpu_model;
  void *lmk2_label_out_put = nullptr;
  void *lmk2_offset_out_put = nullptr;
  void *kps_label_out_put = nullptr;
  void *kps_offset_out_put = nullptr;
  uint32_t lmk2_label_level = 0;
  uint32_t lmk2_offset_level = 0;
  uint32_t kps_label_level = 0;
  uint32_t kps_offset_level = 0;

  for (size_t out_level = 0; out_level < output_tensors.size(); ++out_level) {
    const auto &branch_info = out_level2branch_info_[out_level];
    auto out_type = branch_info.type;
    switch (out_type) {
      case BranchOutType::INVALID:
        break;
      case BranchOutType::BBOX:
        GetRppRects(det_results.boxes, out_level, branch_info.name,
                    branch_info.labels, &(dnn_result.dnn_model->bpu_model),
                    output_tensors);
        break;
      case BranchOutType::LMKS2_LABEL:
        lmk2_label_out_put = output_tensors[out_level].data.virAddr;
        lmk2_label_level = out_level;
        break;
      case BranchOutType::LMKS2_OFFSET:
        lmk2_offset_out_put = output_tensors[out_level].data.virAddr;
        lmk2_offset_level = out_level;
        break;
      case BranchOutType::KPS_LABEL:
        kps_label_out_put = output_tensors[out_level].data.virAddr;
        kps_label_level = out_level;
        break;
      case BranchOutType::KPS_OFFSET:
        kps_offset_out_put = output_tensors[out_level].data.virAddr;
        kps_offset_level = out_level;
        break;
      default:
        break;
    }
  }

  for (size_t out_level = 0; out_level < output_tensors.size(); ++out_level) {
    const auto &branch_info = out_level2branch_info_[out_level];
    auto out_type = branch_info.type;
    switch (out_type) {
      case BranchOutType::INVALID:
        break;
      case BranchOutType::KPS:
        LOGD << "begin GetKps";
        GetKps(bpu_model, out_level, det_results.landmarks[branch_info.name],
               output_tensors[out_level].data.virAddr,
               det_results.boxes[branch_info.box_name]);
        break;
      case BranchOutType::KPS_LABEL:
        LOGD << "begin GetKps2";
        HOBOT_CHECK(kps_label_level == out_level) << "kps label level mismatch";
        GetKps2(bpu_model, det_results.landmarks["kps"], kps_label_out_put,
                kps_offset_out_put, kps_label_level, kps_offset_level,
                det_results.boxes[branch_info.box_name]);
        break;
      case BranchOutType::LMKS2_LABEL:
        LOGD << "begin GetLMKS2";
        HOBOT_CHECK(lmk2_label_level == out_level)
            << "lmks2 label level mismatch";
        GetLMKS2(bpu_model, det_results.landmarks["landmark2"],
                 lmk2_label_out_put, lmk2_offset_out_put, lmk2_label_level,
                 lmk2_offset_level, det_results.boxes[branch_info.box_name]);
        break;
      case BranchOutType::LMKS1:
        LOGD << "begin GetLMKS1";
        GetLMKS1(bpu_model, out_level, det_results.landmarks["landmark1"],
                 output_tensors[out_level].data.virAddr,
                 det_results.boxes[branch_info.box_name]);
        break;
      case BranchOutType::POSE_3D:
        LOGD << "begin GetPose";
        GetPose(bpu_model, out_level, det_results.poses[branch_info.name],
                output_tensors[out_level].data.virAddr,
                det_results.boxes[branch_info.box_name]);
        break;
      case BranchOutType::PLATE_COLOR:
        LOGD << "begin PlateColor";
        GetPlateAttribute(bpu_model, out_level,
                          det_results.attributes[branch_info.name],
                          output_tensors[out_level].data.virAddr,
                          det_results.boxes[branch_info.box_name],
                          plate_color_num_);
        break;
      case BranchOutType::PLATE_ROW:
        LOGD << "begin PlateRow";
        GetPlateAttribute(bpu_model, out_level,
                          det_results.attributes[branch_info.name],
                          output_tensors[out_level].data.virAddr,
                          det_results.boxes[branch_info.box_name],
                          plate_row_num_);
        break;
      default:
        // TODO(shiyu.fu): lmks, mask, reid, vehicle
        break;
    }
  }
}

void MultitaskPostProcessMethod::GetRppRects(
    std::map<std::string, std::vector<BBox>> &boxes, int index,
    const std::string &name, const std::unordered_map<int, std::string> &labels,
    BPU_MODEL_S *bpu_model, std::vector<BPU_TENSOR_S> &output_tensors) {
  BPU_RPP_BBOX rpp_bbox;
  int ret =
      HB_BPU_parseRPPResult(bpu_model, output_tensors.data(), index, &rpp_bbox);
  if (ret != 0) {
    LOGE << "failed to parse rpp bbox";
    return;
  }
  int nBox = rpp_bbox.bbox_num;
  HOBOT_CHECK(rpp_bbox.result_type == BPU_RPP_BBOX::bbox_type_f32);

  auto box_ptr = rpp_bbox.bbox_ptr_f32;
  LOGD << "rpp box num: " << nBox;

  for (int i = 0; i < nBox; ++i) {
    BBox box;
    box.x1_ = box_ptr[i].left;
    box.y1_ = box_ptr[i].top;
    box.x2_ = box_ptr[i].right;
    box.y2_ = box_ptr[i].bottom;
    box.score_ = box_ptr[i].score;
    int type = box_ptr[i].class_label;
    if (labels.find(type) != labels.end()) {
      box.specific_type_ = labels.at(type);
    } else {
      box.specific_type_ = name;
    }
    boxes[box.specific_type_].push_back(std::move(box));
  }
}

void MultitaskPostProcessMethod::GetKps(BPU_MODEL_S bpu_model, int out_level,
                                        std::vector<Landmarks> &kpss,
                                        void *output,
                                        const std::vector<BBox> &body_boxes) {
  int32_t *kps_feature = reinterpret_cast<int32_t *>(output);

  float pos_distance = kps_pos_distance_ * kps_feat_width_;
  size_t body_box_num = body_boxes.size();

  auto aligned_dim = model_output_info_[out_level].aligned_dims;
  uint32_t shift = model_output_info_[out_level].shift;

  int feature_size = aligned_dim[1] * aligned_dim[2] * aligned_dim[3];
  int h_stride = aligned_dim[2] * aligned_dim[3];
  int w_stride = aligned_dim[3];

  for (size_t box_id = 0; box_id < body_box_num; ++box_id) {
    const auto &body_box = body_boxes[box_id];
    float x1 = body_box.x1_;
    float y1 = body_box.y1_;
    float x2 = body_box.x2_;
    float y2 = body_box.y2_;
    float w = x2 - x1 + 1;
    float h = y2 - y1 + 1;

    float scale_x = kps_feat_width_ / w;
    float scale_y = kps_feat_height_ / h;

    Landmarks skeleton;
    skeleton.values_.resize(kps_points_number_);

    auto *mxnet_out_for_one_point_begin = kps_feature + feature_size * box_id;
    for (int kps_id = 0; kps_id < kps_points_number_; ++kps_id) {
      // find the best position
      int max_w = 0;
      int max_h = 0;
      int max_score_before_shift = mxnet_out_for_one_point_begin[kps_id];
      int32_t *mxnet_out_for_one_point = nullptr;
      for (int hh = 0; hh < kps_feat_height_; ++hh) {
        for (int ww = 0; ww < kps_feat_width_; ++ww) {
          mxnet_out_for_one_point =
              mxnet_out_for_one_point_begin + hh * h_stride + ww * w_stride;
          if (mxnet_out_for_one_point[kps_id] > max_score_before_shift) {
            max_w = ww;
            max_h = hh;
            max_score_before_shift = mxnet_out_for_one_point[kps_id];
          }
        }
      }

      float max_score = GetFloatByInt(max_score_before_shift, shift);

      // get delta
      mxnet_out_for_one_point =
          mxnet_out_for_one_point_begin + max_h * h_stride + max_w * w_stride;
      const auto x_delta =
          mxnet_out_for_one_point[2 * kps_id + kps_points_number_];
      float fp_delta_x = GetFloatByInt(x_delta, shift) * pos_distance;

      const auto y_delta =
          mxnet_out_for_one_point[2 * kps_id + kps_points_number_ + 1];
      float fp_delta_y = GetFloatByInt(y_delta, shift) * pos_distance;

      Point point;
      point.x_ =
          (max_w + fp_delta_x + 0.46875 + kps_anchor_param_) / scale_x + x1;
      point.y_ =
          (max_h + fp_delta_y + 0.46875 + kps_anchor_param_) / scale_y + y1;
      point.score_ = SigMoid(max_score);
      skeleton.values_[kps_id] = point;
    }
    kpss.push_back(std::move(skeleton));
  }
}

void MultitaskPostProcessMethod::GetKps2(
    BPU_MODEL_S bpu_model, std::vector<Landmarks> &kpss,
    void *kps_label_output, void *kps_offset_output, int kps_label_level,
    int kps_offset_level, const std::vector<BBox> &body_boxes) {
  int32_t *label_feature = reinterpret_cast<int32_t *>(kps_label_output);
  int32_t *offset_feature = reinterpret_cast<int32_t *>(kps_offset_output);

  int input_height = kps_feat_height_ * kps_feat_stride_;
  int input_width = kps_feat_width_ * kps_feat_stride_;
  float base_center = (kps_feat_stride_ - 1) / 2.0;

  auto aligned_label_dim = model_output_info_[kps_label_level].aligned_dims;
  auto aligned_offset_dim = model_output_info_[kps_offset_level].aligned_dims;
  uint32_t label_shift = model_output_info_[kps_label_level].shift;
  uint32_t offset_shift = model_output_info_[kps_offset_level].shift;

  int label_feature_size =
      aligned_label_dim[1] * aligned_label_dim[2] * aligned_label_dim[3];
  int label_h_stride = aligned_label_dim[2] * aligned_label_dim[3];
  int label_w_stride = aligned_label_dim[3];

  int offset_feature_size =
      aligned_offset_dim[1] * aligned_offset_dim[2] * aligned_offset_dim[3];
  int offset_h_stride = aligned_offset_dim[2] * aligned_offset_dim[3];
  int offset_w_stride = aligned_offset_dim[3];

  size_t body_box_num = body_boxes.size();

  for (size_t box_id = 0; box_id < body_box_num; ++box_id) {
    const auto &body_box = body_boxes[box_id];
    float x1 = body_box.x1_;
    float y1 = body_box.y1_;
    float x2 = body_box.x2_;
    float y2 = body_box.y2_;
    float w = x2 - x1 + 1;
    float h = y2 - y1 + 1;

    float scale_x = w / input_width;
    float scale_y = h / input_height;
    float scale_pos_x = kps_pos_distance_ * scale_x;
    float scale_pos_y = kps_pos_distance_ * scale_y;

    Landmarks skeleton;
    skeleton.values_.resize(kps_points_number_);
    auto *label_feature_begin = label_feature + label_feature_size * box_id;

    for (int kps_id = 0; kps_id < kps_points_number_; ++kps_id) {
      // find the best position
      int max_w = 0;
      int max_h = 0;
      int max_score_before_shift = label_feature_begin[kps_id];
      int32_t *mxnet_out_for_one_point = nullptr;
      for (int hh = 0; hh < kps_feat_height_; ++hh) {
        for (int ww = 0; ww < kps_feat_width_; ++ww) {
          mxnet_out_for_one_point =
              label_feature_begin + hh * label_h_stride + ww * label_w_stride;
          if (mxnet_out_for_one_point[kps_id] > max_score_before_shift) {
            max_w = ww;
            max_h = hh;
            max_score_before_shift = mxnet_out_for_one_point[kps_id];
          }
        }
      }
      float max_score = GetFloatByInt(max_score_before_shift, label_shift);

      float base_x = (max_w * kps_feat_stride_ + base_center) * scale_x + x1;
      float base_y = (max_h * kps_feat_stride_ + base_center) * scale_y + y1;

      // get delta
      int32_t *offset_feature_begin =
          offset_feature + offset_feature_size * box_id;
      mxnet_out_for_one_point = offset_feature_begin + max_h * offset_h_stride +
                                max_w * offset_w_stride;

      const auto x_delta = mxnet_out_for_one_point[2 * kps_id];
      float fp_delta_x = GetFloatByInt(x_delta, offset_shift);

      const auto y_delta = mxnet_out_for_one_point[2 * kps_id + 1];
      float fp_delta_y = GetFloatByInt(y_delta, offset_shift);

      Point point;
      point.x_ = base_x + fp_delta_x * scale_pos_x;
      point.y_ = base_y + fp_delta_y * scale_pos_y;
      point.score_ = SigMoid(max_score);
      skeleton.values_[kps_id] = point;
    }
    kpss.push_back(std::move(skeleton));
  }
}

void MultitaskPostProcessMethod::GetLMKS1(BPU_MODEL_S bpu_model, int out_level,
                                          std::vector<Landmarks> &landmarks,
                                          void *output,
                                          const std::vector<BBox> &face_boxes) {
  size_t face_box_num = face_boxes.size();
  int32_t *lmks1_feature = reinterpret_cast<int32_t *>(output);

  auto aligned_dim = model_output_info_[out_level].aligned_dims;
  uint32_t shift = model_output_info_[out_level].shift;

  int feature_size = aligned_dim[1] * aligned_dim[2] * aligned_dim[3];

  for (size_t box_id = 0; box_id < face_box_num; ++box_id) {
    const auto &face_box = face_boxes[box_id];
    float x1 = face_box.x1_;
    float y1 = face_box.y1_;
    float x2 = face_box.x2_;
    float y2 = face_box.y2_;
    float w = x2 - x1 + 1;
    float h = y2 - y1 + 1;

    int32_t *mxnet_out_for_this_box = lmks1_feature + feature_size * box_id;

    Landmarks landmark;
    for (int i = 0; i < lmk_points_number_; ++i) {
      float x = GetFloatByInt(mxnet_out_for_this_box[2 * i], shift) * w + x1;
      float y =
          GetFloatByInt(mxnet_out_for_this_box[2 * i + 1], shift) * h + y1;
      landmark.values_.push_back(Point(x, y));
    }
    landmarks.push_back(std::move(landmark));
  }
}

void MultitaskPostProcessMethod::GetLMKS2(
    BPU_MODEL_S bpu_model, std::vector<Landmarks> &landmarks,
    void *lmks2_label_output, void *lmks2_offset_output, int lmks2_label_level,
    int lmks2_offset_level, const std::vector<BBox> &face_boxes) {
  int32_t *label_feature = reinterpret_cast<int32_t *>(lmks2_label_output);
  int32_t *offset_feature = reinterpret_cast<int32_t *>(lmks2_offset_output);

  int input_height = lmk_feat_height_ * lmk_feat_stride_;
  int input_width = lmk_feat_width_ * lmk_feat_stride_;
  float base_center = (lmk_feat_stride_ - 1) / 2.0;
  size_t face_box_num = face_boxes.size();

  auto aligned_label_dim = model_output_info_[lmks2_label_level].aligned_dims;
  auto aligned_offset_dim = model_output_info_[lmks2_offset_level].aligned_dims;
  uint32_t label_shift = model_output_info_[lmks2_label_level].shift;
  uint32_t offset_shift = model_output_info_[lmks2_offset_level].shift;

  int label_feature_size =
      aligned_label_dim[1] * aligned_label_dim[2] * aligned_label_dim[3];
  int label_h_stride = aligned_label_dim[2] * aligned_label_dim[3];
  int label_w_stride = aligned_label_dim[3];
  int offset_feature_size =
      aligned_offset_dim[1] * aligned_offset_dim[2] * aligned_offset_dim[3];
  int offset_h_stride = aligned_offset_dim[2] * aligned_offset_dim[3];
  int offset_w_stride = aligned_offset_dim[3];

  for (size_t box_id = 0; box_id < face_box_num; ++box_id) {
    const auto &face_box = face_boxes[box_id];
    float x1 = face_box.x1_;
    float y1 = face_box.y1_;
    float x2 = face_box.x2_;
    float y2 = face_box.y2_;
    float w = x2 - x1 + 1;
    float h = y2 - y1 + 1;

    assert(input_width != 0 && input_height != 0);
    float scale_x = w / input_width;
    float scale_y = h / input_height;
    float scale_pos_x = lmk_pos_distance_ * scale_x;
    float scale_pos_y = lmk_pos_distance_ * scale_y;

    Landmarks landmark;
    landmark.values_.resize(lmk_points_number_);
    int32_t *label_feature_begin = label_feature + label_feature_size * box_id;
    int32_t *offset_feature_begin =
        offset_feature + offset_feature_size * box_id;
    for (int kps_id = 0; kps_id < lmk_points_number_; ++kps_id) {
      // find the best position
      int max_w = 0;
      int max_h = 0;
      int max_score_before_shift = label_feature_begin[kps_id];
      int32_t *mxnet_out_for_one_point = nullptr;
      for (int hh = 0; hh < lmk_feat_height_; ++hh) {
        for (int ww = 0; ww < lmk_feat_width_; ++ww) {
          mxnet_out_for_one_point =
              label_feature_begin + hh * label_h_stride + ww * label_w_stride;
          if (mxnet_out_for_one_point[kps_id] > max_score_before_shift) {
            max_w = ww;
            max_h = hh;
            max_score_before_shift = mxnet_out_for_one_point[kps_id];
          }
        }
      }
      float max_score = GetFloatByInt(max_score_before_shift, label_shift);
      float base_x = (max_w * lmk_feat_stride_ + base_center) * scale_x + x1;
      float base_y = (max_h * lmk_feat_stride_ + base_center) * scale_y + y1;

      // get delta
      mxnet_out_for_one_point = offset_feature_begin + max_h * offset_h_stride +
                                max_w * offset_w_stride;
      auto x_delta = mxnet_out_for_one_point[2 * kps_id];
      float fp_delta_x = GetFloatByInt(x_delta, offset_shift);
      auto y_delta = mxnet_out_for_one_point[2 * kps_id + 1];
      float fp_delta_y = GetFloatByInt(y_delta, offset_shift);
      Point point;
      point.x_ = base_x + fp_delta_x * scale_pos_x;
      point.y_ = base_y + fp_delta_y * scale_pos_y;
      point.score_ = SigMoid(max_score);
      landmark.values_[kps_id] = point;
    }
    landmarks.push_back(std::move(landmark));
  }
}

void MultitaskPostProcessMethod::GetPose(BPU_MODEL_S bpu_model, int out_level,
                                         std::vector<Pose3D> &face_pose,
                                         void *output,
                                         const std::vector<BBox> &face_boxes) {
  size_t face_box_num = face_boxes.size();
  int32_t *pose_feature = reinterpret_cast<int32_t *>(output);

  auto aligned_dim = model_output_info_[out_level].aligned_dims;
  uint32_t shift = model_output_info_[out_level].shift;

  int feature_size = aligned_dim[1] * aligned_dim[2] * aligned_dim[3];

  for (size_t box_id = 0; box_id < face_box_num; ++box_id) {
    int32_t *mxnet_out_for_one_box = pose_feature + feature_size * box_id;
    Pose3D pose;
    pose.yaw_ = GetFloatByInt(mxnet_out_for_one_box[0], shift) * 90.0;
    LOGD << "pose.yaw: " << pose.yaw_;
    pose.pitch_ = GetFloatByInt(mxnet_out_for_one_box[1], shift) * 90.0;
    LOGD << "pose.pitch: " << pose.pitch_;
    pose.roll_ = GetFloatByInt(mxnet_out_for_one_box[2], shift) * 90.0;
    LOGD << "pose.roll: " << pose.roll_;
    face_pose.push_back(pose);
  }
}

void MultitaskPostProcessMethod::GetPlateAttribute(
    BPU_MODEL_S bpu_model, int out_level,
    std::vector<Attribute_<int>> &plate_attribute, void *output,
    const std::vector<BBox> &plate_boxes, const int attribute_num) {
  size_t plate_box_num = plate_boxes.size();
  int32_t *plate_attribute_feature = reinterpret_cast<int32_t *>(output);

  auto aligned_dim = model_output_info_[out_level].aligned_dims;
  int feature_size = aligned_dim[1] * aligned_dim[2] * aligned_dim[3];

  uint32_t shift = model_output_info_[out_level].shift;

  LOGD << "plate attribute: ";
  for (size_t box_id = 0; box_id < plate_box_num; ++box_id) {
    int32_t *mxnet_out_for_one_box =
        plate_attribute_feature + feature_size * box_id;

    Attribute_<int> one_plate_attribute;
    int max_index = 0;
    float max_score = -1000;
    for (int32_t attribute_index = 0; attribute_index < attribute_num;
         ++attribute_index) {
      float attribute_score =
          GetFloatByInt(mxnet_out_for_one_box[attribute_index], shift);
      if (attribute_score > max_score) {
        max_score = attribute_score;
        max_index = attribute_index;
      }
    }
    one_plate_attribute.value_ = max_index;
    one_plate_attribute.score_ = max_score;
    LOGD << "value: " << one_plate_attribute.value_
         << ", score: " << one_plate_attribute.score_;
    plate_attribute.push_back(one_plate_attribute);
  }
}

void MultitaskPostProcessMethod::Convert2FrameworkData(
    OutMsg &det_results, std::map<std::string, std::shared_ptr<BaseDataVector>>
                             &xstream_det_result) {
  // get landmark by landmark2 and landmark1.
  if (det_results.landmarks.find("landmark2") != det_results.landmarks.end() &&
      det_results.landmarks.find("landmark1") != det_results.landmarks.end()) {
    std::vector<Landmarks> vec_landmarks;
    auto &landmarks2 = det_results.landmarks["landmark2"];
    auto &landmarks1 = det_results.landmarks["landmark1"];
    HOBOT_CHECK(landmarks2.size() == landmarks1.size())
        << "landmarks2's size not equal to landmarks1's size.";
    for (size_t lmk_index = 0; lmk_index < landmarks2.size(); ++lmk_index) {
      Landmarks landmarks;
      auto &landmark2 = landmarks2[lmk_index];
      auto &landmark1 = landmarks1[lmk_index];
      HOBOT_CHECK(landmark2.values_.size() == landmark1.values_.size())
          << "landmark2's size not equal to landmark1's size.";
      for (size_t point_index = 0; point_index < landmark2.values_.size();
           ++point_index) {
        auto &landmark2_point = landmark2.values_[point_index];
        auto &landmark1_point = landmark1.values_[point_index];
        if (landmark2_point.score_ > 0.5) {
          landmarks.values_.push_back(std::move(landmark2_point));
        } else {
          landmarks.values_.push_back(std::move(landmark1_point));
        }
      }
      vec_landmarks.push_back(landmarks);
    }
    det_results.landmarks["landmark"] = vec_landmarks;
    det_results.landmarks.erase("landmark2");
    det_results.landmarks.erase("landmark1");
  }

  std::set<int> invalid_idx;
  // box
  for (const auto &boxes : det_results.boxes) {
    xstream_det_result[boxes.first] =
        std::make_shared<xstream::BaseDataVector>();
    xstream_det_result[boxes.first]->name_ = "rcnn_" + boxes.first;
    for (uint i = 0; i < boxes.second.size(); ++i) {
      const auto &box = boxes.second[i];
      if (box.score_ < bbox_threshold_) {
        LOGD << "current bbox's score: " << box.score_
             << " is smaller than threshold: " << bbox_threshold_;
        invalid_idx.emplace(i);
        continue;
      }
      if (boxes.first == "face_box") {
        if (box.score_ < face_pv_thr_) {
          invalid_idx.emplace(i);
          continue;
        }
      }
      auto xstream_box = std::make_shared<BBox>(box);
      xstream_det_result[boxes.first]->datas_.push_back(xstream_box);
      LOGD << boxes.first << ": " << box.x1_ << ", "
           << box.y1_ << ", " << box.x2_ << ", "
           << box.y2_ << ", " << box.score_;
    }
  }

  // landmark
  for (const auto &landmarks_vec : det_results.landmarks) {
    xstream_det_result[landmarks_vec.first] =
        std::make_shared<xstream::BaseDataVector>();
    xstream_det_result[landmarks_vec.first]->name_ =
        "rcnn_" + landmarks_vec.first;
    for (uint i = 0; i < landmarks_vec.second.size(); ++i) {
      if (invalid_idx.find(i) != invalid_idx.end()) {
        continue;
      }
      const auto &landmarks = landmarks_vec.second[i];
      LOGD << "lmk point: [";
      for (const auto &point : landmarks.values_) {
        LOGD << point.x_ << "," << point.y_ << "," << point.score_;
      }
      LOGD << "]";
      auto xstream_landmark = std::make_shared<Landmarks>(landmarks);
      xstream_det_result[landmarks_vec.first]->datas_.push_back(
          xstream_landmark);
    }
  }

  // feature
  for (const auto &feature_vec : det_results.features) {
    xstream_det_result[feature_vec.first] =
        std::make_shared<xstream::BaseDataVector>();
    xstream_det_result[feature_vec.first]->name_ = "rcnn_" + feature_vec.first;
    for (auto &feature : feature_vec.second) {
      auto xstream_feature = std::make_shared<FloatFeature>(feature);
      xstream_det_result[feature_vec.first]->datas_.push_back(xstream_feature);
    }
  }

  // segmentations
  for (const auto &segmentation_vec : det_results.segmentations) {
    xstream_det_result[segmentation_vec.first] =
        std::make_shared<xstream::BaseDataVector>();
    xstream_det_result[segmentation_vec.first]->name_ =
        "rcnn_" + segmentation_vec.first;
    for (auto &segmentation : segmentation_vec.second) {
      auto xstream_segmentation = std::make_shared<Segmentation>(segmentation);
      xstream_det_result[segmentation_vec.first]->datas_.push_back(
          xstream_segmentation);
    }
  }

  // poses
  for (const auto &pose_vec : det_results.poses) {
    xstream_det_result[pose_vec.first] =
        std::make_shared<xstream::BaseDataVector>();
    xstream_det_result[pose_vec.first]->name_ = "rcnn_" + pose_vec.first;
    for (uint i = 0; i < pose_vec.second.size(); ++i) {
      if (invalid_idx.find(i) != invalid_idx.end()) {
        continue;
      }
      const auto &pose = pose_vec.second[i];
      auto xstream_pose = std::make_shared<Pose3D>(pose);
      xstream_det_result[pose_vec.first]->datas_.push_back(xstream_pose);
    }
  }

  // attributes
  for (const auto &attribute_vec : det_results.attributes) {
    xstream_det_result[attribute_vec.first] =
        std::make_shared<xstream::BaseDataVector>();
    xstream_det_result[attribute_vec.first]->name_ =
        "rcnn_" + attribute_vec.first;
    for (auto &attribute : attribute_vec.second) {
      auto xstream_attribute = std::make_shared<Attribute_<int>>(attribute);
      xstream_det_result[attribute_vec.first]->datas_.push_back(
          xstream_attribute);
    }
  }
}

}  // namespace xstream
