/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of multitask_postprocess
 * @file   multitask_postprocess.cc
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.05.13
 */

#include "model_inference/postprocess/horizon_multitask_postprocess.h"
#include <numeric>
#include <set>
#include "model_inference/inference_engine.h"
#include "xstream/vision_type.h"
#include "xstream/xstream_world.h"
#include "json/json.h"

namespace inference {
// used to decide output info for each layer of model. first we should use hbcc
// interface to get model's info,
// and get each layer's output info, then config it in config file.
static std::map<std::string, FasterRCNNBranchOutType> str2faster_rcnn_out_type =
    {{"bbox", FasterRCNNBranchOutType::BBOX},
     {"kps", FasterRCNNBranchOutType::KPS},
     {"mask", FasterRCNNBranchOutType::MASK},
     {"reid", FasterRCNNBranchOutType::REID},
     {"lmks2_label", FasterRCNNBranchOutType::LMKS2_LABEL},
     {"lmks2_offset", FasterRCNNBranchOutType::LMKS2_OFFSET},
     {"lmks1", FasterRCNNBranchOutType::LMKS1},
     {"3d_pose", FasterRCNNBranchOutType::POSE_3D},
     {"plate_color", FasterRCNNBranchOutType::PLATE_COLOR},
     {"plate_row", FasterRCNNBranchOutType::PLATE_ROW},
     {"kps_label", FasterRCNNBranchOutType::KPS_LABEL},
     {"kps_offset", FasterRCNNBranchOutType::KPS_OFFSET},
     {"lmks", FasterRCNNBranchOutType::LMKS}};


// tensor convert
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

// utils func
inline float GetFloatByInt(int32_t value, uint32_t shift) {
  return (static_cast<float>(value)) / (static_cast<float>(1 << shift));
}

inline float SigMoid(const float &input) {
  return 1 / (1 + std::exp(-1 * input));
}

inline void L2Norm(xstream::FloatFeature &feature) {
  float sum = 0.0;
  sum = std::inner_product(feature.values_.begin(),
                           feature.values_.end(),
                           feature.values_.begin(), sum);
  float eps = 1e-10;
  sum = sqrt(sum) + eps;
  for (auto &value : feature.values_) {
    value = value / sum;
  }
  return;
}

int MultitaskPostProcess::Init(const std::string &json_str) {
  LOGD << "MultitaskPostProcess Init";
  // string转json
  Json::Reader Reader;
  Json::Value config;
  Reader.parse(json_str, config);

  Json::Value net_info;
  if (config["net_info"].isNull()) {
    LOGE << "not fount net_info in config";
    return -1;
  } else {
    net_info = config["net_info"];
  }
  std::vector<Json::Value> model_out_sequence;
  if (net_info["model_out_sequence"].isNull()) {
    LOGE << "not fount model_out_sequence in config";
    return -1;
  } else {
    model_out_sequence.resize(net_info["model_out_sequence"].size());
    for (Json::ArrayIndex i = 0;
         i < net_info["model_out_sequence"].size(); ++i) {
      model_out_sequence[i] = net_info["model_out_sequence"][i];
    }
  }

  // out_level2rcnn_branch_info_
  for (size_t i = 0; i < model_out_sequence.size(); ++i) {
    std::string type_str = model_out_sequence[i]["type"].asString();
    HOBOT_CHECK(!type_str.empty());
    if (str2faster_rcnn_out_type.find(type_str) ==
        str2faster_rcnn_out_type.end()) {
      out_level2rcnn_branch_info_[i].type = FasterRCNNBranchOutType::INVALID;
    } else {
      out_level2rcnn_branch_info_[i].type = str2faster_rcnn_out_type[type_str];
      out_level2rcnn_branch_info_[i].name =
          model_out_sequence[i]["name"].asString();
      out_level2rcnn_branch_info_[i].box_name =
          model_out_sequence[i]["box_name"].asString();
      if (!model_out_sequence[i]["labels"].isNull()) {
        for (Json::ArrayIndex i = 0;
             i < model_out_sequence[i]["labels"].size(); ++i) {
          auto pair = model_out_sequence[i]["labels"][i];
          auto label_idx = pair["key"].asInt();
          auto label_val = pair["value"].asString();
          out_level2rcnn_branch_info_[i].labels[label_idx] = label_val;
        }
      }
    }
  }

  HOBOT_CHECK(net_info["model_input_width"].isInt());
  model_input_width_ = net_info["model_input_width"].asInt();
  HOBOT_CHECK(net_info["model_input_height"].isInt());
  model_input_height_ = net_info["model_input_height"].asInt();

  HOBOT_CHECK(net_info["src_image_width"].isInt());
  src_image_width_ = net_info["src_image_width"].asInt();
  HOBOT_CHECK(net_info["src_image_height"].isInt());
  src_image_height_ = net_info["src_image_height"].asInt();

  kps_pos_distance_ = net_info["kps_pos_distance"].isNumeric() ?
       net_info["kps_pos_distance"].asFloat() : 0.1;
  kps_feat_width_ = net_info["kps_feat_width"].isNumeric() ?
      net_info["kps_feat_width"].asInt() : 16;
  kps_feat_height_ = net_info["kps_feat_height"].isNumeric() ?
      net_info["kps_feat_height"].asInt() : 16;
  kps_points_number_ = net_info["kps_points_number"].isNumeric() ?
      net_info["kps_points_number"].asInt() : 17;
  kps_feat_stride_ = net_info["kps_feat_stride"].isNumeric() ?
      net_info["kps_feat_stride"].asInt() : 16;
  kps_anchor_param_ = net_info["kps_anchor_param"].isNumeric() ?
      net_info["kps_anchor_param"].asFloat() : 0.0;

  lmk_feat_height_ = net_info["lmk_feat_height"].isNumeric() ?
      net_info["lmk_feat_height"].asInt() : 8;
  lmk_feat_width_ = net_info["lmk_feat_width"].isNumeric() ?
      net_info["lmk_feat_width"].asInt() : 8;
  lmk_feat_stride_ = net_info["lmk_feat_stride"].isNumeric() ?
      net_info["lmk_feat_stride"].asInt() : 16;
  lmk_points_number_ = net_info["lmk_points_number"].isNumeric() ?
      net_info["lmk_points_number"].asInt() : 5;
  lmk_pos_distance_ = net_info["lmk_pos_distance"].isNumeric() ?
       net_info["lmk_pos_distance"].asFloat() : 12;
  lmk_anchor_param_ = net_info["lmk_anchor_param"].isNumeric() ?
       net_info["lmk_anchor_param"].asFloat() : 0.0;
  face_pose_number_ = net_info["3d_pose_number"].isNumeric() ?
      net_info["3d_pose_number"].asInt() : 3;

  plate_color_num_ = net_info["plate_color_num"].isNumeric() ?
      net_info["plate_color_num"].asInt() : 6;
  plate_row_num_ = net_info["plate_row_num"].isNumeric() ?
       net_info["plate_row_num"].asInt() : 2;

  face_pv_thr_ = net_info["face_pv_thr"].isNumeric() ?
      net_info["face_pv_thr"].asFloat() : 0.0;

  // method_outs_
  HOBOT_CHECK(!config["method_outs"].isNull());
  method_outs_.resize(config["method_outs"].size());
  for (Json::ArrayIndex i = 0; i <method_outs_.size(); ++i) {
    method_outs_[i] = config["method_outs"][i].asString();
  }

  return 0;
}

int MultitaskPostProcess::GetModelInfo(
    const std::shared_ptr<InferenceEngineTask> task) {
  auto &output_tensors = task->output_tensors_;

  output_layer_ = output_tensors.size();

  for (size_t i = 0; i < output_layer_; ++i) {
    auto &output_tensor = output_tensors[i];
    auto &properties = output_tensor.properties;
    const auto &branch_info = out_level2rcnn_branch_info_[i];
    auto out_type = branch_info.type;
    // get shifts
    const uint8_t *shift_value = properties.shift.shift_data;
    // dim of per shape = 4
    int aligned_shape_dim = properties.aligned_shape.num_dimensions;
    HOBOT_CHECK(aligned_shape_dim == 4)
        << "aligned_shape_dim = " << aligned_shape_dim;
    // int *aligned_dim = new int[aligned_shape_dim];
    std::vector<int> align_dims_tmp(aligned_shape_dim);
    for (int dim = 0; dim < aligned_shape_dim; dim++) {
      align_dims_tmp[dim] = properties.aligned_shape.dimension_size[dim];
    }
    auto aligned_dim = std::make_shared<std::vector<int>>(align_dims_tmp);
    switch (out_type) {
      case FasterRCNNBranchOutType::INVALID:
        break;
      case FasterRCNNBranchOutType::KPS:
        // TODO(yaoyao.sun) pack into a function
        kps_shift_ = shift_value[0];
        aligned_kps_dim = aligned_dim;
        break;
      case FasterRCNNBranchOutType::KPS_LABEL:
        // TODO(yaoyao.sun) pack into a function
        kps_label_shift_ = shift_value[0];
        aligned_kps_label_dim = aligned_dim;
        break;
      case FasterRCNNBranchOutType::KPS_OFFSET:
        // TODO(yaoyao.sun) pack into a function
        kps_offset_shift_ = shift_value[0];
        aligned_kps_offset_dim = aligned_dim;
        break;
      case FasterRCNNBranchOutType::MASK:
        mask_shift_ = shift_value[0];
        aligned_mask_dim = aligned_dim;
        break;
      case FasterRCNNBranchOutType::REID:
        reid_shift_ = shift_value[0];
        aligned_reid_dim = aligned_dim;
        break;
      case FasterRCNNBranchOutType::LMKS:
        lmks_shift_ = shift_value[0];
        aligned_lmks_dim = aligned_dim;
        break;
      case FasterRCNNBranchOutType::LMKS2_LABEL:
        lmks2_label_shift_ = shift_value[0];
        aligned_lmks2_label_dim = aligned_dim;
        break;
      case FasterRCNNBranchOutType::LMKS2_OFFSET:
        lmks2_offset_shift_ = shift_value[0];
        aligned_lmks2_offset_dim = aligned_dim;
        break;
      case FasterRCNNBranchOutType::LMKS1:
        lmks1_shift_ = shift_value[0];
        aligned_lmks1_dim = aligned_dim;
        break;
      case FasterRCNNBranchOutType::POSE_3D:
        face_pose_shift_ = shift_value[0];
        aligned_face_pose_dim = aligned_dim;
        break;
      case FasterRCNNBranchOutType::PLATE_COLOR:
        plate_color_shift_ = shift_value[0];
        aligned_plate_color_dim = aligned_dim;
        HOBOT_CHECK(plate_color_num_ <= aligned_plate_color_dim->at(3));
        break;
      case FasterRCNNBranchOutType::PLATE_ROW:
        plate_row_shift_ = shift_value[0];
        aligned_plate_row_dim = aligned_dim;
        HOBOT_CHECK(plate_row_num_ <= aligned_plate_row_dim->at(3));
        break;
      default:
        break;
    }
  }
  return 0;
}

int MultitaskPostProcess::Execute(
    const std::vector<std::shared_ptr<InferenceEngineTask>> &tasks,
    std::vector<xstream::BaseDataPtr> *frame_result) {
  LOGD << "MultitaskPostProcess Execute";
  HOBOT_CHECK(tasks.size() == 1);
  auto task = tasks[0];

  if (!is_init_) {
    GetModelInfo(task);
    is_init_ = true;
  }

  for (size_t out_index = 0; out_index < method_outs_.size(); ++out_index) {
    frame_result->push_back(std::make_shared<xstream::BaseDataVector>());
  }

  FasterRCNNOutMsg det_result;

  void* lmk2_label_out_put = nullptr;
  void* lmk2_offset_out_put = nullptr;
  void* kps_label_out_put = nullptr;
  void* kps_offset_out_put = nullptr;

  // Tensors
  std::vector<Tensor> &output_bpu_tensors = task->output_tensors_;

  // PostProcess
  for (int out_level = 0; out_level < output_layer_; ++out_level) {
    const auto &branch_info = out_level2rcnn_branch_info_[out_level];
    auto out_type = branch_info.type;
    switch (out_type) {
      case FasterRCNNBranchOutType::INVALID:
        break;
      case FasterRCNNBranchOutType::BBOX:
        GetRppRects(output_bpu_tensors[out_level], task, det_result.boxes,
                    out_level, branch_info.name, branch_info.labels);
        break;
      case FasterRCNNBranchOutType::LMKS2_LABEL:
        lmk2_label_out_put = output_bpu_tensors[out_level].sys_mem[0].vir_addr;
        break;
      case FasterRCNNBranchOutType::LMKS2_OFFSET:
        lmk2_offset_out_put = output_bpu_tensors[out_level].sys_mem[0].vir_addr;
        break;
      case FasterRCNNBranchOutType::KPS_LABEL:
        kps_label_out_put = output_bpu_tensors[out_level].sys_mem[0].vir_addr;
        break;
      case FasterRCNNBranchOutType::KPS_OFFSET:
        kps_offset_out_put = output_bpu_tensors[out_level].sys_mem[0].vir_addr;
        break;
      default:
        break;
    }
  }

  for (int out_level = 0; out_level < output_layer_; ++out_level) {
    const auto &branch_info = out_level2rcnn_branch_info_[out_level];
    auto out_type = branch_info.type;
    switch (out_type) {
      case FasterRCNNBranchOutType::INVALID:
        break;
      case FasterRCNNBranchOutType::KPS:
        LOGD << "begin GetKps";
        GetKps(det_result.landmarks[branch_info.name],
               output_bpu_tensors[out_level].sys_mem[0].vir_addr,
               det_result.boxes[branch_info.box_name]);
        break;
      case FasterRCNNBranchOutType::KPS_LABEL:
        LOGD << "begin GetKps2";
        GetKps2(det_result.landmarks["kps"], kps_label_out_put,
                kps_offset_out_put, det_result.boxes[branch_info.box_name]);
        break;
      case FasterRCNNBranchOutType::MASK:
        LOGD << "begin GetMask";
        GetMask(det_result.segmentations[branch_info.name],
                output_bpu_tensors[out_level].sys_mem[0].vir_addr,
                det_result.boxes[branch_info.box_name]);
        break;
      case FasterRCNNBranchOutType::REID:
        LOGD << "begin GetReid";
        GetReid(det_result.features[branch_info.name],
                output_bpu_tensors[out_level].sys_mem[0].vir_addr,
                det_result.boxes[branch_info.box_name]);
        break;
      case FasterRCNNBranchOutType::LMKS:
        LOGD << "begin GetLMKS";
        GetLMKS(det_result.landmarks["landmark"],
                output_bpu_tensors[out_level].sys_mem[0].vir_addr,
                det_result.boxes[branch_info.box_name]);
        break;
      case FasterRCNNBranchOutType::LMKS2_LABEL:
        LOGD << "begin GetLMKS2";
        GetLMKS2(det_result.landmarks["landmark2"], lmk2_label_out_put,
                 lmk2_offset_out_put, det_result.boxes[branch_info.box_name]);
        break;
      case FasterRCNNBranchOutType::LMKS1:
        LOGD << "begin GetLMKS1";
        GetLMKS1(det_result.landmarks["landmark1"],
                 output_bpu_tensors[out_level].sys_mem[0].vir_addr,
                 det_result.boxes[branch_info.box_name]);
        break;
      case FasterRCNNBranchOutType::POSE_3D:
        LOGD << "begin GetPose";
        GetPose(det_result.poses[branch_info.name],
                output_bpu_tensors[out_level].sys_mem[0].vir_addr,
                det_result.boxes[branch_info.box_name]);
        break;
      case FasterRCNNBranchOutType::PLATE_COLOR:
        LOGD << "begin GetPlateColor";
        GetPlateAttribute(&(det_result.attributes[branch_info.name]),
                          output_bpu_tensors[out_level].sys_mem[0].vir_addr,
                          plate_color_shift_, aligned_plate_color_dim,
                          det_result.boxes[branch_info.box_name],
                          plate_color_num_);
        break;
      case FasterRCNNBranchOutType::PLATE_ROW:
        LOGD << "begin GetPlateRow";
        GetPlateAttribute(&(det_result.attributes[branch_info.name]),
                          output_bpu_tensors[out_level].sys_mem[0].vir_addr,
                          plate_row_shift_, aligned_plate_row_dim,
                          det_result.boxes[branch_info.box_name],
                          plate_row_num_);
        break;
      default:
        break;
    }
  }

  // fasterrcnn 不需转浮点，需要自行释放outputtensors
  InferenceEngine::GetInstance()->FreeTensor(task->output_tensors_);

  // CoordinateTransform
  CoordinateTransform(det_result);

  // debug
  for (auto &boxes : det_result.boxes) {
    LOGD << boxes.first << ", num: " << boxes.second.size();
    for (auto &box : boxes.second) {
      LOGD << box;
    }
  }

  // convert FasterRCNNOutMsg to xstream data structure
  GetResultMsg(det_result, *frame_result);

  return 0;
}
typedef struct BPU_BBOX_F32
{
  float left;
  float top;
  float right;
  float bottom;
  float score;
  float class_label;
} BPU_BBOX_F32;

void MultitaskPostProcess::GetRppRects(
    const Tensor &bpu_tensor,
    const std::shared_ptr<InferenceEngineTask> task,
    std::map<std::string, std::vector<xstream::BBox>> &boxes, int index,
    const std::string &name,
    const std::unordered_map<int, std::string> &labels) {
  size_t item_size = sizeof(BPU_BBOX_F32);
  float output_byte_size =
      *reinterpret_cast<float *>(bpu_tensor.sys_mem[0].vir_addr);
  float box_num = output_byte_size / item_size;
  auto box_ptr = reinterpret_cast<BPU_BBOX_F32 *>(
      reinterpret_cast<uintptr_t>(bpu_tensor.sys_mem[0].vir_addr) + item_size);
  int nBox = static_cast<int>(box_num);
  LOGD << "rpp box num: " << nBox;

  for (int i = 0; i < nBox; ++i) {
    xstream::BBox box;
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

void MultitaskPostProcess::GetKps(
    std::vector<xstream::Landmarks> &kpss, void* output,
    const std::vector<xstream::BBox> &body_boxes) {
  int32_t *kps_feature = reinterpret_cast<int32_t *>(output);

  float pos_distance = kps_pos_distance_ * kps_feat_width_;
  size_t body_box_num = body_boxes.size();
  int feature_size = aligned_kps_dim->at(1) *
                     aligned_kps_dim->at(2) *
                     aligned_kps_dim->at(3);
  int h_stride = aligned_kps_dim->at(2) * aligned_kps_dim->at(3);
  int w_stride = aligned_kps_dim->at(3);

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

    xstream::Landmarks skeleton;
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

      float max_score =
          GetFloatByInt(max_score_before_shift, kps_shift_);

      // get delta
      mxnet_out_for_one_point =
          mxnet_out_for_one_point_begin + max_h * h_stride + max_w * w_stride;
      const auto x_delta =
          mxnet_out_for_one_point[2 * kps_id + kps_points_number_];
      float fp_delta_x = GetFloatByInt(x_delta, kps_shift_) * pos_distance;

      const auto y_delta =
          mxnet_out_for_one_point[2 * kps_id + kps_points_number_ + 1];
      float fp_delta_y = GetFloatByInt(y_delta, kps_shift_) * pos_distance;

      xstream::Point point;
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

void MultitaskPostProcess::GetKps2(
    std::vector<xstream::Landmarks> &kpss,
    void* kps_label_output, void* kps_offset_output,
    const std::vector<xstream::BBox> &body_boxes) {
  int32_t *label_feature = reinterpret_cast<int32_t *>(kps_label_output);
  int32_t *offset_feature = reinterpret_cast<int32_t *>(kps_offset_output);

  int input_height = kps_feat_height_ * kps_feat_stride_;
  int input_width = kps_feat_width_ * kps_feat_stride_;
  float base_center = (kps_feat_stride_ - 1) / 2.0;

  int label_feature_size = aligned_kps_label_dim->at(1) *
                           aligned_kps_label_dim->at(2) *
                           aligned_kps_label_dim->at(3);
  int label_h_stride = aligned_kps_label_dim->at(2) *
                       aligned_kps_label_dim->at(3);
  int label_w_stride = aligned_kps_label_dim->at(3);

  int offset_feature_size = aligned_kps_offset_dim->at(1) *
                            aligned_kps_offset_dim->at(2) *
                            aligned_kps_offset_dim->at(3);
  int offset_h_stride = aligned_kps_offset_dim->at(2) *
                        aligned_kps_offset_dim->at(3);
  int offset_w_stride = aligned_kps_offset_dim->at(3);

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

    xstream::Landmarks skeleton;
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
      float max_score = GetFloatByInt(max_score_before_shift, kps_label_shift_);

      float base_x = (max_w * kps_feat_stride_ + base_center) * scale_x + x1;
      float base_y = (max_h * kps_feat_stride_ + base_center) * scale_y + y1;

      // get delta
      int32_t *offset_feature_begin =
          offset_feature + offset_feature_size * box_id;
      mxnet_out_for_one_point = offset_feature_begin + max_h * offset_h_stride +
                                max_w * offset_w_stride;

      const auto x_delta = mxnet_out_for_one_point[2 * kps_id];
      float fp_delta_x = GetFloatByInt(x_delta, kps_offset_shift_);

      const auto y_delta = mxnet_out_for_one_point[2 * kps_id + 1];
      float fp_delta_y = GetFloatByInt(y_delta, kps_offset_shift_);

      xstream::Point point;
      point.x_ = base_x + fp_delta_x * scale_pos_x;
      point.y_ = base_y + fp_delta_y * scale_pos_y;
      point.score_ = SigMoid(max_score);
      skeleton.values_[kps_id] = point;
    }
    kpss.push_back(std::move(skeleton));
  }
}

void MultitaskPostProcess::GetMask(
    std::vector<xstream::Segmentation> &masks, void* output,
    const std::vector<xstream::BBox> &body_boxes) {
  size_t body_box_num = body_boxes.size();
  int32_t *mask_feature = reinterpret_cast<int32_t *>(output);

  int feature_size = aligned_mask_dim->at(1) *
                     aligned_mask_dim->at(2) *
                     aligned_mask_dim->at(3);

  int h_stride = aligned_mask_dim->at(2) * aligned_mask_dim->at(3);
  int w_stride = aligned_mask_dim->at(3);

  for (size_t box_id = 0; box_id < body_box_num; ++box_id) {
    int32_t *mask_feature_begin = mask_feature + feature_size * box_id;
    int32_t *mxnet_out_for_one_box = nullptr;

    float fp_for_this_mask;
    xstream::Segmentation mask;
    for (int hh = 0; hh < aligned_mask_dim->at(1); ++hh) {
      for (int ww = 0; ww < aligned_mask_dim->at(2); ++ww) {
        mxnet_out_for_one_box =
            mask_feature_begin + hh * h_stride + ww * w_stride;
        fp_for_this_mask =
            GetFloatByInt(mxnet_out_for_one_box[0], mask_shift_);
        mask.values_.push_back(fp_for_this_mask);
      }
    }
    mask.height_ = aligned_mask_dim->at(1);
    mask.width_ = aligned_mask_dim->at(2);
    masks.push_back(std::move(mask));
  }
}

void MultitaskPostProcess::GetReid(
    std::vector<xstream::FloatFeature> &reids, void *output,
    const std::vector<xstream::BBox> &body_boxes) {
  size_t body_box_num = body_boxes.size();
  int32_t *reid_feature = reinterpret_cast<int32_t *>(output);

  int feature_size = aligned_reid_dim->at(1) *
                     aligned_reid_dim->at(2) *
                     aligned_reid_dim->at(3);

  for (size_t box_id = 0; box_id < body_box_num; ++box_id) {
    int32_t *mxnet_out_for_one_box = reid_feature + feature_size * box_id;
    float fp_for_this_reid;
    xstream::FloatFeature reid;
    for (int32_t reid_i = 0; reid_i < aligned_reid_dim->at(3); ++reid_i) {
      fp_for_this_reid =
          GetFloatByInt(mxnet_out_for_one_box[reid_i], reid_shift_);
      reid.values_.push_back(fp_for_this_reid);
    }
    // l2norm
    L2Norm(reid);
    reids.push_back(std::move(reid));
  }
}

void MultitaskPostProcess::GetLMKS(
    std::vector<xstream::Landmarks> &landmarks, void* output,
    const std::vector<xstream::BBox> &face_boxes) {
  int32_t *lmk_feature = reinterpret_cast<int32_t *>(output);

  float pos_distance = lmk_pos_distance_ * lmk_feat_width_;
  size_t face_box_num = face_boxes.size();
  int feature_size = aligned_lmks_dim->at(1) *
                     aligned_lmks_dim->at(2) *
                     aligned_lmks_dim->at(3);
  int h_stride = aligned_lmks_dim->at(2) * aligned_lmks_dim->at(3);
  int w_stride = aligned_lmks_dim->at(3);

  for (size_t box_id = 0; box_id < face_box_num; ++box_id) {
    const auto &face_box = face_boxes[box_id];
    float x1 = face_box.x1_;
    float y1 = face_box.y1_;
    float x2 = face_box.x2_;
    float y2 = face_box.y2_;
    float w = x2 - x1 + 1;
    float h = y2 - y1 + 1;

    float scale_x = lmk_feat_width_ / w;
    float scale_y = lmk_feat_height_ / h;

    xstream::Landmarks landmark;
    landmark.values_.resize(lmk_points_number_);

    for (int lmk_id = 0; lmk_id < lmk_points_number_; ++lmk_id) {
      int32_t *lmk_feature_begin = lmk_feature + feature_size * box_id;
      // find the best position
      int max_w = 0;
      int max_h = 0;
      int max_score_before_shift = lmk_feature_begin[lmk_id];
      int32_t *mxnet_out_for_one_point = nullptr;
      for (int hh = 0; hh < lmk_feat_height_; ++hh) {
        for (int ww = 0; ww < lmk_feat_width_; ++ww) {
          mxnet_out_for_one_point =
              lmk_feature_begin + hh * h_stride + ww * w_stride;
          if (mxnet_out_for_one_point[lmk_id] > max_score_before_shift) {
            max_w = ww;
            max_h = hh;
            max_score_before_shift = mxnet_out_for_one_point[lmk_id];
          }
        }
      }
      float max_score = GetFloatByInt(max_score_before_shift, lmks_shift_);

      // get delta
      mxnet_out_for_one_point =
          lmk_feature_begin + max_h * h_stride + max_w * w_stride;
      const auto x_delta =
          mxnet_out_for_one_point[2 * lmk_id + lmk_points_number_];
      float fp_delta_x = GetFloatByInt(x_delta, kps_shift_) * pos_distance;
      const auto y_delta =
          mxnet_out_for_one_point[2 * lmk_id + lmk_points_number_ + 1];
      float fp_delta_y = GetFloatByInt(y_delta, kps_shift_) * pos_distance;

      xstream::Point point;
      point.x_ =
          (max_w + fp_delta_x + 0.46875 + lmk_anchor_param_) / scale_x + x1;
      point.y_ =
          (max_h + fp_delta_y + 0.46875 + lmk_anchor_param_) / scale_y + y1;
      point.score_ = max_score;
      landmark.values_[lmk_id] = point;
    }
    landmarks.push_back(std::move(landmark));
  }
}

void MultitaskPostProcess::GetLMKS2(
    std::vector<xstream::Landmarks> &landmarks,
    void* lmks2_label_output,
    void* lmks2_offset_output,
    const std::vector<xstream::BBox> &face_boxes) {
  int32_t *label_feature = reinterpret_cast<int32_t *>(lmks2_label_output);
  int32_t *offset_feature = reinterpret_cast<int32_t *>(lmks2_offset_output);

  int input_height = lmk_feat_height_ * lmk_feat_stride_;
  int input_width = lmk_feat_width_ * lmk_feat_stride_;
  float base_center = (lmk_feat_stride_ - 1) / 2.0;
  size_t face_box_num = face_boxes.size();

  int label_feature_size = aligned_lmks2_label_dim->at(1) *
                           aligned_lmks2_label_dim->at(2) *
                           aligned_lmks2_label_dim->at(3);
  int label_h_stride = aligned_lmks2_label_dim->at(2) *
                       aligned_lmks2_label_dim->at(3);
  int label_w_stride = aligned_lmks2_label_dim->at(3);
  int offset_feature_size = aligned_lmks2_offset_dim->at(1) *
                            aligned_lmks2_offset_dim->at(2) *
                            aligned_lmks2_offset_dim->at(3);
  int offset_h_stride =
      aligned_lmks2_offset_dim->at(2) * aligned_lmks2_offset_dim->at(3);
  int offset_w_stride = aligned_lmks2_offset_dim->at(3);

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

    xstream::Landmarks landmark;
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
      float max_score = GetFloatByInt(max_score_before_shift,
                                      lmks2_label_shift_);
      float base_x = (max_w * lmk_feat_stride_ + base_center) * scale_x + x1;
      float base_y = (max_h * lmk_feat_stride_ + base_center) * scale_y + y1;

      // get delta
      mxnet_out_for_one_point = offset_feature_begin + max_h * offset_h_stride +
                                max_w * offset_w_stride;
      auto x_delta = mxnet_out_for_one_point[2 * kps_id];
      float fp_delta_x = GetFloatByInt(x_delta, lmks2_offset_shift_);
      auto y_delta = mxnet_out_for_one_point[2 * kps_id + 1];
      float fp_delta_y = GetFloatByInt(y_delta, lmks2_offset_shift_);
      xstream::Point point;
      point.x_ = base_x + fp_delta_x * scale_pos_x;
      point.y_ = base_y + fp_delta_y * scale_pos_y;
      point.score_ = SigMoid(max_score);
      landmark.values_[kps_id] = point;
    }
    landmarks.push_back(std::move(landmark));
  }
}

void MultitaskPostProcess::GetLMKS1(
    std::vector<xstream::Landmarks> &landmarks,
    void* output, const std::vector<xstream::BBox> &face_boxes) {
  size_t face_box_num = face_boxes.size();
  int32_t *lmks1_feature =
      reinterpret_cast<int32_t *>(output);

  int feature_size = aligned_lmks1_dim->at(1) *
                     aligned_lmks1_dim->at(2) *
                     aligned_lmks1_dim->at(3);

  for (size_t box_id = 0; box_id < face_box_num; ++box_id) {
    const auto &face_box = face_boxes[box_id];
    float x1 = face_box.x1_;
    float y1 = face_box.y1_;
    float x2 = face_box.x2_;
    float y2 = face_box.y2_;
    float w = x2 - x1 + 1;
    float h = y2 - y1 + 1;

    int32_t *mxnet_out_for_this_box = lmks1_feature + feature_size * box_id;

    xstream::Landmarks landmark;
    for (int i = 0; i < lmk_points_number_; ++i) {
      float x =
          GetFloatByInt(mxnet_out_for_this_box[2 * i], lmks1_shift_) * w + x1;
      float y =
          GetFloatByInt(mxnet_out_for_this_box[2 * i + 1], lmks1_shift_) * h +
          y1;
      landmark.values_.push_back(xstream::Point(x, y));
    }
    landmarks.push_back(std::move(landmark));
  }
}

void MultitaskPostProcess::GetPose(
    std::vector<xstream::Pose3D> &face_pose,
    void* output, const std::vector<xstream::BBox> &face_boxes) {
  size_t face_box_num = face_boxes.size();
  int32_t *pose_feature = reinterpret_cast<int32_t *>(output);

  int feature_size = aligned_face_pose_dim->at(1) *
                     aligned_face_pose_dim->at(2) *
                     aligned_face_pose_dim->at(3);

  for (size_t box_id = 0; box_id < face_box_num; ++box_id) {
    int32_t *mxnet_out_for_one_box = pose_feature + feature_size * box_id;
    xstream::Pose3D pose;
    pose.yaw_ =
        GetFloatByInt(mxnet_out_for_one_box[0], face_pose_shift_) * 90.0;
    LOGD << "pose.yaw: " << pose.yaw_;
    pose.pitch_ =
        GetFloatByInt(mxnet_out_for_one_box[1], face_pose_shift_) * 90.0;
    LOGD << "pose.pitch: " << pose.pitch_;
    pose.roll_ =
        GetFloatByInt(mxnet_out_for_one_box[2], face_pose_shift_) * 90.0;
    LOGD << "pose.roll: " << pose.roll_;
    face_pose.push_back(pose);
  }
}

void MultitaskPostProcess::GetPlateAttribute(
    std::vector<xstream::Attribute_<int>> *plates_attribute, void* output,
    uint32_t shift, std::shared_ptr<std::vector<int>> aligned_dim,
    const std::vector<xstream::BBox> &plate_boxes,
    int32_t attribute_num) {
  size_t plate_box_num = plate_boxes.size();
  int32_t *plate_attribute_feature = reinterpret_cast<int32_t *>(output);

  int feature_size = aligned_dim->at(1) * aligned_dim->at(2) *
                     aligned_dim->at(3);

  LOGD << "plate attribute: ";
  for (size_t box_id = 0; box_id < plate_box_num; ++box_id) {
    int32_t *mxnet_out_for_one_box =
        plate_attribute_feature + feature_size * box_id;

    xstream::Attribute_<int> one_plate_attribute;
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
         << ", score: " << one_plate_attribute.score_ << "\n";
    plates_attribute->push_back(one_plate_attribute);
  }
}

void MultitaskPostProcess::ConvertTensor2BPUTensor(
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

void MultitaskPostProcess::CoordinateTransform(
    FasterRCNNOutMsg &det_result) {
  for (auto &boxes : det_result.boxes) {
    for (auto &box : boxes.second) {
      box.x1_ = box.x1_ * src_image_width_ / model_input_width_;
      box.y1_ = box.y1_ * src_image_height_ / model_input_height_;
      box.x2_ = box.x2_ * src_image_width_ / model_input_width_;
      box.y2_ = box.y2_ * src_image_height_ / model_input_height_;
    }
  }

  for (auto &landmarks : det_result.landmarks) {
    for (auto &landmark : landmarks.second) {
      for (auto &point : landmark.values_) {
        point.x_ = point.x_ * src_image_width_ / model_input_width_;
        point.y_ = point.y_ * src_image_height_ / model_input_height_;
      }
    }
  }
}

void MultitaskPostProcess::GetResultMsg(
    FasterRCNNOutMsg &det_result,
    std::vector<xstream::BaseDataPtr> &frame_output) {
  std::map<std::string,
      std::shared_ptr<xstream::BaseDataVector>> xstream_det_result;
  // get landmark by landmark2 and landmark1.
  if (det_result.landmarks.find("landmark2") != det_result.landmarks.end() &&
      det_result.landmarks.find("landmark1") != det_result.landmarks.end()) {
    std::vector<xstream::Landmarks> vec_landmarks;
    auto &landmarks2 = det_result.landmarks["landmark2"];
    auto &landmarks1 = det_result.landmarks["landmark1"];
    HOBOT_CHECK(landmarks2.size() == landmarks1.size())
        << "landmarks2's size not equal to landmarks1's size.";
    for (size_t lmk_index = 0; lmk_index < landmarks2.size(); ++lmk_index) {
      xstream::Landmarks landmarks;
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
    det_result.landmarks["landmark"] = vec_landmarks;
    det_result.landmarks.erase("landmark2");
    det_result.landmarks.erase("landmark1");
  }

  std::set<int> invalid_idx;
  // box
  for (const auto &boxes : det_result.boxes) {
    xstream_det_result[boxes.first] =
        std::make_shared<xstream::BaseDataVector>();
    xstream_det_result[boxes.first]->name_ = "rcnn_" + boxes.first;
    for (uint i = 0; i < boxes.second.size(); ++i) {
      const auto &box = boxes.second[i];
      if (boxes.first == "face_box") {
        if (box.score_ < face_pv_thr_) {
          invalid_idx.emplace(i);
          continue;
        }
      }
      auto xstream_box = std::make_shared<xstream::BBox>();
      *xstream_box = std::move(box);
      xstream_det_result[boxes.first]->datas_.push_back(xstream_box);
    }
  }

  // landmark
  for (const auto &landmarks_vec : det_result.landmarks) {
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
      auto xstream_landmark = std::make_shared<xstream::Landmarks>();
      *xstream_landmark = std::move(landmarks);
      xstream_det_result[landmarks_vec.first]->datas_.push_back(
          xstream_landmark);
    }
  }

  // feature
  for (const auto &feature_vec : det_result.features) {
    xstream_det_result[feature_vec.first] =
        std::make_shared<xstream::BaseDataVector>();
    xstream_det_result[feature_vec.first]->name_ = "rcnn_" + feature_vec.first;
    for (auto &feature : feature_vec.second) {
      auto xstream_feature = std::make_shared<xstream::FloatFeature>();
      *xstream_feature = std::move(feature);
      xstream_det_result[feature_vec.first]->datas_.push_back(xstream_feature);
    }
  }

  // segmentations
  for (const auto &segmentation_vec : det_result.segmentations) {
    xstream_det_result[segmentation_vec.first] =
        std::make_shared<xstream::BaseDataVector>();
    xstream_det_result[segmentation_vec.first]->name_ =
        "rcnn_" + segmentation_vec.first;
    for (auto &segmentation : segmentation_vec.second) {
      auto xstream_segmentation = std::make_shared<xstream::Segmentation>();
      *xstream_segmentation = std::move(segmentation);
      xstream_det_result[segmentation_vec.first]->datas_.push_back(
          xstream_segmentation);
    }
  }

  // poses
  for (const auto &pose_vec : det_result.poses) {
    xstream_det_result[pose_vec.first] =
        std::make_shared<xstream::BaseDataVector>();
    xstream_det_result[pose_vec.first]->name_ = "rcnn_" + pose_vec.first;
    for (uint i = 0; i < pose_vec.second.size(); ++i) {
      if (invalid_idx.find(i) != invalid_idx.end()) {
        continue;
      }
      const auto &pose = pose_vec.second[i];
      auto xstream_pose = std::make_shared<xstream::Pose3D>();
      *xstream_pose = std::move(pose);
      xstream_det_result[pose_vec.first]->datas_.push_back(xstream_pose);
    }
  }

  // attributes
  for (const auto &attribute_vec : det_result.attributes) {
    xstream_det_result[attribute_vec.first] =
        std::make_shared<xstream::BaseDataVector>();
    xstream_det_result[attribute_vec.first]->name_ =
        "rcnn_" + attribute_vec.first;
    for (auto &attribute : attribute_vec.second) {
      auto xstream_attribute = std::make_shared<xstream::Attribute_<int>>();
      *xstream_attribute = std::move(attribute);
      xstream_det_result[attribute_vec.first]->datas_.push_back(
          xstream_attribute);
    }
  }

  for (size_t out_index = 0; out_index < method_outs_.size(); ++out_index) {
    if (xstream_det_result[method_outs_[out_index]]) {
      frame_output[out_index] = xstream_det_result[method_outs_[out_index]];
    }
  }
}

}  // namespace inference
