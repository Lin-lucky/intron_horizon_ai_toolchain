/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @brief     the implementation of FaceSnapFilterMethod.h
 * @author    hangjun.yang
 * @email     hangjun.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2010.01.11
 */

#include <map>
#include <cassert>
#include <vector>
#include <string>
#include <fstream>
#include <algorithm>
#include <memory>
#include <json/json.h>

#include "hobotlog/hobotlog.hpp"
#include "facesnap_filter_method/facesnap_filter_method.h"
#include "facesnap_filter_method/face_snap_filter_data_type.hpp"

namespace xstream {

int FaceSnapFilterMethod::Init(const std::string &config_file_path) {
  LOGI << "FaceSnapFilterMethod::Init " << config_file_path << std::endl;
  std::ifstream config_if(config_file_path);
  if (!config_if.good()) {
    LOGI << "FaceSnapFilterParam: no config, "
            "using default parameters" << std::endl;
    filter_param_ = std::make_shared<FaceSnapFilterParam>();
  } else {
    std::ostringstream buf;
    char ch;
    while (buf && config_if.get(ch)) {
      buf.put(ch);
    }
    filter_param_ = std::make_shared<FaceSnapFilterParam>(buf.str());
  }

  filter_param_->config_jv_all_ = filter_param_->config_jv;
  return 0;
}

#define FILTER_LOG(index, item)                                      \
  LOGD << "item index:" << index << " filtered by " << item;

#define FILTER_LOG_VALUE(index, item, value)                         \
  LOGD << "item index:" << index                                     \
  << " filtered by " << item << " with the value: " << value;

std::vector<BaseDataPtr> FaceSnapFilterMethod::DoProcess(\
      const std::vector<BaseDataPtr> &input,
      const xstream::InputParamPtr &param) {
  std::vector<BaseDataPtr> ret;
  ProcessOneBatch(input, &ret, param);
  return ret;
}

void FaceSnapFilterMethod::ProcessOneBatch(
                                  const std::vector<BaseDataPtr> &batch_i,
                                  std::vector<BaseDataPtr> *p_frame_output,
                                  const std::shared_ptr<InputParam> &param_i) {
  int frame_input_size = batch_i.size();
  // at least boxes
  HOBOT_CHECK(frame_input_size >= 1);

  // get face rect size & input slot
  uint16_t face_size = 0;
  std::vector<BaseDataVectorPtr> input_slot;
  input_slot.resize(frame_input_size);
  for (int i = 0; i < frame_input_size; ++i) {
    auto data_vector = \
            std::static_pointer_cast<BaseDataVector>(batch_i[i]);
    if (face_size == 0) {
      face_size = data_vector->datas_.size();
    } else {
      HOBOT_CHECK(face_size == data_vector->datas_.size());
    }
    input_slot[i] = data_vector;
  }

  // generate the output slot, add copy the input data
  std::vector<BaseDataVectorPtr> output_slot;

  // the first slot used to describe the filter condition
  output_slot.resize(frame_input_size + 1);

  bool do_filter = true;

  if (param_i) {
    if (param_i->is_json_format_) {
      int param_ret = UpdateParameter(param_i);
      if (param_ret != 0)  return;
    }
    if (param_i->Format() == "pass-through") {
      LOGI << "pass-through mode";
      do_filter = false;
    }
  }
  if (do_filter) {
    LOGI << "filter mode";
    for (int i = 0; i < frame_input_size; ++i) {
      HOBOT_CHECK("BaseDataVector" == batch_i[i]->type_) << "idx: " << i;
    }
    Copy2Output(input_slot, &output_slot, false);
    // do filter
    AttributeFilter(frame_input_size, face_size, input_slot, output_slot);
    // final : big face mode
    BigFaceFilter(frame_input_size, face_size, &output_slot);
  } else {
    Copy2Output(input_slot, &output_slot, true);
  }
  auto &frame_output = *p_frame_output;
  // compose output
  for (const auto &i : output_slot) {
    frame_output.push_back(\
        std::static_pointer_cast<BaseData>(i));
  }
}

void FaceSnapFilterMethod::AttributeFilter(
    int frame_input_size, int face_size,
    const std::vector<BaseDataVectorPtr> &input_slot,
    const std::vector<BaseDataVectorPtr> &output_slot) {
  for (int face_idx = 0; face_idx < face_size; ++face_idx) {
    int valid_code = isValid(input_slot, face_idx);
    if (valid_code == filter_param_->passed_err_code) {
      continue;
    } else {
      auto desp_sp = output_slot[0]->datas_[face_idx];
      desp_sp->state_ = DataState::FILTERED;
      auto xroc_desp = dynamic_cast<Attribute*>(desp_sp.get());
      xroc_desp->value_ = valid_code;

      // this face_confidence will be filtered
      for (int i = 1; i < frame_input_size + 1; ++i) {
        if (filter_param_->filter_status == 4) {
          output_slot[i]->datas_[face_idx]->state_ = DataState::INVALID;
        } else {
          output_slot[i]->datas_[face_idx]->state_ = DataState::FILTERED;
        }
      }
    }
  }
}

void FaceSnapFilterMethod::BigFaceFilter(int frame_input_size, int face_size,
    std::vector<BaseDataVectorPtr> *p_output_slot) const {
  auto &output_slot = *p_output_slot;
  if (filter_param_->max_box_counts) {
    std::vector<size_t> sorted_indexs;
    for (auto index = 0; index < face_size; ++index) {
      if (output_slot[1]->datas_[index]->state_ == DataState::VALID) {
        sorted_indexs.push_back(index);
      }
    }
    auto face_box_num = std::min(filter_param_->max_box_counts,
                                 static_cast<int>(sorted_indexs.size()));
    std::partial_sort(sorted_indexs.begin(),
                      sorted_indexs.begin() + face_box_num,
                      sorted_indexs.end(),
                      [&output_slot](size_t lhs, size_t rhs) {
                        auto face_box_lhs =
                            std::static_pointer_cast<BBox>(
                                output_slot[1]->datas_[lhs]);
                        auto face_box_rhs =
                            std::static_pointer_cast<BBox>(
                                output_slot[1]->datas_[rhs]);
                        return face_box_lhs->Width()
                            * face_box_lhs->Height()
                            > face_box_rhs->Width()
                                * face_box_rhs->Height();
                      });
    for (size_t face_idx = face_box_num; face_idx < sorted_indexs.size();
         ++face_idx) {
      auto desp_sp = output_slot[0]->datas_[sorted_indexs[face_idx]];
      auto xroc_desp = dynamic_cast<Attribute*>(desp_sp.get());
      xroc_desp->value_ = filter_param_->big_face_err_code;
      for (int i = 0; i < frame_input_size + 1; ++i) {
        FILTER_LOG(sorted_indexs[face_idx], "big face mode")
        output_slot[i]->datas_[sorted_indexs[face_idx]]->state_ =
            DataState::FILTERED;
      }
    }
  }
}

int FaceSnapFilterMethod::isValid(
    const std::vector<BaseDataVectorPtr> &input_slot, int face_idx) {
  auto frame_input_size = input_slot.size();
  // input: [face_box, pose, landmark, blue, brightness, eye_abnormalities,
  //         mouth_abnormal, left_eye, right_eye, left_brow, right_brow,
  //         forehead, left_cheek, right_check, nose, mouth, jaw]
  auto face_box =
      std::static_pointer_cast<BBox>(input_slot[0]->datas_[face_idx]);
  float x1 = face_box->x1_;
  float y1 = face_box->y1_;
  float x2 = face_box->x2_;
  float y2 = face_box->y2_;
  float box_score = face_box->score_;

  // judge filter
  bool size = ReachSizeThreshold(x1, y1, x2, y2);

  bool expand = ExpandThreshold(face_box.get());
  bool has_pose = false, has_lmk = false;
  if (frame_input_size > 1) {
    has_pose = true;
  }
  if (frame_input_size > 2) {
    has_lmk = true;
  }

  float pitch = 0, yaw = 0, roll = 0, quality_score = 0;

  bool pos = !has_pose;
  if (has_pose) {
    auto pose = std::static_pointer_cast<Pose3D>(
        input_slot[1]->datas_[face_idx]);
    if (pose->state_ == DataState::VALID) {
      pitch = pose->pitch_;
      yaw = pose->yaw_;
      roll = pose->roll_;
      pos = ReachPoseThreshold(pitch, yaw, roll);
    }
  }

  bool lmk_verification = !has_lmk;
  if (has_lmk) {
    auto lmk = std::static_pointer_cast<Landmarks>
        (input_slot[2]->datas_[face_idx]);
    if (lmk->state_ == DataState::VALID) {
      lmk_verification = LmkVerification(lmk);
    }
  }

  bool face_confidence = PassPostVerification(box_score);
  bool is_within_snap_area = IsWithinSnapArea(x1,
                                              y1,
                                              x2,
                                              y2,
                                              filter_param_->image_width,
                                              filter_param_->image_height);
  bool is_within_blacklist = IsWithinBlackListArea(x1, y1, x2, y2);
  bool is_within_whitelist = IsWithinWhiteListArea(x1, y1, x2, y2);

  bool quality_verification = true;
  if (input_slot.size() > 3) {
    auto quality =
        std::static_pointer_cast<Quality>(input_slot[3]->datas_[face_idx]);
    quality_score =  quality->score_;
    quality_verification = ReachQualityThreshold(quality->score_);
  }

  int brightness_val = 0;
  bool brightness_valid = true;
  if (input_slot.size() > 4) {
    auto brightness = std::static_pointer_cast<Attribute>(
            input_slot[4]->datas_[face_idx]);
    brightness_val = brightness->value_;
    brightness_valid = ValidBrightness(brightness_val);
  }
  if (is_within_blacklist) {
    FILTER_LOG(face_idx, "blacklist area")
    return filter_param_->black_list_err_code;
  }
  if (!is_within_whitelist) {
    FILTER_LOG(face_idx, "whitelist area")
    return filter_param_->white_list_err_code;
  }
  if (!face_confidence) {
    FILTER_LOG_VALUE(face_idx, "face_confidence", box_score)
    return filter_param_->pv_thr_err_code;
  }
  if (!size) {
    FILTER_LOG_VALUE(face_idx, "size", size)
    return filter_param_->snap_size_thr_err_code;
  }
  if (!is_within_snap_area) {
    FILTER_LOG(face_idx, "bound")
    return filter_param_->snap_area_err_code;
  }
  if (!expand) {
    FILTER_LOG(face_idx, "expand")
    return filter_param_->expand_thr_err_code;
  }

  if (!pos) {
    std::string frontal_str = "pitch: " + std::to_string(pitch)
        + " yaw: " + std::to_string(yaw)
        + " roll: " + std::to_string(roll);
    FILTER_LOG_VALUE(face_idx, "frontal area", frontal_str)
    return filter_param_->frontal_thr_err_code;
  }
  if (!quality_verification) {
    if (filter_param_->light_mode_ == "NormalLight") {
      FILTER_LOG_VALUE(face_idx, "quality", quality_score)
      return filter_param_->quality_thr_err_code;
    } else {
      FILTER_LOG_VALUE(face_idx, "quality_dark", quality_score)
      return filter_param_->quality_dark_thr_err_code;
    }
  }
  bool passed_occluded_condition = true;
  if (input_slot.size() > 7) {
    if (filter_param_->light_mode_ == "NormalLight") {
      // Normal light mode
      for (size_t i = 7; i < input_slot.size(); i++) {
        auto quality = std::static_pointer_cast<Attribute>(
            input_slot[i]->datas_[face_idx]);
        std::string occlude_name = input_data_types_[i];
        auto thr = GetOccludeVal(occlude_name);
        passed_occluded_condition =
            !IsOccluded(quality->score_, thr) && passed_occluded_condition;
        if (!passed_occluded_condition) {
          FILTER_LOG_VALUE(face_idx, occlude_name, quality->score_)
          return GetOccludeErrCode(occlude_name);
        }
      }
    } else {
      // Dark light or back light mode
      for (size_t i = 7; i < input_slot.size(); i++) {
        auto quality = std::static_pointer_cast<Attribute>(
            input_slot[i]->datas_[face_idx]);
        std::string occlude_name = input_data_dark_types_[i];
        auto thr = GetOccludeVal(occlude_name);
        passed_occluded_condition =
            !IsOccluded(quality->score_, thr) && passed_occluded_condition;
        if (!passed_occluded_condition) {
          FILTER_LOG_VALUE(face_idx, occlude_name, quality->score_)
          return GetOccludeErrCode(occlude_name);
        }
      }
    }
  }
  if (!lmk_verification) {
    FILTER_LOG(face_idx, "landmark")
    return filter_param_->lmk_thr_err_code;
  }
  if (!brightness_valid) {
    FILTER_LOG_VALUE(face_idx, "brightness", brightness_val)
    return filter_param_->brightness_err_code;
  }
  bool passed_abnormal_condition = true;
  if (input_slot.size() > 6) {
    for (size_t i = 5; i < 7; i++) {
      auto quality = std::static_pointer_cast<Attribute>(
          input_slot[i]->datas_[face_idx]);
      passed_abnormal_condition =
          !IsAbnormal(quality->score_) && passed_abnormal_condition;
      if (!passed_abnormal_condition) {
        FILTER_LOG_VALUE(face_idx, input_data_types_[i], quality->score_)
        return filter_param_->abnormal_thr_err_code;
      }
    }
  }
  return filter_param_->passed_err_code;
}

void FaceSnapFilterMethod::Copy2Output(
              const std::vector<BaseDataVectorPtr> &input_slot,
              std::vector<BaseDataVectorPtr> *p_output_slot,
              bool pass_through) {
  auto frame_input_size = input_slot.size();
  auto &output_slot = *p_output_slot;
  if (pass_through) {
    for (size_t i = 0; i < frame_input_size; ++i) {
      output_slot[i + 1] = input_slot[i];
    }
  } else {
    if (frame_input_size > 0) {
      output_slot[0] = ConstructFilterOutputSlot0(input_slot[0]->datas_.size());
      output_slot[1] = CopyAttribute<BBox>(input_slot[0]);
    }
    if (frame_input_size > 1) {
      output_slot[2] = CopyAttribute<Pose3D>(input_slot[1]);
    }
    if (frame_input_size > 2) {
      output_slot[3] = CopyAttribute<Landmarks>(input_slot[2]);
    }
    for (size_t i = 3; i < frame_input_size; ++i) {
      output_slot[i + 1] = CopyAttribute(input_slot[i], input_data_types_[i]);
    }
  }
}

void FaceSnapFilterMethod::Finalize() {}

BaseDataVectorPtr FaceSnapFilterMethod::CopyAttribute(\
              const BaseDataVectorPtr &attr,
              const std::string &data_type) {
  BaseDataVectorPtr ret = std::make_shared<BaseDataVector>();
  int attr_size = attr->datas_.size();
  if (0 == attr_size) {
    return ret;
  }

  for (const auto& data : attr->datas_) {
    if ("blur" == data_type) {
      auto actual_data = std::static_pointer_cast<Quality>(data);
      std::shared_ptr<Quality> copy_data = std::make_shared<Quality>();
      *copy_data = *actual_data;
      ret->datas_.push_back(std::static_pointer_cast<BaseData>(copy_data));
    } else if ("age" == data_type) {
      auto actual_data = std::static_pointer_cast<Age>(data);
      std::shared_ptr<Age> copy_data = std::make_shared<Age>();
      *copy_data = *actual_data;
      ret->datas_.push_back(std::static_pointer_cast<BaseData>(copy_data));
    } else if ("gender" == data_type) {
      auto actual_data = std::static_pointer_cast<Gender>(data);
      std::shared_ptr<Gender> copy_data = std::make_shared<Gender>();
      *copy_data = *actual_data;
      ret->datas_.push_back(std::static_pointer_cast<BaseData>(copy_data));
    } else {
      auto actual_data = std::static_pointer_cast<Attribute>(data);
      std::shared_ptr<Attribute> copy_data =
          std::make_shared<Attribute>();
      *copy_data = *actual_data;
      ret->datas_.push_back(std::static_pointer_cast<BaseData>(copy_data));
    }
  }
  return ret;
}

bool FaceSnapFilterMethod::ReachPoseThreshold(\
                          const float &pitch,
                          const float &yaw,
                          const float &roll) {
  if (filter_param_->filter_with_frontal_thr)
  {
    // adapter x1 config
    int32_t pose = 2000 - (yaw * yaw / 16 + pitch * pitch / 9) * 10;
    if (pose < -1999) {
      pose = -1999;
    }
    return pose > filter_param_->frontal_thr;
  }

  float pitch_, yaw_, roll_, a, b, c;
  if (filter_param_->frontal_pitch_thr == 0) {
    pitch_ = 0;
    a = 1;
    LOGV << "pitch filter off";
  } else {
    pitch_ = ValueNorm(-90.f, 90.f, pitch);
    a = filter_param_->frontal_pitch_thr;
  }
  if (filter_param_->frontal_yaw_thr == 0) {
    yaw_ = 0;
    b = 1;
    LOGV << "yaw filter off";
  } else {
    yaw_ = ValueNorm(-90.f, 90.f, yaw);
    b = filter_param_->frontal_yaw_thr;
  }
  if (filter_param_->frontal_roll_thr == 0) {
    roll_ = 0;
    c = 1;
    LOGV << "roll filter off";
  } else {
    roll_ = ValueNorm(-90.f, 90.f, roll);
    c = filter_param_->frontal_roll_thr;
  }
  HOBOT_CHECK(a != 0) << "frontal_pitch_thr could not equal to " << a;
  HOBOT_CHECK(b != 0) << "frontal_yaw_thr could not equal to " << b;
  HOBOT_CHECK(c != 0) << "frontal_roll_thr could not equal to " << c;
  float face_frontal = pitch_ * pitch_ / (a * a)
                     + yaw_ * yaw_ / (b * b)
                     + roll_ * roll_ / (c * c);
  return face_frontal <= 1;
}

bool FaceSnapFilterMethod::ReachSizeThreshold(const float &x1,
                                              const float &y1,
                                              const float &x2,
                                              const float &y2) {
  float width = x2 - x1;
  float height = y2 - y1;
  return (width > filter_param_->snap_size_thr)
      && (height > filter_param_->snap_size_thr);
}

bool FaceSnapFilterMethod::ReachQualityThreshold(const float &quality) {
  if (filter_param_->light_mode_ == "NormalLight") {
    return quality < filter_param_->quality_thr;
  } else {
    return quality < filter_param_->quality_dark_thr;
  }
}

bool FaceSnapFilterMethod::ValidBrightness(const int &brightness) {
  return brightness >= filter_param_->brightness_min
        && brightness <= filter_param_->brightness_max;
}

bool FaceSnapFilterMethod::PassPostVerification(const float &bbox_score) {
  return bbox_score > filter_param_->pv_thr;
}

bool FaceSnapFilterMethod::IsOccluded(const float &val, const float &thr) {
  // occluded condition: val is larger than the thr
  // smaller value means better quality
  return val > thr;
}

bool FaceSnapFilterMethod::IsAbnormal(const float &val) {
  // abnormal condition: val is larger than the thr
  // smaller value means better abnormalities
  return val > filter_param_->abnormal_thr;
}

bool FaceSnapFilterMethod::LmkVerification(\
                        const std::shared_ptr<Landmarks> &lmk) {
  assert(lmk->values_.size() == 5);
  if (filter_param_->lmk_filter_num > 0) {
    int filter_num = 0;
    for (size_t i = 0; i < 5; ++i) {
      if (lmk->values_.at(i).score_ < filter_param_->lmk_thr[i]) {
        if (++ filter_num >= filter_param_->lmk_filter_num) {
          return false;
        }
      }
      if (static_cast<int>(lmk->values_.at(i).x_) >
          filter_param_->image_width ||
          static_cast<int>(lmk->values_.at(i).x_) < 0 ||
          static_cast<int>(lmk->values_.at(i).y_) < 0 ||
          static_cast<int>(lmk->values_.at(i).y_) >
          filter_param_->image_height)
        return false;
    }
  } else {
    for (size_t i = 0; i < 5; ++i) {
      if (lmk->values_.at(i).score_ < filter_param_->lmk_thr[i]) {
        return false;
      }
      if (static_cast<int>(lmk->values_.at(i).x_) >
          filter_param_->image_width ||
          static_cast<int>(lmk->values_.at(i).x_) < 0 ||
          static_cast<int>(lmk->values_.at(i).y_) < 0 ||
          static_cast<int>(lmk->values_.at(i).y_) >
              filter_param_->image_height)
        return false;
    }
  }
  return true;
}

bool FaceSnapFilterMethod::IsWithinSnapArea(const float &x1,
                                            const float &y1,
                                            const float &x2,
                                            const float &y2,
                                            const int &image_width,
                                            const int &image_height) {
  return (x1 > filter_param_->bound_thr_w)
      && ((image_width - x2) > filter_param_->bound_thr_w)
      && (y1 > filter_param_->bound_thr_h)
      && ((image_height - y2) > filter_param_->bound_thr_h);
}

bool FaceSnapFilterMethod::IsWithinBlackListArea(const float &x1,
                                                 const float &y1,
                                                 const float &x2,
                                                 const float &y2) {
  if (filter_param_->black_list_module.GetBlackAreaList().empty()) {
    return false;
  }
  for (auto &black_area : \
           filter_param_->black_list_module.GetBlackAreaList()) {
    if (filter_param_->black_list_module.IsSameBBox(x1,
                                                    y1,
                                                    x2,
                                                    y2,
                                                    black_area)) {
      return true;
    }
  }
  return false;
}

bool FaceSnapFilterMethod::IsWithinWhiteListArea(const float &x1,
                                                 const float &y1,
                                                 const float &x2,
                                                 const float &y2) {
  if (filter_param_->white_list_module.GetWhiteAreaList().empty()) {
    return true;
  }

  for (const auto &white_area : \
           filter_param_->white_list_module.GetWhiteAreaList()) {
    if (filter_param_->white_list_module.IsInZone(x1,
                                                  y1,
                                                  x2,
                                                  y2,
                                                  white_area)) {
      return true;
    }
  }
  return false;
}

bool FaceSnapFilterMethod::ExpandThreshold(BBox *box) {
  auto ret = NormalizeRoi(box, filter_param_->expand_scale,
                          filter_param_->e_norm_method,
                          filter_param_->image_width,
                          filter_param_->image_height);
  return !ret;
}

int FaceSnapFilterMethod::UpdateParameter(InputParamPtr ptr) {
  if (ptr->is_json_format_) {
    std::string content = ptr->Format();
    return filter_param_->UpdateParameter(content);
  } else {
    HOBOT_CHECK(0) << "only support json format config";
    return -1;
  }
}

InputParamPtr FaceSnapFilterMethod::GetParameter() const {
  return filter_param_;
}

BaseDataVectorPtr FaceSnapFilterMethod::ConstructFilterOutputSlot0(
    const size_t &num) {
  BaseDataVectorPtr ret = std::make_shared<BaseDataVector>();
  for (size_t i = 0; i < num; i++) {
    auto data = std::make_shared<Attribute>();
    data->value_ = filter_param_->passed_err_code;
    ret->datas_.emplace_back(data);
  }
  return ret;
}

int FaceSnapFilterMethod::NormalizeRoi(BBox *src,
                                       float norm_ratio,
                                       NormMethod norm_method,
                                       uint32_t total_w,
                                       uint32_t total_h) {
  if (std::fabs(norm_ratio - 1) < 0.00001) return 0;
  BBox dst = *src;
  float box_w = dst.x2_ - dst.x1_;
  float box_h = dst.y2_ - dst.y1_;
  float center_x = (dst.x1_ + dst.x2_) / 2.0f;
  float center_y = (dst.y1_ + dst.y2_) / 2.0f;
  float w_new = box_w;
  float h_new = box_h;
  switch (norm_method) {
    case NormMethod::BPU_MODEL_NORM_BY_WIDTH_LENGTH: {
      w_new = box_w * norm_ratio;
      h_new = box_h + w_new - box_w;
      if (h_new <= 0) return -1;
    } break;
    case NormMethod::BPU_MODEL_NORM_BY_WIDTH_RATIO:
    case NormMethod::BPU_MODEL_NORM_BY_HEIGHT_RATIO:
    case NormMethod::BPU_MODEL_NORM_BY_LSIDE_RATIO: {
      h_new = box_h * norm_ratio;
      w_new = box_w * norm_ratio;
    } break;
    case NormMethod::BPU_MODEL_NORM_BY_HEIGHT_LENGTH: {
      h_new = box_h * norm_ratio;
      w_new = box_w + h_new - box_h;
      if (w_new <= 0) return -1;
    } break;
    case NormMethod::BPU_MODEL_NORM_BY_LSIDE_LENGTH: {
      if (box_w > box_h) {
        w_new = box_w * norm_ratio;
        h_new = box_h + w_new - box_w;
        if (h_new <= 0) return -1;
      } else {
        h_new = box_h * norm_ratio;
        w_new = box_w + h_new - box_h;
        if (w_new <= 0) return -1;
      }
    } break;
    case NormMethod::BPU_MODEL_NORM_BY_LSIDE_SQUARE: {
      if (box_w > box_h) {
        w_new = box_w * norm_ratio;
        h_new = w_new;
      } else {
        h_new = box_h * norm_ratio;
        w_new = h_new;
      }
    } break;
    case NormMethod::BPU_MODEL_NORM_BY_DIAGONAL_SQUARE: {
      float diagonal = sqrt(pow(box_w, 2.0) + pow(box_h, 2.0));
      w_new = h_new = diagonal * norm_ratio;
    } break;
    case NormMethod::BPU_MODEL_NORM_BY_NOTHING: break;
    default: return 0;
  }
  dst.x1_ = center_x - w_new / 2;
  dst.x2_ = center_x + w_new / 2;
  dst.y1_ = center_y - h_new / 2;
  dst.y2_ = center_y + h_new / 2;

  if (dst.x1_ < 0 || dst.y1_ < 0 || dst.x2_ > total_w || dst.y2_ > total_h) {
    return -1;
  }

  return 0;
}

float FaceSnapFilterMethod::GetOccludeVal(const std::string &name) {
  if (name == "left_eye")  return filter_param_->left_eye_occluded_thr;
  if (name == "right_eye")  return filter_param_->right_eye_occluded_thr;
  if (name == "left_brow")  return filter_param_->left_brow_occluded_thr;
  if (name == "right_brow")  return filter_param_->right_brow_occluded_thr;
  if (name == "forehead")  return filter_param_->forehead_occluded_thr;
  if (name == "left_cheek")  return filter_param_->left_cheek_occluded_thr;
  if (name == "right_cheek")  return filter_param_->right_cheek_occluded_thr;
  if (name == "nose")  return filter_param_->nose_occluded_thr;
  if (name == "mouth")  return filter_param_->mouth_occluded_thr;
  if (name == "jaw")  return filter_param_->jaw_occluded_thr;
  if (name == "left_eye_dark")
    return filter_param_->left_eye_occluded_dark_thr;
  if (name == "right_eye_dark")
    return filter_param_->right_eye_occluded_dark_thr;
  if (name == "left_brow_dark")
    return filter_param_->left_brow_occluded_dark_thr;
  if (name == "right_brow_dark")
    return filter_param_->right_brow_occluded_dark_thr;
  if (name == "forehead_dark")
    return filter_param_->forehead_occluded_dark_thr;
  if (name == "left_cheek_dark")
    return filter_param_->left_cheek_occluded_dark_thr;
  if (name == "right_cheek_dark")
    return filter_param_->right_cheek_occluded_dark_thr;
  if (name == "nose_dark")
    return filter_param_->nose_occluded_dark_thr;
  if (name == "mouth_dark")
    return filter_param_->mouth_occluded_dark_thr;
  if (name == "jaw_dark")
    return filter_param_->jaw_occluded_dark_thr;
  return 0;
}

int FaceSnapFilterMethod::GetOccludeErrCode(const std::string &name) {
  if (name == "left_eye")
    return filter_param_->left_eye_occluded_thr_err_code;
  if (name == "right_eye")
    return filter_param_->right_eye_occluded_thr_err_code;
  if (name == "left_brow")
    return filter_param_->left_brow_occluded_thr_err_code;
  if (name == "right_brow")
    return filter_param_->right_brow_occluded_thr_err_code;
  if (name == "forehead")
    return filter_param_->forehead_occluded_thr_err_code;
  if (name == "left_cheek")
    return filter_param_->left_cheek_occluded_thr_err_code;
  if (name == "right_cheek")
    return filter_param_->right_cheek_occluded_thr_err_code;
  if (name == "nose")
    return filter_param_->nose_occluded_thr_err_code;
  if (name == "mouth")
    return filter_param_->mouth_occluded_thr_err_code;
  if (name == "jaw")
    return filter_param_->jaw_occluded_thr_err_code;
  if (name == "left_eye_dark")
    return filter_param_->left_eye_occluded_dark_thr_err_code;
  if (name == "right_eye_dark")
    return filter_param_->right_eye_occluded_dark_thr_err_code;
  if (name == "left_brow_dark")
    return filter_param_->left_brow_occluded_dark_thr_err_code;
  if (name == "right_brow_dark")
    return filter_param_->right_brow_occluded_dark_thr_err_code;
  if (name == "forehead_dark")
    return filter_param_->forehead_occluded_dark_thr_err_code;
  if (name == "left_cheek_dark")
    return filter_param_->left_cheek_occluded_dark_thr_err_code;
  if (name == "right_cheek_dark")
    return filter_param_->right_cheek_occluded_dark_thr_err_code;
  if (name == "nose_dark")
    return filter_param_->nose_occluded_dark_thr_err_code;
  if (name == "mouth_dark")
    return filter_param_->mouth_occluded_dark_thr_err_code;
  if (name == "jaw_dark")
    return filter_param_->jaw_occluded_dark_thr_err_code;
  return 0;
}

}  // namespace xstream

