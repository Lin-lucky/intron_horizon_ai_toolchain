/**
 * Copyright (c) 2020, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail:
 * @Date:
 * @Version:
 * @Brief:
 * @Last Modified by:
 * @Last Modified time:
 */
#include "smart_plugin/method_configer.h"

namespace xproto {

#define CONFIG_OPERATION(Oper, Type) \
for (int i = 0; i < config.Type##__size(); i++) {\
  auto Type = config.mutable_##Type##_(i);\
  std::string key = Type->type_(), module_name;\
  auto value = Type->value_();\
  ret = GetMethodNameandRealKey(&key, &module_name);\
  if (ret != 0) {\
    if (ret == -1) { \
      ret = 0; continue; \
    } \
    LOGE << "can not find module name by key: " << key;\
    return -1;\
  }\
  ret = Oper##Method##Type(module_name, key, value);\
  if (ret != 0) {\
    LOGE << #Oper"Method"#Type" failed";\
    return -1;\
  }\
  Type->set_value_(value);\
}

#define UPDATE_XSTREAM_CONFIG(p_params) \
  for (const auto &p_param : p_params) { \
    xstream_->UpdateConfig(p_param->unique_name_, p_param);\
}

#define GET_XSTREAM_SDK \
  if (weak_sdk_.expired()) { \
    LOGE << "xstream sdk expired"; \
    return -1; \
} auto xstream_ = weak_sdk_.lock();

bool IsNum(std::string str) {
  std::stringstream sin(str);
  double d;
  char c;
  return (sin >> d) && !(sin >> c);
}

void MethodConfiger::DoSetMethodthreshold(
    const std::string &method_name,
    const std::string &key, float value_) {
  std::string value = std::to_string(value_);
  LOGI << "Set " << method_name << " " << key << " to " << value;
  auto &json_writer = method_info_[method_name].json_writer_;
  if (IsNum(value)) {
    std::string::size_type position;
    position = value.find('.');
    if (position != value.npos) {
      json_writer->SetValue(key, std::stof(value));
    } else {
      json_writer->SetValue(key, std::stoi(value));
    }
  } else if (value.find("[") != value.npos) {  // value为数组(例如：lmk_thr)
    std::stringstream str_stream(value);
    Json::Value json_v;
    str_stream >> json_v;
    json_writer->SetValue(key, json_v);
  } else {
    json_writer->SetValue(key, value);
  }
}

bool MethodConfiger::NeedToRunMethod(const std::string &method_name) const {
  return method_info_.at(method_name).work_mode_ == kMethodWork;
}

std::vector<xstream::InputParamPtr> MethodConfiger::BuildMethodInputParam(
    const std::string &method_name) const {
  std::vector<xstream::InputParamPtr> ret;
  if (NeedToRunMethod(method_name)) {
    // snap shot has two mode:crop and snapshot
    if (method_name == kMethodSnapShot) {
      if (!NeedToRunMethod(kMethodGrading)) {
        ret.emplace_back(new CropOnlyDisableParam(method_name));
      }
    }
  } else {
    ret.emplace_back(new PassThroughDisableParam(method_name));
  }
  return ret;
}

void MethodConfiger::BuildInputParam(xstream::InputDataPtr inputdata) {
  for (const auto &method : method_info_) {
    auto params = BuildMethodInputParam(method.first);
    for (const auto &param : params) {
      inputdata->params_.emplace_back(param);
    }
  }
}

int MethodConfiger::DoSetMethodMode(const std::string &method_name,
                           const std::string &mode) {
  auto &module = method_info_[method_name];
  LOGI << "Set " << method_name << " " << mode;
  module.work_mode_ = mode;
  if (method_name == kMethodSnapShot && mode == kMethodIdle) {
    method_info_[kMethodFaceFeature].work_mode_ = kMethodIdle;
    LOGI << "Set " << kMethodFaceFeature << " idle also";
  }
  return 0;
}

int MethodConfiger::GetMethodmode(
    const std::string &method_name,
    const std::string &key, std::string &value) {
  if (method_info_[method_name].work_mode_ == kMethodWork)
    value = "1";
  else
    value = "0";
  return 0;
}

int MethodConfiger::SetMethodmode(
    const std::string &method_name,
    const std::string &key, const std::string &mode) {
  std::string work_mode = kMethodWork;
  if (mode == "0" || mode == "off") work_mode = kMethodIdle;
  DoSetMethodMode(method_name, work_mode);
  return 0;
}

int MethodConfiger::SetMethodthreshold(const std::string &method_name,
    const std::string &key, float value) {
  DoSetMethodthreshold(method_name, key, value);
  return 0;
}

void inline MethodConfiger::SetMethodConfigDirty() {
  for (auto & method : method_info_) {
    method.second.json_writer_->r_updated_ = false;
  }
}

int MethodConfiger::GetMethodthreshold(
    const std::string &module_name,
    const std::string &key, float &value) {  //NOLINT
  int ret;
  GET_XSTREAM_SDK
  if (nullptr == xstream_->GetConfig(module_name)) {
    LOGE << "xstream could not find module " << module_name;
    return 0;
  }
  auto& module = method_info_[module_name];
  Json::Value &method_jsonv = module.json_writer_->jsonRoot;
  if (!module.json_writer_->r_updated_) {
    xstream::InputParamPtr param_ptr = xstream_->GetConfig(module_name);
    ret = ParseMethodConfig(param_ptr, &method_jsonv);
    if (ret != 0) {
      LOGE << "ParseMethodConfig failed, ret: " << ret;
      return -1;
    }
    module.json_writer_->r_updated_ = true;
  }
  // 查找Json中是否存在key这个记录, 有则读取其value.
  if (method_jsonv.isMember(key)) {
    if (method_jsonv[key].isArray()) {
     // *value = method_jsonv[key].toStyledString();
    } else {
      value = method_jsonv[key].asFloat();
    }
  } else {
    LOGE << "Can not find this key[" << key << "] value.";
    return -1;
  }
  return 0;
}

int MethodConfiger::SetMethodConfigToXstream() {
  std::vector<xstream::InputParamPtr> ret;
  GET_XSTREAM_SDK
  for (const auto &method : method_info_) {
    const auto json_writer = method.second.json_writer_;
    if (json_writer->w_updated_) {
      ret.emplace_back(std::make_shared<JsonParam>(
          method.first, json_writer->GetJson()));
      json_writer->w_updated_ = false;
    }
  }
  UPDATE_XSTREAM_CONFIG(ret)
  return 0;
}

int MethodConfiger::HandleAPConfig(::x3::Config &config) {  //NOLINT
  int ret;
  if (config.type_() == "Get") {
    CONFIG_OPERATION(Get, mode)
    CONFIG_OPERATION(Get, threshold)
    SetMethodConfigDirty();
    DumpMethodInto();
  } else if (config.type_() == "Set") {
    CONFIG_OPERATION(Set, mode)
    CONFIG_OPERATION(Set, threshold)
    SetMethodConfigToXstream();
  } else {
    LOGE << "unknown config cmd";
    return -1;
  }
  return 0;
}

void MethodConfiger::DumpMethodInto() {
  for (const auto &method : method_info_) {
    const auto &method_name = method.first;
    const auto &method_info = method.second;
    const auto &json_parameters = method_info.json_writer_->GetJson();
    LOGI << "==== method name: " << method_name << " ====";
    LOGI << "==== method workmode: " << method_info.work_mode_ << " =====";
    LOGI << "==== json parameters: ";
    LOGI << json_parameters;
  }
}

int MethodConfiger::ParseMethodConfig(xstream::InputParamPtr param_ptr,
                                      Json::Value *config_json) {
  if (nullptr == param_ptr || !param_ptr->is_json_format_) {
    LOGE << "Param error. param_ptr=" << param_ptr;
    return -1;
  }
  std::string content = param_ptr->Format();
  // 将字符串解析为Json对象.
  Json::CharReaderBuilder builder;
  JSONCPP_STRING error;
  std::shared_ptr<Json::CharReader> json_reader(builder.newCharReader());
  try {
    bool ret = json_reader->parse(content.c_str(), content.c_str()
        + content.size(), config_json, &error);
    if (!ret) {
      LOGE << "parse string to json failed.";
      return -1;
    }
  } catch (std::exception &e) {
    LOGE << "parse string to json failed."
         << " method name: " << param_ptr->unique_name_;
    return -1;
  }
  return 0;
}

int MethodConfiger::GetMethodNameandRealKey(
    std::string *key, std::string *module_name) {
  if (*key == "face_filter_on") {
    *module_name = kMethodFilter;
  } else if (*key == "face_select_on") {
    *module_name = kMethodGrading;
  } else if (*key == "face_mot_on") {
    *module_name = kMethodFaceMot;
  } else if (*key == "resnap_frame_count") {
    *key = kMethodResnapValue;
    *module_name = kMethodSnapShot;
  } else if (*key == "begin_post_frame_threshold") {
    *key = kMethodBeginPostFrameThr;
    *module_name = kMethodSnapShot;
  }
    //  filter config
  else if (*key == "face_size_pixel_threshold") {  //NOLINT
    *key = kMethodMinRectSize;
    *module_name = kMethodFilter;
  } else if (*key == "pitch_threshold") {
    *key = kMethodFrontalPitchThr;
    *module_name = kMethodFilter;
  } else if (*key == "yaw_threshold") {
    *key = kMethodFrontalYawThr;
    *module_name = kMethodFilter;
  } else if (*key == "roll_threshold") {
    *key = kMethodFrontalRollThr;
    *module_name = kMethodFilter;
  } else if (*key == "face_conf_threshold") {
    *key = kMethodPvThr;
    *module_name = kMethodFilter;
  } else if (*key == "bound_width_threshold") {
    *key = kMethodBound_thr_w;
    *module_name = kMethodFilter;
  } else if (*key == "bound_height_threshold") {
    *key = kMethodBound_thr_h;
    *module_name = kMethodFilter;
  }
    //  filter quality config
  else if (*key == "blur_threshold") {  //NOLINT
    *key = kMethodQualityThr;
    *module_name = kMethodFilter;
  } else if (*key == "abnormal_threshold") {
    *key = kMethodAbnormalThr;
    *module_name = kMethodFilter;
  } else if (*key == "left_eye_threshold") {
    *key = kMethodLeftEyeOccludedThr;
    *module_name = kMethodFilter;
  } else if (*key == "right_eye_threshold") {
    *key = kMethodRightEyeOccludedThr;
    *module_name = kMethodFilter;
  } else if (*key == "left_brow_threshold") {
    *key = kMethodLeftBrowOccludedThr;
    *module_name = kMethodFilter;
  } else if (*key == "right_brow_threshold") {
    *key = kMethodRightBrowOccludedThr;
    *module_name = kMethodFilter;
  } else if (*key == "forehead_threshold") {
    *key = kMethodForeheadOccludedThr;
    *module_name = kMethodFilter;
  } else if (*key == "left_cheek_threshold") {
    *key = kMethodLeftCheekOccludedThr;
    *module_name = kMethodFilter;
  } else if (*key == "right_cheek_threshold") {
    *key = kMethodRightCheekOccludedThr;
    *module_name = kMethodFilter;
  } else if (*key == "nose_threshold") {
    *key = kMethodNoseOccludedThr;
    *module_name = kMethodFilter;
  } else if (*key == "mouth_threshold") {
    *key = kMethodMouthOccludedThr;
    *module_name = kMethodFilter;
  } else if (*key == "jaw_threshold") {
    *key = kMethodJawOccludedThr;
    *module_name = kMethodFilter;
  } else {
    return -1;
  }
  return 0;
}

}  // namespace xproto
