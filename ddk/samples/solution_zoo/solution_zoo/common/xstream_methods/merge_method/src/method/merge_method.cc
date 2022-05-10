/*
 * @Description: implement of data_type
 * @Author: ruoting.ding@horizon.ai
 * @Date: 2019-11-15 17:49:26
 * @LastEditors: yutong.pan@horizon.ai
 * @LastEditTime: 2019-11-26
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */

#include "merge_method/merge_method.h"

#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "merge_method/strategy/head_body.h"
#include "merge_method/strategy/head_face.h"
#include "merge_method/strategy/rgb_nir.h"
#include "hobotlog/hobotlog.hpp"
#include "xstream/vision_type.h"

namespace xstream {

int MergeMethod::Init(const std::string &config_file) {
  auto ret = 0;
  reader_ = std::make_shared<JsonReader>(config_file);
  ret = reader_->ParseJsonFile();
  if (ret == 0) {
    default_method_config_param_ = std::make_shared<MergeParam>();
    default_method_config_param_->UpdateParameter(reader_);

    method_config_param_ = std::make_shared<HeadFaceParam>();
    method_config_param_->UpdateParameter(reader_);
    auto head_face = std::make_shared<HeadFaceStrategy>();
    head_face->Init(method_config_param_);
    strategy_map_["head_face"] = head_face;
#ifdef RGB_NIR_MERGE
    method_config_param_ = std::make_shared<RGBNIRParam>();
    method_config_param_->UpdateParameter(reader_);
    auto rgb_nir = std::make_shared<RGBNIRStrategy>();
    ret = rgb_nir->Init(method_config_param_);
    strategy_map_["rgb_nir"] = rgb_nir;
    if (default_method_config_param_ &&
        default_method_config_param_->reader_->GetStringValue("merge_type") ==
            "rgb_nir" &&
        ret == -1) {
      return ret;
    }
#endif  //  RGB_NIR_MERGE
    method_config_param_ = std::make_shared<HeadBodyParam>();
    method_config_param_->UpdateParameter(reader_);
    auto head_body = std::make_shared<HeadBodyStrategy>();
    ret = head_body->Init(method_config_param_);
    strategy_map_["head_body"] = head_body;
  } else {
    HOBOT_CHECK(0) << "Read config file error";
  }
  return ret;
}

std::vector<BaseDataPtr> MergeMethod::DoProcess(
    const std::vector<BaseDataPtr> &input,
    const InputParamPtr &params) {
  LOGD << "Merge::DoProcess: " << input.size();
  std::vector<BaseDataPtr> output;
  output = ProcessOneBatch(input, params);
  LOGD << "output size: " << output.size();
  return output;
}

std::vector<BaseDataPtr> MergeMethod::ProcessOneBatch(
    const std::vector<BaseDataPtr> &in, const InputParamPtr &param) {
  HOBOT_CHECK(!in.empty());
  if (param && param->is_json_format_) {
    std::string content = param->Format();
    Json::CharReaderBuilder builder;
    builder["collectComments"] = false;
    JSONCPP_STRING error;
    std::shared_ptr<Json::CharReader> json_reader(builder.newCharReader());
    Json::Value out_jv;
    auto ret = json_reader->parse(
        content.c_str(), content.c_str() + content.size(), &out_jv, &error);
    HOBOT_CHECK(ret && out_jv["merge_type"].isString())
        << "merge_type type error";
    auto type = out_jv["merge_type"].asString();
    if (type == "head_body") {
      LOGI << "Head body Mode";
      if (strategy_map_.count("head_body") && strategy_map_["head_body"]) {
        return strategy_map_["head_body"]->ProcessFrame(in, param);
      } else {
        HOBOT_CHECK(0) << "The strategy_map_ is not support";
      }
    } else if (type == "head_face") {
      LOGI << "Head Face Mode";
      if (strategy_map_.count("head_face") && strategy_map_["head_face"]) {
        return strategy_map_["head_face"]->ProcessFrame(in, param);
      } else {
        HOBOT_CHECK(0) << "The strategy_map_ is not support";
      }
    } else if (type == "rgb_nir") {
      LOGI << "RGB NIR Mode";
      if (strategy_map_.count("rgb_nir") && strategy_map_["rgb_nir"]) {
        return strategy_map_["rgb_nir"]->ProcessFrame(in, param);
      } else {
        HOBOT_CHECK(0) << "The strategy_map_ is not support";
      }
    } else {
      HOBOT_CHECK(0) << "The merge type is not support";
    }
  } else {
    if (default_method_config_param_ &&
        default_method_config_param_->reader_->GetStringValue("merge_type") ==
            "head_body") {
      LOGI << "Head body Mode";
      if (strategy_map_.count("head_body") && strategy_map_["head_body"]) {
        return strategy_map_["head_body"]->ProcessFrame(in, param);
      } else {
        HOBOT_CHECK(0) << "The strategy_map_ is not support";
      }
    } else if (default_method_config_param_ &&
               default_method_config_param_->reader_->GetStringValue(
                   "merge_type") == "head_face") {
      LOGI << "Head Face Mode";
      if (strategy_map_.count("head_face") && strategy_map_["head_face"]) {
        return strategy_map_["head_face"]->ProcessFrame(in, param);
      } else {
        HOBOT_CHECK(0) << "The strategy_map_ is not support";
      }
    } else if (default_method_config_param_ &&
               default_method_config_param_->reader_->GetStringValue(
                   "merge_type") == "rgb_nir") {
      LOGI << "RGB NIR Mode";
      if (strategy_map_.count("rgb_nir") && strategy_map_["rgb_nir"]) {
        return strategy_map_["rgb_nir"]->ProcessFrame(in, param);
      } else {
        HOBOT_CHECK(0) << "The strategy_map_ is not support";
      }
    } else {
      LOGI << "The merge type is not support, use default face_head strategy";
      if (strategy_map_.count("head_face") && strategy_map_["head_face"]) {
        return strategy_map_["head_face"]->ProcessFrame(in, param);
      }
    }
  }
  return {};
}

void MergeMethod::Finalize() {
  for (auto &item : strategy_map_) {
    auto strategy = item.second;
    strategy->Finalize();
  }
  LOGI << "MergeMethod::Finalize" << std::endl;
}

int MergeMethod::UpdateParameter(InputParamPtr ptr) {
  if (ptr->is_json_format_) {
    std::string content = ptr->Format();
    Json::CharReaderBuilder builder;
    builder["collectComments"] = false;
    JSONCPP_STRING error;
    std::shared_ptr<Json::CharReader> json_reader(builder.newCharReader());
    Json::Value out_jv;
    auto ret = json_reader->parse(
        content.c_str(), content.c_str() + content.size(), &out_jv, &error);
    auto reader = std::make_shared<JsonReader>(out_jv);
    for (auto &item : strategy_map_) {
      auto strategy = item.second;
      ret = strategy->UpdateParameter(reader);
      if (0 != ret) {
        return ret;
      }
    }
    return 0;
  } else {
    if (ptr && ptr->Format() == "Reset") {
      LOGI << "Reset Merge";
      *method_config_param_ = *default_method_config_param_;
      return 0;
    }
    HOBOT_CHECK(0) << "only support json format config and reset config";
    return -1;
  }
}

InputParamPtr MergeMethod::GetParameter() const { return method_config_param_; }

}  // namespace xstream
