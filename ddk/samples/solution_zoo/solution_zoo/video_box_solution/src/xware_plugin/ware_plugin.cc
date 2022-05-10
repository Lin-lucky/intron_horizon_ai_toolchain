/**
 * Copyright (c) 2020, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail:
 * @Date:
 * @Version: v0.0.1
 * @Brief:
 * @Last Modified by:
 * @Last Modified time:
 */

#include "xware_plugin/ware_plugin.h"

#include <stdio.h>
#include <time.h>

#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "smart_feature_message/smart_feature_message.h"
#include "ware_recog_message/ware_recog_message.h"
#include "xproto/message/msg_registry.h"
#include "xproto/msg_type/protobuf/x3.pb.h"
#include "xware_plugin/json_config_wrapper.h"
#include "xware_plugin/ware_message.h"

XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_FEATURE_MESSAGE)
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_RECOG_MESSAGE)
using solution::video_box::SmartFeatureMessage;

namespace solution {
namespace video_box {
WarePlugin::WarePlugin(const std::string &config_file) {
  unsigned int randp = time(NULL);
  g_recod_id_ = rand_r(&randp);
  config_file_ = config_file;
  LOGI << "WarePlugin config file:" << config_file_;
  Json::Value cfg_jv;
  std::ifstream infile(config_file_);
  if (infile) {
    infile >> cfg_jv;
    config_.reset(new JsonConfigWrapper(cfg_jv));
    ParseConfig();
  } else {
    LOGE << "open WarePlugin config fail";
  }
}

void WarePlugin::ParseConfig() {
  is_recognize_ = config_->GetBoolValue("is_recognize");
  LOGI << "is_recognize: " << is_recognize_;
  is_add_record_ = config_->GetBoolValue("is_add_record");
  LOGI << "is_add_record: " << is_add_record_;
  similar_thres_ = config_->GetFloatValue("similar_thres", similar_thres_);
  LOGI << "similar_thres: " << similar_thres_;
}

int WarePlugin::Init() {
  if (isinit_) return 0;
  isinit_ = true;
  LOGI << "WarePlugin INIT";
  RegisterMsg(
      TYPE_SMART_FEATURE_MESSAGE,
      std::bind(&WarePlugin::OnGetFeatureResult, this, std::placeholders::_1));
  return XPluginAsync::Init();
}

int WarePlugin::Start() {
  LOGI << "WarePlugin Start";
  DB::Get(db_path_);
  db_table_ = DB::Get()->CreateTable(db_name_, db_modul_version_);
  return 0;
}

int WarePlugin::Stop() {
  LOGI << "WarePlugin Stop";
  return 0;
}

int WarePlugin::DeInit() {
  isinit_ = false;
  return XPluginAsync::DeInit();
}

int WarePlugin::OnGetFeatureResult(const XProtoMessagePtr &msg) {
  auto feature_msg = std::static_pointer_cast<SmartFeatureMessage>(msg);
  HOBOT_CHECK(feature_msg);
  LOGD << "OnGetFeatureResult";
  auto features = feature_msg->GetMessageData();
  if (feature_msg->GetMessageType() == "Pic_Face") {
    if (features->feature_infos_.size() > 0) {
      if (features->feature_infos_[0].float_arrays_.size() > 0) {
        std::shared_ptr<FeatureWrapper> feature_w =
            std::make_shared<FeatureWrapper>(
                features->feature_infos_[0].float_arrays_[0], "faceid");
        std::shared_ptr<WareData> result_ptr = std::make_shared<WareData>();
        auto ret =
            db_table_->Search(db_dis_thr_, db_sim_thr_, feature_w, result_ptr);
        if (!ret) {
          for (auto &result : result_ptr->search_result_) {
            LOGE << "add search result, id: " << result.id_
                 << ", dis: " << result.distance_
                 << ", sim: " << result.similar_;
          }
        }
        if (!ret && result_ptr->search_match_ == true) {
          LOGE << "warehouse always exist";
        } else {
          std::shared_ptr<RecordWrapper> record_w =
              std::make_shared<RecordWrapper>(std::to_string(g_recod_id_++),
                                              feature_w);
          auto ret = db_table_->AddRecord(record_w, result_ptr);
          if (!ret) {
            LOGI << "ware add record successful";
          } else {
            LOGE << "ware add record fail,ret:" << ret;
          }
        }
      }
    }
  } else if (feature_msg->GetMessageType() == "Real_Face") {
    for (auto &feat_info : features->feature_infos_) {
      for (auto &feature : feat_info.float_arrays_) {
        std::shared_ptr<FeatureWrapper> feature_w =
            std::make_shared<FeatureWrapper>(feature, "faceid");
        std::shared_ptr<WareData> result_ptr = std::make_shared<WareData>();
        auto ret = db_table_->Search(recog_dis_thr_, recog_sim_thr_, feature_w,
                                     result_ptr);
        if (!ret) {
          for (auto &result : result_ptr->search_result_) {
            LOGE << "recog result,ch_id:" << features->ch_id_
                 << ",id:" << result.id_ << ", dis: " << result.distance_
                 << ", sim: " << result.similar_;
          }
        }
        if (!ret && result_ptr->search_match_ == true) {
          auto recog_info = std::make_shared<RecogResult>();
          recog_info->ch_id = features->ch_id_;
          recog_info->track_id = feat_info.track_id_;
          recog_info->is_recognize = 1;
          recog_info->record_id = result_ptr->search_result_[0].id_;
          recog_info->similar = result_ptr->search_result_[0].similar_;
          recog_info->img_uri_list = "";
          auto recog_msg = std::make_shared<WareRecogMessage>(recog_info);
          PushMsg(recog_msg);
          LOGE << "recog face successful";
        } else {
          LOGE << "recog face fail";
        }
      }
    }
  }
  return 0;
}

int WarePlugin::db_list() {
  std::shared_ptr<WareData> result_ptr = std::make_shared<WareData>();
  auto ret = db_table_->ListRecord(result_ptr);
  if (ret == 0) {
    LOGI << "db_list:";

    for (int i = 0; i < result_ptr->ware_msg_.get()->record__size(); i++) {
      auto record = result_ptr->ware_msg_.get()->record_(i);
      LOGI << record.id_();
    }
  }
  return 0;
}

int WarePlugin::db_table_clear() {
  DB::Get()->DropTable(db_name_, db_modul_version_);
  db_table_.reset();
  return 0;
}

}  // namespace video_box
}  // namespace solution
