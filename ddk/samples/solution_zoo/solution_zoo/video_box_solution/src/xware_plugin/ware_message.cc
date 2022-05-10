/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2019 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/

#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "xware_plugin/ware_message.h"
#include "hobotlog/hobotlog.hpp"

namespace solution {
namespace video_box {

using xproto::XPluginAsync;
using xproto::XProtoMessage;
using xproto::XProtoMessagePtr;

InfoWrapper::InfoWrapper(const x3ware::ModuleInfo &info) {
  info_.type_ =
      static_cast<hobot::warehouse::HobotXWHDataSourceType>(info.store_type_());
  strncpy(info_.db_file_, info.db_dir_().c_str(),
          std::min(static_cast<int>(info.db_dir_().size()) + 1,
                   HOBOT_FILENAME_LENGTH));
  Print();
}

InfoWrapper::~InfoWrapper() {}

hobot::warehouse::HobotXWHDataSourceInfo *InfoWrapper::GetXWHData() {
  return &info_;
}

void InfoWrapper::Print(void) {
  LOGD << "type_: " << info_.type_;
  LOGD << "db_file_: " << info_.db_file_;
}

TableWrapper::TableWrapper(const x3ware::Table &table) {
  strncpy(table_.set_name_, table.name_().c_str(), table.name_().size() + 1);
  name_ = table_.set_name_;
  strncpy(table_.model_version_, table.model_version_().c_str(),
          table.model_version_().size() + 1);
  model_version_ = table_.model_version_;
  table_.feature_size_ = table.feature_size_();
  table_.is_check_attr_ = table.attr_check_();
  table_.distance_threshold_ = table.distance_thr_();
  table_.similar_threshold_ = table.similar_thr_();
  Print();
}

TableWrapper::TableWrapper(const hobot::warehouse::HobotXWHSetInfo &table) {
  memcpy(&table_, &table, sizeof(table));
  name_ = table_.set_name_;
  model_version_ = table_.model_version_;
  Print();
}

TableWrapper::~TableWrapper() {}

hobot::warehouse::HobotXWHSetInfo *TableWrapper::GetXWHData() {
  return &table_;
}

void TableWrapper::Serialize(x3ware::WareMessage *ware_msg) {
  auto table = ware_msg->add_table_();
  table->set_name_(table_.set_name_);
  table->set_model_version_(table_.model_version_);
  table->set_attr_check_(table_.is_check_attr_);
  table->set_feature_size_(table_.feature_size_);
  table->set_distance_thr_(table_.distance_threshold_);
  table->set_similar_thr_(table_.similar_threshold_);
  table->set_db_dir_(table_.db_file_);
}

void TableWrapper::Serialize(const hobot::warehouse::HobotXWHSetInfo &table_in,
                             x3ware::WareMessage *ware_msg) {
  auto table = ware_msg->add_table_();
  table->set_name_(table_in.set_name_);
  table->set_model_version_(table_in.model_version_);
  table->set_attr_check_(table_in.is_check_attr_);
  table->set_feature_size_(table_in.feature_size_);
  table->set_distance_thr_(table_in.distance_threshold_);
  table->set_similar_thr_(table_in.similar_threshold_);
  table->set_db_dir_(table_in.db_file_);
}

void TableWrapper::Print(void) {
  LOGD << "set_name_: " << table_.set_name_;
  LOGD << "model_version_: " << table_.model_version_;
  LOGD << "is_check_attr_: " << table_.is_check_attr_;
  LOGD << "feature_size_: " << table_.feature_size_;
  LOGD << "distance_threshold_: " << table_.distance_threshold_;
  LOGD << "similar_threshold_: " << table_.similar_threshold_;
  LOGD << "db_file_: " << table_.db_file_;
}

RecordWrapper::RecordWrapper(const hobot::warehouse::HobotXWHRecord &record) {
  strncpy(xwhrecord_.id_, record.id_, HOBOT_ID_LENGTH);
  id_ = xwhrecord_.id_;
  if (record.size_ <= 0) return;
  for (int i = 0; i < record.size_; ++i) {
    auto fea_w = std::make_shared<FeatureWrapper>(record.features_[i]);
    features_.emplace_back(fea_w);
    xwhfeatures_.emplace_back(*(fea_w->GetXWHData()));
  }
  xwhrecord_.size_ = xwhfeatures_.size();
  xwhrecord_.features_ = &xwhfeatures_[0];
  Print();
}

RecordWrapper::RecordWrapper(std::string id,
    std::shared_ptr<FeatureWrapper> feature) {
  strncpy(xwhrecord_.id_, id.c_str(), HOBOT_ID_LENGTH);
  id_ = xwhrecord_.id_;
  std::cout << "id:" << id_ << std::endl;
  features_.emplace_back(feature);
  xwhfeatures_.emplace_back(*(feature->GetXWHData()));
  xwhrecord_.size_ = xwhfeatures_.size();
  std::cout << "xwhrecord_.size_" << xwhrecord_.size_ << std::endl;
  xwhrecord_.features_ = &xwhfeatures_[0];
  Print();
}


RecordWrapper::RecordWrapper(const x3ware::Record &record) {
  strncpy(xwhrecord_.id_, record.id_().c_str(), record.id_().size() + 1);
  id_ = xwhrecord_.id_;
  if (record.num_() <= 0) return;
  for (int i = 0; i < record.features__size(); ++i) {
    auto fea_w = std::make_shared<FeatureWrapper>(record.features_(i));
    features_.emplace_back(fea_w);
    xwhfeatures_.emplace_back(*(fea_w->GetXWHData()));
  }
  xwhrecord_.size_ = xwhfeatures_.size();
  xwhrecord_.features_ = &xwhfeatures_[0];
  Print();
}

RecordWrapper::~RecordWrapper() {}

hobot::warehouse::HobotXWHRecord *RecordWrapper::GetXWHData() {
  return &xwhrecord_;
}

void RecordWrapper::Serialize(x3ware::WareMessage *ware_msg) {
  auto record = ware_msg->add_record_();
  record->set_id_(id_);
  record->set_num_(features_.size());
  for (auto &fea_w : features_) {
    fea_w->Serialize(record);
  }
}

void RecordWrapper::Serialize(const hobot::warehouse::HobotXWHRecord &record_in,
                              x3ware::WareMessage *ware_msg,
                              bool page_list) {
  HOBOT_CHECK(ware_msg);
  if (!page_list) {
    auto record = ware_msg->add_record_();
    record->set_id_(record_in.id_);
    record->set_num_(record_in.size_);
    for (int i = 0; i < record_in.size_; ++i) {
      fea_w_.Serialize(record_in.features_[i], record);
    }
  } else {
    auto records = ware_msg->mutable_list_record_();
    HOBOT_CHECK(records);
    auto record = records->add_record_();
    HOBOT_CHECK(record);
    record->set_id_(record_in.id_);
    record->set_num_(record_in.size_);
    for (int i = 0; i < record_in.size_; ++i) {
      fea_w_.Serialize(record_in.features_[i], record);
    }
  }
}

void RecordWrapper::Serialize(const int &page_num,
                              const int &page_id,
                              x3ware::WareMessage *ware_msg) {
  HOBOT_CHECK(ware_msg);
  auto records = ware_msg->mutable_list_record_();
  HOBOT_CHECK(records);
  records->set_seq_id_(page_id);
  records->set_seq_num_(page_num);
}

void RecordWrapper::Print(void) {
  LOGD << "id_: " << id_;
  LOGD << "fea_num: " << xwhfeatures_.size();
  if (xwhfeatures_.size() > 0) {
    LOGD << "img_uri_: " << xwhfeatures_[0].img_uri_;
    LOGD << "feature_attr_: " << xwhfeatures_[0].feature_attr_;
    LOGD << "size_: " << xwhfeatures_[0].size_;
  }
}

FeatureWrapper::FeatureWrapper(
    const hobot::warehouse::HobotXWHFeature &feature) {
  xwhfeature_.feature_attr_ = feature.feature_attr_;
  feature_attr_ = feature.feature_attr_;
  strncpy(xwhfeature_.img_uri_, feature.img_uri_, HOBOT_URI_LENGTH);
  if (feature.size_ <= 0) return;
  if (IsFloatFeature(feature_attr_)) {
    for (int i = 0; i < feature.size_; ++i)
      feature_f_.emplace_back(static_cast<float *>(feature.feature_)[i]);
    xwhfeature_.size_ = feature_f_.size();
    xwhfeature_.feature_ = &feature_f_[0];
  } else {
    for (int i = 0; i < feature.size_; ++i)
      feature_i_.emplace_back(static_cast<int32_t *>(feature.feature_)[i]);
    xwhfeature_.size_ = feature_i_.size();
    xwhfeature_.feature_ = &feature_i_[0];
  }
  Print();
}

FeatureWrapper::FeatureWrapper(const x3ware::Feature &feature) {
  xwhfeature_.feature_attr_ = feature.attr_();
  feature_attr_ = feature.attr_();
  strncpy(xwhfeature_.img_uri_, feature.uri_().c_str(),
          feature.uri_().size() + 1);
  if (feature.size_() <= 0) return;
  if (IsFloatFeature(feature_attr_)) {
    for (auto &fea : feature.feature_f_()) feature_f_.emplace_back(fea);
    xwhfeature_.size_ = feature_f_.size();
    xwhfeature_.feature_ = &feature_f_[0];
  } else {
    for (auto &fea : feature.feature_i_()) feature_i_.emplace_back(fea);
    xwhfeature_.size_ = feature_i_.size();
    xwhfeature_.feature_ = &feature_i_[0];
  }
  Print();
}

FeatureWrapper::FeatureWrapper(xstream::FloatFeature &feature,
                               std::string img_url) {
  xwhfeature_.feature_attr_ = feature_attr_ = ((1 << 6) | 1);
  strncpy(xwhfeature_.img_uri_, img_url.c_str(), img_url.size() + 1);
  LOGD << "feature.values.size:" << feature.values_.size();
  if (feature.values_.size() <= 0) return;
  for (auto &fea : feature.values_) feature_f_.emplace_back(fea);
  xwhfeature_.size_ = feature_f_.size();
  xwhfeature_.feature_ = &feature_f_[0];
  Print();
}

FeatureWrapper::~FeatureWrapper() {}

hobot::warehouse::HobotXWHFeature *FeatureWrapper::GetXWHData() {
  return &xwhfeature_;
}

void FeatureWrapper::Serialize(x3ware::Record *record) {
  auto feature = record->add_features_();
  feature->set_uri_(xwhfeature_.img_uri_);
  feature->set_attr_(feature_attr_);
  if (IsFloatFeature(feature_attr_)) {
    feature->set_size_(feature_f_.size());
    for (auto &fea_f : feature_f_) {
      feature->add_feature_f_(fea_f);
    }
  } else {
    feature->set_size_(feature_i_.size());
    for (auto &fea_i : feature_i_) {
      feature->add_feature_i_(fea_i);
    }
  }
}

void FeatureWrapper::Serialize(
    const hobot::warehouse::HobotXWHFeature &feature_in,
    x3ware::Record *record) {
  auto feature = record->add_features_();
  feature->set_uri_(feature_in.img_uri_);
  feature->set_attr_(feature_in.feature_attr_);
  feature->set_size_(feature_in.size_);
  if (IsFloatFeature(feature_in.feature_attr_)) {
    for (int i = 0; i < feature_in.size_; ++i) {
      feature->add_feature_f_(static_cast<float *>(feature_in.feature_)[i]);
    }
  } else {
    for (int i = 0; i < feature_in.size_; ++i) {
      feature->add_feature_i_(static_cast<int32_t *>(feature_in.feature_)[i]);
    }
  }
}

void FeatureWrapper::Print(void) {
  LOGD << "img_uri_: " << xwhfeature_.img_uri_;
  LOGD << "feature_attr_: " << xwhfeature_.feature_attr_;
  LOGD << "size_: " << xwhfeature_.size_;
  if (feature_f_.size() > 0) {
    LOGD << "feature_f_[0]: " << feature_f_[0] << ", "
         << static_cast<float *>(xwhfeature_.feature_)[0];
    LOGD << "feature_f_[last]: " << feature_f_[feature_f_.size() - 1] << ", "
         << static_cast<float *>(xwhfeature_.feature_)[feature_f_.size() - 1];
  }
  if (feature_i_.size() > 0) {
    LOGD << "feature_i_[0]: " << feature_i_[0] << ", "
         << static_cast<int32_t *>(xwhfeature_.feature_)[0];
    LOGD << "feature_i_[last]: " << feature_i_[feature_i_.size() - 1] << ", "
         << static_cast<int32_t *>(xwhfeature_.feature_)[feature_i_.size() - 1];
  }
}

std::string WarePluginMessage::Serialize() {
  if (message_data_.get() == nullptr) {
    LOGE << "Ware: msg is null";
    return "";
  }

  // 如果是list record，并且没有报错，直接返回已经序列化好的数据
  // 如果在此处序列化，由于list record内部多个分页使用同一个查询结果指针
  // 这种异步操作可能会导致protobuf内部会抛异常
  if (message_type_ == x3ware::WareMessage_Type_RecordOper &&
      message_data_->oper_ == x3ware::WareMessage_Oper_Query &&
      message_data_->error_code_ == hobot::warehouse::Errcode_Ok) {
    return message_data_->proto_str;
  }

  std::string proto_str;
  auto WareMsg = message_data_->ware_msg_;
  if (WareMsg.get() == nullptr)
    WareMsg = std::make_shared<x3ware::WareMessage>();
  WareMsg->set_type_(message_type_);
  WareMsg->set_ack_(message_data_->ack_);
  WareMsg->set_error_code_(message_data_->error_code_);
  if (message_type_ == x3ware::WareMessage_Type_GetVersion) {
    auto info = WareMsg->mutable_info_();
    info->set_version_(message_data_->version_);
  } else if (message_type_ == x3ware::WareMessage_Type_TableOper) {
    WareMsg->set_oper_(message_data_->oper_);
  } else if (message_type_ == x3ware::WareMessage_Type_RecordOper ||
      message_type_ == x3ware::WareMessage_Type_FeatureOper) {
    WareMsg->set_oper_(message_data_->oper_);
  } else if (message_type_ == x3ware::WareMessage_Type_Search) {
    auto search_result = WareMsg->mutable_search_result_();
    search_result->set_match_(message_data_->search_match_);
    search_result->set_num_(message_data_->search_result_.size());
    for (auto &id_score : message_data_->search_result_) {
      auto match_result = search_result->add_match_result_();
      match_result->set_id_(id_score.id_);
      match_result->set_distance_(id_score.distance_);
      match_result->set_similar_(id_score.similar_);
    }
  } else if (message_type_ == x3ware::WareMessage_Type_Compare) {
    auto compare_result = WareMsg->mutable_compare_result_();
    compare_result->set_match_(message_data_->compare_result_.is_matched_);
    compare_result->set_distance_(message_data_->compare_result_.distance_);
    compare_result->set_similar_(message_data_->compare_result_.similar_);
  } else if (message_type_ == x3ware::WareMessage_Type_Init ||
      message_type_ == x3ware::WareMessage_Type_DeInit) {
    // do nothing
  } else if (message_type_ ==
      x3ware::WareMessage_Type_CheckModelVersion) {
    auto model_check = WareMsg->mutable_model_version_check_();
    model_check->set_need_update_(message_data_->need_update_);
  } else {
    LOGE << "No support message type";
  }

  WareMsg->SerializeToString(&proto_str);

  return std::move(proto_str);
}

}  // namespace video_box
}  // namespace solution
