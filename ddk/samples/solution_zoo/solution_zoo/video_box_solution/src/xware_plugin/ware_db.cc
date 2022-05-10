/**
 * Copyright (c) 2020, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail:
 * @Date: 2020-11-02 20:38:52
 * @Version: v0.0.1
 * @Last Modified by:
 * @Last Modified time: 2020-11-02 20:38:52
 */

#include "xware_plugin/ware_db.h"

#include <cstring>
#include <mutex>
#include <thread>

#include "hobotlog/hobotlog.hpp"
#include "xware_plugin/ware_message.h"

namespace solution {
namespace video_box {

using hobot::warehouse::HobotXWHSearchParam;
// #define WARE_OK 0
std::once_flag config_once;
std::shared_ptr<DB> DB::instance_;


/******************************************************************************
WARE_OK = 0,                        // ok
WARE_PARAM_ERR = 1000,              // 参数错误,字段为空，指针为空等
WARE_INIT_FAIL = 1001,              // 初始化失败
WARE_CREATE_TABLE_FAIL = 1002,      // 插入set信息失败
WARE_DELETE_TABLE_FAIL = 1003,      // 删除set信息失败
WARE_UPDATE_THRESHOLD_FAIL = 1004,  // 更新阈值失败
WARE_LIST_TABLE_FAIL = 1005,        // list set失败
WARE_LOAD_TABLE_FAIL = 1006,        // 加载数据库中的set信息失败
WARE_SOURCETYPE_NOT_MATCH = 1007,   // 类型不匹配
WARE_TABLE_INIT_FAIL = 1010,        // 分库初始化失败
WARE_TABLE_EXIST = 1011,            // 分库已经存在
WARE_TABLE_NOT_EXIST = 1012,        // 分库不存在
WARE_FEATURE_SIZE_NOT_MATCH = 1013, // 特征向量大小不匹配
WARE_ADD_RECORD_FAIL = 1014,        // 新增记录失败
WARE_DELETE_RECORD_FAIL = 1015,     // 删除记录失败
WARE_UPDATE_RECORD_FAIL = 1016,     // 更新记录失败
WARE_ADD_FEATURE_FAIL = 1017,       // 新增特征失败
WARE_DELETE_FEATURE_FAIL = 1018,    // 删除特征失败
WARE_UPDATE_FEATURE_FAIL = 1019,    // 更新特征失败
WARE_ID_EXIST = 1020,               // id已经存在
WARE_ID_NOT_EXIST = 1021,           // id不存在
WARE_DECRYPT_FAIL = 1022,           // 特征值解密失败
WARE_OVER_LIMIT = 1023,             // 超过库容
WARE_GET_LIMIT_FAIL = 1024,         // 获取库容大小失败
WARE_GET_RECORD_FAIL = 1025,        // 获取记录失败
WARE_NOT_INIT = 1026,               // ware_module未初始化
*******************************************************************************/

DB::DB(const std::string &table_path) {
  table_path_ = table_path;
  source_info_.type_ = hobot::warehouse::SQLITE;
  snprintf(source_info_.db_file_, strlen(table_path_.c_str()), "%s",
      table_path_.c_str());
  source_info_.is_check_attr_ = 0;  // 0:不校验，1:校验人种属性
  hobot::warehouse::HobotXWHInit(&source_info_);
}

DB::~DB() {}

std::shared_ptr<DB> DB::Get(std::string &table_path) {
  if (!instance_) {
    HOBOT_CHECK(!table_path.empty());
    std::call_once(config_once, [&]() {
      instance_ = std::shared_ptr<DB>(new DB(table_path));
    });
  }
  return instance_;
}

std::shared_ptr<DB> DB::Get() {
  if (instance_ == nullptr) {
    LOGE << "db has not been initialized yet.";
    return nullptr;
  }
  return instance_;
}

std::shared_ptr<DB::DB_TABLE>  DB::CreateTable(std::string &lib_name,
                    std::string &model_version) {
  auto ret = hobot::warehouse::HobotXWHCreateSet(lib_name.c_str(),
      model_version.c_str(), 128, &source_info_);
  std::string table_name = lib_name + model_version;
  if (ret == 0 || ret == hobot::warehouse::Errcode_Set_Is_Existed) {
    table_map_[table_name] = std::make_shared<DB_TABLE>(lib_name,
        model_version);
  }
  return table_map_[table_name];
}

std::shared_ptr<DB::DB_TABLE> DB::GetTable(const std::string &lib_name,
                  const std::string &model_version) {
  std::string table_name = lib_name + model_version;
  return table_map_[table_name];
}
int DB::DropTable(const std::string &lib_name,
                  const std::string &model_version) {
  auto ret = hobot::warehouse::HobotXWHDropSet(lib_name.c_str(),
      model_version.c_str());
  std::string table_name = lib_name + model_version;
  table_map_.erase(table_name);
  return ret;
}

int DB::DB_TABLE::Search(float distance, float similar,
    std::shared_ptr<FeatureWrapper> feature_ptr,
    std::shared_ptr<WareData> result_ptr) {
  int ret;
  if (!feature_ptr || !result_ptr)
    return -1;
  HobotXWHSearchParam param;
  param.top_n_ = 1;
  param.similar_threshold_ = similar;
  param.distance_threshold_ = distance;
  param.size_ = 1;
  param.features_ = &(feature_ptr->GetXWHData())[0];
  auto result = hobot::warehouse::HobotXWHAllocSearchResult();
  if (!result) {
    LOGE << "AllocSearchResult fail!";
    ret = -1;
    goto end;
  }
  ret = hobot::warehouse::HobotXWHSearch(set_name_.c_str(),
      model_version_.c_str(), &param, result);
  if (ret == 0) {
    LOGD << "search result, match: " << result->is_matched_;
    result_ptr->search_match_ = result->is_matched_;
    for (int i = 0; i < result->num_; ++i) {
      result_ptr->search_result_.emplace_back(result->id_score_[i]);
      LOGD << "search result, id: " << result->id_score_[i].id_
           << ", dis: " << result->id_score_[i].distance_
           << ", sim: " << result->id_score_[i].similar_;
    }
  }
  if (hobot::warehouse::HobotXWHReleaseSearchResult(&result) !=
      hobot::warehouse::Errcode_Ok) {
    LOGE << "ReleaseSearchResult Error";
    ret = -1;
  }
end:
  result_ptr->type_ = x3ware::WareMessage_Type_Search;
  result_ptr->oper_ = x3ware::WareMessage_Oper_Add;
  result_ptr->ack_ = (ret == 0) ? x3ware::WareMessage_Ack_Success
                                : x3ware::WareMessage_Ack_Fail;
  result_ptr->error_code_ = ret;
  return ret;
}

int DB::DB_TABLE::AddRecord(std::shared_ptr<RecordWrapper> record_ptr,
                            std::shared_ptr<WareData> result_ptr) {
  if (!record_ptr || !result_ptr)
    return -1;
  LOGI << "set_name_:" << set_name_ << ",model_version_:" << model_version_;
  auto ret = hobot::warehouse::HobotXWHAddRecord(set_name_.c_str(),
      model_version_.c_str(), record_ptr->GetXWHData());
  result_ptr->type_ = x3ware::WareMessage_Type_RecordOper;
  result_ptr->oper_ = x3ware::WareMessage_Oper_Add;
  result_ptr->ack_ = (ret == 0) ? x3ware::WareMessage_Ack_Success
                                : x3ware::WareMessage_Ack_Fail;
  result_ptr->error_code_ = ret;
  return ret;
}

int DB::DB_TABLE::ListRecord(std::shared_ptr<WareData>& result_ptr) {
  hobot::warehouse::HobotXWHListRecordResult *record_list = nullptr;
  auto ret = hobot::warehouse::HobotXWHListRecord(set_name_.c_str(),
      model_version_.c_str(), &record_list);
  if (ret == 0) {
    result_ptr->ware_msg_ = std::make_shared<x3ware::WareMessage>();
    RecordWrapper record_w;
    for (int i = 0; i < record_list->num_; ++i) {
      record_w.Serialize(record_list->record_[i],
                         result_ptr->ware_msg_.get());
      std::this_thread::yield();
    }
  }
  if (hobot::warehouse::HobotXWHReleaseListRecordResult(&record_list) !=
      hobot::warehouse::Errcode_Ok) {
    LOGE << "ReleaseListRecordResult Error";
    ret = -1;
  }
  result_ptr->type_ = x3ware::WareMessage_Type_RecordOper;
  result_ptr->oper_ = x3ware::WareMessage_Oper_Query;
  result_ptr->ack_ = (ret == 0) ? x3ware::WareMessage_Ack_Success
                                : x3ware::WareMessage_Ack_Fail;
  result_ptr->error_code_ = ret;
  return ret;
}

}  // namespace video_box
}  // namespace solution
