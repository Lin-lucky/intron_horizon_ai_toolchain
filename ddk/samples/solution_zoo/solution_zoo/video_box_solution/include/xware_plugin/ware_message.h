/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2019 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef SOLUTION_VIDEO_BOX_INCLUDE_PLUGIN_WARE_MESSAGE_H_
#define SOLUTION_VIDEO_BOX_INCLUDE_PLUGIN_WARE_MESSAGE_H_
#include <memory>
#include <string>
#include <vector>

#include "message/common_data_type/recog_result.h"
#include "protobuf/x3ware.pb.h"
#include "xproto/xproto_world.h"
#include "xstream/vision_type.h"
#include "xstream/xstream_data.h"
#include "xwarehouse/xwarehouse.h"

namespace solution {
namespace video_box {
using xproto::XProtoMessage;

#define TYPE_WARE_UPSTREAM_MESSAGE "XPLUGIN_WARE_UP_MESSAGE"
#define TYPE_WARE_DOWNSTREAM_MESSAGE "XPLUGIN_WARE_DOWM_MESSAGE"

typedef std::shared_ptr<xstream::FloatFeature> XStreamFeaturePtr;

typedef enum {
  WARE_OK = 0,
  WARE_PARAM_ERR = 1000,
  WARE_INIT_FAIL = 1001,
  WARE_CREATE_TABLE_FAIL = 1002,
  WARE_DELETE_TABLE_FAIL = 1003,
  WARE_UPDATE_THRESHOLD_FAIL = 1004,
  WARE_LIST_TABLE_FAIL = 1005,
  WARE_LOAD_TABLE_FAIL = 1006,
  WARE_SOURCETYPE_NOT_MATCH = 1007,
  WARE_TABLE_INIT_FAIL = 1010,
  WARE_TABLE_EXIST = 1011,
  WARE_TABLE_NOT_EXIST = 1012,
  WARE_FEATURE_SIZE_NOT_MATCH = 1013,
  WARE_ADD_RECORD_FAIL = 1014,
  WARE_DELETE_RECORD_FAIL = 1015,
  WARE_UPDATE_RECORD_FAIL = 1016,
  WARE_ADD_FEATURE_FAIL = 1017,
  WARE_DELETE_FEATURE_FAIL = 1018,
  WARE_UPDATE_FEATURE_FAIL = 1019,
  WARE_ID_EXIST = 1020,
  WARE_ID_NOT_EXIST = 1021,
  WARE_DECRYPT_FAIL = 1022,
  WARE_OVER_LIMIT = 1023,
  WARE_GET_LIMIT_FAIL = 1024,
  WARE_GET_RECORD_FAIL = 1025,
  WARE_NOT_INIT = 1026,
} ware_error_t;

class InfoWrapper {
 public:
  InfoWrapper() = default;
  explicit InfoWrapper(const x3ware::ModuleInfo &info);
  ~InfoWrapper();
  hobot::warehouse::HobotXWHDataSourceInfo *GetXWHData();
  void Print(void);

 public:
  hobot::warehouse::HobotXWHDataSourceInfo info_;
};

class TableWrapper {
 public:
  TableWrapper() = default;
  explicit TableWrapper(const x3ware::Table &table);
  explicit TableWrapper(const hobot::warehouse::HobotXWHSetInfo &table);
  ~TableWrapper();
  hobot::warehouse::HobotXWHSetInfo *GetXWHData();
  void Serialize(x3ware::WareMessage *ware_msg);
  void Serialize(const hobot::warehouse::HobotXWHSetInfo &table_in,
                 x3ware::WareMessage *ware_msg);
  void Print(void);

 public:
  char *name_;
  char *model_version_;
  hobot::warehouse::HobotXWHSetInfo table_;
};

class FeatureWrapper {
 public:
  FeatureWrapper() = default;
  explicit FeatureWrapper(const hobot::warehouse::HobotXWHFeature &feature);
  explicit FeatureWrapper(const x3ware::Feature &feature);
  explicit FeatureWrapper(xstream::FloatFeature &feature, std::string img_url);
  ~FeatureWrapper();
  hobot::warehouse::HobotXWHFeature *GetXWHData();
  // 只有浮点未加密用float存储
  inline bool IsFloatFeature(int attr) { return attr == ((1 << 6) | 1); }
  void Serialize(x3ware::Record *record);
  void Serialize(const hobot::warehouse::HobotXWHFeature &feature_in,
                 x3ware::Record *record);
  void Print(void);

 public:
  int feature_attr_ = 0;
  std::vector<float> feature_f_;
  std::vector<int32_t> feature_i_;
  hobot::warehouse::HobotXWHFeature xwhfeature_ = {0};
};

class RecordWrapper {
 public:
  RecordWrapper() {}
  explicit RecordWrapper(const hobot::warehouse::HobotXWHRecord &record);
  explicit RecordWrapper(const x3ware::Record &record);
  explicit RecordWrapper(std::string id,
                         std::shared_ptr<FeatureWrapper> feature);
  ~RecordWrapper();
  hobot::warehouse::HobotXWHRecord *GetXWHData();
  void Serialize(x3ware::WareMessage *ware_msg);
  void Serialize(const hobot::warehouse::HobotXWHRecord &record_in,
                 x3ware::WareMessage *ware_msg, bool page_list = false);
  /*
   * @brief 序列化分页查询结果
   * @param page_num: 总分页数
   * @param page_id: 当前分页编号
   * */
  void Serialize(const int &page_num, const int &page_id,
                 x3ware::WareMessage *ware_msg);
  void Print(void);

 public:
  char *id_;
  std::vector<std::shared_ptr<FeatureWrapper>> features_;
  hobot::warehouse::HobotXWHRecord xwhrecord_ = {0};
  std::vector<hobot::warehouse::HobotXWHFeature> xwhfeatures_;
  FeatureWrapper fea_w_;
};

class WareData {
 public:
  WareData() {}
  explicit WareData(x3ware::WareMessage_Type type) { type_ = type; }
  ~WareData() {}

 public:
  x3ware::WareMessage_Type type_;
  x3ware::WareMessage_Oper oper_;
  x3ware::WareMessage_Ack ack_;
  int error_code_ = 0;
  std::string version_;
  InfoWrapper info_;
  std::vector<std::shared_ptr<TableWrapper>> tables_;
  std::vector<std::shared_ptr<RecordWrapper>> records_;
  bool search_match_ = false;
  std::vector<hobot::warehouse::HobotXWHIdScore> search_result_;
  hobot::warehouse::HobotXWHCompareResult compare_result_ = {0};
  bool need_update_ = false;

  std::shared_ptr<x3ware::WareMessage> ware_msg_ = nullptr;
  std::string proto_str;
};

class WarePluginMessage : public XProtoMessage {
 public:
  WarePluginMessage() = delete;
#if 0
  explicit WarePluginMessage(x3ware::WareMessage_Type type,
                             ConfigMessageMask mask) {
    type_ = TYPE_WARE_DOWNSTREAM_MESSAGE;
    message_mask_ = mask;
    message_type_ = type;
  }
  explicit WarePluginMessage(x3ware::WareMessage_Type type,
                             ConfigMessageMask mask,
                             std::shared_ptr<WareData> data) {
    type_ = TYPE_WARE_UPSTREAM_MESSAGE;
    message_mask_ = mask;
    message_type_ = type;
    message_data_ = data;
  }
#else
  explicit WarePluginMessage(x3ware::WareMessage_Type type) {
    type_ = TYPE_WARE_DOWNSTREAM_MESSAGE;
    message_type_ = type;
  }
  explicit WarePluginMessage(x3ware::WareMessage_Type type,
                             std::shared_ptr<WareData> data) {
    type_ = TYPE_WARE_UPSTREAM_MESSAGE;
    message_type_ = type;
    message_data_ = data;
  }
#endif
  ~WarePluginMessage() = default;

  //  bool IsMessageValid(ConfigMessageMask mask) { return message_mask_ & mask;
  //  }
  x3ware::WareMessage_Type GetMessageType() { return message_type_; }
  std::shared_ptr<WareData> GetMessageData() { return message_data_; }
  std::string Serialize() override;

 private:
  x3ware::WareMessage_Type message_type_;
  //  ConfigMessageMask message_mask_;
  std::shared_ptr<WareData> message_data_;
};
}  //  namespace video_box
}  //  namespace solution
#endif  // SOLUTION_VIDEO_BOX_INCLUDE_PLUGIN_WARE_MESSAGE_H_
