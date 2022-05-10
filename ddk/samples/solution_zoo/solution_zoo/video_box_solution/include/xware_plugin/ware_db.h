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
#ifndef SOLUTION_VIDEO_BOX_WAREPLUGIN_WAREDB_H_
#define SOLUTION_VIDEO_BOX_WAREPLUGIN_WAREDB_H_

#include <memory>
#include <string>
#include <map>
#include "xstream/xstream_data.h"
#include "xstream/vision_type.h"
#include "aes/hobot_aes.h"
extern "C" {
#include "xwarehouse/xwarehouse.h"
#include "xwarehouse/xwarehouse_data.h"
}
#include "protobuf/x3ware.pb.h"
#include "xproto/xproto_world.h"
#include "xware_plugin/ware_message.h"

namespace solution {
namespace video_box {

class DB {
 public:
  static std::shared_ptr<DB> Get(std::string &table_path);
  static std::shared_ptr<DB> Get();
  virtual ~DB();
  static std::shared_ptr<DB> instance_;
  hobot::warehouse::HobotXWHDataSourceInfo source_info_;

  // int ListTable(std::shared_ptr<WareData>& result_ptr);
  class DB_TABLE {
   public:
    DB_TABLE(std::string set_name, std::string model_version):
        set_name_(set_name), model_version_(model_version) {}
    virtual ~DB_TABLE() {}
    int Search(float distance, float similar, std::shared_ptr<FeatureWrapper>
        feature_ptr, std::shared_ptr<WareData> result_ptr);
    int AddRecord(std::shared_ptr<RecordWrapper> record_ptr,
        std::shared_ptr<WareData> result_ptr);
    int PageListRecord(std::shared_ptr<WareData>& result_ptr, bool &finsh,
        bool restart = false);
    int ListRecord(std::shared_ptr<WareData>& result_ptr);
   private:
    std::string set_name_;
    std::string model_version_;
    float distance_threshold_;
    float similar_threshold_;
    int page_size_ = 100;
    int cur_page_ = 0;
    int record_sum_ = 0;
    int page_num_ = 0;
  };

  std::shared_ptr<DB::DB_TABLE> CreateTable(std::string &lib_name,
                                            std::string &model_version);

  std::shared_ptr<DB_TABLE> GetTable(const std::string &lib_name,
                                     const std::string &model_version);
  int DropTable(const std::string &lib_name, const std::string &model_version);

 private:
  explicit DB(const std::string &table_path);
  DB(const DB &db) = delete;
  DB &operator=(const DB &db) = delete;

  float similar_thres_ = 0.7;
  std::string table_path_;
  std::string lib_name_ = "aiexpressfaceid";  // NOLINT
  std::string model_name_ = "X2_1.6";  // NOLINT
  std::map<std::string, std::shared_ptr<DB_TABLE>> table_map_;
};
}  // namespace video_box
}  // namespace solution

#endif  //  SOLUTION_VIDEO_BOX_WAREPLUGIN_WAREDB_H_

