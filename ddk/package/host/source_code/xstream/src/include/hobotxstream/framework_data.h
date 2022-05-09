/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief provides data structure for xstream framework
 * @file framework_data.h
 * @author    chuanyi.yang
 * @email     chuanyi.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.21
 */
#ifndef HOBOTXSTREAM_FRAMEWORK_DATA_H_
#define HOBOTXSTREAM_FRAMEWORK_DATA_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "xstream/xstream_data.h"

namespace xstream {
/// BaseData status
enum BaseDataState {
  DataState_None = 0,
  DataState_Doing,
  DataState_Ready,
  DataState_Error,
};
/// FrameworkData status
enum FrameworkDataState {
  FrameworkDataState_None = 0,
  FrameworkDataState_Doing,
  FrameworkDataState_Ready,
  FrameworkDataState_Error,
};
/// FrameworkData, used in Framework
struct FrameworkData {
  /// datas in slots
  std::vector<BaseDataPtr> datas_;
  /// datas status in slots
  std::vector<BaseDataState> datas_state_;
  /// inputparam of node
  std::unordered_map<std::string, InputParamPtr> method_param_;
  /// name of outputdata has been handled
  std::vector<std::string> output_type_isout_;
  /// the number of nodes have been driven by dataslots
  std::vector<int> driven_nodes_nums_;
  /// pasthroughed from context_ of Inputdata
  const void *context_ = nullptr;
  /// flag data of sync mode
  void *sync_context_ = nullptr;
  /// timestamp
  uint64_t timestamp_;
  /// sequence_id, used to reorder
  uint64_t sequence_id_;
  /// input_data source id, used in multi-input; default value 0(single-input)
  uint32_t source_id_ = 0;
  uint32_t golbal_squence_id_;
  /// framework data status
  FrameworkDataState state_ = FrameworkDataState_None;
};
typedef std::shared_ptr<FrameworkData> FrameworkDataPtr;

/// FrameworkDataBatch
struct FrameworkDataBatch {
  /// batch frameworkdata
  std::vector<FrameworkDataPtr> datas_;
  /// timestamp
  int64_t timestamp_;
};
typedef std::shared_ptr<FrameworkDataBatch> FrameworkDataBatchPtr;

}  // namespace xstream

#endif  // HOBOTXSTREAM_FRAMEWORK_DATA_H_
