/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief provides base data struct for xstream framework
 * @file xstream_data.h
 * @author    chuanyi.yang
 * @email     chuanyi.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.21
 * @update    2021.01.27 by hangjun.yang
 */

#ifndef XSTREAM_XSTREAM_DATA_H_
#define XSTREAM_XSTREAM_DATA_H_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "xstream/version.h"

namespace xstream {

/**
 * DataState defines several basic states of data.
 * In the process of development, data processing may rely on these states.
 * If the data is normally valid, it should be set to VALID.
 */
enum class DataState {
  /// valid
  VALID = 0,
  /// filtered
  FILTERED = 1,
  /// invisible
  INVISIBLE = 2,
  /// disappeared
  DISAPPEARED = 3,
  /// invalid
  INVALID = 4,
};

/**
 * @brief
 * The BaseData structure describes the basic information of the workflow within
 * the framework. The data passed between methods must be derived from BaseData.
 * BaseData data members including: type_: Descriptive information of
 * user-defined data, such as "box", etc. This field currently has no special
 * purpose and can be ignored. name_: The alias of the data. When the framework
 * is running, the name_ of the data is assigned to the name configured in the
 * workflow configuration file. For InputData, the name_ of the externally
 * specified data is required, and the specified name_ needs to be consistent
 * with the name configured in the workflow configuration. error_code_:
 * error_code_. error_detail_: Error information deatail c_data_: context of C
 * structure. It can be ignored. state_: Describe the state of the data, default
 * value is VALID.
 */
struct XSTREAM_EXPORT BaseData {
  BaseData() { type_ = "BaseData"; }
  virtual ~BaseData() {}
  /// type
  std::string type_ = "";
  /// name
  std::string name_ = "";
  /// error code
  int error_code_ = 0;
  /// error detail info
  std::string error_detail_ = "";
  /// data status
  DataState state_ = DataState::VALID;
};

typedef std::shared_ptr<BaseData> BaseDataPtr;

/**
 * @brief
 * BaseDataVector is a derived class of BaseData, used to represent array data,
 * such as saving all body bbox in a frame of image, each element in the datas_
 * array points to a body bbox data. datas_: Used to save each value in the
 * array.
 */
struct XSTREAM_EXPORT BaseDataVector : public BaseData {
  BaseDataVector() { type_ = "BaseDataVector"; }

  std::vector<BaseDataPtr> datas_;
};

/**
 * @brief
 * InputParam is the data structure of the input data parameter. including:
 * unique_name_: Indicates the unique identifier that this parameter acts on a
 * specific node, corresponding to the Method name in the workflow
 * configuration. is_json_format_: Indicates whether the input parameter is a
 * json string. If it is true, you can get the json string through the Format()
 * interface. is_enable_this_method_: Indicates whether to enable the node. If
 * it is true, this parameter is passed into the node, and the node operates
 * normally; if it is false, the node operation is skipped. Note:When preparing
 * InputData, you can update the configuration parameters of Method through
 * InputParam. Users can inherit this class, define specific parameter types,
 *      and then the related Method uses the newly defined data type to parse
 * the configuration parameters.
 */
class XSTREAM_EXPORT InputParam {
 public:
  explicit InputParam(const std::string &unique_name) {
    unique_name_ = unique_name;
    is_json_format_ = false;
    is_enable_this_method_ = true;
  }
  virtual ~InputParam() = default;
  virtual std::string Format() = 0;

 public:
  bool is_json_format_;
  bool is_enable_this_method_;
  std::string unique_name_;
};

typedef std::shared_ptr<InputParam> InputParamPtr;

/**
 * @brief
 * The InputData structure describes the basic information of the frame's input
 * data flow. Including: datas_: The BaseData array represents the data input
 * for one frame. params_: Input Param array, note that there is no positional
 * correspondence between parameter params_ and input data datas_. There are
 * data members in InputParam that describe which Method the InputParam
 * configuration corresponds to. source_id_: Input source number, which
 * indicates the unique identification of the input source. Multiple input
 * sources are numbered starting from 0, and the single input source defaults to
 * 0. source_id_ is used to support multiple channels. context_: Transparent
 * transmission of data, the data will be transparently transmitted to
 * OutputData, which can be used for special needs.
 */
struct XSTREAM_EXPORT InputData {
  /// input data, such as image, timestamp, box..
  std::vector<BaseDataPtr> datas_;
  /// InputParam
  std::vector<InputParamPtr> params_;
  /// input_data source id, used in multi-input; default value 0(single-input)
  uint32_t source_id_ = 0;
  /// context_ pasthroughed to OutputData::context_
  const void *context_ = nullptr;
};
typedef std::shared_ptr<InputData> InputDataPtr;

/**
 * @brief
 * The OutputData structure describes the basic information of the output data
 * stream of the framework. including: error_code_: Error code, non-zero means
 * that the prediction is error. error_detail_: Error information description.
 * unique_name_: the output data of a specific node,
 *               and unique_name_ corresponds to the name of the Method that
 * generated the data in the workflow configuration. output_type_: The name of
 * the group of output data in multiple output, the default is
 * "NODE_WHOLE_OUTPUT" for single output. datas_: all datas of one frame
 * prediction. context_: Transparent transmission field, which is passed in by
 * the input data InputData.context_ of the frame data. sequence_id_: The output
 * data corresponds to the sequence number of the input frame of the input
 * source, starting from 0 and accumulating sequentially. source_id_: Input
 * source number, which indicates the unique identification of the input source.
 *             Multiple input sources are numbered starting from 0, and the
 * single input source defaults to 0. source_id_ is used to support multiple
 * channels. global_sequence_id_: In the case of multiple input sources, it
 * corresponds to the total sequence number of the input frame; in the case of
 * single input source, it is the same as sequence_id_.
 */
struct XSTREAM_EXPORT OutputData {
  /// error code
  int error_code_ = 0;
  /// error info
  std::string error_detail_ = "";
  /// used in Node's outputdata, represents node's name
  std::string unique_name_ = "";
  /// outputdata name
  std::string output_type_ = "";
  /// output data, such as detection_box, landmarks..
  std::vector<BaseDataPtr> datas_;
  /// passthroughed from inputdata
  const void *context_ = nullptr;
  /// sequence_id in single-input
  int64_t sequence_id_ = 0;
  /// input_data source id, used in multi-input; default value 0(single-input)
  uint32_t source_id_ = 0;
  uint64_t global_sequence_id_ = 0;
};
typedef std::shared_ptr<OutputData> OutputDataPtr;

/// callback func
typedef std::function<void(OutputDataPtr)> XStreamCallback;

/**
 * \brief pre-difined param for method
 */
class XSTREAM_EXPORT DisableParam : public InputParam {
 public:
  enum class Mode {
    /// passthrough mode: use inputdata as outputdata;
    /// slot size of outputdata and inputdata must be equal;
    PassThrough,
    /// use pre-difined data as outputdata;
    /// slot size of predifine data and outputdata must be equal;
    UsePreDefine,
    /// set outputdata status as DataState::INVALID
    Invalid,
    /// flexibly passthrough mode
    /// 1. passthrough inputdata to outputdata in order
    /// 2. if slot size of inputdata greater than outputdata,
    ///    only use the first few slots.
    /// 3. if slot size of inputdata less than outputdata,
    ///    set status of last few slots invalid.
    BestEffortPassThrough
  };
  explicit DisableParam(const std::string &unique_name,
                        Mode mode = Mode::Invalid)
      : InputParam(unique_name), mode_{mode} {
    is_enable_this_method_ = false;
  }
  virtual ~DisableParam() = default;
  virtual std::string Format() { return unique_name_ + " : disabled"; }
  Mode mode_;
  /// pre-difined data, used in Mode::UsePreDefine
  std::vector<BaseDataPtr> pre_datas_;
};

typedef std::shared_ptr<DisableParam> DisableParamPtr;

/**
 * @brief
 * SdkCommParam inherits from InputParam, and describes parameters in json.
 * Get the json string by calling the Format().
 */
class XSTREAM_EXPORT SdkCommParam : public InputParam {
 public:
  SdkCommParam(const std::string &unique_name, const std::string &param)
      : InputParam(unique_name) {
    param_ = param;
    is_json_format_ = true;
  }
  virtual std::string Format() { return param_; }
  virtual ~SdkCommParam() = default;

 public:
  std::string param_;  // json format
};
typedef std::shared_ptr<SdkCommParam> CommParamPtr;

}  // namespace xstream

#endif  // XSTREAM_XSTREAM_DATA_H_
