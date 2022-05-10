/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     Method interface of xstream framework
 * @file method.h
 * @author    shuhuan.sun
 * @email     shuhuan.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.21
 */
#ifndef XSTREAM_METHOD_H_
#define XSTREAM_METHOD_H_

#include <memory>
#include <string>
#include <vector>

#include "xstream/version.h"
#include "xstream/xstream_data.h"

namespace xstream {

/**
 * @brief Method info
 * The MethodInfo structure defines the attribute information of the Method,
 * including: is_thread_safe_: Indicates whether the Method is thread safe, the
 * default value is false. When defining Workflow, it supports configuring the
 * number of Node threads. If the number of threads is configured to be greater
 * than 1, if the method calculation process is not thread-safe, multiple Method
 * instances will be defined under multithreading; otherwise, multiple threads
 * will reuse one Method instance. is_need_reorder_: Indicate whether the data
 * is required to call Method processing frame by frame in sequence. For
 * example, some target tracking scenarios require the output data to be
 * orderly. The default is false, which means that order is not required. Note:
 * In the method of is_need_reorder_=true, the number of threads is only allowed
 * to be 1 due to the dependency of the front and rear frame order. At the same
 * time, is_need_reorder_=true will reduce the execution efficiency to a certain
 * extent. is_src_ctx_dept_: Indicates whether the processing of input data
 * depends on the input source. For example, in a target tracking scene, the
 * previous few frames of data will be recorded and track_id will be assigned.
 * This type of method needs to set the attribute to is_src_ctx_dept_ = true.
 */
struct XSTREAM_EXPORT MethodInfo {
  // is thread safe
  bool is_thread_safe_ = false;
  // is need reorder, the order of outputdata must be same as the inputdata
  bool is_need_reorder_ = false;
  // is dependent on inputdata source
  bool is_src_ctx_dept_ = false;
};

class XSTREAM_EXPORT Method {
 public:
  virtual ~Method();
  /**
   * @description: Method's initialization function generally includes loading
   * configuration files, obtaining configuration parameters, and setting key
   * variables.
   * @param[in] config_file_path The path of the
   * configuration file corresponding to the Method.
   * @return The interface returns 0 to indicate that the initialization is
   * successful, otherwise it indicates that the initialization failed.
   */
  virtual int Init(const std::string &config_file_path) = 0;
  /**
   * @description: The core processsing function of Method
   * @param[in] input Indicates the data to be processed, input[i]
   * indicates the data of Batch i, currently only i=0 is supported.
   * @param[in] param Represents the parameter of Method, param[i] represents
   * the parameter of processing Batch i data.
   * @return  The interface returns the output result of Method, output[i]
   * represents the output data of Batch i.
   */
  virtual std::vector<std::vector<BaseDataPtr>> DoProcess(
      const std::vector<std::vector<BaseDataPtr>> &input,
      const std::vector<InputParamPtr> &param) = 0;
  /**
   * @description: The content that needs to be processed before the Method
   * object is destroyed, including the release of key variables, etc.
   * @param None
   * @return None
   */
  virtual void Finalize() = 0;

  /**
   * @brief Init from json string.The difference between the
   * initialization function of Method and the Init function is that the
   * passed-in parameter is the configuration content in Json format, not the
   * file path.
   * @param[in] config The configuration content
   * corresponding to the Method, in Json format.
   * @return The interface returns 0 to indicate that the initialization is
   * successful, otherwise it indicates that the initialization failed.
   */
  virtual int InitFromJsonString(const std::string &config);
  /**
   * @description: Update the parameters of Method.
   * @param[in] ptr Parameters to be updated.
   * @return The interface returns 0 to indicate that the update was successful,
   * otherwise it indicates that the update failed.
   */
  virtual int UpdateParameter(InputParamPtr ptr);
  /**
   * @description: Get the parameters of Method
   * @param None
   * @return  InputParamPtr
   */
  virtual InputParamPtr GetParameter() const;
  /**
   * @description: Get the version information of Method
   * @param None
   * @return The version string.
   */
  virtual std::string GetVersion() const;
  /**
   * @description: Get the attribute information of Method
   * @param None
   * @return Return the attribute information MethodInfo of Method
   */
  virtual MethodInfo GetMethodInfo();
};

typedef std::shared_ptr<Method> MethodPtr;

}  // namespace xstream

#endif  // XSTREAM_METHOD_H_
