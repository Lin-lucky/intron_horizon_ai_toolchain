/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     Method interface of xstream framework
 * @file      simple_method.h
 * @author    shuhuan.sun
 * @email     shuhuan.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.21
 */
#ifndef XSTREAM_SIMPLE_METHOD_H_
#define XSTREAM_SIMPLE_METHOD_H_

#include <memory>
#include <string>
#include <vector>

#include "xstream/version.h"
#include "xstream/method.h"
#include "xstream/xstream_data.h"

namespace xstream {
class XSTREAM_EXPORT SimpleMethod : public Method {
 public:
  virtual ~SimpleMethod();

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
   * @param {*} input[in], Indicates the data to be processed, input[i]
   * indicates the data of Batch i, currently only i=0 is supported. param[in],
   * Represents the parameter of Method, param[i] represents the parameter of
   * processing Batch i data.
   * @return {*} The interface returns the output result of Method, output[i]
   * represents the output data of Batch i.
   */
  virtual std::vector<BaseDataPtr> DoProcess(
      const std::vector<BaseDataPtr> &input, const InputParamPtr &param) = 0;

  /**
   * @description: The content that needs to be processed before the Method
   * object is destroyed, including the release of key variables, etc.
   * @param {*} none
   * @return {*} none
   */
  virtual void Finalize() = 0;

  /**
   * @description: Init from json string.The difference between the
   * initialization function of Method and the Init function is that the
   * passed-in parameter is the configuration content in Json format, not the
   * file path.
   * @param {const std::string} config[in], The configuration content
   * corresponding to the Method, in Json format.
   * @return The interface returns 0 to indicate that the initialization is
   * successful, otherwise it indicates that the initialization failed.
   */
  virtual int InitFromJsonString(const std::string &config);

  /**
   * @description: Update the parameters of Method.
   * @param {InputParamPtr} ptr[in], parameters to be updated.
   * @return The interface returns 0 to indicate that the update was successful,
   * otherwise it indicates that the update failed.
   */
  virtual int UpdateParameter(InputParamPtr ptr);

  /**
   * @description: Get the parameters of Method
   * @param {*}
   * @return {*} InputParamPtr
   */
  virtual InputParamPtr GetParameter() const;

  /**
   * @description: Get the version information of Method
   * @param {*}
   * @return {*}
   */
  virtual std::string GetVersion() const;

  /**
   * @description: Get the attribute information of Method
   * @param {*}
   * @return {*} Return the attribute information MethodInfo of Method
   */
  virtual MethodInfo GetMethodInfo();
  // <parameter> input: input data, input[i][j]: batch i, slot j
  std::vector<std::vector<BaseDataPtr>> DoProcess(
      const std::vector<std::vector<BaseDataPtr>> &input,
      const std::vector<InputParamPtr> &param) final;
};

typedef std::shared_ptr<SimpleMethod> SimpleMethodPtr;

}  // namespace xstream

#endif  // XSTREAM_SIMPLE_METHOD_H_
