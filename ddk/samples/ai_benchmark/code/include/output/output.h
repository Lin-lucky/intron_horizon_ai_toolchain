// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _OUTPUT_OUTPUT_H_
#define _OUTPUT_OUTPUT_H_

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "base/perception_common.h"
#include "input/input_data.h"
#include "rapidjson/document.h"

class OutputModule;

class OutputAssembler {
 public:
  ~OutputAssembler();

  OutputAssembler(const OutputAssembler &) = delete;

  OutputAssembler &operator=(const OutputAssembler &) = delete;

  static OutputAssembler *GetIns();

  int Init(const std::string &config);

  void Send(ImageTensor *data, Perception *perception);

 private:
  OutputAssembler() = default;

  std::vector<OutputModule *> output_list_;
};

class OutputModule {
 public:
  explicit OutputModule(std::string module_name)
      : module_name_(std::move(module_name)) {}

  /**
   * Init OutputModule from file
   * @param[in] config_file: config file path
   * @return 0 if success
   */
  virtual int Init(std::string config_file, std::string config_string);

  virtual int Init(rapidjson::Document &document) = 0;

  /**
   * Write perception data to output
   * @param[in] frame: frame info
   * @param[in] perception: perception data
   */
  virtual void Write(ImageTensor *frame, Perception *perception) = 0;

  /**
   * Get OutputModule Implementation instance
   * @param[in]: module_name
   * @return OutputModule implementation instance
   */
  static OutputModule *GetImpl(const std::string &module_name);

  virtual ~OutputModule() = default;

 protected:
  int LoadConfigFile(std::string &config_file);

  virtual int LoadConfig(std::string &config_string) { return 0; }

 private:
  std::string module_name_;
};

#define DEFINE_AND_REGISTER_OUTPUT(output_name, class_name)               \
  OutputModule *output_name##_output_creator() { return new class_name; } \
  static OutputRegistry process_registry(#output_name,                    \
                                         output_name##_output_creator);

typedef OutputModule *(*OutputCreator)();

class OutputModuleFactory {
 public:
  static OutputModuleFactory *GetInstance() {
    static OutputModuleFactory ins;
    return &ins;
  }

  OutputModule *GetOutputModule(const char *output_module_name) {
    if (output_process_registry_.find(output_module_name) ==
        output_process_registry_.end()) {
      VLOG(EXAMPLE_SYSTEM) << "process " << output_module_name
                           << " has not been registered.";
      return nullptr;
    }
    return output_process_registry_[output_module_name]();
  }

  int32_t OutputRegister(const char *output_module_name, OutputCreator func) {
    if (output_process_registry_.find(output_module_name) !=
        output_process_registry_.end()) {
      VLOG(EXAMPLE_DEBUG) << "process " << output_module_name
                          << " has been registered.";
    }
    output_process_registry_[output_module_name] = func;
    return 0;
  }

  std::unordered_map<std::string, OutputCreator> output_process_registry_;
};

class OutputRegistry {
 public:
  explicit OutputRegistry(const char *output_module_name,
                          OutputCreator func) noexcept {
    OutputModuleFactory::GetInstance()->OutputRegister(output_module_name,
                                                       func);
  }
};

#endif  // _OUTPUT_OUTPUT_H_
