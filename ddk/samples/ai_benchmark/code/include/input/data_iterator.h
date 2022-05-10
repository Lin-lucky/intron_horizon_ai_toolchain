// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _INPUT_DATA_ITERATOR_H_
#define _INPUT_DATA_ITERATOR_H_

#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "input/input_data.h"

class DataIterator {
 public:
  explicit DataIterator(std::string module_name)
      : module_name_(std::move(module_name)) {}

  /**
   * Init Data iterator from file
   * @param[in] config_file: config file path
   *            the config file should be in json format
   * @param[in] config_string: config string
   *            same as config_file
   * @return 0 if success
   */
  virtual int Init(std::string config_file,
                   std::string config_string,
                   hbDNNTensorLayout tensor_layout);

  /**
   * Next Data
   * @param[out] image_tensor: image tensor
   * @return 0 if success
   */
  virtual bool Next(ImageTensor *image_tensor) = 0;

  /**
   * Release image_tensor
   * @param[in] image_tensor: image tensor to be released
   */
  virtual void Release(ImageTensor *image_tensor) = 0;

  /**
   * Check if has next input data
   * @return 0 if has next input data
   */
  virtual bool HasNext() = 0;

  /**
   * Get next frame id
   * @return next frame id
   */
  virtual int NextFrameId() { return ++last_frame_id; }

  /**
   * Get DataIterator Implementation instance
   * @param[in]: module_name
   * @return DataIterator implementation instance
   */
  static DataIterator *GetImpl(const std::string &module_name);

  virtual ~DataIterator() = default;

 protected:
  virtual int LoadConfig(std::string &config_string) { return 0; }

 private:
  int LoadConfigFile(std::string &config_file);

 private:
  std::string module_name_;

 protected:
  bool is_finish_ = false;
  int last_frame_id = -1;
  hbDNNTensorLayout tensor_layout_ = HB_DNN_LAYOUT_NHWC;
};

#define DEFINE_AND_REGISTER_DATA_ITERATOR(iterator_name, class_name)       \
  DataIterator *iterator_name##_input_creator() { return new class_name; } \
  static DataIteratorRegistry process_registry(#iterator_name,             \
                                               iterator_name##_input_creator);

typedef DataIterator *(*inputCreator)();

class DataIteratorFactory {
 public:
  static DataIteratorFactory *GetInstance() {
    static DataIteratorFactory ins;
    return &ins;
  }

  DataIterator *GetDataIterator(const char *data_iterator_name) {
    if (input_process_registry_.find(data_iterator_name) ==
        input_process_registry_.end()) {
      VLOG(EXAMPLE_SYSTEM) << "process " << data_iterator_name
                           << " has not been registered.";
      return nullptr;
    }
    return input_process_registry_[data_iterator_name]();
  }

  int32_t InputRegister(const char *data_iterator_name, inputCreator func) {
    if (input_process_registry_.find(data_iterator_name) !=
        input_process_registry_.end()) {
      VLOG(EXAMPLE_DEBUG) << "process " << data_iterator_name
                          << " has been registered.";
    }
    input_process_registry_[data_iterator_name] = func;
    return 0;
  }

  std::unordered_map<std::string, inputCreator> input_process_registry_;
};

class DataIteratorRegistry {
 public:
  explicit DataIteratorRegistry(const char *data_iterator_name,
                                inputCreator func) noexcept {
    DataIteratorFactory::GetInstance()->InputRegister(data_iterator_name, func);
  }
};

#endif  // _INPUT_DATA_ITERATOR_H_
