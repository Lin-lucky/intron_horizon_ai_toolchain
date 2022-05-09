/**
 * @copyright Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @file      main.cc
 * @brief     main function
 * @author    Qingpeng Liu (qingpeng.liu@horizon.ai)
 * @version   0.0.0.1
 * @date      2020-01-08
 */

#ifndef XSTREAM_FRAMEWORK_TUTORIALS_STAGE6_INCLUDE_METHOD_CALLBACK_H_
#define XSTREAM_FRAMEWORK_TUTORIALS_STAGE6_INCLUDE_METHOD_CALLBACK_H_

#include "bbox.h"

namespace Stage6Async {

class Callback {
 public:
  void OnCallback(xstream::OutputDataPtr output) { ParseOutput(output); }

 private:
  void ParseOutput(xstream::OutputDataPtr output) {
    using xstream::BaseDataVector;

    std::cout << "seq: " << output->sequence_id_ << std::endl;
    std::cout << "output_type: " << output->output_type_ << std::endl;
    std::cout << "method_unique_name: " << output->unique_name_ << std::endl;
    std::cout << "error_code: " << output->error_code_ << std::endl;
    std::cout << "error_detail_: " << output->error_detail_ << std::endl;
    std::cout << "datas_ size: " << output->datas_.size() << std::endl;
  }
};
}  // namespace Stage6Async

#endif  // XSTREAM_FRAMEWORK_TUTORIALS_STAGE6_INCLUDE_METHOD_CALLBACK_H_
