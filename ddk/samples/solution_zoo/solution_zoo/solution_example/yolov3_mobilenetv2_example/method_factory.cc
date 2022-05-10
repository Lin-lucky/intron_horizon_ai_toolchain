/**
 * @file method_factory.cpp
 * @author hangjun.yang
 * @brief 
 * @version 0.1
 * @date 2020-12-25
 *
 * @copyright Copyright (c) 2018
 *
 */

#include <string>
#include "xstream/xstream_world.h"
#include "model_inference/inference_method.h"

namespace xstream {
namespace method_factory {
MethodPtr CreateMethod(const std::string &method_name) {
  if ("InferMethod" == method_name) {
    return MethodPtr(new inference::InferMethod());
  } else if ("PostMethod" == method_name) {
    return MethodPtr(new inference::PostMethod());
  } else {
    return MethodPtr();
  }
}
}  //  namespace method_factory
}  //  namespace xstream
