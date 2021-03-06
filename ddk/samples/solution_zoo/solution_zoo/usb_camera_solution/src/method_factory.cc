/**
 * @file method_factory.cpp
 * @author your name (you@domain.com)
 * @brief DO NOT MODIFY THIS FILE, WHICH IS AUTO GENERATED BY COMPILER
 * @version 0.1
 * @date 2018-11-23
 *
 * @copyright Copyright (c) 2018
 *
 */

#include <string>

#include "filter_method/filter_method.h"
#include "lowpass_filter_method/lowpass_filter_method.h"
#include "merge_method/merge_method.h"
#include "mot_method/mot_method.h"
#include "vote_method/vote_method.h"
#include "model_inference/inference_method.h"
#include "xstream/user_method_factory.h"
#include "contours_method/contours_method.h"
#include "distance_method/distance_method.h"

namespace xstream {
namespace method_factory {
using xstream::MethodPtr;
MethodPtr CreateMethod(const std::string &method_name) {
  if ("MOTMethod" == method_name) {
    return MethodPtr(new xstream::MOTMethod());
  } else if ("MergeMethod" == method_name) {
    return MethodPtr(new xstream::MergeMethod());
  } else if ("VoteMethod" == method_name) {
    return MethodPtr(new xstream::VoteMethod());
  } else if ("FilterMethod" == method_name) {
    return MethodPtr(new xstream::FilterMethod());
  } else if ("LowPassFilterMethod" == method_name) {
    return MethodPtr(new xstream::LowPassFilterMethod());
  } else if ("InferMethod" == method_name) {
    return MethodPtr(new inference::InferMethod());
  } else if ("PostMethod" == method_name) {
    return MethodPtr(new inference::PostMethod());
  } else if ("ContoursMethod" == method_name) {
    return MethodPtr(new xstream::ContoursMethod());
  } else if ("DistanceMethod" == method_name) {
    return MethodPtr(new xstream::DistanceMethod());
  } else {
    return MethodPtr();
  }
}
}  // namespace method_factory
}  // namespace xstream
