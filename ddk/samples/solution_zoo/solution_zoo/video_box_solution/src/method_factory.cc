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

#include "facesnap_filter_method/facesnap_filter_method.h"
#include "grading_method/grading_method.h"
#include "merge_method/merge_method.h"
#include "mot_method/mot_method.h"
#include "multitask_postprocess_method/multitask_postprocess_method.h"
#include "multitask_predict_method/multitask_predict_method.h"
#include "snapshot_method/snapshot_method.h"
#include "vote_method/vote_method.h"
#include "xstream/xstream_world.h"
#include "model_inference/inference_method.h"

namespace xstream {
namespace method_factory {
MethodPtr CreateMethod(const std::string &method_name) {
  if ("MOTMethod" == method_name) {
    return MethodPtr(new MOTMethod());
  } else if ("MergeMethod" == method_name) {
    return MethodPtr(new MergeMethod());
  } else if ("VoteMethod" == method_name) {
    return MethodPtr(new VoteMethod());
  } else if ("MultitaskPredictMethod" == method_name) {
    return MethodPtr(new MultitaskPredictMethod());
  } else if ("MultitaskPostProcessMethod" == method_name) {
    return MethodPtr(new MultitaskPostProcessMethod());
  } else if ("FaceSnapFilterMethod" == method_name) {
    return MethodPtr(new FaceSnapFilterMethod());
  } else if ("GradingMethod" == method_name) {
    return MethodPtr(new GradingMethod());
  } else if ("SnapShotMethod" == method_name) {
    return MethodPtr(new SnapShotMethod());
  } else if ("InferMethod" == method_name) {
    return MethodPtr(new inference::InferMethod());
  } else if ("PostMethod" == method_name) {
    return MethodPtr(new inference::PostMethod());
  } else {
    return MethodPtr();
  }
}
}  //  namespace method_factory
}  //  namespace xstream
