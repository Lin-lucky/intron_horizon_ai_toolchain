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
#include "grading_method/grading_method.h"
#include "merge_method/merge_method.h"
#include "mot_method/mot_method.h"
#include "multitask_postprocess_method/multitask_postprocess_method.h"
#include "multitask_predict_method/multitask_predict_method.h"
#include "rect_input_predict_method/rect_input_predict_method.h"
#include "vehicle_color_postprocess_method/vehicle_color_postprocess_method.h"
#include "vehicle_color_predict_method/vehicle_color_predict_method.h"
#include "plate_num_postprocess_method/plate_num_postprocess_method.h"
#include "vehicle_plate_match_method/vehicle_plate_match_method.h"
#include "plate_vote_method/plate_vote_method.h"
#include "vote_method/vote_method.h"
#include "vehicle_type_postprocess_method/vehicle_type_postprocess_method.h"
#include "snapshot_method/snapshot_method.h"
#include "xstream/user_method_factory.h"
#include "model_inference/inference_method.h"

namespace xstream {
namespace method_factory {
MethodPtr CreateMethod(const std::string &method_name) {
  if ("MOTMethod" == method_name) {
    return MethodPtr(new xstream::MOTMethod());
  } else if ("MergeMethod" == method_name) {
    return MethodPtr(new xstream::MergeMethod());
  } else if ("FilterMethod" == method_name) {
    return MethodPtr(new xstream::FilterMethod());
  } else if ("GradingMethod" == method_name) {
    return MethodPtr(new xstream::GradingMethod());
  } else if ("SnapShotMethod" == method_name) {
    return MethodPtr(new SnapShotMethod());
  } else if ("VoteMethod" == method_name) {
    return MethodPtr(new xstream::VoteMethod());
  } else if ("MultitaskPredictMethod" == method_name) {
    return MethodPtr(new xstream::MultitaskPredictMethod());
  } else if ("MultitaskPostProcessMethod" == method_name) {
    return MethodPtr(new xstream::MultitaskPostProcessMethod());
  } else if ("RectInputPredictMethod" == method_name) {
    return MethodPtr(new xstream::RectInputPredictMethod());
  } else if ("VehicleColorPredictMethod" == method_name) {
    return MethodPtr(new xstream::VehicleColorPredictMethod());
  } else if ("VehicleColorPostProcessMethod" == method_name) {
    return MethodPtr(new xstream::VehicleColorPostProcessMethod());
  } else if ("PlateNumPostProcessMethod" == method_name) {
    return MethodPtr(new xstream::PlateNumPostProcessMethod());
  } else if ("VehiclePlateMatchMethod" == method_name) {
    return MethodPtr(new xstream::VehiclePlateMatchMethod());
  } else if ("PlateVoteMethod" == method_name) {
    return MethodPtr(new xstream::PlateVoteMethod());
  } else if ("VehicleTypePostProcessMethod" == method_name) {
    return MethodPtr(new xstream::VehicleTypePostProcessMethod());
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
