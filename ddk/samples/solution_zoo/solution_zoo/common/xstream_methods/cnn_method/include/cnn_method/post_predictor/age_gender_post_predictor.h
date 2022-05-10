/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: AgeGenderPostPredictor.h
 * @Brief: declaration of the AgeGenderPostPredictor
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-07-17 14:18:28
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-07-17 15:13:07
 */

#ifndef INCLUDE_CNNMETHOD_POSTPREDICTOR_AGEGENDERPOSTPREDICTOR_H_
#define INCLUDE_CNNMETHOD_POSTPREDICTOR_AGEGENDERPOSTPREDICTOR_H_

#include <vector>

#include "cnn_method/post_predictor/post_predictor.h"

namespace xstream {

class AgeGenderPostPredictor : public PostPredictor {
 public:
  virtual void Do(CNNMethodRunData *run_data);

 private:
  void HandleAgeGender(const std::vector<std::vector<int8_t>> &mxnet_outs,
                       std::vector<BaseDataPtr> *output);
  BaseDataPtr AgePostPro(const std::vector<int8_t> &mxnet_out);
  BaseDataPtr GenderPostPro(const std::vector<int8_t> &mxnet_out);
};
}  // namespace xstream
#endif  // INCLUDE_CNNMETHOD_POSTPREDICTOR_AGEGENDERPOSTPREDICTOR_H_
