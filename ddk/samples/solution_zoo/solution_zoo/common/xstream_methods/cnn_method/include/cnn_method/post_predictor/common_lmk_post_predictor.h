/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: CommonLmkPostPredictor.h
 * @Brief: declaration of the CommonLmkPostPredictor
 * @Author: fei.cheng
 * @Email: fei.cheng@horizon.ai
 * @Date: 2020-07-18 14:18:28
 * @Last Modified by: fei.cheng
 * @Last Modified time: 2019-07-18 15:13:07
 */

#ifndef INCLUDE_CNNMETHOD_POSTPREDICTOR_COMMONLMKPOSTPREDICTOR_H_
#define INCLUDE_CNNMETHOD_POSTPREDICTOR_COMMONLMKPOSTPREDICTOR_H_

#include <memory>
#include <string>
#include <vector>

#include "cnn_method/cnn_const.h"
#include "cnn_method/post_predictor/post_predictor.h"

namespace xstream {

class CommonLmkPostPredictor : public PostPredictor {
 public:
  int32_t Init(std::shared_ptr<CNNMethodConfig> config) override;
  virtual void Do(CNNMethodRunData *run_data);

 private:
  void HandleLmk(const std::vector<std::vector<int8_t>> &mxnet_outs,
                     const xstream::BBox &box,
                     const std::vector<std::vector<uint32_t>> &nhwc,
                     const std::vector<std::vector<uint32_t>> &shifts,
                     std::vector<BaseDataPtr> *output);

  BaseDataPtr Lmk3Post(const std::vector<std::vector<int8_t>> &mxnet_outs,
                       const xstream::BBox &box,
                       const std::vector<std::vector<uint32_t>> &nhwc,
                       const std::vector<std::vector<uint32_t>> &shifts);

  BaseDataPtr Lmk4Post(const std::vector<std::vector<int8_t>> &mxnet_outs,
                       const xstream::BBox &box);

  void Lmks4PostProcess(const float *vector_pred,
                        const xstream::BBox &box,
                        std::vector<xstream::Point> &lmks4,
                        int axis);

  int CalIndex(int k, int i);

  int CalIndex(int k, int i, int j);
  size_t lmk_num_;
  int feature_w_;
  int feature_h_;
  int i_o_stride_;
  std::string post_fn_;
  int vector_size_;
  NormMethod norm_type_;
  float expand_scale_;
  float aspect_ratio_;
};
}  // namespace xstream
#endif  // INCLUDE_CNNMETHOD_POSTPREDICTOR_HANDLMKPOSTPREDICTOR_H_
