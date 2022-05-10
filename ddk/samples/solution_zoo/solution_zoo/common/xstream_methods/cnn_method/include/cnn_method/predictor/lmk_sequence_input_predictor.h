/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: LmkSeqInputPredictor.h
 * @Brief: declaration of the LmkSeqInputPredictor
 * @Author: shiyu.fu
 * @Email: shiyu.fu@horizon.ai
 * @Date: 2020-05-25
 */

#ifndef INCLUDE_CNNMETHOD_PREDICTOR_LMKSEQINPUTPREDICTOR_H_
#define INCLUDE_CNNMETHOD_PREDICTOR_LMKSEQINPUTPREDICTOR_H_

#include <memory>
#include "cnn_method/predictor/predictor.h"
#include "cnn_method/util/cnn_method_data.h"
#include "cnn_method/util/act_data_preprocess.h"
#include "cnn_method/cnn_const.h"

namespace xstream {

class LmkSeqInputPredictor : public Predictor {
 public:
  virtual int32_t Init(std::shared_ptr<CNNMethodConfig> config);
  virtual void Do(CNNMethodRunData *run_data);
  virtual void UpdateParam(std::shared_ptr<CNNMethodConfig> config);

 private:
  int input_shift_ = 7;
  int seq_len_ = 0;
  int kps_len_ = 17;
  int buf_len_ = 64;
  float stride_ = 0.04;
  float max_gap_ = 0.04;
  float kps_norm_scale_ = 1.0;
  bool norm_kps_conf_ = true;
  NormParams norm_params_;
  ActDataPreprocess data_processor_;
  LmkSeqOutputType output_type_;
};
}  // namespace xstream
#endif  // INCLUDE_CNNMETHOD_PREDICTOR_LMKINPUTPREDICTOR_H_
