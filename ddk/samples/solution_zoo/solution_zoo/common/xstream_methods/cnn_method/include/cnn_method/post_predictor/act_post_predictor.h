/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: ActPostPredictor.h
 * @Brief: declaration of the ActPostPredictor
 * @Author: shiyu.fu
 * @Email: shiyu.fu@horizon.ai
 * @Date: 2020-05-25
 */

#ifndef CNNMETHOD_POSTPREDICTOR_ACTPOSTPREDICTOR_H_
#define CNNMETHOD_POSTPREDICTOR_ACTPOSTPREDICTOR_H_

#include <memory>
#include <string>
#include <vector>

#include "cnn_method/cnn_const.h"
#include "cnn_method/post_predictor/post_predictor.h"
#include "cnn_method/util/act_data_post_process.h"

namespace xstream {
class ActPostPredictor : public PostPredictor {
 public:
  virtual int32_t Init(std::shared_ptr<CNNMethodConfig> config);
  virtual void Do(CNNMethodRunData *run_data);

 private:
  std::vector<BaseDataPtr> TargetPro(
      const std::vector<std::vector<int8_t>> &mxnet_outs,
      int dim_idx, int dim_size, int track_id, float* timestamp);

  std::vector<BaseDataPtr> DefaultVaule(int size);
  std::string groups_str_;
  int target_group_;
  int en_dump_feature_;
  int en_score_avg_;
  float threshold_;
  std::vector<float> thresholds_;
  std::vector<std::vector<int>> merge_groups_;
  LmkSeqOutputType output_type_;
  ActDataPostprocess post_util_;
};
}  // namespace xstream

#endif  // CNNMETHOD_POSTPREDICTOR_ACTPOSTPREDICTOR_H_
