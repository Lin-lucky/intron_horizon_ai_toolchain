/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     WeightGrading Method
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.25
 */

#ifndef GRADINGMETHOD_WEIGHTGRADING_H_
#define GRADINGMETHOD_WEIGHTGRADING_H_

#include <memory>
#include <string>
#include <vector>

#include "grading_method.h"
#include "grading_method_data_type.h"
#include "json/json.h"
#include "xstream/vision_type.h"

namespace xstream {

struct WeightGradingParam : GradingParam {
 public:
  explicit WeightGradingParam(const std::string &content)
      : GradingParam(content) {
    UpdateParameter(content);
  }
  int UpdateParameter(const std::string &content) override;
  float size_min = 0;
  float size_max = 0;
  float size_inflexion = 0;
  float frontal_thr = 0;
  float size_weight = 0;
  float pose_weight = 0;
  float lmk_weight = 0;
  float quality_weight = 0;
  float normalize_lmk_divisor = 75;
};

struct XStreamFloat : public BaseData {
  float value;
  XStreamFloat() {}
  explicit XStreamFloat(const float &val) { value = val; }
};

class WeightGrading : public Grading {
 public:
  int GradingInit(const std::string &config_file_path) override;

  int ProcessFrame(const std::vector<BaseDataPtr> &in,
                   const InputParamPtr &param,
                   std::vector<BaseDataPtr> *out) override;

  void GradingFinalize() override;

  InputParamPtr GetParameter() override;

  int UpdateParameter(const std::string &content) override;

 private:
  float ProcessRect(const xstream::BBox &bbox);

  float ProcessPose3d(const xstream::Pose3D &pose3d);

  float ProcessLmk(const xstream::Landmarks &lmk);

  float ProcessQuality(const xstream::Quality &quality);

  std::shared_ptr<WeightGradingParam> config_param_;
};
}  //  namespace xstream

#endif  //  GRADINGMETHOD_WEIGHTGRADING_H_
