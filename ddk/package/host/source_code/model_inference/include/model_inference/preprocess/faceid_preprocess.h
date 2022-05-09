/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of horizon_lmk_input_preprocess
 * @file   horizon_lmk_input_preprocess.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.05.19
 */
#ifndef FACEID_PREPROCESS_H_
#define FACEID_PREPROCESS_H_

#include <string>
#include <memory>
#include <vector>
#include "model_inference/preprocess/preprocess.h"
#include "opencv2/opencv.hpp"
#include "xstream/xstream_world.h"

namespace inference {

// 2、从内存读取一个原图，根据用户配置的image_process_pipeline对图像进行预处理
class FaceIDPreProcess : public PreProcess {
 public:
  explicit FaceIDPreProcess(Inferencer* infer) : PreProcess(infer) {}
  virtual ~FaceIDPreProcess() {}

  virtual int Init(const std::string &json_str) {
    return 0;
  }

  virtual int Execute(const std::vector<xstream::BaseDataPtr> &input,
                      std::vector<std::shared_ptr<InferenceEngineTask>>& tasks);

 private:
  const std::vector<float> lmk_template_ = {
    38.2946f, 51.6963f, 73.5318f, 51.5014f, 56.0252f,
    71.7366f, 41.5493f, 92.3655f, 70.7299f, 92.2041f};

  int AlignFace(const std::vector<float> &lmks_pts, const cv::Mat &origImg,
                cv::Mat &dstImg, float fill_value,
                std::vector<float> coord5points);
  int Cp2tform(std::vector<float> &uv, std::vector<float> &xy, cv::Mat &trans);
  int FindNonReflectiveSimilarity(std::vector<float> &uv,
                                  std::vector<float> &xy,
                                  cv::Mat &T,
                                  cv::Mat &Tinv);
  int GetAffinePoints(std::vector<float> &pts_in, cv::Mat &trans,
                      std::vector<float> &pts_out);
};

}  // namespace inference

#endif  // FACEID_PREPROCESS_H_
