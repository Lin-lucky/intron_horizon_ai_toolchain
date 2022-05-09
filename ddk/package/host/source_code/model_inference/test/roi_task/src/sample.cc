/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  sample of Roi Resizer Inferencer
 * @file   sample.cc
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.05.12
 */

#include "xstream/xstream_world.h"
#include "xstream/vision_type.h"
#include "model_inference/inference_task.h"
#include "model_inference/inferencer.h"
#include "hobotlog/hobotlog.hpp"

#include "src/utils.h"
#include "model_inference/inference_method.h"

#include "video_source/video_source.h"
#include "video_source/video_source_type.h"

int main(int argc, char* argv[]) {
  if (argc < 3) {
    std::cout << "exec vio_config infer_config" << std::endl;
    return -1;
  }

  std::string vio_config = argv[1];
  std::ifstream infile(vio_config);
  if (!infile) {
    std::cout << "vio_config file: " << vio_config << ", is not exist!!\n\n";
    return -1;
  }

  std::string infer_config = argv[2];

  SetLogLevel(HOBOT_LOG_DEBUG);
  int ret = 0;

  inference::InferenceEngine::SetPredictType(inference::DNN_PREDICT);

  inference::InferMethod infer_method;
  ret = infer_method.Init(infer_config);
  if (ret != 0) {
    LOGE << "InferMethod Init failed";
    return -1;
  }
  LOGD << "InferMethod Init success";

  std::vector<xstream::BaseDataPtr> input;
  std::vector<xstream::BaseDataPtr> output;
  xstream::InputParamPtr param;

  std::shared_ptr<xstream::PyramidImageFrame> pym_frame = nullptr;
  std::shared_ptr<videosource::PyramidFrame> pym_image = nullptr;
  auto video_source = std::make_shared<videosource::VideoSource>(0, vio_config);
  {
    ret = video_source->Init();
    if (ret) {
      std::cout << "video source init failed, ret: " << ret << std::endl;
      return ret;
    }
    ret = video_source->Start();
    if (ret) {
      std::cout << "video source start failed, ret: " << ret << std::endl;
      return ret;
    }
    bool vin_out_en = video_source->GetVinOutEnable();
    bool vps_en = video_source->GetVpsEnable();
    // 1. get vin output
    if (vin_out_en == true) {
      std::shared_ptr<videosource::ImageFrame> vin_image = nullptr;
      ret = video_source->GetVinImageFrame(vin_image);
      if (ret) {
        std::cout << "get vin image frame failed, ret: " << ret << std::endl;
      }
      ret = video_source->FreeVinImageFrame(vin_image);
      if (ret) {
        std::cout << "free vin image frame failed, ret: " << ret << std::endl;
      }
    }

    // 2. get vps output
    if (vps_en == true) {
      ret = video_source->GetPyramidFrame(pym_image);
      if (ret) {
        std::cout << "get pyramid frame failed, ret: " << ret << std::endl;
      } else {
        // PyramidFrame to PyramidImageFrame
        ConvertPym2Msg(pym_image, pym_frame);
      }
    }
  }

  // 构建input
  if (pym_frame == nullptr) {
    std::cout << "get pyramid frame failed." << std::endl;
    return -1;
  }

  // roi
  {
    auto rois = std::make_shared<xstream::BaseDataVector>();
    auto roi_0 = std::make_shared<xstream::BBox>(271, 106, 558, 420);
    auto roi_1 = std::make_shared<xstream::BBox>(829, 45, 1114, 404);
    auto roi_2 = std::make_shared<xstream::BBox>(1346, 134, 1665, 439);
    rois->datas_.push_back(roi_0);
    rois->datas_.push_back(roi_1);
    rois->datas_.push_back(roi_2);
    input.push_back(rois);
  }
  // pym
  input.push_back(pym_frame);

  output = infer_method.DoProcess(input, param);
  inference::PostMethod post_method;
  post_method.DoProcess(output, param);
  infer_method.Finalize();
  post_method.Finalize();

  // deinit video_source
  {
    if (pym_image != nullptr) {
      ret = video_source->FreePyramidFrame(pym_image);
      if (ret) {
        std::cout << "free pyramid frame failed, ret: " << ret << std::endl;
      }
    }
    std::cout << "\n\nvideo source sample quit" << std::endl;
    ret = video_source->Stop();
    if (ret) {
      std::cout << "video source stop failed, ret: " << ret << std::endl;
      return ret;
    }
    ret = video_source->DeInit();
    if (ret) {
      std::cout << "video source deinit failed, ret: " << ret << std::endl;
      return ret;
    }
  }
  return 0;
}
