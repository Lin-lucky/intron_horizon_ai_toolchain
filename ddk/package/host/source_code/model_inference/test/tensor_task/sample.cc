/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  sample of Inferencer
 * @file   sample.cc
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */

#include "xstream/xstream_world.h"
#include "model_inference/inference_task.h"
#include "model_inference/inferencer.h"
#include "model_inference/inference_method.h"
#include "hobotlog/hobotlog.hpp"

int main(int argc, char* argv[]) {
  if (argc < 4) {
    std::cout << "exec image_path img_height img_width";
    return -1;
  }
  SetLogLevel(HOBOT_LOG_DEBUG);
  inference::InferenceEngine::SetPredictType(inference::DNN_PREDICT);

  int ret = 0;

  inference::InferMethod infer_method;
  ret = infer_method.Init("./configs/yolov3_config.json");
  if (ret != 0) {
    LOGE << "InferMethod Init failed";
    return -1;
  }

  std::vector<xstream::BaseDataPtr> input;
  std::vector<xstream::BaseDataPtr> output;
  xstream::InputParamPtr param;
  // 构建input
  std::shared_ptr<xstream::RawDataImageFrame> image =
      std::make_shared<xstream::RawDataImageFrame>();
  {
    image->pixel_format_ = xstream::kHorizonVisionPixelFormatRawNV12;
    image->frame_id_ = 0;

    std::ifstream ifs(argv[1],
                      std::ios::in | std::ios::binary);
    if (!ifs) {
      HOBOT_CHECK(0) << "Open image file: " << argv[1] << " failed";
    }
    ifs.seekg(0, std::ios::end);
    int img_length = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    char *img_bin = new char[sizeof(char) * img_length];
    ifs.read(img_bin, img_length);
    ifs.close();

    image->data_ = reinterpret_cast<uint8_t*>(img_bin);
    image->height_ = atoi(argv[2]);
    image->width_ = atoi(argv[3]);
  }
  input.push_back(image);

  output = infer_method.DoProcess(input, param);

  inference::PostMethod post_method;
  post_method.DoProcess(output, param);

  return 0;
}
