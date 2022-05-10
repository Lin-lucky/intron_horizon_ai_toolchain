/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of image_process
 * @file   image_process.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */
#ifndef IMAGE_PROCESS_H_
#define IMAGE_PROCESS_H_

#include <vector>
#include <string>
#include "json/json.h"
#include "xstream/xstream_world.h"
#include "xstream/vision_type.h"
#include "model_inference/inference_task.h"

namespace inference {

// 内置通用图像预处理 仅处理nv12格式
class ImageProcess {
 public:
  // 加载配置文件信息，格式：
  //  {
  //     "image_process_pipeline": [
  //                 "crop(0, 0, 960, 465)",
  //                 "pad(960, 960)",
  //                 "resize(416，416)"
  //             ],
  //     "roi_process_pipeline": [
  //                 "scale(2.2)"
  //             ]
  //  }
  // json_value是array, 根据配置初始化process_pipeline_
  int Init(const std::string &json_str);

  class ImageProcType {
   public:
    std::string process_;  // crop、pad、resize
    std::vector<int> params_;
    ImageProcType() = default;
    ImageProcType(
        std::string process,
        std::vector<int> params) :
        process_(process), params_(params) {}
  };
  std::vector<ImageProcType> process_pipeline_;  // Init函数加载

  // OUT: output, 需要外部释放
  int Crop(const uint8_t *input_y, const uint8_t *input_uv,
           const int input_height,
           const int input_width,
           const int input_stride,
           const int crop_x1, const int crop_y1,
           const int crop_x2, const int crop_y2,
           uint8_t **output);

  // 默认对右下角pad
  int Pad(const uint8_t *input_y, const uint8_t *input_uv,
          const int input_height,
          const int input_width,
          const int input_stride,
          const int output_height, const int output_width,
          uint8_t **output);

  // 不保持宽高比，直接resize到目标大小
  int Resize(const uint8_t *input_y, const uint8_t *input_uv,
             const int input_height,
             const int input_width,
             const int input_stride,
             const int output_height, const int output_width,
             uint8_t **output);

  // 注意如果配置crop和resize紧邻，则使用硬件方式
  int Execute(const uint8_t *input_y, const uint8_t *input_uv,
               const int input_height, const int input_width,
               uint8_t **output, int &output_height, int &output_width);

  void Free(const uint8_t *data);
};

}  // namespace inference

#endif  // IMAGE_PROCESS_H_
