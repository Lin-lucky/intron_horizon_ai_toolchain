/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of image_process
 * @file   image_process.cc
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */

#include "model_inference/preprocess/utils/image_process.h"
#include <cstring>
#include "hobotlog/hobotlog.hpp"
#include <libyuv.h>

namespace inference {

// {
//     // 支持none；内置预定义Predefined;用户自定义的class_name
//     "class_name": "pyramid_roi_preprocess",
//     // 输入数据为金字塔，对于run_model模型从指定层来获取金字塔图像
//     "pyramid_layer": 4,
//     "config": {
//         // 内置预定义图像预处理逻辑
//         "image_process_pipeline": [
//             "crop(0, 0, 960, 465)",
//             "pad(960, 960)",
//             "resize(416, 416)"
//         ]
//         // 内置预定义ROI预处理逻辑
//         // 支持的scale缩放类型，待细化，当前仅仅支持等比例缩放到2.2
//         "roi_process_pipeline": [
//             "scale(2.2)"
//         ]
//     }
// }

int ImageProcess::Init(const std::string &json_str) {
  // string转json
  Json::Reader Reader;
  Json::Value json_value;
  Reader.parse(json_str, json_value);

  int pipeline_size = json_value.size();
  for (int i = 0; i < pipeline_size; i++) {
    std::string process_desp = json_value[i].asString();
    // 构建Image_Proc_Type
    // 操作类型
    std::string process = process_desp.substr(0,
        process_desp.find_first_of("("));
    // 操作参数
    std::vector<int> res;
    auto check_str_valid_func = [](std::string str) {
      str.erase(0, str.find_first_not_of(" "));
      str.erase(str.find_last_not_of(" ") + 1);
      LOGD << "check str = \'" << str << "\'";
      if (str.empty()) {
        return false;
      }
      auto it = str.begin();
      while (it != str.end()) {
        if (*it >= '0' && *it <= '9') {
          it++;
          continue;
        } else {
          return false;
        }
      }
      return true;
    };

    int start_idx = process_desp.find_first_of("(");
    int end_idx = process_desp.find_first_of(")");
    if (start_idx > 0 && end_idx > start_idx) {
      std::string params = process_desp.substr(
          start_idx + 1, end_idx - start_idx - 1);
      std::istringstream iss(params);
      for (std::string item; getline(iss, item, ',');) {
        int i_item;
        if (false == check_str_valid_func(item)) {
          LOGF << "Invalid config param: \'" << item << "\'";
          return -1;
        }
        try {
          i_item = std::stoi(item);
        } catch (std::exception& e) {
          LOGF << e.what();
          LOGF << "Invalid param " << res.size() << ": " << item;
          return -1;
        }
        res.push_back(i_item);
      }
    }
    ImageProcType img_process(process, res);
    process_pipeline_.push_back(img_process);
  }
  return 0;
}

int ImageProcess::Crop(
    const uint8_t *input_y, const uint8_t *input_uv,
    const int input_height, const int input_width, const int input_stride,
    const int crop_x1, const int crop_y1, const int crop_x2, const int crop_y2,
    uint8_t **output) {
  if (input_y == nullptr || input_uv == nullptr || output == nullptr ||
      input_height <= 0 || input_width <= 0 ||
      crop_x1 < 0 || crop_y1 < 0 ||
      crop_x2 >= input_width || crop_y2 >= input_height ||
      crop_x2 < crop_x1 || crop_y2 < crop_y1) {
    LOGE << "invalid parameter";
    return -1;
  }
  // AlignEven
  int top_left_x = crop_x1, top_left_y = crop_y1;
  int bottom_right_x = crop_x2, bottom_right_y = crop_y2;
  int output_width = bottom_right_x - top_left_x + 1;
  int output_height = bottom_right_y - top_left_y + 1;
  if (output_width & 1) {
    return -1;
  }
  if (output_height & 1) {
    return -1;
  }

  // alloc
  int output_data_size = output_width * output_height * 3 / 2;
  *output = static_cast<uint8_t *>(
      std::calloc(output_data_size, sizeof(uint8_t)));
  if (nullptr == (*output)) {
    return -1;
  }

  uint8_t *output_y = *output;
  uint8_t *output_uv = output_y + output_width * output_height;
  // crop y
  const uint8_t *input_pos = input_y + top_left_y * input_stride + top_left_x;
  uint8_t *output_pos = output_y;
  for (int y = 0; y < output_height; ++y) {
    memcpy(output_pos, input_pos, output_width);
    output_pos += output_width;
    input_pos += input_stride;
  }
  // copy uv
  input_uv = input_uv + (top_left_y / 2) * input_stride + (top_left_x & (~1));
  for (int y = 0; y < output_height; y = y + 2) {
    memcpy(output_uv, input_uv, output_width);
    output_uv += output_width;
    input_uv += input_stride;
  }
  return 0;
}

// 默认右下角padding 0
int ImageProcess::Pad(
    const uint8_t *input_y, const uint8_t *input_uv,
    const int input_height, const int input_width, const int input_stride,
    const int output_height, const int output_width,
    uint8_t **output) {
  if (output_height < input_height || output_width < input_width ||
      input_y == nullptr || input_uv == nullptr || output == nullptr ||
      input_height <= 0 || input_width <= 0 ||
      (output_height == input_height && output_width == input_width)) {
    return -1;
  }

  int padding_right_width = output_width - input_width;
  int padding_bottom_height = output_height - input_height;
  if (output_height & 1 || input_height & 1 ||
      output_width & 1 || input_width & 1) {
    return -1;
  }
  // alloc
  int output_data_size = output_width * output_height * 3 / 2;
  *output = static_cast<uint8_t *>(
      std::calloc(output_data_size, sizeof(uint8_t)));
  if (nullptr == (*output)) {
    return -1;
  }

  uint8_t *output_y = *output;
  uint8_t *output_uv = output_y + output_width * output_height;
  // 默认padding右下方
  // padding left & right
  for (int h = 0; h < input_height; ++h) {
    // copy origin
    // copy y
    memcpy(output_y, input_y, input_width);
    output_y += input_width;
    input_y += input_stride;
    // copy uv
    if ((h & 1) == 0) {
      memcpy(output_uv, input_uv, input_width);
      output_uv += input_width;
      input_uv += input_stride;
    }

    // padding right
    if (padding_right_width > 0) {
      // padding y
      memset(output_y, 0, padding_right_width);
      output_y += padding_right_width;

      // padding u && v
      if ((h & 1) == 0) {
        memset(output_uv, 128, padding_right_width);
        output_uv += padding_right_width;
      }
    }
  }
  // padding bottom
  for (int h = 0; h < padding_bottom_height; ++h) {
    // padding y
    memset(output_y, 0, output_width);
    output_y += output_width;

    // padding u && v
    if ((h & 1) == 0) {
      memset(output_uv, 128, output_width);
      output_uv += output_width;
    }
  }
  return 0;
}

// 不保持宽高比，直接resize到目标大小
// TODO(zhe.sun) 硬件resize
int ImageProcess::Resize(
    const uint8_t *input_y, const uint8_t *input_uv,
    const int input_height, const int input_width, const int input_stride,
    const int output_height, const int output_width,
    uint8_t **output) {
  if (nullptr == input_y || nullptr == input_uv ||
      input_height <= 0 || input_width <= 0 ||
      output_height <= 0 || output_width <= 0 ||
      (output_height == input_height && output_width == input_width) ||
      nullptr == output) {
    return -1;
  }
  // align even
  if (output_height & 1 || output_width & 1) {
      return -1;
  }
  // alloc
  int output_data_size = output_height * output_width * 3 / 2;
  *output = static_cast<uint8_t *>(
      std::calloc(output_data_size, sizeof(uint8_t)));
  if (nullptr == (*output)) {
    return -1;
  }

  uint8_t *output_y = *output;
  uint8_t *output_uv = output_y + output_width * output_height;
  // split u v
  uint8_t *pre_uv_buf = nullptr;
  int pre_uv_size = input_width * input_height / 2;
  // alloc u v
  pre_uv_buf = static_cast<uint8_t *>(
      std::calloc(pre_uv_size, sizeof(uint8_t)));
  if (nullptr == (pre_uv_buf)) {
    std::free(*output);
    return -1;
  }
  uint32_t u_v_idx = 0;
  uint32_t u_length = pre_uv_size / 2;
  for (int uv_idx = 0; uv_idx < pre_uv_size; uv_idx += 2) {
    pre_uv_buf[u_v_idx] = input_uv[uv_idx];
    pre_uv_buf[u_v_idx + u_length] = input_uv[uv_idx + 1];
    u_v_idx++;
  }

  // alloc resize_uv_size
  uint8_t *resize_uv_buf = nullptr;
  int resize_uv_size = output_width * output_height / 2;
  resize_uv_buf = static_cast<uint8_t *>(
      std::calloc(resize_uv_size, sizeof(uint8_t)));
  if (nullptr == (resize_uv_buf)) {
    std::free(*output);
    std::free(pre_uv_buf);
    return -1;
  }

  // resize y
  libyuv::ScalePlane(input_y, input_width,
                     input_width, input_height,
                     output_y, output_width,
                     output_width,
                     output_height,
                     libyuv::kFilterBox);
  // scale uv
  libyuv::ScalePlane(pre_uv_buf, input_width / 2,
                     input_width / 2, input_height / 2,
                     resize_uv_buf, output_width / 2,
                     output_width / 2,
                     output_height / 2,
                     libyuv::kFilterBox);
  libyuv::ScalePlane(pre_uv_buf + u_length, input_width / 2,
                     input_width / 2, input_height / 2,
                     resize_uv_buf + resize_uv_size / 2,
                     output_width / 2,
                     output_width / 2,
                     output_height / 2,
                     libyuv::kFilterBox);

  // free pre_uv_buf
  Free(pre_uv_buf);

  // copy resize_uv_buf to output
  uint8_t *dst_uv = output_uv;
  uint8_t *src_u = &resize_uv_buf[0];
  uint8_t *src_v = &resize_uv_buf[resize_uv_size / 2];
  int stride_u = output_width / 2,
      stride_v = output_width / 2;
  int halfwidth = output_width >> 1;
  int halfheight = output_height >> 1;
  for (int y = 0; y < halfheight; ++y) {
    uint8_t *next_uv = dst_uv + output_width;
    uint8_t *next_u = src_u + stride_u;
    uint8_t *next_v = src_v + stride_v;
    for (int x = 0; x < halfwidth; ++x) {
      *dst_uv++ = *src_u++;
      *dst_uv++ = *src_v++;
    }
    dst_uv = next_uv;
    src_u = next_u;
    src_v = next_v;
  }

  // free resize_uv_buf
  Free(resize_uv_buf);
  return 0;
}

void ImageProcess::Free(const uint8_t *data) {
  if (data != nullptr) {
    std::free(const_cast<uint8_t *>(data));
  }
}

// 根据process_pipeline_进行处理
int ImageProcess::Execute(
    const uint8_t *input_y, const uint8_t *input_uv,
    const int input_height, const int input_width,
    uint8_t **output, int &output_height, int &output_width) {
  LOGD << "Start ImageProcess Execute...";

  int src_height = input_height, src_width = input_width;
  const uint8_t *src_y = input_y;
  const uint8_t *src_uv = input_uv;
  bool  need_free = false;  // pipe_line失败时是否需要释放前一次的tmp结果

  uint8_t *output_tmp = nullptr;  // 非最后一个pipeline需要
  int ret = 0;

  int pipeline_size = process_pipeline_.size();
  if (pipeline_size == 0) {
    LOGE << "no preprocess pipeline exits";
    return -1;
  }
  for (int i = 0; i < pipeline_size; i++) {
    ImageProcType &process = process_pipeline_[i];
    if (process.process_ == "crop") {
      // check params_
      if (process.params_.size() != 4) {
        LOGE << "crop params invalid";
        if (need_free) {
          Free(src_y);
        }
        return -1;
      }
      int left_top_x = process.params_[0];
      int left_top_y = process.params_[1];
      int right_bottom_x = process.params_[2];
      int right_bottom_y = process.params_[3];
      if (i != pipeline_size-1) {  // 非最后pipeline，使用output_tmp
        ret = Crop(src_y, src_uv, src_height, src_width, src_width,
                   left_top_x, left_top_y, right_bottom_x, right_bottom_y,
                   &output_tmp);
      } else {
        ret = Crop(src_y, src_uv, src_height, src_width, src_width,
                   left_top_x, left_top_y, right_bottom_x, right_bottom_y,
                   output);
      }
      if (need_free) {
          Free(src_y);
      }
      if (ret != 0) {
        LOGE << "crop process failed";
        return -1;
      }
      // 更新output_height,output_width
      output_height = right_bottom_y - left_top_y + 1;
      output_width = right_bottom_x - left_top_x + 1;
      // 准备下个process
      src_height = output_height, src_width = output_width;
      src_y = output_tmp;
      src_uv = output_tmp + src_height * src_width;
      need_free = true;
    } else if (process.process_ == "pad") {
      // check params_
      if (process.params_.size() != 2) {
        LOGE << "pad params invalid";
        if (need_free) {
          Free(src_y);
        }
        return -1;
      }
      output_height = process.params_[0], output_width = process.params_[1];
      if (i != pipeline_size-1) {  // 非最后pipeline，使用output_tmp
        ret = Pad(src_y, src_uv, src_height, src_width, src_width,
                  output_height, output_width, &output_tmp);
      } else {
        ret = Pad(src_y, src_uv, src_height, src_width, src_width,
                  output_height, output_width, output);
      }

      if (need_free) {
          Free(src_y);
      }
      if (ret != 0) {
        LOGE << "pad process failed";
        return -1;
      }
      // 准备下个process
      src_height = output_height, src_width = output_width;
      src_y = output_tmp;
      src_uv = output_tmp + src_height * src_width;
      need_free = true;
    } else if (process.process_ == "resize") {
      // check params_
      if (process.params_.size() != 2) {
        LOGE << "resize params invalid";
        if (need_free) {
          Free(src_y);
        }
        return -1;
      }
      output_height = process.params_[0], output_width = process.params_[1];
      if (i != pipeline_size-1) {  // 非最后pipeline，使用output_tmp
        ret = Resize(src_y, src_uv, src_height, src_width, src_width,
                     output_height, output_width, &output_tmp);
      } else {
        ret = Resize(src_y, src_uv, src_height, src_width, src_width,
                     output_height, output_width, output);
      }

      if (need_free) {
          Free(src_y);
      }
      if (ret != 0) {
        LOGE << "resize process failed";
        return -1;
      }
      // 准备下个process
      src_height = output_height, src_width = output_width;
      src_y = output_tmp;
      src_uv = output_tmp + src_height * src_width;
      need_free = true;
    }
  }
  return 0;
}

}  // namespace inference
