/*
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: RectInputPredictMethod.h
 * @Brief: declaration of RectINputPredictMethod
 * @Author: shiyu.fu
 * @Email: shiyu.fu@horizon.ai
 * @Date: Thu Dec 24 2020 10:57:00
 */

#ifndef INCLUDE_RECTINPUTPREDICTMETHOD_RECTINPUTPREDICTMETHOD_H_
#define INCLUDE_RECTINPUTPREDICTMETHOD_RECTINPUTPREDICTMETHOD_H_

#include <string>
#include <vector>
#include "dnn_util.h"
#include "dnn_predict_method/dnn_predict_method.hpp"
#include "bpu_predict_extension.h"
#include "xstream/vision_type.h"

namespace xstream {

class RectInputPredictMethod : public DnnPredictMethod {
 public:
  RectInputPredictMethod() {}
  virtual ~RectInputPredictMethod() {}

  virtual int Init(const std::string &cfg_path);

  // 派生类需要实现
  // 将Method的输入预处理后，拷贝到金字塔以及roi
  // 该模式是特殊用法，只支持对所有的ROI打包一起，调用一次预测接口
  // X2和X3版本的金字塔数据结构不同，
  // 函数内部需要获取PymImageFrame pyramid
  // IN: input, param; OUT: pyramid, input_bbox, valid_box, output_tensors
  // 返回码：0，成功；否则失败；若存在申请失败，函数内部还需负责已申请空间的释放
  virtual int PrepareInputData(const std::vector<BaseDataPtr> &input,
                               const InputParamPtr param,
                               PyramidImageFrame &pyramid,
                               std::vector<BPU_BBOX> &input_bbox,
                               std::vector<int> &valid_box,
                               std::vector<BPU_TENSOR_S> &output_tensors);

 private:
  NormParams norm_params_;
  FilterMode filter_method_;
  std::string input_data_type_;
  uint32_t max_handle_num_;
};
}  // namespace xstream
#endif  // INCLUDE_RECTINPUTPREDICTMETHOD_RECTINPUTPREDICTMETHOD_H_
