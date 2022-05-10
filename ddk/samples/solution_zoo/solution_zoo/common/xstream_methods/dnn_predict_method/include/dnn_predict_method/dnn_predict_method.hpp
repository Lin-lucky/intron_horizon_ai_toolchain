/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: DnnPredictMethod.h
 * @Brief: declaration of the DnnPredictMethod
 * @Author: zhe.sun
 * @Email: zhe.sun@horizon.ai
 * @Date: 2020-11-23 11:09:18
 * @Last Modified by: zhe.sun
 * @Last Modified time: 2020-11-23 16:01:33
 */

#ifndef INCLUDE_DNNPREDICTMETHOD_DNNPREDICTMETHOD_H_
#define INCLUDE_DNNPREDICTMETHOD_DNNPREDICTMETHOD_H_

#include <mutex>
#include <string>
#include <fstream>
#include <vector>
#include <memory>
#include "json/json.h"
#include "xstream/vision_type.h"
#include "xstream/simple_method.h"
#include "bpu_predict_extension.h"
#include "dnn_async_data.h"
#include "xstream/profiler.h"
#include "hobotlog/hobotlog.hpp"

namespace xstream {

#define ALIGN_16(v) ((v + (16 - 1)) / 16 * 16)

class DnnPredictMethod : public SimpleMethod {
 public:
  DnnPredictMethod() {}
  virtual ~DnnPredictMethod() {}

  int Init(const std::string &cfg_path) override {
    LOGI << "DnnPredictMethod Init";
    // 0. parse config_
    std::ifstream infile(cfg_path.c_str());
    HOBOT_CHECK(infile.good())
        << "DnnPredictMethod error config file path: " << cfg_path;
    infile >> config_;

    auto get_parent_path = [](const std::string path) -> std::string {
      auto pos = path.rfind('/');
      if (std::string::npos != pos) {
        auto parent = path.substr(0, pos);
        return parent + "/";
      } else {
        return std::string("./");
      }
    };

    // 1. model_path_
    std::string model_path = config_["model_file_path"].isString()
                                 ? config_["model_file_path"].asString()
                                 : "";
    HOBOT_CHECK(model_path.size() > 0) << "must set model_file_path";
    std::string parent_path = get_parent_path(cfg_path);
    model_path_ = parent_path + model_path;
    config_["model_file_path"] = model_path_;

    // dnn_is_sync_, dnn_run_with_roi_
    dnn_is_sync_ = config_["dnn_is_sync"].isBool()
                       ? config_["dnn_is_sync"].asBool()
                       : dnn_is_sync_;
    dnn_run_with_roi_ = config_["dnn_run_with_roi"].isBool()
                            ? config_["dnn_run_with_roi"].asBool()
                            : dnn_run_with_roi_;
    dnn_model_group_ = config_["dnn_model_group"].isBool()
                           ? config_["dnn_model_group"].asBool()
                           : dnn_model_group_;
    dnn_model_group_id_ = config_["dnn_model_group_id"].isInt()
                              ? config_["dnn_model_group_id"].asInt()
                              : 0;
    int core_id =
        config_["core_id"].isInt() ? config_["core_id"].asInt() : 0;  // 默认为0
    HOBOT_CHECK(core_id <= 2 && core_id >= 0) << "core_id out of range";
    dnn_ctrl_.core_id = core_id;  // 注意：异步预测模式下指定core_id无效

    // resize_type = 1
    // 结合HB_BPU_runModel接口：支持输入数据缩放到模型输入大小
    // 结合HB_BPU_runModelWithBbox接口：支持软件resize roi
    int resize_type = config_["resize_type"].isInt()
                          ? config_["resize_type"].asInt()
                          : 0;  // 默认为0
    dnn_ctrl_.resize_type = resize_type;

    src_image_width_ = config_["src_image_witdh"].isInt()
                           ? config_["src_image_witdh"].asInt()
                           : 0;
    src_image_height_ = config_["src_image_height"].isInt()
                            ? config_["src_image_height"].asInt()
                            : 0;

    int ret;
    // 2. load model
    dnn_model_ = std::make_shared<BPUModelWrapper>();
    ret = HB_BPU_loadModelFromFile(model_path_.c_str(), &dnn_model_->bpu_model);
    HOBOT_CHECK(ret == 0) << "load model failed: " << HB_BPU_getErrorName(ret);

    if (dnn_model_group_) {
      ret = HB_BPU_setModelGroup(&(dnn_model_->bpu_model), dnn_model_group_id_);
      HOBOT_CHECK(ret == 0)
          << "set model group failed: " << HB_BPU_getErrorName(ret);
    }
    // 3. 获取模型输入大小
    HOBOT_CHECK(dnn_model_->bpu_model.input_num >= 1);
    ret = HB_BPU_getHW(dnn_model_->bpu_model.inputs[0].data_type,
                       &dnn_model_->bpu_model.inputs[0].shape,
                       &model_input_height_, &model_input_width_);
    HOBOT_CHECK(ret == 0) << "load model input size failed: "
                          << HB_BPU_getErrorName(ret);

    // 4. 获取模型输入hwc索引
    ret = HB_BPU_getHWCIndex(dnn_model_->bpu_model.inputs[0].data_type,
                             &dnn_model_->bpu_model.inputs[0].shape.layout,
                             &input_h_idx_, &input_w_idx_, &input_c_idx_);
    HOBOT_CHECK(ret == 0) << "load model input index failed: "
                          << HB_BPU_getErrorName(ret);

    return 0;
  }

  void Finalize() override { dnn_model_ = nullptr; }

  // 主逻辑，完全复用，派生类不需要再实现DoProcess
  std::vector<BaseDataPtr> DoProcess(
      const std::vector<BaseDataPtr> &input,
      const xstream::InputParamPtr &param) override {
    LOGD << "DnnPredictMethod DoProcess";
    HOBOT_CHECK(!input.empty());
    RUN_PROCESS_TIME_PROFILER("DnnPredictMethod_DoProcess");
    std::vector<BaseDataPtr> output;
    {
      // 创建BpuAsyncData，用于传递给DnnPostProcessMethod，或者同步预测结果解析使用
      auto dnn_result = std::make_shared<DnnAsyncData>();
      output.resize(1);
      output[0] = dnn_result;

      // bpu 任务句柄
      std::vector<BPU_TASK_HANDLE> task_handle;

      // 本次预测任务返回码，若添加任务失败，则DnnAsyncData中不需传递相关变量
      int ret = 0;
      if (dnn_run_with_roi_) {
        RUN_PROCESS_TIME_PROFILER("Run_Model_ROI");
        task_handle.resize(1);
        // 输入一个金字塔， 加上一系列ROI
        // 预测库bpu-predict内部根据金字塔每层大小以及ROI，自动完成抠图，缩放到模型输入大小
        // ROI输入方式，调用HB_BPU_runModelWithBbox进行预测，需要准备金字塔与ROI
        // X2和X3版本的金字塔数据结构不同, 需要根据平台做不同处理
        PyramidImageFrame pyramid;  // get from input
        std::vector<BPU_BBOX> input_box;
        std::vector<int> valid_box;  // 大小和Method输入的检测框一样，框是否有效
        std::vector<BPU_TENSOR_S> output_tensor;
        {
          RUN_PROCESS_TIME_PROFILER("DnnPredictMethod_PrepareInputData");
          // 调用派生类实现的预处理部分
          ret = PrepareInputData(input, param, pyramid, input_box, valid_box,
                                 output_tensor);
        }
        if (ret != 0) {
          return output;
        }

        int resizable_cnt = 0;
        // 调用BPU-Predict接口完成预测
        ret = HB_BPU_runModelWithBbox(
            &dnn_model_->bpu_model,
            reinterpret_cast<BPU_ADDR_INFO_S *>(&pyramid.img_.down_scale[0]),
            pyramid.img_.ds_pym_layer, input_box.data(), input_box.size(),
            output_tensor.data(), output_tensor.size(), &dnn_ctrl_,
            dnn_is_sync_, &resizable_cnt, &task_handle[0]);
        if (ret != 0) {
          LOGE << "HB_BPU_runModel failed: " << HB_BPU_getErrorName(ret);
          // 释放output_tensor,task_handle
          FreeTensor(output_tensor);
          HB_BPU_releaseTask(&task_handle[0]);
          task_handle[0] = nullptr;
          return output;
        }
        // 这部分逻辑，解决pyramid选层失败，导致roi未送入模型，避免出现bpu结果与输入roi错位的问题
        LOGI << "resizable count: " << resizable_cnt;
        for (size_t i = 0, bpu_box_idx = 0; i < valid_box.size(); i++) {
          if (valid_box[i]) {
            valid_box[i] = input_box[bpu_box_idx].resizable;
            bpu_box_idx++;
          }
        }
        // 赋值DnnAsyncData,roi方式不需要input_tensors
        dnn_result->dnn_model = dnn_model_;
        dnn_result->output_tensors.push_back(output_tensor);
        dnn_result->task_handle = task_handle;
        dnn_result->valid_box = valid_box;
        dnn_result->dnn_input_box = input_box;
        dnn_result->dnn_is_sync = dnn_is_sync_;
      } else {
        RUN_PROCESS_TIME_PROFILER("Run_Model");
        // Tensor输入方式，调用HB_BPU_runModel完成预测，需要创建输入与输出tensor
        std::vector<std::vector<BPU_TENSOR_S>> input_tensor;
        std::vector<std::vector<BPU_TENSOR_S>> output_tensor;
        // 调用派生类实现的预处理部分  TODO
        int rv = GetSrcImageSize(input, dnn_result->src_image_height,
                                 dnn_result->src_image_width);
        if (rv != 0) {
          LOGE << "Error getting src image size";
        }
        {
          RUN_PROCESS_TIME_PROFILER("DnnPredictMethod_PrepareInputData");
          ret = PrepareInputData(input, param, input_tensor, output_tensor);
        }
        if (ret != 0) {
          return output;
        }
        task_handle.resize(input_tensor.size());
        for (size_t i = 0; i < input_tensor.size(); ++i) {
          // 申请input_tensor或output_tensor失败
          if (input_tensor[i].size() == 0 || output_tensor[i].size() == 0) {
            continue;
          }
          // 调用bpu-predict接口完成预测
          ret = HB_BPU_runModel(&dnn_model_->bpu_model, input_tensor[i].data(),
                                input_tensor[i].size(), output_tensor[i].data(),
                                output_tensor[i].size(), &dnn_ctrl_,
                                dnn_is_sync_, &task_handle[i]);
          if (ret != 0) {
            LOGE << "HB_BPU_runModel failed: " << HB_BPU_getErrorName(ret);
            // 释放input_tensor,output_tensor,task_handle
            // DnnPostProcessMethod中可通过这些字段判断是否需要后处理解析
            FreeTensor(input_tensor[i]);
            FreeTensor(output_tensor[i]);
            HB_BPU_releaseTask(&task_handle[i]);
            task_handle[i] = nullptr;
          }
        }

        // 赋值BpuAsyncData
        dnn_result->dnn_model = dnn_model_;
        dnn_result->input_tensors = input_tensor;
        dnn_result->output_tensors = output_tensor;
        dnn_result->task_handle = task_handle;
        dnn_result->dnn_is_sync = dnn_is_sync_;
      }
    }
    LOGD << "DnnPredictMethod DoProcess Success";
    return output;
  }

 public:
  Json::Value config_;
  std::string model_path_;

  std::shared_ptr<BPUModelWrapper> dnn_model_;
  bool dnn_is_sync_ = false;       // 默认异步,可配置
  bool dnn_run_with_roi_ = false;  // 默认非roi输入,可配置
  bool dnn_model_group_ = false;   // 是否开启group模式
  int dnn_model_group_id_ = 0;     // group模式，该模型的group id
  BPU_RUN_CTRL_S dnn_ctrl_;        // 运行的控制信息,core_id等
  int src_image_width_ = -1;
  int src_image_height_ = -1;

  int input_h_idx_, input_w_idx_, input_c_idx_;
  int model_input_height_, model_input_width_;

  // 申请InputTensor大小
  int AllocInputTensor(std::vector<BPU_TENSOR_S> &input_tensors) {
    input_tensors.resize(dnn_model_->bpu_model.input_num);
    for (int i = 0; i < dnn_model_->bpu_model.input_num; i++) {
      BPU_TENSOR_S &tensor = input_tensors[i];
      BPU_MODEL_NODE_S &node = dnn_model_->bpu_model.inputs[i];
      tensor.data_type = node.data_type;
      if (tensor.data_type == BPU_TYPE_IMG_YUV_NV12) {
        tensor.data_type = BPU_TYPE_IMG_NV12_SEPARATE;
      }
      tensor.data_shape.layout = node.shape.layout;
      tensor.aligned_shape.layout = node.shape.layout;

      tensor.data_shape.ndim = 4;
      tensor.data_shape.d[0] = 1;
      tensor.data_shape.d[1] = node.shape.d[1];
      tensor.data_shape.d[2] = node.shape.d[2];
      tensor.data_shape.d[3] = node.shape.d[3];
      tensor.aligned_shape.ndim = 4;
      tensor.aligned_shape.d[0] = 1;
      tensor.aligned_shape.d[1] = node.aligned_shape.d[1];
      tensor.aligned_shape.d[2] = node.aligned_shape.d[2];
      tensor.aligned_shape.d[3] = node.aligned_shape.d[3];
      LOGD << "input_tensor.data_shape.d[0]: " << tensor.data_shape.d[0] << ", "
           << "input_tensor.data_shape.d[1]: " << tensor.data_shape.d[1] << ", "
           << "input_tensor.data_shape.d[2]: " << tensor.data_shape.d[2] << ", "
           << "input_tensor.data_shape.d[3]: " << tensor.data_shape.d[3] << ", "
           << "input_tensor.data_shape.layout: " << tensor.data_shape.layout;

      int h_ix, w_ix, c_ix;
      HB_BPU_getHWCIndex(tensor.data_type, &tensor.data_shape.layout, &h_ix,
                         &w_ix, &c_ix);
      int input_height = tensor.data_shape.d[h_ix];
      int input_width = tensor.data_shape.d[w_ix];
      int input_channel = tensor.data_shape.d[c_ix];
      LOGD << "input_height: " << input_height << ", "
           << "input_width: " << input_width << ", "
           << "input_channel: " << input_channel;

      int ret = 0;
      switch (tensor.data_type) {
        case BPU_TYPE_IMG_NV12_SEPARATE: {
          int stride = tensor.aligned_shape.d[input_w_idx_];
          int y_length = input_height * stride;
          int uv_length = input_height / 2 * stride;
          ret = HB_SYS_bpuMemAlloc("in_data0", y_length, true, &tensor.data);
          if (ret != 0) {
            LOGE << "bpu alloc mem failed: " << HB_BPU_getErrorName(ret);
            break;
          }
          ret =
              HB_SYS_bpuMemAlloc("in_data1", uv_length, true, &tensor.data_ext);
          if (ret != 0) {
            LOGE << "bpu alloc mem failed: " << HB_BPU_getErrorName(ret);
            // release alloced mem
            HB_SYS_bpuMemFree(&tensor.data);
            break;
          }
          break;
        }
        case BPU_TYPE_TENSOR_U8:
        case BPU_TYPE_TENSOR_S8: {
          int width_stride = tensor.aligned_shape.d[input_w_idx_];
          int channel_stride = tensor.aligned_shape.d[input_c_idx_];
          int length = input_height * width_stride * channel_stride;
          ret = HB_SYS_bpuMemAlloc("in_data0", length, true, &tensor.data);
          break;
        }
        case BPU_TYPE_TENSOR_F32:
        case BPU_TYPE_TENSOR_S32:
        case BPU_TYPE_TENSOR_U32: {
          int width_stride = tensor.aligned_shape.d[input_w_idx_];
          int channel_stride = tensor.aligned_shape.d[input_c_idx_];
          int length = input_height * width_stride * channel_stride;
          ret = HB_SYS_bpuMemAlloc("in_data0", length * 4, true, &tensor.data);
          break;
        }
        case BPU_TYPE_IMG_Y: {
          int stride = tensor.aligned_shape.d[input_w_idx_];
          int image_length = input_height * stride;
          ret =
              HB_SYS_bpuMemAlloc("in_data0", image_length, true, &tensor.data);

          tensor.data_shape.d[input_c_idx_] = 1;
          tensor.aligned_shape.d[input_c_idx_] = 1;
          break;
        }
        case BPU_TYPE_IMG_YUV444:
        case BPU_TYPE_IMG_BGR:
        case BPU_TYPE_IMG_RGB:
        case BPU_TYPE_IMG_BGRP:
        case BPU_TYPE_IMG_RGBP: {
          int image_length = input_height * input_width * 3;
          ret =
              HB_SYS_bpuMemAlloc("in_data0", image_length, true, &tensor.data);
          break;
        }
        default:
          HOBOT_CHECK(0) << "unsupport data_type: " << tensor.data_type;
          break;
      }
      if (ret != 0) {
        LOGE << "bpu alloc mem failed: " << HB_BPU_getErrorName(ret);
        input_tensors.clear();
        return ret;
      }
    }
    return 0;
  }
  // 申请OutputTensor大小
  int AllocOutputTensor(std::vector<BPU_TENSOR_S> &output_tensors) {
    output_tensors.resize(dnn_model_->bpu_model.output_num);
    for (int i = 0; i < dnn_model_->bpu_model.output_num; i++) {
      BPU_TENSOR_S &tensor = output_tensors[i];
      BPU_MODEL_NODE_S &node = dnn_model_->bpu_model.outputs[i];
      tensor.data_type = node.data_type;
      tensor.data_shape = node.shape;
      tensor.aligned_shape = node.aligned_shape;
      int output_size = 1;
      for (int j = 0; j < node.aligned_shape.ndim; j++) {
        output_size *= node.aligned_shape.d[j];
        LOGD << "node.aligned_shape.d[j]:" << node.aligned_shape.d[j];
      }
      if (node.data_type == BPU_TYPE_TENSOR_F32 ||
          node.data_type == BPU_TYPE_TENSOR_S32 ||
          node.data_type == BPU_TYPE_TENSOR_U32) {
        output_size *= 4;
      }
      output_size = ALIGN_16(output_size);
      LOGD << "output_size: " << output_size;
      int ret =
          HB_SYS_bpuMemAlloc("out_data0", output_size, true, &tensor.data);
      if (ret != 0) {
        LOGE << "bpu alloc mem failed: " << HB_BPU_getErrorName(ret);
        // release alloced mem
        FreeTensor(output_tensors);
        return ret;
      }
    }
    return 0;
  }
  int AllocOutputTensor(std::vector<BPU_TENSOR_S> &output_tensors, int num) {
    output_tensors.resize(dnn_model_->bpu_model.output_num);
    for (int i = 0; i < dnn_model_->bpu_model.output_num; i++) {
      BPU_TENSOR_S &tensor = output_tensors[i];
      BPU_MODEL_NODE_S &node = dnn_model_->bpu_model.outputs[i];
      tensor.data_type = node.data_type;
      tensor.data_shape = node.shape;
      tensor.aligned_shape = node.aligned_shape;
      int output_size = 1;
      for (int j = 0; j < node.aligned_shape.ndim; j++) {
        output_size *= node.aligned_shape.d[j];
        LOGD << "node.aligned_shape.d[j]:" << node.aligned_shape.d[j];
      }
      if (node.data_type == BPU_TYPE_TENSOR_F32 ||
          node.data_type == BPU_TYPE_TENSOR_S32 ||
          node.data_type == BPU_TYPE_TENSOR_U32) {
        output_size *= 4;
      }
      output_size = ALIGN_16(output_size);
      LOGD << "output_size: " << output_size;
      int ret = HB_SYS_bpuMemAlloc("out_data0", output_size * num, true,
                                   &tensor.data);
      if (ret != 0) {
        LOGE << "bpu alloc mem failed: " << HB_BPU_getErrorName(ret);
        // release alloced mem
        FreeTensor(output_tensors);
        return ret;
      }
    }
    return 0;
  }

  // 释放InputTensor/OutputTensor
  void FreeTensor(std::vector<BPU_TENSOR_S> &tensors) {
    for (size_t i = 0; i < tensors.size(); i++) {
      BPU_TENSOR_S &tensor = tensors[i];
      switch (tensor.data_type) {
        case BPU_TYPE_IMG_Y:
        case BPU_TYPE_IMG_YUV_NV12:
        case BPU_TYPE_IMG_YUV444:
        case BPU_TYPE_IMG_RGB:
        case BPU_TYPE_IMG_BGR:
        case BPU_TYPE_IMG_BGRP:
        case BPU_TYPE_IMG_RGBP:
        case BPU_TYPE_TENSOR_U8:
        case BPU_TYPE_TENSOR_S8:
        case BPU_TYPE_TENSOR_F32:
        case BPU_TYPE_TENSOR_S32:
        case BPU_TYPE_TENSOR_U32:
          HB_SYS_bpuMemFree(&tensor.data);
          break;
        case BPU_TYPE_IMG_NV12_SEPARATE:
          HB_SYS_bpuMemFree(&tensor.data);
          HB_SYS_bpuMemFree(&tensor.data_ext);
          break;
        default:
          HOBOT_CHECK(0) << "not support data_type: " << tensor.data_type;
          break;
      }
    }
    tensors.clear();
  }

  // 派生类需要实现
  // PrepareInputData内部需要根据一帧图像目标数量，多次调用AllocInputTensor分配空间
  // 框架不进行输入与输出的Tensor分配
  // IN: input, param; OUT: input_tensors, output_tensors
  // 返回码：0，成功；否则失败；
  virtual int PrepareInputData(
      const std::vector<BaseDataPtr> &input, const xstream::InputParamPtr param,
      std::vector<std::vector<BPU_TENSOR_S>> &input_tensors,
      std::vector<std::vector<BPU_TENSOR_S>> &output_tensors) {
    return -1;
  }
  // 派生类需要实现
  // 将Method的输入预处理后，拷贝到金字塔以及roi
  // 该模式是特殊用法，只支持对所有的ROI打包一起，调用一次预测接口
  // X2和X3版本的金字塔数据结构不同，
  // 函数内部需要获取PyramidImageFrame pyramid
  // IN: input, param; OUT: pyramid, input_bbox, valid_box, output_tensors
  // 返回码：0，成功；否则失败；若存在申请失败，函数内部还需负责已申请空间的释放
  virtual int PrepareInputData(const std::vector<BaseDataPtr> &input,
                               const xstream::InputParamPtr param,
                               PyramidImageFrame &pyramid,
                               std::vector<BPU_BBOX> &input_bbox,
                               std::vector<int> &valid_box,
                               std::vector<BPU_TENSOR_S> &output_tensors) {
    return -1;
  }

  virtual int GetSrcImageSize(
      const std::vector<BaseDataPtr> &input,
      int &src_image_height,
      int &src_image_width) {
    src_image_height = src_image_height_;
    src_image_width = src_image_width_;
    return 0;
  }
};

}  // namespace xstream
#endif  // INCLUDE_DNNPREDICTMETHOD_DNNPREDICTMETHOD_H_
