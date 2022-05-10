/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: RectInputPredictor.cpp
 * @Brief: definition of the RectInputPredictor
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-07-16 14:27:05
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-07-16 16:23:27
 */

#include "cnn_method/predictor/rect_input_predictor.h"
#include <algorithm>
#include <memory>
#include <vector>
#include "hobotlog/hobotlog.hpp"
#include "xstream/profiler.h"
#include "xstream/vision_type.h"

using xstream::BBox;
using xstream::ImageFrame;
using xstream::PyramidImageFrame;
typedef std::shared_ptr<ImageFrame> ImageFramePtr;

namespace xstream {

void RectInputPredictor::Do(CNNMethodRunData *run_data) {
  run_data->real_nhwc = model_info_.real_nhwc_;
  run_data->elem_size = model_info_.elem_size_;
  run_data->all_shift = model_info_.all_shift_;

  {  // one frame
    auto &input_data = (*(run_data->input));

    int ret = 0;
    auto rois = std::static_pointer_cast<BaseDataVector>(input_data[0]);
    auto pyramid = std::static_pointer_cast<PyramidImageFrame>(input_data[1]);

    int32_t box_num = rois->datas_.size();
    run_data->input_dim_size = box_num;
    std::vector<int> valid_box(box_num, 1);
    run_data->mxnet_output.resize(box_num);
    run_data->norm_rois.resize(box_num);

    auto &norm_rois = run_data->norm_rois;

    std::vector<BPU_BBOX> boxes;
    int32_t handle_num =
        max_handle_num_ < 0 ? box_num : std::min(max_handle_num_, box_num);
    for (int32_t roi_idx = 0; roi_idx < box_num; roi_idx++) {
      auto &roi = rois->datas_[roi_idx];
      auto p_roi = std::static_pointer_cast<BBox>(roi);
      auto p_norm_roi = std::make_shared<BBox>();
      norm_rois[roi_idx] = std::static_pointer_cast<BaseData>(p_norm_roi);
      if (p_roi->state_ != xstream::DataState::VALID ||
              NormMethod::BPU_MODEL_NORM_BY_NOTHING == norm_params_.norm_type) {
        *p_norm_roi = *p_roi;
      } else {
        // do norm
        BBox *norm_box = p_norm_roi.get();
        NormalizeRoi(p_roi.get(),
                     norm_box,
                     norm_params_,
                     pyramid->Width(0),
                     pyramid->Height(0));
        LOGD << "norm roi norm_type:"
             << static_cast<int>(norm_params_.norm_type)
             << " expand_scale:" << norm_params_.expand_scale
             << " aspect_ratio:" << norm_params_.aspect_ratio
             << "  from:" << p_roi->x1_ << ", " << p_roi->y1_ << ", "
             << p_roi->x2_ << ", " << p_roi->y2_ << "  to:" << p_norm_roi->x1_
             << ", " << p_norm_roi->y1_ << ", " << p_norm_roi->x2_ << ", "
             << p_norm_roi->y2_;
      }

      if (p_roi->state_ != xstream::DataState::VALID ||
          roi_idx >= handle_num) {
        valid_box[roi_idx] = 0;
      } else {
        boxes.push_back(BPU_BBOX{p_norm_roi->x1_, p_norm_roi->y1_,
                                 p_norm_roi->x2_, p_norm_roi->y2_,
                                 p_norm_roi->score_, 0, true});
        LOGD << "box {" << p_norm_roi->x1_ << "," << p_norm_roi->y1_ << ","
             << p_norm_roi->x2_ << "," << p_norm_roi->y2_ << "}";
      }
    }
    {
      RUN_PROCESS_TIME_PROFILER(model_name_ + "_runmodel")
      RUN_FPS_PROFILER(model_name_ + "_runmodel")
      int resizable_cnt = 0;
      if (boxes.size() <= 0) {
        LOGD << "no box to cnn";
        return;
      }
      ret = RunModelWithBBox(*pyramid, boxes.data(),
                             boxes.size(), &resizable_cnt);
      if (ret == -1) {
        return;
      }
      LOGI << "resizable_cnt: " << resizable_cnt;
      for (int32_t i = 0, bpu_box_idx = 0; i < box_num; i++) {
        if (valid_box[i]) {
          LOGD << "BPU_BBOX " << bpu_box_idx << ", resizable: "
               << boxes[bpu_box_idx].resizable;
          valid_box[i] = boxes[bpu_box_idx].resizable;
          if (valid_box[i]) {
            auto p_norm_roi =
                std::static_pointer_cast<BBox>(norm_rois[i]);
            p_norm_roi->x1_ = boxes[bpu_box_idx].x1;
            p_norm_roi->y1_ = boxes[bpu_box_idx].y1;
            p_norm_roi->x2_ = boxes[bpu_box_idx].x2;
            p_norm_roi->y2_ = boxes[bpu_box_idx].y2;
          }
          bpu_box_idx++;
        }
      }
    }

    {
      RUN_PROCESS_TIME_PROFILER(model_name_ + "_do_hbrt")
      RUN_FPS_PROFILER(model_name_ + "_do_hbrt")
      // change raw data to mxnet layout
      int layer_size = model_info_.output_layer_size_.size();
      for (int32_t i = 0, mxnet_rlt_idx = 0; i < box_num; i++) {
        if (valid_box[i]) {
          auto &mxnet_rlt = run_data->mxnet_output[i];
          mxnet_rlt.resize(layer_size);
          for (int j = 0; j < layer_size; j++) {
            mxnet_rlt[j].resize(model_info_.mxnet_output_layer_size_[j]);
            HB_SYS_flushMemCache(&(output_tensors_[j].data),
                                 HB_SYS_MEM_CACHE_INVALIDATE);
            int output_size = 1;
            for (int dim = 0;
                 dim < bpu_model_->outputs[j].aligned_shape.ndim;
                 dim++) {
              output_size *= bpu_model_->outputs[j].aligned_shape.d[dim];
            }
            if (bpu_model_->outputs[j].data_type == BPU_TYPE_TENSOR_F32 ||
                bpu_model_->outputs[j].data_type == BPU_TYPE_TENSOR_S32 ||
                bpu_model_->outputs[j].data_type == BPU_TYPE_TENSOR_U32) {
              output_size *= 4;
            }

            int raw_idx = mxnet_rlt_idx * output_size;
            ConvertOutputToMXNet(reinterpret_cast<int8_t *>(
                                   output_tensors_[j].data.virAddr)+raw_idx,
                                 mxnet_rlt[j].data(), j);
          }
          mxnet_rlt_idx++;
        }
      }
      // ReleaseOutputTensor();
    }
  }
}
}  // namespace xstream
