/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: LmkInputPredictor.cpp
 * @Brief: definition of the LmkInputPredictor
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-07-16 14:27:05
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-07-16 16:23:27
 */

#include <algorithm>
#include <memory>
#include <vector>

#include "cnn_method/predictor/lmk_input_predictor.h"
#include "cnn_method/util/align_face.h"
#include "hobotlog/hobotlog.hpp"
#include "image_utils.h"
#include "opencv2/opencv.hpp"
#include "xstream/profiler.h"
#include "xstream/vision_type.h"

using xstream::FloatPoints;
using xstream::ImageFrame;
using xstream::Landmarks;
using xstream::RawDataImageFrame;
using xstream::SnapshotInfo_;
typedef std::shared_ptr<ImageFrame> ImageFramePtr;

namespace xstream {

void LmkInputPredictor::Do(CNNMethodRunData *run_data) {
  run_data->real_nhwc = model_info_.real_nhwc_;
  run_data->elem_size = model_info_.elem_size_;
  run_data->all_shift = model_info_.all_shift_;

  {  // one frame
    auto &input_data = (*(run_data->input));

    // BaseDataVector<BaseDataVector<shared_ptr<XStreamData<shared_ptr<snap>>>>>
    auto snaps = dynamic_cast<BaseDataVector *>(input_data[0].get());
    HOBOT_CHECK(snaps);
    int person_num = snaps->datas_.size();

    // A person may has more than one snapshot
    int total_snap = 0;
    int handle_num = max_handle_num_ < 0
                              ? person_num
                              : std::min(max_handle_num_, person_num);
    for (int obj_idx = 0, effective_idx = 0;
         effective_idx < handle_num && obj_idx < person_num; obj_idx++) {
      auto one_person_snaps =
          dynamic_cast<BaseDataVector *>(snaps->datas_[obj_idx].get());
      if (!one_person_snaps) {
        continue;
      }
      effective_idx++;
      total_snap += one_person_snaps->datas_.size();
    }
    run_data->mxnet_output.resize(total_snap);
    run_data->input_dim_size = total_snap;

    int layer_size = model_info_.output_layer_size_.size();
    for (int32_t obj_idx = 0, snap_idx = 0;
         obj_idx < person_num && snap_idx < total_snap;
         obj_idx++) {
      auto one_person_snaps =
          dynamic_cast<BaseDataVector *>(snaps->datas_[obj_idx].get());
      if (!one_person_snaps) {
        continue;
      }
      auto snap_num = one_person_snaps->datas_.size();

      for (uint32_t s_idx = 0; s_idx < snap_num; s_idx++) {
        // std::cout << "snap1 = "
        // << one_person_snaps->datas_[s_idx] << std::endl;
        auto snap = std::static_pointer_cast<SnapshotInfo_<BaseDataPtr>>(
            one_person_snaps->datas_[s_idx]);
        auto lmk =
            std::static_pointer_cast<Landmarks>(snap->userdata_[1]);
        // check lmk's status
        if (lmk->values_.size() == 0) {
          LOGD << "invalid lmk";
          snap_idx++;
          continue;
        }

        auto snap_mat =
            std::static_pointer_cast<RawDataImageFrame>(snap->snap_);
        FloatPoints points =
            snap->PointsToSnap(*lmk);
        std::vector<float> face_lmks;
        for (auto &point : points.values_) {
          LOGD << "lmk x:" << point.x_ << ", y:" << point.y_
               << ", width = " << snap_mat->Width()
               << ", height = " << snap_mat->Height();
          HOBOT_CHECK(point.x_ <= snap_mat->Width())
              << "lmk point x large than width, x:" << point.x_;
          HOBOT_CHECK(point.y_ <= snap_mat->Height())
              << "lmk point y large than height, y:" << point.y_;
          face_lmks.push_back(point.x_);
          face_lmks.push_back(point.y_);
        }

        {
          RUN_PROCESS_TIME_PROFILER(model_name_ + "_do_cnn")
          RUN_FPS_PROFILER(model_name_ + "_do_cnn")
          cv::Mat snap_bgr;
          {
            RUN_PROCESS_TIME_PROFILER(model_name_ + "_nv12Tobgr")
            RUN_FPS_PROFILER(model_name_ + "_nv12Tobgr")

            cv::Mat nv12((snap_mat->Height() * 3) >> 1, snap_mat->Width(),
                         CV_8UC1, snap_mat->data_);
            cv::cvtColor(nv12, snap_bgr, CV_YUV2BGR_NV12);
          }
          int height = model_info_.input_nhwc_[1];
          int width = model_info_.input_nhwc_[2];
          LOGD << "input w h:" << width << " " << height;
          cv::Mat face_patch_bgr(height, width, CV_8UC3);
          {
            RUN_PROCESS_TIME_PROFILER(model_name_ + "_alignface")
            RUN_FPS_PROFILER(model_name_ + "_alignface")
            if (!AlignFace(face_lmks, snap_bgr, face_patch_bgr, 0,
                           g_lmk_template)) {
              LOGD << "align face failed";
              snap_idx++;
              continue;
            }
          }
#if 0
          static int snap_idx_dump = 0;
          for (auto &point : points.values) {
            cv::rectangle(snap_bgr,
                          cv::Point(point.x - 2, point.y - 2),
                          cv::Point(point.x + 2, point.y + 2),
                          CV_RGB(255, 0, 0),
                          2);
          }
          cv::imwrite("snap" + std::to_string(snap_idx_dump) + ".jpg",
                      snap_bgr);
          cv::imwrite("align" + std::to_string(snap_idx_dump++) + ".jpg",
                      face_patch_bgr);
#endif

          int img_len = height * width * 3 / 2;
          uint8_t *output_data = nullptr;
          int output_size, output_1_stride, output_2_stride;
          {
            RUN_PROCESS_TIME_PROFILER(model_name_ + "_bgrTonv12")
            RUN_FPS_PROFILER(model_name_ + "_bgrTonv12")
            int ret = 0;
#ifdef USE_BGR2NV12
            ret = HobotXStreamConvertImage(
                face_patch_bgr.data,
                height * width * 3,
                width, height,
                width * 3,
                0,
                IMAGE_TOOLS_RAW_BGR,
                IMAGE_TOOLS_RAW_YUV_NV12,
                &output_data, &output_size,
                &output_1_stride,
                &output_2_stride);
#else
            cv::cvtColor(face_patch_bgr, face_patch_bgr, CV_BGR2YUV_I420);
            ret = HobotXStreamConvertImage(
                face_patch_bgr.data,
                img_len,
                width,
                height,
                width,
                width / 2,
                IMAGE_TOOLS_RAW_YUV_I420,
                IMAGE_TOOLS_RAW_YUV_NV12,
                &output_data,
                &output_size,
                &output_1_stride,
                &output_2_stride);
            HOBOT_CHECK(ret == 0) << "convert img failed";
            LOGD << "convert img success";
#endif
          }
          {
            RUN_PROCESS_TIME_PROFILER(model_name_ + "_runmodel")
            RUN_FPS_PROFILER(model_name_ + "_runmodel")
            if (RunModel(output_data,
                         output_size,
                         model_info_.data_type_) != 0) {
              HobotXStreamFreeImage(output_data);
              snap_idx++;
              continue;
            }
            LOGD << "RunModel success";
          }
          HobotXStreamFreeImage(output_data);
        }
        // change raw data to mxnet layout
        {
          RUN_PROCESS_TIME_PROFILER(model_name_ + "_do_hbrt")
          RUN_FPS_PROFILER(model_name_ + "_do_hbrt")
          auto &one_tgt_mxnet = run_data->mxnet_output[snap_idx];
          one_tgt_mxnet.resize(layer_size);

          for (int j = 0; j < layer_size; j++) {
            one_tgt_mxnet[j].resize(model_info_.mxnet_output_layer_size_[j]);
            HB_SYS_flushMemCache(&(output_tensors_[j].data),
                                 HB_SYS_MEM_CACHE_INVALIDATE);

            ConvertOutputToMXNet(output_tensors_[j].data.virAddr,
                                 one_tgt_mxnet[j].data(), j);
          }
          // release output
          // ReleaseOutputTensor();
          LOGD << "do hbrt success";
        }
        snap_idx++;
      }
    }
  }
}

}  // namespace xstream
