/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of faceid_preprocess
 * @file   faceid_preprocess.cc
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.05.19
 */

#include "model_inference/preprocess/faceid_preprocess.h"
#include "model_inference/inferencer.h"
#include "image_utils.h"

namespace inference {

int FaceIDPreProcess::Execute(
    const std::vector<xstream::BaseDataPtr> &input,
    std::vector<std::shared_ptr<InferenceEngineTask>>& tasks) {
  LOGD << "FaceIDPreProcess Execute";
  HOBOT_CHECK(input.size() == 1);  // snap_list

  // BaseDataVector<BaseDataVector<shared_ptr<XStreamData<shared_ptr<snap>>>>>
  auto snaps = dynamic_cast<xstream::BaseDataVector*>(input[0].get());
  HOBOT_CHECK(snaps);
  int person_num = snaps->datas_.size();

  // A person may has more than one snapshot
  int total_snap = 0;

  for (int obj_idx = 0, effective_idx = 0;
       effective_idx < person_num && obj_idx < person_num; obj_idx++) {
    auto one_person_snaps =
        dynamic_cast<xstream::BaseDataVector*>(snaps->datas_[obj_idx].get());
    if (!one_person_snaps) {
      continue;
    }
    effective_idx++;
    total_snap += one_person_snaps->datas_.size();
  }
  // task数量:total_snap
  tasks.resize(total_snap);

  for (int32_t obj_idx = 0, snap_idx = 0;
       obj_idx < person_num && snap_idx < total_snap;
       obj_idx++) {
    auto one_person_snaps =
        dynamic_cast<xstream::BaseDataVector *>(snaps->datas_[obj_idx].get());
    if (!one_person_snaps) {
      continue;
    }
    auto snap_num = one_person_snaps->datas_.size();

    for (uint32_t s_idx = 0; s_idx < snap_num; s_idx++) {
      auto tensor_task = std::make_shared<TensorInferenceEngineTask>();
      tasks[snap_idx] = tensor_task;

      auto snap = std::static_pointer_cast<xstream::SnapshotInfo_<
          xstream::BaseDataPtr>>(one_person_snaps->datas_[s_idx]);
      auto lmk = std::static_pointer_cast<xstream::Landmarks>(
          snap->userdata_[1]);
      // check lmk's status
      if (lmk->values_.size() == 0) {
        LOGD << "invalid lmk";
        snap_idx++;
        continue;
      }

      auto snap_mat =
          std::static_pointer_cast<xstream::RawDataImageFrame>(snap->snap_);
      xstream::FloatPoints points = snap->PointsToSnap(*lmk);
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

      cv::Mat snap_bgr;
      cv::Mat nv12((snap_mat->Height() * 3) >> 1, snap_mat->Width(),
                   CV_8UC1, snap_mat->data_);
      cv::cvtColor(nv12, snap_bgr, CV_YUV2BGR_NV12);

      int h_idx, w_idx, c_idx;
      InferenceEngine::GetInstance()->GetHWCIndex(
          infer_->input_model_info_[0].tensor_type,
          infer_->input_model_info_[0].tensor_layout,
          &h_idx, &w_idx, &c_idx);
      int height =
          infer_->input_model_info_[0].valid_shape.dimension_size[h_idx];
      int width =
          infer_->input_model_info_[0].valid_shape.dimension_size[w_idx];
      LOGD << "input w h:" << width << " " << height;
      cv::Mat face_patch_bgr(height, width, CV_8UC3);
      if (!AlignFace(face_lmks, snap_bgr, face_patch_bgr, 0,
                     lmk_template_)) {
        LOGD << "align face failed";
        snap_idx++;
        continue;
      }

      int img_len = height * width * 3 / 2;
      uint8_t *output_data = nullptr;
      int output_size, output_1_stride, output_2_stride;
      int ret = 0;
      cv::cvtColor(face_patch_bgr, face_patch_bgr, CV_BGR2YUV_I420);
      ret = HobotXStreamConvertImage(
          face_patch_bgr.data,
          img_len, width, height, width, width / 2,
          IMAGE_TOOLS_RAW_YUV_I420, IMAGE_TOOLS_RAW_YUV_NV12,
          &output_data, &output_size, &output_1_stride, &output_2_stride);
      if (ret != 0) {
        LOGE << "convert img failed";
        snap_idx++;
        continue;
      }
      // prepare tensor properties
      InferenceEngine::GetInstance()->PrepareModelTensorProperties(
          infer_->input_model_info_, tensor_task->input_tensors_);
      InferenceEngine::GetInstance()->PrepareModelTensorProperties(
          infer_->output_model_info_, tensor_task->output_tensors_);
      // alloc Tensor
      InferenceEngine::GetInstance()->
          AllocModelTensor(tensor_task->input_tensors_);
      InferenceEngine::GetInstance()->
          AllocModelTensor(tensor_task->output_tensors_);

      HOBOT_CHECK(tensor_task->input_tensors_.size() == 1);
      auto &input_tensor = tensor_task->input_tensors_[0];
      // copy data to input tensor
      if (input_tensor.properties.tensor_type == IMG_TYPE_NV12) {
        HOBOT_CHECK(input_tensor.sys_mem[0].mem_size == output_size);
        memcpy(input_tensor.sys_mem[0].vir_addr, output_data,
               input_tensor.sys_mem[0].mem_size);
        // 不需flush，run model前进行flush
      } else if (input_tensor.properties.tensor_type ==
                 IMG_TYPE_NV12_SEPARATE) {
        HOBOT_CHECK(input_tensor.sys_mem[0].mem_size == height * width);
        HOBOT_CHECK(input_tensor.sys_mem[1].mem_size ==
                    height * width >> 1);
        memcpy(input_tensor.sys_mem[0].vir_addr, output_data,
               input_tensor.sys_mem[0].mem_size);
        memcpy(input_tensor.sys_mem[1].vir_addr,
               output_data + input_tensor.sys_mem[0].mem_size,
               input_tensor.sys_mem[1].mem_size);
        // 不需flush，run model前进行flush
      } else {
        HOBOT_CHECK(0) << "not support tensor_type: "
            << input_tensor.properties.tensor_type;
      }
      // 释放output_data
      HobotXStreamFreeImage(output_data);
      snap_idx++;
    }
  }

  return 0;
}


int FaceIDPreProcess::AlignFace(
    const std::vector<float> &lmks_pts, const cv::Mat &origImg,
    cv::Mat &dstImg, float fill_value,
    std::vector<float> coord5points) {
  std::vector<float> trans_pts;
  std::vector<float> src;
  std::vector<float> dst;
  assert(lmks_pts.size() == coord5points.size());
  dst.resize(lmks_pts.size());
  src.resize(lmks_pts.size());
  for (size_t i = 0; i < lmks_pts.size(); ++i) {
    dst[i] = coord5points[i];
    src[i] = lmks_pts[i];
  }
  cv::Mat trans(3, 2, CV_32F);
  int ret = Cp2tform(src, dst, trans);
  if (ret == 0) return 0;
  int channel = dstImg.channels();
  float value = fill_value;
  if (channel == 3) {
    cv::warpAffine(origImg,
                   dstImg,
                   trans,
                   cv::Size(dstImg.cols, dstImg.rows),
                   cv::INTER_LINEAR,
                   cv::BORDER_CONSTANT,
                   cv::Scalar(value, value, value));
  } else {
    cv::warpAffine(origImg,
                   dstImg,
                   trans,
                   cv::Size(dstImg.cols, dstImg.rows),
                   cv::INTER_LINEAR,
                   cv::BORDER_CONSTANT,
                   cv::Scalar(value));
  }
  GetAffinePoints(src, trans, trans_pts);
  return 1;
}

int FaceIDPreProcess::Cp2tform(
    std::vector<float> &uv, std::vector<float> &xy, cv::Mat &trans) {
  assert(uv.size() == xy.size() && uv.size() >= 3 * 2);
  cv::Mat T(3, 3, CV_32F), Tinv(3, 3, CV_32F);
  int ret = FindNonReflectiveSimilarity(uv, xy, T, Tinv);
  if (ret == 0) return 0;
  // cv::Mat trans(3, 2, CV_32F);
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 2; col++) {
      trans.at<float>(row, col) = T.at<float>(row, col);
    }
  }
  cv::transpose(trans, trans);
  return 1;
}

int FaceIDPreProcess::FindNonReflectiveSimilarity(
    std::vector<float> &uv, std::vector<float> &xy,
    cv::Mat &T, cv::Mat &Tinv) {
  assert(uv.size() == xy.size());
  assert(uv.size() % 2 == 0);
  assert(uv.size() / 2 >= 3);
  int nPtNum = uv.size() / 2;
  cv::Mat X(nPtNum * 2, 4, CV_32F);
  for (int row = 0; row < nPtNum * 2; row++) {
    for (int col = 0; col < 4; col++) {
      if (row <= nPtNum - 1) {
        if (col <= 1) {
          X.at<float>(row, col) = xy[row * 2 + col];
        } else {
          X.at<float>(row, col) = 1 * (col - 1) % 2;
        }
      } else {
        if (col <= 1) {
          X.at<float>(row, col) =
              xy[(row - nPtNum) * 2 + 1 - col] * (1 - 2 * col);
        } else {
          X.at<float>(row, col) = 1 * (col % 2);
        }
      }
    }
  }
  int size = nPtNum * 2;
  // cv::Mat U(nPtNum * 2, CV_32F);
  cv::Mat U(1, &size, CV_32F);
  for (int row = 0; row < 2 * nPtNum; row++) {
    if (row < nPtNum) {
      U.at<float>(row) = uv[2 * row];
    } else {
      U.at<float>(row) = uv[2 * (row - nPtNum) + 1];
    }
  }

  // Least Squares Solution
  cv::Mat X_trans;
  cv::transpose(X, X_trans);
  cv::Mat X_trans_X = X_trans * X;
  cv::Mat inv_mat;
  double ret = cv::invert(X_trans_X, inv_mat);
  if (ret == 0.0) {
    return 0;
  } else {
    inv_mat = inv_mat * X_trans;
    inv_mat = inv_mat * U;
    cv::Mat &r = inv_mat;
    float sc = r.at<float>(0, 0);
    float ss = r.at<float>(1, 0);
    float tx = r.at<float>(2, 0);
    float ty = r.at<float>(3, 0);
    float trans_final[] = {sc, -1 * ss, 0, ss, sc, 0, tx, ty, 1};
    // cv::Mat Tinv(3, 3, CV_32F, trans_final);
    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 3; col++) {
        Tinv.at<float>(row, col) = trans_final[row * 3 + col];
      }
    }

    double ret = cv::invert(Tinv, T);
    HOBOT_CHECK(ret != 0.0);
    T.at<float>(0, 2) = 0.0;
    T.at<float>(1, 2) = 0.0;
    T.at<float>(2, 2) = 1.0;
    return 1;
  }
}

int FaceIDPreProcess::GetAffinePoints(
    std::vector<float> &pts_in, cv::Mat &trans,
    std::vector<float> &pts_out) {
  pts_out.resize(pts_in.size());
  for (size_t i = 0; i < pts_in.size(); i += 2) {
    pts_out[i] = pts_in[i] * trans.at<float>(0, 0)
                 + pts_in[i + 1] * trans.at<float>(0, 1)
                 + trans.at<float>(0, 2);
    pts_out[i + 1] = pts_in[i] * trans.at<float>(1, 0)
                     + pts_in[i + 1] * trans.at<float>(1, 1)
                     + trans.at<float>(1, 2);
  }
  return 1;
}

}  // namespace inference
