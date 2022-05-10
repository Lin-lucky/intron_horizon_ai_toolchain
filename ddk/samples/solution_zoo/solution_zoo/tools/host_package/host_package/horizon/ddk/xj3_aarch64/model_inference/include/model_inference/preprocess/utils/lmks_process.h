/*
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: lmks_process.h
 * @Brief: declaration of LmksProcess
 * @Author: shiyu.fu
 * @Email: shiyu.fu@horizon.ai
 * @Date: Thu May 20 2021 07:41:00
 */

#ifndef LMKS_PROCESS_H_
#define LMKS_PROCESS_H_

#include <map>
#include <mutex>
#include <vector>
#include <memory>
#include "hobotlog/hobotlog.hpp"
#include "xstream/xstream_world.h"
#include "xstream/vision_type.h"

namespace inference {

using xstream::BaseDataVector;

struct LmksProcessTensor {
 public:
  LmksProcessTensor(int dim1_, int dim2_, int dim3_, int dim4_)
      : dim1(dim1_), dim2(dim2_), dim3(dim3_), dim4(dim4_) {
    if (!(dim1_ > 0 && dim2_ > 0 && dim3_ > 0 && dim4_ > 0)) {
      LOGE << "Failed to create tensor of shape: " << dim1_ << " " << dim2_
           << " " << dim3_ << " " << dim4_;
    }
    HOBOT_CHECK(dim1_ >= 0 && dim2_ > 0 && dim3_ > 0 && dim4_ > 0);
    data = std::vector<float>(dim1_ * dim2_ * dim3_ * dim4_, 0);
  }

  float At(int x1, int x2, int x3, int x4) {
    return data[x1 * dim2 * dim3 * dim4 + x2 * dim3 * dim4 + x3 * dim4 + x4];
  }

  void Set(int x1, int x2, int x3, int x4, float value) {
    if (!(x1 >= 0 && x1 < dim1) || !(x2 >= 0 && x2 < dim2) ||
        !(x3 >= 0 && x3 < dim3) || !(x4 >= 0 && x4 < dim4)) {
      LOGE << "Error set tensor at position: " << x1 << " " << x2 << " " << x3
           << " " << x4 << ", with shape: " << dim1 << " " << dim2 << " "
           << dim3 << " " << dim4;
    }
    HOBOT_CHECK(x1 >= 0 && x1 < dim1);
    HOBOT_CHECK(x2 >= 0 && x2 < dim2);
    HOBOT_CHECK(x3 >= 0 && x3 < dim3);
    HOBOT_CHECK(x4 >= 0 && x4 < dim4);
    data[x1 * dim2 * dim3 * dim4 + x2 * dim3 * dim4 + x3 * dim4 + x4] = value;
  }

  std::vector<float> data;

 private:
  int dim1;
  int dim2;
  int dim3;
  int dim4;
};

class FeatureSequenceBuffer {
 public:
  FeatureSequenceBuffer() {
    max_len_ = 100;
    len_ = 0;
    feats_ = std::make_shared<BaseDataVector>();
    boxes_ = std::make_shared<BaseDataVector>();
  }
  explicit FeatureSequenceBuffer(int buff_len_) : len_(0), max_len_(buff_len_) {
    feats_ = std::make_shared<BaseDataVector>();
    boxes_ = std::make_shared<BaseDataVector>();
  }

  void Update(std::shared_ptr<xstream::BBox> box,
              std::shared_ptr<xstream::Landmarks> kps, uint64_t timestamp);

  void GetClipFeatByEnd(std::shared_ptr<BaseDataVector> kpses,
                        std::shared_ptr<BaseDataVector> boxes, int num,
                        float stride, float margin, uint64_t end);

  void Clean() {
    timestamps_.clear();
    feats_->datas_.clear();
    boxes_->datas_.clear();
  }

 private:
  int len_ = 0;
  int max_len_;
  // Float data type has precision loss if the ts value is large number
  std::vector<uint64_t> timestamps_;
  std::shared_ptr<BaseDataVector> feats_;
  std::shared_ptr<BaseDataVector> boxes_;
};

class LmksProcess {
 public:
  int Init(int num_kps, int seq_len, float stride, float max_gap,
           float kps_norm_scale, bool norm_kps_conf, int buffer_len = 100) {
    num_kps_ = num_kps;
    seq_len_ = seq_len;
    stride_ = stride;
    max_gap_ = max_gap;
    kps_norm_scale_ = kps_norm_scale;
    buff_len_ = buffer_len;
    norm_kps_conf_ = norm_kps_conf;
    LOGD << "num_kps: " << num_kps << ", seq_len: " << seq_len
         << ", stride: " << stride << ", max_gap: " << max_gap
         << ", buff_len: " << buffer_len
         << "kps_norm_scale: " << kps_norm_scale_;
    return 0;
  }

  void Update(std::shared_ptr<xstream::BBox> box,
              std::shared_ptr<xstream::Landmarks> kps, uint64_t timestamp);

  void NormKps(std::shared_ptr<BaseDataVector> kpses,
               std::shared_ptr<BaseDataVector> boxes);

  void GetClipKps(std::shared_ptr<BaseDataVector> kpses, int track_id,
                  uint64_t timestamp);

  void Clean(std::shared_ptr<BaseDataVector> disappeared_track_ids);

  void Execute(std::shared_ptr<xstream::BBox> box,
               std::shared_ptr<xstream::Landmarks> kps,
               std::shared_ptr<BaseDataVector> cached_kpses,
               uint64_t timestamp);

 private:
  std::map<int, FeatureSequenceBuffer> track_buffers_;
  int num_kps_;
  int seq_len_;
  float stride_;
  float max_gap_;
  int buff_len_;
  int rotate_degree_;
  bool norm_kps_conf_;
  float max_score_ = -1;
  float kps_norm_scale_ = 1;
  std::mutex map_mutex_;
};

}  // namespace inference

#endif  // LMKS_PROCESS_H_
