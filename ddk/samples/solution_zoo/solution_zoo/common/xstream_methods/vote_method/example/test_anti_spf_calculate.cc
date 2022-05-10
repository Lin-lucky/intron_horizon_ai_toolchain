//
// Created by yaoyao.sun on 2019-05-24.
// Copyright (c) 2019 Horizon Robotics. All rights reserved.
//

#include <assert.h>
#include <fstream>
#include <iostream>
#include <string>

#include "vote_method/vote_method.h"
#include "hobotlog/hobotlog.hpp"
#include "xstream/xstream_sdk.h"
#include "xstream/vision_type.h"

using xstream::BaseData;
using xstream::BaseDataPtr;
using xstream::BaseDataVector;
using xstream::InputData;
using xstream::InputDataPtr;

using xstream::VoteMethod;
using xstream::BBox;

int TestAntiSpfCalculate(int argc, char **argv) {
  VoteMethod anti_spf_calculate_method;
  std::string config_file = "./config/vote_method.json";
  anti_spf_calculate_method.Init(config_file);

  std::shared_ptr<BaseDataVector> face_rects_ptr(new BaseDataVector());
  std::shared_ptr<BBox> box1(new BBox());
  box1->id_ = 1;
  std::shared_ptr<BBox> box2(new BBox());
  box2->id_ = 2;

  face_rects_ptr->datas_.push_back(box1);
  face_rects_ptr->datas_.push_back(box2);

  std::shared_ptr<BaseDataVector> liveness_ptr(new BaseDataVector);
  std::shared_ptr<xstream::Attribute_<int>> liveness1(
      new xstream::Attribute_<int>());
  liveness1->value_ = 1;
  std::shared_ptr<xstream::Attribute_<int>> liveness2(
      new xstream::Attribute_<int>());
  liveness2->value_ = 2;
  liveness_ptr->datas_.push_back(liveness1);
  liveness_ptr->datas_.push_back(liveness2);

  std::shared_ptr<BaseDataVector> disappeared_track_ids_ptr(
      new BaseDataVector);
  std::shared_ptr<xstream::Attribute_<uint32_t>> disappeard_track_id1(
      new xstream::Attribute_<uint32_t>());
  disappeard_track_id1->value_ = 1;
  disappeared_track_ids_ptr->datas_.push_back(disappeard_track_id1);

  std::vector<std::vector<BaseDataPtr>> input;
  std::vector<xstream::InputParamPtr> param;
  int batch_size = 5;
  input.resize(batch_size);
  param.resize(batch_size);
  for (int i = 0; i < batch_size - 1; ++i) {
    input[i].push_back(face_rects_ptr);
    input[i].push_back(nullptr);
    input[i].push_back(liveness_ptr);
  }
  std::shared_ptr<BaseDataVector> face_rects_ptr2(new BaseDataVector());
  std::shared_ptr<BBox> box12(new BBox());
  box12->id_ = 1;
  std::shared_ptr<BBox> box22(new BBox());
  box22->id_ = 2;
  face_rects_ptr2->datas_.push_back(box12);
  face_rects_ptr2->datas_.push_back(box22);

  std::shared_ptr<BaseDataVector> liveness_ptr2(new BaseDataVector);
  std::shared_ptr<xstream::Attribute_<int>> liveness12(
      new xstream::Attribute_<int>());
  liveness12->value_ = 2;
  std::shared_ptr<xstream::Attribute_<int>> liveness22(
      new xstream::Attribute_<int>());
  liveness22->value_ = 3;

  liveness_ptr2->datas_.push_back(liveness12);
  liveness_ptr2->datas_.push_back(liveness22);

  std::shared_ptr<BaseDataVector> disappeared_track_ids_ptr2(
      new BaseDataVector);
  std::shared_ptr<xstream::Attribute_<uint32_t>> disappeard_track_id12(
      new xstream::Attribute_<uint32_t>());
  disappeard_track_id12->value_ = 1;
  disappeared_track_ids_ptr2->datas_.push_back(disappeard_track_id12);
  input[batch_size - 1].push_back(face_rects_ptr2);
  input[batch_size - 1].push_back(disappeared_track_ids_ptr2);
  input[batch_size - 1].push_back(liveness_ptr2);
#if 0

  std::vector<std::vector<BaseDataPtr>> xstream_output;
  for (int index = 0; index < batch_size; index++) {
    auto one_output =
        anti_spf_calculate_method.DoProcess(input[index], param[index]);
    xstream_output.push_back(one_output);
  }
  HOBOT_CHECK(static_cast<int>(xstream_output.size()) == batch_size);
  for (int i = 0; i < batch_size; ++i) {
    auto one_frame_out = xstream_output[i];
    HOBOT_CHECK(one_frame_out.size() == 2);
    auto out_track_ids_ptr = one_frame_out[0];
    auto out_liveness_ptr = one_frame_out[1];
    auto out_track_ids =
        std::static_pointer_cast<BaseDataVector>(out_track_ids_ptr)->datas_;
    auto out_liveness =
        std::static_pointer_cast<BaseDataVector>(out_liveness_ptr)->datas_;
    HOBOT_CHECK(out_liveness.size() == out_track_ids.size());
    std::cout << "frame id: " << i << std::endl;
    for (size_t i = 0; i < out_liveness.size(); ++i) {
      auto track_id =
          std::static_pointer_cast<xstream::Attribute_<uint32_t>>(
              out_track_ids[i])->value_;
      auto liveness = std::static_pointer_cast<
          xstream::Attribute_<int>>(out_liveness[i])->value_;
      std::cout << "track_id: " << track_id << ", liveness: " << liveness
                << std::endl;
    }
  }

  anti_spf_calculate_method.Finalize();
#endif
  return 0;
}
