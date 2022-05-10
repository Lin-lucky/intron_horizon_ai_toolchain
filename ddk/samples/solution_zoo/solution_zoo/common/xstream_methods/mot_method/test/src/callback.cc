/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @brief     callback
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2019.7.25
 */

#include <fstream>
#include <vector>
#include <string>

#include "hobotlog/hobotlog.hpp"
#include "xstream/vision_type.h"
#include "xstream/xstream_sdk.h"
#include "gtest/gtest.h"
#include "callback.hpp"

using xstream::BBox;
typedef xstream::Attribute_<uint32_t> XStreamUint32;

std::string& TestCallback::GetLog() {
  return id_bbox_track_res_;
}

void TestCallback::SaveOutputLog(const xstream::OutputDataPtr &output) {
  using xstream::BaseDataVector;
  assert(output->datas_.size() >= 2);
  auto &bbox_list = output->datas_[0];
  auto bbox_data =
      std::static_pointer_cast<BaseDataVector>(bbox_list);
  for (auto& pbbox : bbox_data->datas_) {
    assert("BBox" == pbbox->type_);
    auto bbox = std::static_pointer_cast<BBox>(pbbox);
    int est_left = static_cast<int>(bbox->x1_);
    int est_top = static_cast<int>(bbox->y1_);
    int est_right = static_cast<int>(bbox->x2_);
    int est_bottom = static_cast<int>(bbox->y2_);
//    int score =  static_cast<int>(bbox->score);
    id_bbox_track_res_ += (std::to_string(bbox->id_) + " ");
    id_bbox_track_res_ += (std::to_string(0) + " ");
    id_bbox_track_res_ += (std::to_string(est_left) + " ");
    id_bbox_track_res_ += (std::to_string(est_top) + " ");
    id_bbox_track_res_ += (std::to_string(est_right) + " ");
    id_bbox_track_res_ += (std::to_string(est_bottom) + " ");
  }
  id_bbox_track_res_ += "\n";

  auto &disappeared_track_id_list = output->datas_[1];
  auto disappeared_id_data =
      std::static_pointer_cast<BaseDataVector>(disappeared_track_id_list);
  for (auto& pid : disappeared_id_data->datas_) {
    assert("Number" == pid->type_);
    auto disappeared_id = std::static_pointer_cast<XStreamUint32>(pid);
    LOGI << "Track:" << disappeared_id->value_ << " disappeared";
  }
}

void TestCallback::IsSameBBox(const std::vector<xstream::BaseDataPtr> &input,
                              const std::vector<xstream::BaseDataPtr> &output,
                              bool has_id) {
  using xstream::BaseDataVector;
  auto &p_in_bbox_list = input[0];
  auto in_bbox_data =
      std::static_pointer_cast<BaseDataVector>(p_in_bbox_list);
  auto &in_bbox_list = in_bbox_data->datas_;

  auto &p_out_bbox_list = output[0];
  auto out_bbox_data =
      std::static_pointer_cast<BaseDataVector>(p_out_bbox_list);
  auto &out_bbox_list = out_bbox_data->datas_;

  EXPECT_EQ(in_bbox_list.size(), out_bbox_list.size());
  size_t size = in_bbox_list.size();

  for (size_t i = 0; i < size; i++) {
    auto in_bbox = std::static_pointer_cast<BBox>(in_bbox_list[i]);
    auto out_bbox = std::static_pointer_cast<BBox>(out_bbox_list[i]);
    EXPECT_EQ(in_bbox->x1_, out_bbox->x1_);
    EXPECT_EQ(in_bbox->y1_, out_bbox->y1_);
    EXPECT_EQ(in_bbox->x2_, out_bbox->x2_);
    EXPECT_EQ(in_bbox->y2_, out_bbox->y2_);
    if (has_id) {
      EXPECT_TRUE(out_bbox->id_);
    } else {
      EXPECT_FALSE(out_bbox->id_);
    }
  }
}
