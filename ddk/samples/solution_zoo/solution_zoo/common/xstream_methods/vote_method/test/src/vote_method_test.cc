//
// Created by yaoyao.sun on 2019-05-14.
// Copyright (c) 2019 Horizon Robotics. All rights reserved.
//

#include <gtest/gtest.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "xstream/xstream_sdk.h"
#include "xstream/vision_type.h"
#include "vote_method/vote_method.h"

typedef xstream::Attribute_<int32_t> Attribute;
using xstream::BBox;
using xstream::AntiSpoofing;
using xstream::BaseDataPtr;
using xstream::BaseDataVector;
using xstream::InputParamPtr;
using xstream::InputParam;
using xstream::VoteMethod;

TEST(ANTI_SPF_TEST, Basic) {
  VoteMethod anti_spf_calculate_method;
  std::string config_file = "./config/living.json";
  anti_spf_calculate_method.Init(config_file);
  std::vector<BaseDataPtr> xstream_output;
  for (int i = 0; i < 6; ++i) {
    std::shared_ptr<BaseDataVector> face_rects_ptr(new BaseDataVector());
    std::shared_ptr<BBox> box(new BBox());
    box->id_ = 1;
    face_rects_ptr->datas_.push_back(box);

    std::shared_ptr<BaseDataVector> liveness_ptr(new BaseDataVector);
    std::shared_ptr<AntiSpoofing> liveness(new AntiSpoofing());
    liveness->value_ = 1;
    liveness_ptr->datas_.push_back(liveness);

    std::shared_ptr<BaseDataVector> disappeared_track_ids_ptr(
        new BaseDataVector());
    std::vector<BaseDataPtr> input;
    xstream::InputParamPtr param;
    input.push_back(face_rects_ptr);
    input.push_back(disappeared_track_ids_ptr);
    input.push_back(liveness_ptr);

    xstream_output = anti_spf_calculate_method.DoProcess(input, param);
  }

  {
    auto one_frame_out = xstream_output;
    ASSERT_EQ(one_frame_out.size(), 2u);
    auto out_track_ids_ptr = one_frame_out[0];
    auto out_liveness_ptr = one_frame_out[1];
    auto out_track_ids =
        std::static_pointer_cast<BaseDataVector>(out_track_ids_ptr)->datas_;
    auto out_liveness =
        std::static_pointer_cast<BaseDataVector>(out_liveness_ptr)->datas_;
    ASSERT_EQ(out_liveness.size(), out_track_ids.size());
    ASSERT_EQ(out_liveness.size(), 1u);
    auto track_id =
        std::static_pointer_cast<xstream::Attribute_<uint32_t>>(
            out_track_ids[0])->value_;
    auto live_result =
        std::static_pointer_cast<AntiSpoofing>(out_liveness[0])->value_;
    ASSERT_EQ(track_id, 1u);
    ASSERT_EQ(live_result, -1);
  }

  for (int i = 0; i < 11; ++i) {
    std::shared_ptr<BaseDataVector> face_rects_ptr(new BaseDataVector());
    std::shared_ptr<BBox> box(new BBox());
    box->id_ = 1;
    face_rects_ptr->datas_.push_back(box);

    std::shared_ptr<BaseDataVector> liveness_ptr(new BaseDataVector);
    std::shared_ptr<AntiSpoofing> liveness(new AntiSpoofing());
    liveness->value_ = 1;
    liveness_ptr->datas_.push_back(liveness);

    std::shared_ptr<BaseDataVector> disappeared_track_ids_ptr(
        new BaseDataVector);

    std::vector<BaseDataPtr> input;
    xstream::InputParamPtr param;
    input.push_back(face_rects_ptr);
    input.push_back(disappeared_track_ids_ptr);
    input.push_back(liveness_ptr);

    xstream_output = anti_spf_calculate_method.DoProcess(input, param);
    auto one_frame_out = xstream_output;
    ASSERT_EQ(one_frame_out.size(), 2u);
    auto out_track_ids_ptr = one_frame_out[0];
    auto out_liveness_ptr = one_frame_out[1];
    auto out_track_ids =
        std::static_pointer_cast<BaseDataVector>(out_track_ids_ptr)->datas_;
    auto out_liveness =
        std::static_pointer_cast<BaseDataVector>(out_liveness_ptr)->datas_;
    ASSERT_EQ(out_liveness.size(), out_track_ids.size());
    ASSERT_EQ(out_liveness.size(), 1u);
    auto track_id =
        std::static_pointer_cast<xstream::Attribute_<uint32_t>>(
            out_track_ids[0])->value_;
    auto live_result =
        std::static_pointer_cast<AntiSpoofing>(out_liveness[0])->value_;
    ASSERT_EQ(track_id, 1u);
    ASSERT_EQ(live_result, 1);
  }

  for (int i = 0; i < 4; ++i) {
    std::shared_ptr<BaseDataVector> face_rects_ptr(new BaseDataVector());
    std::shared_ptr<BBox> box(new BBox());
    box->id_ = 1;
    face_rects_ptr->datas_.push_back(box);

    std::shared_ptr<BaseDataVector> liveness_ptr(new BaseDataVector);
    std::shared_ptr<AntiSpoofing> liveness(new AntiSpoofing());
    liveness->value_ = 0;
    liveness_ptr->datas_.push_back(liveness);

    std::shared_ptr<BaseDataVector> disappeared_track_ids_ptr(
        new BaseDataVector);

    std::vector<BaseDataPtr> input;
    xstream::InputParamPtr param;
    input.push_back(face_rects_ptr);
    input.push_back(disappeared_track_ids_ptr);
    input.push_back(liveness_ptr);

    xstream_output = anti_spf_calculate_method.DoProcess(input, param);
  }

  {
    auto one_frame_out = xstream_output;
    ASSERT_EQ(one_frame_out.size(), 2u);
    auto out_track_ids_ptr = one_frame_out[0];
    auto out_liveness_ptr = one_frame_out[1];
    auto out_track_ids =
        std::static_pointer_cast<BaseDataVector>(out_track_ids_ptr)->datas_;
    auto out_liveness =
        std::static_pointer_cast<BaseDataVector>(out_liveness_ptr)->datas_;
    ASSERT_EQ(out_liveness.size(), out_track_ids.size());
    ASSERT_EQ(out_liveness.size(), 1u);
    auto track_id =
        std::static_pointer_cast<xstream::Attribute_<uint32_t>>(
            out_track_ids[0])->value_;
    auto live_result =
        std::static_pointer_cast<AntiSpoofing>(out_liveness[0])
            ->value_;
    ASSERT_EQ(track_id, 1u);
    ASSERT_EQ(live_result, 0);
  }

  for (int i = 0; i < 15; ++i) {
    std::shared_ptr<BaseDataVector> face_rects_ptr(new BaseDataVector());
    std::shared_ptr<BBox> box(new BBox());
    box->id_ = 1;
    face_rects_ptr->datas_.push_back(box);

    std::shared_ptr<BaseDataVector> liveness_ptr(new BaseDataVector);
    std::shared_ptr<AntiSpoofing> liveness(new AntiSpoofing());
    liveness->value_ = -1;
    liveness_ptr->datas_.push_back(liveness);

    std::shared_ptr<BaseDataVector> disappeared_track_ids_ptr(
        new BaseDataVector);

    std::vector<BaseDataPtr> input;
    xstream::InputParamPtr param;
    input.push_back(face_rects_ptr);
    input.push_back(disappeared_track_ids_ptr);
    input.push_back(liveness_ptr);

    xstream_output = anti_spf_calculate_method.DoProcess(input, param);
  }

  {
    auto one_frame_out = xstream_output;
    ASSERT_EQ(one_frame_out.size(), 2u);
    auto out_track_ids_ptr = one_frame_out[0];
    auto out_liveness_ptr = one_frame_out[1];
    auto out_track_ids =
        std::static_pointer_cast<BaseDataVector>(out_track_ids_ptr)->datas_;
    auto out_liveness =
        std::static_pointer_cast<BaseDataVector>(out_liveness_ptr)->datas_;
    ASSERT_EQ(out_liveness.size(), out_track_ids.size());
    ASSERT_EQ(out_liveness.size(), 1u);
    auto track_id =
        std::static_pointer_cast<xstream::Attribute_<uint32_t>>(
            out_track_ids[0])->value_;
    auto live_result =
        std::static_pointer_cast<AntiSpoofing>(out_liveness[0])
            ->value_;
    ASSERT_EQ(track_id, 1u);
    ASSERT_EQ(live_result, -1);  // invalid.
  }

  anti_spf_calculate_method.Finalize();
}

TEST(VEHICLE_TYPE_COLOR_TEST, Basic) {
  VoteMethod vehicle;
  std::string config_file = "./config/vehicle_type_color.json";
  vehicle.Init(config_file);

  std::vector<BaseDataPtr> xstream_output;
  for (int i = 0; i < 6; ++i) {
    std::shared_ptr<BaseDataVector> vehicle_rects_ptr(new BaseDataVector());
    std::shared_ptr<BBox> box(new BBox());
    box->id_ = 1;
    vehicle_rects_ptr->datas_.push_back(box);

    std::shared_ptr<BaseDataVector> vehicle_ptr(new BaseDataVector);
    std::shared_ptr<Attribute> vehicle_info(new Attribute);
    vehicle_info->value_ = 1;
    vehicle_ptr->datas_.push_back(vehicle_info);

    std::shared_ptr<BaseDataVector> disappeared_track_ids_ptr(
        new BaseDataVector);

    std::vector<BaseDataPtr> input;
    xstream::InputParamPtr param;
    input.push_back(vehicle_rects_ptr);
    input.push_back(disappeared_track_ids_ptr);
    input.push_back(vehicle_ptr);

    xstream_output = vehicle.DoProcess(input, param);
  }

  {
    auto one_frame_out = xstream_output;
    ASSERT_EQ(one_frame_out.size(), 1u);
    auto out_vehicle_ptr = one_frame_out[0];
    auto out_vehicle =
        std::static_pointer_cast<BaseDataVector>(out_vehicle_ptr)->datas_;
    ASSERT_EQ(out_vehicle.size(), 1u);
    auto vehicle_result =
        std::static_pointer_cast<Attribute>(out_vehicle[0])->value_;
    ASSERT_EQ(vehicle_result, 1);
  }

  for (int i = 0; i < 10; ++i) {
    std::shared_ptr<BaseDataVector> vehicle_rects_ptr(new BaseDataVector());
    std::shared_ptr<BBox> box(new BBox());
    box->id_ = 1;
    vehicle_rects_ptr->datas_.push_back(box);

    std::shared_ptr<BaseDataVector> vehicle_ptr(new BaseDataVector);
    std::shared_ptr<Attribute> vehicle_info(new Attribute());
    vehicle_info->value_ = 2;
    vehicle_ptr->datas_.push_back(vehicle_info);

    std::shared_ptr<BaseDataVector> disappeared_track_ids_ptr(
        new BaseDataVector);

    std::vector<BaseDataPtr> input;
    xstream::InputParamPtr param;
    input.push_back(vehicle_rects_ptr);
    input.push_back(disappeared_track_ids_ptr);
    input.push_back(vehicle_ptr);

    xstream_output = vehicle.DoProcess(input, param);
  }

  {
    auto one_frame_out = xstream_output;
    ASSERT_EQ(one_frame_out.size(), 1u);
    auto out_vehicle_ptr = one_frame_out[0];
    auto out_vehicle =
        std::static_pointer_cast<BaseDataVector>(out_vehicle_ptr)->datas_;
    ASSERT_EQ(out_vehicle.size(), 1u);
    auto vehicle_result =
        std::static_pointer_cast<Attribute>(out_vehicle[0])->value_;
    ASSERT_EQ(vehicle_result, 2);
  }

  vehicle.Finalize();
}

TEST(PLATE_COLOR_TEST, Basic) {
  VoteMethod plate_color;
  std::string config_file = "./config/plate_color.json";
  plate_color.Init(config_file);

  std::vector<BaseDataPtr> xstream_output;
  for (int i = 0; i < 6; ++i) {
    std::shared_ptr<BaseDataVector> plate_color_rects_ptr(new BaseDataVector());
    std::shared_ptr<BBox> box(new BBox());
    box->id_ = 1;
    plate_color_rects_ptr->datas_.push_back(box);

    std::shared_ptr<BaseDataVector> plate_color_ptr(new BaseDataVector);
    std::shared_ptr<Attribute> plate_color_info(new Attribute());
    plate_color_info->value_ = 1;
    plate_color_ptr->datas_.push_back(plate_color_info);

    std::shared_ptr<BaseDataVector> disappeared_track_ids_ptr(
        new BaseDataVector);

    std::vector<BaseDataPtr> input;
    xstream::InputParamPtr param;
    input.push_back(plate_color_rects_ptr);
    input.push_back(disappeared_track_ids_ptr);
    input.push_back(plate_color_ptr);

    xstream_output = plate_color.DoProcess(input, param);
  }

  {
    auto one_frame_out = xstream_output;
    ASSERT_EQ(one_frame_out.size(), 1u);
    auto out_plate_color_ptr = one_frame_out[0];
    auto out_plate_color =
        std::static_pointer_cast<BaseDataVector>(out_plate_color_ptr)->datas_;
    ASSERT_EQ(out_plate_color.size(), 1u);
    auto plate_color_result =
        std::static_pointer_cast<Attribute>(out_plate_color[0])->value_;
    ASSERT_EQ(plate_color_result, 1);
  }

  for (int i = 0; i < 10; ++i) {
    std::shared_ptr<BaseDataVector> plate_color_rects_ptr(new BaseDataVector());
    std::shared_ptr<BBox> box(new BBox());
    box->id_ = 1;
    plate_color_rects_ptr->datas_.push_back(box);

    std::shared_ptr<BaseDataVector> plate_color_ptr(new BaseDataVector);
    std::shared_ptr<Attribute> plate_color_info(new Attribute());
    plate_color_info->value_ = 3;
    plate_color_ptr->datas_.push_back(plate_color_info);

    std::shared_ptr<BaseDataVector> disappeared_track_ids_ptr(
        new BaseDataVector);

    std::vector<BaseDataPtr> input;
    xstream::InputParamPtr param;
    input.push_back(plate_color_rects_ptr);
    input.push_back(disappeared_track_ids_ptr);
    input.push_back(plate_color_ptr);

    xstream_output = plate_color.DoProcess(input, param);
  }

  {
    auto one_frame_out = xstream_output;
    ASSERT_EQ(one_frame_out.size(), 1u);
    auto out_plate_color_ptr = one_frame_out[0];
    auto out_plate_color =
        std::static_pointer_cast<BaseDataVector>(out_plate_color_ptr)->datas_;
    ASSERT_EQ(out_plate_color.size(), 1u);
    auto plate_color_result =
        std::static_pointer_cast<Attribute>(
            out_plate_color[0])->value_;
    ASSERT_EQ(plate_color_result, 3);
  }
  plate_color.Finalize();
}

TEST(GET_INFO_TEST, Basic) {
  VoteMethod vote_method;
  std::string config_file = "./config/living.json";
  auto ret = vote_method.Init(config_file);
  ASSERT_EQ(ret, 0);

  auto params = vote_method.GetParameter();
  EXPECT_EQ(params, nullptr);
  std::string version = vote_method.GetVersion();
  EXPECT_EQ(version, "");
  auto method_param = std::make_shared<xstream::VoteParam>("VoteMethod");
  int rest = vote_method.UpdateParameter(method_param);
  EXPECT_EQ(rest, 0);

  vote_method.Finalize();
}
