//
// Created by yaoyao.sun on 2019-05-14.
// Copyright (c) 2019 Horizon Robotics. All rights reserved.
//

#include "plate_vote_method/plate_vote_method.h"

#include <gtest/gtest.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "xstream/vision_type.h"
#include "xstream/xstream_sdk.h"


using xstream::BaseData;
using xstream::BaseDataPtr;
using xstream::BaseDataVector;
using xstream::InputData;
using xstream::InputDataPtr;
using xstream::InputParamPtr;
using xstream::BBox;

using xstream::PlateVoteMethod;
TEST(PLATE_VOTE_TEST, Basic) {
  PlateVoteMethod plate_vote_method;

  std::string config_file = "./config/plate_vote.json";
  plate_vote_method.Init(config_file);

  std::vector<BaseDataPtr> xstream_output;
  std::vector<int> number1(7, 1);
  std::vector<int> number2(7, 2);
  std::vector<std::string> alphabet1(7, "A");
  std::vector<std::string> alphabet2(7, "B");
  for (int i = 0; i < 4; ++i) {
    std::shared_ptr<BaseDataVector> plate_rects_ptr(new BaseDataVector());
    std::shared_ptr<xstream::BBox> box(new xstream::BBox());
    box->id_ = 1;
    plate_rects_ptr->datas_.push_back(box);

    std::shared_ptr<BaseDataVector> plate_ptr(new BaseDataVector);
    std::shared_ptr<xstream::DataArray_<int>> numerical_plate(
        new xstream::DataArray_<int>());
    numerical_plate->values_ = number1;
    plate_ptr->datas_.push_back(numerical_plate);

    std::shared_ptr<BaseDataVector> disappeared_track_ids_ptr(
        new BaseDataVector);

    std::vector<BaseDataPtr> input;
    xstream::InputParamPtr param;
    input.push_back(plate_rects_ptr);
    input.push_back(disappeared_track_ids_ptr);
    input.push_back(plate_ptr);

    xstream_output = plate_vote_method.DoProcess(input, param);
  }

  {
    auto one_frame_out = xstream_output;
    ASSERT_EQ(one_frame_out.size(), static_cast<std::size_t>(1));
    auto out_plate_ptr = one_frame_out[0];
    auto out_plate =
        std::static_pointer_cast<BaseDataVector>(out_plate_ptr)->datas_;
    ASSERT_EQ(out_plate.size(), static_cast<std::size_t>(1));
    auto plate_result =
        std::static_pointer_cast<xstream::PlateNum>(out_plate[0])
            ->specific_type_;
    ASSERT_EQ(alphabet1.size(), plate_result.size());
    for (size_t i = 0; i < alphabet1.size(); ++i) {
      ASSERT_EQ(alphabet1.at(i), std::string(plate_result.at(i), 1));
    }
  }

  for (int i = 0; i < 5; ++i) {
    std::shared_ptr<BaseDataVector> plate_rects_ptr(new BaseDataVector());
    std::shared_ptr<xstream::BBox> box(new xstream::BBox());
    box->id_ = 1;
    plate_rects_ptr->datas_.push_back(box);

    std::shared_ptr<BaseDataVector> plate_ptr(new BaseDataVector);
    std::shared_ptr<xstream::DataArray_<int>> numerical_plate(
        new xstream::DataArray_<int>());
    numerical_plate->values_ = number2;
    plate_ptr->datas_.push_back(numerical_plate);

    std::shared_ptr<BaseDataVector> disappeared_track_ids_ptr(
        new BaseDataVector);

    std::vector<BaseDataPtr> input;
    xstream::InputParamPtr param;
    input.push_back(plate_rects_ptr);
    input.push_back(disappeared_track_ids_ptr);
    input.push_back(plate_ptr);

    xstream_output = plate_vote_method.DoProcess(input, param);
  }

  {
    auto one_frame_out = xstream_output;
    ASSERT_EQ(one_frame_out.size(), static_cast<std::size_t>(1));
    auto out_plate_ptr = one_frame_out[0];
    auto out_plate =
        std::static_pointer_cast<BaseDataVector>(out_plate_ptr)->datas_;
    ASSERT_EQ(out_plate.size(), static_cast<std::size_t>(1));
    auto plate_result =
        std::static_pointer_cast<xstream::PlateNum>(out_plate[0])
            ->specific_type_;
    ASSERT_EQ(alphabet2.size(), plate_result.size());
    for (size_t i = 0; i < alphabet2.size(); ++i) {
      ASSERT_EQ(alphabet2.at(i), std::string(plate_result.at(i), 1));
    }
  }
  plate_vote_method.Finalize();
}

TEST(PLATE_VOTE_TEST, GetInfo) {
  PlateVoteMethod plate_vote_method;
  std::string config_file = "./config/plate_vote.json";
  auto ret = plate_vote_method.Init(config_file);
  ASSERT_EQ(ret, 0);

  auto param = plate_vote_method.GetParameter();
  ASSERT_TRUE(param == nullptr);
  auto version = plate_vote_method.GetVersion();
  ASSERT_EQ(version, "");
  auto plate_vote_param =
      std::make_shared<xstream::PlateVoteParam>("PlateVoteMethod");
  ret = plate_vote_method.UpdateParameter(plate_vote_param);
  ASSERT_EQ(ret, 0);

  plate_vote_method.Finalize();
}
