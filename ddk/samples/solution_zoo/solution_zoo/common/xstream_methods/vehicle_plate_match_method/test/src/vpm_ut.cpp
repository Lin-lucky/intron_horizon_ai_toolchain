//
// Created by yaoyao.sun on 2019-05-14.
// Copyright (c) 2019 Horizon Robotics. All rights reserved.
//

#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <gtest/gtest.h>

#include "vehicle_plate_match_method/vehicle_plate_match_method.h"
#include "xstream/xstream_sdk.h"
// #include "hobotxstream/method.h"
#include "xstream/vision_type.h"


using xstream::BBox;
using xstream::BaseData;
using xstream::BaseDataPtr;
using xstream::BaseDataVector;
using xstream::InputData;
using xstream::InputDataPtr;
using xstream::InputParamPtr;
using xstream::VehiclePlateMatchMethod;

TEST(VehiclePlateMatchMethod_TEST, Basic) {
  int rv;
  std::string str;
  VehiclePlateMatchMethod vpm_method;
  std::string config_fname("./config/config_match.json");

  rv = vpm_method.Init(config_fname);
  EXPECT_EQ(rv, 0);

  str = vpm_method.GetVersion();

  InputParamPtr param_in;
  rv = vpm_method.UpdateParameter(param_in);
  EXPECT_EQ(rv, 0);

  param_in  = vpm_method.GetParameter();
  EXPECT_EQ(param_in, nullptr);

    // 1. input
    // 1.1 vehicle_bbox_list
    xstream::BBox *bbox1(new xstream::BBox(0, 0, 100, 100));
    bbox1->type_ = "BBox";
    auto data_vbox = std::make_shared<BaseDataVector>();
    data_vbox->name_ = "vehicle_bbox_list";
    data_vbox->datas_.push_back(BaseDataPtr(bbox1));
    // 1.2 vehicle_lmk
    xstream::Landmarks *lmks1 = new xstream::Landmarks();
    lmks1->values_.push_back(xstream::Point(0, 0, 0));
    auto data_vlmk = std::make_shared<BaseDataVector>();
    data_vlmk->name_ = "vehicle_lmk";
    data_vlmk->datas_.push_back(BaseDataPtr(lmks1));
    // 1.3 plate_bbox_list
    xstream::BBox *pbbox1(new xstream::BBox(30, 60, 90, 90));
    pbbox1->type_ = "BBox";
    auto data_pbox = std::make_shared<BaseDataVector>();
    data_pbox->name_ = "plate_bbox_list";
    data_pbox->datas_.push_back(BaseDataPtr(pbbox1));
    // 1.4 plate_lmk
    xstream::Landmarks *plmks1 = new xstream::Landmarks();
    plmks1->values_.push_back(xstream::Point(0, 0, 0));
    auto data_plmk = std::make_shared<BaseDataVector>();
    data_plmk->name_ = "plate_lmk";
    data_plmk->datas_.push_back(BaseDataPtr(plmks1));
    // 1.5 plate_type
    xstream::Attribute_<int> *ptype = new xstream::Attribute_<int>();
    ptype->value_ = 1;
    auto data_ptype = std::make_shared<BaseDataVector>();
    data_ptype->name_ = "plate_type";
    data_ptype->datas_.push_back(BaseDataPtr(ptype));
    // 1.6 plate_color
    xstream::Attribute_<int> *pcolor = new xstream::Attribute_<int>();
    pcolor->value_ = 1;
    auto data_pcolor = std::make_shared<BaseDataVector>();
    data_pcolor->name_ = "plate_color";
    data_pcolor->datas_.push_back(BaseDataPtr(pcolor));

  std::vector<BaseDataPtr> input;
  input.push_back(data_vbox);
  input.push_back(data_pbox);
  input.push_back(data_vlmk);
  input.push_back(data_plmk);
  input.push_back(data_ptype);
  input.push_back(data_pcolor);

  // 2. build param
  xstream::InputParamPtr param;

  // 3. run
  auto xstream_output = vpm_method.DoProcess(input, param);

  // 4. end
  vpm_method.Finalize();
  return;
}
