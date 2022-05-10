/*
 * @Description: implement of data_type
 * @Author: yaoyao.sun@horizon.ai
 * @Date: 2019-4-12 17:49:26
 * @LastEditors: hao.tian@horizon.ai
 * @LastEditTime: 2019-11-20 11:00:32
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "hobotlog/hobotlog.hpp"
#include "xstream/xstream_sdk.h"
#include "xstream/vision_type.h"

typedef xstream::Attribute_<uint32_t> XStreamUint32;

using xstream::BBox;
using xstream::Landmarks;
using xstream::BaseData;
using xstream::BaseDataPtr;
using xstream::BaseDataVector;
using xstream::InputData;
using xstream::InputDataPtr;

int CompareBBox(const BBox &bbox1, const BBox &bbox2) {
  return ((bbox1.x1_ == bbox2.x1_) &&
          (bbox1.y1_ == bbox2.y1_) &&
          (bbox1.x2_ == bbox2.x2_) &&
          (bbox1.y2_ == bbox2.y2_));
}

int CheckResult(
    const std::vector<BBox> &face_box_s,
    const std::vector<BBox> &head_box_s,
    const std::vector<std::pair<BBox, BBox>> &gt_bbox) {
  for (unsigned int i = 0; i < face_box_s.size(); ++i) {
    BBox corresponding_gt_nir_box;
    bool get_corresponding_nir_box = false;
    bool get_corresponding_gt_rgb_box = false;
    for (unsigned int j = 0; j < head_box_s.size(); ++j) {
      if (face_box_s[i].id_ == head_box_s[j].id_) {
        get_corresponding_nir_box = true;
        for (size_t k = 0; k < gt_bbox.size(); ++k) {
          if (CompareBBox(face_box_s[i], gt_bbox[k].first)) {
            get_corresponding_gt_rgb_box = true;
            corresponding_gt_nir_box = gt_bbox[k].second;
            if (CompareBBox(corresponding_gt_nir_box, head_box_s[j])) {
              break;
            } else {
              LOGE << "Camera Calibration performance is bad.";
              return -1;
            }
          }
        }
        if (!get_corresponding_gt_rgb_box) {
          LOGE << "Data chaos! RGB Result Box is not in RGB GT Box list";
          return -1;
        }
      }
    }
    if (!get_corresponding_nir_box) {
      LOGE << "Sadly, rgb box & nir box even not matched!";
      return -1;
    }
  }
  return 0;
}

class XStreamSDKTest : public ::testing::Test {
 public:
  static void SetUpTestCase() {
    SetLogLevel(HOBOT_LOG_VERBOSE);
    flow = xstream::XStreamSDK::CreateSDK();
    flow->SetConfig("config_file", "./config/merge_flow1.json");
    flow->Init();
    auto version = flow->GetVersion("merge_method");
    EXPECT_EQ(version, "0.0.17");
  }
  static void TearDownTestCase() { delete flow; }
  static xstream::XStreamSDK *flow;
};

xstream::XStreamSDK *XStreamSDKTest::flow = nullptr;

TEST_F(XStreamSDKTest, Basic) {
  InputDataPtr input(new InputData());
  auto face_box_list = std::make_shared<BaseDataVector>();
  face_box_list->name_ = "face_box";
  input->datas_.push_back(BaseDataPtr(face_box_list));

  auto disappeared_face_id_list = std::make_shared<BaseDataVector>();
  disappeared_face_id_list->name_ = "disappeared_face_id";
  input->datas_.push_back(BaseDataPtr(disappeared_face_id_list));

  auto head_box_list = std::make_shared<BaseDataVector>();
  head_box_list->name_ = "head_box";
  input->datas_.push_back(BaseDataPtr(head_box_list));

  auto disappeared_head_id_list = std::make_shared<BaseDataVector>();
  disappeared_head_id_list->name_ = "disappeared_head_id";
  input->datas_.push_back(BaseDataPtr(disappeared_head_id_list));

  auto rgb_lmk = std::make_shared<BaseDataVector>();
  rgb_lmk->name_ = "rgb_lmk";
  input->datas_.push_back(BaseDataPtr(rgb_lmk));

  auto nir_lmk = std::make_shared<BaseDataVector>();
  nir_lmk->name_ = "nir_lmk";
  input->datas_.push_back(BaseDataPtr(nir_lmk));

  auto face_box1 = std::make_shared<BBox>();
  face_box1->type_ = "BBox";
  face_box1->id_ = 10;
  face_box1->x1_ = 100;
  face_box1->y1_ = 100;
  face_box1->x2_ = 300;
  face_box1->y2_ = 200;

  auto face_box2 = std::make_shared<BBox>();
  face_box2->type_ = "BBox";
  face_box2->id_ = 12;
  face_box2->x1_ = 400;
  face_box2->y1_ = 100;
  face_box2->x2_ = 500;
  face_box2->y2_ = 200;

  auto face_box3 = std::make_shared<BBox>();
  face_box3->type_ = "BBox";
  face_box3->id_ = 14;
  face_box3->x1_ = 700;
  face_box3->y1_ = 100;
  face_box3->x2_ = 800;
  face_box3->y2_ = 200;

  face_box_list->datas_.emplace_back(BaseDataPtr(face_box1));
  face_box_list->datas_.emplace_back(BaseDataPtr(face_box2));
  face_box_list->datas_.emplace_back(BaseDataPtr(face_box3));

  auto head_box1 = std::make_shared<BBox>();
  head_box1->type_ = "BBox";
  head_box1->id_ = 11;
  head_box1->x1_ = 710;
  head_box1->y1_ = 90;
  head_box1->x2_ = 790;
  head_box1->y2_ = 200;

  auto head_box2 = std::make_shared<BBox>();
  head_box2->type_ = "BBox";
  head_box2->id_ = 13;
  head_box2->x1_ = 90;
  head_box2->y1_ = 110;
  head_box2->x2_ = 270;
  head_box2->y2_ = 200;

  head_box_list->datas_.emplace_back(BaseDataPtr(head_box1));
  head_box_list->datas_.emplace_back(BaseDataPtr(head_box2));

  auto face_track_id1 = std::make_shared<XStreamUint32>();
  face_track_id1->value_ = 1;
  auto face_track_id2 = std::make_shared<XStreamUint32>();
  face_track_id2->value_ = 2;
  disappeared_face_id_list->datas_.emplace_back(BaseDataPtr(face_track_id1));
  disappeared_face_id_list->datas_.emplace_back(BaseDataPtr(face_track_id2));

  auto head_track_id1 = std::make_shared<XStreamUint32>();
  head_track_id1->value_ = 3;
  auto head_track_id2 = std::make_shared<XStreamUint32>();
  head_track_id2->value_ = 4;

  disappeared_head_id_list->datas_.emplace_back(BaseDataPtr(head_track_id1));
  disappeared_head_id_list->datas_.emplace_back(BaseDataPtr(head_track_id2));

  auto face_landmarks1 = std::make_shared<Landmarks>();
  face_landmarks1->type_ = "Landmarks";
  face_landmarks1->values_.resize(5);
  face_landmarks1->values_[0].x_ = 10;
  face_landmarks1->values_[0].y_ = 10;
  face_landmarks1->values_[1].x_ = 15;
  face_landmarks1->values_[1].y_ = 15;
  face_landmarks1->values_[2].x_ = 20;
  face_landmarks1->values_[2].y_ = 20;
  face_landmarks1->values_[3].x_ = 25;
  face_landmarks1->values_[3].y_ = 25;
  face_landmarks1->values_[4].x_ = 30;
  face_landmarks1->values_[4].y_ = 30;

  auto face_landmarks2 = std::make_shared<Landmarks>();
  face_landmarks2->type_ = "Landmarks";
  face_landmarks2->values_.resize(5);
  face_landmarks2->values_[0].x_ = 35;
  face_landmarks2->values_[0].y_ = 35;
  face_landmarks2->values_[1].x_ = 40;
  face_landmarks2->values_[1].y_ = 40;
  face_landmarks2->values_[2].x_ = 45;
  face_landmarks2->values_[2].y_ = 45;
  face_landmarks2->values_[3].x_ = 50;
  face_landmarks2->values_[3].y_ = 50;
  face_landmarks2->values_[4].x_ = 55;
  face_landmarks2->values_[4].y_ = 55;

  rgb_lmk->datas_.emplace_back(BaseDataPtr(face_landmarks1));
  rgb_lmk->datas_.emplace_back(BaseDataPtr(face_landmarks2));

  auto head_landmarks1 = std::make_shared<Landmarks>();
  head_landmarks1->type_ = "Landmarks";
  head_landmarks1->values_.resize(5);
  head_landmarks1->values_[0].x_ = 10;
  head_landmarks1->values_[0].y_ = 10;
  head_landmarks1->values_[1].x_ = 15;
  head_landmarks1->values_[1].y_ = 15;
  head_landmarks1->values_[2].x_ = 20;
  head_landmarks1->values_[2].y_ = 20;
  head_landmarks1->values_[3].x_ = 25;
  head_landmarks1->values_[3].y_ = 25;
  head_landmarks1->values_[4].x_ = 30;
  head_landmarks1->values_[4].y_ = 30;

  auto head_landmarks2 = std::make_shared<Landmarks>();
  head_landmarks2->type_ = "Landmarks";
  head_landmarks2->values_.resize(5);
  head_landmarks2->values_[0].x_ = 35;
  head_landmarks2->values_[0].y_ = 35;
  head_landmarks2->values_[1].x_ = 40;
  head_landmarks2->values_[1].y_ = 40;
  head_landmarks2->values_[2].x_ = 45;
  head_landmarks2->values_[2].y_ = 45;
  head_landmarks2->values_[3].x_ = 50;
  head_landmarks2->values_[3].y_ = 50;
  head_landmarks2->values_[4].x_ = 55;
  head_landmarks2->values_[4].y_ = 55;

  nir_lmk->datas_.emplace_back(BaseDataPtr(head_landmarks1));
  nir_lmk->datas_.emplace_back(BaseDataPtr(head_landmarks2));

  auto out = flow->SyncPredict(input);

  EXPECT_EQ(out->error_code_, 0);
  EXPECT_EQ(out->datas_.size(), static_cast<size_t>(3));

  std::vector<int> face_id, head_id;
  for (int i = 0; i < 2; ++i) {
    auto &data = out->datas_[i];
    auto pdata = std::static_pointer_cast<BaseDataVector>(data);
    ASSERT_TRUE(!pdata->datas_.empty());
    for (const auto &element : pdata->datas_) {
      auto bbox = std::static_pointer_cast<BBox>(element);
      if (pdata->name_ == "merged_face_box") {
        face_id.emplace_back(bbox->id_);
      } else if (pdata->name_ == "merged_head_box") {
        head_id.emplace_back(bbox->id_);
      }
    }
  }
  EXPECT_EQ(face_id.size(), static_cast<size_t>(3));
  EXPECT_EQ(head_id.size(), static_cast<size_t>(2));
}

TEST_F(XStreamSDKTest, State_Check) {
  InputDataPtr input(new InputData());
  auto face_box_list = std::make_shared<BaseDataVector>();
  face_box_list->name_ = "face_box";
  input->datas_.push_back(BaseDataPtr(face_box_list));

  auto disappeared_face_id_list = std::make_shared<BaseDataVector>();
  disappeared_face_id_list->name_ = "disappeared_face_id";
  input->datas_.push_back(BaseDataPtr(disappeared_face_id_list));

  auto head_box_list = std::make_shared<BaseDataVector>();
  head_box_list->name_ = "head_box";
  input->datas_.push_back(BaseDataPtr(head_box_list));

  auto disappeared_head_id_list = std::make_shared<BaseDataVector>();
  disappeared_head_id_list->name_ = "disappeared_head_id";
  input->datas_.push_back(BaseDataPtr(disappeared_head_id_list));

  auto rgb_lmk = std::make_shared<BaseDataVector>();
  rgb_lmk->name_ = "rgb_lmk";
  input->datas_.push_back(BaseDataPtr(rgb_lmk));

  auto nir_lmk = std::make_shared<BaseDataVector>();
  nir_lmk->name_ = "nir_lmk";
  input->datas_.push_back(BaseDataPtr(nir_lmk));

  auto face_box1 = std::make_shared<BBox>();
  face_box1->type_ = "BBox";
  face_box1->id_ = 10;
  face_box1->x1_ = 100;
  face_box1->y1_ = 100;
  face_box1->x2_ = 300;
  face_box1->y2_ = 200;

  auto face_box2 = std::make_shared<BBox>();
  face_box2->state_ = xstream::DataState::FILTERED;
  face_box2->type_ = "BBox";
  face_box2->id_ = 12;
  face_box2->x1_ = 400;
  face_box2->y1_ = 100;
  face_box2->x2_ = 500;
  face_box2->y2_ = 200;

  auto face_box3 = std::make_shared<BBox>();
  face_box3->type_ = "BBox";
  face_box3->id_ = 14;
  face_box3->x1_ = 700;
  face_box3->y1_ = 100;
  face_box3->x2_ = 800;
  face_box3->y2_ = 200;

  face_box_list->datas_.emplace_back(face_box1);
  face_box_list->datas_.emplace_back(face_box2);
  face_box_list->datas_.emplace_back(face_box3);

  auto head_box1 = std::make_shared<BBox>();
  head_box1->type_ = "BBox";
  head_box1->id_ = 11;
  head_box1->x1_ = 710;
  head_box1->y1_ = 90;
  head_box1->x2_ = 790;
  head_box1->y2_ = 200;

  auto head_box2 = std::make_shared<BBox>();
  head_box2->state_ = xstream::DataState::FILTERED;
  head_box2->type_ = "BBox";
  head_box2->id_ = 13;
  head_box2->x1_ = 90;
  head_box2->y1_ = 110;
  head_box2->x2_ = 270;
  head_box2->y2_ = 200;

  head_box_list->datas_.emplace_back(BaseDataPtr(head_box1));
  head_box_list->datas_.emplace_back(BaseDataPtr(head_box2));

  auto face_track_id1 = std::make_shared<XStreamUint32>();
  face_track_id1->value_ = 1;
  auto face_track_id2 = std::make_shared<XStreamUint32>();
  face_track_id2->value_ = 2;
  disappeared_face_id_list->datas_.emplace_back(BaseDataPtr(face_track_id1));
  disappeared_face_id_list->datas_.emplace_back(BaseDataPtr(face_track_id2));

  auto head_track_id1 = std::make_shared<XStreamUint32>();
  head_track_id1->value_ = 3;
  auto head_track_id2 = std::make_shared<XStreamUint32>();
  head_track_id2->value_ = 4;

  disappeared_head_id_list->datas_.emplace_back(BaseDataPtr(head_track_id1));
  disappeared_head_id_list->datas_.emplace_back(BaseDataPtr(head_track_id2));

  auto face_landmarks1 = std::make_shared<Landmarks>();
  face_landmarks1->type_ = "Landmarks";
  face_landmarks1->values_.resize(5);
  face_landmarks1->values_[0].x_ = 10;
  face_landmarks1->values_[0].y_ = 10;
  face_landmarks1->values_[1].x_ = 15;
  face_landmarks1->values_[1].y_ = 15;
  face_landmarks1->values_[2].x_ = 20;
  face_landmarks1->values_[2].y_ = 20;
  face_landmarks1->values_[3].x_ = 25;
  face_landmarks1->values_[3].y_ = 25;
  face_landmarks1->values_[4].x_ = 30;
  face_landmarks1->values_[4].y_ = 30;

  auto face_landmarks2 = std::make_shared<Landmarks>();
  face_landmarks2->type_ = "Landmarks";
  face_landmarks2->values_.resize(5);
  face_landmarks2->values_[0].x_ = 35;
  face_landmarks2->values_[0].y_ = 35;
  face_landmarks2->values_[1].x_ = 40;
  face_landmarks2->values_[1].y_ = 40;
  face_landmarks2->values_[2].x_ = 45;
  face_landmarks2->values_[2].y_ = 45;
  face_landmarks2->values_[3].x_ = 50;
  face_landmarks2->values_[3].y_ = 50;
  face_landmarks2->values_[4].x_ = 55;
  face_landmarks2->values_[4].y_ = 55;

  rgb_lmk->datas_.emplace_back(face_landmarks1);
  rgb_lmk->datas_.emplace_back(face_landmarks2);

  auto head_landmarks1 = std::make_shared<Landmarks>();
  head_landmarks1->type_ = "Landmarks";
  head_landmarks1->values_.resize(5);
  head_landmarks1->values_[0].x_ = 10;
  head_landmarks1->values_[0].y_ = 10;
  head_landmarks1->values_[1].x_ = 15;
  head_landmarks1->values_[1].y_ = 15;
  head_landmarks1->values_[2].x_ = 20;
  head_landmarks1->values_[2].y_ = 20;
  head_landmarks1->values_[3].x_ = 25;
  head_landmarks1->values_[3].y_ = 25;
  head_landmarks1->values_[4].x_ = 30;
  head_landmarks1->values_[4].y_ = 30;

  auto head_landmarks2 = std::make_shared<Landmarks>();
  head_landmarks2->type_ = "Landmarks";
  head_landmarks2->values_.resize(5);
  head_landmarks2->values_[0].x_ = 35;
  head_landmarks2->values_[0].y_ = 35;
  head_landmarks2->values_[1].x_ = 40;
  head_landmarks2->values_[1].y_ = 40;
  head_landmarks2->values_[2].x_ = 45;
  head_landmarks2->values_[2].y_ = 45;
  head_landmarks2->values_[3].x_ = 50;
  head_landmarks2->values_[3].y_ = 50;
  head_landmarks2->values_[4].x_ = 55;
  head_landmarks2->values_[4].y_ = 55;

  nir_lmk->datas_.emplace_back(BaseDataPtr(head_landmarks1));
  nir_lmk->datas_.emplace_back(BaseDataPtr(head_landmarks2));

  auto out = flow->SyncPredict(input);

  EXPECT_EQ(out->error_code_, 0);
  EXPECT_EQ(out->datas_.size(), static_cast<size_t>(3));

  std::vector<int> face_id, head_id;
  for (int i = 0; i < 2; ++i) {
    auto &data = out->datas_[i];
    auto pdata = std::static_pointer_cast<BaseDataVector>(data);
    ASSERT_TRUE(!pdata->datas_.empty());
    for (const auto &element : pdata->datas_) {
      auto bbox = std::static_pointer_cast<BBox>(element);
      if (pdata->name_ == "merged_face_box") {
        if (bbox->state_ == xstream::DataState::VALID) {
          face_id.emplace_back(bbox->id_);
        } else {
          face_id.emplace_back(-1);
        }
      } else if (pdata->name_ == "merged_head_box") {
        if (bbox->state_ == xstream::DataState::VALID) {
          head_id.emplace_back(bbox->id_);
        } else {
          head_id.emplace_back(-1);
        }
      }
    }
  }
  EXPECT_EQ(face_id.size(), static_cast<size_t>(3));
  EXPECT_EQ(head_id.size(), static_cast<size_t>(2));
  EXPECT_EQ(-1, head_id[1]);
  EXPECT_EQ(-1, face_id[1]);
}

class PassThroughDisableParam : public xstream::InputParam {
 public:
  explicit PassThroughDisableParam(const std::string &module_name)
      : xstream::InputParam(module_name) {}
  std::string Format() override { return "pass-through"; }
};

TEST_F(XStreamSDKTest, PassThrough) {
  InputDataPtr input(new InputData());
  auto face_box_list = std::make_shared<BaseDataVector>();
  face_box_list->name_ = "face_box";
  input->datas_.push_back(BaseDataPtr(face_box_list));

  auto disappeared_face_id_list = std::make_shared<BaseDataVector>();
  disappeared_face_id_list->name_ = "disappeared_face_id";
  input->datas_.push_back(BaseDataPtr(disappeared_face_id_list));

  auto head_box_list = std::make_shared<BaseData>();
  head_box_list->name_ = "head_box";
  head_box_list->state_ = xstream::DataState::INVALID;
  input->datas_.push_back(BaseDataPtr(head_box_list));

  auto disappeared_head_id_list = std::make_shared<BaseData>();
  disappeared_head_id_list->name_ = "disappeared_head_id";
  disappeared_head_id_list->state_ = xstream::DataState::INVALID;
  input->datas_.push_back(BaseDataPtr(disappeared_head_id_list));

  auto rgb_lmk = std::make_shared<BaseDataVector>();
  rgb_lmk->name_ = "rgb_lmk";
  input->datas_.push_back(BaseDataPtr(rgb_lmk));

  auto nir_lmk = std::make_shared<BaseDataVector>();
  nir_lmk->name_ = "nir_lmk";
  input->datas_.push_back(BaseDataPtr(nir_lmk));

  auto param = new PassThroughDisableParam("merge_method");
  input->params_.emplace_back(xstream::InputParamPtr(param));
  auto out = flow->SyncPredict(input);
  std::cout << out->error_detail_ << std::endl;
  EXPECT_EQ(out->error_code_, 0);
  EXPECT_EQ(out->datas_.size(), static_cast<size_t>(3));
  EXPECT_EQ(out->datas_[0].get(), face_box_list.get());
  EXPECT_EQ(out->datas_[2].get(), disappeared_face_id_list.get());
}

TEST_F(XStreamSDKTest, DISABLED_Cam_Calib_Test) {
  InputDataPtr input(new InputData());
  auto face_box_list = std::make_shared<BaseDataVector>();
  face_box_list->name_ = "face_box";
  input->datas_.push_back(BaseDataPtr(face_box_list));

  auto disappeared_face_id_list = std::make_shared<BaseDataVector>();
  disappeared_face_id_list->name_ = "disappeared_face_id";
  input->datas_.push_back(BaseDataPtr(disappeared_face_id_list));

  auto head_box_list = std::make_shared<BaseDataVector>();
  head_box_list->name_ = "head_box";
  input->datas_.push_back(BaseDataPtr(head_box_list));

  auto disappeared_head_id_list = std::make_shared<BaseDataVector>();
  disappeared_head_id_list->name_ = "disappeared_head_id";
  input->datas_.push_back(BaseDataPtr(disappeared_head_id_list));

  auto rgb_lmk = std::make_shared<BaseDataVector>();
  rgb_lmk->name_ = "rgb_lmk";
  input->datas_.push_back(BaseDataPtr(rgb_lmk));

  auto nir_lmk = std::make_shared<BaseDataVector>();
  nir_lmk->name_ = "nir_lmk";
  input->datas_.push_back(BaseDataPtr(nir_lmk));

  std::ifstream ifs("/userdata/merge_test.txt", std::istream::in);
  char rgb_image_name[256], nir_image_name[256];
  int target_num;
  std::vector<std::pair<BBox, BBox>> ground_truth_bbox;
  while (!ifs.eof()) {
    ifs >> rgb_image_name >> nir_image_name >> target_num;
    for (int i = 0; i < target_num; ++i) {
      float rgb_x1, rgb_y1, rgb_x2, rgb_y2;
      ifs >> rgb_x1 >> rgb_y1 >> rgb_x2 >> rgb_y2;
      auto rgb_face_box = std::make_shared<BBox>();
      rgb_face_box->type_ = "BBox";
      rgb_face_box->id_ = i;
      rgb_face_box->x1_ = rgb_x1;
      rgb_face_box->y1_ = rgb_y1;
      rgb_face_box->x2_ = rgb_x2;
      rgb_face_box->y2_ = rgb_y2;
      face_box_list->datas_.emplace_back(BaseDataPtr(rgb_face_box));
      float nir_x1, nir_y1, nir_x2, nir_y2;
      ifs >> nir_x1 >> nir_y1 >> nir_x2 >> nir_y2;
      auto nir_face_box = std::make_shared<BBox>();
      nir_face_box->type_ = "BBox";
      nir_face_box->id_ = i + 100;
      nir_face_box->x1_ = nir_x1;
      nir_face_box->y1_ = nir_y1;
      nir_face_box->x2_ = nir_x2;
      nir_face_box->y2_ = nir_y2;
      head_box_list->datas_.emplace_back(nir_face_box);
      ground_truth_bbox.emplace_back(
          std::make_pair(*rgb_face_box, *nir_face_box));
    }
    auto face_track_id1 = std::make_shared<XStreamUint32>();
    face_track_id1->value_ = 1000;
    auto face_track_id2 = std::make_shared<XStreamUint32>();
    face_track_id2->value_ = 2000;
    disappeared_face_id_list->datas_.emplace_back(face_track_id1);
    disappeared_face_id_list->datas_.emplace_back(face_track_id2);

    auto head_track_id1 = std::make_shared<XStreamUint32>();
    head_track_id1->value_ = 3000;
    auto head_track_id2 = std::make_shared<XStreamUint32>();
    head_track_id2->value_ = 4000;

    disappeared_head_id_list->datas_.emplace_back(head_track_id1);
    disappeared_head_id_list->datas_.emplace_back(head_track_id2);

    auto face_landmarks1 = std::make_shared<Landmarks>();
    face_landmarks1->type_ = "Landmarks";
    face_landmarks1->values_.resize(5);
    face_landmarks1->values_[0].x_ = 10;
    face_landmarks1->values_[0].y_ = 10;
    face_landmarks1->values_[1].x_ = 15;
    face_landmarks1->values_[1].y_ = 15;
    face_landmarks1->values_[2].x_ = 20;
    face_landmarks1->values_[2].y_ = 20;
    face_landmarks1->values_[3].x_ = 25;
    face_landmarks1->values_[3].y_ = 25;
    face_landmarks1->values_[4].x_ = 30;
    face_landmarks1->values_[4].y_ = 30;

    auto face_landmarks2 = std::make_shared<Landmarks>();
    face_landmarks2->type_ = "Landmarks";
    face_landmarks2->values_.resize(5);
    face_landmarks2->values_[0].x_ = 35;
    face_landmarks2->values_[0].y_ = 35;
    face_landmarks2->values_[1].x_ = 40;
    face_landmarks2->values_[1].y_ = 40;
    face_landmarks2->values_[2].x_ = 45;
    face_landmarks2->values_[2].y_ = 45;
    face_landmarks2->values_[3].x_ = 50;
    face_landmarks2->values_[3].y_ = 50;
    face_landmarks2->values_[4].x_ = 55;
    face_landmarks2->values_[4].y_ = 55;

    rgb_lmk->datas_.emplace_back(BaseDataPtr(face_landmarks1));
    rgb_lmk->datas_.emplace_back(BaseDataPtr(face_landmarks2));

    auto head_landmarks1 = std::make_shared<Landmarks>();
    head_landmarks1->type_ = "Landmarks";
    head_landmarks1->values_.resize(5);
    head_landmarks1->values_[0].x_ = 10;
    head_landmarks1->values_[0].y_ = 10;
    head_landmarks1->values_[1].x_ = 15;
    head_landmarks1->values_[1].y_ = 15;
    head_landmarks1->values_[2].x_ = 20;
    head_landmarks1->values_[2].y_ = 20;
    head_landmarks1->values_[3].x_ = 25;
    head_landmarks1->values_[3].y_ = 25;
    head_landmarks1->values_[4].x_ = 30;
    head_landmarks1->values_[4].y_ = 30;

    auto head_landmarks2 = std::make_shared<Landmarks>();
    head_landmarks2->type_ = "Landmarks";
    head_landmarks2->values_.resize(5);
    head_landmarks2->values_[0].x_ = 35;
    head_landmarks2->values_[0].y_ = 35;
    head_landmarks2->values_[1].x_ = 40;
    head_landmarks2->values_[1].y_ = 40;
    head_landmarks2->values_[2].x_ = 45;
    head_landmarks2->values_[2].y_ = 45;
    head_landmarks2->values_[3].x_ = 50;
    head_landmarks2->values_[3].y_ = 50;
    head_landmarks2->values_[4].x_ = 55;
    head_landmarks2->values_[4].y_ = 55;

    nir_lmk->datas_.emplace_back(BaseDataPtr(head_landmarks1));
    nir_lmk->datas_.emplace_back(BaseDataPtr(head_landmarks2));

    auto out = flow->SyncPredict(input);

    EXPECT_EQ(out->error_code_, 0);
    EXPECT_EQ(out->datas_.size(), static_cast<size_t>(3));

    std::vector<BBox> face_box_s, head_box_s;
    for (int i = 0; i < 2; ++i) {
      auto &data = out->datas_[i];
      auto pdata = std::static_pointer_cast<BaseDataVector>(data);
      ASSERT_TRUE(!pdata->datas_.empty());
      for (const auto &element : pdata->datas_) {
        auto bbox = std::static_pointer_cast<BBox>(element);
        if (pdata->name_ == "merged_face_box") {
          face_box_s.emplace_back(*bbox);
        } else if (pdata->name_ == "merged_head_box") {
          head_box_s.emplace_back(*bbox);
        }
      }
    }
    auto ret = CheckResult(face_box_s, head_box_s, ground_truth_bbox);
    EXPECT_EQ(ret, 0);
  }
}
