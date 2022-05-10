/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     gtest of Grading Method
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.21
 */

#include <cassert>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <thread>

#include "grading_method/weight_grading.h"
#include "gtest/gtest.h"
#include "xstream/vision_type.h"
#include "xstream/xstream_world.h"
class XStreamGradingMethodTest : public ::testing::Test {
 public:
  XStreamGradingMethodTest() = default;

 protected:
  void SetUp() override {}
  void TearDown() override {}
};

class GradingMethodParam : public xstream::InputParam {
 public:
  GradingMethodParam(std::string method_name, std::string json_config)
      : InputParam(std::move(method_name)) {
    content_ = std::move(json_config);
    is_json_format_ = true;
  }
  std::string Format() override { return content_; };

 private:
  std::string content_ = "";
};

TEST_F(XStreamGradingMethodTest, GradingTest) {
  using xstream::BaseData;
  using xstream::BaseDataPtr;
  using xstream::BaseDataVector;
  using xstream::InputData;
  using xstream::InputDataPtr;

  class Callback {
   public:
    Callback() = default;
    ~Callback() = default;
    std::vector<float> res_;

    void OnCallback(const xstream::OutputDataPtr &output) {
      using xstream::BaseDataVector;
      for (const auto &data : output->datas_) {
        if (data->error_code_ < 0) {
          continue;
        }
        auto *pdata = dynamic_cast<BaseDataVector *>(data.get());
        for (const auto &item : pdata->datas_) {
          assert("Number" == item->type_);
          auto select_score =
              std::static_pointer_cast<xstream::XStreamFloat>(item);
          res_.push_back(select_score->value);
        }
      }
      EXPECT_EQ(res_[0], 0.447049975f);
      EXPECT_EQ(res_[1], 0.451808333f);
    }
  };

  // 初始化sdk
  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();
  Callback callback;
  flow->SetCallback(
      std::bind(&Callback::OnCallback, &callback, std::placeholders::_1));
  flow->SetConfig("config_file", "./config/grading.json");
  flow->Init();

  auto version = flow->GetVersion("grading_example");
  EXPECT_EQ("0.0.12", version);

  xstream::InputParamPtr method_param(
      new GradingMethodParam("grading_example",
                             "{\n"
                             "  \"grading_type\": \"weight_grading\",\n"
                             "  \"size_min\": 40,\n"
                             "  \"size_max\": 200,\n"
                             "  \"size_inflexion\": 80,\n"
                             "  \"size_weight\": 0.2,\n"
                             "  \"pose_weight\": 0.3,\n"
                             "  \"lmk_weight\": 0.3,\n"
                             "  \"quality_weight\": 0.2\n"
                             "}"));
  flow->UpdateConfig("grading_example", method_param);
  auto cfg_jv = flow->GetConfig("grading_example");
  auto cfg_content = cfg_jv->Format();

  auto cfg_gt =
      "{\n\t\"grading_type\" : \"weight_grading\","
      "\n\t\"lmk_weight\" : 0.29999999999999999,"
      "\n\t\"pose_weight\" : 0.29999999999999999,"
      "\n\t\"quality_weight\" : 0.20000000000000001,"
      "\n\t\"size_inflexion\" : 80,"
      "\n\t\"size_max\" : 200,"
      "\n\t\"size_min\" : 40,"
      "\n\t\"size_weight\" : 0.20000000000000001\n}\n";
  EXPECT_EQ(cfg_content, cfg_gt);

  InputDataPtr inputdata(new InputData());
  // construct the box_list and put it into datas_[0]
  std::shared_ptr<BaseDataVector> box_list(new BaseDataVector());
  box_list->name_ = "box_list";

  std::shared_ptr<xstream::BBox> bbox1(new xstream::BBox());
  bbox1->type_ = "BBox";
  bbox1->x1_ = 861;
  bbox1->y1_ = 995;
  bbox1->x2_ = 917;
  bbox1->y2_ = 1063;
  bbox1->score_ = 1.084960;
  std::shared_ptr<xstream::BBox> bbox2(new xstream::BBox());
  bbox2->type_ = "BBox";
  bbox2->x1_ = 1000;
  bbox2->y1_ = 140;
  bbox2->x2_ = 1061;
  bbox2->y2_ = 202;
  bbox2->score_ = 0.9901622;
  box_list->datas_.push_back(bbox1);
  box_list->datas_.push_back(bbox2);
  inputdata->datas_.push_back(box_list);

  // construct the pose_3d_list and put it into datas_[1]
  std::shared_ptr<BaseDataVector> pose_3d_list(new BaseDataVector());
  pose_3d_list->name_ = "pose_3d_list";

  std::shared_ptr<xstream::Pose3D> pose1(new xstream::Pose3D());
  pose1->pitch_ = -14.5738907;
  pose1->yaw_ = 19.8864212;
  pose1->roll_ = -6.61847687;

  std::shared_ptr<xstream::Pose3D> pose2(new xstream::Pose3D());
  pose2->pitch_ = -15.0597382;
  pose2->yaw_ = 2.414505;
  pose2->roll_ = -1.443367;
  pose_3d_list->datas_.push_back(pose1);
  pose_3d_list->datas_.push_back(pose2);
  inputdata->datas_.push_back(pose_3d_list);

  // construct the land_mark_list and put it into datas_[2]
  std::shared_ptr<BaseDataVector> land_mark_list(new BaseDataVector());
  land_mark_list->name_ = "land_mark_list";

  std::shared_ptr<xstream::Landmarks> lmk1(new xstream::Landmarks());
  xstream::Point point0(881, 1021, 13);
  xstream::Point point1(907, 1079, 12);
  xstream::Point point2(897, 1036, 13);
  xstream::Point point3(884, 1047, 12);
  xstream::Point point4(905, 1045, 13);
  lmk1->values_ = {point0, point1, point2, point3, point4};

  std::shared_ptr<xstream::Landmarks> lmk2(new xstream::Landmarks());
  xstream::Point point5(1020, 156, 9);
  xstream::Point point6(1047, 159, 4);
  xstream::Point point7(1034, 169, 13);
  xstream::Point point8(1020, 180, 9);
  xstream::Point point9(1041, 182, 9);
  lmk2->values_ = {point5, point6, point7, point8, point9};

  land_mark_list->datas_.push_back(lmk1);
  land_mark_list->datas_.push_back(lmk2);
  inputdata->datas_.push_back(land_mark_list);

  // construct the quality_list and put it into datas_[3]
  std::shared_ptr<BaseDataVector> quality_list(new BaseDataVector());
  quality_list->name_ = "quality_list";
  std::shared_ptr<xstream::Quality> quality1(new xstream::Quality());
  quality1->value_ = 0;

  std::shared_ptr<xstream::Quality> quality2(new xstream::Quality());
  quality2->value_ = 0;

  quality_list->datas_.push_back(quality1);
  quality_list->datas_.push_back(quality2);
  inputdata->datas_.push_back(quality_list);

  // sync GradingMethod
  auto out = flow->SyncPredict(inputdata);
  callback.OnCallback(out);

  // aync GradingMethod
  flow->AsyncPredict(inputdata);
  sleep(1);

  delete flow;
}

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
