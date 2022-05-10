/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: example_do_pose_lmk.cpp
 * @Brief:
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-04-15 14:27:05
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-04-15 15:18:10
 */

#include <stdint.h>

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "cnn_method/cnn_method.h"
#include "cnn_method/util/util.h"
#include "fasterrcnn_method/fasterrcnn_method.h"
#include "opencv2/opencv.hpp"
#include "xstream/method.h"
#include "xstream/method_factory.h"
#include "xstream/vision_type.h"
#include "xstream/xstream_sdk.h"

// #include "./vio_wrapper_global.h"


typedef std::shared_ptr<xstream::ImageFrame> ImageFramePtr;
using xstream::BaseDataPtr;

struct PyramidResult {
  img_info_t result_info;
};

static void Usage() {
  std::cout << "./example do_fb_rect_cnn "
               "[pose_lmk|age_gender|anti_spf|face_quality|"
               "vehicle_type|vehicle_color|plate_num]"
               "xstream_cfg_file fb_cfg img_list out_file"
            << std::endl;
}

void DumpPoseLmk(std::vector<BaseDataPtr> &, std::ostream &, std::string);
void DumpAgeGender(std::vector<BaseDataPtr> &, std::ostream &, std::string);
void DumpAntiSpf(std::vector<BaseDataPtr> &, std::ostream &, std::string);
void DumpFaceQuality(std::vector<BaseDataPtr> &, std::ostream &, std::string);
void DumpVehicleColor(std::vector<BaseDataPtr> &, std::ostream &, std::string);
void DumpVehicleType(std::vector<BaseDataPtr> &, std::ostream &, std::string);
void DumpPlateNum(std::vector<BaseDataPtr> &, std::ostream &, std::string);
void DumpVehiclePhone(std::vector<BaseDataPtr> &, std::ostream &, std::string);
void DumpVehicleBelt(std::vector<BaseDataPtr> &, std::ostream &, std::string);
void DumpPersonAttrs(std::vector<BaseDataPtr> &, std::ostream &, std::string);

int DoFbRectCnn(int argc, char **argv) {
  if (argc < 6) {
    Usage();
    return 1;
  }
  std::string model_name(argv[1]);
  std::string cfg_file(argv[2]);
  std::string fb_cfg(argv[3]);
  std::string img_list(argv[4]);
  std::string rlt_put_file = argv[5];

  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();
  flow->SetConfig("config_file", cfg_file.c_str());
  flow->SetConfig("profiler", "on");
  flow->SetConfig("profiler_file", "./profiler.txt");
  flow->Init();
  HbVioFbWrapperGlobal fb_handle(fb_cfg);
  fb_handle.Init();
  std::ifstream img_list_file(img_list);
  std::string img_path;
  std::string gt_line;
  std::ofstream output(rlt_put_file, std::ios::out);
  float x1, y1, x2, y2;
  while (getline(img_list_file, gt_line)) {
    std::istringstream gt(gt_line);
    gt >> img_path;
    gt >> x1 >> y1 >> x2 >> y2;
    if (x1 < 0 || x2 >= 1920 || y1 < 0 || y2 >= 1080) continue;

    auto xstream_rois = std::make_shared<xstream::BaseDataVector>();
    xstream_rois->name_ = "face_box";
    auto xstream_roi = std::make_shared<xstream::BBox>();
    xstream_roi->x1_ = x1;
    xstream_roi->y1_ = y1;
    xstream_roi->x2_ = x2;
    xstream_roi->y2_ = y2;
    xstream_rois->datas_.push_back(xstream_roi);
    uint32_t effective_w, effective_h;
    auto py_img = fb_handle.GetImgInfo(img_path, &effective_w, &effective_h);

    xstream::InputDataPtr inputdata(new xstream::InputData());
    py_img->name_ = "pyramid";

    inputdata->datas_.push_back(
        std::static_pointer_cast<xstream::BaseData>(xstream_rois));
    inputdata->datas_.push_back(
        std::static_pointer_cast<xstream::BaseData>(py_img));

    auto out = flow->SyncPredict(inputdata);
    if (model_name == "pose_lmk") {
      printf("the pose_lmk is \n");
      DumpPoseLmk(out->datas_, output, img_path);
    } else if (model_name == "age_gender") {
      DumpAgeGender(out->datas_, output, img_path);
    } else if (model_name == "anti_spf") {
      DumpAntiSpf(out->datas_, output, img_path);
    } else if (model_name == "face_quality") {
      DumpFaceQuality(out->datas_, output, img_path);
    } else if (model_name == "vehicle_type") {
      DumpVehicleType(out->datas_, output, img_path);
    } else if (model_name == "vehicle_color") {
      DumpVehicleColor(out->datas_, output, img_path);
    } else if (model_name == "plate_num") {
      DumpPlateNum(out->datas_, output, img_path);
    } else if (model_name == "vehicle_phone") {
      DumpVehiclePhone(out->datas_, output, img_path);
    } else if (model_name == "vehicle_belt") {
      DumpVehicleBelt(out->datas_, output, img_path);
    } else if (model_name == "person_attrs") {
      DumpPersonAttrs(out->datas_, output, img_path);
    }
    fb_handle.FreeImgInfo(py_img);
  }
  return 0;
}

void DumpPersonAttrs(std::vector<xstream::BaseDataPtr> &result,
                     std::ostream &os, std::string id) {
  os << id;

  std::vector<float> rlts;
  auto hats = std::static_pointer_cast<xstream::BaseDataVector>(result[0]);
  auto glasseds =
      std::static_pointer_cast<xstream::BaseDataVector>(result[1]);
  auto masks = std::static_pointer_cast<xstream::BaseDataVector>(result[2]);
  auto ups = std::static_pointer_cast<xstream::BaseDataVector>(result[3]);
  auto lows = std::static_pointer_cast<xstream::BaseDataVector>(result[4]);
  auto bags = std::static_pointer_cast<xstream::BaseDataVector>(result[5]);
  int target_size = hats->datas_.size();
  for (int target_idx = 0; target_idx < target_size; target_idx++) {
    auto hat = std::static_pointer_cast<xstream::Attribute_<int>>(
        hats->datas_[target_idx]);
    auto glassed = std::static_pointer_cast<xstream::Attribute_<int>>(
        glasseds->datas_[target_idx]);
    auto mask = std::static_pointer_cast<xstream::Attribute_<int>>(
        masks->datas_[target_idx]);
    auto up = std::static_pointer_cast<xstream::Attribute_<int>>(
        ups->datas_[target_idx]);
    auto low = std::static_pointer_cast<xstream::Attribute_<int>>(
        lows->datas_[target_idx]);
    auto bag = std::static_pointer_cast<xstream::Attribute_<int>>(
        bags->datas_[target_idx]);

    if (hat->state_ != xstream::DataState::VALID) {
      os << " -1 -1 -1 -1 -1 -1";
      continue;
    }
    // printf("result %d %d\n", hat->value, glassed->value);
    printf("the person attr value is %d %d %d %d %d %d\n", hat->value_,
           glassed->value_, mask->value_, up->value_, low->value_, bag->value_);
    os << "hat:" << hat->value_;
    os << "glassed:" << glassed->value_;
    os << "mask:" << mask->value_;
    os << "up:" << up->value_;
    os << "low:" << low->value_;
    os << "bag:" << bag->value_;
  }
  os << std::endl;
}

void DumpVehicleBelt(std::vector<xstream::BaseDataPtr> &result,
                     std::ostream &os, std::string id) {
  os << id;
  std::vector<float> rlts;
  auto types = std::static_pointer_cast<xstream::BaseDataVector>(result[0]);
  int target_size = types->datas_.size();
  for (int target_idx = 0; target_idx < target_size; target_idx++) {
    auto type = std::static_pointer_cast<xstream::Attribute_<int>>(
        types->datas_[target_idx]);

    if (type->state_ == xstream::DataState::VALID) {
      rlts.push_back(type->value_);
      printf("the vehicle belt value is %d\n", type->value_);
    }
  }
  for (const auto &value : rlts) {
    os << " " << value;
  }
  os << std::endl;
}

void DumpVehiclePhone(std::vector<xstream::BaseDataPtr> &result,
                      std::ostream &os, std::string id) {
  os << id;
  std::vector<float> rlts;
  auto types = std::static_pointer_cast<xstream::BaseDataVector>(result[0]);
  int target_size = types->datas_.size();
  for (int target_idx = 0; target_idx < target_size; target_idx++) {
    auto type = std::static_pointer_cast<xstream::Attribute_<int>>(
        types->datas_[target_idx]);

    if (type->state_ == xstream::DataState::VALID) {
      rlts.push_back(type->value_);
      printf("the vehicle phone value is %d\n", type->value_);
    }
  }
  for (const auto &value : rlts) {
    os << " " << value;
  }
  os << std::endl;
}

void DumpPlateNum(std::vector<xstream::BaseDataPtr> &result, std::ostream &os,
                  std::string id) {
  os << id;
  std::vector<std::string> rlts;
  auto types = std::static_pointer_cast<xstream::BaseDataVector>(result[0]);
  int target_size = types->datas_.size();
  for (int target_idx = 0; target_idx < target_size; target_idx++) {
    auto type = std::static_pointer_cast<xstream::Attribute_<std::vector<int>>>(
        types->datas_[target_idx]);

    if (type->state_ == xstream::DataState::VALID) {
      std::string temp;
      for (auto &num : type->value_) {
        temp += num;
      }
      rlts.push_back(temp);
      printf("the plate num value is %s\n", temp.c_str());
    } else {
      printf("the plate num is invaild\n");
    }
  }
  for (const auto &value : rlts) {
    os << " " << value;
  }
  os << std::endl;
}

void DumpVehicleColor(std::vector<xstream::BaseDataPtr> &result,
                      std::ostream &os, std::string id) {
  os << id;
  std::vector<float> rlts;
  auto types = std::static_pointer_cast<xstream::BaseDataVector>(result[0]);
  int target_size = types->datas_.size();
  for (int target_idx = 0; target_idx < target_size; target_idx++) {
    auto type = std::static_pointer_cast<xstream::Attribute_<int>>(
        types->datas_[target_idx]);

    if (type->state_ == xstream::DataState::VALID) {
      rlts.push_back(type->value_);
      printf("the vehicle color value is %d\n", type->value_);
    }
  }
  for (const auto &value : rlts) {
    os << " " << value;
  }
  os << std::endl;
}

void DumpVehicleType(std::vector<xstream::BaseDataPtr> &result,
                     std::ostream &os, std::string id) {
  os << id;
  std::vector<float> rlts;
  auto types = std::static_pointer_cast<xstream::BaseDataVector>(result[0]);
  int target_size = types->datas_.size();
  for (int target_idx = 0; target_idx < target_size; target_idx++) {
    auto type = std::static_pointer_cast<xstream::Attribute_<int>>(
        types->datas_[target_idx]);

    if (type->state_ == xstream::DataState::VALID) {
      rlts.push_back(type->value_);
      printf("the vehicle type value is %d\n", type->value_);
    }
  }
  for (const auto &value : rlts) {
    os << " " << value;
  }
  os << std::endl;
}

void DumpAntiSpf(std::vector<xstream::BaseDataPtr> &result, std::ostream &os,
                 std::string id) {
  os << id;
  std::vector<float> rlts;
  auto anti_spfs =
      std::static_pointer_cast<xstream::BaseDataVector>(result[0]);
  int target_size = anti_spfs->datas_.size();
  for (int target_idx = 0; target_idx < target_size; target_idx++) {
    auto anti_spf = std::static_pointer_cast<xstream::Attribute_<int>>(
        anti_spfs->datas_[target_idx]);
    if (anti_spf->state_ == xstream::DataState::VALID) {
      rlts.push_back(anti_spf->score_);
    }
  }
  for (const auto &value : rlts) {
    os << " " << value;
  }
  os << std::endl;
}

void DumpAgeGender(std::vector<xstream::BaseDataPtr> &result,
                   std::ostream &os,
                   std::string id) {
  os << id;
  auto ages = std::static_pointer_cast<xstream::BaseDataVector>(result[0]);
  auto genders = std::static_pointer_cast<xstream::BaseDataVector>(result[1]);
  int target_size = ages->datas_.size();
  for (int target_idx = 0; target_idx < target_size; target_idx++) {
    auto age = std::static_pointer_cast<xstream::Age>(ages->datas_[target_idx]);
    auto gender =
        std::static_pointer_cast<xstream::Gender>(genders->datas_[target_idx]);
    if (age->state_ != xstream::DataState::VALID) {
      os << " -1 -1 -1";
      continue;
    }
    os << " " << age->min_ << " " << age->max_;
    os << " " << gender->value_;
  }
  os << std::endl;
}

void DumpPoseLmk(std::vector<xstream::BaseDataPtr> &result, std::ostream &os,
                 std::string id) {
  os << id;
  auto lmks = std::static_pointer_cast<xstream::BaseDataVector>(result[0]);
  auto poses = std::static_pointer_cast<xstream::BaseDataVector>(result[1]);
  int target_size = lmks->datas_.size();
  for (int target_idx = 0; target_idx < target_size; target_idx++) {
    auto lmk =
        std::static_pointer_cast<xstream::Landmarks>(lmks->datas_[target_idx]);
    auto pose =
        std::static_pointer_cast<xstream::Pose3D>(poses->datas_[target_idx]);
    if (lmk->state_ != xstream::DataState::VALID) {
      continue;
    }
    for (auto &point : lmk->values_) {
      os << " " << point.x_ << " " << point.y_;
    }
    os << " " << pose->pitch_ << " " << pose->yaw_ << " " << pose->roll_;
  }
  os << std::endl;
}
void DumpFaceQuality(std::vector<BaseDataPtr> &result,
                     std::ostream &os,
                     std::string id) {
  os << id;
  auto rois = std::static_pointer_cast<xstream::BaseDataVector>(result[0]);
  int target_size = rois->datas_.size();
  for (int target_idx = 0; target_idx < target_size; target_idx++) {
    auto roi =
        std::static_pointer_cast<xstream::BBox>(rois->datas_[target_idx]);
    os << " " << roi->x1_ << " " << roi->y1_ << " " << roi->x2_
       << " " << roi->y2_;
    for (size_t attr_idx = 1; attr_idx < result.size(); attr_idx++) {
      auto attrs =
          std::static_pointer_cast<xstream::BaseDataVector>(result[attr_idx]);
      auto attr = std::static_pointer_cast<
          xstream::XStreamData<xstream::Attribute<int>>>(
          attrs->datas_[target_idx]);
      if (attr->state_ == xstream::DataState::VALID) {
        if (attr_idx == 2) {
          os << " " << attr->value_;
        } else {
          os << " " << attr->score_;
        }
      } else {
        os << " -1.0";
      }
    }
  }
  os << std::endl;
}
