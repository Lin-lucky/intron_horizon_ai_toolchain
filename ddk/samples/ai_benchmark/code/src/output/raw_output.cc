// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "output/raw_output.h"

#include <iterator>

#include "glog/logging.h"
#include "rapidjson/document.h"

DEFINE_AND_REGISTER_OUTPUT(eval, RawOutputModule)

int RawOutputModule::Init(std::string config_file, std::string config_string) {
  int ret_code = OutputModule::Init(config_file, config_string);
  if (ret_code != 0) {
    return -1;
  }

  ofs_.open(output_file_.c_str(), std::ios::out | std::ios::trunc);
  if (!ofs_.is_open()) {
    VLOG(EXAMPLE_SYSTEM) << "Open " << output_file_ << " failed";
  }

  if (!mask_file_.empty()) {
    mask_ofs_.open(mask_file_.c_str(), std::ios::out | std::ios::trunc);
    if (!ofs_.is_open()) {
      VLOG(EXAMPLE_SYSTEM) << "Open " << mask_file_ << " failed";
    }
  }

  return 0;
}

int RawOutputModule::Init(rapidjson::Document &document) { return 0; }

void RawOutputModule::Write(ImageTensor *frame, Perception *perception) {
  switch (perception->type) {
    case Perception::DET:
      WriteDetLog(frame, perception);
      break;
    case Perception::CLS:
      WriteClsLog(frame, perception);
      break;
    case Perception::SEG:
      WriteParsingLog(frame, perception);
      break;
    case Perception::MASK:
      WriteMaskLog(frame, perception);
      break;
    default: {
      VLOG(EXAMPLE_SYSTEM) << "invaild type in write raw file, the type is : "
                           << perception->type;
    }
  }
}

void RawOutputModule::WriteDetLog(ImageTensor *frame, Perception *perception) {
  std::stringstream result_log;

  result_log << std::setprecision(18)
             << "input_image_name: " << frame->image_name << " ";
  for (auto &box : perception->det) {
    result_log << box.bbox.xmin << "," << box.bbox.ymin << "," << box.bbox.xmax
               << "," << box.bbox.ymax << "," << box.score << "," << box.id
               << " ";
  }
  ofs_ << result_log.str() << std::endl;
}

void RawOutputModule::WriteMaskLog(ImageTensor *frame, Perception *perception) {
  std::stringstream result_log;

  // write mask
  mask_ofs_.write((char *)perception->mask.mask_info.data(),
                  perception->mask.mask_info.size() * sizeof(float));

  result_log << std::setprecision(18)
             << "input_image_name: " << frame->image_name << " ";
  for (auto &box : perception->mask.det_info) {
    result_log << box.bbox.xmin << "," << box.bbox.ymin << "," << box.bbox.xmax
               << "," << box.bbox.ymax << "," << box.score << "," << box.id
               << " ";
  }
  ofs_ << result_log.str() << std::endl;
}

void RawOutputModule::WriteClsLog(ImageTensor *frame, Perception *perception) {
  std::string log_str = std::string("input_image_name: ") + frame->image_name;

  for (auto cls : perception->cls) {
    if (cls.id < 0) break;
    log_str += std::string(" class_id: ") + std::to_string(cls.id) +
               std::string(" class_name: ") + cls.class_name;
  }

  printf("the str is %s\n", log_str.c_str());
  ofs_ << log_str << std::endl;
}

void RawOutputModule::WriteParsingLog(ImageTensor *frame,
                                      Perception *perception) {
  auto result_ptr = static_cast<int8_t *>(malloc(perception->seg.seg.size()));
  std::string log_str;
  int recorded_item_count = 0;
  int block_index = 0;
  for (auto block : perception->seg.seg) {
    if (log_str.length() == 0)
      log_str = std::string("input_image_name: ") + frame->image_name +
                std::string("_block_") + std::to_string(block_index) +
                std::string(" ");
    log_str += std::to_string(block) + " ";
    recorded_item_count++;
    if (recorded_item_count >= 8000) {
      recorded_item_count = 0;
      ofs_ << log_str << std::endl;
      log_str = "";
      block_index += 1;
    }
  }
  if (log_str.length() > 0) ofs_ << log_str << std::endl;

  free(result_ptr);
}

int RawOutputModule::LoadConfig(std::string &config_string) {
  rapidjson::Document document;
  document.Parse(config_string.data());

  if (document.HasParseError()) {
    VLOG(EXAMPLE_SYSTEM) << "Parsing config file failed";
    return -1;
  }

  if (document.HasMember("output_file")) {
    output_file_ = document["output_file"].GetString();
  }

  if (document.HasMember("mask_output_file")) {
    mask_file_ = document["mask_output_file"].GetString();
  }

  return 0;
}

RawOutputModule::~RawOutputModule() {
  if (ofs_.is_open()) {
    ofs_.close();
  }

  if (mask_ofs_.is_open()) {
    mask_ofs_.close();
  }
}
