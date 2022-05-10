/**
 * Copyright (c) 2020, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: xue.liang
 * @Mail: xue.liang@horizon.ai
 * @Date: 2020-12-15 20:38:52
 * @Version: v0.0.1
 * @Brief:
 * @Last Modified by: xue.liang
 * @Last Modified time: 2020-12-16 22:41:30
 */

#include "smart_plugin/key_point_convertor.h"
#include "hobotlog/hobotlog.hpp"

namespace xproto {
void KeyPointConvertor::ConverKeyPoint(xstream::OutputDataPtr xstream_out) {
  xstream::BaseDataVector *kps_list = nullptr;
  xstream::BaseDataVector *face_lmks = nullptr;
  xstream::BaseDataVector *face_box_list = nullptr;
  xstream::BaseDataVector *head_box_list = nullptr;
  xstream::BaseDataVector *body_box_list = nullptr;

  auto name_prefix = [](const std::string name) -> std::string {
    auto pos = name.find('_');
    if (pos == std::string::npos) return "";

    return name.substr(0, pos);
  };
  auto name_postfix = [](const std::string name) -> std::string {
    auto pos = name.rfind('_');
    if (pos == std::string::npos) return "";

    return name.substr(pos + 1);
  };

  for (const auto &output : xstream_out->datas_) {
    LOGD << output->name_ << ", type is " << output->type_;
    auto prefix = name_prefix(output->name_);
    auto postfix = name_postfix(output->name_);
    if (output->name_ == "kps" || output->name_ == "lowpassfilter_body_kps") {
      kps_list = dynamic_cast<xstream::BaseDataVector *>(output.get());
    }

    if (output->name_ == "body_box" || (prefix == "body" && postfix == "box")) {
      body_box_list = dynamic_cast<xstream::BaseDataVector *>(output.get());
    }

    if (output->name_ == "head_box" || (prefix == "head" && postfix == "box")) {
      head_box_list = dynamic_cast<xstream::BaseDataVector *>(output.get());
    }

    if (output->name_ == "lowpassfilter_lmk_106pts") {
      face_lmks = dynamic_cast<xstream::BaseDataVector *>(output.get());
    }

    if (output->name_ == "face_bbox_list" ||
        (prefix == "face" && postfix == "box")) {
      face_box_list = dynamic_cast<xstream::BaseDataVector *>(output.get());
    }
  }
  ConvertFaceLmk(face_lmks);
  ConvertKps(kps_list, body_box_list, head_box_list, face_lmks, face_box_list);
}

void KeyPointConvertor::ConvertFaceLmk(xstream::BaseDataVector *face_lmks) {
  if (!face_lmks) return;

  std::vector<std::shared_ptr<xstream::Point>> tmp_lmk_list;
  for (size_t i = 0; i < face_lmks->datas_.size(); ++i) {
    auto lmk = std::static_pointer_cast<
        xstream::Landmarks>(face_lmks->datas_[i]);
    if (lmk->values_.size() < 106) {
      LOGI << "ConvertFaceLmk recv face lmk less than 106, size:"
           << lmk->values_.size();
      continue;
    }
#if 0
    auto face_box =
        std::static_pointer_cast<xstream::XStreamData<xstream::BBox>>(
            face_box_list->datas_[i]);
    if (!face_box) continue;
    // 查找对应的track_id
    if (face_box->value.id == -1) {
      continue;
    }
#endif
    tmp_lmk_list.resize(lmk->values_.size());
    for (size_t j = 0; j < lmk->values_.size(); ++j) {
      auto point = std::make_shared<xstream::Point>();
      point->x_ = lmk->values_[j].x_;
      point->y_ = lmk->values_[j].y_;
      point->score_ = lmk->values_[j].score_;
      tmp_lmk_list[j] = point;
      LOGD << "x: " << lmk->values_[j].x_
           << " y: " << lmk->values_[j].y_
           << " score: " << lmk->values_[j].score_;
    }
    // convert
    // 0~37 is the same
    size_t j = 42;
    for (size_t i = 38; i < 43; ++i) {
      lmk->values_[i].x_ = tmp_lmk_list[j]->x_;
      lmk->values_[i].y_ = tmp_lmk_list[j]->y_;
      lmk->values_[i].score_ = tmp_lmk_list[j]->score_;
      j++;
    }

    j = 51;
    for (size_t i = 43; i < 47; ++i) {
      lmk->values_[i].x_ = tmp_lmk_list[j]->x_;
      lmk->values_[i].y_ = tmp_lmk_list[j]->y_;
      lmk->values_[i].score_ = tmp_lmk_list[j]->score_;
      j++;
    }

    j = 58;
    for (size_t i = 47; i < 52; ++i) {
      lmk->values_[i].x_ = tmp_lmk_list[j]->x_;
      lmk->values_[i].y_ = tmp_lmk_list[j]->y_;
      lmk->values_[i].score_ = tmp_lmk_list[j]->score_;
      j++;
    }

    j = 66;
    for (size_t i = 52; i < 54; ++i) {
      lmk->values_[i].x_ = tmp_lmk_list[j]->x_;
      lmk->values_[i].y_ = tmp_lmk_list[j]->y_;
      lmk->values_[i].score_ = tmp_lmk_list[j]->score_;
      j++;
    }

    j = 69;
    for (size_t i = 54; i < 57; ++i) {
      lmk->values_[i].x_ = tmp_lmk_list[j]->x_;
      lmk->values_[i].y_ = tmp_lmk_list[j]->y_;
      lmk->values_[i].score_ = tmp_lmk_list[j]->score_;
      j++;
    }

    lmk->values_[57].x_ = tmp_lmk_list[73]->x_;
    lmk->values_[57].y_ = tmp_lmk_list[73]->y_;
    lmk->values_[57].score_ = tmp_lmk_list[73]->score_;

    j = 75;
    for (size_t i = 58; i < 60; ++i) {
      lmk->values_[i].x_ = tmp_lmk_list[j]->x_;
      lmk->values_[i].y_ = tmp_lmk_list[j]->y_;
      lmk->values_[i].score_ = tmp_lmk_list[j]->score_;
      j++;
    }

    j = 78;
    for (size_t i = 60; i < 63; ++i) {
      lmk->values_[i].x_ = tmp_lmk_list[j]->x_;
      lmk->values_[i].y_ = tmp_lmk_list[j]->y_;
      lmk->values_[i].score_ = tmp_lmk_list[j]->score_;
      j++;
    }

    lmk->values_[63].x_ = tmp_lmk_list[82]->x_;
    lmk->values_[63].y_ = tmp_lmk_list[82]->y_;
    lmk->values_[63].score_ = tmp_lmk_list[82]->score_;

    j = 41;
    for (size_t i = 64; i < 68; ++i) {
      lmk->values_[i].x_ = tmp_lmk_list[j]->x_;
      lmk->values_[i].y_ = tmp_lmk_list[j]->y_;
      lmk->values_[i].score_ = tmp_lmk_list[j]->score_;
      j--;
    }

    j = 50;
    for (size_t i = 68; i < 72; ++i) {
      lmk->values_[i].x_ = tmp_lmk_list[j]->x_;
      lmk->values_[i].y_ = tmp_lmk_list[j]->y_;
      lmk->values_[i].score_ = tmp_lmk_list[j]->score_;
      j--;
    }

    lmk->values_[72].x_ = tmp_lmk_list[68]->x_;
    lmk->values_[72].y_ = tmp_lmk_list[68]->y_;
    lmk->values_[72].score_ = tmp_lmk_list[68]->score_;

    lmk->values_[73].x_ = tmp_lmk_list[72]->x_;
    lmk->values_[73].y_ = tmp_lmk_list[72]->y_;
    lmk->values_[73].score_ = tmp_lmk_list[72]->score_;

    // lmk->values_[74].x_ = tmp_lmk_list[74]->x_;
    // lmk->values_[74].y_ = tmp_lmk_list[74]->y_;
    // lmk->values_[74].score_ = tmp_lmk_list[74]->score_;

    lmk->values_[75].x_ = tmp_lmk_list[77]->x_;
    lmk->values_[75].y_ = tmp_lmk_list[77]->y_;
    lmk->values_[75].score_ = tmp_lmk_list[77]->score_;

    lmk->values_[76].x_ = tmp_lmk_list[81]->x_;
    lmk->values_[76].y_ = tmp_lmk_list[81]->y_;
    lmk->values_[76].score_ = tmp_lmk_list[81]->score_;

    lmk->values_[77].x_ = tmp_lmk_list[83]->x_;
    lmk->values_[77].y_ = tmp_lmk_list[83]->y_;
    lmk->values_[77].score_ = tmp_lmk_list[83]->score_;

    lmk->values_[78].x_ = tmp_lmk_list[55]->x_;
    lmk->values_[78].y_ = tmp_lmk_list[55]->y_;
    lmk->values_[78].score_ = tmp_lmk_list[55]->score_;

    lmk->values_[79].x_ = tmp_lmk_list[65]->x_;
    lmk->values_[79].y_ = tmp_lmk_list[65]->y_;
    lmk->values_[79].score_ = tmp_lmk_list[65]->score_;

    lmk->values_[80].x_ = tmp_lmk_list[56]->x_;
    lmk->values_[80].y_ = tmp_lmk_list[56]->y_;
    lmk->values_[80].score_ = tmp_lmk_list[56]->score_;

    lmk->values_[81].x_ = tmp_lmk_list[64]->x_;
    lmk->values_[81].y_ = tmp_lmk_list[64]->y_;
    lmk->values_[81].score_ = tmp_lmk_list[64]->score_;

    lmk->values_[82].x_ = tmp_lmk_list[57]->x_;
    lmk->values_[82].y_ = tmp_lmk_list[57]->y_;
    lmk->values_[82].score_ = tmp_lmk_list[57]->score_;

    lmk->values_[83].x_ = tmp_lmk_list[63]->x_;
    lmk->values_[83].y_ = tmp_lmk_list[63]->y_;
    lmk->values_[83].score_ = tmp_lmk_list[63]->score_;

    // point 84~105 is the same
    tmp_lmk_list.clear();
  }
}

void KeyPointConvertor::ConvertKps(xstream::BaseDataVector *kps_list,
                                   xstream::BaseDataVector *body_box_list,
                                   xstream::BaseDataVector *head_box_list,
                                   xstream::BaseDataVector *face_lmks,
                                   xstream::BaseDataVector *face_box_list) {
  if (kps_list == nullptr || body_box_list == nullptr ||
      head_box_list == nullptr)
    return;
  std::vector<std::shared_ptr<xstream::Point>> tmp_lmk_list;
  for (size_t i = 0; i < kps_list->datas_.size(); ++i) {
    auto lmk = std::static_pointer_cast<
        xstream::Landmarks>(kps_list->datas_[i]);
    if (!lmk) continue;
    if (lmk->values_.size() < 17) {
      LOGI << "UpdateHandTrackID not body kps, error!!!";
      continue;
    }

    auto body_box =
        std::static_pointer_cast<xstream::BBox>(
            body_box_list->datas_[i]);
    if (!body_box) continue;
    int32_t track_id = body_box->id_;
    if (track_id == -1) continue;
    tmp_lmk_list.resize(lmk->values_.size());
    for (size_t j = 0; j < lmk->values_.size(); ++j) {
      auto point = std::make_shared<xstream::Point>();
      point->x_ = lmk->values_[j].x_;
      point->y_ = lmk->values_[j].y_;
      point->score_ = lmk->values_[j].score_;
      tmp_lmk_list[j] = point;
      LOGD << "x: " << lmk->values_[j].x_
           << " y: " << lmk->values_[j].y_
           << " score: " << lmk->values_[j].score_;
    }

    // find box by trackid
    for (size_t k = 0; k < head_box_list->datas_.size(); ++k) {
      auto head_box =
          std::static_pointer_cast<xstream::BBox>(
              head_box_list->datas_[k]);
      if (head_box->id_ != track_id) continue;

      xstream::Point head;
      head.x_ =
          head_box->x1_ + (head_box->x2_ - head_box->x1_) / 2;
      head.y_ = head_box->y1_;
      head.score_ = head_box->score_;
      lmk->values_[0].x_ = head.x_;
      lmk->values_[0].y_ = head.y_;
      lmk->values_[0].score_ = head.score_;
      if (!face_lmks || !face_box_list) {
        LOGI << "ConvertToTCLKps recv null face info, get head line to convert";
        xstream::Point jaw;
        jaw.x_ = head.x_;
        jaw.y_ = head_box->y2_;
        jaw.score_ = head_box->score_;
        lmk->values_[1].x_ = jaw.x_;
        lmk->values_[1].y_ = jaw.y_;
        lmk->values_[1].score_ = jaw.score_;
      }
      break;
    }

    if (face_lmks && face_box_list) {
      for (size_t k = 0; k < face_lmks->datas_.size(); ++k) {
        auto face_lmk = std::static_pointer_cast<
            xstream::Landmarks>(
            face_lmks->datas_[k]);
        auto face_box =
            std::static_pointer_cast<xstream::BBox>(
                face_box_list->datas_[k]);
        if (face_box->id_ != track_id) continue;

        lmk->values_[1].x_ = face_lmk->values_[17].x_;
        lmk->values_[1].y_ = face_lmk->values_[17].y_;
        lmk->values_[1].score_ = face_lmk->values_[17].score_;
        break;
      }
    }
    // convert
    for (size_t j = 2; j < 14; ++j) {
      if (j % 2) {
        lmk->values_[j].x_ = tmp_lmk_list[j + 2]->x_;
        lmk->values_[j].y_ = tmp_lmk_list[j + 2]->y_;
        lmk->values_[j].score_ = tmp_lmk_list[j + 2]->score_;
      } else {
        lmk->values_[j].x_ = tmp_lmk_list[j + 4]->x_;
        lmk->values_[j].y_ = tmp_lmk_list[j + 4]->y_;
        lmk->values_[j].score_ = tmp_lmk_list[j + 4]->score_;
      }
    }

    lmk->values_.resize(14);
    tmp_lmk_list.clear();
  }
}

}  // namespace xproto
