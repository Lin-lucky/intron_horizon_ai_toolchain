/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     snapshot_data_type implementation
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.10
 * @date      2019.04.22
 */

#include "snapshot_method/snapshot_data_type.h"

#include <fstream>
#include <vector>

#include "snapshot_method/error_code.h"
#include "hobotlog/hobotlog.hpp"
#include "json/json.h"

namespace xstream {

FloatPoints SelectSnapShotInfo::PointsToSnap(const FloatPoints &in) {
  FloatPoints out = FloatPoints();
  for (auto &point : in.values_) {
    Point p;
    p.x_ = (floor(point.x_) - snap_base_point.x_) * wide_scale;
    p.y_ = (floor(point.y_) - snap_base_point.y_) * height_scale;
    p.score_ = point.score_;
    out.values_.push_back(p);
  }
  return out;
}

int SnapShotParam::UpdateParameter(const std::string &content) {
  LOGD << "SnapShotParam update config: " << this;
  Json::CharReaderBuilder builder;
  builder["collectComments"] = false;
  JSONCPP_STRING error;
  std::shared_ptr<Json::CharReader> json_reader(builder.newCharReader());
  try {
    Json::Value json_var;
    bool ret = json_reader->parse(
        content.c_str(), content.c_str() + content.size(), &json_var, &error);
    SET_SNAPSHOT_METHOD_PARAM(json_var, Double, scale_rate);
    SET_SNAPSHOT_METHOD_PARAM(json_var, Bool, need_resize);
    SET_SNAPSHOT_METHOD_PARAM(json_var, UInt, output_width);
    SET_SNAPSHOT_METHOD_PARAM(json_var, UInt, output_height);
    SET_SNAPSHOT_METHOD_PARAM(json_var, Bool, save_original_image_frame);
    SET_SNAPSHOT_METHOD_PARAM(json_var, Bool, snapshot_state_enable);
    SET_SNAPSHOT_METHOD_PARAM(json_var, String, snapshot_type);

    LOGD << "scale_rate: " << scale_rate;
    LOGD << "need_resize: " << need_resize;
    LOGD << "output_width: " << output_width;
    LOGD << "output_height: " << output_height;
    LOGD << "save_original_image_frame: " << save_original_image_frame;

    if (ret) {
      return XSTREAM_SNAPSHOT_OK;
    } else {
      return XSTREAM_SNAPSHOT_ERR_PARAM;
    }
  } catch (std::exception &e) {
    return XSTREAM_SNAPSHOT_ERR_PARAM;
  }
}

std::string SnapShotParam::Format() { return config_jv.toStyledString(); }

SelectSnapShotInfoPtr SnapShotInfo::GetSnapShotInfo(
    const ImageFramePtr &frame, const float &select_score,
    const xstream::BBoxPtr &pbbox, SnapShotParam *param,
    std::vector<BaseDataPtr> userdatas) {
  SelectSnapShotInfoPtr snapshot_info(new SelectSnapShotInfo());
  snapshot_info->track_id_ = pbbox->id_;
  snapshot_info->select_value_ = select_score;
  if (param->save_original_image_frame) {
    snapshot_info->origin_image_frame_ = frame;
  } else {
    ImageFramePtr image_frame(new xstream::RawDataImageFrame());
    image_frame->frame_id_ = frame->frame_id_;
    image_frame->time_stamp_ = frame->time_stamp_;
    image_frame->pixel_format_ = frame->pixel_format_;
    image_frame->type_ = frame->type_;
    image_frame->channel_id_ = frame->channel_id_;
    snapshot_info->origin_image_frame_ = image_frame;
  }
  auto &bbox = pbbox;
  auto ad_bbox = ImageUtils::AdjustSnapRect(frame->Width(0), frame->Height(0),
                                            *(bbox), param->scale_rate);
  snapshot_info->snap_base_point.x_ = ad_bbox.x1_;
  snapshot_info->snap_base_point.y_ = ad_bbox.y1_;
  auto snap_width = ad_bbox.Width() + 1;
  auto snap_height = ad_bbox.Height() + 1;
  if (param->need_resize) {
    snapshot_info->wide_scale =
        static_cast<float>(param->output_width) / snap_width;
    snapshot_info->height_scale =
        static_cast<float>(param->output_height) / snap_height;
  } else {
    snapshot_info->wide_scale = 1;
    snapshot_info->height_scale = 1;
  }

  snapshot_info->snap_ =
      ImageUtils::DoFaceCrop(frame, ad_bbox, param->output_width,
                             param->output_height, param->need_resize);
  snapshot_info->userdata_ = std::move(userdatas);
  return snapshot_info;
}

SelectSnapShotInfoPtr CopySelectSnapShotInfo(
    const SelectSnapShotInfoPtr &input) {
  SelectSnapShotInfoPtr cp_info(new SelectSnapShotInfo());
  cp_info->snap_type_ = input->snap_type_;
  cp_info->height_scale = input->height_scale;
  cp_info->wide_scale = input->wide_scale;
  cp_info->snap_base_point = input->snap_base_point;
  cp_info->snap_ = input->snap_;
  cp_info->origin_image_frame_ = input->origin_image_frame_;
  cp_info->select_value_ = input->select_value_;
  cp_info->track_id_ = input->track_id_;
  cp_info->userdata_ = input->userdata_;
  return cp_info;
}

BaseDataVectorPtr SnapShotInfo::GenerateSnapshotInfo(
    const std::vector<SelectSnapShotInfoPtr> &snap_infos, const int32_t &type) {
  BaseDataVectorPtr snap_list(new BaseDataVector());
  for (auto &snap_info : snap_infos) {
    SnapshotInfoBaseDataPtr xstream_snapshot_info
       = CopySelectSnapShotInfo(snap_info);
    xstream_snapshot_info->snap_type_ = type;
    xstream_snapshot_info->type_ = "SnapshotInfo";
    snap_list->datas_.emplace_back(xstream_snapshot_info);
  }
  return snap_list;
}

BaseDataPtr SnapShotInfo::GenerateWithoutSnapshot(const int32_t id) {
  SnapshotInfoBaseDataPtr xstream_snapshot_info = nullptr;
  auto info = static_cast<SelectSnapShotInfo *>(
      std::calloc(1, sizeof(SelectSnapShotInfo)));
  SelectSnapShotInfoPtr cp_info(info);
  cp_info->snap_type_ = FLUSH_POST_TYPE;
  cp_info->track_id_ = id;
  xstream_snapshot_info = cp_info;
  xstream_snapshot_info->type_ = "SnapshotInfo";
  return xstream_snapshot_info;
}

}  // namespace xstream
