/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Fei cheng
 * @Mail: fei.cheng@horizon.ai
 * @Date: 2019-09-14 20:38:52
 * @Version: v0.0.1
 * @Brief: mcplugin impl based on xproto.
 * @Last Modified by: Fei cheng
 * @Last Modified time: 2019-09-14 22:41:30
 */

#include "monitor_control_plugin/monitor_control_plugin.h"

#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "singleton.h"
#include "transport_message/monitor_control_message.h"
#include "transport_message/uvc_message.h"
#include "utils/executor.hpp"
#include "utils/syn_msgHandle_manage.hpp"
#include "utils/votmodule.h"
#include "xproto/message/msg_registry.h"
#include "xproto/msg_type/protobuf/x3.pb.h"

namespace xproto {

using xproto::message::TransportMessage;
using xproto::message::APImageMessage;
using xproto::message::MonitorControlMessage;
using MCMsgPtr = std::shared_ptr<MonitorControlMessage>;

XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_MC_UPSTREAM_MESSAGE)
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_MC_DOWMSTREAM_MESSAGE)

class PluginContext : public horizon::vision::CSingleton<PluginContext> {
 public:
  PluginContext() : exit(false) { plugins.clear(); }
  ~PluginContext() = default;

 public:
  bool exit;
  uint32_t basic_plugin_cnt = 3;
  std::vector<std::shared_ptr<XPluginAsync>> plugins;
};

struct target_key {
  std::string category;
  int id;
  target_key(std::string category, int id) {
    this->category = category;
    this->id = id;
  }
};

struct cmp_key {
  bool operator()(const target_key &a, const target_key &b) {
    if (a.category == b.category) {
      return a.id < b.id;
    }
    return a.category < b.category;
  }
};

bool MonitorControlMessage::IsMessageValid(ConfigMessageMask mask) {
  return message_mask_ & mask ? true : false;
}

MonitorControlPlugin::MonitorControlPlugin(const std::string& config_file) {
  config_file_ = config_file;
  LOGI << "MC config file:" << config_file_;
  Json::Value cfg_jv;
  std::ifstream infile(config_file_);
  if (infile) {
    infile >> cfg_jv;
    config_.reset(new JsonConfigWrapper(cfg_jv));
    ParseConfig();
  } else {
    LOGE << "open mc config fail";
  }
}

void MonitorControlPlugin::ParseConfig() { LOGI << "Parse Config Done!"; }

int MonitorControlPlugin::Init() {
  LOGI << "MonitorControlPlugin INIT";
  RegisterMsg(TYPE_TRANSPORT_MESSAGE,
              std::bind(&MonitorControlPlugin::OnGetUvcResult, this,
                        std::placeholders::_1));
  RegisterMsg(TYPE_SMART_MESSAGE,
              std::bind(&MonitorControlPlugin::OnGetSmarterResult, this,
                        std::placeholders::_1));
  RegisterMsg(TYPE_IMAGE_MESSAGE,
              std::bind(&MonitorControlPlugin::OnGetVioResult, this,
                        std::placeholders::_1));
  RegisterMsg(TYPE_DROP_IMAGE_MESSAGE,
              std::bind(&MonitorControlPlugin::OnGetVioResult,
                        this, std::placeholders::_1));
  RegisterMsg(TYPE_INFO_IMAGE_MESSAGE,
              std::bind(&MonitorControlPlugin::OnGetVioResult,
                        this, std::placeholders::_1));
  return XPluginAsync::Init();
}

std::shared_ptr<xstream::RawDataImageFrame>
MonitorControlPlugin::ConstructVotImgData(
    const std::shared_ptr<VioMessage>& vio_) {
  HOBOT_CHECK(vio_);
  if (-1 == feed_vo_pym_layer_) {
    for (int layer = 0; layer < DOWN_SCALE_MAX; layer++) {
      if (feed_vo_dest_w_ ==
              vio_->image_.front()->img_.down_scale[layer].width &&
          feed_vo_dest_h_ ==
              vio_->image_.front()->img_.down_scale[layer].height) {
        feed_vo_pym_layer_ = layer;
        feed_vo_src_w_ = vio_->image_.front()->img_.down_scale[0].width;
        feed_vo_src_h_ = vio_->image_.front()->img_.down_scale[0].height;
        LOGD << "find pym layer: " << feed_vo_pym_layer_
             << "  feed_vo_src_w_: " << feed_vo_src_w_
             << "  feed_vo_src_h_: " << feed_vo_src_h_;
        break;
      }
    }
  }

  if (-1 == feed_vo_pym_layer_) {
    LOGE << "invalid feed_vo_pym_layer_";
    return nullptr;
  }

  // copy img
  auto sp_img = std::shared_ptr<xstream::RawDataImageFrame>(
          new xstream::RawDataImageFrame,
          [&](xstream::RawDataImageFrame* p){
              if (p) {
                if (p->data_) {
                  std::free(p->data_);
                  p->data_ = NULL;
                }
                delete p;
                p = NULL;
              }
          });
  // copy img
  sp_img->frame_id_ = vio_->sequence_id_;
  const auto& pym_info =
      vio_->image_.front()->img_.down_scale[feed_vo_pym_layer_];
  sp_img->height_ = pym_info.height;
  sp_img->width_ = pym_info.width;
  sp_img->data_ = static_cast<uint8_t *>(
                  std::calloc(sp_img->DataSize(),
                              sizeof(uint8_t)));
  memcpy(sp_img->data_,
         reinterpret_cast<char*>(pym_info.y_vaddr),
         pym_info.width * pym_info.height);
  memcpy(sp_img->data_
         + pym_info.width * pym_info.height,
         reinterpret_cast<char*>(pym_info.c_vaddr),
         pym_info.width * pym_info.height / 2);

  return sp_img;
}

void MonitorControlPlugin::DumpSmart2Json(
        const std::shared_ptr<SmartMessage>& smart,
        const std::shared_ptr<std::string>& sp_smart_pb_) {
  // CustomSmartMessage
  auto custom_smart_msg = dynamic_cast<SmartMessage *>(smart.get());
  HOBOT_CHECK(custom_smart_msg);
  uint64_t frame_id = custom_smart_msg->frame_id_;

  smart_data_t smart_data;
  x3::MessagePack pack;
  x3::FrameMessage frame;
  if (sp_smart_pb_ &&
      pack.ParseFromString(*sp_smart_pb_) &&
      frame.ParseFromString(pack.content_()) &&
      frame.has_smart_msg_()) {
    LOGD << "targets__size:" << frame.smart_msg_().targets__size();
    for (int idx_tar = 0; idx_tar < frame.smart_msg_().targets__size();
         idx_tar++) {
      const auto& target = frame.smart_msg_().targets_(idx_tar);
      auto track_id = target.track_id_();

      if (target.boxes__size() > 0) {
        // box
        for (int idx = 0; idx < target.boxes__size(); idx++) {
          const auto &box = target.boxes_(idx);
          LOGD << "box:" << box.type_()
               << " x1:" << box.top_left_().x_()
               << " y1:" << box.top_left_().y_()
               << " x2:" << box.bottom_right_().x_()
               << " y2:" << box.bottom_right_().y_();

          xstream::BBox rect;
          rect.x1_ = box.top_left_().x_();
          rect.y1_ = box.top_left_().y_();
          rect.x2_ = box.bottom_right_().x_();
          rect.y2_ = box.bottom_right_().y_();
          rect.score_ = box.score();
          if ("face" == box.type_()) {
            smart_data.faces[track_id].box = std::move(rect);
          } else if ("head" == box.type_()) {
          } else if ("body" == box.type_()) {
            smart_data.bodys[track_id].box = std::move(rect);
          } else if ("hand" == box.type_()) {
            smart_data.hands[track_id].box = std::move(rect);
          }
        }
      }

      // points
      for (int idx = 0; idx < target.points__size(); idx++) {
        const auto point = target.points_(idx);
        LOGD << "point type:" << point.type_();
        if (point.type_() == "lmk_106pts") {
          for (int lmk_idx = 0; lmk_idx < point.points__size();
               lmk_idx++) {
            xstream::Point pt(point.points_(lmk_idx).x_(),
                                    point.points_(lmk_idx).y_(),
                                    point.points_(lmk_idx).score_());
            smart_data.faces[track_id].lmk.values_.emplace_back(pt);
          }
        }

        if ("body_landmarks" == point.type_()) {
          for (int lmk_idx = 0; lmk_idx < point.points__size(); lmk_idx++) {
            xstream::Point pt(point.points_(lmk_idx).x_(),
                                    point.points_(lmk_idx).y_(),
                                    point.points_(lmk_idx).score_());
            smart_data.bodys[track_id].lmk.values_.emplace_back(pt);
          }
        }

        if (point.type_() == "hand_landmarks") {
          for (int lmk_idx = 0; lmk_idx < point.points__size(); lmk_idx++) {
            xstream::Point pt(point.points_(lmk_idx).x_(),
                                    point.points_(lmk_idx).y_(),
                                    point.points_(lmk_idx).score_());
            smart_data.hands[track_id].lmk.values_.emplace_back(pt);
          }
        }
      }

      // attr
      for (int idx = 0; idx < target.attributes__size(); idx++) {
        const auto& attr = target.attributes_(idx);
        LOGD << frame_id << " attr: " << attr.type_()
             << " " << attr.value_() << " " << attr.score_();

        if (attr.type_() == "gesture") {
          smart_data.hands[track_id].gesture = attr.value_();
          smart_data.hands[track_id].gesture_score = attr.score_();
        }
//        if (attr.type_() == "gesture_raw") {
//          smart_data.hands[track_id].gesture_score = attr.score_();
//        }
        if (attr.type_() == "age") {
          smart_data.faces[track_id].age = attr.value_();
        }
        if (attr.type_() == "gender") {
          smart_data.faces[track_id].gender = attr.value_();
        }
        if (attr.type_() == "dist") {
          smart_data.faces[track_id].dist = attr.value_();
        }
      }
    }
  } else {
    LOGE << "parse fail";
  }

  Json::Value targets;
  Json::Value faceInfos;
  Json::Value handInfos;
  Json::Value bodyInfos;
  Json::Value bodyKeyPoints;
  Json::Value gestureKeyPoints;

  // face
  if (!smart_data.faces.empty()) {
    for (const auto& face : smart_data.faces) {
      Json::Value faceInfo;
      faceInfo["faceID"] = face.first;
      faceInfo["faceDistance"] = face.second.dist;
      faceInfo["faceQuality"] = face.second.box.score_;
      faceInfo["faceFeatureNum"] = 0;
      faceInfo["facePictureNum"] = 0;

      Json::Value rect;
      rect["left"] = face.second.box.x1_;
      rect["top"] = face.second.box.y1_;
      rect["right"] = face.second.box.x2_;
      rect["bottom"] = face.second.box.y2_;
      faceInfo["faceRectangle"] = rect;

      faceInfo["faceAttributeNum"] = 2;
      Json::Value attr;
      attr["age"] = face.second.age;
      attr["gender"] = face.second.gender;
      faceInfo["faceAttribute"] = attr;

      faceInfo["faceKeyPointsNum"] = face.second.lmk.values_.size();
      Json::Value lmks;
      for (const auto& val : face.second.lmk.values_) {
        Json::Value lmk;
        lmk["x"] = val.x_;
        lmk["y"] = val.y_;
        lmk["z"] = -1;
        lmk["confidence"] = val.score_;
        lmks.append(lmk);
      }
      faceInfo["faceKeyPoints"] = lmks;

      faceInfos.append(faceInfo);
    }
  }

  // hand
  if (!smart_data.hands.empty()) {
    for (const auto& hand : smart_data.hands) {
      Json::Value handInfo;
      handInfo["handID"] = hand.first;
      handInfo["faceID"] = -1;
      handInfo["event"] = -1;
      handInfo["handDistance"] = -1;

      // todo
      auto action = dump_gesture_action::GESTURE_UNKNOWN;
      if (horizon_gesture_dump_router_.find(hand.second.gesture) !=
              horizon_gesture_dump_router_.end()) {
        action = horizon_gesture_dump_router_.at(hand.second.gesture);
      }
      handInfo["action"] = static_cast<int>(action);
      handInfo["actionScore"] = hand.second.gesture_score;

      Json::Value rect;
      rect["left"] = hand.second.box.x1_;
      rect["top"] = hand.second.box.y1_;
      rect["right"] = hand.second.box.x2_;
      rect["bottom"] = hand.second.box.y2_;
      handInfo["rectangle"] = rect;

      handInfo["keyPointsNum"] = hand.second.lmk.values_.size();
      Json::Value lmks;
      for (const auto& val : hand.second.lmk.values_) {
        Json::Value lmk;
        lmk["x"] = val.x_;
        lmk["y"] = val.y_;
        lmk["z"] = -1;
        lmk["confidence"] = val.score_;
        lmks.append(lmk);
      }
      handInfo["landmarks"] = lmks;

      handInfos.append(handInfo);
    }
  } else {
    Json::Value handInfo;
    handInfo["handID"] = -1;

    // todo
    auto action = dump_gesture_action::GESTURE_UNKNOWN;
    handInfo["action"] = static_cast<int>(action);
    handInfo["actionScore"] = 0;

    Json::Value rect;
    rect["left"] = -1;
    rect["top"] = -1;
    rect["right"] = -1;
    rect["bottom"] = -1;
    handInfo["rectangle"] = rect;
    handInfo["keyPointsNum"] = 0;

    handInfos.append(handInfo);
  }

  // body
  if (!smart_data.bodys.empty()) {
    for (const auto& body : smart_data.bodys) {
      Json::Value bodyInfo;
      bodyInfo["bodyID"] = body.first;
      bodyInfo["faceID"] = -1;
      bodyInfo["quality"] = body.second.box.score_;
      bodyInfo["bodyDistance"] = -1;
      bodyInfo["attributeNum"] = 0;

      Json::Value rect;
      rect["left"] = body.second.box.x1_;
      rect["top"] = body.second.box.y1_;
      rect["right"] = body.second.box.x2_;
      rect["bottom"] = body.second.box.y2_;
      bodyInfo["rectangle"] = rect;

      bodyInfo["keyPointsNum"] = body.second.lmk.values_.size();
      Json::Value lmks;
      for (const auto& val : body.second.lmk.values_) {
        Json::Value lmk;
        lmk["x"] = val.x_;
        lmk["y"] = val.y_;
        lmk["z"] = -1;
        lmk["confidence"] = val.score_;
        lmks.append(lmk);
      }
      bodyInfo["keyPoints"] = lmks;

      bodyInfos.append(bodyInfo);
    }
  }

  // bodyKeyPoints
  if (!smart_data.bodys.empty()) {
    for (const auto& body : smart_data.bodys) {
      Json::Value bodyKeyPoint;
      bodyKeyPoint["bodyID"] = body.first;

      Json::Value lmks;
      for (const auto& val : body.second.lmk.values_) {
        Json::Value lmk;
        lmk["x"] = val.x_;
        lmk["y"] = val.y_;
        lmk["z"] = -1;
        lmk["confidence"] = val.score_;
        lmks.append(lmk);
      }
      bodyKeyPoint["data"] = lmks;

      bodyKeyPoints.append(bodyKeyPoint);
    }
  }

  // gestureKeyPoints
  if (!smart_data.hands.empty()) {
    for (const auto& hand : smart_data.hands) {
      Json::Value gestureKeyPoint;
      gestureKeyPoint["handID"] = hand.first;
      Json::Value lmks;
      for (const auto& val : hand.second.lmk.values_) {
        Json::Value lmk;
        lmk["x"] = val.x_;
        lmk["y"] = val.y_;
        lmk["z"] = -1;
        lmk["confidence"] = val.score_;
        lmks.append(lmk);
      }
      gestureKeyPoint["data"] = lmks;

      gestureKeyPoints.append(gestureKeyPoint);
    }
  }

  targets["faceInfos"] = faceInfos;
  if (handInfos.size() > 0) {
    targets["handInfos"] = handInfos;
  }
  targets["bodyInfos"] = bodyInfos;
  targets["bodyKeyPoints"] = bodyKeyPoints;
  if (gestureKeyPoints.size() > 0) {
    targets["gestureKeyPoints"] = gestureKeyPoints;
  }
  std::lock_guard<std::mutex> lg(dump_smart_2_json_mutex_);
  root_[std::to_string(frame_id)] = targets;
}

std::shared_ptr<std::string>
MonitorControlPlugin::ConstructVotSmartData(
    const std::shared_ptr<SmartMessage>& smart_) {
  HOBOT_CHECK(smart_);
  if (0 == feed_vo_src_w_ || 0 == feed_vo_src_h_) {
    LOGE << "invalid src w/h";
    return nullptr;
  }

  auto smart_msg = dynamic_cast<SmartMessage *>(smart_.get());
  HOBOT_CHECK(smart_msg);

  smart_msg->SetAPMode(true);
  std::shared_ptr<std::string> sp_smart_pb_ = std::make_shared<std::string>(
          smart_msg->Serialize(feed_vo_src_w_, feed_vo_src_h_,
                               feed_vo_dest_w_, feed_vo_dest_h_));
  HOBOT_CHECK(sp_smart_pb_);

  std::shared_ptr<std::string> sp_smart_pb_append = nullptr;
  if (enable_dump_smart_) {
    sp_smart_pb_append = DumpSmartData(smart_, sp_smart_pb_);
  }

  if (enable_dump_smart_2_json_) {
    DumpSmart2Json(smart_,
                   sp_smart_pb_append ? sp_smart_pb_append : sp_smart_pb_);
  }

  if (enable_append_smart_ && sp_smart_pb_append) {
    return sp_smart_pb_append;
  }

  return sp_smart_pb_;
}

std::shared_ptr<std::string>
MonitorControlPlugin::DumpSmartData(const std::shared_ptr<SmartMessage>& smart,
                        const std::shared_ptr<std::string>& sp_smart_pb_) {
  auto custom_smart_msg = dynamic_cast<SmartMessage *>(smart.get());
  const auto& smart_result = custom_smart_msg->GetSmartResult();
  HOBOT_CHECK(custom_smart_msg || smart_result || sp_smart_pb_);
  HOBOT_CHECK(feed_vo_src_w_ > 0 && feed_vo_src_h_ > 0 &&
                      feed_vo_dest_w_ > 0 && feed_vo_dest_h_ > 0)
  << "Serialize param error";

  std::shared_ptr<std::string> sp_smart_pb_append = nullptr;
  std::stringstream ss_dump_smart;
  // hand id, hand data
  std::map<int, gesture_dump_t> ss_dump_hand;
  ss_dump_smart << custom_smart_msg->frame_id_;
  float x_ratio = 1.0 * feed_vo_dest_w_ / feed_vo_src_w_;
  float y_ratio = 1.0 * feed_vo_dest_h_ / feed_vo_src_h_;

  // user-defined output parsing declaration.
  xstream::BaseDataVector *face_boxes = nullptr;
  xstream::BaseDataVector *hand_lmks = nullptr;
  auto name_prefix = [](const std::string name) -> std::string {
      auto pos = name.find('_');
      if (pos == std::string::npos)
        return "";

      return name.substr(0, pos);
  };

  auto name_postfix = [](const std::string name) -> std::string {
      auto pos = name.rfind('_');
      if (pos == std::string::npos)
        return "";

      return name.substr(pos + 1);
  };

  std::vector<std::shared_ptr<xstream::BBox>> face_box_list;
  std::vector<std::shared_ptr<xstream::BBox>> head_box_list;
  std::vector<std::shared_ptr<xstream::BBox>> body_box_list;
  std::vector<std::shared_ptr<xstream::BBox>> hand_box_list;
  std::set<target_key, cmp_key> smart_target;
  for (const auto &output : smart_result->datas_) {
    LOGD << "output name: " << output->name_;
    auto prefix = name_prefix(output->name_);
    auto postfix = name_postfix(output->name_);
    if (output->name_ == "face_bbox_list" || output->name_ == "head_box" ||
        output->name_ == "body_box" || postfix == "box") {
      face_boxes = dynamic_cast<xstream::BaseDataVector *>(output.get());
      LOGD << "box type: " << output->name_
           << ", box size: " << face_boxes->datas_.size();
      bool track_id_valid = true;
      if (face_boxes->datas_.size() > 1) {
        track_id_valid = false;
        for (size_t i = 0; i < face_boxes->datas_.size(); ++i) {
          auto face_box = std::static_pointer_cast<xstream::BBox>(
              face_boxes->datas_[i]);
          if (face_box->id_ != 0) {
            track_id_valid = true;
            break;
          }
        }
      }
      for (size_t i = 0; i < face_boxes->datas_.size(); ++i) {
        auto face_box = std::static_pointer_cast<xstream::BBox>(
            face_boxes->datas_[i]);
        if (!track_id_valid) {
          face_box->id_ = i + 1;
        }
        if (prefix == "hand") {
          face_box->specific_type_ = "hand";
        } else {
          face_box->specific_type_ = "person";
        }
        LOGD << output->name_ << " id: " << face_box->id_
             << " x1: " << face_box->x1_ << " y1: " << face_box->y1_
             << " x2: " << face_box->x2_ << " y2: " << face_box->y2_;
        if (prefix == "face") {
          face_box_list.push_back(face_box);
        } else if (prefix == "head") {
          head_box_list.push_back(face_box);
        } else if (prefix == "body") {
          body_box_list.push_back(face_box);
        } else if (prefix == "hand") {
          hand_box_list.push_back(face_box);
        } else {
          LOGE << "unsupport box name: " << output->name_;
        }
        target_key face_key(face_box->specific_type_, face_box->id_);
        if (face_box->id_ != -1) {
          if (smart_target.find(face_key) ==
              smart_target.end()) {  // 新track_id
            smart_target.insert(face_key);
          }

          if (prefix == "hand") {
            std::stringstream ss_hand;
            ss_hand << face_box->x1_ * x_ratio
                    << " " << face_box->y1_ * y_ratio
                    << " " << face_box->x2_ * x_ratio
                    << " " << face_box->y2_ * y_ratio
                    << " " << face_box->score_
                    << " " << prefix
                    << " " << face_box->id_;
            if (ss_dump_hand.find(face_box->id_) != ss_dump_hand.end()) {
              ss_dump_hand[face_box->id_].ss_hand = std::move(ss_hand);
            } else {
              gesture_dump_t gesture_output;
              gesture_output.ss_hand = std::move(ss_hand);
              ss_dump_hand[face_box->id_] = std::move(gesture_output);
            }
          } else {
            ss_dump_smart << " " << face_box->x1_ * x_ratio
                          << " " << face_box->y1_ * y_ratio
                          << " " << face_box->x2_ * x_ratio
                          << " " << face_box->y2_ * y_ratio
                          << " " << face_box->score_
                          << " " << prefix
                          << " " << face_box->id_;
          }
        }
      }
    }

    if (output->name_ == "hand_lmk" ||
        output->name_ == "lowpassfilter_hand_lmk") {
      hand_lmks = dynamic_cast<xstream::BaseDataVector *>(output.get());
      LOGD << "hand_lmk size: " << hand_lmks->datas_.size();
      if (hand_lmks->datas_.size() != hand_box_list.size()) {
        LOGE << "hand_lmk size: " << hand_lmks->datas_.size()
             << ", hand_box size: " << hand_box_list.size();
      }
      for (size_t i = 0; i < hand_lmks->datas_.size(); ++i) {
        auto lmk = std::static_pointer_cast<xstream::Landmarks>
            (hand_lmks->datas_[i]);
        // 查找对应的track_id
        if (hand_box_list[i]->id_ == -1) {
          continue;
        }
        target_key hand_key(hand_box_list[i]->specific_type_,
                            hand_box_list[i]->id_);
        if (smart_target.find(hand_key) == smart_target.end()) {
          LOGE << "Not found the track_id target";
        } else {
          std::stringstream ss_hand_lmk;
          for (size_t i = 0; i < lmk->values_.size(); ++i) {
            ss_hand_lmk << " " << std::round(lmk->values_[i].x_ * x_ratio)
                        << " " << std::round(lmk->values_[i].y_ * y_ratio)
                        << " " << lmk->values_[i].score_;
          }
          if (ss_dump_hand.find(hand_box_list[i]->id_) !=
                  ss_dump_hand.end()) {
            ss_dump_hand[hand_box_list[i]->id_].ss_hand << ss_hand_lmk.str();
          }
        }
      }
    }
    if (output->name_ == "gesture") {
      auto gesture_raws =
              dynamic_cast<xstream::BaseDataVector *>(output.get());
      if (gesture_raws->datas_.size() != hand_box_list.size()) {
        LOGE << "gesture raw size: " << gesture_raws->datas_.size()
             << ", body_box size: " << hand_box_list.size();
      }
      for (size_t i = 0; i < gesture_raws->datas_.size(); ++i) {
        auto gesture_raw = std::static_pointer_cast<xstream::Attribute_<int>>(
            gesture_raws->datas_[i]);
        // 查找对应的track_id
        if (hand_box_list[i]->id_ == -1) {
          LOGW << "hand id invalid";
          continue;
        }
        target_key hand_key(hand_box_list[i]->specific_type_,
                            hand_box_list[i]->id_);
        if (smart_target.find(hand_key) == smart_target.end()) {
          LOGE << "Not found the track_id target";
        } else {
          if (gesture_raw->state_ != xstream::DataState::VALID) {
            LOGW << "gesture state not valid";
            continue;
          }
          if (ss_dump_hand.find(hand_box_list[i]->id_) !=
                  ss_dump_hand.end()) {
            ss_dump_hand[hand_box_list[i]->id_].gest_raw =
                    gesture_raw->value_;
            ss_dump_hand[hand_box_list[i]->id_].gest_score =
                    gesture_raw->score_;
          }
        }
      }
    }

    if (output->name_ == "gesture_model_out") {
      if (output->state_ != xstream::DataState::VALID) {
        continue;
      }

      auto gesture_model_outs =
              dynamic_cast<xstream::BaseDataVector *>(output.get());
      if (!output || !gesture_model_outs ||
              gesture_model_outs->state_ != xstream::DataState::VALID) {
        continue;
      }

      if (gesture_model_outs->datas_.size() != hand_box_list.size()) {
        LOGE << "gesture_model_outs size: "
             << gesture_model_outs->datas_.size()
             << ", body_box size: " << hand_box_list.size();
        continue;
      }


      for (size_t i = 0; i < gesture_model_outs->datas_.size(); ++i) {
        auto gesture_model_out = std::static_pointer_cast<
            xstream::FloatFeature>(gesture_model_outs->datas_[i]);
        HOBOT_CHECK(gesture_model_out);

        // 查找对应的track_id
        if (hand_box_list[i]->id_ == -1) {
          LOGI << "hand id invalid";
          continue;
        }
        target_key hand_key(hand_box_list[i]->specific_type_,
                            hand_box_list[i]->id_);
        if (smart_target.find(hand_key) == smart_target.end()) {
          LOGE << "Not found the track_id target";
        } else {
          if (gesture_model_out->state_ != xstream::DataState::VALID ||
              gesture_model_out->name_ != "model_ouput") {
            LOGI << "gesture state/name not valid";
            continue;
          }

          std::vector<float> model_out_values;
          for (const auto& val : gesture_model_out->values_) {
            model_out_values.push_back(val);
          }
          if (ss_dump_hand.find(hand_box_list[i]->id_) !=
                  ss_dump_hand.end()) {
            ss_dump_hand[hand_box_list[i]->id_].model_out_values =
                    std::move(model_out_values);
          }
        }
      }
    }

    if (output->name_ == "gesture_vote") {
      auto gesture_votes =
              dynamic_cast<xstream::BaseDataVector *>(output.get());
      LOGD << "gesture_vote size: " << gesture_votes->datas_.size();
      if (gesture_votes->datas_.size() != hand_box_list.size()) {
        LOGE << "gesture_vote size: " << gesture_votes->datas_.size()
             << ", body_box size: " << hand_box_list.size();
      }
      for (size_t i = 0; i < gesture_votes->datas_.size(); ++i) {
        auto gesture_vote = std::static_pointer_cast<xstream::Attribute_<int>>(
            gesture_votes->datas_[i]);
        // 查找对应的track_id
        if (hand_box_list[i]->id_ == -1) {
          LOGI << "hand id invalid";
          continue;
        }
        target_key hand_key(hand_box_list[i]->specific_type_,
                            hand_box_list[i]->id_);
        if (smart_target.find(hand_key) == smart_target.end()) {
          LOGE << "Not found the track_id target";
        } else {
          if (gesture_vote->state_ != xstream::DataState::VALID) {
            LOGI << "gesture vote state not valid";
            continue;
          }
          int gesture_ret = -1;
          if (ss_dump_hand.find(hand_box_list[i]->id_) !=
                  ss_dump_hand.end()) {
            ss_dump_hand[hand_box_list[i]->id_].gest_vote =
                    gesture_vote->value_;
          }

          if (ss_dump_hand.find(hand_box_list[i]->id_) !=
                  ss_dump_hand.end()) {
            ss_dump_hand[hand_box_list[i]->id_].gest_final = gesture_ret;
          }
        }
      }
    }
  }

  // parse gesture final val from pb
  x3::MessagePack pack;
  x3::FrameMessage frame;
  if (pack.ParseFromString(*sp_smart_pb_) &&
      frame.ParseFromString(pack.content_())) {
    auto smart_msg = frame.smart_msg_();
    auto target_num = smart_msg.targets__size();
    LOGD << "recv smart frame target_num:" << target_num;
    for (int target_idx = 0; target_idx < target_num; target_idx++) {
      const auto& target = smart_msg.targets_(target_idx);
      if ("hand" != target.type_()) {
        continue;
      }
      auto track_id = target.track_id_();
      if (ss_dump_hand.find(track_id) != ss_dump_hand.end()) {
        // find gesture output from strategy
        for (int idx = 0; idx < target.attributes__size(); idx++) {
          const auto& attr = target.attributes_(idx);
          LOGD << "recv attr type " << attr.type_() << " " << attr.value_();
          if ("gesture" == attr.type_()) {
            ss_dump_hand[track_id].gest_final = attr.value_();
          }
        }

        // append gesture raw
        if (enable_append_smart_) {
          auto append_attr = frame.mutable_smart_msg_()
                  ->mutable_targets_(target_idx)->add_attributes_();
          append_attr->set_type_("gesture_raw");
          append_attr->set_value_(ss_dump_hand.at(track_id).gest_raw);
          append_attr->set_score_(ss_dump_hand.at(track_id).gest_score);
        }
      } else {
        LOGE << "find hand id: " << track_id << " fail from cache";
      }
    }

    if (enable_append_smart_) {
      pack.set_content_(frame.SerializeAsString());
      sp_smart_pb_append =
              std::make_shared<std::string>(pack.SerializeAsString());
    }
  }

  {
    std::lock_guard<std::mutex> lk(dump_smart_mutex_);
    if (ofs_dump_smart_.good()) {
      ofs_dump_smart_ << ss_dump_smart.str();
      for (const auto& dump_hand : ss_dump_hand) {
        ofs_dump_smart_ << " " << dump_hand.second.ss_hand.str()
                       << " " << dump_hand.second.gest_raw
                       << " " << dump_hand.second.gest_vote
                       << " " << dump_hand.second.gest_final
                       << " " << dump_hand.second.gest_score;
        for (const auto& val : dump_hand.second.model_out_values) {
          ofs_dump_smart_ << " " << val;
        }
      }
      ofs_dump_smart_ << std::endl;
    }
  }

  return sp_smart_pb_append;
}

int MonitorControlPlugin::Start() {
  is_running_ = true;
  cp_status_ = CP_READY;

  if (config_->HasKey("enable_dump_smart") &&
      config_->GetBoolValue("enable_dump_smart")) {
    LOGI << "enable enable_dump_smart";
    enable_dump_smart_ = true;

    std::lock_guard<std::mutex> lk(dump_smart_mutex_);
    if (ofs_dump_smart_.is_open()) {
      ofs_dump_smart_.close();
    }
    ofs_dump_smart_.open("dump_smart.txt");
  }
  if (config_->HasKey("enable_append_smart") &&
      config_->GetBoolValue("enable_append_smart")) {
    LOGI << "enable enable_append_smart";
    enable_append_smart_ = true;
  }
  if (config_->HasKey("enable_dump_img") &&
      config_->GetBoolValue("enable_dump_img")) {
    LOGI << "enable enable_dump_img";
    enable_dump_img_ = true;
  }
  if (config_->HasKey("save_dump_img_path")) {
    save_dump_img_path_ = config_->GetSTDStringValue("save_dump_img_path");
    LOGI << "save_dump_img_path: " << save_dump_img_path_;
    if (0 != access(save_dump_img_path_.data(), W_OK)) {
      LOGW << "unexist save_dump_img_path_: " << save_dump_img_path_
           << ", create path...";
      system(("mkdir " + save_dump_img_path_).data());
    }
  }
  if (config_->HasKey("enable_dump_smart_2_json") &&
      config_->GetBoolValue("enable_dump_smart_2_json")) {
    LOGI << "enable enable_dump_smart_2_json";
    enable_dump_smart_2_json_ = true;
  }
  if (config_ && config_->HasKey("enable_auto_start") &&
      config_->GetBoolValue("enable_auto_start")) {
    LOGI << "enable auto start";
    auto_start_ = true;

    for (auto plg : PluginContext::Instance().plugins) {
      if (plg->desc() == "uvcPlugin") {
        plg->Stop();
        break;
      }
    }

    if (config_->HasKey("enable_vot") &&
        config_->GetBoolValue("enable_vot")) {
      LOGI << "enable vot";
      enable_vot_ = true;
    }

    for (auto i = PluginContext::Instance().basic_plugin_cnt;
         i < PluginContext::Instance().plugins.size(); ++i) {
      PluginContext::Instance().plugins[i]->Init();
      PluginContext::Instance().plugins[i]->Start();
    }

    //  start vot
    if (enable_vot_) {
      std::string vot_cfg = "";
      if (config_->HasKey("vot_config")) {
        vot_cfg = config_->GetSTDStringValue("vot_config");
      }
      if (VotModule::Instance(vot_cfg)->Init() != 0 ||
              VotModule::Instance()->Start() != 0) {
        LOGE << "vot init/start fail";
        return -1;
      }
      //  feed vo
      auto task = [this] () {
          while (is_running_) {
            std::map<uint64_t,
                 std::shared_ptr<VotData_t>>::iterator front;
            std::shared_ptr<VotData_t> vot_feed = nullptr;

            std::unique_lock<std::mutex> lg(mut_cache_);
            cv_.wait(lg, [this] () {
                return !is_running_ || !cache_vot_data_.empty();
            });
            if (!is_running_) {
              cache_vot_data_.clear();
              break;
            }

            if (cache_vot_data_.empty()) {
              continue;
            }

            front = cache_vot_data_.begin();

            if (((!front->second->is_drop_frame_ &&
                    front->second->sp_img_ &&
                    front->second->sp_smart_pb_) ||
                    (front->second->is_drop_frame_ &&
                            front->second->sp_img_))) {
              vot_feed = front->second;
              cache_vot_data_.erase(front);
              lg.unlock();
              VotModule::Instance()->Input(vot_feed);
              continue;
            }

            LOGV << "cache_vio_smart_ size:" << cache_vot_data_.size();

            if (cache_vot_data_.size() >= cache_len_limit_) {
              LOGW << "cache_vio_smart_ size:" << cache_vot_data_.size()
                   << "  exceeds limit:" << cache_len_limit_;
              // exception occurred
              vot_feed = front->second;
              cache_vot_data_.erase(front);
              lg.unlock();
              if (vot_feed->sp_img_) {
                VotModule::Instance()->Input(vot_feed);
              }
              continue;
            }
          }

          LOGD << "thread  exit";
      };
      for (int i = 0; i < feed_vo_thread_num_; i++) {
        feed_vo_thread_.emplace_back(std::make_shared<std::thread>(task));
      }
    }
  } else {
    LOGI << "unable auto start";
    auto_start_ = false;
  }

  if (!status_monitor_thread_) {
    status_monitor_thread_ = std::make_shared<std::thread>(
            [this] () {
                unrecv_ap_count_ = 0;
                while (is_running_) {
                  if (CP_WORKING == cp_status_) {
                    unrecv_ap_count_++;
                    if (unrecv_ap_count_ > unrecv_ap_count_max) {
                      LOGW << "unrecv_ap_count_:" << unrecv_ap_count_
                           << " exceeds limit:" << unrecv_ap_count_max;
                      StopPlugin(0);
                    }
                  }
                  std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            });
  }

  if (config_->HasKey("enable_feedback") &&
      config_->GetBoolValue("enable_feedback")) {
    LOGD << "enable enable_feedback";
    if (config_->HasKey("feedback_file_path")) {
      std::string feedback_file_path =
       config_->GetSTDStringValue("feedback_file_path");
      LOGD << "feedback_file_path: " << feedback_file_path;

      task_feedback_ = std::make_shared<std::thread>(
              [this, feedback_file_path] () {
          std::vector<std::string> image_source_list;
          std::ifstream ifs(feedback_file_path);
          if (!ifs.good()) {
            LOGE << "Open file failed: " << feedback_file_path;
            return;
          }
          std::string image_path;
          while (std::getline(ifs, image_path)) {
            image_source_list.push_back(image_path);
          }
          ifs.close();
          LOGI << "image_source_list size: " << image_source_list.size();
          for (const auto& img_file : image_source_list) {
            if (!is_running_) {
              break;
            }
            std::ifstream ifs_img(img_file);
            if (ifs_img.good()) {
              std::stringstream ss;
              ss << ifs_img.rdbuf();
              auto& inst = SynMsgHandleManage<XProtoMessagePtr>::GetInstance();
              auto msg_id = inst.GenMsgId();
              LOGD << "sync send msg_id:" << msg_id;
              PushMsg(std::make_shared<APImageMessage>(
                      ss.str(), "JPEG",
                      0, 0,
                      msg_id));

              XProtoMessagePtr out;
              if (inst.AddPromise(msg_id) < 0 ||
                  inst.GetFuture(msg_id, out, 5000) < 0) {
                LOGW << "recv response time out msg_id:" << msg_id;
                inst.DelPromise(msg_id);
                continue;
              }
              inst.DelPromise(msg_id);

              map_feedback_namelist_[std::to_string(msg_id)] = img_file;
              LOGD << "insert img id: " << msg_id << " name: " << img_file;
              auto valid_frame = std::static_pointer_cast<VioMessage>(out);
              LOGD << "recv vio response:" << valid_frame->param_;
              if ("success" != valid_frame->param_) {
                LOGE << "process img fail img:" << img_file;
              }
              continue;
            } else {
              LOGE << "open img fail: " << img_file;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
          }

          // feedback done, stop plugins and app exits
          // wait for the last img process done
          std::this_thread::sleep_for(std::chrono::seconds(3));
          PluginContext::Instance().exit = true;
          return;
      });
    }
  }

  if (enable_dump_smart_2_json_) {
    std::lock_guard<std::mutex> lg(dump_smart_2_json_mutex_);
    root_.clear();
    map_feedback_namelist_.clear();
  }

  LOGD << "mc start";
  return 0;
}

int MonitorControlPlugin::Stop() {
  LOGD << "mc stop...";
  is_running_ = false;
  cp_status_ = CP_READY;

  LOGD << "feed thread stop start";
  cv_.notify_all();
  for (auto thread : feed_vo_thread_) {
    if (thread) {
      thread->join();
    }
  }
  feed_vo_thread_.clear();
  LOGD << "feed thread stop done";

  status_monitor_thread_->join();
  status_monitor_thread_ = nullptr;

  {
    std::unique_lock<std::mutex> lg(mut_cache_);
    cache_vot_data_.clear();
  }
  LOGD << "cache_vio_smart_ clear";

  if (auto_start_ && enable_vot_) {
    LOGD << "vot stop";
    // for feedback mode, HB_VP_Init is processed in vio plugin,
    // vot module only processes HB_SYS_Alloc
    // vot module should process HB_SYS_Free
    // before HB_VP_Exit is processed in vio plugin
    VotModule::Instance()->Stop();
    VotModule::Instance()->DeInit();
  }

  for (auto i = PluginContext::Instance().basic_plugin_cnt;
       i < PluginContext::Instance().plugins.size(); ++i) {
    PluginContext::Instance().plugins[i]->Stop();
    PluginContext::Instance().plugins[i]->DeInit();
  }
  LOGD << "plugin stop done";

  if (enable_dump_smart_) {
    std::lock_guard<std::mutex> lk(dump_smart_mutex_);
    if (ofs_dump_smart_.is_open()) {
      ofs_dump_smart_.close();
    }
  }

  if (task_feedback_) {
    task_feedback_->join();
    task_feedback_ = nullptr;
  }

  if (enable_dump_smart_2_json_ && !map_feedback_namelist_.empty()) {
    std::string save_json_path{"./dump_json"};
    if (access(save_json_path.data(), F_OK)) {
      system(std::string("rm -r " + save_json_path).data());
      system(std::string("mkdir " + save_json_path).data());
    }
    std::lock_guard<std::mutex> lg(dump_smart_2_json_mutex_);
    LOGD << "root size: " << root_.size();
    LOGD << "map_feedback_namelist_ size: " << map_feedback_namelist_.size();
    for (auto itr = root_.begin(); itr != root_.end(); itr++) {
      LOGD << "key: " << itr.key();
      if (map_feedback_namelist_.find(itr.key().asString()) !=
              map_feedback_namelist_.end()) {
        std::string img_file{map_feedback_namelist_.at(itr.key().asString())};
        std::string json_file{
                save_json_path + img_file.substr(img_file.find_last_of("/"))
                + ".json"};
        LOGD << "img name: " << img_file;
        LOGD << "json name: " << json_file;
        Json::StreamWriterBuilder builder;
        std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
        std::ofstream outputFileStream(json_file);
        if (outputFileStream.is_open()) {
          writer->write(*itr, &outputFileStream);
        } else {
          LOGE << "open file fail: " << json_file;
        }
      } else {
        LOGE << "find frame from feedback namelist fail, id: " << itr.key();
      }
    }
  }

  LOGD << "mc stop end";
  return 0;
}

int MonitorControlPlugin::StartPlugin(uint64_t msg_id) {
  int ret = 0;
  if (cp_status_ != CP_WORKING) {
    cp_status_ = CP_STARTING;
    for (auto i = PluginContext::Instance().basic_plugin_cnt;
         i < PluginContext::Instance().plugins.size(); ++i) {
      LOGI << "Start Plugin, index:" << i;
      PluginContext::Instance().plugins[i]->Init();
      ret = PluginContext::Instance().plugins[i]->Start();
      if (ret) {
        for (auto j = PluginContext::Instance().basic_plugin_cnt; j < i; ++j) {
          LOGW << "Stop Plugin, index:" << j;
          PluginContext::Instance().plugins[j]->Stop();
          PluginContext::Instance().plugins[j]->DeInit();
        }
        LOGE << "StartPlugin failed, index:" << i;
        break;
      }
    }
  }

  auto rsp_data = std::make_shared<SmarterConfigData>(ret == 0);
  auto mcmsg = std::make_shared<MonitorControlMessage>(
                   xproto::message::SET_APP_START,
                   xproto::message::MESSAGE_TO_ADAPTER,
                   rsp_data, "up", msg_id);
  PushMsg(mcmsg);

  if (ret) {
    cp_status_ = CP_ABNORMAL;
  } else {
    cp_status_ = CP_WORKING;
  }

  LOGD << "StartPlugin done msg_id:" << msg_id;
  return 0;
}

int MonitorControlPlugin::StopPlugin(uint64_t msg_id) {
  if (cp_status_ != CP_READY) {
    for (auto i = PluginContext::Instance().basic_plugin_cnt;
         i < PluginContext::Instance().plugins.size(); ++i) {
      LOGI << "stop plugin NO. " << i
           << "  " << PluginContext::Instance().plugins[i]->desc();
      PluginContext::Instance().plugins[i]->Stop();
      PluginContext::Instance().plugins[i]->DeInit();
    }
  }

  auto rsp_data = std::make_shared<SmarterConfigData>(true);
  auto mcmsg = std::make_shared<MonitorControlMessage>(
                 xproto::message::SET_APP_STOP,
                 xproto::message::MESSAGE_TO_ADAPTER,
                 rsp_data, "up", msg_id);
  PushMsg(mcmsg);
  cp_status_ = CP_READY;
  LOGD << "StopPlugin done";
  return 0;
}

int MonitorControlPlugin::GetCPStatus() { return cp_status_; }

int MonitorControlPlugin::GetCPLogLevel() { return log_level_; }

void MonitorControlPlugin::ConstructMsgForCmd(const x3::InfoMessage& InfoMsg,
                                  uint64_t msg_id) {
  ConfigMessageType type;

  if (!InfoMsg.has_command_()) {
    LOGE << "msg has no cmd";
    return;
  }

  auto cmd = InfoMsg.command_();
  LOGI << "cmd:" << cmd.order_();
  if (cmd.order_() == x3::Command_Order::Command_Order_StartX2) {
    type = xproto::message::SET_APP_START;
  } else if (cmd.order_() == x3::Command_Order::Command_Order_StopX2) {
    type = xproto::message::SET_APP_STOP;
  } else {
    LOGE << "unsupport cmd order: " << cmd.order_();
    return;
  }

  if (cmd_func_map.find(type) != cmd_func_map.end()) {
    Executor::GetInstance()->AddTask(
        [this, msg_id, type]() -> int { return cmd_func_map[type](msg_id); });
  } else {
    LOGE << "No support CMD found!";
  }
}

int MonitorControlPlugin::OnGetAPImage(
    const x3::Image &image, uint64_t seq_id) {
  auto hg_msg = std::make_shared<APImageMessage>(
      image.buf_(), image.type_(),
      image.width_(), image.height_(),
      seq_id);
  PushMsg(hg_msg);
  return 0;
}

int MonitorControlPlugin::OnGetUvcResult(const XProtoMessagePtr& msg) {
  auto uvc_msg = std::static_pointer_cast<TransportMessage>(msg);
  LOGD << "recv uvc ";
  x3::MessagePack pack_msg;
  x3::InfoMessage InfoMsg;
  x3::Image image;
  unrecv_ap_count_ = 0;
  auto pack_msg_parse = pack_msg.ParseFromString(uvc_msg->proto_);
  if (pack_msg_parse &&
      InfoMsg.ParseFromString(pack_msg.content_())) {
    if (InfoMsg.has_command_()) {
      auto msg_type = InfoMsg.command_().order_();
      LOGI << "msg_type is " << msg_type;
      if (x3::Command_Order_StartX2 == msg_type ||
          x3::Command_Order_StopX2 == msg_type) {
        uint64_t msg_id = 0;
        if (pack_msg.has_addition_() && pack_msg.addition_().has_frame_()) {
          msg_id = pack_msg.addition_().frame_().sequence_id_();
          LOGD << "msg_id: " << msg_id;
        }
        ConstructMsgForCmd(InfoMsg, msg_id);
      } else {
        LOGE << "Msg sub Type Error, type is " << msg_type;
        return -1;
      }
    }

    if (InfoMsg.has_status_()) {
      auto msg_type = InfoMsg.status_().GetTypeName();
      LOGI << "msg_type is " << msg_type;
      auto msg_stat = InfoMsg.status_().run_status_();
      LOGI << "msg_stat is " << static_cast<int>(msg_stat)
           << "  ts " << InfoMsg.status_().timestamp_();
    }
  } else if (pack_msg_parse && pack_msg.has_addition_() &&
      pack_msg.addition_().has_frame_() &&
      image.ParseFromString(pack_msg.content_())) {
    return OnGetAPImage(image, pack_msg.addition_().frame_().sequence_id_());
  } else {
    LOGE << "parse msg fail";
    LOGE << "pack_msg.has_addition_() " << pack_msg.has_addition_();
    LOGE << "pack_msg.addition_().has_frame_() "
    << pack_msg.addition_().has_frame_();
    LOGE << "pack_msg.addition_().frame id: " <<
    pack_msg.addition_().frame_().sequence_id_();
    LOGE << "image ParseFromString " <<
    image.ParseFromString(pack_msg.content_());
    return -1;
  }

  return 0;
}

int MonitorControlPlugin::OnGetSmarterResult(const XProtoMessagePtr& msg) {
  if (!is_running_ || !auto_start_ || !enable_vot_) {
    return 0;
  }
  auto smartmsg = std::static_pointer_cast<SmartMessage>(msg);
  HOBOT_CHECK(smartmsg);
  auto frame_id = smartmsg->frame_id_;
  LOGD << "smart frame_id:" << frame_id;
  auto sp_smart = ConstructVotSmartData(smartmsg);
  std::lock_guard<std::mutex> lg(mut_cache_);

  if (cache_vot_data_.find(frame_id) == cache_vot_data_.end()) {
    // recv vio msg faster than smart msg
    // if run here, noting vio plugin do not send drop frame
    auto cache_vio_smart = std::make_shared<VotData_t>();
    cache_vio_smart->sp_smart_pb_ = sp_smart;
    cache_vot_data_[frame_id] = cache_vio_smart;
  } else {
    // filter ware plugin msg
    if (!cache_vot_data_[frame_id]->sp_smart_pb_) {
      cache_vot_data_[frame_id]->sp_smart_pb_ = sp_smart;
    }
  }
  cv_.notify_one();
  LOGV << "cache_vio_smart_ size:" << cache_vot_data_.size();

  return 0;
}

int MonitorControlPlugin::OnGetVioResult(const XProtoMessagePtr& msg) {
  if (!is_running_ || !auto_start_ || !enable_vot_) {
    return 0;
  }

  auto valid_frame = std::static_pointer_cast<VioMessage>(msg);
  bool is_drop_frame = false;
  if (valid_frame == nullptr) {
    LOGE << "valid_frame is null";
    return -1;
  } else {
    if (TYPE_IMAGE_MESSAGE == valid_frame->type()) {
      is_drop_frame = false;
    } else if (TYPE_DROP_IMAGE_MESSAGE == valid_frame->type()) {
      is_drop_frame = true;
    } else if (TYPE_INFO_IMAGE_MESSAGE == valid_frame->type_) {
      LOGD << "recv msg id " << valid_frame->sequence_id_;
      auto &inst = SynMsgHandleManage<XProtoMessagePtr>::GetInstance();
      inst.SetPromise(valid_frame->sequence_id_, msg);
      return 0;
    } else {
      LOGE << "invalid type " << valid_frame->type();
      is_drop_frame = false;
    }

    if (valid_frame->image_.empty()) {
      LOGE << "valid_frame->image_.empty() is empty";
      return 1;
    } else {
      if (valid_frame->image_.front() == nullptr) {
        LOGE << "valid_frame->image_.front()is empty";
        return -1;
      }
    }
  }
  HOBOT_CHECK(valid_frame && !valid_frame->image_.empty() &&
              valid_frame->image_.front());
  LOGD << "type:" << valid_frame->type()
       << " seq id:" << valid_frame->sequence_id_
       << " ts:" << valid_frame->time_stamp_;
  auto frame_id = valid_frame->image_.front()->frame_id_;
  LOGI << "vio frame_id:" << frame_id;
  std::shared_ptr<xstream::RawDataImageFrame> sp_img = nullptr;
  sp_img = ConstructVotImgData(valid_frame);
  std::lock_guard<std::mutex> lg(mut_cache_);

  if (cache_vot_data_.find(frame_id) == cache_vot_data_.end()) {
    auto cache_vio_smart = std::make_shared<VotData_t>();
    cache_vio_smart->is_drop_frame_ = is_drop_frame;
    cache_vio_smart->sp_img_ = sp_img;
    cache_vot_data_[frame_id] = cache_vio_smart;
  } else {
    cache_vot_data_[frame_id]->is_drop_frame_ = is_drop_frame;
    cache_vot_data_[frame_id]->sp_img_ = sp_img;
  }

  cv_.notify_one();
  LOGV << "cache_vio_smart_ size:" << cache_vot_data_.size();

  // dump pym layer 0 img
  if (enable_dump_img_) {
    std::string pic_name {save_dump_img_path_ + "/pym0" +
                "_" + std::to_string(valid_frame->image_.front()->frame_id_) +
                "_" + std::to_string(valid_frame->image_.front()->Width(0)) +
                "_" + std::to_string(valid_frame->image_.front()->Height(0)) +
                ".nv12"};
    LOGD << "pic_name:" << pic_name;
    std::ofstream ofs(pic_name);
    if (ofs.is_open()) {
      ofs.write(reinterpret_cast<char*>(valid_frame->image_.front()->Data(0)),
                valid_frame->image_.front()->DataSize(0));
      ofs.write(reinterpret_cast<char*>(valid_frame->image_.front()->DataUV(0)),
                valid_frame->image_.front()->DataUVSize(0));
    }
  }

  return 0;
}

std::string MonitorControlMessage::Serialize() {
  LOGD << "serialize msg_id_:" << msg_id_;
  x3::MessagePack pack;
  pack.set_flow_(x3::MessagePack_Flow_CP2AP);
  pack.set_type_(x3::MessagePack_Type::MessagePack_Type_kXConfig);
  pack.mutable_addition_()->mutable_frame_()->set_sequence_id_(msg_id_);

  x3::InfoMessage info;
  auto ack =
      std::dynamic_pointer_cast<SmarterConfigData>(message_data_)->status_
          ? x3::Response_Ack::Response_Ack_Success
          : x3::Response_Ack::Response_Ack_Fail;
  info.mutable_response_()->set_ack_(ack);
  pack.set_content_(info.SerializeAsString());

  return pack.SerializeAsString();
}
}  // namespace xproto
