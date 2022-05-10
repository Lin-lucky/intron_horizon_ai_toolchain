/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2019-08-01 20:38:52
 * @Version: v0.0.1
 * @Brief: smart_plugin impl based on xstream.
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-09-29 05:04:11
 */

#include "smart_plugin/smart_plugin.h"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "media_codec/media_codec_manager.h"
#include "attribute_convert.h"
#include "hobotlog/hobotlog.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "smart_plugin/convert.h"
#include "smart_plugin/convertpb.h"
#include "smart_plugin/gesture_threshold.h"
#include "smart_plugin/key_point_convertor.h"
#include "smart_plugin/merge_hand_body.h"
#include "smart_plugin/runtime_monitor.h"
#include "smart_plugin/smart_config.h"
#include "smart_plugin/utils/time_helper.h"
#include "smart_plugin/workflow_data_type.h"
#include "xproto/message/flowmsg.h"
#include "xproto/message/msg_registry.h"
#include "xproto/msg_type/protobuf/x3.pb.h"
#include "xproto/msg_type/vio_message.h"
#include "xproto/plugin/xpluginasync.h"
#include "xstream/xstream_world.h"

#ifdef USE_MC
#include "transport_message/monitor_control_message.h"
#include "smart_plugin/method_configer.h"
#include "transport_message/uvc_message.h"
#endif
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_SMART_LEGIBLE_MESSAGE);
namespace xproto {

using xstream::BBox;
using xstream::Segmentation;

using xproto::message::VioMessage;
using ImageFramePtr = std::shared_ptr<xstream::ImageFrame>;
using horizon::vision::MediaCodecManager;
using xproto::message::Target;
using xproto::message::TargetPtr;
using xstream::InputDataPtr;
using xstream::OutputDataPtr;
using xstream::Profiler;
using xstream::XStreamSDK;

extern GestureThreshold gesture_thr;
extern bool gesture_as_event;
extern int dist_calibration_w;
extern float dist_fit_factor;
extern float dist_fit_impower;
extern bool dist_smooth;


SmartPlugin::SmartPlugin(const std::string &config_file) {
  config_file_ = config_file;
  LOGI << "smart config file:" << config_file_;
  monitor_.reset(new RuntimeMonitor());
  Json::Value cfg_jv;
  std::ifstream infile(config_file_);
  infile >> cfg_jv;
  config_.reset(new JsonConfigWrapper(cfg_jv));
  ParseConfig();
  GetWorkflowTargetsConfig();
  smart_parse_thread_.CreatThread(1);
  jpg_encode_thread_.CreatThread(1);
  map_thread_.CreatThread(1);
  codec_inited_ = false;
  chn_ = -1;
}

void SmartPlugin::ParseConfig() {
  xstream_workflow_cfg_file_ =
      config_->GetSTDStringValue("xstream_workflow_file");
  sdk_monitor_interval_ = config_->GetIntValue("time_monitor");
  enable_profile_ = config_->GetBoolValue("enable_profile");
  profile_log_file_ = config_->GetSTDStringValue("profile_log_path");
  if (config_->HasMember("enable_result_to_json")) {
    result_to_json_ = config_->GetBoolValue("enable_result_to_json");
  }
  if (config_->HasMember("dump_result")) {
    dump_result_ = config_->GetBoolValue("dump_result");
  }
  LOGI << "xstream_workflow_file:" << xstream_workflow_cfg_file_;
  LOGI << "enable_profile:" << enable_profile_
       << ", profile_log_path:" << profile_log_file_;

  dist_calibration_w =
          config_->GetIntValue("distance_calibration_width");
  dist_fit_factor = config_->GetFloatValue("distance_fit_factor");
  dist_fit_impower = config_->GetFloatValue("distance_fit_impower");
  dist_smooth = config_->GetBoolValue("distance_smooth", true);
  LOGI << "dist_calibration_w:" << dist_calibration_w
       << " dist_fit_factor:" << dist_fit_factor
       << " dist_fit_impower:" << dist_fit_impower
       << " dist_smooth:" << dist_smooth;
  hand_id_merge_ = config_->GetBoolValue("hand_id_merge", false);
  convert_keypoint_format_ =
      config_->GetBoolValue("convert_keypoint_format", false);
  if (config_->HasMember("codec_param")) {
    auto codec_config = config_->GetSubConfig("codec_param");
    codec_param_.use_vb_ = codec_config->GetIntValue("use_vb");
    codec_param_.jpeg_quality_ = codec_config->GetIntValue("jpeg_quality");
    codec_param_.frame_buf_depth_ =
        codec_config->GetIntValue("frame_buf_depth");
    codec_param_.is_cbr_ = codec_config->GetIntValue("is_cbr");
    codec_param_.bitrate_ = codec_config->GetIntValue("bitrate");
    codec_param_.jpg_encode_time_ =
        codec_config->GetIntValue("jpg_encode_time");
  }
}

void SmartPlugin::GetWorkflowTargetsConfig() {
  if (xstream_workflow_cfg_file_.empty()) {
    return;
  }
  Json::Value cfg_jv;
  std::ifstream infile(xstream_workflow_cfg_file_);
  infile >> cfg_jv;
  auto target_config = std::make_shared<JsonConfigWrapper>(cfg_jv);
  if (!target_config) {
    LOGE << "Create Target config failed.";
    return;
  }

  auto name_prefix = [](const std::string name,
                        const char separator) -> std::string {
    auto pos = name.find(separator);
    if (pos == std::string::npos) return "";

    return name.substr(0, pos);
  };

  auto name_postfix = [](const std::string name,
                         const char separator) -> std::string {
    auto pos = name.rfind(separator);
    if (pos == std::string::npos) return "";

    return name.substr(pos + 1);
  };

  auto name_trim = [](std::string &name) {
    if (name.empty()) {
      return;
    }
    name.erase(0, name.find_first_not_of(" "));
    name.erase(name.find_last_not_of(" ") + 1);
  };

  if (target_config->HasMember("declaration")) {
    auto decla_config = target_config->GetSubConfig("declaration");
    LOGD << "Prase declaration";
    if (decla_config && decla_config->HasMember("targets")) {
      auto targets_config = decla_config->GetSubConfig("targets");
      auto target_keys = targets_config->GetJsonKeys();
      LOGD << "targets_key num =  " << target_keys.size();
      for (auto target_name : target_keys) {
        LOGD << "targets_key: " << target_name;
        auto values = targets_config->GetSTDStringArray(target_name);
        for (auto v : values) {
          auto output_name = name_prefix(v, ':');
          name_trim(output_name);  // trim space
          auto output_type = name_postfix(v, ':');
          name_trim(output_type);  // trim space
          workflow_targets_[output_name] = target_name;
          workflow_output_types_[output_name] = output_type;
          LOGD << "filed: " << output_name << ", " << output_type << ".";
        }
      }
    }
  }
}

int SmartPlugin::Init() {
  // init for xstream sdk
  sdk_.reset(xstream::XStreamSDK::CreateSDK());
  sdk_->SetConfig("config_file", xstream_workflow_cfg_file_);
  if (sdk_->Init() != 0) {
    return -1;
  }
  if (sdk_monitor_interval_ != 0) {
    sdk_->SetConfig("time_monitor", std::to_string(sdk_monitor_interval_));
  }
  if (enable_profile_) {
    sdk_->SetConfig("profiler", "on");
    sdk_->SetConfig("profiler_file", profile_log_file_);
    if (config_->HasMember("profiler_time_interval")) {
      int time_interval = config_->GetIntValue("profiler_time_interval");
      sdk_->SetConfig("profiler_time_interval", std::to_string(time_interval));
    }
  }
  sdk_->SetCallback(
      std::bind(&SmartPlugin::OnCallback, this, std::placeholders::_1));

  RegisterMsg(TYPE_IMAGE_MESSAGE,
              std::bind(&SmartPlugin::Feed, this, std::placeholders::_1));
#ifdef USE_MC
  MethodConfiger::Get()->weak_sdk_ = sdk_;
  RegisterMsg(TYPE_TRANSPORT_MESSAGE,
      std::bind(&SmartPlugin::OnApInfoMessage, this, std::placeholders::_1));
#endif
  return XPluginAsync::Init();
}

#ifdef USE_MC
struct APCfgRespMessage: public xproto::message::APRespMessage{
  APCfgRespMessage(bool status, uint64_t seq_id, const x3::Config &config)
  : APRespMessage(seq_id), status_(status) {
    config.SerializeToString(&proto_);
  }
  std::string Serialize();
  bool status_;
};

std::string APCfgRespMessage::Serialize() {
  LOGE << "serialize msg_id_:" << sequence_id_;
  x3::MessagePack pack;
  pack.set_flow_(x3::MessagePack_Flow_CP2AP);
  pack.set_type_(x3::MessagePack_Type::MessagePack_Type_kXConfig);
  pack.mutable_addition_()->mutable_frame_()->set_sequence_id_(sequence_id_);

  x3::InfoMessage info;
  auto ack = status_
             ? x3::Response_Ack::Response_Ack_Success
             : x3::Response_Ack::Response_Ack_Fail;
  info.mutable_response_()->set_ack_(ack);
  if (!proto_.empty() && status_)
    info.add_config_()->ParseFromString(proto_);
  pack.set_content_(info.SerializeAsString());
  return pack.SerializeAsString();
}

int SmartPlugin::OnApInfoMessage(const XProtoMessagePtr& msg) {
  int ret = -1;
  auto uvc_msg = std::static_pointer_cast<
      basic_msgtype::TransportMessage>(msg);
  x3::MessagePack pack_msg;
  x3::InfoMessage InfoMsg;
  auto pack_msg_parse = pack_msg.ParseFromString(uvc_msg->proto_);
  if (pack_msg_parse &&
      pack_msg.type_() == x3::MessagePack_Type::MessagePack_Type_kXConfig &&
      InfoMsg.ParseFromString(pack_msg.content_()) &&
      InfoMsg.config__size() > 0) {
    x3::Config *config_proto = InfoMsg.mutable_config_(0);
    auto sequence_id = pack_msg.addition_().frame_().sequence_id_();
    ret = MethodConfiger::Get()->HandleAPConfig(*config_proto);
    auto response = std::make_shared<APCfgRespMessage>(
        ret == 0,
        sequence_id,
        *config_proto);
    PushMsg(response);
  }
  return 0;
}
#endif

int SmartPlugin::Feed(XProtoMessagePtr msg) {
  if (!run_flag_) {
    return 0;
  }
  // feed video frame to xstreamsdk.
  // 1. parse valid frame from msg
  LOGI << "smart plugin got one msg";
  auto valid_frame = std::static_pointer_cast<VioMessage>(msg);
  jpg_encode_thread_.PostTask(std::bind(&SmartPlugin::EncodeJpg, this, msg));

  xstream::InputDataPtr input =
      Convertor::ConvertInput(valid_frame.get(), GetWorkflowInputImageName());
  auto xstream_input_data =
      std::static_pointer_cast<xstream::ImageFrame>(
          input->datas_[0]);
  auto frame_id = xstream_input_data->frame_id_;
  SmartInput *input_wrapper = new SmartInput();
  input_wrapper->frame_info = valid_frame;
  input_wrapper->context = input_wrapper;
  monitor_->PushFrame(input_wrapper);
#ifdef USE_MC
  MethodConfiger::Get()->BuildInputParam(input);
#endif
  if (sdk_->AsyncPredict(input) < 0) {
    auto intput_frame = monitor_->PopFrame(frame_id);
    delete static_cast<SmartInput *>(intput_frame.context);
    LOGW << "AsyncPredict failed, frame_id = " << frame_id;
    return -1;
  }
  LOGI << "feed one task to xtream workflow";

  return 0;
}


int SmartPlugin::Start() {
  if (run_flag_) {
    return 0;
  }
  LOGI << "SmartPlugin Start";
  run_flag_ = true;
  root_.clear();
  map_thread_.PostTask(std::bind(&SmartPlugin::SmartImageMapProc, this));
  return 0;
}

int SmartPlugin::Stop() {
  if (!run_flag_) {
    return 0;
  }
  run_flag_ = false;
  map_smart_condition_.notify_one();

  while (sdk_->GetTaskNum() != 0 || jpg_encode_thread_.GetTaskNum() != 0 ||
         smart_parse_thread_.GetTaskNum() != 0 ||
         map_thread_.GetTaskNum() != 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (result_to_json_) {
    remove("smart_data.json");
    Json::StreamWriterBuilder builder;
    std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
    std::ofstream outputFileStream("smart_data.json");
    writer->write(root_, &outputFileStream);
  }
  CodecDeInit();
  LOGI << "SmartPlugin Stop";
  return 0;
}

int SmartPlugin::DeInit() {
  Profiler::Release();
  XPluginAsync::DeInit();
  return 0;
}

#ifdef DUMP_SNAP
static int SaveImg(const ImageFramePtr &img_ptr, const std::string &path) {
  if (!img_ptr) {
    return 0;
  }
  auto width = img_ptr->Width();
  auto height = img_ptr->Height();
  uint8_t *snap_data = static_cast<uint8_t *>(
          std::calloc(img_ptr->DataSize() + img_ptr->DataUVSize(),
          sizeof(uint8_t)));
  if (!snap_data) {
    return -1;
  }
  memcpy(snap_data, reinterpret_cast<uint8_t *>(img_ptr->Data()),
         img_ptr->DataSize());
  memcpy(snap_data + img_ptr->DataSize(),
         reinterpret_cast<uint8_t *>(img_ptr->DataUV()),
         img_ptr->DataUVSize());
  cv::Mat bgrImg;
  switch (img_ptr->pixel_format) {
    case kHorizonVisionPixelFormatNone: {
      case kHorizonVisionPixelFormatImageContainer: {
        return -1;
      }
      case kHorizonVisionPixelFormatRawRGB: {
        auto rgbImg = cv::Mat(height, width, CV_8UC3, snap_data);
        cv::cvtColor(rgbImg, bgrImg, CV_RGB2BGR);
        break;
      }
      case kHorizonVisionPixelFormatRawRGB565:
        break;
      case kHorizonVisionPixelFormatRawBGR: {
        bgrImg = cv::Mat(height, width, CV_8UC3, snap_data);
        break;
      }
      case kHorizonVisionPixelFormatRawGRAY: {
        auto cv_img = cv::Mat(height, width, CV_8UC1, snap_data);
        cv::cvtColor(cv_img, bgrImg, CV_GRAY2BGR);
        break;
      }
      case kHorizonVisionPixelFormatRawNV21: {
        break;
      }
      case kHorizonVisionPixelFormatRawNV12: {
        auto nv12Img = cv::Mat(height * 3 / 2, width, CV_8UC1, snap_data);
        cv::cvtColor(nv12Img, bgrImg, CV_YUV2BGR_NV12);
        break;
      }
      case kHorizonVisionPixelFormatRawI420: {
        auto i420Img = cv::Mat(height * 3 / 2, width, CV_8UC1, snap_data);
        cv::cvtColor(i420Img, bgrImg, CV_YUV2BGR_I420);
        break;
      }
      default:
        break;
    }
  }
  free(snap_data);
  cv::imwrite(path + ".png", bgrImg);
  return 0;
}

int DumpSnap(const XStreamSnapshotInfoPtr &snapshot_info, std::string dir) {
  char filename[50];
  snprintf(filename, sizeof(filename), "%s/FaceSnap_%d_%d_%li_%d_%li.yuv",
           dir.c_str(),
           snapshot_info->value->snap->Width(),
           snapshot_info->value->snap->Height(),
           snapshot_info->value->origin_image_frame->time_stamp,
           snapshot_info->value->track_id,
           snapshot_info->value->origin_image_frame->frame_id);

  // save yuv
  FILE *pfile = nullptr;
  auto data_size = snapshot_info->value->snap->DataSize();
  void * snap_data =
      reinterpret_cast<void *>(snapshot_info->value->snap->Data());
  if (snap_data && data_size) {
    pfile = fopen(filename, "w");
    if (!pfile) {
      std::cerr << "open file " << filename << " failed" << std::endl;
      return -1;
    }
    if (!fwrite(snap_data, data_size, 1, pfile)) {
      std::cout << "fwrite data to " << filename << " failed" << std::endl;
      fclose(pfile);
    }
    fclose(pfile);
  }

  // save bgr
  snprintf(filename, sizeof(filename), "%s/FaceSnap%li_%d_%li", dir.c_str(),
           snapshot_info->value->origin_image_frame->time_stamp,
           snapshot_info->value->track_id,
           snapshot_info->value->origin_image_frame->frame_id);
  std::string s_filename(filename);
  SaveImg(snapshot_info->value->snap, s_filename);


  return 0;
}
#endif
void SmartPlugin::OnCallback(xstream::OutputDataPtr xstream_out) {
  // On xstream async-predict returned,
  // transform xstream standard output to smart message.
  LOGI << "smart plugin got one smart result";
  HOBOT_CHECK(!xstream_out->datas_.empty()) << "Empty XStream Output";

  ImageFramePtr rgb_image = nullptr;

  for (const auto &output : xstream_out->datas_) {
    LOGD << output->name_ << ", type is " << output->type_;
    if (output->name_ == GetWorkflowInputImageName()) {
      rgb_image = std::dynamic_pointer_cast<ImageFrame>(output);
    }
#ifdef DUMP_SNAP
    if ("snap_list" == output->name_) {
      auto &data = output;
      if (data->error_code_ < 0) {
        LOGE << "data error: " << data->error_code_ << std::endl;
      }
      auto *psnap_data = dynamic_cast<BaseDataVector *>(data.get());
      if (!psnap_data->datas_.empty()) {
        for (const auto &item : psnap_data->datas_) {
          assert("BaseDataVector" == item->type_);
          //  get the item score
          auto one_target_snapshot_info =
                  std::static_pointer_cast<BaseDataVector>(item);
          for (auto &snapshot_info : one_target_snapshot_info->datas_) {
            auto one_snap =
                  std::static_pointer_cast<XStreamSnapshotInfo>(snapshot_info);
            if (one_snap->value->snap) {
              DumpSnap(one_snap, "./");
            }
          }
        }
      }
    }
#endif
  }

  if (hand_id_merge_) {
    MergeHandBody::Instance()->UpdateHandTrackID(xstream_out);
  }

  if (convert_keypoint_format_) {
    KeyPointConvertor::ConverKeyPoint(xstream_out);
  }

  auto smart_msg = CreateSmartMessage(xstream_out);
  smart_parse_thread_.PostTask(
      std::bind(&SmartPlugin::CreateSmartLegibleMessage, this, xstream_out));
  // Set origin input named "image" as output always.
  HOBOT_CHECK(rgb_image);
  smart_msg->channel_id_ = (rgb_image)->channel_id_;
  smart_msg->time_stamp_ = (rgb_image)->time_stamp_;
  smart_msg->frame_id_ = (rgb_image)->frame_id_;
  smart_msg->image_name_ = (rgb_image)->image_name_;
  LOGD << "smart result image name = " << smart_msg->image_name_;
  LOGI << "smart result frame_id = " << smart_msg->frame_id_ << std::endl;
  auto input = monitor_->PopFrame(smart_msg->frame_id_);
  auto vio_msg = input.vio_msg;

  delete static_cast<SmartInput *>(input.context);
  monitor_->FrameStatistic();
  smart_msg->frame_fps_ = monitor_->GetFrameFps();
  PushMsg(smart_msg);
  // smart_msg->Serialize();
  if (result_to_json_) {
    /// output structure data
    smart_msg->Serialize_Print(root_);
  }
  if (dump_result_) {
    smart_msg->Serialize_Dump_Result();
  }
}

void SmartPlugin::CreateSmartLegibleMessage(
    xstream::OutputDataPtr xstream_out) {
  LOGD << "SmartPlugin::CreateSmartLegibleMessage";
  if (!run_flag_) {
    LOGD << "Aleardy stop, SmartPlugin::CreateSmartLegibleMessage return";
    return;
  }
  ImageFramePtr rgb_image = nullptr;
  auto legible_message = std::make_shared<SmartLegibleMessage>();
  struct target_key {
    std::string category;
    size_t id;
    target_key(std::string category, size_t id) {
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
  std::map<target_key, TargetPtr, cmp_key> smart_target;

  for (const auto &output : xstream_out->datas_) {
    if (output->name_ == GetWorkflowInputImageName()) {
      rgb_image = std::dynamic_pointer_cast<ImageFrame>(output);
      HOBOT_CHECK(rgb_image);
      legible_message->paramid_img_ = rgb_image;
      legible_message->channel_id_ = (rgb_image)->channel_id_;
      legible_message->time_stamp_ = (rgb_image)->time_stamp_;
      legible_message->frame_id_ = (rgb_image)->frame_id_;
      legible_message->image_name_ = (rgb_image)->image_name_;
      continue;
    }
    auto real_output_name = output->name_;
    std::string real_output_type = "";
    if (workflow_output_types_.find(real_output_name) !=
        workflow_output_types_.end()) {
      real_output_type = workflow_output_types_[real_output_name];
    }
    std::string real_output_target = "";
    if (workflow_targets_.find(real_output_name) != workflow_targets_.end()) {
      real_output_target = workflow_targets_[real_output_name];
    }
    LOGD << output->name_ << ", type is " << real_output_type
         << ", real target name = " << real_output_target;
    if (real_output_type.length() == 0) {
      // 无类型的数据无法自动转换
      continue;
    }
    auto output_data =
        std::dynamic_pointer_cast<xstream::BaseDataVector>(output);
    if (!output_data) {
      LOGD << "output: " << output->name_
           << ", type is not xstream::BaseDataVector";
      continue;
    }
    LOGD << "data type: " << output->name_
         << ", data size: " << output_data->datas_.size();
    for (size_t i = 0; i < output_data->datas_.size(); ++i) {
      auto one_data = output_data->datas_[i];
      if (one_data->state_ != xstream::DataState::VALID) {
        // 无效数据
        continue;
      }
      target_key target_key(real_output_target, i);
      if (smart_target.find(target_key) == smart_target.end()) {  // 新target
        auto target = std::make_shared<Target>();
        target->face_feature_ = nullptr;
        target->face_pose_ = nullptr;
        target->map_seg_ = nullptr;
        if (real_output_type == kBBox) {
          auto one_box = std::dynamic_pointer_cast<xstream::BBox>(one_data);
          if (one_box) {
            target->track_id_ = one_box->id_;
            target->type_ = one_box->specific_type_;  // target类型用box类型替换
          }
        }
        if (target->type_.empty()) {
          target->type_ = real_output_target;
        }
        legible_message->smart_data_.targets_.push_back(target);
        smart_target[target_key] = target;
      }
      auto target = smart_target[target_key];
      one_data->name_ = output->name_;  // 透传name_
      if (real_output_type == kBBox) {
        auto one_box = std::dynamic_pointer_cast<xstream::BBox>(one_data);
        if (one_box) {
          target->boxs_.push_back(one_box);
        } else {
          LOGD << "Data type cast failed";
        }
      } else if (real_output_type == kLandMarks) {
        auto one_lmk = std::dynamic_pointer_cast<xstream::Landmarks>(one_data);
        if (one_lmk) {
          target->lmks_.push_back(one_lmk);
        } else {
          LOGD << "Data type cast failed";
        }
      } else if (real_output_type == kAttribute) {
        auto one_attr =
            std::dynamic_pointer_cast<xstream::Attribute_<int32_t>>(one_data);
        if (one_attr) {
          target->attributes_.push_back(one_attr);
        } else {
          LOGD << "Data type cast failed";
        }
      } else if (real_output_type == kSegmentation) {
        auto one_seg =
            std::dynamic_pointer_cast<xstream::Segmentation>(one_data);
        if (one_seg) {
          target->body_seg_.push_back(one_seg);
        } else {
          LOGD << "Data type cast failed";
        }
      } else if (real_output_type == kFeature) {
        auto one_feature =
            std::dynamic_pointer_cast<xstream::FloatFeature>(one_data);
        if (one_feature) {
          target->face_feature_ = one_feature;
        } else {
          LOGD << "Data type cast failed";
        }
      } else if (real_output_type == kPose3D) {
        auto one_pose = std::dynamic_pointer_cast<xstream::Pose3D>(one_data);
        if (one_pose) {
          target->face_pose_ = one_pose;
        } else {
          LOGD << "Data type cast failed";
        }
      } else {
        LOGD << "Not support this type: " << real_output_type;
      }
    }
  }  // for (const auto &output : xstream_out->datas_)
  // PushMsg(legible_message);
  {
    std::lock_guard<std::mutex> lock(map_smart_mutex_);
    smart_msgs_.push(legible_message);
  }
  map_smart_condition_.notify_one();
  LOGD << "CreateLegibleMessage suc.";
}

int SmartPlugin::CodecInit(int image_width, int image_height) {
  if (codec_inited_) {
    return 0;
  }
  /* 1. media codec init */
  /* 1.1 get media codec manager and module init */
  MediaCodecManager &manager = MediaCodecManager::Get();
  auto rv = manager.ModuleInit();
  HOBOT_CHECK(rv == 0);
  /* 1.2 get media codec venc chn */
  chn_ = manager.GetEncodeChn();
  /* 1.3 media codec venc chn init */
  int pic_width = image_width;
  int pic_height = image_height;
  int frame_buf_depth = codec_param_.frame_buf_depth_;
  int is_cbr = codec_param_.is_cbr_;
  int bitrate = codec_param_.bitrate_;

  rv = manager.EncodeChnInit(chn_, PT_JPEG, pic_width, pic_height,
                             frame_buf_depth, HB_PIXEL_FORMAT_NV12, is_cbr,
                             bitrate);
  HOBOT_CHECK(rv == 0);
  /* 1.4 set media codec venc jpg chn qfactor params */
  rv = manager.SetUserQfactorParams(chn_, codec_param_.jpeg_quality_);
  HOBOT_CHECK(rv == 0);
  /* 1.5 set media codec venc jpg chn qfactor params */
  rv = manager.EncodeChnStart(chn_);
  HOBOT_CHECK(rv == 0);
  /* 1.6 alloc media codec vb buffer init */
  if (codec_param_.use_vb_) {
    int vb_num = frame_buf_depth;
    int pic_stride = image_width;
    int pic_size = pic_stride * pic_height * 3 / 2;  // nv12 format
    int vb_cache_enable = 1;
    rv = manager.VbBufInit(chn_, pic_width, pic_height, pic_stride, pic_size,
                           vb_num, vb_cache_enable);
    HOBOT_CHECK(rv == 0);
  }
  codec_inited_ = true;
  return 0;
}

void SmartPlugin::CodecDeInit() {
  if (!codec_inited_) {
    return;
  }
  /* 3. media codec deinit */
  /* 3.1 media codec chn stop */
  MediaCodecManager &manager = MediaCodecManager::Get();
  manager.EncodeChnStop(chn_);
  /* 3.2 media codec chn deinit */
  manager.EncodeChnDeInit(chn_);
  /* 3.3 media codec vb buf deinit */
  if (codec_param_.use_vb_) {
    manager.VbBufDeInit(chn_);
  }
  /* 3.4 media codec module deinit */
  manager.ModuleDeInit();
}

void SmartPlugin::EncodeJpg(XProtoMessagePtr msg) {
  int rv;
  cv::Mat yuv_img;
  std::vector<uchar> img_buf;
  VideoEncodeSourceBuffer *frame_buf = nullptr;
  VideoEncodeSourceBuffer src_buf = {0};
  VideoEncodeStreamBuffer *stream_buf = nullptr;
  if (!run_flag_) {
    LOGD << "Aleardy stop, SmartPlugin::EncodeJpg return";
    return;
  }
  LOGI << "SmartPlugin EncodeJpg";
  auto frame = std::dynamic_pointer_cast<VioMessage>(msg);
  HOBOT_CHECK(frame);
  VioMessage *vio_msg = frame.get();
  auto timestamp = frame->time_stamp_;
  // get pyramid size
  auto pym_image = vio_msg->image_[0];
  int origin_image_width = pym_image->img_.down_scale[0].width;  // 取原图
  int origin_image_height = pym_image->img_.down_scale[0].height;
  if (!codec_inited_) {
    CodecInit(origin_image_width, origin_image_height);
  }
  /* 2. start encode yuv to jpeg */
  /* 2.1 get media codec vb buf for store src yuv data */
  MediaCodecManager &manager = MediaCodecManager::Get();
  if (codec_param_.use_vb_) {
    rv = manager.GetVbBuf(chn_, &frame_buf);
    HOBOT_CHECK(rv == 0);
  } else {
    frame_buf = &src_buf;
    memset(frame_buf, 0x00, sizeof(VideoEncodeSourceBuffer));
  }
  frame_buf->frame_info.pts = frame->time_stamp_;
  /* 2.2 get src yuv data */
  rv = GetYUV(frame_buf, vio_msg, 0, codec_param_.use_vb_);
  HOBOT_CHECK(rv == 0);
  if (0 == rv) {
    /* 2.3. encode yuv data to jpg */
    auto ts0 = Timer::current_time_stamp();
    rv = manager.EncodeYuvToJpg(chn_, frame_buf, &stream_buf);
    if (codec_param_.jpg_encode_time_ == 1) {
      auto ts1 = Timer::current_time_stamp();
      LOGW << "******Encode yuv to jpeg cost: " << ts1 - ts0 << "ms";
    }
    RawDataImageFramePtr raw_image(new xstream::RawDataImageFrame(),
                                   [&](xstream::RawDataImageFrame *p) {
                                     if (p) {
                                       if (p->data_) {
                                         LOGD << "delete rawdata image frame";
                                         std::free(p->data_);
                                         p->data_ = NULL;
                                       }
                                       delete (p);
                                       p = nullptr;
                                     }
                                   });
    if (rv == 0) {
      auto data_ptr = stream_buf->stream_info.pstPack.vir_ptr;
      auto data_size = stream_buf->stream_info.pstPack.size;
      raw_image->pixel_format_ =
          xstream::kHorizonVisionPixelFormatImageContainer;
      raw_image->data_ = static_cast<uint8_t *>(calloc(1, data_size));
      raw_image->image_size_ = data_size;
      raw_image->time_stamp_ = frame->image_[0]->time_stamp_;
      raw_image->channel_id_ = frame->image_[0]->channel_id_;
      raw_image->frame_id_ = frame->image_[0]->frame_id_;
      raw_image->image_name_ = frame->image_[0]->image_name_;
      raw_image->width_ = origin_image_width;
      raw_image->height_ = origin_image_height;
      memcpy(raw_image->data_, data_ptr, data_size);
      /* dump jpg picture */
#if 0
      static int dmp_jpg_num = 20;
      if (dmp_jpg_num-- > 0) {
        static int frame_id = 0;
        std::string file_name =
            "out_stream_" + std::to_string(frame_id++) + ".jpg";
        std::fstream fout(file_name, std::ios::out | std::ios::binary);
        fout.write((const char *)raw_image->data_, raw_image->image_size_);
        fout.close();
      }
#endif
    } else {
      raw_image->pixel_format_ =
          xstream::kHorizonVisionPixelFormatImageContainer;
      raw_image->image_size_ = 0;  // 编码失败数据为0
      raw_image->time_stamp_ = frame->image_[0]->time_stamp_;
      raw_image->channel_id_ = frame->image_[0]->channel_id_;
      raw_image->frame_id_ = frame->image_[0]->frame_id_;
      raw_image->image_name_ = frame->image_[0]->image_name_;
      LOGE << "X3 media codec jpeg encode failed!";
    }
    {
      std::lock_guard<std::mutex> lock(map_smart_mutex_);
      jpg_datas_.push(raw_image);
    }
    map_smart_condition_.notify_one();
    /* 2.4 free jpg stream buf */
    if (stream_buf != nullptr) {
      rv = manager.FreeStream(chn_, stream_buf);
      HOBOT_CHECK(rv == 0);
    }
    /* 2.5 free media codec vb buf */
    if (codec_param_.use_vb_) {
      rv = manager.FreeVbBuf(chn_, frame_buf);
      HOBOT_CHECK(rv == 0);
    }
  }
  return;
}

void SmartPlugin::SmartImageMapProc() {
  static uint64_t pre_frame_id = 0;
  while (run_flag_) {
    std::unique_lock<std::mutex> lock(map_smart_mutex_);
    map_smart_condition_.wait(lock);
    if (jpg_datas_.size() > 3 || smart_msgs_.size() > 3) {
      LOGW << "SmartImageMapProc, jpg_datas size = " << jpg_datas_.size()
           << ", smart_msg size = " << smart_msgs_.size();
    }
    if (!run_flag_) {
      break;
    }
    while (!smart_msgs_.empty() && !jpg_datas_.empty()) {
      auto msg = smart_msgs_.top();
      auto frame = jpg_datas_.top();
      if (msg->time_stamp_ == frame->time_stamp_) {
        msg->background_img_ = frame;
        PushMsg(msg);
        pre_frame_id = msg->frame_id_;
        smart_msgs_.pop();
        jpg_datas_.pop();
      } else {
        // avoid smart or image result lost
        while (smart_msgs_.size() > cache_size_) {
          auto msg_inner = smart_msgs_.top();
          auto frame_inner = jpg_datas_.top();
          if (msg_inner->time_stamp_ < frame_inner->time_stamp_) {
            // 消息对应的图片一直没有过来，删除消息并push到总线
            smart_msgs_.pop();
            PushMsg(msg_inner);  // 不带原图信息
            pre_frame_id = msg_inner->frame_id_;
          } else {
            break;
          }
        }
        while (jpg_datas_.size() > cache_size_) {
          auto msg_inner = smart_msgs_.top();
          auto frame_inner = jpg_datas_.top();
          if (frame_inner->time_stamp_ < msg_inner->time_stamp_) {
            // 图像对应的消息一直没有过来，删除图像
            jpg_datas_.pop();
          } else {
            break;
          }
        }
        break;
      }
    }
    if (smart_msgs_.size() > cache_size_) {
      LOGF << "smartplugin has cache smart message nun > " << cache_size_;
    }
    if (jpg_datas_.size() > cache_size_) {
      LOGF << "smartplugin has cache image nun > " << cache_size_;
    }
  }
}

int SmartPlugin::GetYUV(VideoEncodeSourceBuffer *frame_buf, VioMessage *vio_msg,
                        int level, int use_vb) {
  LOGI << "SmartPlugin GetYUV: " << __FUNCTION__;
  if (!vio_msg || vio_msg->num_ == 0) return -1;
  auto pym_image = vio_msg->image_[0];
  auto height = pym_image->img_.down_scale[level].height;
  auto width = pym_image->img_.down_scale[level].width;
  auto stride = pym_image->img_.down_scale[level].step;
  auto y_vaddr = pym_image->img_.down_scale[level].y_vaddr;
  auto y_paddr = pym_image->img_.down_scale[level].y_paddr;
  auto c_vaddr = pym_image->img_.down_scale[level].c_vaddr;
  auto c_paddr = pym_image->img_.down_scale[level].c_paddr;
  HOBOT_CHECK(height) << "width = " << width << ", height = " << height;
  auto img_y_size = height * stride;
  auto img_uv_size = img_y_size / 2;

  if (use_vb) {
    HOBOT_CHECK(frame_buf != nullptr);
    HOBOT_CHECK(frame_buf->frame_info.vir_ptr[0] != NULL);
    HOBOT_CHECK(frame_buf->frame_info.vir_ptr[1] != NULL);
    memcpy(frame_buf->frame_info.vir_ptr[0],
           reinterpret_cast<uint8_t *>(y_vaddr), img_y_size);
    memcpy(frame_buf->frame_info.vir_ptr[1],
           reinterpret_cast<uint8_t *>(c_vaddr), img_uv_size);
  } else {
    frame_buf->frame_info.width = width;
    frame_buf->frame_info.height = height;
    frame_buf->frame_info.stride = stride;
    frame_buf->frame_info.size = stride * height * 3 / 2;
    frame_buf->frame_info.vir_ptr[0] = reinterpret_cast<char *>(y_vaddr);
    frame_buf->frame_info.phy_ptr[0] = (uint32_t)y_paddr;
    frame_buf->frame_info.vir_ptr[1] = reinterpret_cast<char *>(c_vaddr);
    frame_buf->frame_info.phy_ptr[1] = (uint32_t)c_paddr;
    frame_buf->frame_info.pix_format = HB_PIXEL_FORMAT_NV12;
  }

#if 0  // dump yuv data
  static bool first = true;
  if (first) {
      static int frame_id = 0;
      std::string file_name = "out_stream_" +
          std::to_string(frame_id++) + ".yuv";
      std::fstream fout(file_name, std::ios::out | std::ios::binary);
      fout.write((const char *)img_y_addr, img_y_size);
      fout.write((const char *)img_uv_addr, img_uv_size);
      fout.close();
      first = false;  // only dump a yuv
  }
#endif
#if 0  // debug test
  for (std::size_t i = 0; i < vio_msg->image_.size(); i++) {
    pym_image = vio_msg->image_[i];
    auto pipe_id = pym_image->channel_id;
    auto pym_buffer = static_cast<pym_buffer_t*>(pym_image->context);
    auto frame_id = pym_buffer->pym_img_info.frame_id;
    auto ts = pym_buffer->pym_img_info.time_stamp;
    LOGW << "pipe_id: " << pipe_id << " frame_id: "
      << frame_id << " ts: " << ts;
  }
#endif

  return 0;
}
}  // namespace xproto
