/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     Method interface of xsoul framework
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.15
 */

#include <unistd.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <cassert>
#include <string>
#include <fstream>
#include <memory>
#include <vector>

#include "xstream/xstream_sdk.h"
#include "xstream/xstream_data.h"
#include "xstream/vision_type.h"
#include "hobotlog/hobotlog.hpp"

using xstream::BBox;
using xstream::ImageFrame;

typedef xstream::Attribute_<uint32_t> XStreamUint32;
typedef std::shared_ptr<xstream::ImageFrame> ImageFramePtr;

struct inputContext {
  std::ofstream* pofs;
  std::string imgname;
};

class Callback {
 public:
  Callback() = default;

  ~Callback() = default;

  void OnCallback(const xstream::OutputDataPtr &output) {
    using xstream::BaseDataVector;
    std::cout << "======================" << std::endl;
    std::cout << "seq: " << output->sequence_id_ << std::endl;
    std::cout << "error_code: " << output->error_code_ << std::endl;
    std::cout << "error_detail_: " << output->error_detail_ << std::endl;
    std::cout << "datas_ size: " << output->datas_.size() << std::endl;

    assert(output->datas_.size() >= 2);
    auto &bbox_list = output->datas_[0];
  //  auto &disappeared_track_id_list = output->datas_[1];
    if (bbox_list->error_code_ < 0) {
      std::cout << "bbox_list data error: "
                << bbox_list->error_code_ << std::endl;
    }
    std::cout << "bbox_list data type_name : " << bbox_list->type_
              << " " << bbox_list->name_ << std::endl;
    auto bbox_data = std::static_pointer_cast<BaseDataVector>(bbox_list);
    std::cout << "bbox_list data data size: "
              << bbox_data->datas_.size() << std::endl;

    const inputContext* context =
            reinterpret_cast<const inputContext*>(output->context_);
    std::ofstream* ofs = context->pofs;
    *ofs << context->imgname;
    for (auto & pbbox : bbox_data->datas_) {
      assert("BBox" == pbbox->type_);
      auto bbox = std::static_pointer_cast<BBox>(pbbox);
      *ofs << " " << bbox->x1_
           << " " << bbox->y1_
           << " " << bbox->x2_
           << " " << bbox->y2_
           << " " << bbox->id_;
    }

    *ofs << std::endl;

    delete context;
  }
};

class MotMethodParam : public xstream::InputParam {
 public:
  MotMethodParam(const std::string &method_name,
      const std::string &json_config) : InputParam(method_name) {
    content_ = json_config;
    is_json_format_ = true;
  }
  std::string Format() override { return content_; };
 private:
  std::string content_ = "";
};

struct MyImageFrame : public xstream::ImageFrame {
  MyImageFrame() { type_ = "MyImageFrame"; }
  /// \~Chinese ????????????
  uint64_t Data() override { return 0; }
  /// \~Chinese UV????????????
  uint64_t DataUV() override { return 0; }
  /// \~Chinese ????????????
  uint32_t DataSize() override { return 0; }
  /// \~Chinese UV????????????
  uint32_t DataUVSize() override { return 0; }
  /// \~Chinese ??????
  uint32_t Width() override { return 1920; }
  /// \~Chinese ??????
  uint32_t Height() override { return 1080; }
  /// \~Chinese ??????
  uint32_t Stride() override { return 0; }
  /// \~Chinese uv??????
  uint32_t StrideUV() override { return 0; }
};


int ConstructPvMsgFromIpcStr(xstream::BaseDataVector *boxes,
                             const std::string &pv_str,
                             const std::vector<xstream::BBox>
                                     &black_area,
                             std::string* img_name, float det_thres,
                             float overlap_ratio) {
  std::istringstream ss(pv_str);
  ss >> *img_name;

  float x1 = 0, y1 = 0, x2 = 0, y2 = 0, score = 0;

  while (ss >> x1) {
    ss >> y1;
    ss >> x2;
    ss >> y2;
    ss >> score;
    // if (score >= det_thres) {
      std::shared_ptr<BBox> bbox(new BBox());
      bbox->type_ = "BBox";
      bbox->x1_ = x1;
      bbox->y1_ = y1;
      bbox->x2_ = x2;
      bbox->y2_ = y2;
      bbox->score_ = score;
      boxes->datas_.push_back(xstream::BaseDataPtr(bbox));
    // }
  }
  return 0;
}

void LogRollBackHandler(const char* filename, std::size_t size) {
  std::stringstream ss;
  // printf("\n filename:%s \n", filename);
  ss << "cp " << filename << " /userdata/log/log_backup.log";
  system(ss.str().c_str());
}

void LogConfig() {
  // ??????????????????
  // el::Configurations conf("../../config/hobot_log.conf");
  // el::Loggers::reconfigureAllLoggers(conf);
  // ????????????LoggingFlag::StrictLogFileSizeCheck??????MAX_LOG_FILE_SIZE????????????????????????
  // AddFlag(HOBOT_MAX_FILE_SIZE_FLAG);
  // ??????Fatal????????????????????????
  // AddFlag(HOBOT_DISABLE_ABORT_FATAL_FLAG);
  // ???????????????????????????
  // AddFlag(HOBOT_HIERARCHICAL_FLAG);
  // ??????????????????STL??????????????????,??????????????????????????? -DELPP_STL_LOGGING
  // AddFlag(HOBOT_NEW_LINE_CONTAINER_FLAG);
  // ?????????????????????<<"123"<<"456";??????123 456
  // AddFlag(HOBOT_AUTO_SPACING_FLAG);
  // ????????????????????????????????????????????????????????????,??????Debug?????????????????????
  // SetLogLevel(HOBOT_LOG_DEBUG);

  // ????????????LOGV
  // verbose?????????1???????????????verbose???1???LOGV?????????????????????????????????????????????0
  // SetVerboseLevel(1);
  // ????????????
  // RegisterRollBackHandler(LogRollBackHandler);
  // ??????crash??????,???????????????
  // AddFlag(HOBOT_CRASH_REASON_FLAG);
  // ??????CHECK????????????????????????????????????
  // AddFlag(HOBOT_DISABLE_ABORT_FATAL_FLAG);
}


int main(int argc, char const *argv[]) {
  // LogConfig();
  using xstream::BaseData;
  using xstream::BaseDataPtr;
  using xstream::BaseDataVector;
  using xstream::InputData;
  using xstream::InputDataPtr;

  // ?????????sdk
  xstream::XStreamSDK* flow = xstream::XStreamSDK::CreateSDK();
  Callback callback;
  flow->SetCallback(
      std::bind(&Callback::OnCallback, &callback, std::placeholders::_1));
  flow->SetConfig("config_file", "../../config/iou2_mot.json");
  flow->Init();

  InputDataPtr inputdata(new InputData());

  /* pass-through mode

  xstream::InputParamPtr input_param(
      new MotMethodParam("IOU_example", "pass-through"));
  inputdata->params_.push_back(input_param);

  */
  std::string data_pre = "x2_ipc";
  std::string det_file_path =
          "./" + data_pre +
          "_det_result_beta.txt";
  std::string mot_result_path =
                        "./" + data_pre +
                        "_cpp_result.txt";

  std::ofstream ofs(mot_result_path);

  std::ifstream postv_fn(det_file_path, std::ios::in);
  if (postv_fn.fail()) {
    std::cout << "Open track_result failed" << std::endl;
    return -1;
  }

  std::vector<xstream::BBox> black_areas;

  int frame_num = 0;
  int frame_rate = 40*1000;
  float det_thres = 0.91;
  float overlap_ratio = 0.8;

  std::string postv_str;
  while (getline(postv_fn, postv_str)) {
    frame_num += 1;

    std::string img_name = "";
    std::cout << "Processed " << frame_num << " images." << std::endl;

    auto img = std::make_shared<MyImageFrame>();
    img->time_stamp_ = frame_num * frame_rate;  //????????????

    img->type_ = "ImageFrame";
    img->name_ = "image";
    inputdata->datas_.push_back(BaseDataPtr(img));

    std::shared_ptr<BaseDataVector> face_head_box(new BaseDataVector);
    ConstructPvMsgFromIpcStr(face_head_box.get(), postv_str, black_areas,
                &img_name, det_thres, overlap_ratio);
    face_head_box->name_ = "face_head_box_list";
    inputdata->datas_.push_back(BaseDataPtr(face_head_box));

    // async MOTMethod
    inputContext* context = new inputContext;
    context->pofs = &ofs;
    context->imgname = img_name;
    inputdata->context_ = context;
    flow->AsyncPredict(inputdata);
    // sleep(1);

    // sync MOTMethod
    // auto out = flow->SyncPredict(inputdata);
    // callback.OnCallback(out);
  }

  sleep(2);
  ofs.close();
  delete flow;
  return 0;
}
