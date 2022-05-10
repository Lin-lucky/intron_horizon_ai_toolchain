/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: example_do_det.cpp
 * @Brief:
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-05-22 14:27:05
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-05-22 15:18:10
 */

#include <stdint.h>

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "cnn_method/cnn_method.h"
#include "cnn_method/util/util.h"
#include "fasterrcnn_method/fasterrcnn_method.h"
#include "json/json.h"
#include "xstream/method.h"
#include "xstream/method_factory.h"
#include "xstream/vision_type.h"
#include "xstream/xstream_sdk.h"

// #include "./vio_wrapper_global.h"


typedef std::shared_ptr<xstream::ImageFrame> ImageFramePtr;
struct PyramidResult {
  img_info_t result_info;
};

static void Usage() {
  std::cout << "./example do_det xstream_cfg_file fb_cfg img_list" << std::endl;
}

void PrintFace(std::vector<xstream::BaseDataPtr> &result) {
  auto rois = std::static_pointer_cast<xstream::BaseDataVector>(result[0]);
  for (size_t roi_idx = 0; roi_idx < rois->datas_.size(); roi_idx++) {
    auto roi = std::static_pointer_cast<xstream::BBox>(rois->datas_[roi_idx]);
    std::cout << "x1:" << roi->x1_ << " y1:" << roi->y1_ << " x2:" << roi->x2_
              << " y2:" << roi->y2_ << std::endl;
  }
}

int DoDet(int argc, char **argv) {
  if (argc < 4) {
    Usage();
    return 1;
  }
  std::string cfg_file(argv[1]);
  std::string fb_cfg(argv[2]);
  std::string img_list(argv[3]);

  xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();
  flow->SetConfig("config_file", cfg_file.c_str());
  flow->SetConfig("profiler", "on");
  flow->SetConfig("profiler_file", "./profiler.txt");
  flow->Init();

  HbVioFbWrapperGlobal fb_handle(fb_cfg);
  fb_handle.Init();

  std::ifstream img_list_file(img_list);
  std::string img_path;
  while (getline(img_list_file, img_path)) {
    uint32_t effective_w, effective_h;
    auto py_img = fb_handle.GetImgInfo(img_path, &effective_w, &effective_h);

    xstream::InputDataPtr inputdata(new xstream::InputData());
    py_img->name_ = "pyramid";
    inputdata->datas_.push_back(
        std::static_pointer_cast<xstream::BaseData>(py_img));
    auto out = flow->SyncPredict(inputdata);
    PrintFace(out->datas_);
    fb_handle.FreeImgInfo(py_img);
  }
  delete flow;
  return 0;
}
