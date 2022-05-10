/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @brief     FasterRCNN Method
 * @author    yaoyao.sun
 * @date      2019.04.12
 */

#include "fasterrcnn_method/fasterrcnn_method.h"

#include <stdint.h>

#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "fasterrcnn_method/fasterrcnn_imp.h"
#include "fasterrcnn_method/result.h"
#include "hobotlog/hobotlog.hpp"
#include "xstream/xstream_data.h"

namespace xstream {

int FasterRCNNMethod::Init(const std::string &config_file) {
  LOGD << "Init FasterRCNNMethod";
  faster_rcnn_imp_.reset(new faster_rcnn_method::FasterRCNNImp());
  int ret = faster_rcnn_imp_->Init(config_file);
  HOBOT_CHECK(ret == 0) << "faster_rcnn_method init failed.";
  method_param_.reset(new FasterRCNNParam("FasterRCNNMethod"));
  LOGD << "Finish FasterRCNNMethod Init";
  return 0;
}

int FasterRCNNMethod::UpdateParameter(InputParamPtr ptr) {
  faster_rcnn_imp_->UpdateParameter(ptr);
  return 0;
}

InputParamPtr FasterRCNNMethod::GetParameter() const {
  return method_param_;
}

std::string FasterRCNNMethod::GetVersion() const {
  return faster_rcnn_imp_->GetVersion();
}

std::vector<BaseDataPtr>
FasterRCNNMethod::DoProcess(const std::vector<BaseDataPtr> &input,
                            const InputParamPtr &param) {
  LOGD << "Run FasterRCNNMethod";
  LOGD << "input's size: " << input.size();
  std::vector<BaseDataPtr> output;
  faster_rcnn_imp_->RunSingleFrame(input, output);
  return output;
}



void FasterRCNNMethod::Finalize() {
  faster_rcnn_imp_->Finalize();
}

}   // namespace xstream
