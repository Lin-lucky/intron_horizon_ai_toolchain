/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2019-08-02 04:05:52
 * @Version: v0.0.1
 * @Brief: implemenation of converter.
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-08-02 06:26:01
 */
#include "smart_plugin/convert.h"

#include <turbojpeg.h>

#include <string>

#include "hobotlog/hobotlog.hpp"
#include "xproto/msg_type/vio_message.h"
#include "xstream/xstream_world.h"
using ImageFramePtr = std::shared_ptr<xstream::ImageFrame>;
namespace xproto {

int Convertor::image_compress_quality = 50;

xstream::InputDataPtr Convertor::ConvertInput(
    const xproto::message::VioMessage *input,
    std::string input_name) {
  xstream::InputDataPtr inputdata(new xstream::InputData());
  HOBOT_CHECK(input != nullptr && input->num_ > 0 && input->is_valid_uri_);

  // \todo need a better way to identify mono or semi cameras
  for (uint32_t image_index = 0; image_index < 1; ++image_index) {
    xstream::BaseDataPtr xstream_input_data;
    if (input->num_ > image_index) {
      std::shared_ptr<xstream::PyramidImageFrame> pym_img =
          input->image_[image_index];
      LOGI << "vio message, frame_id = " << pym_img->frame_id_;
      for (uint32_t i = 0; i < PYRAMID_DOWN_SCALE_MAX; ++i) {
        LOGD << "vio message, pym_level_" << i
        << ", width=" << pym_img->img_.down_scale[i].width
        << ", height=" << pym_img->img_.down_scale[i].height
        << ", stride=" << pym_img->img_.down_scale[i].step;
      }
      auto xstream_img = std::make_shared<xstream::ImageFrame>();
      xstream_img = pym_img;
      xstream_img->type_ = "PyramidImageFrame";
      LOGI << "Input Frame ID = " << xstream_img->frame_id_
           << ", Timestamp = " << xstream_img->time_stamp_;
      xstream_input_data = xstream::BaseDataPtr(xstream_img);
    } else {
      xstream_input_data = std::make_shared<xstream::BaseData>();
      xstream_input_data->state_ = xstream::DataState::INVALID;
    }

    if (image_index == uint32_t{0}) {
      if (input->num_ == 1) {
        xstream_input_data->name_ = input_name;  // default is image
      } else {
        LOGW << "image input name may has error";
        xstream_input_data->name_ = "rgb_image";
      }
    } else {
      LOGW << "image input name may has error";
      xstream_input_data->name_ = "nir_image";
    }
    LOGI << "input name:" << xstream_input_data->name_;
    inputdata->datas_.emplace_back(xstream_input_data);
  }

  return inputdata;
}

}  // namespace xproto
