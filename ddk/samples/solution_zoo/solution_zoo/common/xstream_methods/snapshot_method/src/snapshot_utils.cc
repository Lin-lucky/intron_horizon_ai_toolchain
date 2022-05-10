/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     image_utils implementation
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.10
 * @date      2019.04.22
 */

#include "snapshot_method/snapshot_utils.h"

#include <chrono>
#include <memory>

#include "snapshot_method/error_code.h"
#include "hobotlog/hobotlog.hpp"
#include "xstream/xstream_data.h"


namespace xstream {

using std::chrono::duration;
using std::chrono::high_resolution_clock;

using RawDataImageFramePtr = std::shared_ptr<RawDataImageFrame>;

int ImageUtils::Data2CVImage(const uint32_t &height, const uint32_t &width,
                             const HorizonVisionPixelFormat &format,
                             uint8_t *data, cv::Mat &cv_img) {
  if (!data) {
    return XSTREAM_SNAPSHOT_ERR_PARAM;
  }
  switch (format) {
    case kHorizonVisionPixelFormatNone:
    case kHorizonVisionPixelFormatImageContainer: {
      return XSTREAM_SNAPSHOT_ERR_PARAM;
    }
    case kHorizonVisionPixelFormatRawRGB:
    case kHorizonVisionPixelFormatRawBGR: {
      cv_img = cv::Mat(height, width, CV_8UC3);
      memcpy(cv_img.data, data, height * width * 3);
      break;
    }

    case kHorizonVisionPixelFormatRawNV12: {
      cv_img = cv::Mat(height * 3 / 2, width, CV_8UC1);
      memcpy(cv_img.data, data, height * width * 3 / 2);
      break;
    }
    default:
      return XSTREAM_SNAPSHOT_ERR_PARAM;
  }
  return XSTREAM_SNAPSHOT_OK;
}

#ifndef ALIGN
#define ALIGN(x, a) (((x) + (a)-1) & ~(a - 1))
#endif
BBox ImageUtils::AdjustSnapRect(const uint32_t &frame_width,
                                const uint32_t &frame_height,
                                const BBox &in_bbox, const float &scale) {
  auto s32LeftTopX = static_cast<int>(in_bbox.x1_);
  auto s32LeftTopY = static_cast<int>(in_bbox.y1_);
  auto s32Width = static_cast<int>(in_bbox.Width());
  auto s32Height = static_cast<int>(in_bbox.Height());

  auto s32CenterX = s32LeftTopX + (s32Width >> 1);
  auto s32CenterY = s32LeftTopY + (s32Height >> 1);
  s32CenterX = ALIGN(s32CenterX, 2);
  s32CenterY = ALIGN(s32CenterY, 2);

  s32Width = static_cast<int>(s32Width * scale);
  s32Height = static_cast<int>(s32Height * scale);

  s32Width = ALIGN(s32Width, 4);
  s32Height = ALIGN(s32Height, 4);

  if (s32Width >= static_cast<int>(frame_width) &&
      s32Height >= static_cast<int>(frame_height)) {
    s32Width = s32Height = std::min(frame_width, frame_height) - 8;
  }
  if (s32Width > static_cast<int>(frame_width)) {
    s32Width = frame_width - 8;
  }
  if (s32Height > static_cast<int>(frame_height)) {
    s32Height = frame_height - 8;
  }
  if (s32Width < s32Height) {
    s32Width = s32Height;
  }
  if (s32Height < s32Width) {
    s32Height = s32Width;
  }

  s32LeftTopX = s32CenterX - (s32Width >> 1);
  s32LeftTopY = s32CenterY - (s32Height >> 1);
  auto s32RightBotX = s32CenterX + (s32Width >> 1);
  auto s32RightBotY = s32CenterY + (s32Height >> 1);

  if (s32LeftTopX < 0 || s32LeftTopY < 0 ||
      s32RightBotX >= static_cast<int>(frame_width) ||
      s32RightBotY >= static_cast<int>(frame_height)) {
    if (s32LeftTopX < 0) {
      s32LeftTopX = 0;
    }
    if (s32LeftTopY < 0) {
      s32LeftTopY = 0;
    }
    if (s32RightBotX >= static_cast<int>(frame_width)) {
      s32LeftTopX = s32LeftTopX - (s32RightBotX - frame_width);
    }
    if (s32RightBotY >= static_cast<int>(frame_height)) {
      s32LeftTopY = s32LeftTopY - (s32RightBotY - frame_height);
    }
  }

  BBox out_bbox;
  out_bbox.x1_ = s32LeftTopX;
  out_bbox.y1_ = s32LeftTopY;
  out_bbox.x2_ = s32LeftTopX + s32Width - 1;
  out_bbox.y2_ = s32LeftTopY + s32Height - 1;
  return out_bbox;
}

static int HobotXStreamCropImageFrameWithPaddingBlack(\
                       void *input,
                       const int top_left_x,
                       const int top_left_y,
                       const int bottom_right_x,
                       const int bottom_right_y,
                       enum HobotXStreamImageToolsPixelFormat *output_format,
                       uint8_t **output,
                       int *output_size,
                       int *output_width,
                       int *output_height,
                       int *output_first_stride,
                       int *output_second_stride) {
  if (nullptr == input
      || bottom_right_x < top_left_x
      || bottom_right_y < top_left_y
      || nullptr == output_format
      || nullptr == output
      || nullptr == output_size
      || nullptr == output_width
      || nullptr == output_height) {
    return -1;
  }
  int pyramid_layer = 0;
  xstream::ImageFrame *image_frame = static_cast<xstream::ImageFrame *>(input);
  HobotXStreamImageToolsPixelFormat input_format = IMAGE_TOOLS_RAW_NONE;
  xstream::FotmatDataArrayType data_array_type =
              xstream::FotmatDataArrayType::kContinueType;
  switch (image_frame->pixel_format_) {
    case kHorizonVisionPixelFormatRawRGB: {
      input_format = IMAGE_TOOLS_RAW_RGB;
      break;
    }
    case kHorizonVisionPixelFormatRawBGR: {
      input_format = IMAGE_TOOLS_RAW_BGR;
      break;
    }
    case kHorizonVisionPixelFormatRawNV12: {
      input_format = IMAGE_TOOLS_RAW_YUV_NV12;
      break;
    }
    default:
      return -1;
  }
  *output_format = input_format;
  if (xstream::FotmatDataArrayType::kContinueType == data_array_type) {
    return HobotXStreamCropImageWithPaddingBlack(
        reinterpret_cast<uint8_t *>(image_frame->Data(pyramid_layer)),
        static_cast<int>(image_frame->DataSize(pyramid_layer)),
        static_cast<int>(image_frame->Width(pyramid_layer)),
        static_cast<int>(image_frame->Height(pyramid_layer)),
        static_cast<int>(image_frame->Stride(pyramid_layer)),
        static_cast<int>(image_frame->StrideUV(pyramid_layer)), input_format,
        top_left_x, top_left_y, bottom_right_x, bottom_right_y, output,
        output_size, output_width, output_height, output_first_stride,
        output_second_stride);
  } else {
    const uint8_t *input_yuv_data[3] = {
        reinterpret_cast<uint8_t *>(image_frame->Data(pyramid_layer)),
        reinterpret_cast<uint8_t *>(image_frame->DataUV(pyramid_layer)), 0};
    const int input_yuv_size[3] = {
        static_cast<int>(image_frame->DataSize(pyramid_layer)),
        static_cast<int>(image_frame->DataUVSize(pyramid_layer)), 0};
    return HobotXStreamCropYuvImageWithPaddingBlack(
        input_yuv_data, input_yuv_size,
        static_cast<int>(image_frame->Width(pyramid_layer)),
        static_cast<int>(image_frame->Height(pyramid_layer)),
        static_cast<int>(image_frame->Stride(pyramid_layer)),
        static_cast<int>(image_frame->StrideUV(pyramid_layer)), input_format,
        top_left_x, top_left_y, bottom_right_x, bottom_right_y, output,
        output_size, output_width, output_height, output_first_stride,
        output_second_stride);
  }
  return -1;
}

ImageFramePtr ImageUtils::DoFaceCrop(const ImageFramePtr &frame,
                                     const BBox &crop_rect,
                                     const uint32_t &output_width,
                                     const uint32_t &output_height,
                                     const bool &need_resize) {
  if (!frame->Data(0)) return nullptr;

  auto u32Width = static_cast<unsigned>(crop_rect.Width() + 1);
  auto u32Height = static_cast<unsigned>(crop_rect.Height() + 1);

  assert(u32Width == u32Height);
  bool bNeedScale = (u32Width != output_width) || (u32Height != output_height);
  int width = 0;
  int height = 0;
  int first_stride = 0;
  int second_stride = 0;
  int crop_data_size = 0;

  unsigned char *pCropBuf = nullptr;
  auto *output_format = new HobotXStreamImageToolsPixelFormat();

  int s32Ret = HobotXStreamCropImageFrameWithPaddingBlack(
      frame.get(), static_cast<const int>(crop_rect.x1_),
      static_cast<const int>(crop_rect.y1_),
      static_cast<const int>(crop_rect.x2_),
      static_cast<const int>(crop_rect.y2_), output_format, &pCropBuf,
      &crop_data_size, &width, &height, &first_stride, &second_stride);

  if (s32Ret < 0) {
    LOGE << "crop failed!\n";
    if (bNeedScale) {
      std::free(pCropBuf);
    }
    return nullptr;
  }
  int out_data_size, out_stride, out_stride_uv;
  xstream::RawDataImageFramePtr snap_frame(new xstream::RawDataImageFrame(),
   [&](xstream::RawDataImageFrame *p){
    if (p) {
      LOGD << "delete rawdata image frame";
      std::free(p->data_);
    }
  });

  if (bNeedScale && need_resize) {
    HobotXStreamImageToolsResizeInfo resize_info{};
    uint8_t *scale_data;
    s32Ret = HobotXStreamResizeImage(
        pCropBuf, crop_data_size, width, height, first_stride, second_stride,
        *output_format, 1, output_width, output_height, &scale_data,
        &out_data_size, &out_stride, &out_stride_uv, &resize_info);
    snap_frame->data_ = scale_data;
    snap_frame->width_ = output_width;
    snap_frame->height_ = output_height;
    if (s32Ret < 0) {
      LOGE << "hobot scale failed!\n";
      std::free(pCropBuf);
      return nullptr;
    }
    std::free(pCropBuf);
  } else {
    snap_frame->data_ = pCropBuf;
    snap_frame->width_ = width;
    snap_frame->height_ = height;
  }

  delete output_format;

  if (snap_frame) {
    snap_frame->pixel_format_ = frame->pixel_format_;
    snap_frame->frame_id_ = frame->frame_id_;
    snap_frame->time_stamp_ = frame->time_stamp_;
  }

  return snap_frame;
}

}  // namespace xstream
