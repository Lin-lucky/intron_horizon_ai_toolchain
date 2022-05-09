/**
 *  Copyright (c) 2021 Horizon Robotics. All rights reserved.
 *  @file video_source_type.h
 *  \~English @brief this c++ header file defines the vision related data
 *
 */
#ifndef VIDEO_SOURCE_VIDEO_SOURCE_TYPE_H_
#define VIDEO_SOURCE_VIDEO_SOURCE_TYPE_H_
#include <string>
#include <vector>

namespace videosource {

enum HorizonVisionPixelFormat {
    kHorizonVisionPixelFormatNone = 0,
    kHorizonVisionPixelFormatRaw,
    kHorizonVisionPixelFormatYUY2,
    kHorizonVisionPixelFormatNV12,
    kHorizonVisionPixelFormatPYM,
    kHorizonVisionPixelFormatJPEG = 30,  // hard codec support
    kHorizonVisionPixelFormatMJPEG,      // hard codec support
    kHorizonVisionPixelFormatH264,       // hard codec support
    kHorizonVisionPixelFormatH265,       // hard codec support
    kHorizonVisionPixelFormatMax
};

struct FrameInfo {
  virtual ~FrameInfo() {}
  HorizonVisionPixelFormat pixel_format_ =
    HorizonVisionPixelFormat::kHorizonVisionPixelFormatNone;
  uint32_t channel_id_ = 0;
  uint64_t time_stamp_ = 0;  // HW time stamp
  uint64_t system_time_stamp_ = 0;  // system time
  uint64_t frame_id_ = 0;
  uint32_t buf_index_ = 0;
  uint32_t vps_chn_ = 0;
};

struct ImageLevelInfo {
  uint16_t width;
  uint16_t height;
  uint16_t stride;
  uint64_t y_paddr;
  uint64_t c_paddr;
  uint64_t y_vaddr;
  uint64_t c_vaddr;
};

// source image frame: nv12 or raw format
struct ImageFrame : public FrameInfo {
  ImageFrame() {
    pixel_format_ = HorizonVisionPixelFormat::kHorizonVisionPixelFormatNV12;
  }
  ImageLevelInfo src_info_ = { 0 };
  void *src_context_ = nullptr;
};

// pyramid image frame, compatible xj3 and j5
struct PyramidFrame : public FrameInfo {
  PyramidFrame() {
    pixel_format_ = HorizonVisionPixelFormat::kHorizonVisionPixelFormatPYM;
  }

  /**
   * 1. src_info is source image output for xj3 or j5.
   *    a) pyramid 0-layer ouput for xj3
   *    b) source image ouput without pyramid process for j5
   * 2. bl_ds is bilinear downscale for xj3 or j5.
   *    a) xj3 including all bilinear base layer and roi layer.
   *    b) j5 only for bilinear base layer.
   * 3. gs_ds is gauss downscale only for j5, including all gauss base layer
   * 4. roi_ds is roi downscale only for j5, including some downscale roi layer
   *    which input based on either of them as follow:
   *    a) bilinear pyramid certain some base layer
   *    b) gauss pyramid certain some base layer
   *    c) source image.
   * 5. roi_us is roi upscale for xj3 or j5.
   *    a) including some upscale roi layer
   *       which base on as well as roi_ds for j5.
   *    b) it is independent module for xj3.
   */
  ImageLevelInfo src_info_ = { 0 };
  // max bilinear layer is 24 for xj3 or 5 for j5
  std::vector<ImageLevelInfo> bl_ds_;
  // max gauss layer is 5 for j5
  std::vector<ImageLevelInfo> gs_ds_;
  // max roi downscale is 6 for j5
  std::vector<ImageLevelInfo> roi_ds_;
  // max roi upscale layer is 6 for xj3 or 1 for j5
  std::vector<ImageLevelInfo> roi_us_;
  // pyramid context for xj3 or j5
  void *pym_context_ = nullptr;
  // source context for j5
  void *src_context_ = nullptr;
};

}  // namespace videosource

#endif  // VIDEO_SOURCE_VIDEO_SOURCE_TYPE_H_
