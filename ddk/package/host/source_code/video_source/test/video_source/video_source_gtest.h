/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2019 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef INCLUDE_VIDEO_SOURCE_GTEST_H_
#define INCLUDE_VIDEO_SOURCE_GTEST_H_
#include <string>
#include "video_source/video_source.h"
#include "video_source/video_source_type.h"

using videosource::VideoSource;
using videosource::VideoSourceLogLevel;
using videosource::ImageFrame;
using videosource::PyramidFrame;
using videosource::HorizonVisionPixelFormat;

enum TestType {
  kTEST_VIDEO_SOURCE_TYPE,
  kTEST_IMAGE_SOURCE_TYPE,
  kTEST_TYPE_MAX,
};

struct CmdOptions {
  int mGlobalWidth;
  int mGlobalHeight;
  TestType mGlobalType;
  std::string mGlobalConfigFile;
  enum VideoSourceLogLevel mGlobalLogLevel;
  HorizonVisionPixelFormat mGlobalPixFmt;
  std::string mGlobalInFileName;
  std::string mGlobalOutFilePath;
};

#endif  // INCLUDE_VIDEO_SOURCE_GTEST_H_
