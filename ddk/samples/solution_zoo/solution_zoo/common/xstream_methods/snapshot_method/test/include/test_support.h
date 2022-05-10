/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     dump helper header
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2019.05.23
 */

#ifndef COMMON_XSTREAM_METHODS_SNAPSHOT_METHOD_\
TEST_INCLUDE_TEST_SUPPORT_H_
#define COMMON_XSTREAM_METHODS_SNAPSHOT_METHOD_\
TEST_INCLUDE_TEST_SUPPORT_H_

#include <memory>
#include <string>

#include "opencv2/opencv.hpp"
#include "snapshot_method/snapshot_data_type.h"
#include "xstream/vision_type.h"
#include "xstream/xstream_world.h"

int WriteLog(const xstream::XStreamSnapshotInfoPtr &snapshot_info);

// static int SaveImg(const ImageFramePtr &img_ptr, const std::string &path);

int DumpSnap(const xstream::XStreamSnapshotInfoPtr &snapshot_info,
             std::string dir = ".");

int ConstructInput(const std::string &smart_frame,
                   const std::string &video_path, xstream::InputDataPtr &input,
                   const std::string &img_format, bool filter);

int ConstructInputInvalidUserdata(const std::string &smart_frame,
                                  const std::string &video_path,
                                  xstream::InputDataPtr &input,
                                  const std::string &img_format, bool filter);

#endif  //  COMMON_XSTREAM_METHODS_SNAPSHOT_METHOD_TEST_INCLUDE_TEST_SUPPORT_H_
