/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     test class ImageConvertor
 * @author    hangjun.yang
 * @email     hangjun.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2020.09.15
 */

#include <chrono>
#include <fstream>
#include <iostream>
#include "gtest/gtest.h"
#include "hobotlog/hobotlog.hpp"
#include "image_utils/image_utils.h"
#include "include/common.h"

namespace xstream {

class ImageToolsTest : public ::testing::Test {
 protected:
  virtual void SetUp() {}

  virtual void TearDown() {}
};

TEST_F(ImageToolsTest, NV12ToBGR) {
  using std::chrono::duration;
  using std::chrono::duration_cast;
  using std::chrono::microseconds;
  using std::chrono::steady_clock;
  std::ifstream istrm;
  // 1080p
  istrm.open("test.jpg", std::ios::binary | std::ios::in);
  if (!istrm.good()) {
    return;
  }
  istrm.seekg(0, std::ios::end);
  int jpg_data_size = istrm.tellg();
  istrm.seekg(0, std::ios::beg);
  uint8_t *jpg_data = nullptr;
  HobotXStreamAllocImage(jpg_data_size, &jpg_data);

  istrm.read(reinterpret_cast<char *>(jpg_data), jpg_data_size);
  istrm.close();

  // 1)调用解码接口，解码成1080p的NV12
  uint8_t *nv12_output_data = nullptr;
  int nv12_output_size = 0;
  int nv12_width = 0;
  int nv12_height = 0;
  int output_nv12_default_first_stride = 0;
  int output_nv12_default_second_stride = 0;

  int ret = HobotXStreamDecodeImage(
      jpg_data, jpg_data_size, IMAGE_TOOLS_RAW_YUV_NV12, &nv12_output_data,
      &nv12_output_size, &nv12_width, &nv12_height,
      &output_nv12_default_first_stride, &output_nv12_default_second_stride);
  if (ret != 0) {
    LOGD << "decode test.jpg to nv12 failed ret:" << ret;
    HobotXStreamFreeImage(jpg_data);
    jpg_data = nullptr;
    return;
  }
  // 2) 1080p的NV12缩放到720P的NV12
  int nv12_dst_width = 1280;
  int nv12_dst_height = 720;
  uint8_t *nv12_720p_output_data = nullptr;
  int nv12_720p_output_size = 0;
  int nv12_720p_first_stride = 0;
  int nv12_720p_second_stride = 0;
  struct HobotXStreamImageToolsResizeInfo resize_info;
  ret = HobotXStreamResizeImage(
      nv12_output_data, nv12_output_size, nv12_width, nv12_height,
      output_nv12_default_first_stride, output_nv12_default_second_stride,
      IMAGE_TOOLS_RAW_YUV_NV12, 1, nv12_dst_width, nv12_dst_height,
      &nv12_720p_output_data, &nv12_720p_output_size, &nv12_720p_first_stride,
      &nv12_720p_second_stride, &resize_info);
  if (ret != 0) {
    LOGD << "resize nv12 image to 720p failed ret:" << ret;
    HobotXStreamFreeImage(jpg_data);
    jpg_data = nullptr;
    return;
  }
  // std::string filename = "test_720p.nv12";
  // SaveFile(filename, nv12_720p_output_data, nv12_720p_output_size);
  // 3) 720P的NV12图像转换成BGR，并统计转换的时间
  uint8_t *bgr_720p_output_data = nullptr;
  int bgr_720p_output_size = 0;
  int bgr_720p_first_stride = 0;
  int bgr_720p_second_stride = 0;

  uint8_t *y = nullptr;
  int y_size = nv12_720p_first_stride * nv12_dst_height;
  HobotXStreamAllocImage(y_size, &y);

  uint8_t *uv = nullptr;
  int uv_size = y_size >> 1;
  HobotXStreamAllocImage(uv_size, &uv);
  memmove(y, nv12_720p_output_data, y_size);
  memmove(uv, nv12_output_data + y_size, uv_size);

  const uint8_t *input_yuv_data[3] = {y, uv, nullptr};
  const int input_yuv_size[3] = {y_size, uv_size, 0};
  auto start = steady_clock::now();
  auto begin_timestamp = std::chrono::system_clock::now();
  ret = HobotXStreamConvertYuvImage(
      input_yuv_data, input_yuv_size, nv12_dst_width, nv12_dst_height,
      nv12_720p_first_stride, nv12_720p_second_stride, IMAGE_TOOLS_RAW_YUV_NV12,
      IMAGE_TOOLS_RAW_BGR, &bgr_720p_output_data, &bgr_720p_output_size,
      &bgr_720p_first_stride, &bgr_720p_second_stride);

  if (ret == 0) {
    auto end_timestamp = std::chrono::system_clock::now();
    auto duration =
        duration_cast<microseconds>(end_timestamp - begin_timestamp);
    std::cout << "std::cout convert 720p nv12 image to bgr cast:"
              << (static_cast<double>(duration.count()) *
                  microseconds::period::num / microseconds::period::den)
              << "s";
    LOGD << "convert 720p nv12 image to bgr cast:"
         << (static_cast<double>(duration.count()) * microseconds::period::num /
             microseconds::period::den)
         << "s";

    auto end = steady_clock::now();
    auto tt = duration_cast<microseconds>(end - start);
    std::cout << "cost:" << tt.count() << " us over!";
  }
  // SaveFile("test_720p.bgr", bgr_720p_output_data, bgr_720p_output_size);
  HobotXStreamFreeImage(jpg_data);
  HobotXStreamFreeImage(y);
  HobotXStreamFreeImage(uv);
  jpg_data = nullptr;
}
}  // namespace xstream
