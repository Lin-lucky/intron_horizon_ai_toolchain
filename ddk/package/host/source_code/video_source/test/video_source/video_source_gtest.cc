/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include <cstdint>
#include <fstream>
#include <gtest/gtest.h>
#include <memory>
#include "hobotlog/hobotlog.hpp"
#include "./video_source_gtest.h"

extern CmdOptions cmd_options;

namespace videosource {

class VideoSourceTest : public testing::Test {
 protected:
  virtual void SetUp() {
    test_type_ = TestType::kTEST_VIDEO_SOURCE_TYPE;
    config_file_ = cmd_options.mGlobalConfigFile;
    log_level_ = cmd_options.mGlobalLogLevel;
  }
  virtual void TearDown() {}
  virtual int SyncGetFrame(
      std::shared_ptr<VideoSource> &video_source);
  virtual int AsyncGetFrame(
      std::shared_ptr<VideoSource> &video_source);

 public:
  TestType test_type_;
  std::string config_file_;
  enum VideoSourceLogLevel log_level_;
};

class ImageSourceTest : public testing::Test {
 protected:
  virtual void SetUp() {
    test_type_ = TestType::kTEST_IMAGE_SOURCE_TYPE;
    config_file_ = cmd_options.mGlobalConfigFile;
    log_level_ = cmd_options.mGlobalLogLevel;
    image_name_ = cmd_options.mGlobalInFileName;
    image_width_ = cmd_options.mGlobalWidth;
    image_height_ = cmd_options.mGlobalHeight;
    image_format_ = cmd_options.mGlobalPixFmt;
  }
  virtual void TearDown() {}
  int ReadDdrImage(std::string &image_path,
      uint32_t &image_width, uint32_t &image_height,
      HorizonVisionPixelFormat &image_format);
  int GetOutput(
      const std::shared_ptr<VideoSource> &video_source);

 public:
  TestType test_type_;
  std::string config_file_;
  enum VideoSourceLogLevel log_level_;
  std::string image_name_;
  int image_width_;
  int image_height_;
  HorizonVisionPixelFormat image_format_;
};

int VideoSourceTest::AsyncGetFrame(
    std::shared_ptr<VideoSource> &video_source) {
  int ret = -1;
  bool vin_out_en = video_source->GetVinOutEnable();
  bool vps_en = video_source->GetVpsEnable();
  // 1. get vin output
  if (vin_out_en == true) {
    std::shared_ptr<ImageFrame> vin_image = nullptr;
    ret = video_source->GetVinImageFrame(vin_image);
    if (ret) {
      LOGE << "get vin image frame failed, ret: " << ret;
      return ret;
    }
    ret = video_source->FreeVinImageFrame(vin_image);
    if (ret) {
      LOGE << "free vin image frame failed, ret: " << ret;
      return ret;
    }
  }

  // 2. get vps output
  if (vps_en == true) {
    // try get ipu nv12 image frame
    std::vector<std::shared_ptr<ImageFrame>> ipu_image_list;
    ret = video_source->GetVpsImageFrame(ipu_image_list);
    if (ret) {
      LOGE << "get vps image frame failed, ret: " << ret;
      return ret;
    }
    ret = video_source->FreeVpsImageFrame(ipu_image_list);
    if (ret) {
      LOGE << "free vps image frame failed, ret: " << ret;
      return ret;
    }
    // try get pyramid image frame
    std::shared_ptr<PyramidFrame> pym_image = nullptr;
    ret = video_source->GetPyramidFrame(pym_image);
    if (ret) {
      LOGE << "get pyramid frame failed, ret: " << ret;
      return ret;
    }
    ret = video_source->FreePyramidFrame(pym_image);
    if (ret) {
      LOGE << "free pyramid frame failed, ret: " << ret;
      return ret;
    }
  }
  return 0;
}

int VideoSourceTest::SyncGetFrame(
    std::shared_ptr<VideoSource> &video_source) {
  int ret = -1;
  std::shared_ptr<ImageFrame> vin_image = nullptr;
  std::vector<std::shared_ptr<ImageFrame>> ipu_image_list;
  std::shared_ptr<PyramidFrame> pym_image = nullptr;

  bool vin_out_en = video_source->GetVinOutEnable();
  bool vps_en = video_source->GetVpsEnable();
  // 1. get vin output
  if (vin_out_en == true) {
    ret = video_source->GetVinImageFrame(vin_image);
    if (ret) {
      LOGE << "get vin image frame failed, ret: " << ret;
      return ret;
    }
  }
  // 2. get vps output
  if (vin_out_en == true && vps_en == true) {
    // 2.1 send vin frame to vps
    ret = video_source->SendFrameToVps(vin_image);
    if (ret) {
      LOGE << "send frame to vps failed, ret: " << ret;
      return ret;
    }
    // 2.3 try get ipu nv12 image frame
    ret = video_source->GetVpsImageFrame(ipu_image_list);
    if (ret) {
      LOGE << "get vps image frame failed, ret: " << ret;
      return ret;
    }
    // 2.3 try get pyramid image frame
    ret = video_source->GetPyramidFrame(pym_image);
    if (ret) {
      LOGE << "get pyramid frame failed, ret: " << ret;
      return ret;
    }
  }
  // 3. free vin frame
  if (vin_out_en == true) {
    ret = video_source->FreeVinImageFrame(vin_image);
    if (ret) {
      LOGE << "free vin image frame failed, ret: " << ret;
      return ret;
    }
  }
  // 4. free vps frame
  if (vin_out_en == true && vps_en == true) {
    ret = video_source->FreeVpsImageFrame(ipu_image_list);
    if (ret) {
      LOGE << "free vps image frame failed, ret: " << ret;
      return ret;
    }
    ret = video_source->FreePyramidFrame(pym_image);
    if (ret) {
      LOGE << "free pyramid frame failed, ret: " << ret;
      return ret;
    }
  }
  return 0;
}

TEST_F(VideoSourceTest, CreateVideoSoruce) {
  int channel_id = 0;
  auto video_source = std::make_shared<VideoSource>(channel_id, config_file_);
  EXPECT_TRUE(video_source);
}

TEST_F(VideoSourceTest, GetSourceType) {
  int ret = 0;
  int channel_id = 0;
  auto video_source = std::make_shared<VideoSource>(channel_id, config_file_);
  EXPECT_TRUE(video_source);
  video_source->SetLoggingLevel(log_level_);

  auto source_type = video_source->GetSourceType();
  if (source_type == VideoSourceType::kSOURCE_TYPE_MIPI_CAM
      || source_type == VideoSourceType::kSOURCE_TYPE_USB_CAM
      || source_type == VideoSourceType::kSOURCE_TYPE_FEEDBACK
      || source_type == VideoSourceType::kSOURCE_TYPE_RTSP_CLIENT) {
    LOGI << "get source type: " << source_type;
  } else {
    LOGE << "source type is not support";
    ret = -1;
  }
  EXPECT_EQ(ret, 0);
}

TEST_F(VideoSourceTest, Init) {
  int ret = -1;
  int channel_id = 0;
  auto video_source = std::make_shared<VideoSource>(channel_id, config_file_);
  EXPECT_TRUE(video_source);
  video_source->SetLoggingLevel(log_level_);

  ret = video_source->Init();
  EXPECT_EQ(ret, 0) << "video source init failed, ret: " << ret;
  // if video source has init, next init will pass
  ret = video_source->Init();
  EXPECT_EQ(ret, 0) << "video source init failed, ret: " << ret;
  ret = video_source->DeInit();
  EXPECT_EQ(ret, 0) << "video source deinit failed, ret: " << ret;
  ret = video_source->Init();
  EXPECT_EQ(ret, 0) << "video source init failed, ret: " << ret;
  ret = video_source->DeInit();
  EXPECT_EQ(ret, 0) << "video source deinit failed, ret: " << ret;
}

TEST_F(VideoSourceTest, DeInit) {
  int ret = -1;
  int channel_id = 0;
  auto video_source = std::make_shared<VideoSource>(channel_id, config_file_);
  EXPECT_TRUE(video_source);
  video_source->SetLoggingLevel(log_level_);

  // video source has not init, deinit will failed
  ret = video_source->DeInit();
  EXPECT_NE(ret, 0) << "video source deinit success, ret: " << ret;

  // video source init first and then deinit
  ret = video_source->Init();
  EXPECT_EQ(ret, 0) << "video source init failed, ret: " << ret;
  ret = video_source->DeInit();
  EXPECT_EQ(ret, 0) << "video source deinit failed, ret: " << ret;
  video_source = nullptr;

  // create other video source
  channel_id = 0;
  video_source = std::make_shared<VideoSource>(channel_id, config_file_);
  EXPECT_TRUE(video_source);
  video_source->SetLoggingLevel(log_level_);
  ret = video_source->Init();
  EXPECT_EQ(ret, 0) << "video source init failed, ret: " << ret;
  ret = video_source->DeInit();
  EXPECT_EQ(ret, 0) << "video source deinit failed, ret: " << ret;
}

TEST_F(VideoSourceTest, Start) {
  int ret = -1;
  int channel_id = 0;
  auto video_source = std::make_shared<VideoSource>(channel_id, config_file_);
  EXPECT_TRUE(video_source);
  video_source->SetLoggingLevel(log_level_);

  // if video source has not init, start will pass.
  ret = video_source->Start();
  EXPECT_NE(ret, 0) << "video source start success, ret: " << ret;

  ret = video_source->Init();
  EXPECT_EQ(ret, 0) << "video source init failed, ret: " << ret;
  ret = video_source->Start();
  EXPECT_EQ(ret, 0) << "video source start failed, ret: " << ret;
  ret = video_source->Stop();
  EXPECT_EQ(ret, 0) << "video source stop failed, ret: " << ret;
  // if video source init, deinit has must call last
  ret = video_source->DeInit();
  EXPECT_EQ(ret, 0) << "video source deinit failed, ret: " << ret;
}

TEST_F(VideoSourceTest, Stop) {
  int ret = -1;
  int channel_id = 0;
  auto video_source = std::make_shared<VideoSource>(channel_id, config_file_);
  EXPECT_TRUE(video_source);
  video_source->SetLoggingLevel(log_level_);

  // if video source has not init, stop will pass.
  ret = video_source->Stop();
  EXPECT_NE(ret, 0) << "video source stop success, ret: " << ret;

  ret = video_source->Init();
  EXPECT_EQ(ret, 0) << "video source init failed, ret: " << ret;
  ret = video_source->Start();
  EXPECT_EQ(ret, 0) << "video source start failed, ret: " << ret;
  ret = video_source->Stop();
  EXPECT_EQ(ret, 0) << "video source stop failed, ret: " << ret;
  ret = video_source->DeInit();
  EXPECT_EQ(ret, 0) << "video source deinit failed, ret: " << ret;
}

TEST_F(VideoSourceTest, GetVinImageFrame) {
  int ret = -1;
  int channel_id = 0;
  int run_time = 5000;  // 5s
  auto video_source = std::make_shared<VideoSource>(channel_id, config_file_);
  EXPECT_TRUE(video_source);
  video_source->SetLoggingLevel(log_level_);


  ret = video_source->Init();
  EXPECT_EQ(ret, 0) << "video source init failed, ret: " << ret;
  ret = video_source->Start();
  EXPECT_EQ(ret, 0) << "video source start failed, ret: " << ret;
  bool vin_out_en = video_source->GetVinOutEnable();

  auto start_time = std::chrono::system_clock::now();
  while (1) {
    auto curr_time = std::chrono::system_clock::now();
    auto cost_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          curr_time - start_time);
    if (cost_time.count() >= run_time) {
      LOGI << "run time end, cost_time: " << cost_time.count();
      break;
    }
    if (vin_out_en == true) {
      std::shared_ptr<ImageFrame> vin_image = nullptr;
      ret = video_source->GetVinImageFrame(vin_image);
      EXPECT_EQ(ret, 0) << "get vin image frame failed, ret: " << ret;
      ret = video_source->FreeVinImageFrame(vin_image);
      EXPECT_EQ(ret, 0) << "free vin image frame failed, ret: " << ret;
    }
  }
  ret = video_source->Stop();
  EXPECT_EQ(ret, 0) << "video source stop failed, ret: " << ret;
  ret = video_source->DeInit();
  EXPECT_EQ(ret, 0) << "video source deinit failed, ret: " << ret;
}

TEST_F(VideoSourceTest, GetVpsImageFrame) {
  int ret = -1;
  int channel_id = 0;
  int run_time = 10000;  // 10s
  auto video_source = std::make_shared<VideoSource>(channel_id, config_file_);
  EXPECT_TRUE(video_source);
  video_source->SetLoggingLevel(log_level_);

  ret = video_source->Init();
  EXPECT_EQ(ret, 0) << "video source init failed, ret: " << ret;
  ret = video_source->Start();
  EXPECT_EQ(ret, 0) << "video source start failed, ret: " << ret;
  bool is_sync_mode = video_source->GetSyncMode();

  auto start_time = std::chrono::system_clock::now();
  while (1) {
    auto curr_time = std::chrono::system_clock::now();
    auto cost_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          curr_time - start_time);
    if (cost_time.count() >= run_time) {
      LOGI << "run time end, cost_time: " << cost_time.count();
      break;
    }
    if (is_sync_mode == true) {
      ret = SyncGetFrame(video_source);
      EXPECT_EQ(ret, 0) << "sync get image frame failed, ret: " << ret;
    } else {
      ret = AsyncGetFrame(video_source);
      EXPECT_EQ(ret, 0) << "async get image frame failed, ret: " << ret;
    }
  }
  ret = video_source->Stop();
  EXPECT_EQ(ret, 0) << "video source stop failed, ret: " << ret;
  ret = video_source->DeInit();
  EXPECT_EQ(ret, 0) << "video source deinit failed, ret: " << ret;
}

TEST_F(ImageSourceTest, CreateVideoSource) {
  LOGI << "enter ImageSourceTest create video source";
  auto video_source = std::make_shared<VideoSource>(config_file_);
  EXPECT_TRUE(video_source);
}

int ImageSourceTest::GetOutput(
    const std::shared_ptr<VideoSource> &video_source) {
  int ret = -1;
  if (video_source == nullptr) {
    LOGE << "video_source is nullptr";
    return -1;
  }
  bool vin_out_en = video_source->GetVinOutEnable();
  bool vps_en = video_source->GetVpsEnable();
  // 1. get vin output
  if (vin_out_en == true) {
    std::shared_ptr<ImageFrame> vin_image = nullptr;
    ret = video_source->GetVinImageFrame(vin_image);
    if (ret) {
      std::cout << "get vin image frame failed, ret: " << ret << std::endl;
      return ret;
    }
    ret = video_source->FreeVinImageFrame(vin_image);
    if (ret) {
      std::cout << "free vin image frame failed, ret: " << ret << std::endl;
      return ret;
    }
  }

  // 2. get vps output
  if (vps_en == true) {
    std::shared_ptr<PyramidFrame> pym_image = nullptr;
    ret = video_source->GetPyramidFrame(pym_image);
    if (ret) {
      std::cout << "get pyramid frame failed, ret: " << ret << std::endl;
      return ret;
    }
    ret = video_source->FreePyramidFrame(pym_image);
    if (ret) {
      std::cout << "free pyramid frame failed, ret: " << ret << std::endl;
      return ret;
    }
  }
  return 0;
}

int ImageSourceTest::ReadDdrImage(std::string &image_path,
    uint32_t &image_width, uint32_t &image_height,
    HorizonVisionPixelFormat &image_format) {
  int ret = -1;
  int len;
  char *data = nullptr;

  // read image to ddr
  ret = access(image_path.c_str(), F_OK);
  EXPECT_EQ(ret, 0) << "File not exist: " << image_path;
  std::ifstream ifs(image_path, std::ios::in | std::ios::binary);
  EXPECT_TRUE(ifs) << "Failed load image: " << image_path;
  ifs.seekg(0, std::ios::end);
  len = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  data = new char[len];
  ifs.read(data, len);

  LOGI << "enter ImageSourceTest create video source";
  auto video_source = std::make_shared<VideoSource>(config_file_);
  EXPECT_TRUE(video_source);
  video_source->SetLoggingLevel(log_level_);
  ret = video_source->ReadImage(data, len, image_width, image_height,
      image_format);
  EXPECT_EQ(ret, 0) << "read image: " << image_path << " failed, ret: " << ret;
  ret = GetOutput(video_source);
  EXPECT_EQ(ret, 0) << "get output failed, ret: " << ret;

  delete[] data;
  ifs.close();
  return 0;
}

TEST_F(ImageSourceTest, ReadFileJpegImage) {
  int ret = -1;
  std::string image_path = "configs/video_source/x3dev/feedback/data/2160p.jpg";
  uint32_t image_width = 3840;
  uint32_t image_height = 2160;
  HorizonVisionPixelFormat image_format =
    HorizonVisionPixelFormat::kHorizonVisionPixelFormatJPEG;

  LOGI << "enter ImageSourceTest create video source";
  auto video_source = std::make_shared<VideoSource>(config_file_);
  EXPECT_TRUE(video_source);
  video_source->SetLoggingLevel(log_level_);
  ret = video_source->ReadImage(image_path, image_width, image_height,
      image_format);
  EXPECT_EQ(ret, 0) << "read image: " << image_name_ << " failed, ret: " << ret;
  ret = GetOutput(video_source);
  EXPECT_EQ(ret, 0) << "get ouput failed, ret: " << ret;
}

TEST_F(ImageSourceTest, ReadFileNv12Image) {
  int ret = -1;
  std::string image_path =
    "configs/video_source/x3dev/feedback/data/nv12_1080p.yuv";
  uint32_t image_width = 1920;
  uint32_t image_height = 1080;
  HorizonVisionPixelFormat image_format =
    HorizonVisionPixelFormat::kHorizonVisionPixelFormatNV12;

  LOGI << "enter ImageSourceTest create video source";
  auto video_source = std::make_shared<VideoSource>(config_file_);
  EXPECT_TRUE(video_source);
  video_source->SetLoggingLevel(log_level_);
  ret = video_source->ReadImage(image_path, image_width, image_height,
      image_format);
  EXPECT_EQ(ret, 0) << "read image: " << image_name_ << " failed, ret: " << ret;
  ret = GetOutput(video_source);
  EXPECT_EQ(ret, 0) << "get ouput failed, ret: " << ret;
}

TEST_F(ImageSourceTest, ReadDdrJpegImage) {
  int ret = -1;
  std::string image_path =
    "configs/video_source/x3dev/feedback/data/2160p.jpg";
  uint32_t image_width = 3840;
  uint32_t image_height = 2160;
  HorizonVisionPixelFormat image_format =
    HorizonVisionPixelFormat::kHorizonVisionPixelFormatJPEG;

  ret = ReadDdrImage(image_path, image_width, image_height, image_format);
  EXPECT_EQ(ret, 0) << "video source read ddr image failed, ret: " << ret;
}

TEST_F(ImageSourceTest, ReadDdrNv12Image) {
  int ret = -1;
  std::string image_path =
    "configs/video_source/x3dev/feedback/data/nv12_1080p.yuv";
  uint32_t image_width = 1920;
  uint32_t image_height = 1080;
  HorizonVisionPixelFormat image_format =
    HorizonVisionPixelFormat::kHorizonVisionPixelFormatNV12;

  ret = ReadDdrImage(image_path, image_width, image_height, image_format);
  EXPECT_EQ(ret, 0) << "video source read ddr image failed, ret: " << ret;
}

}  // namespace videosource
