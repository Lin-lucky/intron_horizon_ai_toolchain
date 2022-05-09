#include <iostream>
#include <fstream>
#include <memory>
#include <signal.h>
#include <assert.h>
#include <gtest/gtest.h>
#include "uvc_server_gtest.h"
#include "hobotlog/hobotlog.hpp"

#include "uvc_server/uvc_server.h"

const char *picture_path[] = {
  "./data/1080p.nv12",
  "./data/1080p_pic.mjpeg",
  "./data/chn0.h264"
};

using uvccomponent::UvcEvent;
using uvccomponent::UvcEventCallback;
using uvccomponent::UvcServer;
using uvc_gtest::TestTransfer;

std::shared_ptr<TestTransfer> uvc_sender = std::make_shared<TestTransfer>();
// Send nv12 data success
TEST(UvcServerTest, NV12) {
  int ret = -1;
  int run_time = 30000;  // 30s
  ret = uvc_sender->Init();
  ASSERT_EQ(ret, 0) << "uvc server init failed, ret: " << ret;
  ret = uvc_sender->Start();
  ASSERT_EQ(ret, 0) << "uvc server start failed, ret: " << ret;
  std::ifstream ifs(picture_path[0], std::ios::in | std::ios::binary);
  if (!ifs) {
    std::cout << "[ERROR] Open nv12 file: " << picture_path[0] << " failed"
              << std::endl;
  }
  ifs.seekg(0, std::ios::end);
  int img_length = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  char *img_nv12 = new char[sizeof(char) * img_length];
  ifs.read(img_nv12, img_length);
  ifs.close();

  int height = 1080;
  int width = 1920;
  auto start_time = std::chrono::system_clock::now();
  while (1) {
    uvc_sender->SendNv12Data(img_nv12, img_length, width, height);
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    auto curr_time = std::chrono::system_clock::now();
    auto cost_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(
        curr_time - start_time);
    if (cost_time.count() >= run_time) {
      LOGW << "run time end,cost_time: " << cost_time.count();
      break;
    }
  }
}

// Send mjpeg picture data success
TEST(UvcServerTest, MJPEG_PIC) {
  int ret = -1;
  int run_time = 30000;  // 30s
  ret = uvc_sender->Start();
  ASSERT_EQ(ret, 0) << "uvc server start failed, ret: " << ret;
  std::ifstream ifs(picture_path[1], std::ios::in | std::ios::binary);
  if (!ifs) {
    std::cout << "[ERROR] Open mjpeg file: " << picture_path[1] << " failed"
              << std::endl;
  }
  ifs.seekg(0, std::ios::end);
  int img_length = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  char *img_mjpeg = new char[sizeof(char) * img_length];
  ifs.read(img_mjpeg, img_length);
  ifs.close();

  int height = 1080;
  int width = 1920;
  auto start_time = std::chrono::system_clock::now();
  while (1) {
    uvc_sender->SendNv12Data(img_mjpeg, img_length, width, height);
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    auto curr_time = std::chrono::system_clock::now();
    auto cost_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(
        curr_time - start_time);
    if (cost_time.count() >= run_time) {
      LOGW << "run time end,cost_time: " << cost_time.count();
      break;
    }
  }
}

// Send H264 data success
TEST(UvcServerTest, H264) {
  int ret = -1;
  int run_time = 30000;  // 30s
  ret = uvc_sender->Start();
  EXPECT_EQ(ret, 0) << "uvc server start failed, ret: " << ret;

  // get AVPacket
  AVPacket packet;
  bool restart_flag = true;
  AVFormatContext* input_ctx = NULL;
  auto start_time = std::chrono::system_clock::now();
  while (1) {
    if (restart_flag == true) {
      // ffmpeg init
      ret = avformat_open_input(&input_ctx, picture_path[2], NULL, NULL);
      EXPECT_EQ(ret, 0) << "open media file failed, ret: " << ret;
      ret = av_find_best_stream(input_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, NULL, 0);
      EXPECT_EQ(ret, 0) << "av_find_best_stream failed, ret: " << ret;
    }
    restart_flag = false;
    auto curr_time = std::chrono::system_clock::now();
    auto cost_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          curr_time - start_time);
    if (cost_time.count() >= run_time) {
      LOGW << "run time end, cost_time: " << cost_time.count();
      break;
    }
    ret = av_read_frame(input_ctx, &packet);
    if (ret < 0) {
      av_packet_unref(&packet);
      avformat_close_input(&input_ctx);
      restart_flag = true;
      continue;
    } else {
      // forward H264 file data
      uvc_sender->SendH264Data(packet.data, packet.size);
    }
    av_packet_unref(&packet);
    usleep(40000);
  }
  avformat_close_input(&input_ctx);
  ret = uvc_sender->Stop();
  EXPECT_EQ(ret, 0) << "uvc server stop failed, ret: " << ret;
  ret = uvc_sender->DeInit();
  EXPECT_EQ(ret, 0) << "uvc server deinit failed, ret: " << ret;
}

// create uvc server and init success
TEST(UvcServerTest, Init) {
  int ret = -1;
  uvc_gtest::TestTransfer UvcEvt;
  std::shared_ptr<UvcServer> uvc_server = UvcServer::GetInstance();
  ret = uvc_server->Init(&UvcEvt);
  EXPECT_EQ(ret, 0) << "uvc server init failed, ret: " << ret;
}

// create uvc server and deinit success
TEST(UvcServerTest, DeInit) {
  int ret = -1;
  std::shared_ptr<UvcServer> uvc_server = UvcServer::GetInstance();
  ret = uvc_server->DeInit();
  EXPECT_EQ(ret, 0) << "uvc server deinit failed, ret: " << ret;
}

// create uvc server and start success
TEST(UvcServerTest, Start) {
  int ret = -1;
  std::shared_ptr<UvcServer> uvc_server = UvcServer::GetInstance();
  ret = uvc_server->Start();
  EXPECT_EQ(ret, 0) << "uvc server start failed, ret: " << ret;
}

// create uvc server and stop success
TEST(UvcServerTest, Stop) {
  int ret = -1;
  std::shared_ptr<UvcServer> uvc_server = UvcServer::GetInstance();
  ret = uvc_server->Stop();
  EXPECT_EQ(ret, 0) << "uvc server stop failed, ret: " << ret;
}

// create uvc server and startprocess success
TEST(UvcServerTest, Startprocess) {
  int ret = -1;
  uvc_gtest::TestTransfer UvcEvt;
  std::shared_ptr<UvcServer> uvc_server = UvcServer::GetInstance();
  ret = uvc_server->Init(&UvcEvt);
  ASSERT_EQ(ret, 0) << "uvc server init failed, ret: " << ret;
  ret = uvc_server->Start();
  ASSERT_EQ(ret, 0) << "uvc server start failed, ret: " << ret;
  ret = uvc_server->Stop();
  ASSERT_EQ(ret, 0) << "uvc server stop failed, ret: " << ret;
  ret = uvc_server->DeInit();
  ASSERT_EQ(ret, 0) << "uvc server deinit failed, ret: " << ret;
}
