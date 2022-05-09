/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include <cstdint>
#include <assert.h>
#include <ostream>
#include <signal.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include "video_source/video_source.h"
#include "video_source/video_source_type.h"


using std::chrono::milliseconds;
using videosource::VideoSource;
using videosource::VideoSourceLogLevel;
using videosource::VideoSourceErrorCode;
using videosource::ImageFrame;
using videosource::PyramidFrame;
using videosource::HorizonVisionPixelFormat;

struct Context {
  volatile bool exit;
  Context() : exit(false) {}
};

Context g_ctx;

static void signal_handle(int param) {
  std::cout << "recv signal " << param << ", stop" << std::endl;
  if (param == SIGINT) {
    g_ctx.exit = true;
  }
}

enum TestType {
  kTEST_VIDEO_SOURCE_TYPE,
  KTEST_IMAGE_SOURCE_TYPE,
  kTEST_TYPE_MAX,
};

static int video_source_async_test(std::shared_ptr<VideoSource> &video_source) {
  int ret = -1;
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
    // try get ipu nv12 image frame
    std::vector<std::shared_ptr<ImageFrame>> ipu_image_list;
    ret = video_source->GetVpsImageFrame(ipu_image_list);
    if (ret) {
      std::cout << "get vps image frame failed, ret: " << ret << std::endl;
      return ret;
    }
    ret = video_source->FreeVpsImageFrame(ipu_image_list);
    if (ret) {
      std::cout << "free vps image frame failed, ret: " << ret << std::endl;
      return ret;
    }
    // try get pyramid image frame
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

static int video_source_sync_test(std::shared_ptr<VideoSource> &video_source) {
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
      std::cout << "get vin image frame failed, ret: " << ret << std::endl;
      return ret;
    }
  }
  // 2. get vps output
  if (vin_out_en == true && vps_en == true) {
    // 2.1 send vin frame to vps
    ret = video_source->SendFrameToVps(vin_image);
    if (ret) {
      std::cout << "send frame to vps failed, ret: " << ret << std::endl;
      return ret;
    }
    // 2.3 try get ipu nv12 image frame
    ret = video_source->GetVpsImageFrame(ipu_image_list);
    if (ret) {
      std::cout << "get vps image frame failed, ret: " << ret << std::endl;
      return ret;
    }
    // 2.3 try get pyramid image frame
    ret = video_source->GetPyramidFrame(pym_image);
    if (ret) {
      std::cout << "get pyramid frame failed, ret: " << ret << std::endl;
      return ret;
    }
  }
  // 3. free vin frame
  if (vin_out_en == true) {
    ret = video_source->FreeVinImageFrame(vin_image);
    if (ret) {
      std::cout << "free vin image frame failed, ret: " << ret << std::endl;
      return ret;
    }
  }
  // 4. free vps frame
  if (vin_out_en == true && vps_en == true) {
    ret = video_source->FreeVpsImageFrame(ipu_image_list);
    if (ret) {
      std::cout << "free vps image frame failed, ret: " << ret << std::endl;
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


static int video_source_test(
    TestType &test_type,
    const std::string &config_file,
    const std::string &run_mode,
    enum VideoSourceLogLevel &level) {
  int ret = -1;
  int channel_id = 0;
  if (test_type == TestType::kTEST_VIDEO_SOURCE_TYPE && run_mode != "loop") {
    auto video_source = std::make_shared<VideoSource>(channel_id, config_file);
    video_source->SetLoggingLevel(level);
    std::cout << "video source config: " << config_file << std::endl;
    ret = video_source->Init();
    if (ret) {
      std::cout << "video source init failed, ret: " << ret << std::endl;
      return ret;
    }
    ret = video_source->Start();
    if (ret) {
      std::cout << "video source start failed, ret: " << ret << std::endl;
      return ret;
    }
    bool is_sync_mode = video_source->GetSyncMode();
    if (run_mode == "normal") {
      while (!g_ctx.exit) {
        if (is_sync_mode == false) {
          ret = video_source_async_test(video_source);
          if (ret) {
            std::cout << "video source async test failed, ret: "
            << ret << std::endl;
          }
        } else {
          ret = video_source_sync_test(video_source);
          if (ret) {
            std::cout << "video source sync test failed, ret: "
            << ret << std::endl;
          }
        }
        if (ret == -VideoSourceErrorCode::kERROR_CODE_SOURCE_IS_STOP) {
          std::cout << "video source has not data, ret: " << ret << std::endl;
          g_ctx.exit = true;
        }
      }
    } else if (run_mode == "ut") {
        int run_time = 20000;
        auto start_time = std::chrono::system_clock::now();
        while (!g_ctx.exit) {
          if (is_sync_mode == false) {
            ret = video_source_async_test(video_source);
            if (ret) {
              std::cout << "video source async test failed, ret: "
              << ret << std::endl;
            }
          } else {
            ret = video_source_sync_test(video_source);
            if (ret) {
              std::cout << "video source sync test failed, ret: "
              << ret << std::endl;
            }
          }
          auto curr_time = std::chrono::system_clock::now();
          auto cost_time =
          std::chrono::duration_cast<std::chrono::milliseconds>(
            curr_time - start_time);
          if (cost_time.count() >= run_time) {
            std::cout << "run time end, cost_time:" << cost_time.count()
            << std::endl;
            g_ctx.exit = true;
          }
          if (ret == -VideoSourceErrorCode::kERROR_CODE_SOURCE_IS_STOP) {
            std::cout << "video source has not data, ret: " << ret << std::endl;
            g_ctx.exit = true;
          }
        }
      }
    video_source->Stop();
    video_source->DeInit();
  } else if (test_type == TestType::kTEST_VIDEO_SOURCE_TYPE &&
                                                run_mode == "loop") {
      std::cout << "start loop mode" << std::endl;
      auto video_source = std::make_shared<VideoSource>(channel_id,
                                                      config_file);
      video_source->SetLoggingLevel(level);
      std::cout << "video source config: " << config_file << std::endl;
      int run_time = 20000;
      bool is_sync_mode = video_source->GetSyncMode();
      while (!g_ctx.exit) {
        ret = video_source->Init();
        if (ret) {
          std::cout << "video source init failed, ret: " << ret << std::endl;
          return ret;
        }
        ret = video_source->Start();
        if (ret) {
          std::cout << "video source start failed, ret: " << ret << std::endl;
          return ret;
        }
        bool stop_flag = false;
        auto start_time = std::chrono::system_clock::now();
        while (!stop_flag && !g_ctx.exit) {
          if (is_sync_mode == false) {
            ret = video_source_async_test(video_source);
            if (ret) {
              std::cout << "video source async test failed, ret: "
              << ret << std::endl;
            }
          } else {
            ret = video_source_sync_test(video_source);
            if (ret) {
              std::cout << "video source sync test failed, ret: "
              << ret << std::endl;
            }
          }
          auto curr_time = std::chrono::system_clock::now();
          auto cost_time =
          std::chrono::duration_cast<std::chrono::milliseconds>(
          curr_time - start_time);
          if (cost_time.count() >= run_time) {
            std::cout << "run time end, cost_time:" << cost_time.count()
            << std::endl;
            stop_flag = true;
          }
        }
        if (ret == -VideoSourceErrorCode::kERROR_CODE_SOURCE_IS_STOP) {
          std::cout << "video source has not data, ret: " << ret << std::endl;
          g_ctx.exit = true;
        }
      video_source->Stop();
      video_source->DeInit();
    }
  }
    return 0;
}


static int image_source_test(
    const std::string &config_file,
    enum VideoSourceLogLevel &level,
    const std::string &image_name,
    const uint32_t &width,
    const uint32_t &height,
    const HorizonVisionPixelFormat &pixel_format) {
  int ret = -1;

  std::cout << "image source test,"
    << " config_file: " << config_file
    << " log_level: " << level
    << " image_name: " << image_name
    << " image_width: " << width
    << " image_height: " << height
    << " image_format: " << pixel_format
    << std::endl;

  if (pixel_format == videosource::kHorizonVisionPixelFormatNV12
      || pixel_format == videosource::kHorizonVisionPixelFormatJPEG) {
  } else {
    std::cout << "Unsupport image source pixel format: "
      << pixel_format << std::endl;
    return -1;
  }

  auto video_source = std::make_shared<VideoSource>(config_file);
  video_source->SetLoggingLevel(level);

  ret = video_source->ReadImage(image_name, width, height, pixel_format);
  if (ret) {
    std::cout << "read image: " << image_name
      << " failed, ret: " << ret << std::endl;
    return ret;
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


int main(int argc, char **argv) {
  int ret = -1;
  std::string test_type_str;
  TestType test_type;
  std::string config_file;
  std::string log_level;
  std::string image_name;
  std::string run_mode = "normal";
  uint32_t image_width, image_height;
  enum HorizonVisionPixelFormat image_format;
  if (argc < 3) {
    std::cout << "Usage: ./sample video_source_config_file"
              << " [-i/-d/-w/-f] " << std::endl;
  }
  if (argv[1] != nullptr) {
    run_mode.assign(argv[1]);
    if (run_mode != "ut"
        && run_mode != "normal"
        && run_mode != "loop") {
      std::cout << "not support mode: " << run_mode << std::endl;
      return 0;
    }
  }
  std::cout << "run_mode: " << run_mode << std::endl;
  if (argv[2] == nullptr) {
    test_type = TestType::kTEST_VIDEO_SOURCE_TYPE;
  } else {
    test_type_str = argv[2];
    int tmp = atoi(test_type_str.c_str());
    test_type = static_cast<TestType>(tmp);
  }
  if (argv[3] == nullptr) {
    std::cout << "set default video_source config: "
      << "[./configs/video_source/x3dev/feedback/x3_feedback_1080p_chn0.json]"
      << std::endl;
    config_file =
      "./configs/video_source/x3dev/feedback/x3_feedback_1080p_chn0.json";
  } else {
    config_file = argv[3];
  }
  if (argv[4] == nullptr) {
    std::cout << "set default log level: [-i] ";
    log_level = "-i";
  } else {
    log_level = argv[4];
  }
  std::cout << "test type: " << test_type << std::endl;
  if (test_type == TestType::KTEST_IMAGE_SOURCE_TYPE) {
    if (argv[5] == nullptr) {
    std::cout << "set default image name: "
      "[./configs/video_source/x3dev/feedback/data/1080p.jpg] ";
    image_name = "./configs/video_source/x3dev/feedback/data/1080p.jpg";
    } else {
      image_name = argv[5];
    }
    if (argv[6] == nullptr) {
    image_width = 1920;
    } else {
      image_width = atoi(argv[6]);
    }
    if (argv[7] == nullptr) {
    image_height = 1080;
    } else {
      image_height = atoi(argv[7]);
    }
    if (argv[8] == nullptr) {
      image_format = HorizonVisionPixelFormat::kHorizonVisionPixelFormatJPEG;
    } else {
      int image_format_tmp = atoi(argv[8]);
      image_format = static_cast<HorizonVisionPixelFormat>(image_format_tmp);
    }
  }

  signal(SIGINT, signal_handle);
  signal(SIGPIPE, signal_handle);
  signal(SIGSEGV, signal_handle);

  enum VideoSourceLogLevel level;
  if (log_level == "-i") {
    level = VideoSourceLogLevel::kLOG_LEVEL_INFO;
  } else if (log_level == "-d") {
    level = VideoSourceLogLevel::kLOG_LEVEL_DEBUG;
  } else if (log_level == "-w") {
    level = VideoSourceLogLevel::kLOG_LEVEL_WARN;
  } else if (log_level == "-e") {
    level = VideoSourceLogLevel::kLOG_LEVEL_ERROR;
  } else if (log_level == "-f") {
    level = VideoSourceLogLevel::kLOG_LEVEL_FATAL;
  } else {
    level = VideoSourceLogLevel::kLOG_LEVEL_INFO;
    std::cout << "set default log level: [-i] " << std::endl;
  }
  if (test_type == TestType::kTEST_VIDEO_SOURCE_TYPE) {
    ret = video_source_test(test_type, config_file, run_mode, level);
    assert(ret == 0);
    std::cout << "video source test success..." << std::endl;
  } else if (test_type == TestType::KTEST_IMAGE_SOURCE_TYPE &&
                    (run_mode == "normal" || run_mode == "loop")) {
    std::string config_file =
      "./configs/video_source/x3dev/feedback/x3_image_feedback.json";
    while (!g_ctx.exit) {
    image_source_test(config_file, level, image_name,
        image_width, image_height, image_format);
    }
    std::cout << "image source test success..." << std::endl;
  } else if (test_type == TestType::KTEST_IMAGE_SOURCE_TYPE &&
                                                    run_mode == "ut") {
    std::string config_file =
      "./configs/video_source/x3dev/feedback/x3_image_feedback.json";
    ret = image_source_test(config_file, level, image_name,
        image_width, image_height, image_format);
    assert(ret == 0);
    std::cout << "image source test success..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(10));
    std::cout << "run ut time end" << std::endl;
  } else {
    std::cout << "Unsupport test_type, val: " << test_type;
    return -1;
  }

  return 0;
}
