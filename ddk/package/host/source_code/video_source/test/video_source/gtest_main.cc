// Copyright 2006, Google Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//     * Neither the name of Google Inc. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <getopt.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <fstream>
#include "gtest/gtest.h"
#include "hobotlog/hobotlog.hpp"
#include "./video_source_gtest.h"

CmdOptions cmd_options;

static const char short_options[] = "W:H:T:C:L:F:I:O:h";
static const struct option long_options[] = {
  {"width", required_argument, nullptr, 'W'},
  {"height", required_argument, nullptr, 'H'},
  {"input_type", required_argument, nullptr, 'T'},
  {"config_file", required_argument, nullptr, 'C'},
  {"log_level", required_argument, nullptr, 'L'},
  {"pixel_format", required_argument, nullptr, 'F'},
  {"input_file_name", required_argument, nullptr, 'I'},
  {"output_file_path", required_argument, nullptr, 'O'},
  {"help", required_argument, nullptr, 'h'},
  {nullptr, 0, nullptr, 0}};

static std::map<std::string, enum HorizonVisionPixelFormat> g_pix_fmt_map = {
  {"raw", videosource::kHorizonVisionPixelFormatRaw},
  {"yuy2", videosource::kHorizonVisionPixelFormatYUY2},
  {"nv12", videosource::kHorizonVisionPixelFormatNV12},
  {"pym", videosource::kHorizonVisionPixelFormatPYM},
  {"jpeg", videosource::kHorizonVisionPixelFormatJPEG},
  {"mjpeg", videosource::kHorizonVisionPixelFormatMJPEG},
  {"h264", videosource::kHorizonVisionPixelFormatH264},
  {"h265", videosource::kHorizonVisionPixelFormatH265},
};

void print_usage(const char *prog)
{
  LOGI << "Usage: " << prog;
  puts("  -W --width             \t video source input image width"
         " for ReadImage interface\n"
      "  -H --height            \t video source input image height"
         " for ReadImage interface\n"
      "  -T --input_type        \t video source input type(video or image)"
      "  -C --config_file       \t video source input config file"
      "  -L --log_level         \t video source log level"
      "  -F --pixel_format      \t video source input image pixel format"
      "  -I --input_file_name   \t input JPEG or NV12 file name for"
         " video feedback or ReadImage interface\n"
      "  -O --output_file_path  \t video source output file path\n"
      "  -h --help              \t print usage\n");
  exit(1);
}

static int parse_log_level(const std::string &log_level) {
  enum VideoSourceLogLevel level;
  if (log_level == "-i") {
    level = VideoSourceLogLevel::kLOG_LEVEL_INFO;
    SetLogLevel(HOBOT_LOG_INFO);
  } else if (log_level == "-d") {
    level = VideoSourceLogLevel::kLOG_LEVEL_DEBUG;
    SetLogLevel(HOBOT_LOG_DEBUG);
  } else if (log_level == "-w") {
    level = VideoSourceLogLevel::kLOG_LEVEL_WARN;
    SetLogLevel(HOBOT_LOG_WARN);
  } else if (log_level == "-e") {
    level = VideoSourceLogLevel::kLOG_LEVEL_ERROR;
    SetLogLevel(HOBOT_LOG_ERROR);
  } else if (log_level == "-f") {
    level = VideoSourceLogLevel::kLOG_LEVEL_FATAL;
    SetLogLevel(HOBOT_LOG_FATAL);
  } else {
    level = VideoSourceLogLevel::kLOG_LEVEL_INFO;
    SetLogLevel(HOBOT_LOG_INFO);
    std::cout << "set default log level: [-i] " << std::endl;
  }
  cmd_options.mGlobalLogLevel = level;
  return 0;
}

static int parse_pixfmt_func(const std::string &pix_fmt) {
  int image_format_tmp = atoi(pix_fmt.c_str());
  cmd_options.mGlobalPixFmt =
    static_cast<HorizonVisionPixelFormat>(image_format_tmp);
  return 0;
}

static bool path_exists(const std::string &fn, bool is_dir) {
  if (is_dir) {
    struct stat myStat = {0};
    if ((stat(fn.c_str(), &myStat) != 0) || !S_ISDIR(myStat.st_mode)) {
      LOGE << "Directory [ " << fn << "] does not exist.";
      return false;
    }
    return true;
  } else {
    std::ifstream file_path(fn.c_str());
    return file_path.good();
  }
}

static int check_filepath_valid(const std::string& file_path) {
  bool bret;

  bret = path_exists(file_path, true);
  if (!bret) {
    LOGE << "input file path: " << file_path << " does not exist!";
    HOBOT_CHECK(bret == true);
  }
  return 0;
}

static int check_filename_valid(const std::string& file_name) {
  bool bret;

  bret = path_exists(file_name, false);
  if (!bret) {
    LOGE << "input file name: " << file_name << " does not exist!";
    HOBOT_CHECK(bret == true);
  }
  return 0;
}

int parse_opts(int argc, char *argv[]) {
  int ret;

  while (1) {
    int cmd_ret;

    cmd_ret =
      getopt_long(argc, argv, short_options, long_options, NULL);

    if (cmd_ret == -1)
      break;
    LOGD << " cmd_ret:" << cmd_ret << " optarg:" << optarg;
    switch (cmd_ret) {
      case 'W':
        {
          cmd_options.mGlobalWidth = atoi(optarg);
          LOGI << "mGlobalWidth: " << cmd_options.mGlobalWidth;
          break;
        }
      case 'H':
        {
          cmd_options.mGlobalHeight = atoi(optarg);
          LOGI << "mGlobalHeight: " << cmd_options.mGlobalHeight;
          break;
        }
      case 'T':
        {
          int tmp = atoi(optarg);
          TestType test_type = static_cast<TestType>(tmp);
          cmd_options.mGlobalType = test_type;
          LOGI << "mGlobalType: " << cmd_options.mGlobalType;
          break;
        }
      case 'C':
        {
          ret = check_filename_valid(optarg);
          if (ret) {
            LOGE << "parse input config file path error!!!";
            return -1;
          }
          cmd_options.mGlobalConfigFile = optarg;
          LOGI << "mGlobalConfigFile: " << cmd_options.mGlobalConfigFile;
          break;
        }
      case 'L':
        {
          parse_log_level(optarg);
          LOGI << "mGlobalLogLevel: " << cmd_options.mGlobalLogLevel;
          break;
        }
      case 'F':
        {
          ret = parse_pixfmt_func(optarg);
          if (ret) {
            LOGE << "parse pixel format error!!!";
            return -1;
          }
          break;
        }
      case 'I':
        {
          ret = check_filename_valid(optarg);
          if (ret) {
            LOGE << "parse input file path error!!!";
            return -1;
          }
          cmd_options.mGlobalInFileName = optarg;
          LOGI << "mGlobalInFileName: " << cmd_options.mGlobalInFileName;
          break;
        }
      case 'O':
        {
          ret = check_filepath_valid(optarg);
          if (ret) {
            LOGE << "parse output file path error!!!";
            return -1;
          }
          cmd_options.mGlobalOutFilePath = optarg;
          LOGI << "mGlobalOutFilePath: " << cmd_options.mGlobalOutFilePath;
          break;
        }
      case 'h':
        {
          print_usage(argv[0]);
          break;
        }
    }
  }
  return 0;
}

GTEST_API_ int main(int argc, char **argv) {
  int ret;
  printf("Running main() from gtest_main.cc\n");
  SetLogLevel(HOBOT_LOG_INFO);
  testing::InitGoogleTest(&argc, argv);

  ret = parse_opts(argc, argv);
  if (ret) {
    return ret;
  }
  return RUN_ALL_TESTS();
}

