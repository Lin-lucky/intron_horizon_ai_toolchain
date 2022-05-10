/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: example_fb.cpp
 * @Brief:
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-04-15 14:27:05
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-04-15 15:18:10
 */
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <iostream>
// #include "./vio_wrapper_global.h"

static void Usage() {
  std::cout << "./example do_fb hb_cfg_file file_name" << std::endl;
}
int DoFB(int argc, char **argv) {
  if (argc < 3) {
    Usage();
    return 1;
  }
  const char *cfg_file = argv[1];
  const char *file_name = argv[2];
  char result_name[120];
  FILE *save_fd;
  uint32_t ew, eh;
  HbVioFbWrapperGlobal fb(cfg_file);
  fb.Init();
  auto pym = fb.GetImgInfo(file_name, &ew, &eh);
  snprintf(result_name, sizeof(result_name), "down_4.yuv");
  save_fd = fopen(result_name, "w+");
  if (!save_fd) {
    printf("Can't open the save file\n");
    return 1;
  }
  int y_size =
      pym->down_scale[0].width * pym->down_scale[0].height;
  int uv_size = y_size >> 1;
  printf("frwite size: %d, %d", y_size, uv_size);
  int len = fwrite(
      reinterpret_cast<uint8_t *>(pym->down_scale[0].y_vaddr),
      1,
      y_size,
      save_fd);
  if (len <= 0) {
    printf("Write file Error1\n");
    return 1;
  }
  len = fwrite(
      reinterpret_cast<uint8_t *>(pym->down_scale[0].c_vaddr),
      1,
      uv_size,
      save_fd);
  if (len <= 0) {
    printf("Write file Error2\n");
    return 1;
  }
  fb.FreeImgInfo(pym);
  return 0;
}
