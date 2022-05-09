/**
 *  Copyright (C) 2021 Horizon Robotics Inc.
 *  All rights reserved.
 *  @Author: yong.wu
 *  @Email: <yong.wu@horizon.ai>
 *  @Date: 2021-04-12
 *  @Version: v0.0.1
 *  @Brief: implemenation of decoder manager.
 */
#include "video_source/decode/decode_manager.h"
#include <memory>
#include <stdio.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <mutex>
#include <queue>
#include "hobotlog/hobotlog.hpp"

namespace videosource {

DecodeHandle DecodeManager::CreateDecodeHandle(
    const HorizonVisionPixelFormat &input_fmt) {
  std::lock_guard<std::mutex> lg(mutex_);
  int ret = -1;
  int decode_chn = -1;
  int output_chn_id = -1;
  DecodeHandle handle = nullptr;

  if (input_fmt < kHorizonVisionPixelFormatJPEG
      || input_fmt > kHorizonVisionPixelFormatH265) {
    LOGE << "UnSupport input format: " << input_fmt;
    return handle;
  }

  ret = ModuleInit(input_fmt);
  if (ret) {
    LOGE << "decode module init failed, ret: " << ret;
    return handle;
  }

  ret = GetDecodeIdFromFifo(input_fmt, decode_chn);
  if (ret) {
    LOGE << "get decode id from fifo failed, ret: " << ret;
    return handle;
  }
  output_chn_id = decode_chn;

  DecChnInfo* chn_info = new DecChnInfo();
  chn_info->channel_id = output_chn_id;
  chn_info->format = input_fmt;
  handle = reinterpret_cast<DecodeHandle>(chn_info);

  return handle;
}

int DecodeManager::FreeDecodeHandle(IN DecodeHandle handle) {
  std::lock_guard<std::mutex> lg(mutex_);
  int ret = -1;
  DecChnInfo* chn_info = reinterpret_cast<DecChnInfo*>(handle);
  if (!chn_info) {
    LOGE << "decode chn info is nullptr";
    return -1;
  }
  HorizonVisionPixelFormat input_fmt = chn_info->format;
  int input_chn_id = chn_info->channel_id;
  delete chn_info;

  ret = FreeDecodeIdToFifo(input_fmt, input_chn_id);
  if (ret) {
    LOGE << "free decode id to fifo failed, ret: " << ret;
    return ret;
  }

  ret = ModuleDeInit(input_fmt);
  if (ret) {
    LOGE << "decode module deinit failed, ret: " << ret;
    return ret;
  }

  return 0;
}

int DecodeManager::ModuleInit(const HorizonVisionPixelFormat &input_fmt) {
  int ret = -1;
  if (m_ref_cnt_++ == 0 && init_flag_ == false) {
    ret = HB_VDEC_Module_Init();
    HOBOT_CHECK(ret == 0) << "decode media module init failed";
    init_flag_ = true;
  }
  ret = CreateDecodeIdFifo(input_fmt);
  if (ret) {
    LOGE << "create decode id fifo failed, ret: " << ret;
    return ret;
  }
  LOGI << "decode manager init success, ref_cnt: " << m_ref_cnt_;
  return 0;
}

int DecodeManager::ModuleDeInit(const HorizonVisionPixelFormat &input_fmt) {
  int ret = -1;
  m_ref_cnt_--;
  if (m_ref_cnt_ <= 0 && init_flag_) {
    int ret = HB_VDEC_Module_Uninit();
    HOBOT_CHECK(ret == 0) << "decode media module deinit failed";
    init_flag_ = false;
  }
  ret = DestroyDecodeIdFifo(input_fmt);
  if (ret) {
    LOGE << "destroy decode id fifo failed, ret: " << ret;
    return ret;
  }
  LOGI << "quit decode manager deinit, ref_cnt: " << m_ref_cnt_;
  return 0;
}

int DecodeManager::TryTestFifo(const std::string &input_fifo_name) {
  int ret = -1;
  int fifo_fd;
  VpuIdStatus vpu_chn_id = { 0 };
  VpuIdStatus jpu_chn_id = { 0 };

  ret = open(input_fifo_name.c_str(), O_RDWR | O_NONBLOCK);
  if (ret < 0) {
    perror("open decode manager fifo");
    return -1;
  }
  fifo_fd = ret;

  if (input_fifo_name == VPU_FIFO_NAME) {
    ret = read(fifo_fd, &vpu_chn_id, sizeof(VpuIdStatus));
    if (ret <= 0) {
      perror("read vpu fifo");
      return -1;
    }
    // write callback data to vpu fifo
    ret = write(fifo_fd, &vpu_chn_id, sizeof(VpuIdStatus));
    if (ret != sizeof(VpuIdStatus)) {
      perror("write vpu fifo");
      return -1;
    }
  } else if (input_fifo_name == JPU_FIFO_NAME) {
    ret = read(fifo_fd, &jpu_chn_id, sizeof(JpuIdStatus));
    if (ret <= 0) {
      perror("read jpu fifo");
      return -1;
    }
    // write callback data to jpu fifo
    ret = write(fifo_fd, &jpu_chn_id, sizeof(JpuIdStatus));
    if (ret != sizeof(JpuIdStatus)) {
      perror("write jpu fifo");
      return -1;
    }
  } else {
    LOGE << "UnSupport input_fifo_name: " << input_fifo_name;
    return -1;
  }
  return 0;
}

int DecodeManager::CreateSharedFifo(const std::string &input_fifo_name,
    const int &input_max_num, int &output_fifo_fd) {
  int ret = -1;
  int try_time = 0;
  VpuIdStatus vpu_chn_id = { 0 };
  JpuIdStatus jpu_chn_id = { 0 };

  do {
    auto start_time = std::chrono::system_clock::now();
    ret = mkfifo(input_fifo_name.c_str(), 0666);
    auto curr_time = std::chrono::system_clock::now();
    auto cost_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          curr_time - start_time).count();
    LOGW << "mkfifo " << input_fifo_name
      << " cost_time: " << cost_time << "ms";
    if (ret == 0) {
      if (input_fifo_name == VPU_FIFO_NAME) {
        vpu_is_main_process_ = true;
      } else if (input_fifo_name == JPU_FIFO_NAME) {
        jpu_is_main_process_ = true;
      } else {
        LOGE << "UnSupport input_fifo_name: " << input_fifo_name;
        return -1;
      }
      LOGW << "create decode manager,"
        << " input_fifo_name: " << input_fifo_name
        << " success, this is main process";
      break;
    } else {
      // maybe fifo has been created
      perror("mkfifo decode manager");
      LOGE << "create decoder manager fifo failed, ret: " << ret
        << " fifo maybe has been created";
      ret = TryTestFifo(input_fifo_name);
      if (ret) {
        // last same process exception quit
        LOGE << "try test fifo failed, ret: " << ret;
        unlink(input_fifo_name.c_str());
      } else {
        break;  // other process normal open fifo
      }
    }
  } while (try_time++ < 3);

  if (ret) {
    LOGE << "create input fifo failed, ret: " << ret;
    return -1;
  }

  try_time = 0;
  do {
    ret = open(input_fifo_name.c_str(), O_RDWR | O_NONBLOCK);
    if (ret < 0) {
      LOGE << "open fifo failed, ret: " << ret;
    } else {
      LOGI << "codec manager get correct fd: " << ret;
      break;
    }
  } while (try_time++ < 3);

  if (ret <= 0) {
    LOGE << "open input fifo failed, ret: " << ret;
    unlink(input_fifo_name.c_str());
    return -1;
  }
  output_fifo_fd = ret;

  if (input_fifo_name == VPU_FIFO_NAME) {
    if (vpu_is_main_process_ == false) {
      LOGI << "this process is not main process"
        << " input_fifo_name: " << input_fifo_name;
      return 0;  // other process normal open fifo
    }
  } else if (input_fifo_name == JPU_FIFO_NAME) {
    if (jpu_is_main_process_ == false) {
      LOGI << "this process is not main process"
        << " input_fifo_name: " << input_fifo_name;
      return 0;  // other process normal open fifo
    }
  } else {
    LOGE << "UnSupport input_fifo_name: " << input_fifo_name;
    return -1;
  }

  /**
   * attention:
   * vpu and jpu can not use same channel_id in hapi interface,
   * so the scope of use is divided, such as
   * vpu channel id [0 ~ 31]
   * jpu channel id [32 ~ 63]
   *
   * ps: vpu and jpu may be use same channel_id in vapi interface
   */
  int start_id;
  if (input_fifo_name == VPU_FIFO_NAME) {
    for (start_id = 0; start_id < VPU_FIFO_NUM; start_id++) {
      vpu_chn_id.status[start_id] = 255;  // 1 is used, 255 is free
    }
    ret = write(output_fifo_fd, &vpu_chn_id, sizeof(VpuIdStatus));
    if (ret != sizeof(VpuIdStatus)) {
      LOGE << "write input vpu chn id to fifo failed, ret: " << ret
        << " fifo_fd: " << output_fifo_fd
        << " errno: " << errno;
      return -1;
    }
  } else if (input_fifo_name == JPU_FIFO_NAME) {
    for (start_id = VPU_FIFO_NUM; start_id < JPU_FIFO_NUM; start_id++) {
      jpu_chn_id.status[start_id] = 255;  // 0 is unused, 1 is used, 255 is free
    }
    ret = write(output_fifo_fd, &jpu_chn_id, sizeof(JpuIdStatus));
    if (ret != sizeof(JpuIdStatus)) {
      LOGE << "write input vpu chn id to fifo failed, ret: " << ret
        << " fifo_fd: " << output_fifo_fd
        << " errno: " << errno;
      return -1;
    }
  } else {
    LOGE << "UnSupport input_fifo_name: " << input_fifo_name;
    return -1;
  }
  return 0;
}

int DecodeManager::DestroySharedFifo(const std::string &input_fifo_name,
    const int &input_fifo_fd) {

  if (input_fifo_fd > 0) {
    close(input_fifo_fd);
  }
  if (input_fifo_name == VPU_FIFO_NAME) {
    if (vpu_is_main_process_ == true) {
      vpu_fifo_fd_ = -1;
      vpu_is_main_process_ = false;
      unlink(input_fifo_name.c_str());
    }
  } else if (input_fifo_name == JPU_FIFO_NAME) {
    if (jpu_is_main_process_ == true) {
      jpu_fifo_fd_ = -1;
      jpu_is_main_process_ = false;
      unlink(input_fifo_name.c_str());
    }
  } else {
    LOGE << "UnSupport input_fifo_name: " << input_fifo_name;
    return -1;
  }
  return 0;
}

int DecodeManager::CreateDecodeIdFifo(
    const HorizonVisionPixelFormat &input_fmt) {
  int ret = -1;

  if (input_fmt < kHorizonVisionPixelFormatJPEG
      || input_fmt > kHorizonVisionPixelFormatH265) {
    LOGE << "UnSupport input format: " << input_fmt;
    return -1;
  }
  if (input_fmt == kHorizonVisionPixelFormatJPEG
      || input_fmt == kHorizonVisionPixelFormatMJPEG) {
    if (jpu_fifo_fd_ < 0) {
      ret = CreateSharedFifo(JPU_FIFO_NAME, JPU_FIFO_NUM, jpu_fifo_fd_);
      if (ret) {
        LOGE << "create vpu shared fifo failed, ret: " << ret;
        return ret;
      }
    }
  }
  if (input_fmt == kHorizonVisionPixelFormatH264
      || input_fmt == kHorizonVisionPixelFormatH265) {
    if (vpu_fifo_fd_ < 0) {
      ret = CreateSharedFifo(VPU_FIFO_NAME, VPU_FIFO_NUM, vpu_fifo_fd_);
      if (ret) {
        LOGE << "create vpu shared fifo failed, ret: " << ret;
        return ret;
      }
    }
  }
  return 0;
}

int DecodeManager::DestroyDecodeIdFifo(
    const HorizonVisionPixelFormat &input_fmt) {
  if (input_fmt == kHorizonVisionPixelFormatJPEG ||
      input_fmt == kHorizonVisionPixelFormatMJPEG) {
    DestroySharedFifo(JPU_FIFO_NAME, jpu_fifo_fd_);
  }
  if (input_fmt == kHorizonVisionPixelFormatH264 ||
      input_fmt == kHorizonVisionPixelFormatH265) {
    DestroySharedFifo(VPU_FIFO_NAME, vpu_fifo_fd_);
  }
  return 0;
}

int DecodeManager::GetDecodeIdFromFifo(
    const HorizonVisionPixelFormat &input_fmt,
    int &output_decode_id) {
  int ret, recv;
  std::string fifo_name;
  int fifo_fd;
  VpuIdStatus vpu_chn_id = { 0 };
  JpuIdStatus jpu_chn_id = { 0 };

  if (input_fmt == kHorizonVisionPixelFormatJPEG ||
      input_fmt == kHorizonVisionPixelFormatMJPEG) {
    fifo_name = JPU_FIFO_NAME;
    fifo_fd = jpu_fifo_fd_;
    // from front to behind find for jpu fifo
    ret = read(fifo_fd, &jpu_chn_id, sizeof(JpuIdStatus));
    if (ret <= 0) {
      perror("read vpu id fifo");
      return -1;
    }
    for (recv = 0; recv < JPU_FIFO_NUM; recv++) {
      if (jpu_chn_id.status[recv] == 255) {  // 255 is free
        break;
      }
    }
    if (recv == JPU_FIFO_NUM) {
      LOGE << "all id has been used, get jpu chn_id failed";
      return -1;
    }
    jpu_chn_id.status[recv] = 1;  // used
    ret = write(fifo_fd, &jpu_chn_id, sizeof(JpuIdStatus));
    if (ret != sizeof(JpuIdStatus)) {
      LOGE << "write input chn id to jpu fifo failed, ret: " << ret
        << " fifo_fd: " << fifo_fd
        << " errno: " << errno;
      return -1;
    }
  } else if (input_fmt == kHorizonVisionPixelFormatH264 ||
      input_fmt == kHorizonVisionPixelFormatH265) {
    fifo_name = VPU_FIFO_NAME;
    fifo_fd = vpu_fifo_fd_;
    // from front to behind find for vpu fifo
    ret = read(fifo_fd, &vpu_chn_id, sizeof(VpuIdStatus));
    if (ret <= 0) {
      perror("read vpu fifo");
    }
    for (recv = 0; recv < VPU_FIFO_NUM; recv++) {
      if (vpu_chn_id.status[recv] == 255) {  // 255 is free
        break;
      }
    }
    if (recv == VPU_FIFO_NUM) {
      LOGE << "all id has been used, get vpu chn_id failed";
      return -1;
    }
    vpu_chn_id.status[recv] = 1;  // used
    ret = write(fifo_fd, &vpu_chn_id, sizeof(VpuIdStatus));
    if (ret != sizeof(VpuIdStatus)) {
      LOGE << "write input chn id to vpu fifo failed, ret: " << ret
        << " fifo_fd: " << fifo_fd
        << " errno: " << errno;
      return -1;
    }
  } else {
      LOGE << "Unsupport pixel format: " << input_fmt;
      return -1;
  }

  output_decode_id = recv;
  LOGW << "get decode_chn: " << output_decode_id << " from "
    << fifo_name << " success...";
  return 0;
}

int DecodeManager::FreeDecodeIdToFifo(const HorizonVisionPixelFormat &input_fmt,
    int &input_decode_id) {
  int ret = -1;
  int fifo_fd;
  VpuIdStatus vpu_chn_id = { 0 };
  JpuIdStatus jpu_chn_id = { 0 };

  if (input_fmt == kHorizonVisionPixelFormatJPEG ||
      input_fmt == kHorizonVisionPixelFormatMJPEG) {
    if (input_decode_id < 0 || input_decode_id >= JPU_FIFO_NUM) {
      LOGE << "input decode id is invalid, id: " << input_decode_id;
      return -1;
    }
    fifo_fd = jpu_fifo_fd_;
    ret = read(fifo_fd, &jpu_chn_id, sizeof(JpuIdStatus));
    if (ret <= 0) {
      perror("read jpu fifo");
      return -1;
    }
    jpu_chn_id.status[input_decode_id] = 255;  // not used
    // write callback data to jpu fifo
    ret = write(fifo_fd, &jpu_chn_id, sizeof(JpuIdStatus));
    if (ret != sizeof(JpuIdStatus)) {
      perror("write jpu fifo");
      return -1;
    }
  } else if (input_fmt == kHorizonVisionPixelFormatH264 ||
      input_fmt == kHorizonVisionPixelFormatH265) {
    if (input_decode_id < 0 || input_decode_id >= VPU_FIFO_NUM) {
      LOGE << "input decode id is invalid, id: " << input_decode_id;
      return -1;
    }
    fifo_fd = vpu_fifo_fd_;
    ret = read(fifo_fd, &vpu_chn_id, sizeof(VpuIdStatus));
    if (ret <= 0) {
      perror("read vpu fifo");
      return -1;
    }
    vpu_chn_id.status[input_decode_id] = 255;  // not used
    // write callback data to vpu fifo
    ret = write(fifo_fd, &vpu_chn_id, sizeof(VpuIdStatus));
    if (ret != sizeof(VpuIdStatus)) {
      perror("write vpu fifo");
      return -1;
    }
  } else {
      LOGE << "Unsupport pixel format: " << input_fmt;
      return -1;
  }
  return 0;
}

}  // namespace videosource
