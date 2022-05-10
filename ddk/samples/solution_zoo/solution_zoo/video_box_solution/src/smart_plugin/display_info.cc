/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */

#include "smart_plugin/display_info.h"

#include "hobotlog/hobotlog.hpp"

namespace solution {
namespace video_box {

DisplayInfo::DisplayInfo() {}

DisplayInfo::~DisplayInfo() {}

void computeXYPossition_win4(const int channel_id, int &x_offset, int &y_offset,
                             const bool display_720p) {
  if (display_720p) {
    if (channel_id == 0) {
      x_offset = 0;
      y_offset = 0;
    } else if (channel_id == 1) {
      x_offset = 640;
      y_offset = 0;
    } else if (channel_id == 2) {
      x_offset = 0;
      y_offset = 360;
    } else if (channel_id == 3) {
      x_offset = 640;
      y_offset = 360;
    } else {
      LOGE << "display mode 0 recv channle id:" << channel_id << " error!";
      x_offset = 0;
      y_offset = 0;
    }
    return;
  }

  if (channel_id == 0) {
    x_offset = 0;
    y_offset = 0;
  } else if (channel_id == 1) {
    x_offset = 960;
    y_offset = 0;
  } else if (channel_id == 2) {
    x_offset = 0;
    y_offset = 540;
  } else if (channel_id == 3) {
    x_offset = 960;
    y_offset = 540;
  } else {
    LOGE << "display mode 0 recv channle id:" << channel_id << " error!";
    x_offset = 0;
    y_offset = 0;
  }
}

void computeXYPossition_win5(const int channel_id, int &x_offset, int &y_offset,
                             const bool display_720p) {
  if (channel_id == 0) {
    x_offset = 0;
    y_offset = 0;
  } else if (channel_id == 1) {
    x_offset = 0;
    y_offset = 720;
  } else if (channel_id == 2) {
    x_offset = 480;
    y_offset = 720;
  } else if (channel_id == 3) {
    x_offset = 960;
    y_offset = 720;
  } else if (channel_id == 4) {
    x_offset = 1440;
    y_offset = 720;
  } else {
    LOGE << "display mode 1 recv channle id:" << channel_id << " error!";
    x_offset = 0;
    y_offset = 0;
  }
}

void computeXYPossition_win9(const int channel_id, int &x_offset, int &y_offset,
                             const bool display_720p) {
  if (channel_id == 0) {
    x_offset = 0;
    y_offset = 0;
  } else if (channel_id == 1) {
    x_offset = 640;
    y_offset = 0;
  } else if (channel_id == 2) {
    x_offset = 1280;
    y_offset = 0;
  } else if (channel_id == 3) {
    x_offset = 0;
    y_offset = 360;
  } else if (channel_id == 4) {
    x_offset = 640;
    y_offset = 360;
  } else if (channel_id == 5) {
    x_offset = 1280;
    y_offset = 360;
  } else if (channel_id == 6) {
    x_offset = 0;
    y_offset = 720;
  } else if (channel_id == 7) {
    x_offset = 640;
    y_offset = 720;
  } else if (channel_id == 8) {
    x_offset = 1280;
    y_offset = 720;
  } else {
    LOGE << "display mode 0 recv channle id:" << channel_id << " error!";
    x_offset = 0;
    y_offset = 0;
  }
}

void DisplayInfo::computeXYPossition(const int dispaly_mode,
                                     const int channel_num,
                                     const int channel_id, int &x_offset,
                                     int &y_offset, const bool display_720p) {
  int win_num = 1;
  if (dispaly_mode == 0) {
    if (channel_num > 1 && channel_num <= 4) {
      win_num = 4;
    } else {
      win_num = 9;
    }
  } else if (dispaly_mode == 1) {
    win_num = 9;
  } else if (dispaly_mode == 2) {
    win_num = 5;
  } else {
    LOGE << "unknow display mode:" << dispaly_mode;
    win_num = 4;
  }

  switch (win_num) {
    case 1: {
      x_offset = 0;
      y_offset = 0;
      break;
    }
    case 4:
      computeXYPossition_win4(channel_id, x_offset, y_offset, display_720p);
      break;
    case 5:
      computeXYPossition_win5(channel_id, x_offset, y_offset, display_720p);
      break;
    case 9:
      computeXYPossition_win9(channel_id, x_offset, y_offset, display_720p);
      break;
    default: {
      x_offset = 0;
      y_offset = 0;
      break;
    }
  }
}

void DisplayInfo::computeResolution(const int dispaly_mode,
                                    const int channel_num, const int channel_id,
                                    uint32_t &width, uint32_t &height,
                                    const bool display_720p) {
  if (display_720p) {
    if (dispaly_mode == 0) {
      if (channel_num == 1) {  // 1窗口
        width = 1280;
        height = 720;
      } else if (channel_num > 1 && channel_num <= 4) {  // 4窗口
        width = 640;
        height = 360;
      } else {  // 9窗口, resize
        width = 640;
        height = 360;
      }
    } else if (dispaly_mode == 1) {  // 9窗口, resize
      width = 640;
      height = 360;
    } else {
      LOGE << "unknow display_mode, return 720p resolution!!!";
      width = 1280;
      height = 720;
    }
    return;
  }

  if (dispaly_mode == 0) {
    if (channel_num == 1) {  // 1窗口
      width = 1920;
      height = 1080;
    } else if (channel_num > 1 && channel_num <= 4) {  // 4窗口
      width = 960;
      height = 540;
    } else {  // 9窗口
      width = 640;
      height = 360;
    }
  } else if (dispaly_mode == 1) {  // 9窗口
    width = 640;
    height = 360;
  } else if (dispaly_mode == 2) {  // 5窗口
    if (channel_id == 0) {
      width = 1920;
      height = 720;
    } else {
      width = 480;
      height = 360;
    }
  } else {
    LOGE << "unknow display_mode, return 1080p resolution!!!";
    width = 1920;
    height = 1080;
  }
}

int DisplayInfo::computePymLayer(const int dispaly_mode, const int channel_num,
                                 const int channel_id,
                                 const bool display_720p) {
  int width = 0;
  int height = 0;

  if (display_720p) {
    if (dispaly_mode == 0) {
      if (channel_num == 1) {  // 1窗口
        width = 1280;
        height = 720;
      } else if (channel_num > 1) {  // 4窗口, 9窗口做resize
        width = 640;
        height = 360;
      }
    } else if (dispaly_mode == 1) {  // 9窗口, resize
      width = 640;
      height = 360;
    } else {
      LOGE << "unknow display_mode, return 1080p resolution!!!";
      width = 1280;
      height = 720;
    }
  } else {
    if (dispaly_mode == 0) {
      if (channel_num == 1) {  // 1窗口
        width = 1920;
        height = 1080;
      } else if (channel_num > 1 && channel_num <= 4) {  // 4窗口
        width = 960;
        height = 540;
      } else {  // 9窗口
        width = 640;
        height = 360;
      }
    } else if (dispaly_mode == 1) {  // 9窗口
      width = 640;
      height = 360;
    } else if (dispaly_mode == 2) {  // 5窗口
      if (channel_id == 0) {
        width = 1920;
        height = 1080;
      } else {
        width = 960;
        height = 540;
      }
    } else {
      LOGE << "unknow display_mode, return 1080p resolution!!!";
      width = 1920;
      height = 1080;
    }
  }

  if (width == 960 && height == 540)
    return 4;
  else if (width == 640 && height == 360)
    return 5;
  else if (width == 1280 && height == 720)
    return 1;
  else
    return 0;
}
}  // namespace video_box
}  // namespace solution
