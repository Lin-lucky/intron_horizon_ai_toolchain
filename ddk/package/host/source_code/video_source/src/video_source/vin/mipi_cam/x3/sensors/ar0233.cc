/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include "video_source/vin/mipi_cam/x3/mipi_cam_vin_params.h"

namespace videosource {

MIPI_SENSOR_INFO_S SENSOR_4LANE_AR0233_30FPS_12BIT_1080P_954_INFO = {
  .deseEnable = 1,
  .inputMode = INPUT_MODE_MIPI,
  .deserialInfo = {
    .bus_type = 0,
    .bus_num = 4,
    .deserial_addr = 0x3d,
    .deserial_name = const_cast<char*>("s954"),
  },
  .sensorInfo = {
    .port = 0,
    .dev_port = 0,
    .bus_type = 0,
    .bus_num = 4,
    .fps = 30,
    .resolution = 1080,
    .sensor_addr = 0x10,
    .serial_addr = 0x18,
    .entry_index = 1,
    .sensor_mode = PWL_M,
    .reg_width = 16,
    .sensor_name = const_cast<char*>("ar0233"),
    .extra_mode = 0,
    .deserial_index = 0,
    .deserial_port = 0}
};

MIPI_SENSOR_INFO_S SENSOR_4LANE_AR0233_30FPS_12BIT_1080P_960_INFO = {
  .deseEnable = 1,
  .inputMode = INPUT_MODE_MIPI,
  .deserialInfo = {
    .bus_type = 0,
    .bus_num = 4,
    .deserial_addr = 0x30,
    .deserial_name = const_cast<char*>("s960")
  },
  .sensorInfo = {
    .port = 0,
    .dev_port = 0,
    .bus_type = 0,
    .bus_num = 4,
    .fps = 30,
    .resolution = 1080,
    .sensor_addr = 0x10,
    .serial_addr = 0x18,
    .entry_index = 1,
    .sensor_mode = PWL_M,
    .reg_width = 16,
    .sensor_name = const_cast<char*>("ar0233"),
    .extra_mode = 0,
    .deserial_index = 0,
    .deserial_port = 0}
};

MIPI_ATTR_S MIPI_4LANE_SENSOR_AR0233_30FPS_12BIT_1080P_954_ATTR = {
  .mipi_host_cfg = {
    4,    /* lane */
    0x2c, /* datatype */
    24,   /* mclk */
    1224, /* mipiclk */
    30,   /* fps */
    1920, /* width */
    1080, /*height */
    2000, /* linlength */
    1700, /* framelength */
    30,   /* settle */
    4,
    {0, 1, 2, 3}
  },
  .dev_enable = 0 /*  mipi dev enable */
};

MIPI_ATTR_S MIPI_4LANE_SENSOR_AR0233_30FPS_12BIT_1080P_960_ATTR = {
  .mipi_host_cfg = {
    4,    /* lane */
    0x2c, /* datatype */
    24,   /* mclk */
    3200, /* mipiclk */
    30,   /* fps */
    1920, /* width */
    1080, /*height */
    2000, /* linlength */
    1111, /* framelength */
    30,   /* settle */
    4,
    {0, 1, 2, 3}
  },
  .dev_enable = 0 /*  mipi dev enable */
};

VIN_DEV_ATTR_S DEV_ATTR_AR0233_1080P_BASE = {
  .stSize = {
    0,    /*format*/
    1920, /*width*/
    1080, /*height*/
    2     /*pix_length*/
  },
  {
    .mipiAttr = {
      .enable = 1,
      .ipi_channels = 1,
      .ipi_mode = 0,
      .enable_mux_out = 1,
      .enable_frame_id = 1,
      .enable_bypass = 0,
      .enable_line_shift = 0,
      .enable_id_decoder = 0,
      .set_init_frame_id = 1,
      .set_line_shift_count = 0,
      .set_bypass_channels = 1,
    },
  },
  .DdrIspAttr = {
    .stride = 0,
    .buf_num = 4,
    .raw_feedback_en = 0,
    .data = {
      .format = 0,
      .width = 1920,
      .height = 1080,
      .pix_length = 2,
    }
  },
  .outDdrAttr = {
    .stride = 2880,
    .buffer_num = 10,
  },
  .outIspAttr = {
    .dol_exp_num = 1,
    .enable_dgain = 0,
    .set_dgain_short = 0,
    .set_dgain_medium = 0,
    .set_dgain_long = 0,
  }
};

VIN_PIPE_ATTR_S PIPE_ATTR_AR0233_1080P_BASE = {
  .ddrOutBufNum = 8,
  .frameDepth = 0,
  .snsMode = SENSOR_PWL_MODE,
  .stSize = {
    .format = 0,
    .width = 1920,
    .height = 1080,
  },
  .cfaPattern = static_cast<VIN_PIPE_CFA_PATTERN_E>(1),
  .temperMode = 2,
  .ispBypassEn = 0,
  .ispAlgoState = 1,
  .bitwidth = 12,
  .startX = 0,
  .startY = 0,
  .calib = {
    .mode = 1,
    .lname = const_cast<char*>("/etc/cam/lib_ar0233_pwl.so"),
  }
};

}  // namespace videosource
