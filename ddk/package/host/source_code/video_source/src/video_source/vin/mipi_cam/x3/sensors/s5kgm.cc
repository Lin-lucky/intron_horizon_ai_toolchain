/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include "video_source/vin/mipi_cam/x3/mipi_cam_vin_params.h"

namespace videosource {

MIPI_SENSOR_INFO_S SENSOR_S5KGM1SP_30FPS_10BIT_LINEAR_INFO = {
  .deseEnable = 0,
  .inputMode = INPUT_MODE_MIPI,
  .deserialInfo = {},
  .sensorInfo = {.port = 0,
    .dev_port = 0,
    .bus_type = 0,
    .bus_num = 4,
    .fps = 30,
    .resolution = 3000,
    .sensor_addr = 0x10,
    .serial_addr = 0,
    .entry_index = 1,
    .sensor_mode = NORMAL_M ,
    .reg_width = 16,
    .sensor_name = const_cast<char*>("s5kgm1sp"),
    .extra_mode = 0,
    .deserial_index = -1,
    .deserial_port = 0
  }
};

MIPI_ATTR_S MIPI_SENSOR_S5KGM1SP_30FPS_10BIT_LINEAR_ATTR = {
  .mipi_host_cfg = {
    4,            /* lane */
    0x2b,         /* datatype */
    24,           /* mclk */
    4600,         /* mipiclk */
    30,           /* fps */
    4000,         /* width */
    3000,         /* height */
    5024,         /* linlength */
    3194,         /* framelength */
    30,           /* settle */
    2,            /*chnnal_num*/
    {0, 1}        /*vc */
  },
  .dev_enable = 0  /*  mipi dev enable */
};

VIN_DEV_ATTR_S DEV_ATTR_S5KGM1SP_LINEAR_BASE = {
  .stSize = {
    0,       /*format*/
    4000,    /*width*/
    3000,    /*height*/
    1        /*pix_length*/
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
      .width = 4000,
      .height = 3000,
      .pix_length = 1,
    }
  },
  .outDdrAttr = {
    .stride = 5000,
    .buffer_num = 8,
  },
  .outIspAttr = {
    .dol_exp_num = 1,
    .enable_dgain = 0,
    .set_dgain_short = 0,
    .set_dgain_medium = 0,
    .set_dgain_long = 0,
  }
};


VIN_PIPE_ATTR_S PIPE_ATTR_S5KGM1SP_LINEAR_BASE = {
  .ddrOutBufNum = 8,
  .frameDepth = 0,
  .snsMode = SENSOR_NORMAL_MODE,
  .stSize = {
    .format = 0,
    .width = 4000,
    .height = 3000,
  },
  .cfaPattern = PIPE_BAYER_GRBG,
  .temperMode = 2,
  .ispBypassEn = 0,
  .ispAlgoState = 1,
  .bitwidth = 10,
  .startX = 0,
  .startY = 0,
  .calib = {
    .mode = 1,
    .lname = const_cast<char*>("/etc/cam/s5kgm1sp_linear.so"),
  }
};

}  // namespace videosource
