/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include "video_source/vin/mipi_cam/x3/mipi_cam_vin_params.h"

namespace videosource {

MIPI_SENSOR_INFO_S SENSOR_4LANE_IMX327_30FPS_12BIT_LINEAR_INFO = {
  .deseEnable = 0,
  .inputMode = INPUT_MODE_MIPI,
  .deserialInfo = {},
  .sensorInfo = {
    .port = 0,
    .dev_port = 0,
    .bus_type = 0,
    .bus_num = 0,
    .fps = 30,
    .resolution = 1097,
    .sensor_addr = 0x36,
    .serial_addr = 0,
    .entry_index = 1,
    .sensor_mode = static_cast<MIPI_SENSOR_MODE_E>(1),
    .reg_width = 16,
    .sensor_name = const_cast<char*>("imx327"),
    .extra_mode = 0,
    .deserial_index = -1,
    .deserial_port = 0
  }
};

MIPI_SENSOR_INFO_S SENSOR_4LANE_IMX327_30FPS_12BIT_DOL2_INFO = {
  .deseEnable = 0,
  .inputMode = INPUT_MODE_MIPI,
  .deserialInfo = {},
  .sensorInfo = {
    .port = 0,
    .dev_port = 0,
    .bus_type = 0,
    .bus_num = 0,
    .fps = 30,
    .resolution = 2228,
    .sensor_addr = 0x36,
    .serial_addr = 0,
    .entry_index = 0,
    .sensor_mode = static_cast<MIPI_SENSOR_MODE_E>(2),
    .reg_width = 16,
    .sensor_name = const_cast<char*>("imx327"),
    .extra_mode = 0,
    .deserial_index = -1,
    .deserial_port = 0
  }
};

MIPI_ATTR_S MIPI_4LANE_SENSOR_IMX327_30FPS_12BIT_NORMAL_SENSOR_CLK_ATTR = {
  .mipi_host_cfg =
  {
    4,    /* lane */
    0x2c, /* datatype */
    3713, /* mclk */
    891,  /* mipiclk */
    30,   /* fps */
    1952, /* width */
    1097, /*height */
    2152, /* linlength */
    1150, /* framelength */
    20    /* settle */
  },
  .dev_enable = 0 /*  mipi dev enable */
};

MIPI_ATTR_S MIPI_4LANE_SENSOR_IMX327_30FPS_12BIT_DOL2_ATTR = {
  .mipi_host_cfg =
  {
    4,    /* lane */
    0x2c, /* datatype */
    24,   /* mclk    */
    1782, /* mipiclk */
    30,   /* fps */
    1952, /* width  */
    2228, /*height */
    2152, /* linlength */
    2300, /* framelength */
    20    /* settle */
  },
  .dev_enable = 0 /* mipi dev enable */
};

MIPI_ATTR_S MIPI_4LANE_SENSOR_IMX327_30FPS_12BIT_NORMAL_ATTR = {
  .mipi_host_cfg = {
    4,    /* lane */
    0x2c, /* datatype */
    24,   /* mclk */
    891,  /* mipiclk */
    30,   /* fps */
    1952, /* width */
    1097, /*height */
    2152, /* linlength */
    1150, /* framelength */
    20    /* settle */
  },
  .dev_enable = 0 /*  mipi dev enable */
};

VIN_DEV_ATTR_S DEV_ATTR_IMX327_LINEAR_BASE = {
  .stSize = {
    0,    /*format*/
    1952, /*width*/
    1097, /*height*/
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
      .width = 1952,
      .height = 1097,
      .pix_length = 2,
    }
  },
  .outDdrAttr = {
    .stride = 2928,
    .buffer_num = 10,
  },
  .outIspAttr = {
    .dol_exp_num = 1,
    .enable_dgain = 0,
    .set_dgain_short = 0,
    .set_dgain_medium = 0,
    .set_dgain_long = 0,
    .short_maxexp_lines = 0,
    .medium_maxexp_lines = 0,
    .vc_short_seq = 0,
    .vc_medium_seq = 0,
    .vc_long_seq = 0,
  }
};

VIN_DEV_ATTR_S DEV_ATTR_IMX327_DOL2_BASE = {
  .stSize = {
    0,    /*format*/
    1948, /*width*/
    1109, /*height*/
    2     /*pix_length*/
  },
  {
    .mipiAttr = {
      .enable = 1,
      .ipi_channels = 2,
      .ipi_mode = 0,
      .enable_mux_out = 1,
      .enable_frame_id = 1,
      .enable_bypass = 0,
      .enable_line_shift = 0,
      .enable_id_decoder = 1,
      .set_init_frame_id = 0,
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
      .width = 1952,
      .height = 1097,
      .pix_length = 2,
    }
  },
  .outDdrAttr = {
    .stride = 2928,
    .buffer_num = 10,
  },
  .outIspAttr = {
    .dol_exp_num = 2,
    .enable_dgain = 0,
    .set_dgain_short = 0,
    .set_dgain_medium = 0,
    .set_dgain_long = 0,
    .short_maxexp_lines = 0,
    .medium_maxexp_lines = 0,
    .vc_short_seq = 0,
    .vc_medium_seq = 0,
    .vc_long_seq = 1,
  },
};

VIN_DEV_ATTR_EX_S DEV_ATTR_IMX327_MD_BASE = {
  .path_sel = 0,
  .roi_top = 0,
  .roi_left = 0,
  .roi_width = 1280,
  .roi_height = 640,
  .grid_step = 128,
  .grid_tolerance = 10,
  .threshold = 10,
  .weight_decay = 128,
  .precision = 0
};

VIN_PIPE_ATTR_S PIPE_ATTR_IMX327_LINEAR_BASE = {
  .ddrOutBufNum = 8,
  .frameDepth = 0,
  .snsMode = SENSOR_NORMAL_MODE,
  .stSize = {
    .format = 0,
    .width = 1920,
    .height = 1080,
  },
  .cfaPattern = PIPE_BAYER_RGGB,
  .temperMode = 2,
  .ispBypassEn = 0,
  .ispAlgoState = 1,
  .bitwidth = 12,
  .startX = 0,
  .startY = 12,
  .calib = {
    .mode = 1,
    .lname = const_cast<char*>("/etc/cam/libimx327_linear.so"),
  }
};

VIN_PIPE_ATTR_S PIPE_ATTR_IMX327_DOL2_BASE = {
  .ddrOutBufNum = 5,
  .frameDepth = 0,
  .snsMode = SENSOR_DOL2_MODE,
  .stSize = {
    .format = 0,
    .width = 1920,
    .height = 1080,
  },
  .cfaPattern = PIPE_BAYER_RGGB,
  .temperMode = 2,
  .ispBypassEn = 0,
  .ispAlgoState = 1,
  .bitwidth = 12,
  .startX = 0,
  .startY = 12,
  .calib = {
    .mode = 1,
    .lname = const_cast<char*>("/etc/cam/libimx327_linear.so"),
  }
};

}  // namespace videosource
