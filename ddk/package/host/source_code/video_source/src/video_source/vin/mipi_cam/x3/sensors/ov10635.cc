/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include "video_source/vin/mipi_cam/x3/mipi_cam_vin_params.h"

namespace videosource {

MIPI_SENSOR_INFO_S SENSOR_2LANE_OV10635_30FPS_YUV_720P_954_INFO = {
  .deseEnable = 1,
  .inputMode = INPUT_MODE_MIPI,
  .deserialInfo = {
    .bus_type = 0,
    .bus_num = 4,
    .deserial_addr = 0x3d,
    .deserial_name = const_cast<char*>("s954")
  },
  .sensorInfo = {
    .port = 0,
    .dev_port = 0,
    .bus_type = 0,
    .bus_num = 4,
    .fps = 30,
    .resolution = 720,
    .sensor_addr = 0x40,
    .serial_addr = 0x1c,
    .entry_index = 1,
    .sensor_mode = {},
    .reg_width = 16,
    .sensor_name = const_cast<char*>("ov10635"),
    .extra_mode = 0,
    .deserial_index = 0,
    .deserial_port = 0
  }
};

MIPI_SENSOR_INFO_S SENSOR_2LANE_OV10635_30FPS_YUV_720P_960_INFO = {
  .deseEnable = 1,
  .inputMode = INPUT_MODE_MIPI,
  .deserialInfo = {
    .bus_type = 0,
    .bus_num = 1,
    .deserial_addr = 0x30,
    .deserial_name = const_cast<char*>("s960")
  },
  .sensorInfo = {
    .port = 0,
    .dev_port = 0,
    .bus_type = 0,
    .bus_num = 4,
    .fps = 30,
    .resolution = 720,
    .sensor_addr = 0x40,
    .serial_addr = 0x1c,
    .entry_index = 0,
    .sensor_mode = {},
    .reg_width = 16,
    .sensor_name = const_cast<char*>("ov10635"),
    .extra_mode = 0,
    .deserial_index = 0,
    .deserial_port = 0
  }
};

MIPI_ATTR_S MIPI_2LANE_OV10635_30FPS_YUV_720P_954_ATTR = {
  .mipi_host_cfg = {
    2,    /* lane */
    0x1e, /* datatype */
    24,   /* mclk */
    1600, /* mipiclk */
    30,   /* fps */
    1280, /* width */
    720,  /*height */
    3207, /* linlength */
    748,  /* framelength */
    30,   /* settle */
    4,
    {0, 1, 2, 3}
  },
  .dev_enable = 0 /*  mipi dev enable */
};

MIPI_ATTR_S MIPI_2LANE_OV10635_30FPS_YUV_720P_960_ATTR = {
  .mipi_host_cfg = {
    2,    /* lane */
    0x1e, /* datatype */
    24,   /* mclk */
    3200, /* mipiclk */
    30,   /* fps */
    1280, /* width  */
    720,  /*height */
    3207, /* linlength */
    748,  /* framelength */
    30,   /* settle */
    4,
    {0, 1, 2, 3}
  },
  .dev_enable = 0 /*  mipi dev enable */
};

MIPI_ATTR_S MIPI_2LANE_OV10635_30FPS_YUV_LINE_CONCATE_720P_960_ATTR = {
  .mipi_host_cfg = {
    2,    /* lane */
    0x1e, /* datatype */
    24,   /* mclk */
    3200, /* mipiclk */
    30,   /* fps */
    1280, /* width  */
    720,  /* height */
    3207, /* linlength */
    748,  /* framelength */
    30,   /* settle */
    2,
    {0, 1}
  },
  .dev_enable = 0  /*  mipi dev enable */
};

VIN_DEV_ATTR_S DEV_ATTR_OV10635_YUV_BASE = {
  .stSize = {
    8,    /*format*/
    1280, /*width*/
    720,  /*height*/
    0     /*pix_length*/
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
      .format = 8,
      .width = 1280,
      .height = 720,
      .pix_length = 0,
    }
  },
  .outDdrAttr = {
    .stride = 1280,
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

VIN_DEV_ATTR_S DEV_ATTR_OV10635_YUV_LINE_CONCATE_BASE = {
  .stSize = {
    8,    /* format */
    2560, /* width */
    720,  /* height */
    0     /* pix_length */
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
      .format = 8,
      .width = 2560,
      .height = 720,
      .pix_length = 0,
    }
  },
  .outDdrAttr = {
    .stride = 2560,
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

VIN_DEV_ATTR_EX_S DEV_ATTR_OV10635_MD_BASE = {
  .path_sel = 1,
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

VIN_PIPE_ATTR_S PIPE_ATTR_OV10635_YUV_BASE = {
  .ddrOutBufNum = 8,
  .frameDepth = 0,
  .snsMode = SENSOR_NORMAL_MODE,
  .stSize = {
    .format = 0,
    .width = 1280,
    .height = 720,
  },
  .cfaPattern = PIPE_BAYER_RGGB,
  .temperMode = 0,
  .ispBypassEn = 1,
  .ispAlgoState = 0,
  .bitwidth = 12,
  .startX = 0,
  .startY = 0,
  .calib = {
    .mode = 0,
    .lname = NULL,
  }
};

VIN_PIPE_ATTR_S VIN_ATTR_OV10635_YUV_LINE_CONCATE_BASE = {
  .ddrOutBufNum = 8,
  .frameDepth = 0,
  .snsMode = SENSOR_NORMAL_MODE,
  .stSize = {
    .format = 0,
    .width = 2560,
    .height = 720,
  },
  .cfaPattern = PIPE_BAYER_RGGB,
  .temperMode = 0,
  .ispBypassEn = 1,
  .ispAlgoState = 0,
  .bitwidth = 12,
  .startX = 0,
  .startY = 0,
  .calib = {
    .mode = 0,
    .lname = NULL,
  }
};

VIN_DIS_ATTR_S DIS_ATTR_OV10635_BASE = {
  .picSize = {
    .pic_w = 1279,
    .pic_h = 719,
  },
  .disPath = {
    .rg_dis_enable = 0,
    .rg_dis_path_sel = 1,
  },
  .disHratio = 65536,
  .disVratio = 65536,
  .xCrop = {
    .rg_dis_start = 0,
    .rg_dis_end = 1279,
  },
  .yCrop = {
    .rg_dis_start = 0,
    .rg_dis_end = 719,
  },
  .disBufNum = 5,
};

VIN_LDC_ATTR_S LDC_ATTR_OV10635_BASE = {
  .ldcEnable = 0,
  .ldcPath = {
    .rg_y_only = 0,
    .rg_uv_mode = 0,
    .rg_uv_interpo = 0,
    .reserved1 = 0,
    .rg_h_blank_cyc = 32,
    .reserved0 = 0
  },
  .yStartAddr = 524288,
  .cStartAddr = 786432,
  .picSize = {
    .pic_w = 1279,
    .pic_h = 719,
  },
  .lineBuf = 99,
  .xParam = {
    .rg_algo_param_b = 1,
    .rg_algo_param_a = 1,
  },
  .yParam = {
    .rg_algo_param_b = 1,
    .rg_algo_param_a = 1,
  },
  .offShift = {
    .rg_center_xoff = 0,
    .rg_center_yoff = 0,
  },
  .xWoi = {
    .rg_start = 0,
    .reserved1 = 0,
    .rg_length = 1279,
    .reserved0 = 0
  },
  .yWoi = {
    .rg_start = 0,
    .reserved1 = 0,
    .rg_length = 719,
    .reserved0 = 0
  }
};

}  // namespace videosource
