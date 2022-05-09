/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include "video_source/vin/mipi_cam/x3/mipi_cam_vin_params.h"

namespace videosource {

MIPI_SENSOR_INFO_S SENSOR_TESTPATTERN_INFO = {
  .deseEnable = {},
  .inputMode = {},
  .deserialInfo = {},
  .sensorInfo = {
    .port = {},
    .dev_port = {},
    .bus_type = {},
    .bus_num = {},
    .fps = {},
    .resolution = {},
    .sensor_addr = {},
    .serial_addr = {},
    .entry_index = {},
    .sensor_mode = {},
    .reg_width = {},
    .sensor_name = const_cast<char*>("virtual"),
  }
};

VIN_DEV_ATTR_S DEV_ATTR_FEED_BACK_1097P_BASE = {
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
    .raw_feedback_en = 1,
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
  }
};

VIN_DEV_ATTR_S DEV_ATTR_TEST_PATTERN_YUV422_BASE = {
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
      .enable_pattern = 1,
    },
  },
  .DdrIspAttr = {
    .stride = 0,
    .buf_num = 8,
    .raw_feedback_en = 0,
    .data = {
      .format = 8,
      .width = 1280,
      .height = 720,
      .pix_length = 2,
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

VIN_DEV_ATTR_S DEV_ATTR_TEST_PATTERN_1080P_BASE = {
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
      .enable_pattern = 1,
    },
  },
  .DdrIspAttr = {
    .stride = 0,
    .buf_num = 6,
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

VIN_DEV_ATTR_S DEV_ATTR_TEST_PATTERN_4K_BASE = {
  .stSize = {
    0,    /*format*/
    3840, /*width*/
    2160, /*height*/
    1     /*pix_length*/
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
      .enable_pattern = 1,
    },
  },
  .DdrIspAttr = {
    .stride = 0,
    .buf_num = 8,
    .raw_feedback_en = 0,
    .data = {
      .format = 0,
      .width = 3840,
      .height = 2160,
      .pix_length = 1,
    }
  },
  .outDdrAttr = {
    .stride = 4800,
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

VIN_DEV_ATTR_S DEV_ATTR_TEST_PATTERN_12M_BASE = {
  .stSize = {
    0,    /*format*/
    4000, /*width*/
    3000, /*height*/
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
      .enable_pattern = 1,
    },
  },
  .DdrIspAttr = {
    .stride = 0,
    .buf_num = 8,
    .raw_feedback_en = 0,
    .data = {
      .format = 0,
      .width = 4000,
      .height = 3000,
      .pix_length = 2,
    }
  },
  .outDdrAttr = {
    .stride = 6000,
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

VIN_PIPE_ATTR_S PIPE_ATTR_TEST_PATTERN_1080P_BASE = {
  .ddrOutBufNum = 4,
  .frameDepth = 0,
  .snsMode = SENSOR_NORMAL_MODE,
  .stSize = {
    .format = 0,
    .width = 1920,
    .height = 1080,
  },
  .cfaPattern = PIPE_BAYER_RGGB,
  .temperMode = 0,
  .ispBypassEn = 0,
  .ispAlgoState = 1,
  .bitwidth = 12,
  .startX = 0,
  .startY = 0,
  .calib = {
    .mode = 0,
    .lname = NULL,
  }
};

VIN_PIPE_ATTR_S PIPE_ATTR_TEST_PATTERN_12M_BASE = {
  .ddrOutBufNum = 8,
  .frameDepth = 0,
  .snsMode = SENSOR_NORMAL_MODE,
  .stSize = {
    .format = 0,
    .width = 4000,
    .height = 3000,
  },
  .cfaPattern = PIPE_BAYER_RGGB,
  .temperMode = 0,
  .ispBypassEn = 0,
  .ispAlgoState = 1,
  .bitwidth = 12,
  .startX = 0,
  .startY = 0,
  .calib = {
    .mode = 0,
    .lname = NULL,
  }
};

VIN_PIPE_ATTR_S PIPE_ATTR_TEST_PATTERN_4K_BASE = {
  .ddrOutBufNum = 8,
  .frameDepth = 0,
  .snsMode = SENSOR_NORMAL_MODE,
  .stSize = {
    .format = 0,
    .width = 3840,
    .height = 2160,
  },
  .cfaPattern = PIPE_BAYER_RGGB,
  .temperMode = 3,
  .ispBypassEn = 0,
  .ispAlgoState = 0,
  .bitwidth = 10,
  .startX = 0,
  .startY = 0,
  .calib = {
    .mode = 0,
    .lname = NULL,
  }
};

VIN_DIS_ATTR_S DIS_ATTR_BASE = {
  .picSize = {
    .pic_w = 1919,
    .pic_h = 1079,
  },
  .disPath = {
    .rg_dis_enable = 0,
    .rg_dis_path_sel = 1,
  },
  .disHratio = 65536,
  .disVratio = 65536,
  .xCrop = {
    .rg_dis_start = 0,
    .rg_dis_end = 1919,
  },
  .yCrop = {
    .rg_dis_start = 0,
    .rg_dis_end = 1079,
  },
  .disBufNum = 8,
};

VIN_DIS_ATTR_S DIS_ATTR_12M_BASE = {
  .picSize = {
    .pic_w = 3999,
    .pic_h = 2999,
  },
  .disPath = {
    .rg_dis_enable = 0,
    .rg_dis_path_sel = 1,
  },
  .disHratio = 65536,
  .disVratio = 65536,
  .xCrop = {
    .rg_dis_start = 0,
    .rg_dis_end = 3999,
  },
  .yCrop = {
    .rg_dis_start = 0,
    .rg_dis_end = 2999,
  }
};

VIN_LDC_ATTR_S LDC_ATTR_12M_BASE = {
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
    .pic_w = 3999,
    .pic_h = 2999,
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
    .rg_length = 3999,
    .reserved0 = 0
  },
  .yWoi = {
    .rg_start = 0,
    .reserved1 = 0,
    .rg_length = 2999,
    .reserved0 = 0
  }
};

VIN_LDC_ATTR_S LDC_ATTR_BASE = {
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
    .pic_w = 1919,
    .pic_h = 1079,
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
    .rg_length = 1919,
    .reserved0 = 0
  },
  .yWoi = {
    .rg_start = 0,
    .reserved1 = 0,
    .rg_length = 1079,
    .reserved0 = 0
  }
};

}  // namespace videosource
