/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include <iostream>
#include "sensor_plugin.h"

MIPI_SENSOR_INFO_S SENSOR_F37_30FPS_10BIT_LINEAR_INFO = {
    .deseEnable = 0,
    .inputMode = INPUT_MODE_MIPI,
    .deserialInfo = {},
    .sensorInfo = {
        .port = 0,
        .dev_port = 0,
        .bus_type = 0,
        .bus_num = 2,
        .fps = 30,
        .resolution = 1080,
        .sensor_addr = 0x40,
        .serial_addr = 0,
        .entry_index = 0,
        .sensor_mode = static_cast<MIPI_SENSOR_MODE_E>(1),
        .reg_width = 8,
        .sensor_name = const_cast<char*>("f37"),
        .extra_mode = 0,
        .deserial_index = -1,
        .deserial_port = 0,
    }
};

MIPI_ATTR_S MIPI_SENSOR_F37_30FPS_10BIT_LINEAR_ATTR = {
    .mipi_host_cfg = {
        1,           /* lane */
        0x2b,        /* datatype */
        24,          /* mclk */
        864,        /* mipiclk */
        30,          /* fps */
        1920,        /* width */
        1080,        /*height */
        2560,        /* linlength */
        1125,        /* framelength */
        20,          /* settle */
        4,           /*chnnal_num*/
        {0, 1, 2, 3} /*vc */
    },
    .dev_enable = 0 /*  mipi dev enable */
};

MIPI_ATTR_S MIPI_SENSOR_F37_30FPS_10BIT_LINEAR_SENSOR_CLK_ATTR = {
    .mipi_host_cfg = {
        1,           /* lane */
        0x2b,        /* datatype */
        2400,        /* mclk */
        864,        /* mipiclk */
        30,          /* fps */
        1920,        /* width  */
        1080,        /*height */
        2560,        /* linlength */
        1125,        /* framelength */
        20,          /* settle */
        4,           /*chnnal_num*/
        {0, 1, 2, 3} /*vc */
    },
    .dev_enable = 0 /*  mipi dev enable */
};

VIN_DEV_ATTR_S DEV_ATTR_F37_LINEAR_BASE = {
    .stSize = {
        0,    /*format*/
        1920, /*width*/
        1080, /*height*/
        1     /*pix_length*/
    },
    {
    .mipiAttr =
        {
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
    .DdrIspAttr = {.stride = 0,
                   .buf_num = 4,
                   .raw_feedback_en = 0,
                   .data =
                       {
                           .format = 0,
                           .width = 1920,
                           .height = 1080,
                           .pix_length = 1,
                       }},
    .outDdrAttr =
        {
            .stride = 2400,
            .buffer_num = 8,
        },
    .outIspAttr = {
        .dol_exp_num = 1,
        .enable_dgain = 0,
        .set_dgain_short = 0,
        .set_dgain_medium = 0,
        .set_dgain_long = 0,
    }};

VIN_PIPE_ATTR_S PIPE_ATTR_F37_LINEAR_BASE = {
    .ddrOutBufNum = 8,
    .frameDepth = 0,
    .snsMode = SENSOR_NORMAL_MODE,
    .stSize = {
        .format = 0,
        .width = 1920,
        .height = 1080,
    },
    .cfaPattern = static_cast<VIN_PIPE_CFA_PATTERN_E>(3),
    .temperMode = 2,
    .ispBypassEn = 0,
    .ispAlgoState = 1,
    .bitwidth = 10,
    .startX = 0,
    .startY = 0,
    .calib = {
        .mode = 1,
        .lname = const_cast<char*>("/etc/cam/libjxf37_linear.so"),
    }
};

VIN_DIS_ATTR_S DIS_ATTR_F37_BASE = {
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
  .disBufNum = 5,
};

VIN_LDC_ATTR_S LDC_ATTR_F37_BASE = {
  .ldcEnable = 0,
  .ldcPath = {
    .rg_y_only = 0,
    .rg_uv_mode = 0,
    .rg_uv_interpo = 0,
    .reserved1 = 0,
    .rg_h_blank_cyc = 32,
    .reserved0 = 0,
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


static int GetSensorAttr(SnsPlgCtrlInfo *ctrl_info,
    MIPI_SENSOR_INFO_S *pst_sns_attr) {
  int ret = -1;
  IotSensorType_E sensor_type =
    static_cast<IotSensorType_E>(ctrl_info->sensor_type);

  if (sensor_type == kLINEAR_SENSOR_TYPE) {
    std::cout << "f37 sensor plugin type is linear" << std::endl;
    *pst_sns_attr = SENSOR_F37_30FPS_10BIT_LINEAR_INFO;
  } else if (sensor_type == kDOL2_SENSOR_TYPE) {
    std::cout << "sensor type is dol2" << std::endl;
  } else if (sensor_type == kDOL3_SENSOR_TYPE) {
    std::cout << "sensor type is dol3" << std::endl;
  } else {
    std::cout << "not support sensor type: " << sensor_type << std::endl;
    return ret;
  }
  return 0;
}

static int GetMipiAttr(SnsPlgCtrlInfo *ctrl_info,
    MIPI_ATTR_S *pst_mipi_attr) {
  int ret = -1;
  int need_clk = ctrl_info->need_clk;
  IotSensorType_E sensor_type =
    static_cast<IotSensorType_E>(ctrl_info->sensor_type);

  std::cout << "f37 sensor plugin need_clk:" << need_clk << std::endl;
  if (sensor_type == kLINEAR_SENSOR_TYPE) {
    if (need_clk == 1) {
      *pst_mipi_attr = MIPI_SENSOR_F37_30FPS_10BIT_LINEAR_SENSOR_CLK_ATTR;
    } else {
      *pst_mipi_attr = MIPI_SENSOR_F37_30FPS_10BIT_LINEAR_ATTR;
    }
  } else if (sensor_type == kDOL2_SENSOR_TYPE) {
  } else if (sensor_type == kDOL3_SENSOR_TYPE) {
  } else {
    std::cout << "not support sensor type: " << sensor_type << std::endl;
    return ret;
  }
  return 0;
}

static int GetVinDevAttr(SnsPlgCtrlInfo *ctrl_info,
    VIN_DEV_ATTR_S *pst_dev_attr) {
  int ret = -1;
  IotSensorType_E sensor_type =
    static_cast<IotSensorType_E>(ctrl_info->sensor_type);

  if (sensor_type == kLINEAR_SENSOR_TYPE) {
    *pst_dev_attr = DEV_ATTR_F37_LINEAR_BASE;
  } else if (sensor_type == kDOL2_SENSOR_TYPE) {
  } else if (sensor_type == kDOL3_SENSOR_TYPE) {
  } else {
    std::cout << "not support sensor type: " << sensor_type << std::endl;
    return ret;
  }
  return 0;
}

static int GetVinDevAttrEx(SnsPlgCtrlInfo *ctrl_info,
    VIN_DEV_ATTR_EX_S *pst_dev_attr_ex) {
  int ret = -1;
  IotSensorType_E sensor_type =
    static_cast<IotSensorType_E>(ctrl_info->sensor_type);

  if (sensor_type == kLINEAR_SENSOR_TYPE) {
  } else if (sensor_type == kDOL2_SENSOR_TYPE) {
  } else if (sensor_type == kDOL3_SENSOR_TYPE) {
  } else {
    std::cout << "not support sensor type: " << sensor_type << std::endl;
    return ret;
  }
  return 0;
}

static int GetVinPipeAttr(SnsPlgCtrlInfo *ctrl_info,
     VIN_PIPE_ATTR_S *pst_pipe_attr) {
  int ret = -1;
  IotSensorType_E sensor_type =
    static_cast<IotSensorType_E>(ctrl_info->sensor_type);

  if (sensor_type == kLINEAR_SENSOR_TYPE) {
    *pst_pipe_attr = PIPE_ATTR_F37_LINEAR_BASE;
  } else if (sensor_type == kDOL2_SENSOR_TYPE) {
  } else if (sensor_type == kDOL3_SENSOR_TYPE) {
  } else {
    std::cout << "not support sensor type: " << sensor_type << std::endl;
    return ret;
  }
  return 0;
}

static int GetDisAttr(SnsPlgCtrlInfo *ctrl_info,
     VIN_DIS_ATTR_S *pst_dis_attr) {
  int ret = -1;
  IotSensorType_E sensor_type =
    static_cast<IotSensorType_E>(ctrl_info->sensor_type);

  if (sensor_type == kLINEAR_SENSOR_TYPE) {
    *pst_dis_attr = DIS_ATTR_F37_BASE;
  } else if (sensor_type == kDOL2_SENSOR_TYPE) {
  } else if (sensor_type == kDOL3_SENSOR_TYPE) {
  } else {
    std::cout << "not support sensor type: " << sensor_type << std::endl;
    return ret;
  }
  return 0;
}

static int GetLdcAttr(SnsPlgCtrlInfo *ctrl_info,
     VIN_LDC_ATTR_S *pst_ldc_attr) {
  int ret = -1;
  IotSensorType_E sensor_type =
    static_cast<IotSensorType_E>(ctrl_info->sensor_type);

  if (sensor_type == kLINEAR_SENSOR_TYPE) {
    *pst_ldc_attr = LDC_ATTR_F37_BASE;
  } else if (sensor_type == kDOL2_SENSOR_TYPE) {
  } else if (sensor_type == kDOL3_SENSOR_TYPE) {
  } else {
    std::cout << "not support sensor type: " << sensor_type << std::endl;
    return ret;
  }
  return 0;
}

SnsPlgModule f37_plugin = {
  .module = "f37_plugin",
  .get_sns_attr = GetSensorAttr,
  .get_mipi_attr = GetMipiAttr,
  .get_vin_dev_attr = GetVinDevAttr,
  .get_vin_dev_attr_ex = GetVinDevAttrEx,
  .get_vin_pipe_attr = GetVinPipeAttr,
  .get_dis_attr = GetDisAttr,
  .get_ldc_attr = GetLdcAttr,
};
