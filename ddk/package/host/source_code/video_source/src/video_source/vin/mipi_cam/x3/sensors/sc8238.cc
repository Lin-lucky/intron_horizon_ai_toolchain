/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include "video_source/vin/mipi_cam/x3/mipi_cam_vin_params.h"

namespace videosource {

MIPI_SENSOR_INFO_S SENSOR_SC8238_30FPS_10BIT_LINEAR_INFO = {
        .deseEnable = 0,
        .inputMode = INPUT_MODE_MIPI,
        .deserialInfo = {},
        .sensorInfo = {
        .port = 0,
        .dev_port = 0,
        .bus_type = 0,
        .bus_num = 2,
        .fps = 30,
        .resolution = 2160,
        .sensor_addr = 0x30,
        .serial_addr = 0,
        .entry_index = 0,
        .sensor_mode = static_cast<MIPI_SENSOR_MODE_E>(1),
        .reg_width = 16,
        .sensor_name = const_cast<char*>("sc8238"),
        .extra_mode = 0,
        .deserial_index = -1,
        .deserial_port = 0,
    }
};

MIPI_SENSOR_INFO_S SENSOR_SC8238_30FPS_10BIT_DOL2_INFO = {
    .deseEnable = 0,
    .inputMode = INPUT_MODE_MIPI,
    .deserialInfo = {},
    .sensorInfo = {
    .port = 0,
    .dev_port = 0,
    .bus_type = 0,
    .bus_num = 2,
    .fps = 30,
    .resolution = 2160,
    .sensor_addr = 0x30,
    .serial_addr = 0,
    .entry_index = 0,
    .sensor_mode = static_cast<MIPI_SENSOR_MODE_E>(2),
    .reg_width = 16,
    .sensor_name = const_cast<char*>("sc8238"),
    .extra_mode = 0,
    .deserial_index = -1,
    .deserial_port = 0
    }
};

MIPI_ATTR_S MIPI_SENSOR_SC8238_30FPS_10BIT_LINEAR_ATTR = {
    .mipi_host_cfg = {
            4,           /* lane */
            0x2b,        /* datatype */
            24,          /* mclk */
            2533,        /* mipiclk */
            30,          /* fps */
            3840,        /* width */
            2160,        /*height */
            3872,        /* linlength */
            2180,        /* framelength */
            30,          /* settle */
            4,           /*chnnal_num*/
            {0, 1, 2, 3} /*vc */
    },
    .dev_enable = 0 /*  mipi dev enable */
};

MIPI_ATTR_S MIPI_SENSOR_SC8238_30FPS_10BIT_LINEAR_SENSOR_CLK_ATTR = {
     .mipi_host_cfg = {
             4,           /* lane */
             0x2b,        /* datatype */
             2400,        /* mclk */
             2533,        /* mipiclk */
             30,          /* fps */
             3840,        /* width  */
             2160,        /*height */
             3872,        /* linlength */
             2180,        /* framelength */
             30,          /* settle */
             4,           /*chnnal_num*/
             {0, 1, 2, 3} /*vc */
     },
   .dev_enable = 0 /*  mipi dev enable */
};

MIPI_ATTR_S MIPI_SENSOR_SC8238_30FPS_10BIT_DOL2_ATTR = {
    .mipi_host_cfg = {
            4,           /* lane */
            0x2b,        /* datatype */
            24,          /* mclk */
            2808,        /* mipiclk */
            30,          /* fps */
            3840,        /* width */
            2160,        /*height */
            4160,        /* linlength */
            2250,        /* framelength */
            30,          /* settle */
            4,           /*chnnal_num*/
            {0, 1, 2, 3} /*vc */
      },
    .dev_enable = 0 /*  mipi dev enable */
};

MIPI_ATTR_S MIPI_SENSOR_SC8238_30FPS_10BIT_DOL2_SENSOR_CLK_ATTR = {
        .mipi_host_cfg = {
                4,        /* lane */
                0x2b,       /* datatype */
                2400,       /* mclk    */
                2808,      /* mipiclk */
                30,       /* fps */
                3840,       /* width  */
                2160,       /*height */
                4160,       /* linlength */
                2250,       /* framelength */
                30,       /* settle */
                4,       /*chnnal_num*/
                {0, 1, 2, 3}     /*vc */
        },
        .dev_enable = 0  /*  mipi dev enable */
};

VIN_DEV_ATTR_S DEV_ATTR_SC8238_LINEAR_BASE = {
        .stSize = {
                0,    /*format*/
                3840, /*width*/
                2160, /*height*/
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
                                .width = 3840,
                                .height = 2160,
                                .pix_length = 1,
                        }},
        .outDdrAttr =
                {
                        .stride = 5008,
                        .buffer_num = 6,
                },
        .outIspAttr = {
                .dol_exp_num = 1,
                .enable_dgain = 0,
                .set_dgain_short = 0,
                .set_dgain_medium = 0,
                .set_dgain_long = 0,
        }
};

VIN_DEV_ATTR_S DEV_ATTR_SC8238_DOL2_BASE = {
    .stSize =
        {
            0,    /*format*/
            3840, /*width*/
            2160, /*height*/
            1     /*pix_length*/
        },
    {
    .mipiAttr =
        {
            .enable = 1,
            .ipi_channels = 2,
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
                           .width = 3840,
                           .height = 2160,
                           .pix_length = 1,
                       }},
    .outDdrAttr =
        {
            .stride = 4800,
            .buffer_num = 10,
        },
    .outIspAttr = {
        .dol_exp_num = 2,
        .enable_dgain = 0,
        .set_dgain_short = 0,
        .set_dgain_medium = 0,
        .set_dgain_long = 0,
    }
};

VIN_PIPE_ATTR_S PIPE_ATTR_SC8238_LINEAR_BASE = {
    .ddrOutBufNum = 8,
    .frameDepth = 0,
    .snsMode = SENSOR_NORMAL_MODE,
    .stSize = {
        .format = 0,
        .width = 3840,
        .height = 2160,
    },
    .cfaPattern = static_cast<VIN_PIPE_CFA_PATTERN_E>(3),
    .temperMode = 3,
    .ispBypassEn = 0,
    .ispAlgoState = 1,
    .bitwidth = 10,
    .startX = 0,
    .startY = 0,
    .calib = {
        .mode = 1,
        .lname = const_cast<char*>("/etc/cam/sc8238_linear.so"),
    }
};

VIN_PIPE_ATTR_S PIPE_ATTR_SC8238_DOL2_BASE = {
    .ddrOutBufNum = 8,
    .frameDepth = 0,
    .snsMode = SENSOR_DOL2_MODE,
    .stSize = {
        .format = 0,
        .width = 3840,
        .height = 2160,
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
        .lname = const_cast<char*>("/etc/cam/sc8238_linear.so"),
    }
};

VIN_DIS_ATTR_S DIS_ATTR_SC8238_BASE = {
  .picSize = {
    .pic_w = 3839,
    .pic_h = 2159,
  },
  .disPath = {
    .rg_dis_enable = 0,
    .rg_dis_path_sel = 1,
  },
  .disHratio = 65536,
  .disVratio = 65536,
  .xCrop = {
    .rg_dis_start = 0,
    .rg_dis_end = 3839,
  },
  .yCrop = {
    .rg_dis_start = 0,
    .rg_dis_end = 2159,
  },
  .disBufNum = 5,
};

VIN_LDC_ATTR_S LDC_ATTR_SC8238_BASE = {
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
        .pic_w = 3839,
        .pic_h = 2159,
    },
    .lineBuf = 99,
    .xParam = {
        .rg_algo_param_b = 3,
        .rg_algo_param_a = 2,
    },
    .yParam = {
        .rg_algo_param_b = 5,
        .rg_algo_param_a = 4,
    },
    .offShift = {
        .rg_center_xoff = 0,
        .rg_center_yoff = 0,
    },
    .xWoi = {
        .rg_start = 0,
        .reserved1 = 0,
        .rg_length = 3839,
        .reserved0 = 0
    },
    .yWoi = {
        .rg_start = 0,
        .reserved1 = 0,
        .rg_length = 2159,
        .reserved0 = 0
    }
};

}  // namespace videosource
