
int userconfig[] =
{
	0, //memory mode
	1, //out mode
	0x5, //channel_enable bits:0101
	1280, //panel_width
	720, //panel_height
	     //-----------------------
	0, //channel 0
	1280, //display width
	720, //display height
	1280, //buf width
	720, //buf height
	0, //xposition
	0, //yposition
	4, //format
	255, //alpha
	     //-----------------------
	1, //channel 1
	1280, //display width
	720, //display height
	1280, //buf width
	720, //buf height
	0, //xposition
	0, //yposition
	4, //format
	255, //alpha
	     //-----------------------
	2, //channel 2
	1280, //display width
	720, //displayheight
	1280, //buf width
	720, //buf height
	0, //xposition
	0, //yposition
	1, //format
	128, //alpha
	     //-----------------------
	3, //channel 3
	1280, //display width
	720, //displayheight
	1280, //buf width
	720, //buf height
	0, //xposition
	0, //yposition
	1, //format
	255, //alpha
};
int channelconfig[] =
{
	0, //channel 0
	1, //enable
	3, //pri
	1920, //display width
	1080, //displayheight
	1920, //buf width
	1080, //buf height
	0, //xposition
	0, //yposition
	4, //format
	255, //alpha
	0, //keycolor
	0, //alpha_sel
	0, //ov_mode
	1, //alpha_en
	   //-----------------------
	1, //channel 1
	0, //enable
	2, //pri
	1920, //display width
	1080, //displayheight
	1920, //buf width
	1080, //buf height
	0, //xposition
	0, //yposition
	4, //format
	255, //alpha
	0, //keycolor
	0, //alpha_sel
	0, //ov_mode
	1, //alpha_en
	   //-----------------------
	2, //channel 2
	1, //enable
	1, //pri
	1920, //display width
	1080, //displayheight
	1920, //buf width
	1080, //buf height
	0, //xposition
	0, //yposition
	4, //format
	255, //alpha
	0, //keycolor
	0, //alpha_sel
	0, //ov_mode
	1, //alpha_en
	   //-----------------------
	3, //channel 3
	0, //enable
	0, //pri
	1920, //display width
	1080, //displayheight
	1920, //buf width
	1080, //buf height
	0, //xposition
	0, //yposition
	4, //format
	255, //alpha
	0, //keycolor
	0, //alpha_sel
	0, //ov_mode
	1, //alpha_en
};
int output_cfg[] =
{
	0xff7f88, //bgcolor
	0, //output
	1920, //width
	1080, //height
	8, //display_addr_type
	0, //dithering_flag
	0, //dithering_en
	0, //gamma_en
	0, //hue_en
	0, //sat_en
	0, //con_en
	0, //bright_en
	0, //theta_sign
	0, //contrast
	0xdf, //theta_abs
	0x6a, //saturation
	0, //off_contrast
	0x9a, //off_bright
	0, //dbi_refresh_mode
	2, //panel_corlor_type
	0, //interlace_sel
	0, //odd_polarity
	0, //pixel_rate
	0, //ycbcr_out
	0, //uv_sequence
	0, //itu_r656_en
	0, //auto_dbi_refresh_cnt
	0, //auto_dbi_refresh_en
};
int scale_config[] =
{
	0, //scale_en
	1920, //src_width
	1080, //src_height
	1920, //tgt_width
	1080, //tgt_height
	0, //step_x
	0, //step_y
	0, //pos_x
	0, //pos_y
};
int gammma_config[] =
{
	0, //gamma_x1_r
	0, //gamma_x2_r
	0, //gamma_x3_r
	0, //gamma_x4_r
	0, //gamma_x5_r
	0, //gamma_x6_r
	0, //gamma_x7_r
	0, //gamma_x8_r
	0, //gamma_x9_r
	0, //gamma_x10_r
	0, //gamma_x11_r
	0, //gamma_x12_r
	0, //gamma_x13_r
	0, //gamma_x14_r
	0, //gamma_x15_r
	0, //reserve
	0, //gamma_x1_g
	0, //gamma_x2_g
	0, //gamma_x3_g
	0, //gamma_x4_g
	0, //gamma_x5_g
	0, //gamma_x6_g
	0, //gamma_x7_g
	0, //gamma_x8_g
	0, //gamma_x9_g
	0, //gamma_x10_g
	0, //gamma_x11_g
	0, //gamma_x12_g
	0, //gamma_x13_g
	0, //gamma_x14_g
	0, //gamma_x15_g
	0, //reserve
	0, //gamma_x1_b
	0, //gamma_x2_b
	0, //gamma_x3_b
	0, //gamma_x4_b
	0, //gamma_x5_b
	0, //gamma_x6_b
	0, //gamma_x7_b
	0, //gamma_x8_b
	0, //gamma_x9_b
	0, //gamma_x10_b
	0, //gamma_x11_b
	0, //gamma_x12_b
	0, //gamma_x13_b
	0, //gamma_x14_b
	0, //gamma_x15_b
	0, //reserve
	0, //gamma_y1_start_r
	0, //gamma_y1_r
	0, //gamma_y2_r
	0, //gamma_y3_r
	0, //gamma_y4_r
	0, //gamma_y5_r
	0, //gamma_y6_r
	0, //gamma_y7_r
	0, //gamma_y8_r
	0, //gamma_y9_r
	0, //gamma_y10_r
	0, //gamma_y11_r
	0, //gamma_y12_r
	0, //gamma_y13_r
	0, //gamma_y14_r
	0, //gamma_y15_r
	0, //gamma_y1_start_g
	0, //gamma_y1_g
	0, //gamma_y2_g
	0, //gamma_y3_g
	0, //gamma_y4_g
	0, //gamma_y5_g
	0, //gamma_y6_g
	0, //gamma_y7_g
	0, //gamma_y8_g
	0, //gamma_y9_g
	0, //gamma_y10_g
	0, //gamma_y11_g
	0, //gamma_y12_g
	0, //gamma_y13_g
	0, //gamma_y14_g
	0, //gamma_y15_g
	0, //gamma_y1_start_b
	0, //gamma_y1_b
	0, //gamma_y2_b
	0, //gamma_y3_b
	0, //gamma_y4_b
	0, //gamma_y5_b
	0, //gamma_y6_b
	0, //gamma_y7_b
	0, //gamma_y8_b
	0, //gamma_y9_b
	0, //gamma_y10_b
	0, //gamma_y11_b
	0, //gamma_y12_b
	0, //gamma_y13_b
	0, //gamma_y14_b
	0, //gamma_y15_b
	0, //gamma_y16_r
	0, //gamma_y16_b
	0, //gamma_y16_g
};
