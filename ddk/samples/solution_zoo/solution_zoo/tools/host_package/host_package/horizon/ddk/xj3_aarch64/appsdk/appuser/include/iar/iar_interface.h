/* Copyright: Horizon Robotics
 * Owner: rui.guo@horizon.ai
 * Year : 2019/8/10
 */
#ifndef _IAR_INTERFACE_H
#define _IAR_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif                         /* __cplusplus */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <asm/types.h>
#include <pthread.h>
#include <math.h>
#ifdef XJ3_WB
#include "hb_vio_interface.h"
#include "hb_vio_buffer_mgr.h"
#include "hb_utils.h"
#include "dev_ioctl.h"
#endif

/***********************************************************
 ** 		   Macro global var and struction			  **
***********************************************************/
#define IAR_DEV_PATH	"/dev/iar_cdev"
#define IAR_CH1_PATH	"/dev/iar_channel_0"
#define IAR_CH3_PATH	"/dev/iar_channel_2"

#define IAR_CFG_PATH "/etc/iar/iar.json"

#define ENABLE 1
#define DISABLE 0

#define IAR_DEBUG_PRINT(format, args...)  printf("IAR debug: " format, ## args)
#define IAR_ERR_PRINT(format, args...)	printf("IAR err: " format, ## args)

#define BIT(nr)	(1UL << (nr))

#define CLEAR(x)	memset(&(x), 0, sizeof(x))
enum {
	IAR_CHANNEL_1 = 0,
	IAR_CHANNEL_2 = 1,
	IAR_CHANNEL_3 = 2,
	IAR_CHANNEL_4 = 3,
	IAR_CHANNEL_MAX = 4,
};
enum {
	IAR_PRI_1 = 0,
	IAR_PRI_2 = 1,
	IAR_PRI_3 = 2,
	IAR_PRI_4 = 3,
	IAR_PRI_MAX = 4,
};
enum config_type {
	USER_CFG,
	CHANNEL_CFG,
	OVERLAY_CFG,
	SCALE_CFG,
	GAMMA_CFG,
	OUTPUT_CFG,
};

enum hdmi_mode {
	MODE_1080P60 = 5,
	MODE_1080P50,
	MODE_1080P30,
	MODE_1080P25,
	MODE_1080P59_94,
	MODE_1080P29_97,
	MODE_1080I60,
	MODE_1080I50,
	MODE_1080I59_94,
	MODE_720P60,
	MODE_720P50,
	MODE_720P30,
	MODE_720P25,
	MODE_720P59_94,
	MODE_720P29_97,
	MODE_USER, /* User timing. */
	MODE_BUTT
};

typedef struct _iar_mode_t {
	unsigned int memory_mode;
	unsigned int output_mode;
	unsigned int channel_enable;
	unsigned int panel_width;
	unsigned int panel_height;
}iar_mode_t;

typedef struct _layer_config_t {
	unsigned int channel;
	unsigned int dis_width;
	unsigned int dis_height;
	unsigned int src_width;
	unsigned int src_height;
	unsigned int layer_xpos;
	unsigned int layer_ypos;
	unsigned int layer_format;
	unsigned int layer_alpha;
}layer_config_t;

#define MAX_FRAME_BUF_SIZE	(1920*1080*4)

typedef unsigned int	phys_addr_t;
typedef unsigned int	uint32_t;
typedef unsigned char	uint8_t;

typedef struct _frame_info_t {
	void *addr;
	unsigned int size;
}frame_info_t;

// enum mem_mode_e {
// 	NORMAL_MODE,
// 	DMA_MODE,
// };

enum format_yuv_e {
	FORMAT_YUV422_UYVY	  = 0,
	FORMAT_YUV422_VYUY	  = 1,
	FORMAT_YUV422_YVYU	  = 2,
	FORMAT_YUV422_YUYV	  = 3,
	FORMAT_YUV422SP_UV	  = 4,
	FORMAT_YUV422SP_VU	  = 5,
	FORMAT_YUV420SP_UV	  = 6,
	FORMAT_YUV420SP_VU	  = 7,
	FORMAT_YUV422P_UV	  = 8,
	FORMAT_YUV422P_VU	  = 9,
	FORMAT_YUV420P_UV	  = 10,
	FORMAT_YUV420P_VU	  = 11,
};

enum format_rgb_e {
	FORMAT_8BPP 		  = 0,
	FORMAT_RGB565		  = 1,
	FORMAT_RGB888		  = 2,
	FORMAT_RGB888P		  = 3,
	FORMAT_ARGB8888 	  = 4,
	FORMAT_RGBA8888 	  = 5,
};

enum output_channel_e {
	OUTPUT_CHANNEL0 	  = 1,
	OUTPUT_CHANNEL1		  = 2,
	OUTPUT_CHANNEL2		  = 4,
	OUTPUT_CHANNEL3		  = 8,
};

enum output_mode_e {
	OUTPUT_MIPI 		  = 0,
	OUTPUT_BT1120		  = 1,
	OUTPUT_RGB888		  = 2,
};

struct disp_timing {
        uint32_t hbp;
        uint32_t hfp;
        uint32_t hs;
        uint32_t vbp;
        uint32_t vfp;
        uint32_t vs;
        uint32_t vfp_cnt;
};

typedef struct _channel_base_cfg_t {
	uint32_t	channel;
	uint32_t	enable;
	uint32_t	pri;
	uint32_t	width;
	uint32_t	height;
	uint32_t	buf_width; //default set buf width equal to display width
	uint32_t	buf_height;
	uint32_t	xposition;
	uint32_t	yposition;
	uint32_t	format;
	uint32_t	alpha;
	uint32_t	keycolor;
	uint32_t	alpha_sel;
	uint32_t	ov_mode;
	uint32_t	alpha_en;
	uint32_t	crop_width;
	uint32_t	crop_height;
} channel_base_cfg_t;

typedef struct _ppcon1_cfg_t {
	uint32_t	dithering_flag;
	uint32_t	dithering_en;
	uint32_t	gamma_en;
	uint32_t	hue_en;
	uint32_t	sat_en;
	uint32_t	con_en;
	uint32_t	bright_en;
	uint32_t	theta_sign;
	uint32_t	contrast;
} ppcon1_cfg_t;

typedef struct _ppcon2_cfg_t {
	uint32_t	theta_abs; //ppcon2
	uint32_t	saturation;
	uint32_t	off_contrast;
	uint32_t	off_bright;
	float		gamma_value;
} ppcon2_cfg_t;

typedef struct _refresh_cfg_t {
	uint32_t	dbi_refresh_mode; //refresh mode
	uint32_t	panel_corlor_type;
	uint32_t	interlace_sel;
	uint32_t	odd_polarity;
	uint32_t	pixel_rate;
	uint32_t	ycbcr_out;
	uint32_t	uv_sequence;
	uint32_t	itu_r656_en;

	uint32_t	auto_dbi_refresh_cnt;
	uint32_t	auto_dbi_refresh_en;
} refresh_cfg_t;

typedef struct _output_cfg_t {
	uint32_t	bgcolor;
	uint32_t	out_sel;
	uint32_t	width;
	uint32_t	height;
	uint32_t        big_endian;
	uint32_t	display_addr_type;
	uint32_t	display_cam_no;
	uint32_t        display_addr_type_layer1;
	uint32_t        display_cam_no_layer1;
	ppcon1_cfg_t	ppcon1;
	ppcon2_cfg_t	ppcon2;
	refresh_cfg_t	refresh_cfg;
	uint32_t	panel_type;
	uint32_t	rotate;
	uint32_t	user_control_disp;
	uint32_t	user_control_disp_layer1;
} output_cfg_t;

typedef struct _upscaling_cfg_t {
	uint32_t	enable;
	uint32_t	src_width;
	uint32_t	src_height;
	uint32_t	tgt_width;
	uint32_t	tgt_height;
	uint32_t	step_x;
	uint32_t	step_y;
	uint32_t	pos_x;
	uint32_t	pos_y;
} upscaling_cfg_t;

struct position_cfg_t {
	uint32_t	layer;
	uint32_t	x;
	uint32_t	y;
};

#ifdef XJ3_WB
typedef struct _wb_cfg_t {
	uint32_t src_sel;
	uint32_t format;
} wb_cfg_t;
#endif


typedef struct _frame_buf_t {
	void *vaddr;
	phys_addr_t paddr;
} frame_buf_t;

struct gamma_reg_bits_s {
	unsigned int part_a:8;
	unsigned int part_b:8;
	unsigned int part_c:8;
	unsigned int part_d:8;
};
typedef union _gamma_para_t
{
	unsigned int	value;
	struct gamma_reg_bits_s bit;
}gama_para_t;

typedef struct _gamma_cfg_t {
	gama_para_t    gamma_xr[4];
	gama_para_t    gamma_xg[4];
	gama_para_t    gamma_xb[4];
	gama_para_t    gamma_yr[4];
	gama_para_t    gamma_yg[4];
	gama_para_t    gamma_yb[4];
	gama_para_t    gamma_y16rgb;
} gamma_cfg_t;

typedef struct _update_cmd_t
{
	unsigned int enable_flag[IAR_CHANNEL_MAX];
	unsigned int frame_size[IAR_CHANNEL_MAX];
	frame_buf_t srcframe[IAR_CHANNEL_MAX];
}update_cmd_t;

struct display_video_vaddr {
        void *channel0_y_addr;
        void *channel0_c_addr;
        void *channel1_y_addr;
        void *channel1_c_addr;
};

struct display_vio_channel_pipe {
        uint8_t disp_layer_no;
        uint8_t vio_pipeline;
        uint8_t vio_channel;
};

struct mipi_dsi_lcp {
	uint32_t cmd_len;
	uint8_t *cmd_data;
};

struct mipi_dsi_s1p {
	uint8_t cmd;
	uint8_t data;
};

struct video_timing {
        uint32_t vid_pkt_size;
        uint32_t vid_num_chunks;
        uint32_t vid_null_size;
        uint32_t vid_hsa;
        uint32_t vid_hbp;
        uint32_t vid_hline_time;
        uint32_t vid_vsa;
        uint32_t vid_vbp;
        uint32_t vid_vfp;
        uint32_t vid_vactive_line;
};

struct mipi_dsi_core_init_data {
	uint32_t width;
	uint32_t height;
	struct video_timing timing;
};

typedef struct iar_handle_s {
	int iardev_fd;
	int lt9211_fd;
	int channel_fd[IAR_CHANNEL_MAX];
	update_cmd_t update_cmd;
	int memory_mode;
	// unsigned int frame_size[IAR_CHANNEL_MAX];
	channel_base_cfg_t channel_base_cfg[IAR_CHANNEL_MAX];
	gamma_cfg_t gamma_cfg;
	output_cfg_t output_cfg;
	upscaling_cfg_t scale_cfg;
	char *framebuf[IAR_CHANNEL_MAX];
#ifdef XJ3_WB
	buffer_mgr_t *iar_output_bufmgr[IAR_CHANNEL_MAX];
	buffer_mgr_t *iar_wb_bufmgr;
	wb_cfg_t wb_cfg;
#endif
}iar_handle_s;

int hb_disp_init_cfg(const char *cfg_file);
int hb_disp_init(void);
int hb_disp_close(void);
int hb_disp_start(void);
int hb_disp_stop(void);
int hb_disp_set_lcd_backlight(unsigned int backlight_level);
int hb_disp_pause_video_disp(int pause);
int hb_disp_layer_on(unsigned int layer_number);
int hb_disp_layer_off(unsigned int layer_number);
int hb_disp_video_set_channel(unsigned int channel_number);
int hb_set_video_bufaddr(void *addr0_y, void *addr0_c,
			void *addr1_y, void *addr1_c);
int hb_set_layer_cfg(unsigned int layer_no,
		unsigned int width, unsigned int height,
		unsigned int x_pos, unsigned int y_pos);
int hb_disp_set_video_display_ddr_layer(unsigned int ddr_layer_no);
int hb_disp_power_on(void);
int hb_disp_power_off(void);
int hb_disp_set_timing(struct disp_timing *user_timing);

float hb_disp_get_gamma_cfg();
int hb_disp_set_gamma_cfg(float gamma);
int hb_disp_get_output_cfg(output_cfg_t *cfg);
int hb_disp_set_output_cfg(output_cfg_t *cfg);
int hb_disp_get_upscaling_cfg(upscaling_cfg_t *cfg);
int hb_disp_set_upscaling_cfg(upscaling_cfg_t *cfg);
int hb_disp_get_channel_cfg(uint32_t chn, channel_base_cfg_t *cfg);
int hb_disp_set_channel_cfg(uint32_t chn, channel_base_cfg_t *cfg);
int hb_disp_out_upscale(uint32_t src_w, uint32_t src_h,
		uint32_t tag_w, uint32_t tag_h);

int hb_get_disp_done();
int hb_disp_set_vio_channel(char disp_layer, char pipeline, char channel_no);
int hb_check_video_bufaddr_valid(size_t graphic_size, uint32_t disp_layer_no);

#ifdef XJ3_WB
int hb_disp_wb_start(int srcsel, int format);
int hb_disp_wb_stop();
int hb_disp_release_screen_frame(hb_vio_buffer_t * iar_buffer);
int hb_disp_get_screen_frame(hb_vio_buffer_t * iar_buffer);
int hb_disp_wb_setcfg(int sel, int format);

int hb_disp_output_start(int layer_no);
int hb_disp_output_start_withoutbufmgr(int layer_no);
int hb_disp_output_stop_withoutbufmgr(int layer_no);
int hb_disp_send_frame(int layer_no, hb_vio_buffer_t *iar_buffer);
int hb_disp_get_frame(int layer_no, hb_vio_buffer_t *iar_buffer);
int iar_output_node_qbuf(int fd, int layer_no, hb_vio_buffer_t * buf);
buf_node_t *iar_output_dqbuf_from_Reprocess(int fd,
		int layer_no, buffer_mgr_t * buf_mgr);
int iar_wb_node_qbuf(int fd, hb_vio_buffer_t * buf);
buf_node_t *iar_wb_dqbuf(int fd, buffer_mgr_t * buf_mgr);
#endif
iar_handle_s* hb_disp_get_handle();
int hb_disp_layer_init(int layer);
int hb_disp_shut_down_hdmi(void);
int hb_disp_start_hdmi(uint32_t vmode);
int hb_disp_set_interlace_mode(void);
int hb_disp_set_pixel_clk(unsigned int pixel_clk);
int hb_iar_module_start(void);
int hb_disp_set_display_position(uint32_t layer, uint32_t x, uint32_t y);
int hb_disp_set_crop_position(uint32_t layer, uint32_t x, uint32_t y);
int hb_disp_update_par(void);
int hb_disp_config_hdmi_mode(uint32_t vmode);
int hb_disp_set_mipi_dsi_lcp(uint32_t len, uint8_t *tx_buf);
int hb_disp_set_mipi_dsi_s1p(uint8_t cmd, uint8_t data);
int hb_disp_set_mipi_dsi_snp(uint8_t data);
int hb_disp_init_mipi_dsi_core(struct mipi_dsi_core_init_data *init_data);
int hb_disp_mipi_panel_init_begin(void);
int hb_disp_mipi_panel_init_end(void);
#ifdef __cplusplus
}
#endif
#endif
