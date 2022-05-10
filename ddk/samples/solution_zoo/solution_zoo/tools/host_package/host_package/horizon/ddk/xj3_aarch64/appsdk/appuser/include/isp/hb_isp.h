#ifndef __HB_ISP_H__
#define __HB_ISP_H__

#include <stdint.h>
#include <stdbool.h>
#define CTX_SIZE       (0x18e88 - 0xab6c + 0x4000)

#define HB_ISP_MAX_AWB_ZONES (33 * 33)
#define HB_ISP_FULL_HISTOGRAM_SIZE 1024

typedef enum {
	NORMAL_MODE = 1,
	DOL2_MODE,
	DOL3_MODE,
	DOL4_MODE,
	PWL_MODE,
	INVAILD_MODE,
} sensor_mode_e;

typedef enum {
	BAYER_RGGB = 0,
	BAYER_GRBG,
	BAYER_GBRG,
	BAYER_BGGR,
	MONOCHROME,
} sensor_cfa_pattern_e;

typedef enum {
	AWB_AUTO = 0x35,
	AWB_MANUAL,
	AWB_DAY_LIGHT,
	AWB_CLOUDY,
	AWB_INCANDESCENT,
	AWB_FLOURESCENT,
	AWB_TWILIGHT,
	AWB_SHADE,
	AWB_WARM_FLOURESCENT,
} isp_awb_mode_e;

typedef struct {
	uint16_t rg;
	uint16_t bg;
        uint32_t sum;
} isp_awb_statistics_s;

typedef struct {
	uint32_t frame_id;
	uint64_t timestamp;
	uint16_t crc16;
	void *ptr;
	uint32_t len;
} isp_context_t;

typedef struct{
	bool crc_en;
	void *data;
	uint32_t len;
	uint32_t frame_id;
	uint64_t timestamp;
	uint32_t  buf_idx;
} isp_statistics_t;

extern int hb_isp_iridix_ctrl(uint32_t provider_ctx_id, uint32_t user_ctx_id, uint8_t flag);
extern int hb_isp_get_ae_statistics(uint32_t ctx_id, isp_statistics_t *ae_statistics, int time_out);
extern int hb_isp_get_awb_statistics(uint32_t ctx_id, isp_statistics_t *awb_statistics, int time_out);
extern int hb_isp_release_ae_statistics(uint32_t ctx_id, isp_statistics_t *ae_statistics);
extern int hb_isp_release_awb_statistics(uint32_t ctx_id, isp_statistics_t *awb_statistics);
extern int hb_isp_get_context(uint32_t ctx_id, isp_context_t *ptx);
extern int hb_isp_set_context(uint32_t ctx_id, isp_context_t *ptx);
extern int hb_isp_dev_init(void);
extern int hb_isp_dev_deinit(void);

#endif
