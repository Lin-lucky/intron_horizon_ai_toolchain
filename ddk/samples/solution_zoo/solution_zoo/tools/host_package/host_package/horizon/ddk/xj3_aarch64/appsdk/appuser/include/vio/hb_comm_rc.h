/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @file:
 * @brief:
 * @author:
 * @email:
 * @date: 2021.01.19
 */

#ifndef VIDEO_MJPEG_INC_HB_COMM_RC_H_
#define VIDEO_MJPEG_INC_HB_COMM_RC_H_

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

#include <stdint.h>

#define HB_ERR_VENC_INVALID_PARAM 0xF000001A

typedef struct HB_VENC_MJPEG_QUALITY_VALUE_S {
	uint32_t width;
	uint32_t height;
	uint32_t quality;
	uint32_t size;  // bytes
} VENC_MJPEG_QUALITY_VALUE_S;

uint32_t HB_VENC_GenerateJpegQualityFactor(uint32_t preQF, uint32_t preSize,
		uint32_t minQF, uint32_t maxQF, uint32_t bitRate,
		uint32_t frameRate, uint32_t width, uint32_t height,
		uint32_t *laterQF);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

#endif  // VIDEO_MJPEG_INC_HB_COMM_RC_H_
