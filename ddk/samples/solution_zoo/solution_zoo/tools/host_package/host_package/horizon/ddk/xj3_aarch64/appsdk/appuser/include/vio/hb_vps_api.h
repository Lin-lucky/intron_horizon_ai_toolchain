/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef __HB_VPS_API_H
#define __HB_VPS_API_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "hb_errno.h"
#include "hb_sys.h"

#if 0
#define HB_ERR_VPS_INVALID_GROUPID			-1
#define HB_ERR_VPS_BUFMGR					-2
#define HB_ERR_VPS_GROUP_FAIL				-3
#define HB_ERR_VPS_GROUP_UNEXIST			-4
#define HB_ERR_VPS_CHN_UNEXIST				-5
#define HB_ERR_VPS_ROTATE					-6
#define HB_ERR_EALREADY_BIND				-7
#endif

enum HB_VPS_ERR_E {
	ERR_VPS_INVALID_GROUPID = 1,
	ERR_VPS_BUFMGR,
	ERR_VPS_GROUP_FAIL,
	ERR_VPS_GROUP_UNEXIST,
	ERR_VPS_CHN_UNEXIST,
	ERR_VPS_ROTATE,
	ERR_VPS_NULL_PARA,
	ERR_VPS_BAD_ARG,
	ERR_VPS_UN_PREPARED,
	ERR_VPS_SENDFRAME,
	ERR_VPS_CHN_DISABLE,
	ERR_VPS_TIMEOUT,
	ERR_VPS_CHN_FD,
	ERR_VPS_SET_AFTER_START,
	ERR_VPS_SET_BEFORE_START,
	ERR_VPS_SET_AT_WRONG_TIME,
	ERR_VPS_UN_SUPPORT_SIZE,
	ERR_VPS_FRAME_UNEXIST,
	ERR_VPS_DEV_FRAME_DROP,
	ERR_VPS_NOT_ENOUGH,
	ERR_VPS_UN_SUPPORT_RATE,
	ERR_VPS_FRAME_RATE,
	/* pym layer error: */
	ERR_VPS_LAYER_UNEXIST = 64,
	ERR_VPS_LAYER_PYM_NOT_READY,
	ERR_VPS_LAYER_LG_DSUS_LAYER_EN,
	ERR_VPS_LAYER_ENTITY_INIT,
	ERR_VPS_LAYER_DISABLE,
	ERR_VPS_LAYER_GET_BIND_OTHER_DEV,
	ERR_VPS_LAYER_GET_FRM_DONE_NULL,
	ERR_VPS_LAYER_OUT_BUF_MGR_NULL
};

#define HB_ERR_VPS_INVALID_GROUPID\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_INVALID_GROUPID)
#define HB_ERR_VPS_BUFMGR\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_BUFMGR)
#define HB_ERR_VPS_GROUP_FAIL\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_GROUP_FAIL)
#define HB_ERR_VPS_GROUP_UNEXIST\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_GROUP_UNEXIST)
#define HB_ERR_VPS_CHN_UNEXIST\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_CHN_UNEXIST)
#define HB_ERR_VPS_ROTATE\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_ROTATE)
#define HB_ERR_VPS_NULL_PARA\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_NULL_PARA)
#define HB_ERR_VPS_BAD_ARG\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_BAD_ARG)
#define HB_ERR_VPS_UN_PREPARED\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_UN_PREPARED)
#define HB_ERR_VPS_SENDFRAME\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_SENDFRAME)
#define HB_ERR_VPS_CHN_DISABLE\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_CHN_DISABLE)
#define HB_ERR_VPS_TIMEOUT\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_TIMEOUT)
#define HB_ERR_VPS_CHN_FD\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_CHN_FD)
#define HB_ERR_VPS_SET_AFTER_START\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_SET_AFTER_START)
#define HB_ERR_VPS_SET_BEFORE_START\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_SET_BEFORE_START)
#define HB_ERR_VPS_SET_AT_WRONG_TIME\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_SET_AT_WRONG_TIME)
#define HB_ERR_VPS_UN_SUPPORT_SIZE\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_UN_SUPPORT_SIZE)
#define HB_ERR_VPS_FRAME_UNEXIST\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_FRAME_UNEXIST)
#define HB_ERR_VPS_DEV_FRAME_DROP\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_DEV_FRAME_DROP)
#define HB_ERR_VPS_NOT_ENOUGH\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_NOT_ENOUGH)
#define HB_ERR_VPS_UN_SUPPORT_RATE\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_UN_SUPPORT_RATE)
#define HB_ERR_VPS_FRAME_RATE\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_FRAME_RATE)
/* pym layer error: */
#define HB_ERR_VPS_LAYER_UNEXIST\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_LAYER_UNEXIST)
#define HB_ERR_VPS_LAYER_PYM_NOT_READY\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_LAYER_PYM_NOT_READY)
#define HB_ERR_VPS_LAYER_LG_DSUS_LAYER_EN\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_LAYER_LG_DSUS_LAYER_EN)
#define HB_ERR_VPS_LAYER_ENTITY_INIT\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_LAYER_ENTITY_INIT)
#define HB_ERR_VPS_LAYER_DISABLE\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_LAYER_DISABLE)
#define HB_ERR_VPS_LAYER_GET_BIND_OTHER_DEV\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_LAYER_GET_BIND_OTHER_DEV)
#define HB_ERR_VPS_LAYER_GET_FRM_DONE_NULL\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_LAYER_GET_FRM_DONE_NULL)
#define HB_ERR_VPS_LAYER_OUT_BUF_MGR_NULL\
	HB_DEF_ERR(HB_ID_VPS, ERR_VPS_LAYER_OUT_BUF_MGR_NULL)


typedef enum HB_PYM_LAYER_ID_E
{
	PYM_DS_BASE_0 = 0,
	PYM_DS_ROI_1,
	PYM_DS_ROI_2,
	PYM_DS_ROI_3,
	PYM_DS_BASE_4,
	PYM_DS_ROI_5,
	PYM_DS_ROI_6,
	PYM_DS_ROI_7,
	PYM_DS_BASE_8,
	PYM_DS_ROI_9,
	PYM_DS_ROI_10,
	PYM_DS_ROI_11,
	PYM_DS_BASE_12,
	PYM_DS_ROI_13,
	PYM_DS_ROI_14,
	PYM_DS_ROI_15,
	PYM_DS_BASE_16,
	PYM_DS_ROI_17,
	PYM_DS_ROI_18,
	PYM_DS_ROI_19,
	PYM_DS_BASE_20,
	PYM_DS_ROI_21,
	PYM_DS_ROI_22,
	PYM_DS_ROI_23,
	PYM_US_0,
	PYM_US_1,
	PYM_US_2,
	PYM_US_3,
	PYM_US_4,
	PYM_US_5,
	PYM_MAX_LAYER_NUM,
}PYM_LAYER_ID_E;


typedef struct HB_VPS_GRP_ATTR_S {
	uint32_t		maxW;
	uint32_t		maxH;
	uint8_t 		frameDepth;
	int			imageSplice;
} VPS_GRP_ATTR_S;

typedef struct HB_RECT_S {
	uint16_t	x;
	uint16_t	y;
	uint16_t	width;
	uint16_t	height;
} RECT_S;

typedef struct HB_VPS_CROP_INFO_S {
	uint8_t		en;
	RECT_S		cropRect;
} VPS_CROP_INFO_S;

typedef struct HB_FRAME_RATE_CTRL_S {
	uint32_t		srcFrameRate;
	uint32_t		dstFrameRate;
} FRAME_RATE_CTRL_S;

typedef struct HB_VPS_CHN_ATTR_S {
	uint32_t		width;
	uint32_t		height;
	int				pixelFormat;
	uint8_t			enMirror;
	uint8_t			enFlip;
	uint8_t			enScale;
	uint32_t		frameDepth;
	FRAME_RATE_CTRL_S	frameRate;
} VPS_CHN_ATTR_S;

typedef enum HB_ROTATION_E {
	ROTATION_0 = 0,
	ROTATION_90,
	ROTATION_180,
	ROTATION_270,
	ROTATION_MAX,
} ROTATION_E;

typedef struct HB_VPS_DYNAMIC_SRC_INFO_S {
	uint8_t 		src_change_en;
	uint16_t 		new_width;
	uint16_t 		new_height;
} DYNAMIC_SRC_INFO_S;

typedef struct HB_PYM_SCALE_INFO_S {
	uint8_t			factor;
	uint16_t		roi_x;
	uint16_t		roi_y;
	uint16_t		roi_width;
	uint16_t		roi_height;
} PYM_SCALE_INFO_S;

typedef struct HB_VPS_PYM_CHN_ATTR_S {
	uint32_t		frame_id;
	uint32_t		ds_uv_bypass;
	uint16_t		ds_layer_en;
	uint8_t			us_layer_en;
	uint8_t			us_uv_bypass;
	int				timeout;
	uint32_t		frameDepth;
	DYNAMIC_SRC_INFO_S	dynamic_src_info;
	#define			MAX_PYM_DS_NUM			24
	#define			MAX_PYM_US_NUM			6
	PYM_SCALE_INFO_S ds_info[MAX_PYM_DS_NUM];
	PYM_SCALE_INFO_S us_info[MAX_PYM_US_NUM];
} VPS_PYM_CHN_ATTR_S;

/*typedef struct HB_VIDEO_REGION_INFO_S {*/
	//uint32_t 		regionNum;
	//RECT_S			*region;
/*} VIDEO_REGION_INFO_S;*/

typedef struct HB_DIS_MV_INFO_S {
	int    gmvX;
        int    gmvY;
        int    xUpdate;
        int    yUpdate;
} DIS_MV_INFO_S;

typedef struct HB_VPS_PYM_LAYER_ATTR_S {
	uint32_t		frameDepth;
	uint32_t		uv_bypass;
	uint8_t			factor;
	uint16_t		roi_x;
	uint16_t		roi_y;
	uint16_t		roi_width;
	uint16_t		roi_height;
	uint16_t		target_width;
	uint16_t		target_height;
}VPS_PYM_LAYER_ATTR_S;


int HB_VPS_CreateGrp(int VpsGrp, const VPS_GRP_ATTR_S *grpAttr);
int HB_VPS_DestroyGrp(int VpsGrp);
int HB_VPS_StartGrp(int VpsGrp);
int HB_VPS_StopGrp(int VpsGrp);
int HB_VPS_GetGrpAttr(int VpsGrp, VPS_GRP_ATTR_S *grpAttr);
int HB_VPS_SetGrpAttr(int VpsGrp, const VPS_GRP_ATTR_S *grpAttr);

int HB_VPS_SetGrpRotate(int VpsGrp, ROTATION_E enRotation);
int HB_VPS_GetGrpRotate(int VpsGrp, ROTATION_E *enRotation);
int HB_VPS_SetGrpRotateRepeat(int VpsGrp, ROTATION_E enRotation);
int HB_VPS_SetGrpGdc(int VpsGrp, char* buf_addr, uint32_t buf_len,
								ROTATION_E enRotation);
int HB_VPS_SetChnGdc(int VpsGrp, int VpsChn, char* buf_addr, uint32_t buf_len,
								ROTATION_E enRotation);
int HB_VPS_UpdateGdcSize(int VpsGrp, int VpsChn,
				uint16_t out_width, uint16_t out_height);

int HB_VPS_SetChnAttr(int VpsGrp, int VpsChn, const VPS_CHN_ATTR_S *chnAttr);
int HB_VPS_GetChnAttr(int VpsGrp, int VpsChn, VPS_CHN_ATTR_S *chnAttr);
int HB_VPS_SetChnFrameRate(int VpsGrp, int VpsChn,
		FRAME_RATE_CTRL_S *frameRate);

int HB_VPS_SetChnCrop(int VpsGrp, int VpsChn, const VPS_CROP_INFO_S *cropInfo);
int HB_VPS_GetChnCrop(int VpsGrp, int VpsChn, VPS_CROP_INFO_S *cropInfo);

int HB_VPS_SetChnRotate(int VpsGrp, int VpsChn, ROTATION_E enRotation);
int HB_VPS_GetChnRotate(int VpsGrp, int VpsChn, ROTATION_E *enRotation);

int HB_VPS_SetPymChnAttr(int VpsGrp, int VpsChn,
										const VPS_PYM_CHN_ATTR_S *pymChnAttr);
int HB_VPS_GetPymChnAttr(int VpsGrp, int VpsChn,
										VPS_PYM_CHN_ATTR_S *pymChnAttr);
int HB_VPS_ChangePymUs(int VpsGrp, uint8_t us_num, uint8_t enable);
int HB_VPS_EnableChn(int VpsGrp, int VpsChn);
int HB_VPS_DisableChn(int VpsGrp, int VpsChn);
int HB_VPS_SendFrame(int VpsGrp, void* videoFrame, int ms);
int HB_VPS_SendFirstFrame(int VpsGrp, RECT_S *rect, void *videoFrame);
int HB_VPS_SendMiddleFrame(int VpsGrp, RECT_S *rect, void *videoFrame);
int HB_VPS_SendLastFrame(int VpsGrp, RECT_S *rect, void *videoFrame);
static inline int VPS_SendFrameOpt(int VpsGrp, RECT_S *rect, void *videoFrame,
		uint16_t first, uint16_t last);

int HB_VPS_GetChnFrame(int VpsGrp, int VpsChn, void *videoFrame, int ms);
int HB_VPS_GetChnFrame_Cond(int VpsGrp, int VpsChn, void *videoFrame,
		int ms, int time);
int HB_VPS_ReleaseChnFrame(int VpsGrp, int VpsChn, void *videoFrame);

/*int HB_VPS_GetRegionLuma(int VpsGrp, int VpsChn,*/
									//const VIDEO_REGION_INFO_S *regionInfo,
                            /*uint64_t *lumaData);*/

int HB_VPS_GetChnFd(int VpsGrp, int VpsChn);

int HB_VPS_CloseFd(void);

int HB_VPS_SetPymLayerAttr(int VpsGrp, PYM_LAYER_ID_E Layer,
		const VPS_PYM_LAYER_ATTR_S *pymLayerAttr);
int HB_VPS_GetPymLayerAttr(int VpsGrp, PYM_LAYER_ID_E Layer,
		VPS_PYM_LAYER_ATTR_S *pymLayerAttr);
int HB_VPS_EnablePymLayer(int VpsGrp, PYM_LAYER_ID_E Layer);
int HB_VPS_DisablePymLayer(int VpsGrp, PYM_LAYER_ID_E Layer);
int HB_VPS_GetLayerFrame(int VpsGrp, PYM_LAYER_ID_E Layer,
		void *videoFrame, int ms);
int HB_VPS_ReleaseLayerFrame(int VpsGrp, PYM_LAYER_ID_E Layer,
		void *videoFrame);

int HB_VPS_TriggerSnapFrame(int VpsGrp, int VpsChn, uint32_t frameCnt);

// int HB_VPS_SetRoiInfo(int VpsGrp, DIS_MV_INFO_S *mv_info);

#ifdef __cplusplus
}
#endif

#endif  // __HB_VPS_API_H
