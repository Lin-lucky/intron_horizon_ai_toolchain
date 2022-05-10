/*************************************************************************
 *                     COPYRIGHT NOTICE
 *            Copyright 2019 Horizon Robotics, Inc.
 *                   All rights reserved.
 *************************************************************************/
#ifndef __HB_ISP_API_H__
#define __HB_ISP_API_H__

#include <stdint.h>
#include "hb_isp_algo.h"

// ------------------------------------------------------------------------------ //
//		VALUE LIST
// ------------------------------------------------------------------------------ //
#define NORMAL                                            0x0000003E
#define BLACK_AND_WHITE                                   0x0000003F
#define NEGATIVE                                          0x00000040
#define SEPIA                                             0x00000041
#define VIVID                                             0x00000042

/*****************************************************************************************
 *					Control API 				 	 *
 *****************************************************************************************/
typedef enum HB_ISP_OP_TYPE_E {
	OP_TYPE_AUTO = 0,
	OP_TYPE_MANUAL,
} ISP_OP_TYPE_E;

typedef enum HB_ISP_FW_STATE_E {
	ISP_FW_STATE_RUN = 0,
	ISP_FW_STATE_FREEZE,
} ISP_FW_STATE_E;

typedef struct HB_ISP_CTRL_PARAM_ATTR_S {
	uint32_t u32Offset;
	uint32_t u32Thresh;
	uint32_t u32Slope;
} ISP_CTRL_PARAM_ATTR_S;

typedef struct HB_ISP_CTRL_PARAMA_ATTR_S {
	uint32_t u32Thresh;
	uint32_t u32Slope;
} ISP_CTRL_PARAMA_ATTR_S;

typedef union HB_ISP_MODULE_CTRL_U {
	uint64_t u64Value;
	struct {
		uint64_t bitBypassVideoTestGen			: 1 ;	/* 0x18EAC */
		uint64_t bitBypassInputFormatter		: 1 ;
		uint64_t bitBypassDecompander			: 1 ;
		uint64_t bitBypassSensorOffsetWDR		: 1 ;
		uint64_t bitBypassGainWDR			: 1 ;
		uint64_t bitBypassFrameStitch			: 1 ;
		uint64_t bitBypassDigitalGain			: 1 ;	/* 0x18EB0 */
		uint64_t bitBypassFrontendSensorOffset		: 1 ;
		uint64_t bitBypassFeSqrt			: 1 ;
		uint64_t bitBypassRAWFrontend			: 1 ;
		uint64_t bitBypassDefectPixel			: 1 ;
		uint64_t bitBypassSinter			: 1 ;	/* 0x18EB8 */
		uint64_t bitBypassTemper			: 1 ;
		uint64_t bitBypassCaCorrection			: 1 ;
		uint64_t bitBypassSquareBackend			: 1 ;	/* 0x18EBC */
		uint64_t bitBypassSensorOffsetPreShading	: 1 ;
		uint64_t bitBypassRadialShading			: 1 ;
		uint64_t bitBypassMeshShading			: 1 ;
		uint64_t bitBypassWhiteBalance			: 1 ;
		uint64_t bitBypassIridixGain			: 1 ;
		uint64_t bitBypassIridix			: 1 ;
		uint64_t bitBypassMirror			: 1 ;	/* 0x18EC0 */
		uint64_t bitBypassDemosaicRGB			: 1 ;
		uint64_t bitBypassPfCorrection			: 1 ;
		uint64_t bitBypassCCM				: 1 ;
		uint64_t bitBypassCNR				: 1 ;
		uint64_t bitBypass3Dlut				: 1 ;
		uint64_t bitBypassNonequGamma			: 1 ;
		uint64_t bitBypassFrCrop			: 1 ;	/* 0x18EC4 */
		uint64_t bitBypassFrGammaRGB			: 1 ;
		uint64_t bitBypassFrSharpen			: 1 ;
		uint64_t bitBypassFrCsConv			: 1 ;
		uint64_t bitBypassRAW				: 1 ;	/* [32] 0x18ECC */
		uint64_t bitRsv					: 31 ;	/* [33:63]*/
	};
} ISP_MODULE_CTRL_U;

extern int HB_ISP_GetSetInit(void);
extern int HB_ISP_GetSetExit(void);

extern int HB_ISP_SetFWState(uint8_t pipeId, const ISP_FW_STATE_E enState);
extern int HB_ISP_GetFWState(uint8_t pipeId, ISP_FW_STATE_E *penState);

extern int HB_ISP_SetModuleControl(uint8_t pipeId, const ISP_MODULE_CTRL_U *punModCtrl);
extern int HB_ISP_GetModuleControl(uint8_t pipeId, ISP_MODULE_CTRL_U *punModCtrl);

extern int HB_ISP_SetRegister(uint8_t pipeId, uint32_t u32Addr, uint32_t u32Value);
extern int HB_ISP_GetRegister(uint8_t pipeId, uint32_t u32Addr, uint32_t *pu32Value);

extern int HB_ISP_SwitchScence(uint8_t pipeId, const char *cname);

/*****************************************************************************************
 *				  3ALib Register API			 	 	 *
 *****************************************************************************************/
/* AE */
typedef enum ae_state {
    AE_STATE_INACTIVE,
    AE_STATE_SEARCHING,
    AE_STATE_CONVERGED,
} ae_state_t;

typedef struct _ae_stats_data_ {
	uint32_t *fullhist;
	uint32_t fullhist_size;
	uint32_t fullhist_sum;
	uint16_t *zone_hist;
	uint32_t zone_hist_size;
} ae_stats_data_t;

typedef struct _ae_input_data_ {
	void *custom_input;
	void *acamera_input;
} ae_input_data_t;

typedef struct _ae_output_data_ {
	void *custom_output;
	void *acamera_output;
} ae_output_data_t;

typedef struct _ae_acamera_output_ {
	int32_t exposure_log2;
	uint32_t exposure_ratio;
	uint8_t ae_converged;
	uint16_t sensor_ctrl_enable;
	ae_out_info_t ae_out_info;
	ae_1024bin_weight_t ae_1024bin_weight;
} ae_acamera_output_t;

typedef struct HB_ISP_AE_FUNC_S {
	void *(*init_func)(uint32_t ctx_id);
	int32_t (*proc_func)(void *ae_ctx, ae_stats_data_t *stats, ae_input_data_t *input, ae_output_data_t *output);
	int32_t (*deinit_func)(void *ae_ctx);
} ISP_AE_FUNC_S;

/* AWB */
typedef enum awb_state {
    AWB_STATE_INACTIVE,
    AWB_STATE_SEARCHING,
    AWB_STATE_CONVERGED,
} awb_state_t;

typedef struct _awb_zone_t {
	uint16_t rg;
	uint16_t bg;
	uint32_t sum;
} awb_zone_t;

typedef struct _awb_stats_data_ {
	awb_zone_t *awb_zones;
	uint32_t zones_size;
} awb_stats_data_t;

typedef struct _awb_input_data_ {
	void *custom_input;
	void *acamera_input;
} awb_input_data_t;

typedef struct _awb_output_data_ {
	void *custom_output;
	void *acamera_output;
} awb_output_data_t;

typedef struct _awb_acamera_output_ {
	uint16_t rg_coef;
	uint16_t bg_coef;
	int32_t temperature_detected;
	uint8_t p_high;
	uint8_t light_source_candidate;
	int32_t awb_warming[3];
	uint8_t awb_converged;
} awb_acamera_output_t;

typedef struct HB_ISP_AWB_FUNC_S {
	void *(*init_func)(uint32_t ctx_id);
	int32_t (*proc_func)(void *awb_ctx, awb_stats_data_t *stats, awb_input_data_t *input, awb_output_data_t *output);
	int32_t (*deinit_func)(void *awb_ctx);
} ISP_AWB_FUNC_S;

/* AF */
typedef enum af_state {
	AF_STATE_INACTIVE,
	AF_STATE_SCAN,
	AF_STATE_FOCUSED,
	AF_STATE_UNFOCUSED
} af_state_t;

typedef struct _af_stats_data_ {
	uint32_t *zones_stats;
	uint32_t zones_size;
} af_stats_data_t;

typedef struct _af_input_data_ {
	void *custom_input;
	void *acamera_input;
} af_input_data_t;

typedef struct _af_output_data_ {
	void *custom_output;
	void *acamera_output;
} af_output_data_t;

typedef struct _af_acamera_output_ {
	uint16_t af_lens_pos;
	int32_t af_sharp_val;
	af_state_t state;
} af_acamera_output_t;

typedef struct HB_ISP_AF_FUNC_S {
	void *(*init_func)(uint32_t ctx_id);
	int32_t (*proc_func)(void *af_ctx, af_stats_data_t *stats, af_input_data_t *input, af_output_data_t *output);
	int32_t (*deinit_func)(void *af_ctx);
} ISP_AF_FUNC_S;

extern int HB_ISP_AELibRegCallback(uint8_t pipeId, char *name, ISP_AE_FUNC_S *pstAeFunc);
extern int HB_ISP_AWBLibRegCallback(uint8_t pipeId, char *name, ISP_AWB_FUNC_S *pstAWBFunc);
extern int HB_ISP_AFLibRegCallback(uint8_t pipeId, char *name, ISP_AF_FUNC_S *pstAFFunc);

extern int HB_ISP_AELibUnRegCallback(uint8_t pipeId);
extern int HB_ISP_AWBLibUnRegCallback(uint8_t pipeId);
extern int HB_ISP_AFLibUnRegCallback(uint8_t pipeId);

/*****************************************************************************************
 *				  Module Tune API 				 	 *
 *****************************************************************************************/
#define ISP_AUTO_ISO_STRENGTH_NUM	16
#define ISP_IRIDIX_ASYMMETRY_NUM            65

/* AE */ //cmd
typedef struct HB_ISP_AE_ATTR_S {
	uint32_t u32MaxExposureRatio;
	uint32_t u32MaxIntegrationTime;
	uint32_t u32MaxSensorAnalogGain;
	uint32_t u32MaxSensorDigitalGain;
	uint32_t u32MaxIspDigitalGain;
	uint32_t u32Exposure;
	uint32_t u32ExposureRatio;
	uint32_t u32IntegrationTime;
//	uint32_t u32LongIntegrationTime;
//	uint32_t u32ShortIntegrationTime;
	uint32_t u32SensorAnalogGain;
	uint32_t u32SensorDigitalGain;
	uint32_t u32IspDigitalGain;
	ISP_OP_TYPE_E enOpType;
} ISP_AE_ATTR_S;

/* AE ex attr */ //lut
typedef struct HB_ISP_AE_ATTR_EX_S {
	uint32_t u32Compensation;
	uint32_t u32Speed;
	uint32_t u32Tolerance;
} ISP_AE_ATTR_EX_S;

/*AF*/ // cmd
typedef struct HB_ISP_AF_ATTR_S {
	uint32_t u32ZoomPos;
	ISP_OP_TYPE_E enOpType;
} ISP_AF_ATTR_S;

/* AWB */ //cmd
typedef struct HB_ISP_AWB_ATTR_S {
	uint32_t u16RGain;
	uint32_t u16BGain;
	ISP_OP_TYPE_E enOpType;
} ISP_AWB_ATTR_S;

/* Black Level */
typedef struct HB_ISP_BLACK_LEVEL_ATTR_S {
	uint32_t u32OffsetR;
	uint32_t u32OffsetGr;
	uint32_t u32OffsetGb;
	uint32_t u32OffsetB;
	ISP_OP_TYPE_E enOpType;
} ISP_BLACK_LEVEL_ATTR_S;

/* Demosaic */ //reg + lut
typedef struct HB_ISP_DEMOSAIC_ATTR_S {
	uint32_t u32FcSlope;
	uint32_t u32FcAliasSlope;
	uint32_t u32FcAliasThresh;
	ISP_CTRL_PARAM_ATTR_S VhParam;
	ISP_CTRL_PARAM_ATTR_S AaParam;
	ISP_CTRL_PARAM_ATTR_S VaParam;
	ISP_CTRL_PARAM_ATTR_S UuParam;
	ISP_CTRL_PARAM_ATTR_S UuShParam;
	ISP_CTRL_PARAM_ATTR_S SharpLumaLowD;
	ISP_CTRL_PARAMA_ATTR_S SharpLumaHighD;
	ISP_CTRL_PARAM_ATTR_S SharpLumaLowUD;
	ISP_CTRL_PARAMA_ATTR_S SharpLumaHighUD;
	uint32_t u32MinDstrength;
	uint32_t u32MinUDstrength;
	uint32_t u32MaxDstrength;
	uint32_t u32MaxUDstrength;
	uint16_t u16NpOffset[ISP_AUTO_ISO_STRENGTH_NUM][2];
} ISP_DEMOSAIC_ATTR_S;

/* Sharpen */ //lut
typedef struct HB_ISP_SHARPEN_AUTO_ATTR_S {
	uint16_t u16SharpFR[ISP_AUTO_ISO_STRENGTH_NUM][2];
	uint16_t u16SharpAltD[ISP_AUTO_ISO_STRENGTH_NUM][2];
	uint16_t u16SharpAltDU[ISP_AUTO_ISO_STRENGTH_NUM][2];
	uint16_t u16SharpAltUD[ISP_AUTO_ISO_STRENGTH_NUM][2];
} ISP_SHARPEN_AUTO_ATTR_S;

typedef struct HB_ISP_SHARPEN_MANUAL_ATTR_S {
	uint32_t u32Strength;
	uint32_t u32SharpAltD;
	uint32_t u32SharpAltUd;
        uint32_t u32SharpAltLd;
        uint32_t u32SharpAltLdu;
        uint32_t u32SharpAltLu;
} ISP_SHARPEN_MANUAL_ATTR_S;

typedef struct HB_ISP_SHARPEN_ATTR_S {
	ISP_CTRL_PARAM_ATTR_S LumaLow;
	ISP_CTRL_PARAM_ATTR_S LumaHigh;
	uint32_t u32ClipStrMax;
	uint32_t u32ClipStrMin;
	uint32_t u32AlphaUndershoot;
	uint32_t u32SadAmp;
	ISP_SHARPEN_MANUAL_ATTR_S stManual;
	ISP_SHARPEN_AUTO_ATTR_S stAuto;
	ISP_OP_TYPE_E enOpType;
} ISP_SHARPEN_ATTR_S;

/* Gamma */ //lut
typedef struct HB_ISP_GAMMA_ATTR_S {
	uint16_t au16Gamma[129];
} ISP_GAMMA_ATTR_S;

/* Iridix */ //lut + reg
typedef struct HB_ISP_IRIDIX_AUTO_ATTR_S {
	uint8_t u8AvgCoef;
	uint32_t au32EvLimNoStr[2];
	uint32_t u32EvLimFullStr;
	uint32_t au32StrengthDkEnhControl[15];
} ISP_IRIDIX_AUTO_ATTR_S;

typedef struct HB_ISP_IRIDIX_MANUAL_ATTR_S {
	uint32_t u32RoiHorStart;
	uint32_t u32RoiHorEnd;
	uint32_t u32RoiVerStart;
	uint32_t u32RoiVerEnd;
	uint32_t u32StrengthInRoi;
	uint32_t u32StrengthOutRoi;
	uint32_t u32DarkEnh;
} ISP_IRIDIX_MANUAL_ATTR_S;

typedef struct HB_ISP_IRIDIX_ATTR_S {
	ISP_OP_TYPE_E enOpType;
	ISP_IRIDIX_AUTO_ATTR_S stAuto;
	ISP_IRIDIX_MANUAL_ATTR_S stManual;
	uint32_t u32BlackLevel;
	uint32_t u32WhiteLevel;
	uint32_t u32Svariance;
	uint32_t u32Bright_pr;
	uint32_t u32Contrast;
	uint32_t u32IridixOn;
	uint32_t u32FilterMux;
	uint32_t u32VarianceSpace;
	uint32_t u32VarianceIntensity;
	uint32_t u32SlopeMax;
	uint32_t u32SlopeMin;
	uint32_t u32FwdPerceptCtrl;
	uint32_t u32RevPerceptCtrl;
	uint32_t u32FwdAlpha;
	uint32_t u32RevAlpha;
	uint32_t u32GtmSelect;
	uint32_t u32IridixAsymmetry[ISP_IRIDIX_ASYMMETRY_NUM];
} ISP_IRIDIX_ATTR_S;

/* CNR */ //lut
typedef struct HB_ISP_CNR_ATTR_S {
	uint16_t u16UvDelta12Slope[ISP_AUTO_ISO_STRENGTH_NUM][2];
} ISP_CNR_ATTR_S;

/* Sinter */ //lut
typedef struct HB_ISP_SINTER_AUTO_ATTR_S {
	uint16_t aau16Strength[ISP_AUTO_ISO_STRENGTH_NUM][2];
	uint16_t aau16Strength1[ISP_AUTO_ISO_STRENGTH_NUM][2];
	uint16_t aau16Strength4[ISP_AUTO_ISO_STRENGTH_NUM][2];
	uint16_t aau16Thresh1[ISP_AUTO_ISO_STRENGTH_NUM][2];
	uint16_t aau16Thresh4[ISP_AUTO_ISO_STRENGTH_NUM][2];
} ISP_SINTER_AUTO_ATTR_S;

typedef struct HB_ISP_SINTER_MANUAL_PARAM_ATTR_S {
	uint32_t u32GlobalStrength;
	uint32_t u32Thresh1h;
	uint32_t u32Thresh4h;
	uint32_t u32Thresh1v;
	uint32_t u32Thresh4v;
	uint32_t u32Strength1;
	uint32_t u32Strength4;
	uint32_t u32NoiseLevel0;
	uint32_t u32NoiseLevel1;
	uint32_t u32NoiseLevel2;
	uint32_t u32NoiseLevel3;
	uint32_t u32IntConfig;
	uint32_t u32SadFiltThresh;
} ISP_SINTER_MANUAL_PARAM_ATTR_S;

typedef struct HB_ISP_SINTER_ATTR_S {
	ISP_OP_TYPE_E enOpType;
	ISP_SINTER_AUTO_ATTR_S stAuto;
	ISP_SINTER_MANUAL_PARAM_ATTR_S stManual;
} ISP_SINTER_ATTR_S;

/* Temper */ //lut + reg
typedef struct HB_ISP_TEMPER_ATTR_S {
	uint16_t aau16Strength[ISP_AUTO_ISO_STRENGTH_NUM][2];
	uint32_t u32RecursionLimit;
	uint32_t u32LutEnable;
} ISP_TEMPER_ATTR_S;

/* Scene Mode */ //cmd
typedef struct HB_ISP_SCENE_MODES_ATTR_S {
	uint32_t u32ColorMode;
	uint32_t u32BrightnessStrength;
	uint32_t u32ContrastStrength;
	uint32_t u32SaturationStrength;
	uint32_t u32HueTheta;
} ISP_SCENE_MODES_ATTR_S;

/* Mesh Shading */ //reg
typedef struct HB_MESH_SHADING_ATTR_S {
	uint32_t u32Enable;
	uint32_t u32MeshScale;
	uint32_t u32MeshAlphaMode;
	uint32_t u32MeshWidth;
	uint32_t u32MeshHeight;
	uint32_t u32ShadingStrength;
} MESH_SHADING_ATTR_S;

/* Mesh Shading LUT */ //reg
typedef struct HB_MESH_SHADING_LUT_S {
	uint8_t au8LsAR[1024];
	uint8_t au8LsAG[1024];
	uint8_t au8LsAB[1024];
	uint8_t au8LsTl84R[1024];
	uint8_t au8LsTl84G[1024];
	uint8_t au8LsTl84B[1024];
	uint8_t au8LsD65R[1024];
	uint8_t au8LsD65G[1024];
	uint8_t au8LsD65B[1024];
} MESH_SHADING_LUT_S;

/* Radial Shading */ //reg
typedef struct HB_RADIAL_SHADING_ATTR_S {
	uint32_t u32Enable;
	uint32_t u32CenterRX;
	uint32_t u32CenterRY;
	uint32_t u32CenterGX;
	uint32_t u32CenterGY;
	uint32_t u32CenterBX;
	uint32_t u32CenterBY;
	uint32_t u32OffCenterMultRX;
	uint32_t u32OffCenterMultRY;
	uint32_t u32OffCenterMultGX;
	uint32_t u32OffCenterMultGY;
	uint32_t u32OffCenterMultBX;
	uint32_t u32OffCenterMultBY;
} RADIAL_SHADING_ATTR_S;

/* Radial Shading LUT */ //reg
typedef struct HB_RADIAL_SHADING_LUT_S {
	uint16_t au16RGain[129];
	uint16_t au16GGain[129];
	uint16_t au16BGain[129];
} RADIAL_SHADING_LUT_S;

/* CSC */ //lut + reg
typedef struct HB_ISP_CSC_ATTR_S {
	uint32_t u32ClipMinY;
	uint32_t u32ClipMaxY;
	uint32_t u32ClipMinUV;
	uint32_t u32ClipMaxUV;
	uint32_t u32MaskRY;
	uint32_t u32MaskGU;
	uint32_t u32MaskBV;
	uint16_t aau16Coefft[12];
} ISP_CSC_ATTR_S;

/* AE */
extern int HB_ISP_SetAeAttr(uint8_t pipeId, const ISP_AE_ATTR_S *pstAeAttr);
extern int HB_ISP_GetAeAttr(uint8_t pipeId, ISP_AE_ATTR_S *pstAeAttr);

/* AWB */
extern int HB_ISP_SetAwbAttr(uint8_t pipeId, const ISP_AWB_ATTR_S *pstAwbAttr);
extern int HB_ISP_GetAwbAttr(uint8_t pipeId, ISP_AWB_ATTR_S *pstAwbAttr);

/* AF */
extern int HB_ISP_SetAfAttr(uint8_t pipeId, ISP_AF_ATTR_S *pstAfAttr);
extern int HB_ISP_GetAfAttr(uint8_t pipeId, ISP_AF_ATTR_S *pstAfAttr);

/* Black Level */
extern int HB_ISP_SetBlackLevelAttr(uint8_t pipeId, const ISP_BLACK_LEVEL_ATTR_S *pstBlackLevelAttr);
extern int HB_ISP_GetBlackLevelAttr(uint8_t pipeId, ISP_BLACK_LEVEL_ATTR_S *pstBlackLevelAttr);

/* Demosaic */
extern int HB_ISP_SetDemosaicAttr(uint8_t pipeId, const ISP_DEMOSAIC_ATTR_S *pstDemosaicAttr);
extern int HB_ISP_GetDemosaicAttr(uint8_t pipeId, ISP_DEMOSAIC_ATTR_S *pstDemosaicAttr);

/* Sharpen */
extern int HB_ISP_SetSharpenAttr(uint8_t pipeId, const ISP_SHARPEN_ATTR_S *pstSharpenAttr);
extern int HB_ISP_GetSharpenAttr(uint8_t pipeId, ISP_SHARPEN_ATTR_S *pstSharpenAttr);

/* Gamma */
extern int HB_ISP_SetGammaAttr(uint8_t pipeId, const ISP_GAMMA_ATTR_S *pstGammaAttr);
extern int HB_ISP_GetGammaAttr(uint8_t pipeId, ISP_GAMMA_ATTR_S *pstGammaAttr);

/* Iridix */
extern int HB_ISP_SetIridixAttr(uint8_t pipeId, const ISP_IRIDIX_ATTR_S *pstIridixAttr);
extern int HB_ISP_GetIridixAttr(uint8_t pipeId, ISP_IRIDIX_ATTR_S *pstIridixAttr);
extern int HB_ISP_SetIridixStrengthLevel(uint8_t pipeId, uint16_t level);
extern int HB_ISP_GetIridixStrengthLevel(uint8_t pipeId, uint16_t *level);

/* CNR */
extern int HB_ISP_SetCnrAttr(uint8_t pipeId, const ISP_CNR_ATTR_S *pstCnrAttr);
extern int HB_ISP_GetCnrAttr(uint8_t pipeId, ISP_CNR_ATTR_S *pstCnrAttr);

/* Sinter */
extern int HB_ISP_SetSinterAttr(uint8_t pipeId, const ISP_SINTER_ATTR_S *pstSinterAttr);
extern int HB_ISP_GetSinterAttr(uint8_t pipeId, ISP_SINTER_ATTR_S *pstSinterAttr);

/* Temper */
extern int HB_ISP_SetTemperAttr(uint8_t pipeId, const ISP_TEMPER_ATTR_S *pstTemperAttr);
extern int HB_ISP_GetTemperAttr(uint8_t pipeId, ISP_TEMPER_ATTR_S *pstTemperAttr);

/* Mesh Shading */
extern int HB_ISP_SetMeshShadingAttr(uint8_t pipeId, const MESH_SHADING_ATTR_S *pstMeshShadingAttr);
extern int HB_ISP_GetMeshShadingAttr(uint8_t pipeId, MESH_SHADING_ATTR_S *pstMeshShadingAttr);
extern int HB_ISP_SetMeshShadingLUT(uint8_t pipeId, const MESH_SHADING_LUT_S *pstMeshShadingLUT);
extern int HB_ISP_GetMeshShadingLUT(uint8_t pipeId, MESH_SHADING_LUT_S *pstMeshShadingLUT);

/* Radial Shading */
extern int HB_ISP_SetRadialShadingAttr(uint8_t pipeId, const RADIAL_SHADING_ATTR_S *pstRadialShadingAttr);
extern int HB_ISP_GetRadialShadingAttr(uint8_t pipeId, RADIAL_SHADING_ATTR_S *pstRadialShadingAttr);
extern int HB_ISP_SetRadialShadingLUT(uint8_t pipeId, const RADIAL_SHADING_LUT_S *pstRadialShadingLUT);
extern int HB_ISP_GetRadialShadingLUT(uint8_t pipeId, RADIAL_SHADING_LUT_S *pstRadialShadingLUT);

/* CSC */
extern int HB_ISP_SetCSCAttr(uint8_t pipeId, const ISP_CSC_ATTR_S *pstCSCAttr);
extern int HB_ISP_GetCSCAttr(uint8_t pipeId, ISP_CSC_ATTR_S *pstCSCAttr);

/* Scene Mode */
extern int HB_ISP_SetSceneModesAttr(uint8_t pipeId, const ISP_SCENE_MODES_ATTR_S *pstSceneModesAttr);
extern int HB_ISP_GetSceneModesAttr(uint8_t pipeId, ISP_SCENE_MODES_ATTR_S *pstSceneModesAttr);

/* af/ae_5bin/awb zone info */
typedef struct HB_ISP_ZONE_ATTR_S {
	uint8_t u8Horiz;
	uint8_t u8Vert;
} ISP_ZONE_ATTR_S;

typedef enum ISP_METERING_DATA_TYPE_S {
     ISP_AE = 1,
     ISP_AWB,
	 ISP_AF,
	 ISP_AE_5BIN,
	 ISP_LUMVAR,
     TYPE_MAX,
} ISP_METERING_DATA_TYPE_E;

extern int HB_ISP_GetMeteringData(uint8_t pipeId, void *data, ISP_METERING_DATA_TYPE_E type, int latest_flag);
extern int HB_ISP_GetAwbZoneInfo(uint8_t pipeId, ISP_ZONE_ATTR_S *awbZoneInfo);
extern int HB_ISP_GetAfZoneInfo(uint8_t pipeId, ISP_ZONE_ATTR_S *afZoneInfo);
extern int HB_ISP_GetAe5binZoneInfo(uint8_t pipeId, ISP_ZONE_ATTR_S *ae5binZoneInfo);
extern int HB_ISP_GetAeZoneInfo(uint8_t pipeId, ISP_ZONE_ATTR_S *aeZoneInfo);
extern int HB_ISP_SetAwbZoneInfo(uint8_t pipeId, ISP_ZONE_ATTR_S awbZoneInfo);
extern int HB_ISP_SetAfZoneInfo(uint8_t pipeId, ISP_ZONE_ATTR_S afZoneInfo);
extern int HB_ISP_SetAe5binZoneInfo(uint8_t pipeId, ISP_ZONE_ATTR_S ae5binZoneInfo);
extern int HB_ISP_SetAeZoneInfo(uint8_t pipeId, ISP_ZONE_ATTR_S aeZoneInfo);

/* Statistics */
#define HB_ISP_MAX_AWB_ZONES (33 * 33)
#define HB_ISP_AF_ZONES_COUNT_MAX (33 * 33)
#define HB_ISP_MAX_AE_5BIN_ZONES (33 * 33)
#define HB_ISP_MAX_AE_ZONES (33 * 33)
#define HB_ISP_FULL_HISTOGRAM_SIZE 1024

typedef struct HB_ISP_STATISTICS_AWB_ZONE_ATTR_S {
	uint16_t u16Rg;
	uint16_t u16Bg;
	uint32_t u32Sum;
} ISP_STATISTICS_AWB_ZONE_ATTR_S;

typedef struct HB_ISP_STATISTICS_AE_5BIN_ZONE_ATTR_S {
	uint16_t u16Hist0;
	uint16_t u16Hist1;
	uint16_t u16Hist3;
	uint16_t u16Hist4;
} ISP_STATISTICS_AE_5BIN_ZONE_ATTR_S;

extern int HB_ISP_GetAeFullHist(uint8_t pipeId, uint32_t *pu32AeFullHist);
extern int HB_ISP_GetAwbZoneHist(uint8_t pipeId, ISP_STATISTICS_AWB_ZONE_ATTR_S *pstAwbZonesAttr);
extern int HB_ISP_GetAe5binZoneHist(uint8_t pipeId, ISP_STATISTICS_AE_5BIN_ZONE_ATTR_S *pst32Ae5bin);
extern int HB_ISP_GetAfZoneHist(uint8_t pipeId, af_stats_data_t *pstAfZonesAttr);

extern int HB_ISP_ApiCtrl(uint8_t pipeId, uint8_t direction, int type, int cmd, uint32_t *val);

extern int HB_ISP_GetVDTTimeOut(uint8_t pipeId, uint8_t vdt_type, uint64_t timeout);
extern int HB_ISP_GetAfKernelInfo(uint8_t pipeId, uint32_t *af_kernel);
extern int HB_ISP_SetAfKernelInfo(uint8_t pipeId, uint32_t af_kernel);

/* AE new param */
typedef struct HB_ISP_AE_PARAM_S {
	uint32_t u32TotalGain;
	ISP_OP_TYPE_E GainOpType;
	uint32_t u32IntegrationTime;
	uint32_t u32ExposureRatio;
	ISP_OP_TYPE_E IntegrationOpType;
} ISP_AE_PARAM_S;

extern int HB_ISP_SetAeParam(uint8_t pipeId, const ISP_AE_PARAM_S *pstAeParam);
extern int HB_ISP_GetAeParam(uint8_t pipeId, ISP_AE_PARAM_S *pstAeParam);

/* ae roi info */
typedef struct HB_ISP_AE_ROI_ATTR_S {
	uint8_t u8XStart;
	uint8_t u8YStart;
	uint8_t u8XEnd;
	uint8_t u8YEnd;
} ISP_AE_ROI_ATTR_S;

extern int HB_ISP_SetAeRoiInfo(uint8_t pipeId, ISP_AE_ROI_ATTR_S aeRoiInfo);
extern int HB_ISP_GetAeRoiInfo(uint8_t pipeId, ISP_AE_ROI_ATTR_S *aeRoiInfo);

typedef struct HB_ISP_STATISTICS_LUMVAR_ZONE_ATTR_S {
	uint16_t u16Var;
	uint16_t u16Mean;
} ISP_STATISTICS_LUMVAR_ZONE_ATTR_S;

extern int HB_ISP_GetLumaZoneHist(uint8_t pipeId, ISP_STATISTICS_LUMVAR_ZONE_ATTR_S *pst32Luma);

/* awb statistical selection area */
typedef struct HB_ISP_AWB_STAT_AREA_ATTR_S {
	uint32_t u32WhiteLevel;
	uint32_t u32BlackLevel;
	uint32_t u32CrRefMax;
	uint32_t u32CrRefMin;
	uint32_t u32CbRefMax;
	uint32_t u32CbRefMin;
	uint32_t u32CrRefHigh;
	uint32_t u32CrRefLow;
	uint32_t u32CbRefHigh;
	uint32_t u32CbRefLow;
} ISP_AWB_STAT_AREA_ATTR_S;

extern int HB_ISP_GetAwbStatAreaAttr(uint8_t pipeId, ISP_AWB_STAT_AREA_ATTR_S *pstAwbStatAreaAttr);
extern int HB_ISP_SetAwbStatAreaAttr(uint8_t pipeId, ISP_AWB_STAT_AREA_ATTR_S *pstAwbStatAreaAttr);

/** **/
typedef struct HB_ISP_I2C_DATA_S {
	uint8_t u8DevId;
	uint8_t u8IntPos;
	uint8_t u8Update;
	uint8_t u8DelayFrameNum;
	uint32_t u32RegAddr;
	uint32_t u32AddrByteNum;
	uint32_t u32Data;
	uint32_t u32DataByteNum;
} ISP_I2C_DATA_S;

int HB_ISP_StartI2CBus(uint8_t pipeId);
void HB_ISP_StopI2CBus(uint8_t pipeId);
int HB_ISP_SendI2CData(ISP_I2C_DATA_S data);

typedef struct HB_TMPER_NP_LUT_S {
        uint8_t au8Np[128];
} TEMPER_NP_LUT_S;

extern int HB_ISP_GetTempLut(uint8_t pipeId, TEMPER_NP_LUT_S *pstTemperLUT);
extern int HB_ISP_SetTempLut(uint8_t pipeId, TEMPER_NP_LUT_S *pstTemperLUT);

// awb
typedef struct HB_ISP_MESH_RGBG_WEIGHT_S {
	uint16_t u16MeshRgbgWeight[15][15];
} ISP_MESH_RGBG_WEIGHT_S;

typedef struct HB_ISP_MESH_LS_WEIGHT_S {
	uint16_t u16MeshLsWeight[15][15];
} ISP_MESH_LS_WEIGHT_S;

typedef struct HB_ISP_MESH_COLOR_TEMP_WEIGHT_S {
	uint16_t u16MeshColorTempWeight[15][15];
} ISP_MESH_COLOR_TEMP_WEIGHT_S;

typedef struct HB_ISP_AWB_POS_STATUS_S {
	uint16_t u16AwbRgPos[15];
	uint16_t u16AwbBgPos[15];
} ISP_AWB_POS_STATUS_S;

typedef struct HB_ISP_AWB_LIGHT_SOURCE_S {
        uint16_t u16ColorTemp[7];
        uint16_t u16RgPosCalc[7];
	uint16_t u16BgPosCalc[7];
} ISP_AWB_LIGHT_SOURCE_S;

typedef struct HB_ISP_AWB_CCT_CTRL_S {
	uint16_t u16AwbColourPre[4];
	uint16_t u16AwbWarmLsA[3];
	uint16_t u16AwbWarmLsD75[3];
	uint16_t u16AwbWarmLsD50[3];
} ISP_AWB_CCT_CTRL_S;

typedef struct HB_ISP_MIX_LIGHT_PARAM_S {
	uint32_t u32MixLightParm[8];
} ISP_MIX_LIGHT_PARAM_S;

typedef struct HB_ISP_SKY_PARAM_S {
	uint16_t u16SkyLuxTh;
	uint16_t u16WbStrength[3];
	uint16_t u16Ct65Pos;
	uint16_t u16Ct40Pos;
} ISP_SKY_PARAM_S;

typedef struct HB_ISP_AWB_DEFAULT_PARAM_S {
	uint16_t u16Ct30Pos;
} ISP_AWB_DEFAULT_PARAM_S;

extern int HB_ISP_SetSkyCtrlAttr(uint8_t pipeId, const ISP_SKY_PARAM_S *pstSkyCtrlAttr);
extern int HB_ISP_GetSkyCtrlAttr(uint8_t pipeId, ISP_SKY_PARAM_S *pstSkyCtrlAttr);
extern int HB_ISP_SetMixLightAttr(uint8_t pipeId, const ISP_MIX_LIGHT_PARAM_S *pstMixLightAttr);
extern int HB_ISP_GetMixLightAttr(uint8_t pipeId, ISP_MIX_LIGHT_PARAM_S *pstMixLightAttr);
extern int HB_ISP_SetCctCtrlAttr(uint8_t pipeId, const ISP_AWB_CCT_CTRL_S *pstCctAttr);
extern int HB_ISP_GetCctCtrlAttr(uint8_t pipeId, ISP_AWB_CCT_CTRL_S *pstCctAttr);
extern int HB_ISP_SetAwbRgBgWeightAttr(uint8_t pipeId, const ISP_MESH_RGBG_WEIGHT_S *pstWeightAttr);
extern int HB_ISP_GetAwbRgBgWeightAttr(uint8_t pipeId, ISP_MESH_RGBG_WEIGHT_S *pstWeightAttr);
extern int HB_ISP_GetAwbLsWeightAttr(uint8_t pipeId, ISP_MESH_LS_WEIGHT_S *pstWeightAttr);
extern int HB_ISP_SetAwbLsWeightAttr(uint8_t pipeId, const ISP_MESH_LS_WEIGHT_S *pstWeightAttr);
extern int HB_ISP_GetAwbDefultParmAttr(uint8_t pipeId, ISP_AWB_DEFAULT_PARAM_S *pstAwbAttr);
extern int HB_ISP_SetAwbDefultParmAttr(uint8_t pipeId, const ISP_AWB_DEFAULT_PARAM_S *pstAwbAttr);
extern int HB_ISP_GetEvToLuxStatustAttr(uint8_t pipeId, uint8_t *pstEvtoluxStatustAttr);
extern int HB_ISP_SetEvToLuxStatustAttr(uint8_t pipeId, const uint8_t *pstEvtoluxStatustAttr);
extern int HB_ISP_SetAwbColorTempWeightAttr(uint8_t pipeId, const ISP_MESH_COLOR_TEMP_WEIGHT_S *pstWeightAttr);
extern int HB_ISP_GetAwbColorTempWeightAttr(uint8_t pipeId, ISP_MESH_COLOR_TEMP_WEIGHT_S *pstWeightAttr);
extern int HB_ISP_SetAwbPosStatusAttr(uint8_t pipeId, const ISP_AWB_POS_STATUS_S *pstPosAttr);
extern int HB_ISP_GetAwbPosStatusAttr(uint8_t pipeId, ISP_AWB_POS_STATUS_S *pstPosAttr);
extern int HB_ISP_SetAwbLightSourceAttr(uint8_t pipeId, const ISP_AWB_LIGHT_SOURCE_S *pstLightAttr);
extern int HB_ISP_GetAwbLightSourceAttr(uint8_t pipeId, ISP_AWB_LIGHT_SOURCE_S *pstLightAttr);


typedef struct HB_ISP_WDR_OFFSET_S {
	uint32_t u32WdrLR;
	uint32_t u32WdrLGr;
	uint32_t u32WdrLGb;
	uint32_t u32WdrLB;
	uint32_t u32WdrMR;
	uint32_t u32WdrMGr;
	uint32_t u32WdrMGb;
	uint32_t u32WdrMB;
	uint32_t u32WdrSR;
	uint32_t u32WdrSGr;
	uint32_t u32WdrSGb;
	uint32_t u32WdrSB;
	uint32_t u32WdrVsR;
	uint32_t u32WdrVsGr;
	uint32_t u32WdrVsGb;
	uint32_t u32WdrVsB;
} ISP_WDR_OFFSET_S;

extern int HB_ISP_GetWdrOffsetAttr(uint8_t pipeId, ISP_WDR_OFFSET_S *pstWdrOffsetAttr);
extern int HB_ISP_SetWdrOffsetAttr(uint8_t pipeId, ISP_WDR_OFFSET_S *pstWdrOffsetAttr);

typedef enum HB_ISP_HDR_AEXP_TYPE_E {
        AEXP_TYPE_SHORT = 0,
        AEXP_TYPE_LONG = 1,
} ISP_HDR_AEXP_TYPE_E;

typedef struct HB_ISP_HDR_AEXP_S {
	ISP_HDR_AEXP_TYPE_E Aexp_Type;
} ISP_HDR_AEXP_S;

extern int HB_ISP_SetHdrAexpTypeAttr(uint8_t pipeId, ISP_HDR_AEXP_S *pstAexpTypeAttr, uint32_t scale_bottom, uint32_t scale_top);

typedef enum HB_ISP_AF_MODE_E {
        AF_MODE_MANUAL = 0,
        AF_MODE_CONTINUOUS = 1,
} ISP_AF_MODE_E;

typedef enum HB_ISP_AF_STATUS_E {
	AF_STATUS_INVALID = 0,
	AF_STATUS_FAST_SEARCH = 1,
	AF_STATUS_TRACKING = 2,
	AF_STATUS_CONVERGED = 3,
} ISP_AF_STATUS_E;

typedef struct HB_ISP_AF_WEIGHT_S {
	uint8_t af_h;
	uint8_t af_v;
	uint8_t weight[33*33];
} ISP_AF_WEIGHT_S;

typedef struct HB_ISP_AF_LENS_INFO_S {
	uint32_t pos;
	uint32_t range_low;
	uint32_t range_high;
} ISP_AF_LENS_INFO_S;

extern int HB_ISP_GetAfStatus(uint8_t pipeId, ISP_AF_STATUS_E *pstAfStatusAttr);
extern int HB_ISP_GetAfInfo(uint8_t pipeId, ISP_AF_LENS_INFO_S *ptrLenInfo);
extern int HB_ISP_SetAfManualPos(uint8_t pipeId, ISP_AF_MODE_E stAfModeAttr, uint32_t pos);
extern int HB_ISP_SetAfSpeed(uint8_t pipeId, uint32_t speed);

extern int HB_ISP_SetAeAttrEx(uint8_t pipeId, const ISP_AE_ATTR_EX_S *pstAeAttrEx);
extern int HB_ISP_GetAeAttrEx(uint8_t pipeId, ISP_AE_ATTR_EX_S *pstAeAttrEx);

typedef struct HB_AE_ZONES_WEIGHT_S {
	uint8_t au8Np[1089];
} AE_ZONES_WEIGHT_S;

extern int HB_ISP_GetAeWeight(uint8_t pipeId, AE_ZONES_WEIGHT_S *pstAeWeightLUT);
extern int HB_ISP_SetAeWeight(uint8_t pipeId, AE_ZONES_WEIGHT_S *pstAeWeightLUT);

extern int HB_ISP_GetAeMinIntertime(uint8_t pipeId, uint32_t *pstAeMinTime);
extern int HB_ISP_SetAeMinIntertime(uint8_t pipeId, uint32_t stAeMinTime);

#endif
