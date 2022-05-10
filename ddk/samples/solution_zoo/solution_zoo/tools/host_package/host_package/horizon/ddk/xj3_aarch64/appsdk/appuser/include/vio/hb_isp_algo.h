#ifndef __HB_ISP_ALGO_H__
#define __HB_ISP_ALGO_H__

#include <stdint.h>

#define ISP_METERING_ZONES_AE5_S                          4
#define ISP_METERING_ZONES_AE5_V                          33
#define ISP_METERING_ZONES_AE5_H                          33

/*****************************************************************************************
 *				                    Algo Structure Definition          			 	 	 *
 *****************************************************************************************/
typedef struct _modulation_entry_t {
    uint16_t x, y;
} modulation_entry_t;

/* AE */
typedef struct _ae_balanced_param_t {
    uint32_t pi_coeff;
    uint32_t target_point;
    uint32_t tail_weight;
    uint32_t long_clip;
    uint32_t er_avg_coeff;
    uint32_t hi_target_prc;
    uint32_t hi_target_p;
    uint32_t enable_iridix_gdg;
    uint32_t AE_tol;
} ae_balanced_param_t;

typedef struct _ae_misc_info_ {
    int32_t sensor_exp_number;

    int32_t total_gain;
    int32_t max_exposure_log2;
    uint32_t global_max_exposure_ratio;

    uint32_t iridix_contrast;
    uint32_t global_exposure;
    uint8_t global_ae_compensation;
    uint8_t global_manual_exposure;
    uint8_t global_manual_exposure_ratio;
    uint8_t global_exposure_ratio;
} ae_misc_info_t;

typedef struct _ae_calibration_data_ {
    uint8_t *ae_corr_lut;
    uint32_t ae_corr_lut_len;

    uint32_t *ae_exp_corr_lut;
    uint32_t ae_exp_corr_lut_len;

    modulation_entry_t *ae_hdr_target;
    uint32_t ae_hdr_target_len;

    modulation_entry_t *ae_exp_ratio_adjustment;
    uint32_t ae_exp_ratio_adjustment_len;
} ae_calibration_data_t;

typedef struct _ae_1024bin_weight_ {
        uint32_t zones_size;
        uint8_t zones_weight[ISP_METERING_ZONES_AE5_V * ISP_METERING_ZONES_AE5_H];
} ae_1024bin_weight_t;

typedef struct _ae_5bin_info_ {
        uint32_t zones_size;
        uint16_t zones_v;
        uint16_t zones_h;
        uint16_t threshold0_1;
        uint16_t threshold1_2;
        uint16_t threshold3_4;
        uint16_t threshold4_5;
        uint16_t normal_bin0;
        uint16_t normal_bin1;
        uint16_t normal_bin3;
        uint16_t normal_bin4;
} ae_5bin_info_t;

typedef struct _ae_out_info_ {
        uint32_t line[4];
        uint32_t line_num;
        uint32_t sensor_again[4];
        uint32_t sensor_again_num;
        uint32_t sensor_dgain[4];
        uint32_t sensor_dgain_num;
        uint32_t isp_dgain;
} ae_out_info_t;

typedef struct _ae_acamera_input_ {
    ae_balanced_param_t *ae_ctrl; 
    ae_misc_info_t misc_info;
    ae_calibration_data_t cali_data;
    ae_5bin_info_t ae_5bin_data;
    uint32_t ctx_id; 
} ae_acamera_input_t;

/* AWB */
typedef struct _awb_misc_info_ {
    uint16_t log2_gain;
    int cur_exposure_log2;
    uint32_t iridix_contrast;
    uint8_t global_manual_awb;
    uint16_t global_awb_red_gain;
    uint16_t global_awb_blue_gain;
} awb_misc_info_t;

typedef unsigned short ( *calibration_light_src_t )[2];
typedef struct _awb_calibration_data_ {
    calibration_light_src_t cali_light_src;
    uint32_t cali_light_src_len;

    uint32_t *cali_evtolux_ev_lut;
    uint32_t cali_evtolux_ev_lut_len;

    uint32_t *cali_evtolux_lux_lut;
    uint32_t cali_evtolux_lux_lut_len;

    uint8_t *cali_awb_avg_coef;
    uint32_t cali_awb_avg_coef_len;

    uint16_t *cali_rg_pos;
    uint32_t cali_rg_pos_len;

    uint16_t *cali_bg_pos;
    uint32_t cali_bg_pos_len;

    uint16_t *cali_color_temp;
    uint32_t cali_color_temp_len;

    uint16_t *cali_ct_rg_pos_calc;
    uint32_t cali_ct_rg_pos_calc_len;

    uint16_t *cali_ct_bg_pos_calc;
    uint32_t cali_ct_bg_pos_calc_len;

    modulation_entry_t *cali_awb_bg_max_gain;
    uint32_t cali_awb_bg_max_gain_len;

    uint16_t *cali_mesh_ls_weight;
    uint16_t *cali_mesh_rgbg_weight;
    uint8_t *cali_evtolux_probability_enable;
    uint32_t *cali_awb_mix_light_param;
    uint16_t *cali_ct65pos;
    uint16_t *cali_ct40pos;
    uint16_t *cali_ct30pos;
    uint16_t *cali_sky_lux_th;
    uint16_t *cali_wb_strength;
    uint16_t *cali_mesh_color_temperature;
    uint16_t *cali_awb_warming_ls_a;
    uint16_t *cali_awb_warming_ls_d75;
    uint16_t *cali_awb_warming_ls_d50;
    uint16_t *cali_awb_colour_preference;
} awb_calibration_data_t;

typedef struct _awb_acamera_input_ {
    awb_misc_info_t misc_info;
    awb_calibration_data_t cali_data;
} awb_acamera_input_t;

/* AF */
typedef struct _af_lms_param_t {
    uint32_t pos_min_down;
    uint32_t pos_min;
    uint32_t pos_min_up;
    uint32_t pos_inf_down;
    uint32_t pos_inf;
    uint32_t pos_inf_up;
    uint32_t pos_macro_down;
    uint32_t pos_macro;
    uint32_t pos_macro_up;
    uint32_t pos_max_down;
    uint32_t pos_max;
    uint32_t pos_max_up;
    uint32_t fast_search_positions;
    uint32_t skip_frames_init;
    uint32_t skip_frames_move;
    uint32_t dynamic_range_th;
    uint32_t spot_tolerance;
    uint32_t exit_th;
    uint32_t caf_trigger_th;
    uint32_t caf_stable_th;
    uint32_t print_debug;
} af_lms_param_t;

typedef struct _af_info_ {
    uint8_t af_mode;
    uint8_t refocus_required;
    uint8_t zones_horiz;
    uint8_t zones_vert;
    uint32_t roi;
    uint32_t af_pos_manual;
    uint32_t zoom_step_info;
} af_info_t;

typedef struct _af_misc_info_ {
    int16_t accel_angle;
    uint16_t lens_min_step;
} af_misc_info_t;

typedef struct _af_calibration_data_ {
    af_lms_param_t *af_param;
    uint16_t *af_zone_whgh_h;
    uint32_t af_zone_whgh_h_len;
    uint16_t *af_zone_whgh_v;
    uint32_t af_zone_whgh_v_len;
} af_calibration_data_t;

typedef struct _af_acamera_input_ {
    af_info_t af_info;
    af_misc_info_t misc_info;
    af_calibration_data_t cali_data;
} af_acamera_input_t;
#endif
