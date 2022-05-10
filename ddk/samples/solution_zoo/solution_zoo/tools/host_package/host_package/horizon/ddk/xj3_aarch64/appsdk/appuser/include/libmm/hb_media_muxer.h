/*************************************************************************
 *                     COPYRIGHT NOTICE
 *            Copyright 2019 Horizon Robotics, Inc.
 *                   All rights reserved.
 *************************************************************************/
#ifndef __HB_MEDIA_MUXER_H__
#define __HB_MEDIA_MUXER_H__

#include "hb_media_basic_types.h"
#include "hb_media_codec.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * Define the media muxer state.
 **/
typedef enum _media_muxer_state {
    MEDIA_MUXER_STATE_NONE = -1,
    MEDIA_MUXER_STATE_UNINITIALIZED,
    MEDIA_MUXER_STATE_INITIALIZED,
    MEDIA_MUXER_STATE_STARTED,
    MEDIA_MUXER_STATE_ERROR,
    MEDIA_MUXER_STATE_TOTAL
} media_muxer_state_t;

/**
 * Define the output format of media muxer.
 **/
typedef enum _mx_output_format {
    /* Media muxer chooses the default media format "MP4" */
    MEDIA_MUXER_OUTPUT_FORMAT_DEFAULT = 0,
    MEDIA_MUXER_OUTPUT_FORMAT_MP4,
    MEDIA_MUXER_OUTPUT_FORMAT_TOTAL,
} mx_output_format_t;

/**
 * Define the input format of audio stream.
 **/
typedef struct _mx_audio_stream_input_params {
    /**
     * The average bitrate
     * Values[>0]bps
     *
     * - Note: It's unchangable parameters in the same sequence.
     * - Default: 0
     */
    hb_s64 bit_rate;

    /**
     * Audio sample format.
     * Values[@see mc_audio_sample_format_t]
     *
     * - Note: It's unchangable parameters in the same sequence.
     * - Default: MC_AV_SAMPLE_FMT_NONE
     */
    mc_audio_sample_format_t sample_fmt;

    /**
     * Audio sample rate.
     * Values[@see mc_audio_sample_rate_t]
     *
     * - Note: It's unchangable parameters in the same sequence.
     * - Default: MC_AV_SAMPLE_RATE_NONE
     */
    mc_audio_sample_rate_t sample_rate;

    /**
     * Audio channel layout.
     *
     * - Note: It's unchangable parameters in the same sequence.
     * - Default: MC_AV_CHANNEL_LAYOUT_NONE
     */
    mc_audio_channel_layout_t channel_layout;

    /**
     * Number of audio channels. It's related with channel layout.
     *
     * - Note: It's unchangable parameters in the same sequence.
     * - Default: 0
     */
    hb_s32 channels;
} mx_audio_stream_input_params_t;

/**
 * Define the input format of video stream.
 **/
typedef struct _mx_video_stream_input_params {
    /**
     * The target average bitrate of the encoded data in kbps.
     * Values[0,700000000]bps
     * - Note: 
     * - Default: 0
     */
    hb_u32 bit_rate;

    /**
     * The target frame rate of the encoded data in fps.
     * Values[0,240]fps
     *
     * - Note: It's unchangable parameter in the same sequence.
     * - Default: 0
     */
    hb_u32 frame_rate;

    /**
     * Output pixel format. TODO, remove this
     * !!!TODO confirm the available output format.
     * Values[MC_PIXEL_FORMAT_YUV420P,MC_PIXEL_FORMAT_NV12,MC_PIXEL_FORMAT_NV21]
     *
     * - Note: It's unchangable parameters in the same sequence.
     * - Default: MC_PIXEL_FORMAT_NONE
     */
    mc_pixel_format_t pix_fmt;

    /**
     * Specify the width and height of input video in luma samples.
     *
     * - Note: It's unchangable parameters in the same sequence.
     * - Default: 0
     */
    hb_s32 width, height;

    /*
     * I frame interval.
     * Values[0,2047]
     *
     * - Note: It's unchangable parameters in the same sequence.
     * - Default: 0
     */
    hb_s32 intra_period;
} mx_video_stream_input_params_t;

/**
 * Define the input format of subtitle stream.
 **/
typedef struct _mx_subtitle_stream_input_params {
	/* Reserverd */
} mx_subtitle_stream_input_params_t;

/**
 * Define the parameters of video/audio stream.
 **/
typedef struct _mx_stream_params {
    /**
     * Specify the codec type.
     * Valid vaules @see media_codec_id_t
     *
     * - Note: It's unchangable parameters in the same sequence.
     * - Encoding: Support.
     * - Decoding: Support.
     * - Default: MEDIA_CODEC_ID_NONE
     */
    media_codec_id_t codec_id;

    /**
     * PTS Time base numerator. Like ffmpeg AVRational rational number
     * (pair of numerator and denominator).
     * Values(0,]
     *
     * - Note:
     * - Default: 0
     */
    hb_s32 numerator;

    /**
     * PTS Time base denominator. Like ffmpeg AVRational rational number
     * (pair of numerator and denominator).
     * Values(0,]
     *
     * - Note:
     * - Default: 0
     */
    hb_s32 denominator;

    /**
     * The union member is decided according to the
     * @see mx_stream_params_t.codec_id.
     */
    union {
        mx_audio_stream_input_params_t audio_params;
        mx_video_stream_input_params_t video_params;
        mx_subtitle_stream_input_params_t subtitle_params;
    };
} mx_stream_params_t;

/**
 * Define the parameters of media muxer stream.
 **/
typedef struct _mx_stream {
    /**
     * Specify the stream type.
     * The valid numbers are as follows.
     *    0 : video stream
     *    1 : audio stream
     *    2 : subtitle stream
     *
     * - Note:
     * - Default: 0
     */
    hb_s32 is_audio;

    /**
     * Buffer virtual address.
     *
     * - Note:
     * - Default: 0
     */
    hb_u8 *vir_ptr;

    /**
     * Buffer physical address.
     *
     * - Note:
     * - Default: 0
     */
    hb_u64 phy_ptr;

    /**
     * Buffer size.
     *
     * - Note:
     * - Default: 0
     */
    hb_u32 size;

    /**
     * Frame pts
     *
     * - Note:
     * - Default: 0
     */
    hb_u64 pts;

    /**
     * IDR Frame
     * The valid numbers are as follows.
     *    0 : not IDR frame
     *    1 : IDR frame
     *
     * - Note:
     * - Default: 0
     */
    hb_bool is_key_frame;
} mx_stream_t;

/**
 * Define the parameters of media muxer context.
 **/
typedef struct _media_muxer_context {
    /**
     * Specify the output data path. For example, data path should be
     * "/data/outputFile.mp4"
     *
     * - Note: It's unchangable parameters in the same sequence.
     * - Default: NULL
     */
    hb_string output_file_name;

    /**
     * Specify the format of the output file.
     * The valid values @see mx_output_format_t.
     *
     * - Note: It's unchangable parameters in the same sequence.
     * - Default: MEDIA_MUXER_OUTPUT_FORMAT_DEFAULT
     */
    mx_output_format_t output_format;

    /**
     * Private data. Users must not modify this value!!!
     * Values[0,31]
     * - Note: 
     * - Default: -1
     */
    hb_s32 instance_index;
} media_muxer_context_t;

/**
 * Get the default media muxer context.
 *
 * @param[out]       muxer context
 *
 * @return >=0 on success, negative HB_MEDIA_ERROR in case of failure
 * @see media_muxer_context_t
 */
extern hb_s32 hb_mm_mx_get_default_context(media_muxer_context_t
				*context);

/**
 * Initialize the media muxer. If success, MediaMuxer will
 * enter into MEDIA_MUXER_STATE_INITIALIZED state. And it's invalid to 
 * reinitialize the media muxer if the MediaMuxer's state isn't
 * MEDIA_MUXER_STATE_UNINITIALIZED.
 *
 * @param[in]       muxer context
 *
 * @return >=0 on success, negative HB_MEDIA_ERROR in case of failure
 * @see media_muxer_context_t
 */
extern hb_s32 hb_mm_mx_initialize(media_muxer_context_t *context);

/**
 * Add audio/video stream into the muxer.
 *
 * @param[in]       muxer context
 * @param[in]       stream parameters @see mx_stream_params_t
 *
 * @return >=0 on success, negative HB_MEDIA_ERROR in case of failure
 * @see media_muxer_context_t
 * @see mx_stream_params_t
 */
extern hb_s32 hb_mm_mx_add_stream(media_muxer_context_t *context,
				const mx_stream_params_t *params);

/**
 * Start the media muxer.
 *
 * @param[in]       muxer context
 * @param[in]       stream parameters @see mx_stream_params_t
 *
 * @return >=0 on success, negative HB_MEDIA_ERROR in case of failure
 * @see media_muxer_context_t
 * @see mx_stream_params_t
 */
extern hb_s32 hb_mm_mx_start(media_muxer_context_t *context);

/**
 * Stop the media muxer. If success, MediaMuxer will enter into
 * MEDIA_MUXER_STATE_UNINITIALIZED state.
 *
 * @param[in]       muxer context
 *
 * @return >=0 on success, negative HB_MEDIA_ERROR in case of failure
 * @see media_muxer_context_t
 */
extern hb_s32 hb_mm_mx_stop(media_muxer_context_t *context);

/**
 * Write the audio/video stream.
 *
 * @param[in]       muxer context
 * @param[in]       audio/video stream @see mx_stream_t
 *
 * @return >=0 on success, negative HB_MEDIA_ERROR in case of failure
 * @see media_muxer_context_t
 * @see mx_stream_t
 */
extern hb_s32 hb_mm_mx_write_stream(media_muxer_context_t *context,
				const mx_stream_t *buffer);

/**
 * Get the state of media muxer.
 *
 * @param[in]       muxer context
 * @param[out]      muxer state @see media_muxer_state_t
 *
 * @return >=0 on success, negative HB_MEDIA_ERROR in case of failure
 * @see media_muxer_context_t
 * @see media_muxer_state_t
 */
extern hb_s32 hb_mm_mx_get_state(media_muxer_context_t *context,
				media_muxer_state_t *state);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __HB_MEDIA_MUXER_H__ */
