/*************************************************************************
 *                     COPYRIGHT NOTICE
 *            Copyright 2019 Horizon Robotics, Inc.
 *                   All rights reserved.
 *************************************************************************/
#ifndef __HB_MEDIA_ERROR_H__
#define __HB_MEDIA_ERROR_H__

#include <errno.h>
#include <stddef.h>
#include "hb_media_basic_types.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* error handling */
#if EDOM > 0
/**
 * Returns a negative error code from a POSIX error code,
 * to return from library functions.
 */
#define HB_MEDIA_ERR(e) (-(e))

/**
 * Returns a POSIX error code from a library function error return value.
 */
#define HB_MEDIA_UNERR(e) (-(e))
#else
/**
 * Some platforms have E* and errno already negated.
 */
#define HB_MEDIA_ERR(e) (e)
#define HB_MEDIA_UNERR(e) (e)
#endif

/* Unknown Error */
#define HB_MEDIA_ERR_UNKNOWN                   0xF0000001
/* CODEC not found */
#define HB_MEDIA_ERR_CODEC_NOT_FOUND           0xF0000002
/* Failed to open codec device */
#define HB_MEDIA_ERR_CODEC_OPEN_FAIL           0xF0000003
/* Timeout to operate codec device */
#define HB_MEDIA_ERR_CODEC_RESPONSE_TIMEOUT    0xF0000004
/* Failed to initialize codec device */
#define HB_MEDIA_ERR_CODEC_INIT_FAIL           0xF0000005
/* Operation is not allowed */
#define HB_MEDIA_ERR_OPERATION_NOT_ALLOWED     0xF0000006
/* Insufficient resource */
#define HB_MEDIA_ERR_INSUFFICIENT_RES          0xF0000007
/* No free instance left */
#define HB_MEDIA_ERR_NO_FREE_INSTANCE          0xF0000008
/* Invalid parameters */
#define HB_MEDIA_ERR_INVALID_PARAMS            0xF0000009
/* Invalid instance */
#define HB_MEDIA_ERR_INVALID_INSTANCE          0xF000000A
/* Invalid buffer */
#define HB_MEDIA_ERR_INVALID_BUFFER            0xF000000B
/* Invalid command */
#define HB_MEDIA_ERR_INVALID_COMMAND           0xF000000C
/* Wait timeout */
#define HB_MEDIA_ERR_WAIT_TIMEOUT              0xF000000D
/* file cannot be operated successfully */
#define HB_MEDIA_ERR_FILE_OPERATION_FAILURE    0xF000000E
/* fail to set parameters */
#define HB_MEDIA_ERR_PARAMS_SET_FAILURE        0xF000000F
/* fail to get parameters */
#define HB_MEDIA_ERR_PARAMS_GET_FAILURE        0xF0000010
/* audio encoding/decoding failed */
#define HB_MEDIA_ERR_CODING_FAILED             0xF0000011
/* audio output buffer full*/
#define HB_MEDIA_ERR_OUTPUT_BUF_FULL           0xF0000012
/* Unsupported feature*/
#define HB_MEDIA_ERR_UNSUPPORTED_FEATURE       0xF0000013

#define HB_ERR_MAX_STRING_SIZE 64

/**
 * Query a description of the HB_MEDIA_ERR code.
 *
 * @param[in]  err_num      error code
 * @param[out] err_buf      buffer to which description is written
 * @param[in]  errbuf_size  the size in bytes of errbuf
 * @return 0 on success, a negative value if a description for errnum
 * cannot be found
 */
extern hb_s32 hb_mm_strerror(hb_s32 err_num, hb_string err_buf,
				size_t errbuf_size);

/**
 * Fill the provided buffer with a string containing an error string
 * corresponding to the HB_MEDIA_ERR code errnum.
 *
 * @param[in]  err_num         error code to describe
 * @param[out] err_buf         a buffer
 * @param[in]  errbuf_size     size in bytes of errbuf
 * @return the buffer in input, filled with the error description
 * @see hb_mm_strerror()
 */
static inline hb_string hb_mm_make_error_string(hb_s32 err_num,
						hb_string err_buf,
						size_t errbuf_size) {
	hb_mm_strerror(err_num, err_buf, errbuf_size);
	return err_buf;
}
/**
 * Convenience macro, the return value should be used only directly in
 * function arguments but never stand-alone.
 */
#define hb_mm_err2str(errnum) \
	hb_mm_make_error_string(errnum, (char[HB_ERR_MAX_STRING_SIZE]){0},\
	HB_ERR_MAX_STRING_SIZE)

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __HB_MEDIA_ERROR_H__ */
