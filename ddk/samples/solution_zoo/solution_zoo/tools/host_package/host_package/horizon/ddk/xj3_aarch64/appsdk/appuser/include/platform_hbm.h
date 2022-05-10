/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *  Author: ming.yu<ming.yu@horizon.ai>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef LIBLOAD_HBM_INC_PLATFORM_HBM_H_
#define LIBLOAD_HBM_INC_PLATFORM_HBM_H_
#include <stdint.h>

/* ============== define definition ============== */
#ifndef bpu_addr_t
#define bpu_addr_t uint64_t
#endif
/* ============== struct definition ============== */

/*
 * @Description: hbm information
 * @fd: flash fd stored hbm file
 * @offset: hbm file offset of fd
 * @inst_addr: instruct data in secure address;
 * @par_addr: parameter data in secure address;
 */
typedef struct platform_hbm_info {
        int32_t fd;
        uint32_t offset;
        bpu_addr_t inst_addr;
        bpu_addr_t par_addr;
} platform_hbm_info_t;

/* ============== interface definition ============== */

/*
 * @Description: crc flash check init
 * @platfor_hbm: platform hbm information
 * @hbm_name: hbm file name
 * @return: 0 on success, others value on failure
 */
int32_t platform_hbm_info_get(platform_hbm_info_t *platform_hbm,
			      char *hbm_name);

/*
 * @Description: get number of hbm file in flash
 * @return: negative number---failure, others value---number of hbm file
 */
int32_t platform_get_hbm_number();

/*
 * @Description: get hbm name stored in flash
 * @index: hbm index,MAX is (platform_get_hbm_number() - 1)
 * @hbm_name: hbm name buffer,malloc by caller
 * @return: NULL--failure, or hbm name, the same as parameter hbm_name
 */
char *platform_get_hbm_name(unsigned int index, char *hbm_name);
#endif  // LIBLOAD_HBM_INC_PLATFORM_HBM_H_
