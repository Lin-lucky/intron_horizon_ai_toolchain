/*****************************************************************************
 * KEROS Interface driver
 *
 *
 * Copyright(C) CHIPSBRAIN GLOBAL CO., Ltd.
 * All rights reserved.
 *
 * File Name    : keros_interface.h
 * Author       : ARES HA
 *
 * Version      : V0.3
 * Date         : 2015.09.08
 * Description  : Keros Interface Header
 ****************************************************************************/
#ifndef __KEROS_INTERFACE_H__
#define __KEROS_INTERFACE_H__

#include "keros_lib_1_8v.h"
#include "keros_lib_3_3v.h"

int keros_interface_i2c_init(char * dev_name, uint8_t dev_addr);
void keros_interface_i2c_exit(void);
uint8_t keros_read_data(uint16_t sub_addr, int read_len, uint8_t *r_data);
uint8_t keros_write_data(uint16_t sub_addr, uint8_t *w_data, int write_len);
void keros_delay(uint32_t wait_time);
uint8_t keros_power_on(void);

#endif /* __KEROS_INTERFACE_H_ */
