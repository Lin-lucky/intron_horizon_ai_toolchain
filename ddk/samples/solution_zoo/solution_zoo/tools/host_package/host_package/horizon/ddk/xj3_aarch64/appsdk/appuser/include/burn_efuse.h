/*************************************************************************
 *                     COPYRIGHT NOTICE
 *            Copyright 2016-2022 Horizon Robotics, Inc.
 *                   All rights reserved.
 *************************************************************************/
#ifndef LIBEFUSE_INC_BURN_EFUSE_H_
#define LIBEFUSE_INC_BURN_EFUSE_H_
#include <asm/ioctl.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <log.h>

#define STRINGIZE_NO_EXPANSION(x) #x
#define STRINGIZE(x) STRINGIZE_NO_EXPANSION(x)
#define HERE __FILE__ ":" STRINGIZE(__LINE__)
#define L_INFO "[EFUSE_INFO][" HERE "] "
#define L_WARNING "[EFUSE_WARNING]" HERE "] "
#define L_ERROR "[EFUSE_ERROR][" HERE "] "
#define L_DEBUG "[EFUSE_DEBUG][" HERE "] "
/*
 * bank: which bank to write or read
 * bank_value: bank value
 * lock: bank lock value
 */
struct io_efuse_data {
        uint32_t bank;
        uint32_t bank_value;
        bool     lock;
};

struct efuse_debug_bit {
	bool dis_bif_spi;
	bool dis_bif_sd;
	bool dis_jtag;
	bool verify_uboot;
	bool verify_bpu;
	bool lock;
};

enum efuse_debug {
	BIF_SPI = 0,
	BIF_SD,
	JTAG,
	VERIFY_UBOOT,
	VERIFY_BPU,
	DEBUG_ALL
};

static inline int32_t check_debug_level(void)
{
        static int32_t debug_flag = 0;
        const char *dbg_env;
        int32_t ret;

        if (debug_flag >= 0) {
                return debug_flag;
        } else {
                dbg_env = getenv("EFUSE_DEBUG_FLAG");
                if (dbg_env != NULL) {
                        ret = atoi(dbg_env);
                        if (ret <= 0) {
                                debug_flag = 0;
                        } else {
                                debug_flag = ret;
                        }
                } else {
                        debug_flag = 0;
                }
        }

        return debug_flag;
}

#ifndef pr_fmt
#define pr_fmt(fmt)             fmt
#endif

#define EFUSE_LOGERR(fmt, ...)                               \
        do {                                                            \
                fprintf(stderr, L_ERROR "" pr_fmt(fmt), ##__VA_ARGS__);          \
                ALOGE(L_ERROR "" fmt, ##__VA_ARGS__); \
        } while (0);
#define EFUSE_LOGINFO(fmt, ...)                               \
        do {                                                            \
                fprintf(stdout, L_INFO "" pr_fmt(fmt), ##__VA_ARGS__);          \
                ALOGI(L_INFO "" fmt, ##__VA_ARGS__); \
        } while (0);
#define EFSUE_LOGDEBUG(fmt, ...)                              \
        do {                                                            \
                int loglevel = check_debug_level();          \
                if (loglevel > 0) {            \
                        fprintf(stdout, L_DEBUG "" pr_fmt(fmt), ##__VA_ARGS__);         \
		}                              \
                ALOGD(L_DEBUG "" fmt, ##__VA_ARGS__);              \
        } while (0);

/*
 * description: set efuse bank
 * input:efsue_data,data to efuse
 * return: 0--success, others-failed
 */
int32_t set_efuse_bank(struct io_efuse_data *efuse_data);
/*
 * description: get efuse bank
 * input: bank, get bank value, range is 7~11
 * return: greater or equel 0 --success, others-failed
 */
int32_t get_efuse_bank_value(uint32_t bank);

/*
 * description: get customer id
 * input:none
 * return: negative value--failed, others--customer id
 */
int32_t get_customer_id(void);

/*
 * description: get product id
 * input:none
 * return: negative value--failed, others--product id
 */
int32_t get_product_id(void);

/*
 * description: get debug bit in efsue
 * input:efuse debug,which debug to get
 * return: positvie value--success, othres--failed
 */
int32_t get_debug_efuse(enum efuse_debug);

/*
 * description: lock bank
 * input:bank, set to lock this bank
 * return: 0--success, other value--failed
 */
int32_t set_efuse_lock(uint32_t bank);

/*
 * description: get bank lock status
 * input:bank, bank to get lock status
 * return: 0--unlock 1--lock, other value--failed
 */
int32_t get_bank_lock(uint32_t bank);

#endif  // LIBEFUSE_INC_BURN_EFUSE_H_
