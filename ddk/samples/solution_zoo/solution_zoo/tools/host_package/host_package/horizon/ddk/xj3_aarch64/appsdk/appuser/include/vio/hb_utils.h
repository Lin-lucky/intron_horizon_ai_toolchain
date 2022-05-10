/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef HB_X2A_VIO_HB_UTILS_H
#define HB_X2A_VIO_HB_UTILS_H

#include "cJSON.h"
#include <stdio.h>
#include <stdint.h>
#include <sys/time.h>
#include <semaphore.h>
#include <errno.h>
#include "hb_vio_common.h"
#include "logging.h"

#define HB_VIO_NAME_STR_SIZE (128)
#define HB_VIO_MAX_WEIGHT 3840
#define HB_VIO_MAX_height 2160
#define HB_VIO_MAX_VC_NUM 4

#define XJ3_VIO_DEBUG (1)

#define vio_err(format, ...)			\
do {									\
	char str[30];						\
	struct timeval ts;					\
	gettimeofday(&ts, NULL);			\
	snprintf(str, sizeof(str), "%ld.%06ld", ts.tv_sec, ts.tv_usec);			\
	pr_err("[%s]%s[%d]: "format"\n", str, __func__, __LINE__, ##__VA_ARGS__);\
} while (0)

#define vio_warn(format, ...)			\
do {									\
	char str[30];						\
	struct timeval ts;					\
	gettimeofday(&ts, NULL);			\
	snprintf(str, sizeof(str), "%ld.%06ld", ts.tv_sec, ts.tv_usec);			\
	pr_warn("[%s]%s[%d]: "format"\n", str, __func__, __LINE__, ##__VA_ARGS__);\
} while (0)

#define vio_log(format, ...)			\
do {									\
	char str[30];						\
	struct timeval ts;					\
	gettimeofday(&ts, NULL);			\
	snprintf(str, sizeof(str), "%ld.%06ld", ts.tv_sec, ts.tv_usec);			\
	pr_info("[%s]%s[%d]: "format"\n", str, __func__, __LINE__, ##__VA_ARGS__);\
} while (0)

#if XJ3_VIO_DEBUG
#define vio_dbg(format, ...)			\
do {									\
	char str[30];						\
	struct timeval ts;					\
	gettimeofday(&ts, NULL);			\
	snprintf(str, sizeof(str), "%ld.%06ld", ts.tv_sec, ts.tv_usec);			\
	pr_debug("[%s]%s[%d]: "format"\n", str, __func__, __LINE__, ##__VA_ARGS__);\
} while (0)
#else
#define vio_dbg(format, ...)
#endif

#define XJ3_CPU_SETSIZE 4
#define XJ3_CPU_0 0
#define XJ3_CPU_1 1
#define XJ3_CPU_2 2
#define XJ3_CPU_3 3

#define SIF_YUV_CONSEC "SIF_YUV_CONSECTIVE"
#define ISP_YUV_CONSEC "ISP_YUV_CONSECTIVE"
#define IPU_YUV_CONSEC "IPU_YUV_CONSECTIVE"
#define PYM_YUV_CONSEC "PYM_YUV_CONSECTIVE"
#define GDC_YUV_CONSEC "GDC_YUV_CONSECTIVE"

#define SECTOMS (1000)
#define SECTOUS (1000000)
#define SECTONS (1000000000)
#define MSTONS  (1000000)
#define MAX_SEM_WAIT_TIMEOUT    ((int64_t)(~0ULL>>1))

int set_thread_affinity(pthread_t pid, int cpu_num);
int get_thread_policy(pthread_attr_t *attr);
int set_thread_policy(pthread_attr_t *attr,  int policy);
void get_thread_priority_rang(int policy, int *min, int *max);
int get_thread_priority(pthread_attr_t *attr);
int set_thread_priority(pthread_attr_t *attr, int priority);


void time_add_ms(struct timeval *time, uint16_t ms);
int get_cost_time_ms(struct timeval *start, struct timeval *end);
int dumpToFile(char *filename, char *srcBuf, unsigned int size);
int dumpToFile2plane(char *filename, char *srcBuf, char *srcBuf1,
		     unsigned int size, unsigned int size1);
int dumpToFile2planeStride(char *filename, char *srcBuf, char *srcBuf1,
	unsigned int size, unsigned int size1, int width, int height, int stride);
int32_t timespec_add_ms(struct timespec *ts, int64_t msecs);
int32_t sem_timedwait_msecs(sem_t *sem, int64_t msecs);
int32_t sem_timedwait_relative(sem_t *sem, struct timespec *monotime);
#endif //HB_X2A_VIO_HB_UTILS_H
