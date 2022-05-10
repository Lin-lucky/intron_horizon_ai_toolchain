/*
 * Horizon Robotics
 *
 *  Copyright (C) 2019 Horizon Robotics Inc.
 *  All rights reserved.
 *  Author: leye.wang<leye.wang@horizon.ai>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef INCLUDE_LOGGING_H_
#define INCLUDE_LOGGING_H_

#include <stdio.h>
#include <stdlib.h>
#define ALOG_SUPPORT

#ifdef ALOG_SUPPORT
#include <log.h>
#endif

#define STRINGIZE_NO_EXPANSION(x) #x
#define STRINGIZE(x) STRINGIZE_NO_EXPANSION(x)
#define HERE __FILE__ ":" STRINGIZE(__LINE__)

#ifndef SUBSYS_NAME
#define SUBSYS_NAME
#endif
#define SUBSYS STRINGIZE(SUBSYS_NAME)

#define L_INFO "[INFO][" SUBSYS "][" HERE "] "
#define L_WARNING "[WARNING]" SUBSYS "][" HERE "] "
#define L_ERROR "[ERROR][" SUBSYS "][" HERE "] "
#define L_DEBUG "[DEBUG][" SUBSYS "][" HERE "] "

/* output log by console */
#define CONSOLE_DEBUG_LEVEL		14
#define CONSOLE_INFO_LEVEL		13
#define CONSOLE_WARNING_LEVEL	12
#define CONSOLE_ERROR_LEVEL		11

/* output log by ALOG */
#ifdef ALOG_SUPPORT
#define ALOG_DEBUG_LEVEL		4
#define ALOG_INFO_LEVEL			3
#define ALOG_WARNING_LEVEL		2
#define ALOG_ERROR_LEVEL		1
#else
#define ALOG_DEBUG_LEVEL		CONSOLE_DEBUG_LEVEL
#define ALOG_INFO_LEVEL			CONSOLE_INFO_LEVEL
#define ALOG_WARNING_LEVEL		CONSOLE_WARNING_LEVEL
#define ALOG_ERROR_LEVEL		CONSOLE_ERROR_LEVEL
#endif

#ifndef pr_fmt
#define pr_fmt(fmt)		fmt
#endif

/* get log level from environment variable */
/* we use console debug level in default */
#define LOGLEVEL_ENV	"LOGLEVEL"

static inline int get_loglevel(void)
{
	char *loglevel_env = NULL;
	int loglevel_value = CONSOLE_ERROR_LEVEL;

	loglevel_env = getenv(LOGLEVEL_ENV);
	if (loglevel_env != NULL) {
		loglevel_value = atoi(loglevel_env);
		/* loglevel value should in the configuration area */
		if (((loglevel_value >= CONSOLE_ERROR_LEVEL) &&
					(loglevel_value <= CONSOLE_DEBUG_LEVEL)) ||
				((loglevel_value >= ALOG_ERROR_LEVEL) &&
				 (loglevel_value <= ALOG_DEBUG_LEVEL)))
			return loglevel_value;
	}

	/* default log level */
	loglevel_value = CONSOLE_ERROR_LEVEL;

	return loglevel_value;
}

/* pr_info defintion */
#ifdef ALOG_SUPPORT
#define pr_info(fmt, ...)				\
	do {								\
			int loglevel = get_loglevel();		\
			if (loglevel >= CONSOLE_INFO_LEVEL)		\
				fprintf(stdout, L_INFO "" pr_fmt(fmt), ##__VA_ARGS__);		\
			else if (loglevel >= ALOG_INFO_LEVEL && loglevel <= ALOG_DEBUG_LEVEL)	\
				ALOGI(fmt, ##__VA_ARGS__);		\
	} while (0);
#else
#define pr_info(fmt, ...)				\
	do {								\
			int loglevel = get_loglevel();		\
			if (loglevel >= CONSOLE_INFO_LEVEL)		\
				fprintf(stdout, L_INFO "" pr_fmt(fmt), ##__VA_ARGS__);		\
	} while (0);
#endif

/* pr_warn defintion */
#ifdef ALOG_SUPPORT
#define pr_warn(fmt, ...)				\
	do {								\
			int loglevel = get_loglevel();		\
			if (loglevel >= CONSOLE_WARNING_LEVEL)		\
				fprintf(stdout, L_WARNING "" pr_fmt(fmt), ##__VA_ARGS__);		\
			else if (loglevel >= ALOG_WARNING_LEVEL && loglevel <= ALOG_DEBUG_LEVEL)	\
				ALOGW(fmt, ##__VA_ARGS__);		\
	} while (0);
#else
#define pr_warn(fmt, ...)				\
	do {								\
			int loglevel = get_loglevel();		\
			if (loglevel >= CONSOLE_WARNING_LEVEL)		\
				fprintf(stdout, L_WARNING "" pr_fmt(fmt), ##__VA_ARGS__);		\
	} while (0);
#endif

/* pr_err defintion */
#ifdef ALOG_SUPPORT
#define pr_err(fmt, ...)				\
	do {								\
			int loglevel = get_loglevel();		\
			if (loglevel >= CONSOLE_ERROR_LEVEL)		\
				fprintf(stdout, L_ERROR "" pr_fmt(fmt), ##__VA_ARGS__);		\
			else if (loglevel >= ALOG_ERROR_LEVEL && loglevel <= ALOG_DEBUG_LEVEL)	\
				ALOGE(fmt, ##__VA_ARGS__);		\
	} while (0);
#else
#define pr_err(fmt, ...)				\
	do {								\
			int loglevel = get_loglevel();		\
			if (loglevel >= CONSOLE_ERROR_LEVEL)		\
				fprintf(stdout, L_ERROR "" pr_fmt(fmt), ##__VA_ARGS__);		\
	} while (0);
#endif

/* pr_debug defintion */
#ifdef ALOG_SUPPORT
#define pr_debug(fmt, ...)				\
	do {								\
			int loglevel = get_loglevel();		\
			if (loglevel >= CONSOLE_DEBUG_LEVEL)		\
				fprintf(stdout, L_DEBUG "" pr_fmt(fmt), ##__VA_ARGS__);		\
			else if (loglevel == ALOG_DEBUG_LEVEL)		\
				ALOGD(fmt, ##__VA_ARGS__);		\
	} while (0);
#else
#define pr_debug(fmt, ...)				\
	do {								\
			int loglevel = get_loglevel();		\
			if (loglevel >= CONSOLE_DEBUG_LEVEL)		\
				fprintf(stdout, L_DEBUG "" pr_fmt(fmt), ##__VA_ARGS__);		\
	} while (0);
#endif

#endif /* INCLUDE_LOGGING_H_ */
