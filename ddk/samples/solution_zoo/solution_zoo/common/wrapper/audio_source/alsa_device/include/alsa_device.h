/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * alsa_device.h
 *	alsa device header file
 *
 * Copyright (C) 2019 Horizon Robotics, Inc.
 *
 * Contact: jianghe xu<jianghe.xu@horizon.ai>
 */

#ifndef _ALSA_DEVICE2_H_
#define _ALSA_DEVICE2_H_

#include <alsa/asoundlib.h>

typedef struct alsa_device {
	snd_pcm_t		*handle;		/* sound device handle */
	char			*name;			/* alsa device name (eg. default) */
	snd_pcm_format_t	format;			/* sample format */
	snd_pcm_stream_t	direct;			/* stream direction */
	unsigned int		rate;                   /* stream rate */
	unsigned int		channels;               /* count of channels */
	unsigned int		buffer_time;            /* ring buffer length in us */
	unsigned int		period_time;            /* period time in us */
	unsigned int		nperiods;               /* number of periods */
	int			mode;			/* SND_PCM_NONBLOCK, SND_PCM_ASYNC... */
	snd_pcm_uframes_t	period_size;		/* period_size, how many frames one period contains */
	snd_pcm_uframes_t	buffer_size;		/* buffer_size, totally alsa buffer. nperiods * period_size */
} alsa_device_t;

alsa_device_t *alsa_device_allocate(void);
int alsa_device_init(alsa_device_t *adev);
int alsa_device_read(alsa_device_t *adev, void *buffer,
		snd_pcm_uframes_t frames);
int alsa_device_write(alsa_device_t *adev, void *buffer,
		snd_pcm_uframes_t frames);
void alsa_device_deinit(alsa_device_t *adev);
void alsa_device_free(alsa_device_t *obj);

/* helper function */
void alsa_device_debug_enable(int enable);
#endif	/* _ALSA_DEVICE_H_ */
