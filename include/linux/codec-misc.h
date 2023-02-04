/* SPDX-License-Identifier: GPL-2.0 */

/*
 * codec-misc.h -- codec misc driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 */

#ifndef _CODEC_MISC_H
#define _CODEC_MISC_H

typedef int (*state_cb)(void);
typedef char* (*number_cb)(void);

bool codec_detect_available(void);
void codec_detect_state_callback(state_cb cb);
void codec_detect_hs_state_callback(state_cb cb);
void codec_detect_number_callback(number_cb cb);
int codec_detect_status_notifier(unsigned long val);
int codec_misc_amp_put(int ch, long val);

enum {
	CODEC_STATE_UNKNOWN = -99,
	CODEC_STATE_ONLINE = 0,
};

enum {
	WDSP_STAT_CRASH = 0,
	WDSP_STAT_DOWN,
	WDSP_STAT_UP,
};

#endif /* _CODEC_MISC_H */
