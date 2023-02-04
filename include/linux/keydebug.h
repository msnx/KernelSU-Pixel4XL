/*
 * include/linux/keydebug.h - platform data structure for keydebug driver
 *
 * Copyright (C) 2018 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LINUX_KEYDEBUG_H
#define _LINUX_KEYDEBUG_H

#define KEYDEBUG_NAME "keydebug"

struct keydebug_platform_data {
	uint32_t key_down_delay;
	uint32_t dbg_fn_delay;
	uint32_t *keys_down; /* 0 terminated */
	struct platform_device *pdev_child;
	struct delayed_work delayed_work;
	bool keydebug_requested;
};

#endif /* _LINUX_KEYDEBUG_H */
