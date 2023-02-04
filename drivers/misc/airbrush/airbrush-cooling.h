/* SPDX-License-Identifier: GPL-2.0
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
 */
#ifndef _AIRBRUSH_COOLING_H
#define _AIRBRUSH_COOLING_H

#define AIRBRUSH_COOLING_STATE_MAX 4

#include <linux/types.h>

struct ab_cooling;
struct device_node;

struct ab_cooling_ops {
	void (*state_updated)(const struct ab_cooling *cooling,
			unsigned long old_state, unsigned long new_state,
			void *cooling_op_data);
};

struct ab_cooling *ab_cooling_register(struct device_node *np, char *type,
		const struct ab_cooling_ops *ops, void *cooling_op_data,
		bool enable);
void ab_cooling_unregister(struct ab_cooling *cooling);

#endif // _AIRBRUSH_COOLING_H
