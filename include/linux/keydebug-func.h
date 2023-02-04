/*
 * include/linux/keydebug-func.h - function and structure for debug
 * functions used by keydebug driver
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

#ifndef _LINUX_KEYDEBUG_FUNC_H
#define _LINUX_KEYDEBUG_FUNC_H

void kernel_top_monitor(void);
void kernel_top_init(void);
void kernel_top_exit(void);

void keydebug_showallcpus(void);

#endif /* _LINUX_KEYDEBUG_FUNC_H */
