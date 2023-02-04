/*
 * iaxxx-sysfs.h -- iaxxx sysfs attributes
 *
 * Copyright 2018 Knowles Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _IAXXX_SYSFS_H
#define _IAXXX_SYSFS_H

int iaxxx_init_sysfs(struct iaxxx_priv *priv);
void iaxxx_remove_sysfs(struct iaxxx_priv *priv);

#endif /* _IAXXX_SYSFS_H */
