/*
 * iaxxx-dbgfs.h -- IAxxx Debug-fs support
 *
 * Copyright 2016 Knowles Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 */


#ifndef __IAXXX_DBGFS_H__
#define __IAXXX_DBGFS_H__

struct iaxxx_priv;

#ifdef CONFIG_DEBUG_FS
int iaxxx_dfs_add_regmap(struct device *dev,
			struct regmap *map, void **dfs_node);
int iaxxx_dfs_switch_regmap(struct device *dev,
			struct regmap *map, void *dfs_node);
int iaxxx_dfs_del_regmap(struct device *dev, struct regmap *map);
struct dentry *iaxxx_dfs_get_root(void);
#else
static inline int iaxxx_dfs_add_regmap(struct device *dev, struct regmap *map)
{
	return 0;
}
static inline int iaxxx_dfs_del_regmap(struct device *dev, struct regmap *map)
{
	return 0;
}
#endif

#endif /* __IAXXX_DBGFS_H__ */
