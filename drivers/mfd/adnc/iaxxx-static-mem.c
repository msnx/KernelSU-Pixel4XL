/*
 * iaxxx-static-mem.c -- Static memory blocks
 *
 * Copyright 2019 Knowles Corporation
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
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include "iaxxx.h"

/* The max available memory in the hardware */
#define IAXXX_MAX_AVAILABLE_MEMORY     (4 * 1024 * 1024)

/* counter to create unique id for each static block allocated */
static int static_mem_blk_unique_id;

static struct iaxxx_static_mem_blk *iaxxx_get_static_mem_blk_from_list(
		struct iaxxx_priv *priv,
		uint32_t static_mem_blk_id)
{
	struct list_head *node, *tmp;
	struct iaxxx_static_mem_blk *blk_data;

	if (list_empty_careful(&priv->static_mem_blks_list))
		return NULL;

	list_for_each_safe(node, tmp,
		&priv->static_mem_blks_list) {
		blk_data = list_entry(node,
				struct iaxxx_static_mem_blk, blk_node);
		if (blk_data->blk_id == static_mem_blk_id)
				return blk_data;
	}

	return NULL;
}

/* Create the static memory block of the
 * given size.
 * Returns handle to the block allocated.
 * If not, returns < 0
 */
int iaxxx_create_static_mem_blk(struct iaxxx_priv *priv,
		const uint32_t mem_blk_size)
{
	struct iaxxx_static_mem_blk *blk_data;

	blk_data = kvzalloc(sizeof(*blk_data), GFP_KERNEL);
	if (!blk_data) {
		dev_err(priv->dev, "blk_data alloc fail\n");
		return -ENOMEM;
	}

	blk_data->blk_mem  = kvzalloc(mem_blk_size, GFP_KERNEL);
	if (!blk_data->blk_mem) {
		kvfree(blk_data);
		return -ENOMEM;
	}
	blk_data->blk_id   = static_mem_blk_unique_id++;
	blk_data->blk_size = mem_blk_size;
	list_add_tail(&blk_data->blk_node, &priv->static_mem_blks_list);
	return blk_data->blk_id;
}

/* Allocate memory from static memory block
 * Returns address of memory
 * If not, returns NULL.
 */
void *iaxxx_alloc_from_static_mem_blk(struct iaxxx_priv *priv,
		const int static_mem_blk_id, const uint32_t size_to_allocate)
{
	struct iaxxx_static_mem_blk *blk_data;

	blk_data = iaxxx_get_static_mem_blk_from_list(priv, static_mem_blk_id);

	if (size_to_allocate > IAXXX_MAX_AVAILABLE_MEMORY) {
		dev_err(priv->dev, "static mem block requested > available\n");
		return NULL;
	}

	if (!blk_data)
		goto exit;

	if (size_to_allocate <= blk_data->blk_size)
		return blk_data->blk_mem;
	kvfree(blk_data->blk_mem);

	/* Since realloc can happen anytime, use kvmalloc */
	blk_data->blk_mem = kvzalloc(size_to_allocate, GFP_KERNEL);
	if (!blk_data->blk_mem) {
		list_del(&blk_data->blk_node);
		kvfree(blk_data);
		goto exit;
	}
	return blk_data->blk_mem;

exit:
	dev_err(priv->dev, "static memblk alloc fail\n");
	return NULL;
}

/* Destroy the static memory block allocated.
 */
void iaxxx_destroy_static_mem_blk(struct iaxxx_priv *priv,
		const int static_mem_blk_id)
{
	struct iaxxx_static_mem_blk *blk_data;

	blk_data = iaxxx_get_static_mem_blk_from_list(priv, static_mem_blk_id);

	if (blk_data) {
		kvfree(blk_data->blk_mem);
		list_del(&blk_data->blk_node);
		kvfree(blk_data);
	}
}
