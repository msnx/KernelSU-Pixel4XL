/*
 * iaxxx-static-mem.h -- IAxxx Static (allocated at boot-time) memory handling
 *
 * Copyright 2019 Knowles Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */
#ifndef __IAXXX_STATIC_MEM_H__
#define __IAXXX_STATIC_MEM_H__
int iaxxx_create_static_mem_blk(struct iaxxx_priv *priv,
		const uint32_t mem_blk_size);
void *iaxxx_alloc_from_static_mem_blk(struct iaxxx_priv *priv,
		const int static_mem_blk, const uint32_t size_to_allocate);
void iaxxx_destroy_static_mem_blk(struct iaxxx_priv *priv,
		int static_mem_blk);
#endif /* __IAXXX_STATIC_MEM_H__ */
