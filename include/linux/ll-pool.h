/*
 * Linked List Based Memory Allocator
 *
 * Copyright (C) 2019 Google, Inc.
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

#ifndef __LL_POOL_H__
#define __LL_POOL_H__

#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/scatterlist.h>
#include <linux/types.h>

/* Allocator with allocation granularity of page size
 * Note: this allocator may sleep, do not use in interrupt context
 */

/**
 * ll_pool: allocator instance object contains meta-data
 *
 * @lock: mutex lock to synchronize usage of the allocator
 * @free_list_head: list to all free memory blocks in order
 * @base_addr: starting address for the managed range
 * @remaining_size: remaining free bytes
 * @mem_block_slab: slab for allocating mem block struct
 * @allocation_slab: slab for allocating allocation struct
 */
struct ll_pool {
	struct mutex lock;
	struct list_head free_list_head;
	phys_addr_t base_addr;
	size_t remaining_size;

	struct kmem_cache *mem_block_slab;
	struct kmem_cache *allocation_slab;
};

/**
 * ll_pool_alloc(): Allocate from ll pool
 * @pool: allocator pool to allocate memory from
 * @len: length of memory requested in bytes
 * @contiguous: whether allocation is to be contiguous
 *
 * Use this function to allocate memory from the given memory pool.
 *
 * Return: pointer to the sg_table(scattergather list table) of the allocation.
 */
struct sg_table *ll_pool_alloc(struct ll_pool *pool, size_t len,
		bool contiguous);

/**
 * ll_pool_free(): Free an allocation from a given pool
 * @pool: allocator pool the allocation is from
 * @table: sg_table(scattergather list table) of the allocation to be freed
 *
 * Function takes 2 parameters: the pool and the table. After calling This
 * function the sg_table pointer becomes invalid for the user.
 *
 * Return: Function does not return anything.
 */
void ll_pool_free(struct ll_pool *pool, struct sg_table *table);

/**
 * ll_pool_create(): create a remote linked list based allocator pool
 * @base: base address that the pool manages
 * @size: size in bytes of the memory to be managed
 *
 * Create an allocator pool object.
 *
 * Return: pointer to the pool object
 */
struct ll_pool *ll_pool_create(phys_addr_t base, size_t size);

/**
 * ll_pool_destroy(): destroy a given remote linked list based
 * allocator pool
 * @pool: allocator pool to be destroyed
 *
 * Destroy the given remote linked list based allocator pool. If allocations
 * still exist, they will become invalid/unmanaged, and there will be a Memory
 * leak due to unreleased sg_tables.
 *
 * Return: Function does not return anything.
 */
void ll_pool_destroy(struct ll_pool *pool);

#endif /* __LL_POOL_H__ */
