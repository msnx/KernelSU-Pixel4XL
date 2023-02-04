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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

/* Currently this allocator only supports management of memory up to 2G
 * size.
 */
#define MAX_SIZE_SHIFT 32

#include <linux/list.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/ll-pool.h>
#include <linux/slab.h>
#include <linux/scatterlist.h>
#include <linux/types.h>

/**
 * ll_allocation: struct holding allocation information
 *
 * @nblks: number of blocks in this allocation
 * @size: total size allocated
 * @block_list: list of blocks that is allocated for this allocation
 * @table: scatter gather table for this allocation
 */
struct ll_allocation {
	size_t nblks;
	size_t size;
	struct list_head block_list;
	struct sg_table table;
};

struct ll_mem_blk {
	phys_addr_t offset;
	size_t size;

	struct list_head ll_head;
};

/* Caller must hold pool->lock */
static void ll_free_blk(struct ll_pool *pool,
		struct ll_mem_blk *blk)
{
	struct ll_mem_blk *fblk, *fblk_next, *fblk_prev;
	struct list_head *fl_head;

	fl_head = &pool->free_list_head;

	list_del(&blk->ll_head);

	/* Free list empty: add blk in */
	if (list_empty(fl_head)) {
		list_add_tail(&blk->ll_head, fl_head);
		return;
	}

	/* If blk to free is before first free block, insert and merge if
	 * possible
	 */
	fblk = list_first_entry(fl_head, struct ll_mem_blk, ll_head);
	if (fblk->offset > blk->offset) {
		if ((blk->offset + blk->size) == fblk->offset) {
			fblk->size += blk->size;
			fblk->offset -= blk->size;
			kmem_cache_free(pool->mem_block_slab, blk);
		} else {
			list_add(&blk->ll_head, fl_head);
		}

		return;
	}

	/* if blk should be inserted at the end of the list:
	 * This handles both general and singular list case
	 */
	fblk = list_last_entry(fl_head, struct ll_mem_blk, ll_head);
	if (fblk->offset < blk->offset) {
		/* merge block if possible */
		if ((fblk->offset + fblk->size) == blk->offset) {
			fblk->size += blk->size;
			kmem_cache_free(pool->mem_block_slab, blk);
		} else {
			list_add_tail(&blk->ll_head, fl_head);
		}

		return;
	}

	/* In all other case, the free blok is to be inserted at the middle
	 * of the list.
	 */
	fblk = list_first_entry(fl_head, struct ll_mem_blk, ll_head);
	fblk = list_next_entry(fblk, ll_head);

	list_for_each_entry_safe_from(fblk, fblk_next, fl_head, ll_head) {
		if (blk->offset > fblk->offset)
			continue;

		fblk_prev = list_prev_entry(fblk, ll_head);
		/* case 1: 3 block merge-able
		 * case 2: if can merge with following block only
		 * case 3: if can merge with previous block only
		 * case 4: cannot merge
		 */
		if (((blk->offset + blk->size) == fblk->offset) &&
				((fblk_prev->offset + fblk_prev->size) ==
				 blk->offset)) {
			fblk_prev->size += blk->size;
			fblk_prev->size += fblk->size;

			list_del_init(&fblk->ll_head);
			kmem_cache_free(pool->mem_block_slab, fblk);
			kmem_cache_free(pool->mem_block_slab, blk);
		} else if ((blk->offset + blk->size) == fblk->offset) {
			fblk->size += blk->size;
			fblk->offset -= blk->size;
			kmem_cache_free(pool->mem_block_slab, blk);
		} else if ((fblk_prev->offset + fblk_prev->size) ==
				blk->offset) {
			fblk_prev->size += blk->size;
			kmem_cache_free(pool->mem_block_slab, blk);
		} else {
			list_add(&blk->ll_head, &fblk_prev->ll_head);
		}

		return;
	}
}

/* Caller must hold pool->lock */
static void ll_free_allocation(struct ll_pool *pool,
		struct ll_allocation *allocation)
{
	struct ll_mem_blk *blk, *blk_next;

	/* For each allocation block, free and merge */
	list_for_each_entry_safe(blk, blk_next, &allocation->block_list,
			ll_head)
		ll_free_blk(pool, blk);

	/* Add freed memory size back to available size */
	pool->remaining_size += allocation->size;

	kmem_cache_free(pool->allocation_slab, allocation);
}

/* Caller must hold pool->lock */
static struct ll_allocation *ll_alloc_contiguous(
		struct ll_pool *pool, size_t len)
{
	struct ll_allocation *allocation;
	struct ll_mem_blk *blk, *blk_next, *new_blk;

	/* Creating allocation structure and initialize to contiguous */
	allocation = kmem_cache_alloc(pool->allocation_slab, GFP_KERNEL);
	if (!allocation)
		return ERR_PTR(-ENOMEM);

	allocation->nblks = 1;
	allocation->size = len;
	INIT_LIST_HEAD(&allocation->block_list);

	list_for_each_entry_safe(blk, blk_next, &pool->free_list_head,
			ll_head) {
		if (blk->size < len)
			continue;

		if (blk->size == len) {
			list_del_init(&blk->ll_head);
			list_add_tail(&blk->ll_head, &allocation->block_list);
		} else {
			new_blk = kmem_cache_alloc(pool->mem_block_slab,
					GFP_KERNEL);
			if (!new_blk)
				return ERR_PTR(-ENOMEM);

			new_blk->size = len;
			new_blk->offset = blk->offset;

			list_add_tail(&new_blk->ll_head,
					&allocation->block_list);

			blk->size -= len;
			blk->offset += len;
		}

		break;
	}

	/* If allocation failed return error */
	if (list_empty(&allocation->block_list)) {
		kmem_cache_free(pool->allocation_slab, allocation);
		return ERR_PTR(-ENOMEM);
	}

	pool->remaining_size -= len;
	return allocation;
}

/* Caller must hold pool->lock */
static struct ll_allocation *ll_alloc_noncontiguous(
		struct ll_pool *pool, size_t len)
{
	struct ll_allocation *allocation;
	struct ll_mem_blk *blk, *blk_next, *new_blk;

	/* Creating allocation structure and initialize to contiguous */
	allocation = kmem_cache_alloc(pool->allocation_slab, GFP_KERNEL);
	if (!allocation)
		return ERR_PTR(-ENOMEM);

	allocation->nblks = 0;
	allocation->size = len;
	INIT_LIST_HEAD(&allocation->block_list);

	list_for_each_entry_safe(blk, blk_next, &pool->free_list_head,
			ll_head) {
		if (blk->size <= len) {
			list_del(&blk->ll_head);
			list_add_tail(&blk->ll_head, &allocation->block_list);
			len -= blk->size;
			allocation->nblks++;
		} else {
			new_blk = kmem_cache_alloc(pool->mem_block_slab,
					GFP_KERNEL);
			if (!new_blk)
				return ERR_PTR(-ENOMEM);

			new_blk->size = len;
			new_blk->offset = blk->offset;
			INIT_LIST_HEAD(&new_blk->ll_head);

			list_add_tail(&new_blk->ll_head,
					&allocation->block_list);
			allocation->nblks++;

			blk->size -= len;
			blk->offset += len;

			len = 0;
		}

		if (len == 0)
			break;
	}

	if (len > 0) {
		ll_free_allocation(pool, allocation);
		pr_err("%s: possible inconsistent state or racing\n",
				__func__);
		return ERR_PTR(-ENOMEM);
	}

	pool->remaining_size -= allocation->size;
	return allocation;
}

static int ll_build_sgtable(struct ll_pool *pool,
		struct ll_allocation *allocation)
{
	struct ll_mem_blk *blk, *blk_next;
	struct scatterlist *sg;
	struct sg_table *table;
	int ret;

	if (unlikely(allocation->nblks == 0))
		return -EINVAL;

	table = &allocation->table;
	ret = sg_alloc_table(table, allocation->nblks, GFP_KERNEL);
	if (ret < 0)
		return ret;

	sg = table->sgl;
	list_for_each_entry_safe(blk, blk_next, &allocation->block_list,
			ll_head) {
		sg_dma_address(sg) = pool->base_addr + blk->offset;
		sg_dma_len(sg) = blk->size;
		sg->length = blk->size;

		sg = sg_next(sg);
	}

	return 0;
}

struct sg_table *ll_pool_alloc(struct ll_pool *pool, size_t len,
		bool contiguous)
{
	int ret;
	struct ll_allocation *alloc;

	if (!pool) {
		pr_err("%s: NULL ll pool\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	len = PAGE_ALIGN(len);

	mutex_lock(&pool->lock);
	if (len > pool->remaining_size || len == 0) {
		pr_err("%s: Unable to allocate %llu bytes\n", __func__,
				len);
		mutex_unlock(&pool->lock);
		return ERR_PTR(-ENOMEM);
	}

	if (contiguous)
		alloc = ll_alloc_contiguous(pool, len);
	else
		alloc = ll_alloc_noncontiguous(pool, len);

	mutex_unlock(&pool->lock);

	if (IS_ERR(alloc))
		return (void *)alloc;

	ret = ll_build_sgtable(pool, alloc);
	if (ret < 0) {
		mutex_lock(&pool->lock);
		ll_free_allocation(pool, alloc);
		mutex_unlock(&pool->lock);

		return ERR_PTR(ret);
	}

	return &alloc->table;
}

void ll_pool_free(struct ll_pool *pool, struct sg_table *table)
{
	struct ll_allocation *alloc;

	if (!table)
		return;

	alloc = container_of(table, struct ll_allocation, table);
	sg_free_table(table);

	mutex_lock(&pool->lock);
	ll_free_allocation(pool, alloc);
	mutex_unlock(&pool->lock);
}

struct ll_pool *ll_pool_create(phys_addr_t base, size_t size)
{
	int ret;
	struct ll_pool *pool;
	struct ll_mem_blk *blk;

	/* check if base is aligned with minimal order */
	if (!IS_ALIGNED(base, PAGE_SIZE)) {
		pr_err("%s: base address:0x%016lx is not page aligned\n",
				__func__, base);
		return ERR_PTR(-EINVAL);
	}

	/* check if size is aligned with minimal order and is power of 2 */
	if (!IS_ALIGNED(size, PAGE_SIZE)) {
		pr_err("%s: size: 0x%016lx is not page aligned and/or is not power of power of 2\n",
				__func__, size);
		return ERR_PTR(-EINVAL);
	}

	pool = kzalloc(sizeof(*pool), GFP_KERNEL);
	if (!pool)
		return ERR_PTR(-ENOMEM);

	pool->base_addr = base;
	pool->remaining_size = size;

	mutex_init(&pool->lock);
	INIT_LIST_HEAD(&pool->free_list_head);

	/* Creating slab for mem block object */
	pool->mem_block_slab = kmem_cache_create("ll_mem_block",
			sizeof(struct ll_mem_blk), 0,
			/* flag */ 0, /* ctor */ NULL);
	if (!pool->mem_block_slab) {
		pr_err("%s: creating mem block slab failed\n", __func__);
		ret = -ENOMEM;
		goto free_pool;
	}

	/* Creating slab for allocation object */
	pool->allocation_slab = kmem_cache_create("ll_allocation",
			sizeof(struct ll_allocation), 0,
			/* flag */ 0, /* ctor */ NULL);
	if (!pool->allocation_slab) {
		pr_err("%s: creating allocation object slab failed\n",
				__func__);
		ret = -ENOMEM;
		goto destroy_mem_block_slab;
	}

	/* Allocate the mem block that represent the full range */
	blk = kmem_cache_alloc(pool->mem_block_slab, GFP_KERNEL);
	if (!blk) {
		pr_err("%s: allocating mem block from slab failed\n",
				__func__);
		ret = -ENOMEM;
		goto destroy_allocation_slab;
	}

	blk->offset = 0;
	blk->size = size;

	list_add_tail(&blk->ll_head, &pool->free_list_head);

	return pool;

destroy_allocation_slab:
	kmem_cache_destroy(pool->allocation_slab);
destroy_mem_block_slab:
	kmem_cache_destroy(pool->mem_block_slab);
free_pool:
	kfree(pool);
	return ERR_PTR(ret);
}

static void ll_free_all_free_mem_blocks_struct(
		struct ll_pool *pool)
{
	struct ll_mem_blk *blk, *blk_next;

	list_for_each_entry_safe(blk, blk_next, &pool->free_list_head,
			ll_head)
		kmem_cache_free(pool->mem_block_slab, blk);
}

void ll_pool_destroy(struct ll_pool *pool)
{
	if (!pool)
		return;

	/* Warning if allocations still exist */
	WARN_ON(!list_is_singular(&pool->free_list_head));

	if (!list_empty(&pool->free_list_head))
		ll_free_all_free_mem_blocks_struct(pool);

	kmem_cache_destroy(pool->mem_block_slab);
	kmem_cache_destroy(pool->allocation_slab);
	kfree(pool);
}
