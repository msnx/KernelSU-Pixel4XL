/*
 * Android Airbrush coprocessor DMA library
 *
 * Copyright 2018 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/ab-dram.h>
#include <linux/airbrush-sm-notifier.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/dma-direction.h>
#include <linux/errno.h>
#include <linux/iommu.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/pagemap.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/time64.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/mfd/abc-pcie.h>
#include <linux/mfd/abc-pcie-dma.h>
#include <uapi/linux/abc-pcie-dma.h>
#include "abc-pcie-private.h"

#define UPPER(address) ((unsigned int)((address & 0xFFFFFFFF00000000) >> 32))
#define LOWER(address) ((unsigned int)(address & 0x00000000FFFFFFFF))
/* TODO(isca): b/120289070 */
#define DMA_CHAN	1
#define DMA_CHANS_PER_READ_XFER 3
#define DMA_CHANS_PER_WRITE_XFER 1

/* ((size / 500 bytes to us) + setup time ) * timeout buffer multiplier
 * + 500 ms buffer
 */
#define SIZE_TO_US_TIMEOUT(sz) ((sz / 500 + 200) * 10 + 500000)
static struct abc_pcie_dma abc_dma;

static DEFINE_SPINLOCK(dma_spinlock);

/* pending_[\w\_]+_q: List of all pending transactions (all sessions).
 * First element in the pending_q is always the currently executing
 * transfer.
 */
static struct list_head pending_to_dev_q;
static struct list_head pending_from_dev_q;

/* Global session used for "sessionless" kernel calls */
static struct abc_pcie_dma_session global_session;

/* Global cache for waiter objects */
static struct kmem_cache *waiter_cache;

static void abc_pcie_exec_dma_xfer(struct abc_dma_xfer *xfer);
int abc_pcie_sg_release_from_dma_buf(struct abc_pcie_sg_list *sgl);

/* TODO: extend to include multiple concurrent dma channels */
static int dma_callback(uint8_t chan, enum dma_data_direction dir,
			enum abc_dma_trans_status status)
{
	struct abc_dma_xfer *xfer;
	struct abc_dma_wait_info *wait_info;
	struct list_head *pending_q;

	dev_dbg(&abc_dma.pdev->dev,
		"%s: DMA callback DIR(%d), status(%d)\n",
		__func__, (int)dir, (int)chan);

	spin_lock(&dma_spinlock);
	pending_q = (dir == DMA_TO_DEVICE) ? &pending_to_dev_q :
						&pending_from_dev_q;
	/* Update state on finished transfer, dequeue from pending list */
	xfer = list_first_entry_or_null(pending_q, struct abc_dma_xfer,
					list_pending);
	if (WARN_ON(!xfer)) {
		dev_err(&abc_dma.pdev->dev, "%s: Got interrupt but no xfer!\n",
			__func__);
		spin_unlock(&dma_spinlock);
		return 0;
	}

	wait_info = xfer->wait_info;
	if (status == DMA_ABORT)
		wait_info->error = -EIO;

	if (xfer->transfer_method == DMA_TRANSFER_USING_MBLOCK) {
		if (WARN_ON(!(xfer->mblk_desc->channel_mask & 1 << chan))) {
			dev_err(&abc_dma.pdev->dev,
				"%s: Got interrupt on an unexpected channel!\n",
				__func__);
		}
		xfer->mblk_desc->channel_mask &= ~(1 << chan);
		if (xfer->mblk_desc->channel_mask) {
			spin_unlock(&dma_spinlock);
			return 0;
		}
	}

	complete_all(&wait_info->done);

	xfer->pending = false;
	list_del(&xfer->list_pending);

	/* Look for the next transfer to execute */
	xfer = list_first_entry_or_null(pending_q, struct abc_dma_xfer,
					list_pending);

	/* Note that the assumption here is abc_pcie_exec_dma_xfer never returns
	 * error. The validation checks done by pcie_exec_dma_xfer and its
	 * child functions are already done by the create function. If this
	 * assumption is no longer true then we must iterate thru the pending
	 * queue until empty or we have a good transaction to execute or risk
	 * not clearing the transactions in the pending queue.
	 */
	if (xfer)
		abc_pcie_exec_dma_xfer(xfer);
	spin_unlock(&dma_spinlock);
	return 0;
}

static void poison_all_transfers(void)
{
	struct abc_dma_xfer *xfer;
	struct abc_pcie_dma_session *session;

	list_for_each_entry(session, &abc_dma.sessions, list_session) {
		list_for_each_entry(xfer, &session->transfers,
					list_transfers) {
			xfer->poisoned = true;
		}
	}
}

/**
 * handle_pending_q_stop:
 *  Returns head of pending_q or null
 *  Must be protected by dma_spinlock before calling.
 */
static struct abc_dma_xfer *handle_pending_q_stop(struct list_head *pending_q,
						bool wait_active_transfer)
{
	struct abc_dma_xfer *xfer, *nxt_xfer, *first_xfer;
	struct abc_dma_wait_info *wait_info;

	first_xfer = NULL;

	list_for_each_entry_safe(xfer, nxt_xfer, pending_q, list_pending) {
		if (!first_xfer) {
			first_xfer = xfer;
			if (wait_active_transfer)
				continue;
		}
		xfer->pending = false;
		list_del(&xfer->list_pending);
		wait_info = xfer->wait_info;
		wait_info->error = -ECANCELED;
		complete_all(&wait_info->done);
	}

	return first_xfer;
}

static void wait_transfer_on_state_transition(
					struct abc_pcie_dma_session *session,
					struct abc_dma_wait_info *wait_info)
{
	int error;
	uint32_t start_id;

	mutex_lock(&session->lock);
	wait_info->active_waiters++;
	mutex_unlock(&session->lock);
	abc_pcie_dma_do_wait(session, wait_info, -1, &error, &start_id);
}

/**
 * handle_dma_state_transition:
 *  This function is assumed to be called *only* when there is
 *  a transition change.
 *  This function sets the new dma_state.
 *  Caller must hold a semaphore.
 */
static void handle_dma_state_transition(bool pcie_link_up,
					enum abc_dma_dram_state_e dram_state,
					bool wait_active_transfer)
{
	unsigned long flags;
	struct abc_dma_xfer *first_from_dev, *first_to_dev;

	if (pcie_link_up && (dram_state == AB_DMA_DRAM_UP))
		goto set_dma_state;

	spin_lock_irqsave(&dma_spinlock, flags);
	/* Clear all pending queues except for first element */
	first_to_dev = handle_pending_q_stop(&pending_to_dev_q,
						wait_active_transfer);

	first_from_dev = handle_pending_q_stop(&pending_from_dev_q,
						wait_active_transfer);

	spin_unlock_irqrestore(&dma_spinlock, flags);

	if (dram_state == AB_DMA_DRAM_DOWN)
		poison_all_transfers();


	if (wait_active_transfer && first_to_dev)
		wait_transfer_on_state_transition(first_to_dev->session,
						first_to_dev->wait_info);

	if (wait_active_transfer && first_from_dev)
		wait_transfer_on_state_transition(first_from_dev->session,
						first_from_dev->wait_info);

set_dma_state:
	abc_dma.pcie_link_up = pcie_link_up;
	abc_dma.dram_state = dram_state;

	dev_dbg(&abc_dma.pdev->dev,
		"pcie_link:%s dram_state:%s\n",
		abc_dma.pcie_link_up ? "Up" : "Down",
		dram_state_str(abc_dma.dram_state));
}

static void dma_pcie_link_ops_pre_disable(void)
{
	dev_dbg(&abc_dma.pdev->dev, "Begin pre_disable sequence\n");

	down_write(&abc_dma.state_transition_rwsem);
	if (abc_dma.pcie_link_up)
		handle_dma_state_transition(/*pcie_link_up=*/false,
					    abc_dma.dram_state,
					    /*wait_active_transfer=*/true);
	up_write(&abc_dma.state_transition_rwsem);

	dev_dbg(&abc_dma.pdev->dev, "Done pre_disable sequence\n");
}

static void dma_pcie_link_ops_post_enable(void)
{
	dev_dbg(&abc_dma.pdev->dev, "Begin post_enable sequence\n");

	down_write(&abc_dma.state_transition_rwsem);
	if (!abc_dma.pcie_link_up)
		handle_dma_state_transition(/*pcie_link_up=*/true,
					    abc_dma.dram_state,
					    /*wait_active_transfer=*/false);
	up_write(&abc_dma.state_transition_rwsem);

	dev_dbg(&abc_dma.pdev->dev, "Done post_enable sequence\n");
}

static void dma_pcie_link_ops_link_error(void)
{
	dev_warn(&abc_dma.pdev->dev, "Begin link_error sequence\n");

	down_write(&abc_dma.state_transition_rwsem);
	if (abc_dma.pcie_link_up || (abc_dma.dram_state != AB_DMA_DRAM_DOWN))
		handle_dma_state_transition(/*pcie_link_up=*/false,
					    /*dram_state=*/AB_DMA_DRAM_DOWN,
					    /*wait_active_transfer=*/false);

	clear_bit(0, &abc_dma.link_poisoned);
	up_write(&abc_dma.state_transition_rwsem);

	dev_warn(&abc_dma.pdev->dev, "Done link_error sequence\n");
}

static int dma_dram_blocking_listener(struct notifier_block *nb,
					unsigned long action,
					void *data)
{
	struct ab_clk_notifier_data *clk_data =
					(struct ab_clk_notifier_data *)data;
	bool pcie_link_up = abc_dma.pcie_link_up;
	enum abc_dma_dram_state_e dram_state = abc_dma.dram_state;
	bool wait_active_transfer = false;

	down_write(&abc_dma.state_transition_rwsem);
	if (action & AB_DRAM_ABORT_RATE_CHANGE) {
		/* Note that dram_abort is considered fatal an the system
		 * will be brought down.
		 */
		pcie_link_up = false;
		dram_state = AB_DMA_DRAM_DOWN;
	} else if (action & AB_DRAM_DATA_PRE_OFF) {
		dram_state = AB_DMA_DRAM_DOWN;
		wait_active_transfer = true;
	} else if (action & AB_DRAM_PRE_RATE_CHANGE &&
			clk_data->new_rate == 0) {
		wait_active_transfer = true;
		dram_state = AB_DMA_DRAM_SUSPEND;
	} else if (action & AB_DRAM_POST_RATE_CHANGE &&
		clk_data->new_rate > 0) {
		dram_state = AB_DMA_DRAM_UP;
	} else if (action & AB_DRAM_PRE_RATE_CHANGE &&
			clk_data->new_rate > 0 && clk_data->old_rate > 0) {
			dev_warn(&abc_dma.pdev->dev,
				"Unsupported clk rate change\n");
		up_write(&abc_dma.state_transition_rwsem);
		return NOTIFY_DONE;
	}

	if ((pcie_link_up != abc_dma.pcie_link_up) ||
		(dram_state != abc_dma.dram_state))
		handle_dma_state_transition(pcie_link_up, dram_state,
						wait_active_transfer);
	up_write(&abc_dma.state_transition_rwsem);
	return NOTIFY_OK;
}

/**
 * abc_find_xfer:
 * Find a xfer based on id in a particular session.
 * IMPORTANT: Any user to this function must take a session lock.
 * TODO(alexperez): Consider using idr for faster lookups and given properties
 *                  of session list.
 */
struct abc_dma_xfer *abc_pcie_dma_find_xfer(
			struct abc_pcie_dma_session *session, uint64_t id)
{
	bool found = false;
	struct abc_dma_xfer *xfer;

	list_for_each_entry(xfer, &session->transfers, list_transfers) {
		if (xfer->id == id) {
			found = true;
			break;
		}
	}

	xfer = found ? xfer : NULL;

	return xfer;
}

static int abc_pcie_dma_alloc_ab_dram(size_t size,
		struct abc_pcie_dma_mblk_desc *mblk_desc)
{
	struct dma_buf *ab_dram_dmabuf;

	ab_dram_dmabuf = ab_dram_alloc_dma_buf_kernel(size);
	if (IS_ERR(ab_dram_dmabuf))
		return PTR_ERR(ab_dram_dmabuf);

	memset(&mblk_desc->mapping, 0, sizeof(mblk_desc->mapping));
	mblk_desc->ab_dram_dma_buf = ab_dram_dmabuf;

	mblk_desc->size = size;
	mblk_desc->dma_paddr = ab_dram_get_dma_buf_paddr(ab_dram_dmabuf);

	return 0;
}

/**
 * API to build scatterlist for CMA buffers.
 * @param[in] dir Direction of transfer
 * @param[in] size Total size of the transfer in bytes
 * @param[out] lbuf
 * @return POSIX error or zero if successful.
 */
int abc_pcie_local_cma_build(enum dma_data_direction dir,
				size_t size, struct abc_buf_desc *buf_desc)
{
	struct abc_pcie_sg_list *sgl = buf_desc->sgl;

	sgl->sc_list = kmalloc(sizeof(struct scatterlist), GFP_KERNEL);
	if (!sgl->sc_list)
		return -ENOMEM;

	sg_init_table(sgl->sc_list, 1);
	sg_set_page(sgl->sc_list, (struct page *)buf_desc->local_addr, size,
			/*offset=*/0);
	sg_dma_address(sgl->sc_list) = (dma_addr_t)buf_desc->local_addr;
#ifdef CONFIG_NEED_SG_DMA_LENGTH
	sg_dma_len(sgl->sc_list) = size;
#endif
	sgl->n_num = 1;

	return 0;
}

/**
 * Import dma_buf (from ION buffer)
 * @param[in]  fd       Handle of dma_buf passed from user
 * @param[out] sgl      pointer of Scatter gather list which has information of
 *			scatter gather list and num of its entries
 * @return 0            on SUCCESS, negative on failure
 */
static int abc_pcie_sg_import_dma_buf(int fd, struct abc_pcie_sg_list *sgl)
{
	int ret;

	sgl->dma_buf = dma_buf_get(fd);
	if (IS_ERR(sgl->dma_buf)) {
		ret = PTR_ERR(sgl->dma_buf);
		dev_err(&abc_dma.pdev->dev,
				"%s: failed to get dma_buf, err %d\n",
				__func__, ret);
		return ret;
	}

	sgl->attach = dma_buf_attach(sgl->dma_buf,
				     abc_dma.dma_dev);
	if (IS_ERR(sgl->attach)) {
		ret = PTR_ERR(sgl->attach);
		dev_err(&abc_dma.pdev->dev,
				"%s: failed to attach dma_buf, err %d\n",
				__func__, ret);
		goto err_put;
	}

	sgl->sg_table = dma_buf_map_attachment(sgl->attach, sgl->dir);
	if (IS_ERR(sgl->sg_table)) {
		ret = PTR_ERR(sgl->sg_table);
		dev_err(&abc_dma.pdev->dev,
				"%s: failed to map dma_buf, err %d\n",
				__func__, ret);
		goto err_detach;
	}

	return 0;

err_detach:
	dma_buf_detach(sgl->dma_buf, sgl->attach);
err_put:
	dma_buf_put(sgl->dma_buf);
	return ret;
}

/**
 * API to build a scatter-gather list for multi-block DMA transfer for a
 * dma_buf
 * @param[in] dir Direction of transfer
 * @param[in] size Total size of the transfer in bytes
 * @param[out] lbuf
 * @return POSIX error or zero if successful.
 */
int abc_pcie_sg_retrieve_from_dma_buf(enum dma_data_direction dir,
					size_t size,
					struct abc_buf_desc *buf_desc)
{
	int ret;
	int fd = buf_desc->fd;
	struct abc_pcie_sg_list *sgl = buf_desc->sgl;

	sgl->dir = dir;

	/* Retrieve sg_table from dma_buf framework */
	ret = abc_pcie_sg_import_dma_buf(fd, sgl);
	if (ret)
		return ret;

	/* Use sg_table->sgl as our sc_list */
	sgl->sc_list = sgl->sg_table->sgl;
	sgl->n_num = sgl->sg_table->nents;

	return 0;
}

int abc_pcie_sg_release_from_dma_buf(struct abc_pcie_sg_list *sgl)
{
	dma_buf_unmap_attachment(sgl->attach, sgl->sg_table, sgl->dir);
	sgl->sc_list = NULL;
	sgl->n_num = 0;
	dma_buf_detach(sgl->dma_buf, sgl->attach);
	dma_buf_put(sgl->dma_buf);
	return 0;
}

/**
 * Local function to retrieve pages from memory allocated by vmalloc
 * It increments page ref count by 1
 * TODO(b/114422444): AB DMA driver Clean-up & Improvements
 * @param[in] start Starting address of vmalloc memory
 * @param[in] nr_pages Number of pages
 * @param[in] write Not used
 * @param[out] pages Pointers to pages that retrived from vmalloc_to_page call
 * @return Number of pages retrived
 */
static int get_vmalloc_pages(void *addr, int nr_pages, int write,
		      struct page **pages)
{
	int i;

	for (i = 0; i < nr_pages; i++) {
		*(pages + i) = vmalloc_to_page(addr);
		get_page(*(pages + i));
		addr += PAGE_SIZE;
	}
	return nr_pages;
}

/**
 * API to build Scatter Gather list to do Multi-block DMA transfer for a buffer
 * allocated from vmalloc
 * TODO(b/114422444): AB DMA driver Clean-up & Improvements
 * @param[in] dir Direction of transfer
 * @param[in] size Total size of the transfer in bytes
 * @param[out] lbuf
 * @return POSIX error or zero if successful.
 */
static int abc_pcie_vmalloc_buf_sg_build(enum dma_data_direction dir,
					size_t size,
					struct abc_buf_desc *buf_desc)
{
	int i, fp_offset, count;
	int n_num, p_num;
	int first_page, last_page;
	void *dmadest = buf_desc->local_addr;
	struct abc_pcie_sg_list *sgl = buf_desc->sgl;

	sgl->dir = dir;

	/* page num calculation */
	first_page = ((dma_addr_t) dmadest & PAGE_MASK) >> PAGE_SHIFT;
	last_page = (((dma_addr_t) dmadest + size - 1) & PAGE_MASK)
		>> PAGE_SHIFT;
	fp_offset = (dma_addr_t) dmadest & ~PAGE_MASK;
	p_num = last_page - first_page + 1;

	sgl->mypage = kvzalloc(p_num * sizeof(struct page *), GFP_KERNEL);
	if (!sgl->mypage)
		goto free_sg;
	sgl->sc_list = kvmalloc(p_num * sizeof(struct scatterlist), GFP_KERNEL);
	if (!sgl->sc_list)
		goto free_sg;

	n_num = get_vmalloc_pages(dmadest, p_num, 0, sgl->mypage);

	if (n_num < p_num) {
		dev_err(&abc_dma.pdev->dev,
			"%s: fail to get user_pages\n", __func__);
		goto release_page;
	}

	sg_init_table(sgl->sc_list, n_num);
	if (n_num == 1) {
		sg_set_page(sgl->sc_list, *(sgl->mypage), size, fp_offset);
	} else {
		sg_set_page(sgl->sc_list, *(sgl->mypage),
				PAGE_SIZE - fp_offset, fp_offset);

		for (i = 1; i < n_num - 1; i++) {
			sg_set_page(sgl->sc_list + i, *(sgl->mypage + i),
					PAGE_SIZE, 0);
		}

		sg_set_page(sgl->sc_list + n_num - 1,
				*(sgl->mypage + n_num - 1),
				size - (PAGE_SIZE - fp_offset)
					- ((n_num - 2) * PAGE_SIZE), 0);
	}

	count = dma_map_sg_attrs(abc_dma.dma_dev, sgl->sc_list, n_num, sgl->dir,
				   DMA_ATTR_SKIP_CPU_SYNC);
	if (count <= 0) {
		dev_err(&abc_dma.pdev->dev,
			"%s: failed to map scatterlist region (%d) n_num(%d)\n",
		       __func__, count, n_num);
		goto release_page;
	}

	sgl->n_num = n_num;

	return 0;

release_page:
	for (i = 0; i < n_num; i++)
		put_page(*(sgl->mypage + i));
free_sg:
	kvfree(sgl->mypage);
	sgl->mypage = NULL;
	kvfree(sgl->sc_list);
	sgl->sc_list = NULL;
	sgl->n_num = 0;

	return -EINVAL;
}

/**
 * API to build Scatter Gather list to do Multi-block DMA transfer for a user
 * local buffer
 * @param[in] dir Direction of transfer
 * @param[in] size Total size of the transfer in bytes
 * @param[out] lbuf
 * @return POSIX error or zero if successful.
 */
int abc_pcie_user_local_buf_sg_build(enum dma_data_direction dir,
					size_t size,
					struct abc_buf_desc *buf_desc)
{
	int i, fp_offset, count;
	int n_num, p_num;
	int first_page, last_page;
	void *dmadest = buf_desc->local_addr;
	struct abc_pcie_sg_list *sgl = buf_desc->sgl;

	sgl->dir = dir;

	/* page num calculation */
	first_page = ((unsigned long) dmadest & PAGE_MASK) >> PAGE_SHIFT;
	last_page = (((unsigned long) dmadest + size - 1) & PAGE_MASK)
		>> PAGE_SHIFT;
	fp_offset = (unsigned long) dmadest & ~PAGE_MASK;
	p_num = last_page - first_page + 1;

	sgl->mypage = kvzalloc(p_num * sizeof(struct page *), GFP_KERNEL);
	if (!sgl->mypage) {
		sgl->n_num = 0;
		return -EINVAL;
	}
	sgl->sc_list = kvmalloc(p_num * sizeof(struct scatterlist), GFP_KERNEL);
	if (!sgl->sc_list) {
		kvfree(sgl->mypage);
		sgl->mypage = NULL;
		sgl->n_num = 0;
		return -EINVAL;
	}

	down_read(&current->mm->mmap_sem);

	n_num = get_user_pages((unsigned long)dmadest, p_num,
			       FOLL_WRITE | FOLL_FORCE, sgl->mypage, NULL);

	up_read(&current->mm->mmap_sem);
	if (n_num < p_num) {
		dev_err(&abc_dma.pdev->dev,
			"%s: fail to get user_pages\n", __func__);
		goto release_page;
	}

	sg_init_table(sgl->sc_list, n_num);
	if (n_num == 1) {
		sg_set_page(sgl->sc_list, *(sgl->mypage), size, fp_offset);
	} else {
		sg_set_page(sgl->sc_list, *(sgl->mypage), PAGE_SIZE - fp_offset,
				fp_offset);
		for (i = 1; i < n_num-1; i++) {
			sg_set_page(sgl->sc_list + i, *(sgl->mypage + i),
					PAGE_SIZE, 0);
		}
		sg_set_page(sgl->sc_list + n_num-1, *(sgl->mypage + n_num-1),
				size - (PAGE_SIZE - fp_offset)
					- ((n_num-2)*PAGE_SIZE), 0);
	}

	count = dma_map_sg_attrs(abc_dma.dma_dev, sgl->sc_list, n_num, sgl->dir,
				   DMA_ATTR_SKIP_CPU_SYNC);
	if (count <= 0) {
		dev_err(&abc_dma.pdev->dev,
			"%s: failed to map scatterlist region (%d) n_num(%d)\n",
		       __func__, count, n_num);
		goto release_page;
	}

	sgl->n_num = n_num;

	return 0;

release_page:
	for (i = 0; i < n_num; i++)
		put_page(*(sgl->mypage + i));

	kvfree(sgl->mypage);
	sgl->mypage = NULL;
	kvfree(sgl->sc_list);
	sgl->sc_list = NULL;
	sgl->n_num = 0;

	return -EINVAL;
}

/**
 * API to release scatter gather list for a user local buffer
 * @param[in] *sgl pointer to the scatter gather list that was built during
 *		abc_pcie_user_local_buf_sg_build
 * @return 0 for SUCCESS
 */
int abc_pcie_user_local_buf_sg_destroy(struct abc_pcie_sg_list *sgl)
{
	int i;
	struct page *page;

	dma_unmap_sg_attrs(abc_dma.dma_dev, sgl->sc_list, sgl->n_num, sgl->dir,
				DMA_ATTR_SKIP_CPU_SYNC);
	for (i = 0; i < sgl->n_num; i++) {
		page = *(sgl->mypage + i);
		/* Mark page as dirty before releasing the pages. */
		if (!PageReserved(page))
			SetPageDirty(page);
		put_page(page);
	}
	kvfree(sgl->mypage);
	sgl->mypage = NULL;
	kvfree(sgl->sc_list);
	sgl->sc_list = NULL;
	sgl->n_num = 0;

	return 0;
}

/* Build abc_pcie_sg_entry structure for the remote buffer
 * Since buffers on ABC are contiguous the generated scatterlist
 * only has two entries, one with the starting address and one with
 * the terminator entry
 * TODO(alexperez):  Remove support for specifying an arbitrary
 * physical address once bringup is done.
 */
int abc_pcie_user_remote_buf_sg_build(enum dma_data_direction dir,
					size_t size,
					struct abc_buf_desc *buf_desc)
{
	struct abc_pcie_sg_list *sgl = buf_desc->sgl;

	sgl->sc_list = kvmalloc(sizeof(struct scatterlist), GFP_KERNEL);
	if (!sgl->sc_list)
		return -ENOMEM;

	sg_init_table(sgl->sc_list, 1);
	sg_set_page(sgl->sc_list, (struct page *)buf_desc->remote_addr, size,
			/*offset=*/0);
	sg_dma_address(sgl->sc_list) = buf_desc->remote_addr;
#ifdef CONFIG_NEED_SG_DMA_LENGTH
	sg_dma_len(sgl->sc_list) = size;
#endif
	sgl->n_num = 1;

	return 0;
}

/**
 * API to release scatter gather list for a user remote buffer
 * @param[in] **sg pointer to pointer to sg entry allocated during
 *		abc_pcie_user_remote_buf_sg_build
 * @return 0 for SUCCESS
 */
int abc_pcie_user_remote_buf_sg_destroy(struct abc_pcie_sg_list *sgl)
{
	kvfree(sgl->sc_list);

	return 0;
}

int abc_pcie_clean_dma_local_buffers(struct abc_dma_xfer *xfer)
{
	struct abc_pcie_sg_list *sgl;
	struct abc_pcie_dma_mblk_desc *mblk_desc = xfer->mblk_desc;
	struct dma_element_t *sblk_desc = xfer->sblk_desc;

	if (xfer->transfer_method == DMA_TRANSFER_USING_MBLOCK &&
		mblk_desc) {
		/**
		 * Note that this code assumes all the elements associated
		 * with the creation of mblk_desc are valid.
		 */
		ab_dram_free_dma_buf_kernel(mblk_desc->ab_dram_dma_buf);
		kfree(mblk_desc);
		xfer->mblk_desc = NULL;
	} else if (xfer->transfer_method == DMA_TRANSFER_USING_SBLOCK &&
		sblk_desc) {
		kfree(sblk_desc);
		xfer->sblk_desc = NULL;
	}


	sgl = xfer->local_buf.sgl;

	switch ((xfer->local_buf).buf_type) {
	case DMA_BUFFER_KIND_USER:
	case DMA_BUFFER_KIND_VMALLOC:
		abc_pcie_user_local_buf_sg_destroy(sgl);
		break;
	case DMA_BUFFER_KIND_DMA_BUF:
		abc_pcie_sg_release_from_dma_buf(sgl);
		break;
	case DMA_BUFFER_KIND_CMA:
		kvfree(sgl->sc_list);
		break;
	default:
		break;
	}

	kfree(sgl);

	xfer->local_buf.sgl = NULL;

	return 0;
}

int abc_pcie_clean_dma_remote_buffers(struct abc_dma_xfer *xfer)
{
	struct abc_pcie_sg_list *sgl;


	sgl = xfer->remote_buf.sgl;

	switch ((xfer->remote_buf).buf_type) {
	case DMA_BUFFER_KIND_USER:
		abc_pcie_user_remote_buf_sg_destroy(sgl);
		break;
	case DMA_BUFFER_KIND_DMA_BUF:
		abc_pcie_sg_release_from_dma_buf(sgl);
		break;
	default:
		break;
	}

	kfree(sgl);

	xfer->remote_buf.sgl = NULL;

	return 0;
}

static char *abc_pcie_dma_xfer_method_to_str(dma_xfer_method_t method)
{
	switch (method) {
	case DMA_TRANSFER_USING_PIO:
		return "PIO";
	case DMA_TRANSFER_USING_SBLOCK:
		return "SBLOCK";
	case DMA_TRANSFER_USING_MBLOCK:
		return "MBLOCK";
	default:
		return "";
	}
}

/**
 * Internal DMA API for cleaning a transfer and removing it from the internal,
 * data structures. De-allocates abc_dma_xfer passed to it.
 * @xfer[in] Data structure describing the DMA transfer including
 *               local and remote buffer descriptors & dma chan
 */
void abc_pcie_clean_dma_xfer_locked(struct abc_dma_xfer *xfer)
{
	unsigned long flags;
	struct abc_dma_xfer *head_xfer;
	struct abc_dma_wait_info *wait_info;
	struct list_head *pending_q;
	struct abc_pcie_sg_list *sgl;
	bool wait_for_transfer = false;

	pending_q = (xfer->dir == DMA_TO_DEVICE) ? &pending_to_dev_q :
						&pending_from_dev_q;

	wait_info = xfer->wait_info;

	/* Clear from pending list if it is in there */
	spin_lock_irqsave(&dma_spinlock, flags);

	head_xfer = list_first_entry_or_null(pending_q, struct abc_dma_xfer,
					list_pending);
	if (head_xfer == xfer)
		wait_for_transfer = true;
	else if (xfer->pending)
		list_del(&xfer->list_pending);

	spin_unlock_irqrestore(&dma_spinlock, flags);

	if (wait_for_transfer) {
		/* Wait for transfer to be done. Note that the interrupt
		 * handler will dequeue from pending list as well as
		 * re-start the new transaction.
		 * Check for timeout
		 */
		unsigned long remaining, timeout = usecs_to_jiffies(
			SIZE_TO_US_TIMEOUT(xfer->size));

		dev_dbg(&abc_dma.pdev->dev,
			"%s: Waiting for transfer to finish\n", __func__);

		remaining = wait_for_completion_timeout(
			&wait_info->done, timeout);
		if (WARN_ON(remaining == 0)) {
			dev_err(&abc_dma.pdev->dev,
				"%s: DMA Transfer timed out! id:%0llu, size:%lld, type:%s\n",
				__func__, xfer->id, xfer->size,
				abc_pcie_dma_xfer_method_to_str(
					xfer->transfer_method));
			set_bit(0, &abc_dma.link_poisoned);
			poison_all_transfers();
			ab_sm_report_fatal("PCIe DMA timeout");
			return;
		}
	}

	if (wait_info != NULL) {
		wait_info->xfer = NULL;
		/* Handle the case where there are no waiters */
		if (!wait_info->active_waiters) {
			kmem_cache_free((xfer->session)->waiter_cache,
					wait_info);
			xfer->wait_info = NULL;
		} else {
			complete_all(&wait_info->done);
		}
	}

	list_del(&xfer->list_transfers);

	if (!xfer->synced &&
		(xfer->local_buf).buf_type != DMA_BUFFER_KIND_DMA_BUF &&
		(xfer->local_buf).buf_type != DMA_BUFFER_KIND_CMA) {
		sgl = (xfer->local_buf).sgl;
		dma_sync_sg_for_cpu(abc_dma.dma_dev, sgl->sc_list, sgl->n_num,
					sgl->dir);
	}

	/* Release all buffer allocations */
	abc_pcie_clean_dma_local_buffers(xfer);
	abc_pcie_clean_dma_remote_buffers(xfer);
	dev_dbg(&abc_dma.pdev->dev, "%s: destroying xfer: id:%0llu\n",
		__func__, xfer->id);
	kfree(xfer);
}

/**
 * Kernel API for cleaning a transfer and removing it from the internal,
 * data structures. De-allocates abc_dma_xfer passed to it.
 * @xfer[in] Data structure describing the DMA transfer including
 *               local and remote buffer descriptors & dma chan
 */
void abc_pcie_clean_dma_xfer(struct abc_dma_xfer *xfer)
{
	struct abc_pcie_dma_session *session = xfer->session;

	down_read(&abc_dma.state_transition_rwsem);

	mutex_lock(&session->lock);
	abc_pcie_clean_dma_xfer_locked(xfer);
	mutex_unlock(&session->lock);

	up_read(&abc_dma.state_transition_rwsem);
}
EXPORT_SYMBOL(abc_pcie_clean_dma_xfer);

int abc_pcie_dma_get_kernel_wait(struct abc_dma_xfer *xfer,
					struct abc_pcie_dma_session **session,
					struct abc_dma_wait_info **wait_info)
{
	*wait_info = NULL;
	mutex_lock(&(xfer->session)->lock);
	if (xfer->wait_info == NULL) {
		mutex_unlock(&(xfer->session)->lock);
		dev_err(&abc_dma.pdev->dev, "%s: Wait info struct is NULL!\n",
			__func__);
		return -EINVAL;
	}
	(xfer->wait_info)->active_waiters++;
	*session = xfer->session;
	*wait_info = xfer->wait_info;
	mutex_unlock(&(xfer->session)->lock);
	return 0;
}

int abc_pcie_dma_get_user_wait(uint64_t id,
				struct abc_pcie_dma_session *session,
				struct abc_dma_wait_info **wait_info)
{
	int err = 0;
	struct abc_dma_xfer *xfer;
	*wait_info = NULL;

	mutex_lock(&session->lock);
	xfer = abc_pcie_dma_find_xfer(session, id);
	if (!xfer) {
		err = -EINVAL;
		dev_err(&abc_dma.pdev->dev, "%s: Could not find xfer:%0d\n",
			__func__, id);
		goto release_lock;
	}

	if (xfer->wait_info == NULL) {
		err = -EINVAL;
		dev_err(&abc_dma.pdev->dev, "%s: Wait info struct is NULL!\n",
			__func__);
		goto release_lock;
	}
	(xfer->wait_info)->active_waiters++;

	*wait_info = xfer->wait_info;
release_lock:
	mutex_unlock(&session->lock);
	return err;
}

int abc_pcie_dma_do_wait(struct abc_pcie_dma_session *session,
			struct abc_dma_wait_info *wait_info, int timeout,
			int *error,
			uint32_t *start_id)
{
	int err = 0;
	long remaining = 0;
	struct abc_dma_xfer *xfer;
	struct abc_pcie_sg_list *sgl;

	if (wait_info->xfer != NULL && timeout < 0)
		timeout = SIZE_TO_US_TIMEOUT(wait_info->xfer->size);

	remaining = wait_for_completion_interruptible_timeout(&wait_info->done,
					usecs_to_jiffies(timeout));

	mutex_lock(&session->lock);
	xfer = wait_info->xfer;
	*start_id = wait_info->start_id;
	dev_dbg(&abc_dma.pdev->dev,
		"%s: completed xfer: id:%0llu start_id:%0u timeout:%0d [%0d]\n",
		__func__, wait_info->xfer_id, wait_info->start_id, timeout,
		remaining);
	/* Note that DMA_BUF does its own sync, only needs to be done for
	 * malloc and vmalloc buffers
	 */
	if (xfer != NULL && !xfer->synced &&
		(xfer->local_buf).buf_type != DMA_BUFFER_KIND_DMA_BUF &&
		(xfer->local_buf).buf_type != DMA_BUFFER_KIND_CMA) {
		sgl = (xfer->local_buf).sgl;
		dma_sync_sg_for_cpu(abc_dma.dma_dev, sgl->sc_list, sgl->n_num,
					sgl->dir);
		xfer->synced = true;
	}
	wait_info->active_waiters--;
	*error = wait_info->error;
	if (!wait_info->active_waiters && xfer == NULL)
		kmem_cache_free(session->waiter_cache, wait_info);
	mutex_unlock(&session->lock);

	if (remaining == 0) {
		err = -ETIMEDOUT;
		dev_dbg(&abc_dma.pdev->dev, "%s: DMA Timed Out (%d)\n",
			__func__, err);

	} else if (remaining < 0) {
		err = -ERESTARTSYS;
		dev_dbg(&abc_dma.pdev->dev, "%s: DMA Restart (%d)\n",
			__func__, err);
	}

	return err;
}

/**
 * Kernel API for waiting for a transfer to be done.
 * @xfer[in] Data structure describing the DMA transfer including
 *               local and remote buffer descriptors & dma chan.
 * @timeout[in] Timeout usec, 0=zero wait, <0 = Infinity
 * @error[out] If any error encountered in the transaction itself it will be
 *            indicated here.
 * @start_id[out] start_id incremented with every re-start
 */
int abc_pcie_wait_dma_xfer(struct abc_dma_xfer *xfer, int timeout, int *error,
	uint32_t *start_id)
{
	int err = 0;
	struct abc_pcie_dma_session *session;
	struct abc_dma_wait_info *wait_info;
	*error = 0;

	err = abc_pcie_dma_get_kernel_wait(xfer, &session, &wait_info);
	if (err)
		return err;

	err = abc_pcie_dma_do_wait(session, wait_info, timeout, error,
				start_id);

	return err;
}
EXPORT_SYMBOL(abc_pcie_wait_dma_xfer);

static void add_entry(void *base_vaddr, size_t index, uint32_t header,
			uint64_t src_addr, uint64_t dst_addr, size_t size)
{
	struct abc_pcie_dma_ll_element entry;

	dev_dbg(&abc_dma.pdev->dev,
		"[%zu]: hdr=%08x src=%016lx dst=%016lx len=%zu\n",
		index, header, src_addr, dst_addr, size);

	entry.header = header;
	entry.size = size;
	entry.sar_low = LOWER(src_addr);
	entry.sar_high = UPPER(src_addr);
	entry.dar_low = LOWER(dst_addr);
	entry.dar_high = UPPER(dst_addr);

	memcpy_toio(base_vaddr + sizeof(entry) * index, &entry, sizeof(entry));
}

static void add_data_entry(void *base_vaddr, size_t index, uint64_t src_addr,
				uint64_t dst_addr, size_t size, size_t rem,
				bool last)
{
	if (index >= rem)
		return;

	add_entry(base_vaddr, index,
			last ? LL_IRQ_DATA_ELEMENT : LL_DATA_ELEMENT, src_addr,
			dst_addr, size);
}

static void add_link_entry(void *base_vaddr, size_t index, size_t rem)
{
	if (index >= rem)
		return;

	add_entry(base_vaddr, index, LL_LAST_LINK_ELEMENT, 0, 0, 0);
}

static size_t max_entry_size = UINT_MAX;

static ssize_t max_entry_size_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", max_entry_size);
}

static ssize_t max_entry_size_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int err;
	unsigned long size;

	err = kstrtoul(buf, 0, &size);
	if (err)
		return err;

	if ((size == 0) || (size > UINT_MAX))
		return -EINVAL;

	max_entry_size = size;

	return count;
}


static DEVICE_ATTR(max_entry_size, 0664, max_entry_size_show,
			max_entry_size_store);

static void seek_scatterlist(struct scatterlist **sc_list, int *count,
				size_t *offset)
{
	int sge_rem = *count;
	uint64_t off_rem = *offset;
	struct scatterlist *sge = *sc_list;

	while ((sge_rem > 0) && (off_rem > sg_dma_len(sge))) {
		off_rem -= sg_dma_len(sge);
		sge = sg_next(sge);
		sge_rem--;
	}

	*sc_list = sge;
	*count = sge_rem;
	*offset = off_rem;
}

static int num_dma_channels_read = DMA_CHANS_PER_READ_XFER;
static int num_dma_channels_write = DMA_CHANS_PER_WRITE_XFER;

static ssize_t num_dma_read_channels_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", num_dma_channels_read);
}

static ssize_t num_dma_read_channels_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int err;
	unsigned long channels;

	err = kstrtoul(buf, 0, &channels);
	if (err)
		return err;

	if ((channels == 0) || (channels > ABC_DMA_MAX_CHAN))
		return -EINVAL;

	num_dma_channels_read = channels;
	return count;
}


static DEVICE_ATTR(num_dma_read_channels, 0664, num_dma_read_channels_show,
			num_dma_read_channels_store);

static ssize_t num_dma_write_channels_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", num_dma_channels_write);
}

static ssize_t num_dma_write_channels_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int err;
	unsigned long channels;

	err = kstrtoul(buf, 0, &channels);
	if (err)
		return err;

	if ((channels == 0) || (channels > ABC_DMA_MAX_CHAN))
		return -EINVAL;

	num_dma_channels_write = channels;
	return count;
}


static DEVICE_ATTR(num_dma_write_channels, 0664, num_dma_write_channels_show,
			num_dma_write_channels_store);

static int abc_pcie_build_transfer_list(struct abc_buf_desc *src_buf,
				struct abc_buf_desc *dst_buf,
				size_t xfer_size,
				struct abc_pcie_dma_mblk_desc *mblk_desc,
				int *num_entries /*in-out*/)
{
	size_t size_rem = xfer_size;
	size_t entry_index = 0;
	uint32_t entry_size = 0, channel_entry_index = 0;
	uint64_t entry_src_addr;	/* valid when entry_size > 0 */
	uint64_t entry_dst_addr;	/* valid when entry_size > 0 */
	size_t entries_rem = *num_entries;
	int src_sge_rem = src_buf->sgl->n_num;
	size_t src_off = src_buf->offset;
	struct scatterlist *src_sge = src_buf->sgl->sc_list;
	size_t src_addr;		/* local to primary loop */
	int dst_sge_rem = dst_buf->sgl->n_num;
	size_t dst_off = dst_buf->offset;
	struct scatterlist *dst_sge = dst_buf->sgl->sc_list;
	size_t dst_addr;		/* local to primary loop */
	size_t size;			/* local to primary loop */
	int num_dma_channels;
	int entries_per_channel;
	void *base_vaddr;

	if (mblk_desc) {
		num_dma_channels = mblk_desc->num_dma_channels;
		entries_per_channel = mblk_desc->entries_per_channel[0];
		base_vaddr = mblk_desc->mapping.bar_vaddr;
	} else {
		num_dma_channels = 1;
		entries_per_channel = INT_MAX;
		base_vaddr = NULL;
	}

	seek_scatterlist(&src_sge, &src_sge_rem, &src_off);
	seek_scatterlist(&dst_sge, &dst_sge_rem, &dst_off);

	while ((src_sge_rem > 0) && (dst_sge_rem > 0) && (size_rem > 0)) {
		size = sg_dma_len(src_sge) - src_off;
		size = min(size, sg_dma_len(dst_sge) - dst_off);
		size = min(size, max_entry_size);
		size = min(size, size_rem);

		src_addr = sg_dma_address(src_sge) + src_off;
		dst_addr = sg_dma_address(dst_sge) + dst_off;

		if ((entry_size > 0) &&
			(entry_size < max_entry_size) &&
			(src_addr == (entry_src_addr + entry_size)) &&
			(dst_addr == (entry_dst_addr + entry_size))) {
			size = min(size, max_entry_size - entry_size);
			entry_size += size;
		} else {
			if (entry_size > 0) {
				channel_entry_index++;
				add_data_entry(base_vaddr, entry_index,
					entry_src_addr, entry_dst_addr,
					entry_size, entries_rem,
					/*last=*/ channel_entry_index ==
					entries_per_channel);
				entry_index++;
			}

			entry_size = size;
			entry_src_addr = src_addr;
			entry_dst_addr = dst_addr;
		}

		if (channel_entry_index == entries_per_channel) {
			add_link_entry(base_vaddr, entry_index, entries_rem);
			entry_index++;
			channel_entry_index = 0;
			num_dma_channels--;
			entries_per_channel = (entries_rem - entry_index - 1)
				/ num_dma_channels;
			mblk_desc->entries_per_channel[
				mblk_desc->num_dma_channels -
				num_dma_channels] =
				entries_per_channel;
		}
		size_rem -= size;

		src_off += size;
		if (src_off == sg_dma_len(src_sge)) {
			src_sge = sg_next(src_sge);
			src_sge_rem--;
			src_off = 0;
		}

		dst_off += size;
		if (dst_off == sg_dma_len(dst_sge)) {
			dst_sge = sg_next(dst_sge);
			dst_sge_rem--;
			dst_off = 0;
		}
	}

	if (size_rem) {
		dev_err(&abc_dma.pdev->dev,
			"%s: transfer oob: src off %zu, dst off %zu, size %zu\n",
			__func__, src_buf->offset, dst_buf->offset, xfer_size);
		return -EINVAL;
	}

	add_data_entry(base_vaddr, entry_index, entry_src_addr,
			entry_dst_addr, entry_size, entries_rem, /*last=*/true);
	entry_index++;
	add_link_entry(base_vaddr, entry_index, entries_rem);
	entry_index++;

	if (entries_rem && WARN_ON(entries_rem != entry_index)) {
		dev_err(&abc_dma.pdev->dev,
			"%s unexpected number of entries: entries %zu, expected %zu\n",
			__func__, entry_index, entries_rem);
	}

	dev_dbg(&abc_dma.pdev->dev, "%zu entries\n", entry_index);

	*num_entries = entry_index;

	return 0;
}

static struct abc_buf_desc *pick_src_buf(struct abc_dma_xfer *xfer)
{
	if (xfer->dir == DMA_TO_DEVICE)
		return &xfer->local_buf;
	else
		return &xfer->remote_buf;
}

static struct abc_buf_desc *pick_dst_buf(struct abc_dma_xfer *xfer)
{
	if (xfer->dir == DMA_TO_DEVICE)
		return &xfer->remote_buf;
	else
		return &xfer->local_buf;
}

static int abc_pcie_setup_sblk_xfer(struct abc_dma_xfer *xfer)
{
	struct abc_buf_desc *src_buf = pick_src_buf(xfer);
	int src_sge_rem = src_buf->sgl->n_num;
	size_t src_off = src_buf->offset;
	struct scatterlist *src_sge = src_buf->sgl->sc_list;
	struct abc_buf_desc *dst_buf = pick_dst_buf(xfer);
	int dst_sge_rem = dst_buf->sgl->n_num;
	size_t dst_off = dst_buf->offset;
	struct scatterlist *dst_sge = dst_buf->sgl->sc_list;
	struct dma_element_t *dma_blk;

	/* offset and bound are checked by abc_pcie_build_transfer_list */
	seek_scatterlist(&src_sge, &src_sge_rem, &src_off);
	seek_scatterlist(&dst_sge, &dst_sge_rem, &dst_off);

	xfer->transfer_method = DMA_TRANSFER_USING_SBLOCK;

	dma_blk = kzalloc(sizeof(struct dma_element_t), GFP_KERNEL);
	if (!dma_blk)
		return -ENOMEM;

	dma_blk->src_addr = LOWER(sg_dma_address(src_sge) + src_off);
	dma_blk->src_u_addr = UPPER(sg_dma_address(src_sge) + src_off);
	dma_blk->dst_addr = LOWER(sg_dma_address(dst_sge) + dst_off);
	dma_blk->dst_u_addr = UPPER(sg_dma_address(dst_sge) + dst_off);
	dma_blk->len = xfer->size;

	xfer->sblk_desc = dma_blk;

	return 0;
}

/**
 * Build linked list for multi-block transfer locally and transfer
 * to endpoint via single block DMA transfer.
 * @xfer[in] xfer to be operated on.
 */
static int abc_pcie_setup_mblk_xfer(struct abc_dma_xfer *xfer, int num_entries)
{
	int dma_chan;
	size_t ll_size;
	struct abc_pcie_dma_mblk_desc *mblk_desc;
	int err = 0;

	xfer->transfer_method = DMA_TRANSFER_USING_MBLOCK;

	dev_dbg(&abc_dma.pdev->dev,
		"%s: MBLK transfer (dir=%s)\n",
		__func__,
		(xfer->dir == DMA_TO_DEVICE) ? "AP2EP" : "EP2AP");

	mblk_desc = kzalloc(sizeof(struct abc_pcie_dma_mblk_desc), GFP_KERNEL);
	if (!mblk_desc)
		return -ENOMEM;

	xfer->mblk_desc = mblk_desc;

	num_entries--;

	mblk_desc->num_dma_channels = xfer->dir == DMA_TO_DEVICE ?
		num_dma_channels_read : num_dma_channels_write;

	mblk_desc->num_dma_channels = min(mblk_desc->num_dma_channels,
		num_entries);

	/* entries per channel not including list end entry */
	mblk_desc->entries_per_channel[0] =
		(num_entries + mblk_desc->num_dma_channels - 1)
		/ mblk_desc->num_dma_channels;

	/* number of total entries including list end entries */
	num_entries = num_entries + mblk_desc->num_dma_channels;

	ll_size = num_entries * sizeof(struct abc_pcie_dma_ll_element);

	err = abc_pcie_dma_alloc_ab_dram(ll_size, mblk_desc);
	if (err) {
		dev_err(&abc_dma.pdev->dev,
			"%s: failed to alloc ch%d BAR size: %d error: %d\n",
			__func__, dma_chan, ll_size, err);
		err = -ENOMEM;
		goto release_mblk;
	}

	/**
	 * Send link list to AB. Note that this does not touch the
	 * DMA engine as the DMA engine is only protected by
	 * dma_spinlock.
	 */
	mutex_lock(&abc_dma.iatu_mutex);

	mblk_desc->mapping.iatu = abc_dma.iatu;
	err = abc_pcie_map_iatu(abc_dma.dma_dev,
			&abc_dma.pdev->dev /* owner */, BAR_2,
			mblk_desc->size, mblk_desc->dma_paddr,
			&mblk_desc->mapping);
	if (err) {
		dev_err(&abc_dma.pdev->dev,
			"%s: failed to map to BAR ch%d error: %d\n",
			__func__, dma_chan, err);

			goto unlock_unmap_buf;
	}

	err = abc_pcie_build_transfer_list(
			pick_src_buf(xfer), pick_dst_buf(xfer), xfer->size,
			mblk_desc, &num_entries);

	abc_pcie_unmap_iatu(abc_dma.dma_dev,
			&abc_dma.pdev->dev /* owner */,
			&mblk_desc->mapping);

	if (err)
		goto unlock_unmap_buf;

	mutex_unlock(&abc_dma.iatu_mutex);

	return 0;

unlock_unmap_buf:
	ab_dram_free_dma_buf_kernel(mblk_desc->ab_dram_dma_buf);
	mutex_unlock(&abc_dma.iatu_mutex);
release_mblk:
	kfree(mblk_desc);
	xfer->mblk_desc = NULL;

	return err;
}

/**
 * dma_create_xfer_from_kernel_desc:
 * Allocates transfer and copies data from descriptor to transfer.
 */
static int dma_alloc_xfer_from_kernel_desc(
	struct abc_pcie_kernel_dma_desc *desc, struct abc_dma_xfer **new_xfer)
{
	int err;
	struct abc_dma_xfer *xfer;

	if (!(desc->dir == DMA_FROM_DEVICE || desc->dir == DMA_TO_DEVICE)) {
		dev_err(&abc_dma.pdev->dev, "%s: Invalid dir specification\n",
			__func__);
		return -EINVAL;
	}

	xfer = kzalloc(sizeof(struct abc_dma_xfer), GFP_KERNEL);
	if (!xfer)
		return -ENOMEM;

	xfer->dir = desc->dir;
	xfer->size = desc->size;

	xfer->local_buf.buf_type = desc->local_buf_kind;
	switch (desc->local_buf_kind) {
	case DMA_BUFFER_KIND_USER:
	case DMA_BUFFER_KIND_VMALLOC:
		xfer->local_buf.local_addr = desc->local_buf;
		xfer->local_buf.offset = 0;
		break;
	case DMA_BUFFER_KIND_DMA_BUF:
		xfer->local_buf.fd = desc->local_dma_buf_fd;
		xfer->local_buf.offset = desc->local_dma_buf_off;
		break;
	case DMA_BUFFER_KIND_CMA:
		xfer->local_buf.local_addr = desc->local_buf;
		xfer->local_buf.offset = 0;
		break;
	default:
		dev_err(&abc_dma.pdev->dev,
			"%s: Invalid descriptor type\n", __func__);
		err = -EINVAL;
		goto free_xfer;
	}

	xfer->remote_buf.buf_type = desc->remote_buf_kind;
	switch (desc->remote_buf_kind) {
	case DMA_BUFFER_KIND_USER:
		xfer->remote_buf.remote_addr = desc->remote_buf;
		xfer->remote_buf.offset = 0;
		break;
	case DMA_BUFFER_KIND_DMA_BUF:
		xfer->remote_buf.fd = desc->remote_dma_buf_fd;
		xfer->remote_buf.offset = desc->remote_dma_buf_off;
		break;
	default:
		dev_err(&abc_dma.pdev->dev,
			"%s: Invalid descriptor type\n", __func__);
		err = -EINVAL;
		goto free_xfer;
	}
	*new_xfer = xfer;
	return 0;

free_xfer:
	kfree(xfer);
	return err;
}

/**
 * Create and prepare a transfer.
 * @session[in] session to which this transfer belongs to.
 * @desc[in] Describes the transfer.
 * @new_xfer[out] Transfer structure that is re-used for the rest of the calls.
 * @id[out] Newly assign ID for the created transfer.
 * @return 0 on success
 */
int abc_pcie_create_dma_xfer(struct abc_pcie_dma_session *session,
				struct abc_pcie_kernel_dma_desc *desc,
				struct abc_dma_xfer **new_xfer, uint64_t *id)
{
	int num_entries = 0;
	struct abc_dma_xfer *xfer;
	struct abc_buf_desc *lbuf;
	struct abc_buf_desc *rbuf;
	int err = 0;

	if (!session)
		return -EINVAL;

	down_read(&abc_dma.state_transition_rwsem);

	if (!abc_dma.pcie_link_up || (abc_dma.dram_state != AB_DMA_DRAM_UP) ||
			test_bit(0, &abc_dma.link_poisoned)) {
		up_read(&abc_dma.state_transition_rwsem);
		return -EREMOTEIO;
	}

	err = dma_alloc_xfer_from_kernel_desc(desc, &xfer);
	if (err) {
		up_read(&abc_dma.state_transition_rwsem);
		return err;
	}

	dev_dbg(&abc_dma.pdev->dev, "%s: xfer_id:%0llu", __func__,
		session->next_xfer_id);

	if (new_xfer)
		*new_xfer = xfer;
	xfer->session = session;
	lbuf = &xfer->local_buf;
	rbuf = &xfer->remote_buf;

	if (xfer->size == 0) {
		dev_err(&abc_dma.pdev->dev, "%s: Invalid transfer size: 0\n",
				__func__);
		up_read(&abc_dma.state_transition_rwsem);
		return -EINVAL;
	}

	dev_dbg(&abc_dma.pdev->dev,
		"%s: local_buf_type=%d, local_buf=%pK, size=%u\n",
		__func__, lbuf->buf_type, lbuf->local_addr, xfer->size);

	/* Create scatterlist of local buffer */
	lbuf->sgl = kzalloc(sizeof(struct abc_pcie_sg_list), GFP_KERNEL);
	if (!lbuf->sgl) {
		up_read(&abc_dma.state_transition_rwsem);
		return -ENOMEM;
	}

	switch (lbuf->buf_type) {
	case DMA_BUFFER_KIND_USER:
		err = abc_pcie_user_local_buf_sg_build(xfer->dir, xfer->size,
							lbuf);
		break;
	case DMA_BUFFER_KIND_DMA_BUF:
		err = abc_pcie_sg_retrieve_from_dma_buf(xfer->dir, xfer->size,
							lbuf);
		break;
	case DMA_BUFFER_KIND_VMALLOC:
		err = abc_pcie_vmalloc_buf_sg_build(xfer->dir, xfer->size,
							lbuf);
		break;
	case DMA_BUFFER_KIND_CMA:
		err = abc_pcie_local_cma_build(xfer->dir, xfer->size, lbuf);
		break;
	default:
		dev_err(&abc_dma.pdev->dev,
			"%s: Unknown local DMA buffer type %d\n",
		       __func__, lbuf->buf_type);
		err = -EINVAL;
		break;
	}
	if (err < 0) {
		kfree(lbuf->sgl);
		goto clean_transfer;
	}

	/* Create scatterlist of remote buffer */
	rbuf->sgl = kzalloc(sizeof(struct abc_pcie_sg_list), GFP_KERNEL);
	if (!rbuf->sgl) {
		err = -ENOMEM;
		goto clean_local_buffer;
	}

	switch (rbuf->buf_type) {
	case DMA_BUFFER_KIND_USER:
		err = abc_pcie_user_remote_buf_sg_build(xfer->dir,
							xfer->size,
							rbuf);
		break;
	case DMA_BUFFER_KIND_DMA_BUF:
		err = abc_pcie_sg_retrieve_from_dma_buf(xfer->dir,
							xfer->size,
							rbuf);
		if (!err && !is_ab_dram_dma_buf(rbuf->sgl->dma_buf)) {
			dev_err(&abc_dma.pdev->dev,
					"%s: Remote DMA buffer is is not Airbrush memory\n",
					__func__);
			err = -EINVAL;
		}
		break;
	default:
		dev_err(&abc_dma.pdev->dev,
				"%s: Unknown remote DMA buffer type %d\n",
				__func__, rbuf->buf_type);
		err = -EINVAL;
		break;
	}
	if (err < 0) {
		kfree(rbuf->sgl);
		goto clean_local_buffer;
	}
	/* count the number of expected entries */
	err = abc_pcie_build_transfer_list(
			pick_src_buf(xfer), pick_dst_buf(xfer), xfer->size,
			/*mblk_desc=*/NULL, &num_entries);
	if (err)
		goto clean_buffers;

	if (num_entries == 2)
		err = abc_pcie_setup_sblk_xfer(xfer);
	else
		err = abc_pcie_setup_mblk_xfer(xfer, num_entries);
	if (err)
		goto clean_buffers;

	/* Update transfer state and add to the list */
	mutex_lock(&session->lock);
	list_add_tail(&xfer->list_transfers, &session->transfers);
	xfer->pending = false;
	xfer->id = session->next_xfer_id;
	if (id)
		*id = xfer->id;
	session->next_xfer_id++;
	mutex_unlock(&session->lock);
	up_read(&abc_dma.state_transition_rwsem);

	dev_dbg(&abc_dma.pdev->dev, "%s: created xfer: id:%0llu\n",
		__func__, xfer->id);
	return 0;

	/* In case of error clear & free xfer */
clean_buffers:
	abc_pcie_clean_dma_remote_buffers(xfer);
clean_local_buffer:
	abc_pcie_clean_dma_local_buffers(xfer);
clean_transfer:
	kfree(xfer);
	*new_xfer = NULL;
	up_read(&abc_dma.state_transition_rwsem);
	return err;
}
EXPORT_SYMBOL(abc_pcie_create_dma_xfer);

/**
 * Create and prepare a sessionless transfer.
 * @desc[in] Describes the transfer.
 * @new_xfer[out] Transfer structure that is re-used for the rest of the calls.
 * @return 0 on success
 */
int abc_pcie_create_sessionless_dma_xfer(struct abc_pcie_kernel_dma_desc *desc,
					struct abc_dma_xfer **new_xfer)
{
	return abc_pcie_create_dma_xfer(&global_session, desc, new_xfer, NULL);
}

/**
 * Top-level API for asynchronous DMA transfers
 * Starts the hardware DMA transfer.
 * @xfer[in] Data structure describing the DMA transfer including
 *               local and remote buffer descriptors & dma chan
 */
static void abc_pcie_exec_dma_xfer(struct abc_dma_xfer *xfer)
{
	int err = 0;
	int dma_chan = DMA_CHAN;
	struct abc_dma_wait_info *wait_info;

	if (xfer->transfer_method == DMA_TRANSFER_USING_SBLOCK)
		err = dma_sblk_start(dma_chan, xfer->dir, xfer->sblk_desc);

	else { /* Multi block transfer */
		phys_addr_t mblk_addr[ABC_DMA_MAX_CHAN];
		int c, total_entries = 0;

		xfer->mblk_desc->channel_mask = 0;
		WARN_ON(dma_chan + xfer->mblk_desc->num_dma_channels >
			ABC_DMA_MAX_CHAN);

		for (c = 0; c < xfer->mblk_desc->num_dma_channels; ++c) {
			xfer->mblk_desc->channel_mask |= (1 << (c + DMA_CHAN));
			mblk_addr[c] = xfer->mblk_desc->dma_paddr +
				total_entries *
				sizeof(struct abc_pcie_dma_ll_element);
			total_entries +=
				xfer->mblk_desc->entries_per_channel[c] + 1;
		}

		err = dma_mblk_start(dma_chan, xfer->dir,
			    mblk_addr, xfer->mblk_desc->num_dma_channels);
		if (err)
			xfer->mblk_desc->channel_mask = 0;
	}

	if (!err)
		return;

	/* We have error clean, dequeue from pending, clean the transfer */
	wait_info = xfer->wait_info;
	wait_info->error = err;
	list_del(&xfer->list_pending);
	xfer->pending = false;
}

/**
 * Internal function for starting DMA transfers
 * @xfer[in] Data structure describing the DMA transfer including
 *               local and remote buffer descriptors & dma chan
 */
int abc_pcie_start_dma_xfer_locked(struct abc_dma_xfer *xfer,
					uint32_t *start_id)
{
	int err = 0;
	unsigned long flags;
	struct abc_dma_wait_info *wait_info, *last_wait_info;
	bool schedule_run = false;
	struct list_head *pending_q;
	struct abc_pcie_sg_list *sgl = (xfer->local_buf).sgl;

	pending_q = (xfer->dir == DMA_TO_DEVICE) ? &pending_to_dev_q :
						&pending_from_dev_q;

	/**
	 * Guard against an application trying to re-start a transaction
	 * when it has not finished or is in an error state.
	 */
	spin_lock_irqsave(&dma_spinlock, flags);
	if (xfer->pending) {
		dev_err(&abc_dma.pdev->dev,
			"%s: Error: Cannot re-start pending DMA [ID:%0d]\n",
			__func__, xfer->id);
		err = -EBUSY; /* Ask user to try again */
	}
	spin_unlock_irqrestore(&dma_spinlock, flags);
	if (err)
		return err;

	if ((xfer->local_buf).buf_type != DMA_BUFFER_KIND_DMA_BUF &&
		(xfer->local_buf).buf_type != DMA_BUFFER_KIND_CMA)
		dma_sync_sg_for_device(abc_dma.dma_dev, sgl->sc_list,
					sgl->n_num, sgl->dir);

	/* Configure wait structures */
	wait_info = xfer->wait_info;
	last_wait_info = NULL;

	/* Reuse wait_info if there are no references to it. */
	if (wait_info != NULL && !wait_info->active_waiters) {
		reinit_completion(&wait_info->done);
	} else {
		if (wait_info)
			last_wait_info = wait_info;
		wait_info = (struct abc_dma_wait_info *)kmem_cache_alloc(
				(xfer->session)->waiter_cache, GFP_KERNEL);
		if (!wait_info)
			return -ENOMEM;
		if (last_wait_info)
			last_wait_info->xfer = NULL;
		init_completion(&wait_info->done);
	}
	wait_info->error = 0;
	wait_info->active_waiters = 0;
	wait_info->start_id = (xfer->wait_info == NULL) ? 0 :
				(xfer->wait_info)->start_id + 1;
	wait_info->xfer_id = xfer->id;
	wait_info->xfer = xfer;
	xfer->wait_info = wait_info;
	xfer->synced = false;

	dev_dbg(&abc_dma.pdev->dev, "%s: starting xfer: id:%0llu start_id:%0u\n",
			__func__, wait_info->xfer_id, wait_info->start_id);

	*start_id = wait_info->start_id;

	spin_lock_irqsave(&dma_spinlock, flags);
	xfer->pending = true;
	if (list_empty(pending_q))
		schedule_run = true;
	list_add_tail(&xfer->list_pending, pending_q);
	if (schedule_run)
		abc_pcie_exec_dma_xfer(xfer);
	spin_unlock_irqrestore(&dma_spinlock, flags);

	return err;
}

/**
 * Kernel client API for starting DMA transfers
 * @xfer[in] Data structure describing the DMA transfer including
 *               local and remote buffer descriptors & dma chan
 */
int abc_pcie_start_dma_xfer(struct abc_dma_xfer *xfer, uint32_t *start_id)
{
	int err;

	down_read(&abc_dma.state_transition_rwsem);
	if (!abc_dma.pcie_link_up || (abc_dma.dram_state != AB_DMA_DRAM_UP) ||
		xfer->poisoned) {
		up_read(&abc_dma.state_transition_rwsem);
		dev_err(&abc_dma.pdev->dev,
			"Cannot process: pcie_link:%s dram_state:%s poisoned:%0d\n",
			abc_dma.pcie_link_up ? "Up" : "Down",
			dram_state_str(abc_dma.dram_state), xfer->poisoned);
		return -EREMOTEIO;
	}

	mutex_lock(&(xfer->session)->lock);
	err = abc_pcie_start_dma_xfer_locked(xfer, start_id);
	mutex_unlock(&(xfer->session)->lock);

	up_read(&abc_dma.state_transition_rwsem);
	return err;
}
EXPORT_SYMBOL(abc_pcie_start_dma_xfer);

/**
 * Issue a Synchronous DMA transfer
 * @session[in] Currently open session.
 * @desc[in] Describes the transfer.
 * @return 0 on success
 */
int abc_pcie_issue_dma_xfer_sync(struct abc_pcie_dma_session *session,
				struct abc_pcie_kernel_dma_desc *desc)
{
	int err = 0;
	struct abc_dma_xfer *xfer;
	int xfer_error = 0;
	uint32_t start_id;

	err = abc_pcie_create_dma_xfer(session, desc, &xfer, NULL);
	if (err)
		return err;

	err = abc_pcie_start_dma_xfer(xfer, &start_id);
	if (err)
		goto  clean_transfer;

	err = abc_pcie_wait_dma_xfer(xfer, -1, &xfer_error, &start_id);

clean_transfer:
	abc_pcie_clean_dma_xfer(xfer);

	return err;
}
EXPORT_SYMBOL(abc_pcie_issue_dma_xfer_sync);

/**
 * Issue a Sessionless Synchronous DMA transfer
 * @desc[in] Describes the transfer.
 * @return 0 on success
 */
int abc_pcie_issue_sessionless_dma_xfer_sync(
					struct abc_pcie_kernel_dma_desc *desc)
{
	return abc_pcie_issue_dma_xfer_sync(&global_session, desc);
}

/**
 * Initializes a session.
 * @session[out] New session.
 * @return 0 on success
 */
int abc_pcie_dma_open_session(struct abc_pcie_dma_session *session)
{
	INIT_LIST_HEAD(&session->transfers);
	mutex_init(&session->lock);
	session->waiter_cache = waiter_cache;
	session->uapi = &abc_dma.uapi;
	down_write(&abc_dma.state_transition_rwsem);
	list_add_tail(&session->list_session, &abc_dma.sessions);
	up_write(&abc_dma.state_transition_rwsem);
	return 0;
}

/**
 * Cleans up a session.
 * @session[in] Existing session.
 * @return 0 on success
 */
void abc_pcie_dma_close_session(struct abc_pcie_dma_session *session)
{
	struct abc_dma_xfer *xfer, *nxt_xfer;

	list_for_each_entry_safe(xfer, nxt_xfer, &session->transfers,
		list_transfers) {
		abc_pcie_clean_dma_xfer(xfer);
	}
	down_write(&abc_dma.state_transition_rwsem);
	list_del(&session->list_session);
	up_write(&abc_dma.state_transition_rwsem);
}

static const struct attribute *abc_pcie_dma_attrs[] = {
	&dev_attr_max_entry_size.attr,
	&dev_attr_num_dma_read_channels.attr,
	&dev_attr_num_dma_write_channels.attr,
	NULL,
};

/**
 * Closes and destroys a session.
 * @session[in] Existing session.
 * @return 0 on success
 */
int abc_pcie_dma_drv_probe(struct platform_device *pdev)
{
	int err = 0;
	int dma_chan = 0;
	int i;

	abc_dma.pdev = pdev;
	abc_dma.pcie_link_up = true;
	abc_dma.dram_state = AB_DMA_DRAM_DOWN;
	abc_dma.link_poisoned = 0;

	err = sysfs_create_files(&pdev->dev.kobj, abc_pcie_dma_attrs);
	if (err) {
		dev_err(&abc_dma.pdev->dev, "failed to create sysfs entries\n");
		return err;
	}

	/* Depending on the IOMMU configuration the board the IPU may need
	 * to use MFD parent's device for mapping DMA buffers.  Otherwise, the
	 * dma device can be set to our own device and use SWIOTLB to map
	 * buffers.
	 */
	if (iommu_present(pdev->dev.parent->bus)) {
		abc_dma.dma_dev = pdev->dev.parent;
	} else {
		/* MFD doesn't seem to configure DMA ops for client devices */
		arch_setup_dma_ops(&pdev->dev, 0, 0, NULL,
				false /* coherent */);
		abc_dma.dma_dev = &pdev->dev;
	}
	abc_dma.uapi.mdev.parent = &pdev->dev;
	abc_dma.uapi.abc_dma = &abc_dma;
	err = init_abc_pcie_dma_uapi(&abc_dma.uapi);
	if (err)
		goto remove_sysfs;

	abc_dma.iatu = abc_pcie_get_inbound_iatu(abc_dma.dma_dev,
		&abc_dma.pdev->dev /* owner */);
	if (abc_dma.iatu < 0) {
		err = abc_dma.iatu;
		goto remove_uapi;
	}

	INIT_LIST_HEAD(&abc_dma.sessions);
	INIT_LIST_HEAD(&pending_to_dev_q);
	INIT_LIST_HEAD(&pending_from_dev_q);

	mutex_init(&abc_dma.iatu_mutex);

	for (dma_chan = 0; dma_chan < ABC_DMA_MAX_CHAN; dma_chan++) {
		err = abc_reg_dma_irq_callback(&dma_callback, dma_chan);
		if (err) {
			dev_err(&abc_dma.pdev->dev,
				"%s: failed to register ch%d dma irq callback: %d\n",
				__func__, dma_chan, err);
			goto restore_callback;
		}
	}

	init_rwsem(&abc_dma.state_transition_rwsem);
	/* Initialize Global Session */
	err = abc_pcie_dma_open_session(&global_session);
	if (err)
		goto restore_callback;

	waiter_cache = kmem_cache_create("wait_info",
					sizeof(struct abc_dma_wait_info),
					ARCH_SLAB_MINALIGN, 0, NULL);

	if (!waiter_cache) {
		err = -ENOMEM;
		goto close_dma_session;
	}

	global_session.waiter_cache = waiter_cache;

	/* Prepare DRAM and PCIE notifiers */
	abc_dma.dram_nb.notifier_call = dma_dram_blocking_listener;
	err = ab_sm_register_clk_event_for_dma(&abc_dma.dram_nb);
	if (err)
		goto close_dma_session;

	/* Register PCIe notify ops */
	abc_dma.pcie_notify_ops.pre_disable = dma_pcie_link_ops_pre_disable;
	abc_dma.pcie_notify_ops.post_enable = dma_pcie_link_ops_post_enable;
	abc_dma.pcie_notify_ops.link_error = dma_pcie_link_ops_link_error;
	ab_pcie_register_dma_device_ops(&abc_dma.pcie_notify_ops);

	return 0;

close_dma_session:
	abc_pcie_dma_close_session(&global_session);
	abc_dma.dram_nb.notifier_call = NULL;

restore_callback:
	for (i = dma_chan - 1; i >= 0; i--)
		abc_reg_dma_irq_callback(NULL, i);

	abc_pcie_put_inbound_iatu(abc_dma.dma_dev,
		&abc_dma.pdev->dev /* owner */, abc_dma.iatu);

remove_uapi:
	remove_abc_pcie_dma_uapi(&abc_dma.uapi);

remove_sysfs:
	sysfs_remove_files(&pdev->dev.kobj, abc_pcie_dma_attrs);

	return err;
}

int abc_pcie_dma_drv_remove(struct platform_device *pdev)
{
	remove_abc_pcie_dma_uapi(&abc_dma.uapi);
	abc_pcie_dma_close_session(&global_session);
	kmem_cache_destroy(waiter_cache);
	sysfs_remove_files(&pdev->dev.kobj, abc_pcie_dma_attrs);

	abc_pcie_put_inbound_iatu(abc_dma.dma_dev,
		&abc_dma.pdev->dev /* owner */, abc_dma.iatu);

	/* Elements for each session are removed upon close */
	list_del(&pending_to_dev_q);
	list_del(&pending_from_dev_q);

	ab_pcie_unregister_dma_device_ops();
	ab_sm_unregister_clk_event_for_dma(&abc_dma.dram_nb);

	return 0;
}

static const struct platform_device_id abc_pcie_dma_ids[] = {
	{
		.name = DRV_NAME_ABC_PCIE_DMA,
	}, {
		/* check abc-pcie-fsys */
	}
};
MODULE_DEVICE_TABLE(platform, abc_pcie_dma_ids);

static struct platform_driver abc_pcie_dma_driver = {
	.probe    = abc_pcie_dma_drv_probe,
	.remove   = abc_pcie_dma_drv_remove,
	.id_table = abc_pcie_dma_ids,
	.driver   = {
		.name = DRV_NAME_ABC_PCIE_DMA,
		.probe_type = PROBE_FORCE_SYNCHRONOUS,
	},
};
module_platform_driver(abc_pcie_dma_driver);

MODULE_AUTHOR("Google Inc.");
MODULE_DESCRIPTION("ABC PCIe endpoint DMA host side driver");
MODULE_LICENSE("GPL");
