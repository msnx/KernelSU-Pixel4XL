/*
 * Resource management for the Paintbox programmable IPU
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

#include <linux/completion.h>
#include <linux/err.h>
#include <linux/ipu-core.h>
#include <linux/ipu-jqs-messages.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include <uapi/linux/ipu.h>

#include "ipu-client.h"
#include "ipu-power.h"
#include "ipu-resource.h"
#include "ipu-regs.h"
#include "ipu-stp.h"

typedef void (*resource_allocator_t)(struct paintbox_data *,
		struct paintbox_session *, unsigned int);

/* The caller to this function must hold pb->lock */
static int ipu_resource_send_allocate(struct paintbox_data *pb,
		struct paintbox_session *session)
{
	struct jqs_message_alloc_resources req;

	INIT_JQS_MSG(req, JQS_MESSAGE_TYPE_ALLOC_RESOURCES);

	req.session_id = session->session_id;
	req.stp_id_mask = session->stp_id_mask;
	req.lbp_id_mask = session->lbp_id_mask;
	req.dma_channel_id_mask = session->dma_channel_id_mask;

	dev_dbg(pb->dev,
			"%s: session id %u stp mask: 0x%08x lbp mask: 0x%08x dma mask: 0x%08x\n",
			__func__, req.session_id, req.stp_id_mask,
			req.lbp_id_mask, req.dma_channel_id_mask);

	return ipu_jqs_send_sync_message(pb, (const struct jqs_message *)&req);
}

/* The caller to this function must hold pb->lock */
static int ipu_resource_send_release(struct paintbox_data *pb,
		struct paintbox_session *session)
{
	struct jqs_message_release_resources req;

	INIT_JQS_MSG(req, JQS_MESSAGE_TYPE_RELEASE_RESOURCES);

	req.session_id = session->session_id;
	req.stp_id_mask = session->stp_id_mask;
	req.lbp_id_mask = session->lbp_id_mask;
	req.dma_channel_id_mask = session->dma_channel_id_mask;

	dev_dbg(pb->dev,
			"%s: session id %u stp mask: 0x%08x lbp mask: 0x%08x dma mask: 0x%08x\n",
			__func__, req.session_id, req.stp_id_mask,
			req.lbp_id_mask, req.dma_channel_id_mask);

	return ipu_jqs_send_sync_message(pb, (const struct jqs_message *)&req);
}

/* The caller to this function must hold pb->lock */
static void ipu_resource_add_stp_to_session(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int stp_id)
{
	struct paintbox_stp *stp;

	stp = &pb->stp.stps[ipu_stp_id_to_index(stp_id)];
	stp->pm_enabled = true;
	stp->session = session;
	list_add_tail(&stp->session_entry, &session->stp_list);

	session->stp_id_mask |= 1 << stp->stp_id;
	pb->stp.available_stp_mask &= ~(1ULL << stp_id);

	dev_dbg(pb->dev, "stp%u allocated\n", stp_id);
}

/* The caller to this function must hold pb->lock */
static void ipu_resource_remove_stps_from_session(struct paintbox_data *pb,
		struct paintbox_session *session)
{
	struct paintbox_stp *stp, *stp_next;

	list_for_each_entry_safe(stp, stp_next, &session->stp_list,
			session_entry) {
		list_del(&stp->session_entry);

		stp->session = NULL;
		stp->pm_enabled = false;
		session->stp_id_mask &= ~(1 << stp->stp_id);

		pb->stp.available_stp_mask |= 1ULL << stp->stp_id;
		pb->power.stp_active_mask &= ~(1 <<
				ipu_stp_id_to_index(stp->stp_id));
	}
}

/* The caller to this function must hold pb->lock */
static void ipu_resource_add_lbp_to_session(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int lbp_id)
{
	struct paintbox_lbp *lbp;

	lbp = &pb->lbp.lbps[lbp_id];
	lbp->session = session;
	lbp->pm_enabled = true;
	session->lbp_id_mask |= 1 << lbp_id;

	list_add_tail(&lbp->session_entry, &session->lbp_list);

	pb->lbp.available_lbp_mask &= ~(1ULL << lbp_id);

	dev_dbg(pb->dev, "lbp%u allocated\n", lbp_id);
}

/* The caller to this function must hold pb->lock */
static void ipu_resource_remove_lbps_from_session(struct paintbox_data *pb,
		struct paintbox_session *session)
{
	struct paintbox_lbp *lbp, *lbp_next;

	list_for_each_entry_safe(lbp, lbp_next, &session->lbp_list,
			session_entry) {
		list_del(&lbp->session_entry);
		lbp->session = NULL;
		lbp->pm_enabled = false;
		session->lbp_id_mask &= ~(1 << lbp->pool_id);

		pb->lbp.available_lbp_mask |= 1ULL << lbp->pool_id;
		pb->power.lbp_active_mask &= ~(1 << lbp->pool_id);

		dev_dbg(pb->dev, "lbp%u release\n", lbp->pool_id);
	}
}

/* The caller to this function must hold pb->lock */
void ipu_resource_add_dma_channel_to_session(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int channel_id)
{
	struct paintbox_dma_channel *channel;

	channel = &pb->dma.channels[channel_id];
	channel->session = session;
	list_add_tail(&channel->session_entry, &session->dma_list);

	session->dma_channel_id_mask |= 1ULL << channel_id;
	pb->dma.available_channel_mask &= ~(1ULL << channel_id);

	pb->power.regs.dma_chan_en |= 1 << channel->channel_id;

	dev_dbg(pb->dev, "dma channel%u allocated\n", channel_id);
}

/* The caller to this function must hold pb->lock */
static void ipu_resource_remove_dma_channels_from_session(
		struct paintbox_data *pb, struct paintbox_session *session)
{
	struct paintbox_dma_channel *channel, *channel_next;

	list_for_each_entry_safe(channel, channel_next, &session->dma_list,
			session_entry) {
		list_del(&channel->session_entry);
		channel->session = NULL;
		session->dma_channel_id_mask &= ~(1ULL << channel->channel_id);
		pb->dma.available_channel_mask |= 1ULL << channel->channel_id;

		pb->power.regs.dma_chan_en &= ~(1 << channel->channel_id);

		dev_dbg(pb->dev, "dma channel%u release\n",
				channel->channel_id);
	}
}

/* The caller to this function must hold pb lock */
static void ipu_resource_notify_first_session_waiter(struct paintbox_data *pb)
{
	struct paintbox_session *first_entry;

	if (list_empty(&pb->bulk_alloc_waiting_list))
		return;

	first_entry = list_first_entry(&pb->bulk_alloc_waiting_list,
			struct paintbox_session, alloc_wait_list_entry);

	complete(&first_entry->bulk_alloc_completion);
}

/* The caller to this function must hold pb lock */
void ipu_resource_remove_session_from_wait_list(
		struct paintbox_session *session)
{
	if (session->waiting_alloc) {
		list_del(&session->alloc_wait_list_entry);
		session->waiting_alloc = false;
	}

	/* Signal completion on first entry to avoid
	 * starvation.
	 */
	ipu_resource_notify_first_session_waiter(session->dev);
}

/* The caller to this function must hold pb->lock */
static void ipu_resource_allocate_internal(struct paintbox_data *pb,
		struct paintbox_session *session, uint64_t req_mask,
		resource_allocator_t allocator)
{
	uint64_t iterator;
	int index;

	iterator = req_mask;

	for (index = 0; iterator > 0; iterator >>= 1, index++) {
		if (!(iterator & 0x1))
			continue;

		allocator(pb, session, index);
	}
}

/* The caller to this function must hold pb->lock
 * The caller need to release resource on failure (non-zero return)
 */
static int ipu_resource_allocate(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct ipu_resource_allocate_request *req)
{
	ipu_resource_allocate_internal(pb, session, req->stp_mask,
			&ipu_resource_add_stp_to_session);

	ipu_resource_allocate_internal(pb, session, req->lbp_mask,
			&ipu_resource_add_lbp_to_session);

	ipu_resource_allocate_internal(pb, session,
			req->dma_channel_mask,
			&ipu_resource_add_dma_channel_to_session);

	/* Update the DMA channel power configuration */
	ipu_writel(pb->dev, pb->power.regs.dma_chan_en,
			IPU_CSR_AON_OFFSET + IPU_DMA_CHAN_EN);

	return ipu_resource_send_allocate(pb, session);
}

static int ipu_resource_validate_request_mask(struct paintbox_data *pb,
		struct ipu_resource_allocate_request req)
{
	if (req.stp_mask >> (pb->stp.num_stps + 1)) {
		dev_err(pb->dev, "%s: STP request invalid\n", __func__);
		return -EINVAL;
	}

	if (req.lbp_mask >> pb->lbp.num_lbps) {
		dev_err(pb->dev, "%s: LBP request invalid\n", __func__);
		return -EINVAL;
	}

	if (req.dma_channel_mask >> pb->dma.num_channels) {
		dev_err(pb->dev, "%s: DMA request invalid\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static inline void ipu_resource_remove_held_resources(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct ipu_resource_allocate_request *req)
{
	req->stp_mask &= ~session->stp_id_mask;
	req->lbp_mask &= ~session->lbp_id_mask;
	req->dma_channel_mask &= ~session->dma_channel_id_mask;
}

/* The caller to this function must hold pb lock */
static bool ipu_resource_check_availability(struct paintbox_data *pb,
		struct ipu_resource_allocate_request *req)
{
	uint64_t busy_mask;
	bool is_available = true;

	busy_mask = req->stp_mask & ~pb->stp.available_stp_mask;
	if (busy_mask) {
		dev_warn(pb->dev,
				"%s: Some or all requested STPs are busy, busy mask: 0x%016llx\n",
				__func__, busy_mask);
		is_available = false;
	}

	busy_mask = req->lbp_mask & ~pb->lbp.available_lbp_mask;
	if (busy_mask) {
		dev_warn(pb->dev,
				"%s: Some or all requested LBPs are busy, busy mask: 0x%016llx\n",
				__func__, busy_mask);
		is_available = false;
	}

	busy_mask = req->dma_channel_mask & ~pb->dma.available_channel_mask;
	if (busy_mask) {
		dev_warn(pb->dev,
				"%s: Some or all requested DMA Channels are busy, busy mask: 0x%016llx\n",
				__func__, busy_mask);
		is_available = false;
	}

	return is_available;
}

/* The caller to this function must hold pb->lock */
static void ipu_resource_release_internal(struct paintbox_data *pb,
		struct paintbox_session *session)
{
	ipu_resource_remove_stps_from_session(pb, session);
	ipu_resource_remove_lbps_from_session(pb, session);
	ipu_resource_remove_dma_channels_from_session(pb, session);

	ipu_power_core_power_walk_down(pb);

	/* Update the DMA channel power configuration */
	ipu_writel(pb->dev, pb->power.regs.dma_chan_en,
			IPU_CSR_AON_OFFSET + IPU_DMA_CHAN_EN);
}

/* The caller to this function must hold pb->lock */
int ipu_resource_session_release(struct paintbox_data *pb,
		struct paintbox_session *session)
{
	int ret;

	if (!session->stp_id_mask &&
			!session->lbp_id_mask &&
			!session->dma_channel_id_mask)
		return 0;

	ret = ipu_resource_send_release(pb, session);

	/* Always release the resources even if there is an error. */
	ipu_resource_release_internal(pb, session);

	/* If the resource release fails either due to a communications issue
	 * with the JQS or a failure to stop a DMA channel then the IPU will
	 * need to be reset.
	 */
	if (ret < 0)
		ipu_client_request_reset(pb);

	return ret;
}

int ipu_resource_allocate_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	int ret;
	struct ipu_resource_allocate_request __user *user_req;
	struct ipu_resource_allocate_request req;
	long time_remaining = LONG_MAX;
	uint64_t timeout_remaining_ns;
	bool available = false;

	user_req = (struct ipu_resource_allocate_request __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	ret = ipu_resource_validate_request_mask(pb, req);
	if (ret < 0)
		return ret;

	timeout_remaining_ns = req.timeout_ns;

	mutex_lock(&pb->lock);
	if (ipu_reset_is_requested(pb)) {
		mutex_unlock(&pb->lock);
		return -ECONNRESET;
	}

	/* Remove resources already held by the session from the request. */
	ipu_resource_remove_held_resources(pb, session, &req);

	/* If all the resources requested are already held by the session then
	 * there is nothing more to do.
	 */
	if (req.stp_mask == 0 && req.lbp_mask == 0 &&
			req.dma_channel_mask == 0) {
		mutex_unlock(&pb->lock);
		return 0;
	}

	do {
		struct paintbox_session *first_entry;

		/* Adding session to the end of waiting list */
		if (!session->waiting_alloc) {
			list_add_tail(&session->alloc_wait_list_entry,
					&pb->bulk_alloc_waiting_list);
			session->waiting_alloc = true;
		}

		first_entry = list_first_entry(&pb->bulk_alloc_waiting_list,
				struct paintbox_session, alloc_wait_list_entry);

		if (first_entry == session) {
			available = ipu_resource_check_availability(pb,
					&req);

			/* If requested resources are available, skip to
			 * allocation
			 */
			if (available) {
				ipu_resource_remove_session_from_wait_list(
						session);
				break;
			}
		}

		reinit_completion(&session->bulk_alloc_completion);
		mutex_unlock(&pb->lock);

		if (timeout_remaining_ns == LONG_MAX) {
			ret = wait_for_completion_interruptible
				(&session->bulk_alloc_completion);
			if (ret) {
				mutex_lock(&pb->lock);
				if (ipu_reset_is_requested(pb)) {
					mutex_unlock(&pb->lock);
					return -ECONNRESET;
				}
				goto err_exit;
			}
			time_remaining = LONG_MAX;
		} else {
			time_remaining =
				wait_for_completion_interruptible_timeout
				(&session->bulk_alloc_completion,
				 nsecs_to_jiffies64(timeout_remaining_ns));

			if (time_remaining > 0)
				timeout_remaining_ns =
					jiffies_to_nsecs(time_remaining);
		}

		mutex_lock(&pb->lock);
		if (ipu_reset_is_requested(pb)) {
			mutex_unlock(&pb->lock);
			return -ECONNRESET;
		}
	} while (time_remaining > 0);

	if (time_remaining == 0 && !available) {
		ret = -ETIMEDOUT;
		goto err_exit;
	} else if (time_remaining < 0) {
		ret = time_remaining;
		goto err_exit;
	}

	/* If the JQS went down and was reset while the other session had the
	 * resources then exit without allocating the resources.
	 */
	if (!ipu_is_jqs_ready(pb->dev)) {
		mutex_unlock(&pb->lock);
		dev_err(pb->dev,
				"%s: unable to allocate resources, JQS is down\n",
				__func__);
		return -ENETDOWN;
	}

	ret = ipu_resource_allocate(pb, session, &req);
	if (ret < 0) {
		/* An error allocating the resources would indicate a comms
		 * error which will require a reset of the IPU and JQS.  Clean
		 * up the allocated resources and request a reset.
		 */
		ipu_resource_release_internal(pb, session);
		ipu_client_request_reset(pb);
	}

	mutex_unlock(&pb->lock);

	return ret;

err_exit:
	ipu_resource_remove_session_from_wait_list(session);
	mutex_unlock(&pb->lock);
	return ret;
}

int ipu_resource_release_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	int ret;

	mutex_lock(&pb->lock);
	if (ipu_reset_is_requested(pb)) {
		mutex_unlock(&pb->lock);
		return -ECONNRESET;
	}

	ret = ipu_resource_session_release(pb, session);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	ipu_resource_notify_first_session_waiter(pb);

	/* TODO(b/115530379): Power down if there are no waiters */

	mutex_unlock(&pb->lock);

	return 0;
}
