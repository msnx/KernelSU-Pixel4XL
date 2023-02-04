/*
 * Buffer management support for the Paintbox programmable IPU
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

#include <linux/ab-dram.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/idr.h>
#include <linux/ipu-core.h>
#include <linux/ipu-jqs-messages.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include <uapi/linux/ipu.h>

#include "ipu-buffer.h"
#include "ipu-client.h"

/* The caller to this function must hold pb->lock */
static inline struct paintbox_buffer *ipu_buffer_get_buffer(
		struct paintbox_session *session, uint32_t buffer_id)
{
	return (struct paintbox_buffer *)idr_find(&session->buffer_idr,
			buffer_id);
}

/* The caller to this function must hold pb->lock */
static int ipu_buffer_alloc(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_buffer **buffer)
{
	struct paintbox_buffer *tmp;
	int ret;

	tmp = kzalloc(sizeof(struct paintbox_buffer), GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	ret = idr_alloc(&session->buffer_idr, tmp, 0, PAINTBOX_BUFFER_ID_MAX,
			GFP_KERNEL);
	if (ret < 0) {
		dev_err(pb->dev, "%s: error allocating buffer id, err = %d\n",
				__func__, ret);
		kfree(tmp);

		return ret;
	}

	tmp->buffer_id = ret;

	*buffer = tmp;

	return 0;
}

/* The caller to this function must hold pb->lock */
static void ipu_buffer_release_buffer(struct paintbox_session *session,
		struct paintbox_buffer *buffer)
{
	idr_remove(&session->buffer_idr, buffer->buffer_id);
	kfree(buffer);
}

/* The caller to this function must hold pb->lock */
static int ipu_buffer_map_dma_buf(struct paintbox_data *pb,
		struct paintbox_session *session, int dma_buf_fd,
		enum dma_data_direction dir, struct paintbox_buffer *buffer)
{
	int ret;

	buffer->dir = dir;

	buffer->dma_buf = dma_buf_get(dma_buf_fd);
	if (IS_ERR(buffer->dma_buf))
		return PTR_ERR(buffer->dma_buf);

#if IS_ENABLED(CONFIG_MFD_ABC_PCIE_SMMU_IOVA)
	/* in case of sMMU with virtual addresses local buffers should be mapped
	 * using the sMMU and not the IPU-IOMMU
	 */
	if (!is_ab_dram_dma_buf(buffer->dma_buf))
		buffer->attach = dma_buf_attach(buffer->dma_buf,
			pb->dev->parent->parent);
	else
		buffer->attach = dma_buf_attach(buffer->dma_buf, pb->dev);
#else
	buffer->attach = dma_buf_attach(buffer->dma_buf, pb->dev);
#endif

	if (IS_ERR(buffer->attach)) {
		ret = PTR_ERR(buffer->attach);
		dev_err(pb->dev, "%s: failed to attach dma_buf, err %d\n",
				__func__, ret);
		goto err_put;
	}

	buffer->sg_table = dma_buf_map_attachment(buffer->attach, dir);
	if (IS_ERR(buffer->sg_table)) {
		ret = PTR_ERR(buffer->sg_table);
		dev_err(pb->dev, "%s: failed to map dma_buf, err %d\n",
				__func__, ret);
		goto err_detach;
	}

	return 0;

err_detach:
	dma_buf_detach(buffer->dma_buf, buffer->attach);
err_put:
	dma_buf_put(buffer->dma_buf);

	return ret;
}

/* The caller to this function must hold pb->lock */
static void ipu_buffer_unmap_dma_buf(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_buffer *buffer)
{
	dma_buf_unmap_attachment(buffer->attach, buffer->sg_table, buffer->dir);
	dma_buf_detach(buffer->dma_buf, buffer->attach);
	dma_buf_put(buffer->dma_buf);
}

/* The caller to this function must hold pb->lock */
static int ipu_buffer_map_iommu(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_buffer *buffer)
{
	int ret;
	struct scatterlist *s;
	int i;

	/* ab dram doesn't populate the page_link which is required
	 * for the kernel iommu logic
	 */
	if (is_ab_dram_dma_buf(buffer->dma_buf))
		for_each_sg(buffer->sg_table->sgl, s,
				buffer->sg_table->nents, i)
			s->page_link = (unsigned long)phys_to_page(
				s->dma_address);

	ret = dma_map_sg_attrs(pb->dev, buffer->sg_table->sgl,
				buffer->sg_table->nents, buffer->dir,
				DMA_ATTR_SKIP_CPU_SYNC);
	if (ret == 0) {
		dev_err(pb->dev,
				"%s: DMA map error, nents %d requested",
				__func__, buffer->sg_table->nents);
		return -EINVAL;
	}

	return 0;
}

/* The caller to this function must hold pb->lock */
static void ipu_buffer_unmap_iommu(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_buffer *buffer)
{
	dma_unmap_sg_attrs(pb->dev, buffer->sg_table->sgl,
			buffer->sg_table->nents, buffer->dir,
			DMA_ATTR_SKIP_CPU_SYNC);
}

/* The caller to this function must hold pb->lock */
static int ipu_buffer_send_jqs_buffers_register(struct paintbox_data *pb,
		struct paintbox_session *session,
		unsigned int num_buffers,
		struct ipu_dma_buf_register_entry *bufs)
{
	struct jqs_message_register_buffers msg;
	struct paintbox_buffer *buffer;
	unsigned int i, buffer_id;

	if (WARN_ON(!bufs))
		return -EINVAL;

	if (WARN_ON((num_buffers > MAX_BUFFER_REGISTRATION) ||
				(num_buffers == 0)))
		return -EINVAL;

	INIT_JQS_MSG(msg, JQS_MESSAGE_TYPE_REGISTER_BUFFERS);

	msg.header.size =
		offsetof(struct jqs_message_register_buffers, registrations) +
		sizeof(msg.registrations[0]) * num_buffers;


	msg.session_id = session->session_id;
	msg.num_buffers = num_buffers;

	for (i = 0; i < num_buffers; i++) {
		buffer_id = bufs[i].buffer_id;
		buffer = ipu_buffer_get_buffer(session, buffer_id);
		if (!buffer) {
			dev_err(pb->dev, "%s: Unable to find buffer %d\n",
					__func__, buffer_id);
			return -EINVAL;
		}
		msg.registrations[i].buffer_id = buffer_id;
		msg.registrations[i].buffer_addr =
				sg_dma_address(buffer->sg_table->sgl);
		msg.registrations[i].buffer_size =
				sg_dma_len(buffer->sg_table->sgl);

		dev_dbg(pb->dev, "%s: session_id %u buffer id %u addr %pad sz %u\n",
				__func__, session->session_id,
				buffer_id,
				&msg.registrations[i].buffer_addr,
				msg.registrations[i].buffer_size);
	}

	return ipu_jqs_send_sync_message(pb, (const struct jqs_message *)&msg);
}

/* The caller to this function must hold pb->lock */
static int ipu_buffer_send_jqs_buffers_unregister(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int num_buffers,
		uint32_t *buffer_ids)
{
	struct jqs_message_unregister_buffers msg;

	if (WARN_ON(!buffer_ids))
		return -EINVAL;

	if (WARN_ON((num_buffers > MAX_BUFFER_REGISTRATION) ||
				(num_buffers == 0)))
		return -EINVAL;

	INIT_JQS_MSG(msg, JQS_MESSAGE_TYPE_UNREGISTER_BUFFERS);

	msg.header.size =
		offsetof(struct jqs_message_unregister_buffers, buffer_ids) +
		sizeof(msg.buffer_ids[0]) * num_buffers;
	msg.session_id = session->session_id;
	msg.num_buffers = num_buffers;

	memcpy(msg.buffer_ids, buffer_ids, num_buffers * sizeof(uint32_t));

	return ipu_jqs_send_sync_message(pb, (const struct jqs_message *)&msg);
}

static bool ipu_buffer_iommu_active(struct paintbox_data *pb)
{
	return iommu_present(&ipu_bus_type) && ipu_is_iommu_active(pb->dev);
}

static bool ipu_buffer_is_32b_buffer(struct paintbox_buffer *buffer)
{
	/* all 32 bit buffers use ab physical addresses and will be < 1GB
	 * address range which is identity mapped
	 */
	return sg_dma_address(buffer->sg_table->sgl) <
		IPU_IOMMU_IDENTITY_MAP_SIZE;
}

/* The caller to this function must hold pb->lock */
static void ipu_buffer_unmap_and_release_buffer(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_buffer *buffer)
{
	/* Unmap the buffer from the IOMMU. */
	if (ipu_buffer_iommu_active(pb) &&
			is_ab_dram_dma_buf(buffer->dma_buf) &&
			!ipu_buffer_is_32b_buffer(buffer))
		ipu_buffer_unmap_iommu(pb, session, buffer);

	ipu_buffer_unmap_dma_buf(pb, session, buffer);
	ipu_buffer_release_buffer(session, buffer);
}

static int ipu_buffer_consolidate_mmu_sg_list(struct paintbox_data *pb,
	struct paintbox_buffer *buffer)
{
	int i;
	size_t total_size = 0;
	struct scatterlist *s;
	int ret = 0;
	dma_addr_t start_address = buffer->sg_table->sgl->dma_address;

	for_each_sg(buffer->sg_table->sgl, s,
			buffer->sg_table->nents, i) {
		if (s->dma_length > 0 &&
				total_size + start_address != s->dma_address) {
			dev_err(pb->dev,
				"%s unable to consolidate sg list for nent %d",
				__func__, i);
			ret = -EFAULT;
			break;
		}
		total_size += s->dma_length;
		if (i > 0) {
			s->dma_length = 0;
			s->dma_address = 0;
		}
	}
	buffer->sg_table->sgl->dma_length = total_size;
	return ret;
}

static bool ipu_buffer_check_valid_32b_buffer(struct paintbox_data *pb,
	struct paintbox_buffer *buffer)
{
	/* we are using the fact that we identity map the first
	 * IPU_IOMMU_IDENTITY_MAP_SIZE of memory and that ab_dram
	 * buffers are contiguous and just don't map the 32 bit address
	 * buffers.
	 */
	unsigned long buff_end_address;

	if (!is_ab_dram_dma_buf(buffer->dma_buf)) {
		dev_err(pb->dev,
			"32 bit address is only supported for AB Dram buffers");
		return false;
	}
	if (buffer->sg_table->nents > 1) {
		dev_err(pb->dev,
			"32 bit address doesn't support non-contiguous buffers");
		return false;
	}
	buff_end_address = sg_dma_address(buffer->sg_table->sgl) +
		sg_dma_len(buffer->sg_table->sgl);
	if (buff_end_address > IPU_IOMMU_IDENTITY_MAP_SIZE) {
		dev_err(pb->dev,
			"32 bit address out of supported range");
		return false;
	}
	return true;
}
/* The caller to this function must hold pb->lock */
static int ipu_buffer_dma_buf_process_registration(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct ipu_dma_buf_register_entry *entry,
		bool b32_address)
{
	struct paintbox_buffer *buffer;
	int ret;
	bool ipu_iommu_mapped = false;

	if (WARN_ON(!entry))
		return -EINVAL;

	/* Allocate a buffer id within the session. */
	ret = ipu_buffer_alloc(pb, session, &buffer);
	if (ret < 0)
		return ret;

	/* Convert the DMA Buf fd to a scatter list. */
	ret = ipu_buffer_map_dma_buf(pb, session, entry->dma_buf_fd,
			entry->dir, buffer);
	if (ret < 0)
		goto release_buffer;

	if (b32_address) {
		if (!ipu_buffer_check_valid_32b_buffer(pb, buffer)) {
			ret = -EINVAL;
			goto unmap_dma_buf;
		}
	} else if (ipu_buffer_iommu_active(pb)) {
		/*
		 * iommu mapping is not needed for ion. the ion infrastructure
		 * calls map_dma_buf internally.
		 */
		if (is_ab_dram_dma_buf(buffer->dma_buf)) {
			ret = ipu_buffer_map_iommu(pb, session, buffer);
			if (ret < 0)
				goto unmap_dma_buf;
			ipu_iommu_mapped = true;
		}
	}

	ret = ipu_buffer_consolidate_mmu_sg_list(pb, buffer);
	if (ret < 0)
		goto unmap_ipu_iommu;

	entry->buffer_id = buffer->buffer_id;

	return 0;

unmap_ipu_iommu:
	if (ipu_iommu_mapped)
		ipu_buffer_unmap_iommu(pb, session, buffer);
unmap_dma_buf:
	ipu_buffer_unmap_dma_buf(pb, session, buffer);
release_buffer:
	ipu_buffer_release_buffer(session, buffer);
	return ret;
}

int ipu_buffer_dma_buf_bulk_register_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool b32_address)
{
	struct ipu_dma_buf_bulk_register_req __user *user_req;
	struct ipu_dma_buf_bulk_register_req req;
	struct ipu_dma_buf_register_entry *req_bufs;
	struct paintbox_buffer *buffer;
	unsigned int num_processed_bufs = 0;
	unsigned int i;
	int ret, len;

	/* copy number of buffer to kernel to determine request list size */
	user_req = (struct ipu_dma_buf_bulk_register_req __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	if (req.num_buffers > MAX_BUFFER_REGISTRATION || req.num_buffers == 0) {
		dev_err(pb->dev, "%s: Invalid request to register %d buffers",
				__func__, req.num_buffers);
		return -EINVAL;
	}

	if (!req.bufs) {
		dev_err(pb->dev, "%s: request buf array is NULL", __func__);
		return -EINVAL;
	}

	len = req.num_buffers * sizeof(struct ipu_dma_buf_register_entry);

	req_bufs = kzalloc(len, GFP_KERNEL);
	if (!req_bufs)
		return -ENOMEM;

	if (copy_from_user(req_bufs, req.bufs, len)) {
		kfree(req_bufs);
		return -EFAULT;
	}

	mutex_lock(&pb->lock);
	if (ipu_reset_is_requested(pb)) {
		mutex_unlock(&pb->lock);
		kfree(req_bufs);
		return -ECONNRESET;
	}

	for (i = 0; i < req.num_buffers; i++) {
		ret = ipu_buffer_dma_buf_process_registration(pb, session,
				&req_bufs[i], b32_address);
		if (ret != 0)
			break;

		num_processed_bufs++;
	}

	if (num_processed_bufs != req.num_buffers) {
		dev_err(pb->dev, "%s: Buffer registration processing incomplete\n",
				__func__);
		goto unmap_and_cleanup;
	}

	if (copy_to_user(req.bufs, req_bufs, len)) {
		dev_err(pb->dev, "%s: Copy to user failed\n", __func__);
		ret = -EFAULT;
		goto unmap_and_cleanup;
	}
	/* Notify the JQS that the buffer has been mapped. */
	ret = ipu_buffer_send_jqs_buffers_register(pb, session,
			req.num_buffers, req_bufs);
	if (ret < 0)
		goto unmap_and_cleanup;

	mutex_unlock(&pb->lock);
	kfree(req_bufs);

	return 0;

/* Unwinding registration on failure */
unmap_and_cleanup:
	for (i = 0; i < num_processed_bufs; i++) {
		buffer = ipu_buffer_get_buffer(session, req_bufs[i].buffer_id);
		if (likely(buffer))
			ipu_buffer_unmap_and_release_buffer(pb, session,
					buffer);
	}
	mutex_unlock(&pb->lock);
	kfree(req_bufs);

	return ret;
}

int ipu_buffer_dma_buf_bulk_unregister_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct ipu_dma_buf_bulk_unregister_req __user *user_req;
	struct ipu_dma_buf_bulk_unregister_req req;
	struct paintbox_buffer *buffer;
	uint32_t *buf_ids;
	int ret, len, i;

	user_req = (struct ipu_dma_buf_bulk_unregister_req __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	if (req.num_buffers > MAX_BUFFER_REGISTRATION || req.num_buffers == 0) {
		dev_err(pb->dev, "%s: Invalid request to unregister %d buffers",
				__func__, req.num_buffers);
		return -EINVAL;
	}

	len = req.num_buffers * sizeof(uint32_t);

	buf_ids = kzalloc(len, GFP_KERNEL);
	if (!buf_ids)
		return -ENOMEM;

	if (copy_from_user(buf_ids, req.buf_ids, len)) {
		kfree(buf_ids);
		return -EFAULT;
	}

	mutex_lock(&pb->lock);
	if (ipu_reset_is_requested(pb)) {
		mutex_unlock(&pb->lock);
		kfree(buf_ids);
		return -ECONNRESET;
	}

	ret = ipu_buffer_send_jqs_buffers_unregister(pb, session,
			req.num_buffers, buf_ids);
	if (ret == -EBUSY) {
		/* If the JQS returns that the buffer is busy then report back
		 * the busy error immediately.  The userspace will need to try
		 * again when the DMA transfer on that buffer has completed.
		 */
		goto err_exit;
	}

	/* TODO(b/114760293):  Other errors returned by
	 * ipu_buffer_send_jqs_buffer_unregister() would be catastrophic
	 * errors and we need to take the catastrophic error path once that is
	 * implemented.  In the event of a catastrophic error the driver should
	 * still unmap the buffer since the JQS will be reset.
	 */
	for (i = 0; i < req.num_buffers; i++) {
		buffer = ipu_buffer_get_buffer(session, buf_ids[i]);
		if (!buffer) {
			dev_err(pb->dev, "%s: Unable to find buffer %d\n",
				__func__, buf_ids[i]);
			ret = -EINVAL;
		}
		ipu_buffer_unmap_and_release_buffer(pb, session, buffer);
	}

err_exit:
	mutex_unlock(&pb->lock);
	kfree(buf_ids);

	return ret;
}

int ipu_buffer_init_session(struct paintbox_data *pb,
		struct paintbox_session *session)
{
	session->buffer_id_table = ipu_alloc_jqs_memory(pb->dev,
			JQS_SESSION_MEMORY_SIZE);
	if (!session->buffer_id_table)
		return PTR_ERR(session->buffer_id_table);

	idr_init(&session->buffer_idr);

	return 0;
}

/* The caller to this function must hold pb->lock */
void ipu_buffer_release_session(struct paintbox_data *pb,
		struct paintbox_session *session)
{
	struct paintbox_buffer *buffer;
	int buffer_id;

	idr_for_each_entry(&session->buffer_idr, buffer, buffer_id)
		ipu_buffer_unmap_and_release_buffer(pb, session, buffer);

	ipu_free_jqs_memory(pb->dev, session->buffer_id_table);
	idr_destroy(&session->buffer_idr);
}
