/*
 * Core driver for the Paintbox command queue
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

#include <linux/anon_inodes.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/ipu-core.h>
#include <linux/ipu-jqs-messages.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "ipu-client.h"
#include "ipu-queue.h"

struct paintbox_cmd_queue {
	struct list_head session_entry;
	/* can be null for orphaned cmd_queues */
	struct paintbox_session *session;
	struct paintbox_data *pb;
	int queue_id;

	/* Set if the queue has been detached due to an error. */
	int err;
};

/* The caller to this function must hold pb->lock */
static int ipu_queue_jqs_send_allocate(struct paintbox_data *pb,
		struct paintbox_session *session, uint32_t queue_id)
{
	struct jqs_message_alloc_queue req;

	dev_dbg(pb->dev, "%s: session_id %u queue_id %u\n", __func__,
			session->session_id, queue_id);

	INIT_JQS_MSG(req, JQS_MESSAGE_TYPE_ALLOC_QUEUE);

	req.session_id = session->session_id;
	req.q_id = queue_id;

	return ipu_jqs_send_sync_message(pb, (const struct jqs_message *)&req);
}

/* The caller to this function must hold pb->lock */
static int ipu_queue_jqs_send_release(struct paintbox_data *pb,
		struct paintbox_session *session, uint32_t queue_id)
{
	struct jqs_message_free_queue req;

	dev_dbg(pb->dev, "%s: session_id %u queue_id %u\n", __func__,
			session->session_id, queue_id);

	INIT_JQS_MSG(req, JQS_MESSAGE_TYPE_FREE_QUEUE);

	req.session_id = session->session_id;
	req.q_id = queue_id;

	return ipu_jqs_send_sync_message(pb, (const struct jqs_message *)&req);
}

/* The caller to this function must hold pb->lock */
static int ipu_queue_remove_from_session(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_cmd_queue *cmd_queue)
{
	int ret = 0;

	/* Check if the session has already been removed from the session */
	if (session == NULL)
		return 0;

	if (WARN_ON(cmd_queue->session != session))
		return -EACCES;

	list_del(&cmd_queue->session_entry);
	cmd_queue->session = NULL;

	/* If the JQS is up then this a client initiated shutdown.  Notify the
	 * JQS that the queue is released from the session and unblock any
	 * threads waiting on the queue with ECONNABORTED.
	 *
	 * Otherwise, if JQS is down then report that the connection was reset.
	 */
	if (ipu_is_jqs_ready(pb->dev)) {
		ret = ipu_queue_jqs_send_release(pb, session,
				cmd_queue->queue_id);
		cmd_queue->err = -ECONNABORTED;
	} else {
		cmd_queue->err = -ECONNRESET;
	}

	/* Call down to the transport layer to remove the queue.  If the
	 * message to the JQS failed then we still want to free up the queue at
	 * the transport layer.
	 */
	ipu_free_queue(pb->dev, cmd_queue->queue_id, cmd_queue->err);

	return ret;
}

/* The caller to this function must hold pb->lock */
int ipu_queue_session_release(struct paintbox_data *pb,
		struct paintbox_session *session)
{
	struct paintbox_cmd_queue *cmd_queue, *cmd_queue_next;
	int err, ret = 0;

	list_for_each_entry_safe(cmd_queue, cmd_queue_next,
			&session->cmd_queue_list, session_entry) {
		err = ipu_queue_remove_from_session(pb, session, cmd_queue);
		if (err < 0)
			ret = err;
	}

	return ret;
}

static int ipu_queue_flush(struct file *fp, fl_owner_t id)
{
	struct paintbox_cmd_queue *cmd_queue = fp->private_data;
	struct paintbox_data *pb = cmd_queue->pb;

	mutex_lock(&pb->lock);

	ipu_flush_queue(pb->dev, cmd_queue->queue_id, -EAGAIN);

	mutex_unlock(&pb->lock);

	return 0;
}

static int ipu_queue_release(struct inode *ip, struct file *fp)
{
	struct paintbox_cmd_queue *cmd_queue = fp->private_data;
	struct paintbox_data *pb = cmd_queue->pb;
	int ret;

	mutex_lock(&pb->lock);

	ret = ipu_queue_remove_from_session(pb, cmd_queue->session, cmd_queue);

	if (WARN_ON(ret < 0)) {
		mutex_unlock(&pb->lock);
		return ret;
	}
	if (WARN_ON(cmd_queue->session)) {
		mutex_unlock(&pb->lock);
		return -EINVAL;
	}

	mutex_unlock(&pb->lock);

	kfree(cmd_queue);
	fput(fp);

	return 0;
}

static ssize_t ipu_queue_read(struct file *fp, char __user *buf, size_t size,
		loff_t *offset)
{
	struct paintbox_cmd_queue *cmd_queue = fp->private_data;
	struct paintbox_data *pb = cmd_queue->pb;

	mutex_lock(&pb->lock);
	if (ipu_reset_is_requested(pb)) {
		mutex_unlock(&pb->lock);
		return -ECONNRESET;
	}

	if (cmd_queue->session == NULL) {
		int ret = cmd_queue->err;

		mutex_unlock(&pb->lock);
		return ret;
	}

	mutex_unlock(&pb->lock);

	return ipu_user_read(pb->dev, cmd_queue->queue_id,
			(char __user *)buf, size, fp->f_flags & O_NONBLOCK);
}

static ssize_t ipu_queue_write(struct file *fp, const char __user *buf,
		size_t size, loff_t *offset)
{
	struct paintbox_cmd_queue *cmd_queue = fp->private_data;
	struct paintbox_data *pb = cmd_queue->pb;

	mutex_lock(&pb->lock);
	if (ipu_reset_is_requested(pb)) {
		mutex_unlock(&pb->lock);
		return -ECONNRESET;
	}

	if (cmd_queue->session == NULL) {
		int ret = cmd_queue->err;

		mutex_unlock(&pb->lock);
		return ret;
	}

	mutex_unlock(&pb->lock);

	return ipu_user_write(pb->dev, cmd_queue->queue_id, buf, size);
}

static long ipu_queue_ioctl(struct file *fp, unsigned int cmd,
		unsigned long arg)
{
	struct paintbox_cmd_queue *cmd_queue = fp->private_data;
	struct paintbox_data *pb = cmd_queue->pb;
	int ret;

	mutex_lock(&pb->lock);
	if (ipu_reset_is_requested(pb)) {
		mutex_unlock(&pb->lock);
		return -ECONNRESET;
	}

	if (cmd_queue->session == NULL) {
		int ret = cmd_queue->err;

		mutex_unlock(&pb->lock);
		return ret;
	}

	mutex_unlock(&pb->lock);

	switch (cmd) {
	case IPU_CMD_QUEUE_SET_EVENTFD:
		ret = ipu_user_set_eventfd(pb->dev, cmd_queue->queue_id, arg);
		break;
	case IPU_CMD_QUEUE_CLEAR_EVENTFD:
		ret = ipu_user_clear_eventfd(pb->dev, cmd_queue->queue_id);
		break;
	default:
		dev_err(pb->dev, "%s: unknown ioctl 0x%0x\n", __func__, cmd);
		return -EINVAL;
	}

	return ret;
}

static const struct file_operations ipu_queue_fops = {
	.read = ipu_queue_read,
	.write = ipu_queue_write,
	.release = ipu_queue_release,
	.unlocked_ioctl = ipu_queue_ioctl,
	.flush = ipu_queue_flush
};

int ipu_queue_allocate_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct paintbox_cmd_queue *cmd_queue;
	int ret;

	cmd_queue = kzalloc(sizeof(struct paintbox_cmd_queue), GFP_KERNEL);
	if (!cmd_queue)
		return -ENOMEM;

	mutex_lock(&pb->lock);
	if (ipu_reset_is_requested(pb)) {
		mutex_unlock(&pb->lock);
		kfree(cmd_queue);
		return -ECONNRESET;
	}

	/* call down the the transport layer to create the queue */
	ret = ipu_alloc_queue(pb->dev);
	if (ret < 0) {
		dev_err(pb->dev, "%s: queue alloc failed, queue_id %d\n",
				__func__, ret);
		goto release_queue_memory;
	}

	cmd_queue->queue_id = ret;
	cmd_queue->pb = pb;
	cmd_queue->session = session;

	/* inform jqs of new queue */
	ret = ipu_queue_jqs_send_allocate(pb, session, cmd_queue->queue_id);
	if (ret < 0)
		goto release_queue;

	/* send the file descriptor through the return status */
	ret = anon_inode_getfd("paintbox_cmd_queue", &ipu_queue_fops, cmd_queue,
			O_RDWR | O_CLOEXEC);
	if (ret < 0) {
		dev_err(pb->dev, "%s: get fd failed ret %d", __func__, ret);
		goto send_jqs_release;
	}

	/* attach the cmd_queue to the session */
	list_add_tail(&cmd_queue->session_entry, &session->cmd_queue_list);

	mutex_unlock(&pb->lock);

	return ret;

send_jqs_release:
	ipu_queue_jqs_send_release(pb, session,  cmd_queue->queue_id);
release_queue:
	ipu_free_queue(pb->dev, cmd_queue->queue_id, ret);
release_queue_memory:
	mutex_unlock(&pb->lock);
	kfree(cmd_queue);

	return ret;
}
