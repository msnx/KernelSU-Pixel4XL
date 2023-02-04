/*
 * Android Airbrush coprocessor DMA library
 *
 * Copyright 2019 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/dma-direction.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/mfd/abc-pcie-dma.h>
#include <uapi/linux/abc-pcie-dma.h>

/**
 * Converts and validates a user dma descriptor into a kernel
 * abc_dma_xfer structure.
 */
static int abc_pcie_convert_user_to_kernel_desc(struct device *dev,
					struct abc_pcie_dma_desc *udesc,
					struct abc_pcie_kernel_dma_desc *kdesc)
{
	kdesc->size = udesc->size;

	switch (udesc->dir) {
	case ABC_DMA_TO_DEVICE:
		kdesc->dir = DMA_TO_DEVICE;
		break;
	case ABC_DMA_FROM_DEVICE:
		kdesc->dir = DMA_FROM_DEVICE;
		break;
	default:
		dev_err(dev, "%s: Invalid direction specification\n", __func__);
		return -EINVAL;
	}

	if (udesc->local_buf_type == DMA_BUFFER_USER) {
		kdesc->local_buf_kind = DMA_BUFFER_KIND_USER;
		kdesc->local_buf = udesc->local_buf;
	} else if (udesc->local_buf_type == DMA_BUFFER_DMA_BUF) {
		kdesc->local_buf_kind = DMA_BUFFER_KIND_DMA_BUF;
		kdesc->local_dma_buf_fd = udesc->local_dma_buf_fd;
		kdesc->local_dma_buf_off = udesc->local_dma_buf_off;
	} else {
		dev_err(dev, "%s: Unable to convert descriptor\n", __func__);
		return -EINVAL;
	}

	if (udesc->remote_buf_type == DMA_BUFFER_USER) {
		dev_err(dev, "%s: Operation not allowed\n", __func__);
		return -EINVAL;
	} else if (udesc->remote_buf_type == DMA_BUFFER_DMA_BUF) {
		kdesc->remote_buf_kind = DMA_BUFFER_KIND_DMA_BUF;
		kdesc->remote_dma_buf_fd = udesc->remote_dma_buf_fd;
		kdesc->remote_dma_buf_off = udesc->remote_dma_buf_off;
	} else {
		dev_err(dev, "%s: Unable to convert descriptor\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static int abc_pcie_dma_user_xfer_start(struct device *dev,
				struct abc_pcie_dma_session *session,
				struct abc_pcie_dma_desc_start *desc);

static int abc_pcie_dma_user_xfer_wait(struct device *dev,
				struct abc_pcie_dma_session *session,
				struct abc_pcie_dma_desc_wait *wait_desc);

static int abc_pcie_dma_user_xfer_clean(struct device *dev,
				struct abc_pcie_dma_session *session,
				uint64_t id);

static int abc_prepare_and_send_usr_sync_xfer(struct device *dev,
				struct abc_pcie_dma_session *session,
				struct abc_pcie_dma_desc *user_desc)
{
	int err = 0;
	uint64_t id;
	struct abc_pcie_kernel_dma_desc kernel_desc;
	struct abc_pcie_dma_desc_start start_desc;
	struct abc_pcie_dma_desc_wait wait_desc;

	err = abc_pcie_convert_user_to_kernel_desc(dev, user_desc,
							&kernel_desc);
	if (err)
		goto exit_sync;

	err = abc_pcie_create_dma_xfer(session, &kernel_desc, NULL, &id);
	if (err)
		goto exit_sync;

	start_desc.id = id;
	err = abc_pcie_dma_user_xfer_start(dev, session, &start_desc);
	if (err)
		goto clean_and_exit;

	wait_desc.id = id;
	wait_desc.timeout = -1;
	err = abc_pcie_dma_user_xfer_wait(dev, session, &wait_desc);
	if (!err && wait_desc.error)
		err = wait_desc.error;

clean_and_exit:
	abc_pcie_dma_user_xfer_clean(dev, session, id);

exit_sync:
	if (err)
		dev_err(dev, "%s: failed to perform DMA (%d)\n", __func__, err);

	return err;
}

static void dma_legacy_desc_conversion(
				struct abc_pcie_dma_desc_legacy *legacy_desc,
				struct abc_pcie_dma_desc *desc)
{
	desc->local_buf_type = legacy_desc->local_buf_type;
	/* abc_pcie_issue_dma_xfer will catch invalid buffer types */
	if (desc->local_buf_type == DMA_BUFFER_USER)
		desc->local_buf = legacy_desc->local_buf;
	else {
		desc->local_dma_buf_fd = legacy_desc->local_dma_buf_fd;
		desc->local_dma_buf_off = 0;
	}
	desc->remote_buf_type = legacy_desc->remote_buf_type;
	if (desc->remote_buf_type == DMA_BUFFER_USER)
		desc->remote_buf = legacy_desc->remote_buf;
	else {
		desc->remote_dma_buf_fd = legacy_desc->remote_dma_buf_fd;
		desc->remote_dma_buf_off = 0;
	}
	desc->size = legacy_desc->local_buf_size;
	desc->dir = legacy_desc->dir;
}

static int abc_pcie_dma_user_xfer_legacy(struct device *dev,
					struct abc_pcie_dma_session *session,
					unsigned long arg)
{
	struct abc_pcie_dma_desc_legacy dma_desc_legacy;
	struct abc_pcie_dma_desc dma_desc;

	dev_dbg(dev, "%s: Received IOCTL for legacy synchronous DMA request\n",
		 __func__);
	if (copy_from_user(&dma_desc_legacy, (void __user *)arg,
		sizeof(dma_desc_legacy))) {
		dev_err(dev, "%s: failed to copy from userspace\n", __func__);
		return -EFAULT;
	}
	dma_legacy_desc_conversion(&dma_desc_legacy, &dma_desc);
	return abc_prepare_and_send_usr_sync_xfer(dev, session, &dma_desc);
}

static int abc_pcie_dma_user_xfer_sync(struct device *dev,
					struct abc_pcie_dma_session *session,
					unsigned long arg)
{
	struct abc_pcie_dma_desc dma_desc;
	dev_dbg(dev, "%s: Received IOCTL for synchronous DMA request\n",
		__func__);

	if (copy_from_user(&dma_desc, (void __user *)arg, sizeof(dma_desc))) {
		dev_err(dev, "%s: failed to copy from userspace\n", __func__);
		return -EFAULT;
	}
	return abc_prepare_and_send_usr_sync_xfer(dev, session, &dma_desc);
}

static int abc_pcie_dma_user_ioctl_xfer_create(struct device *dev,
					struct abc_pcie_dma_session *session,
					unsigned long arg)
{
	int err = 0;
	struct abc_pcie_dma_desc_async async_desc;
	struct abc_pcie_kernel_dma_desc kernel_desc;
	uint64_t id;

	dev_dbg(dev, "%s: Received IOCTL for async create request\n", __func__);
	if (copy_from_user(&async_desc, (void __user *)arg,
				sizeof(async_desc))) {
		dev_err(dev, "%s: failed to copy from userspace\n", __func__);
		return -EFAULT;
	}

	err = abc_pcie_convert_user_to_kernel_desc(dev, &async_desc.dma_desc,
							&kernel_desc);
	if (err)
		return err;

	err = abc_pcie_create_dma_xfer(session, &kernel_desc, NULL, &id);
	if (err)
		return err;

	async_desc.id = id;

	if (copy_to_user((void __user *)arg, &async_desc, sizeof(async_desc))) {
		abc_pcie_dma_user_xfer_clean(dev, session, id);
		dev_err(dev, "%s: failed to copy to userspace\n", __func__);
		return -EFAULT;
	}

	return 0;
}

static int abc_pcie_dma_user_xfer_start(struct device *dev,
					struct abc_pcie_dma_session *session,
					struct abc_pcie_dma_desc_start *desc)
{
	int err = 0;
	struct abc_dma_xfer *xfer = NULL;
	struct abc_pcie_dma *abc_dma = (session->uapi)->abc_dma;

	down_read(&abc_dma->state_transition_rwsem);
	if (!abc_dma->pcie_link_up || (abc_dma->dram_state != AB_DMA_DRAM_UP)) {
		up_read(&abc_dma->state_transition_rwsem);
		dev_err(dev,
			"DMA is not active: pcie_link:%s dram_state:%s\n",
			abc_dma->pcie_link_up ? "Up" : "Down",
			dram_state_str(abc_dma->dram_state));
		return -EREMOTEIO;
	}

	mutex_lock(&session->lock);
	xfer = abc_pcie_dma_find_xfer(session, desc->id);
	if (!xfer) {
		mutex_unlock(&session->lock);
		up_read(&abc_dma->state_transition_rwsem);
		dev_err(dev, "%s: Could not find xfer id:%0llu\n", __func__,
			desc->id);
		return -EINVAL;
	}

	if (xfer->poisoned) {
		err = -EREMOTEIO;
		dev_err(dev, "%s: Transfer (id:%0llu) has been poisoned.\n",
			__func__, desc->id);
	} else {
		err = abc_pcie_start_dma_xfer_locked(xfer, &desc->start_id);
	}
	mutex_unlock(&session->lock);
	up_read(&abc_dma->state_transition_rwsem);

	return err;
}

static int abc_pcie_dma_user_ioctl_xfer_start(struct device *dev,
					struct abc_pcie_dma_session *session,
					unsigned long arg)
{
	int err = 0;
	struct abc_pcie_dma_desc_start desc;

	dev_dbg(dev, "%s: Received IOCTL for start request\n", __func__);
	if (copy_from_user(&desc, (void __user *)arg, sizeof(desc))) {
		dev_err(dev, "%s: failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	err = abc_pcie_dma_user_xfer_start(dev, session, &desc);
	if (err)
		return err;

	if (copy_to_user((void __user *)arg, &desc, sizeof(desc))) {
		dev_err(dev, "%s: failed to copy to userspace\n", __func__);
		return -EFAULT;
	}

	return 0;
}

static int abc_pcie_dma_user_xfer_wait(struct device *dev,
				struct abc_pcie_dma_session *session,
				struct abc_pcie_dma_desc_wait *wait_desc)
{
	int err = 0;
	struct abc_dma_wait_info *wait_info;

	err = abc_pcie_dma_get_user_wait(wait_desc->id, session, &wait_info);
	if (err)
		return err;

	err = abc_pcie_dma_do_wait(session, wait_info, wait_desc->timeout,
				&wait_desc->error, &wait_desc->start_id);

	return err;
}

static int abc_pcie_dma_user_ioctl_xfer_wait(struct device *dev,
					struct abc_pcie_dma_session *session,
					unsigned long arg)
{
	int err = 0;
	struct abc_pcie_dma_desc_wait wait_desc;

	dev_dbg(dev, "%s: Received IOCTL for wait request\n", __func__);
	if (copy_from_user(&wait_desc, (void __user *)arg, sizeof(wait_desc))) {
		dev_err(dev, "%s: failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	err = abc_pcie_dma_user_xfer_wait(dev, session, &wait_desc);
	if (err)
		return err;

	if (copy_to_user((void __user *)arg, &wait_desc, sizeof(wait_desc))) {
		dev_err(dev, "%s: failed to copy to userspace\n", __func__);
		return -EFAULT;
	}

	return err;
}

static int abc_pcie_dma_user_xfer_clean(struct device *dev,
					struct abc_pcie_dma_session *session,
					uint64_t id)
{
	int err = 0;
	struct abc_dma_xfer *xfer = NULL;
	struct abc_pcie_dma *abc_dma = (session->uapi)->abc_dma;

	down_read(&abc_dma->state_transition_rwsem);
	mutex_lock(&session->lock);
	xfer = abc_pcie_dma_find_xfer(session, id);
	if (!xfer) {
		err = -EINVAL;
		dev_err(dev, "%s: Could not find xfer id:%0llu\n", __func__,
			id);
	} else {
		abc_pcie_clean_dma_xfer_locked(xfer);
	}
	mutex_unlock(&session->lock);
	up_read(&abc_dma->state_transition_rwsem);
	return err;
}


static int abc_pcie_dma_user_ioctl_xfer_clean(struct device *dev,
					struct abc_pcie_dma_session *session,
					unsigned long arg)
{
	int err = 0;
	uint64_t id = (uint64_t)arg;

	dev_dbg(dev, "%s: Received IOCTL for clean request\n", __func__);
	err = abc_pcie_dma_user_xfer_clean(dev, session, id);

	return err;
}

int abc_pcie_dma_open(struct inode *inode, struct file *filp)
{
	int err = 0;
	struct abc_pcie_dma_session *session;
	struct abc_pcie_dma_uapi *uapi;
	struct miscdevice *mdev;

	mdev = (struct miscdevice *)filp->private_data;
	uapi = container_of(mdev, struct abc_pcie_dma_uapi, mdev);

	session = kzalloc(sizeof(struct abc_pcie_dma_session), GFP_KERNEL);
	if (!session)
		return -ENOMEM;

	err = abc_pcie_dma_open_session(session);
	if (err)
		return err;
	filp->private_data = session;
	return 0;
}

int abc_pcie_dma_release(struct inode *inode, struct file *filp)
{
	struct abc_pcie_dma_session *session;

	session = (struct  abc_pcie_dma_session *)filp->private_data;
	abc_pcie_dma_close_session(session);
	kfree(session);
	return 0;
}

/* IOCTL interface */
long abc_pcie_dma_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	int err = 0;
	struct device *dev;
	struct abc_pcie_dma_session *session;

	session = (struct abc_pcie_dma_session *)file->private_data;
	dev = (session->uapi)->mdev.parent;


	if (_IOC_TYPE(cmd) != ABC_PCIE_DMA_IOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case ABC_PCIE_DMA_IOC_POST_DMA_XFER_LEGACY:
		err = abc_pcie_dma_user_xfer_legacy(dev, session, arg);
		break;
	case ABC_PCIE_DMA_IOC_POST_DMA_XFER_SYNC:
		err = abc_pcie_dma_user_xfer_sync(dev, session, arg);
		break;
	case ABC_PCIE_DMA_IOC_POST_DMA_XFER_CREATE:
		err = abc_pcie_dma_user_ioctl_xfer_create(dev, session, arg);
		break;
	case ABC_PCIE_DMA_IOC_POST_DMA_XFER_START:
		err = abc_pcie_dma_user_ioctl_xfer_start(dev, session, arg);
		break;
	case ABC_PCIE_DMA_IOC_POST_DMA_XFER_WAIT:
		err = abc_pcie_dma_user_ioctl_xfer_wait(dev, session, arg);
		break;
	case ABC_PCIE_DMA_IOC_POST_DMA_XFER_CLEAN:
		err = abc_pcie_dma_user_ioctl_xfer_clean(dev, session, arg);
		break;
	default:
		dev_err(dev,
			"%s: unknown ioctl %c, dir=%d, #%d (0x%08x)\n",
			__func__, _IOC_TYPE(cmd), _IOC_DIR(cmd), _IOC_NR(cmd),
			cmd);
		break;
	}

	return err;
}

static const struct file_operations abc_dma_fops = {
	.owner = THIS_MODULE,
	.open = abc_pcie_dma_open,
	.release = abc_pcie_dma_release,
	.unlocked_ioctl = abc_pcie_dma_ioctl,
	.llseek = no_llseek,
};

int init_abc_pcie_dma_uapi(struct abc_pcie_dma_uapi *uapi)
{
	int err = 0;

	uapi->mdev.minor = MISC_DYNAMIC_MINOR;
	uapi->mdev.name = DRV_NAME_ABC_PCIE_DMA;
	uapi->mdev.fops = &abc_dma_fops;

	err = misc_register(&uapi->mdev);
	if (err) {
		dev_err((uapi->mdev).parent, "misc_register failed\n");
		return err;
	}

	return 0;
}

void remove_abc_pcie_dma_uapi(struct abc_pcie_dma_uapi *uapi)
{
	misc_deregister(&uapi->mdev);
}
