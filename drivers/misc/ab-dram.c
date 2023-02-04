/*
 * Airbrush DRAM Manager
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
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/ab-dram.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/ll-pool.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <uapi/linux/ab-dram.h>

/* TODO(b/116617722): Add carveout support */
#define AIRBRUSH_DRAM_START_PADDR 0x24000000
#define AIRBRUSH_DRAM_SIZE (448UL << 20)

#define MAX_ABD_SESSION 100
#define MAX_ALLOC_REF_PER_SESSION 2000

static bool initialized;

struct ab_dram_data {
	struct miscdevice dev;
	struct dentry *debug_root;
	struct mutex lock;
	struct ll_pool *pool;
	int session_count;
};

struct ab_dram_session {
	struct mutex lock;
	int alloc_count;
};

static struct ab_dram_data *internal_data;

struct ab_dram_dma_buf_attachment {
	struct device *dev;
	struct sg_table *table;
	struct list_head list;
};

/**
 * struct ab_dram_buffer - metadata for a particular buffer
 * @dev_data:		back pointer to the ab_dram_data
 * @size:		size of the buffer
 * @contiguous:		Whether the buffer is contiguous
 * @ab_paddr:		physical address in Airbrush memory space, it stores
 * 			the paddr for first segment for non-contiguous
 * 			allocation
 * @lock:		Lock used for buffer access synchronization
 * @sg_table:		the sg table for the buffer
 * @attachments:	list of dma_buf attachments for the buffer
 */
struct ab_dram_buffer {
	struct ab_dram_data *dev_data;
	struct ab_dram_session *session;
	size_t size;
	bool contiguous;
	dma_addr_t ab_paddr;
	struct mutex lock;
	struct sg_table *sg_table;
	struct list_head attachments;
};

static int ab_dram_alloc(struct ab_dram_data *dev_data,
		struct ab_dram_buffer *buffer, size_t len, bool contiguous)
{
	struct sg_table *table;

	table = ll_pool_alloc(dev_data->pool, len, contiguous);
	if (IS_ERR(table))
		return PTR_ERR(table);

	buffer->ab_paddr = sg_dma_address(table->sgl);
	buffer->sg_table = table;

	return 0;
}

static void ab_dram_free(struct ab_dram_buffer *buffer)
{
	struct ab_dram_data *dev_data = internal_data;
	struct sg_table *table;

	if (!buffer)
		return;

	table = buffer->sg_table;
	ll_pool_free(dev_data->pool, table);

	buffer->sg_table = NULL;
}

/* Increment session allocation count
 * Returning 0 on success, -EDQUOT on failure
 */
static __must_check int ab_dram_session_get(struct ab_dram_session *session)
{
	int ret = 0;

	/* NULL session means it is a kernel request */
	if (session == NULL)
		return 0;

	mutex_lock(&session->lock);

	if (session->alloc_count >= MAX_ALLOC_REF_PER_SESSION)
		ret = -EDQUOT;
	else
		session->alloc_count++;

	mutex_unlock(&session->lock);

	return ret;
}

static void ab_dram_session_put(struct ab_dram_session *session)
{
	if (session == NULL)
		return;

	mutex_lock(&session->lock);
	if (WARN_ON(--session->alloc_count < 0))
		session->alloc_count = 0;

	if (session->alloc_count == 0) {
		mutex_unlock(&session->lock);
		kfree(session);
		return;
	}
	mutex_unlock(&session->lock);
}

static struct ab_dram_buffer *ab_dram_buffer_create(
		struct ab_dram_data *dev_data, struct ab_dram_session *session,
		size_t len, bool contiguous)
{
	struct ab_dram_buffer *buffer;
	int ret;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		return ERR_PTR(-ENOMEM);

	buffer->dev_data = dev_data;
	buffer->session = session;
	buffer->size = len;
	buffer->contiguous = contiguous;
	INIT_LIST_HEAD(&buffer->attachments);
	mutex_init(&buffer->lock);

	mutex_lock(&dev_data->lock);

	ret = ab_dram_alloc(dev_data, buffer, len, contiguous);
	if (ret)
		goto err_exit;

	ret = ab_dram_session_get(session);
	if (ret < 0) {
		ab_dram_free(buffer);
		goto err_exit;
	}
	mutex_unlock(&dev_data->lock);

	return buffer;

err_exit:
	mutex_unlock(&dev_data->lock);
	kfree(buffer);
	return ERR_PTR(ret);
}

static void ab_dram_buffer_destroy(struct ab_dram_buffer *buffer)
{
	if (!buffer)
		return;

	mutex_lock(&internal_data->lock);
	ab_dram_free(buffer);
	ab_dram_session_put(buffer->session);
	mutex_unlock(&internal_data->lock);
	kfree(buffer);
}

static struct sg_table *ab_dram_map_dma_buf(
		struct dma_buf_attachment *attachment,
		enum dma_data_direction dir)
{
	struct ab_dram_dma_buf_attachment *abd_attach = attachment->priv;

	return abd_attach->table;
}

static void ab_dram_unmap_dma_buf(struct dma_buf_attachment *attachment,
			      struct sg_table *table,
			      enum dma_data_direction dir)
{
}

static void ab_dram_dma_buf_release(struct dma_buf *dmabuf)
{
	ab_dram_buffer_destroy(dmabuf->priv);
}

static struct sg_table *dup_sg_table(struct sg_table *table)
{
	struct sg_table *new_table;
	int ret, i;
	struct scatterlist *sg, *new_sg;

	new_table = kzalloc(sizeof(*new_table), GFP_KERNEL);
	if (!new_table)
		return ERR_PTR(-ENOMEM);

	ret = sg_alloc_table(new_table, table->nents, GFP_KERNEL);
	if (ret) {
		kfree(new_table);
		return ERR_PTR(-ENOMEM);
	}

	new_sg = new_table->sgl;
	for_each_sg(table->sgl, sg, table->nents, i) {
		memcpy(new_sg, sg, sizeof(*sg));
		new_sg = sg_next(new_sg);
	}

	return new_table;
}

static void free_duped_table(struct sg_table *table)
{
	sg_free_table(table);
	kfree(table);
}

static int ab_dram_dma_buf_attach(struct dma_buf *dmabuf, struct device *dev,
			      struct dma_buf_attachment *attachment)
{
	struct ab_dram_dma_buf_attachment *abd_attach;
	struct sg_table *table;
	struct ab_dram_buffer *buffer = dmabuf->priv;

	abd_attach = kzalloc(sizeof(*abd_attach), GFP_KERNEL);
	if (!abd_attach)
		return -ENOMEM;

	table = dup_sg_table(buffer->sg_table);
	if (IS_ERR(table)) {
		kfree(abd_attach);
		return -ENOMEM;
	}

	abd_attach->table = table;
	abd_attach->dev = dev;
	INIT_LIST_HEAD(&abd_attach->list);

	attachment->priv = abd_attach;

	mutex_lock(&buffer->lock);
	list_add(&abd_attach->list, &buffer->attachments);
	mutex_unlock(&buffer->lock);

	return 0;
}

static void ab_dram_dma_buf_detach(struct dma_buf *dmabuf,
				struct dma_buf_attachment *attachment)
{
	struct ab_dram_dma_buf_attachment *abd_attach = attachment->priv;
	struct ab_dram_buffer *buffer = dmabuf->priv;

	free_duped_table(abd_attach->table);

	mutex_lock(&buffer->lock);
	list_del(&abd_attach->list);
	mutex_unlock(&buffer->lock);

	kfree(abd_attach);
}

static void *ab_dram_dma_buf_kmap(struct dma_buf *dmabuf, unsigned long offset)
{
	/* TODO: Implement if needed */
	pr_err("kmap not implemented\n");
	return ERR_PTR(-ENOTTY);
}

static int ab_dram_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	/* TODO: Implement if needed */
	pr_err_ratelimited("mmap not implemented.\n");
	return -ENOTTY;
}

static const struct dma_buf_ops dma_buf_ops = {
	.map_dma_buf = ab_dram_map_dma_buf,
	.unmap_dma_buf = ab_dram_unmap_dma_buf,
	.release = ab_dram_dma_buf_release,
	.attach = ab_dram_dma_buf_attach,
	.detach = ab_dram_dma_buf_detach,
	.map = ab_dram_dma_buf_kmap,
	.map_atomic = ab_dram_dma_buf_kmap,
	.mmap = ab_dram_mmap,
};

static struct dma_buf *ab_dram_alloc_dma_buf(struct ab_dram_session *session,
		size_t len, bool contiguous)
{
	struct ab_dram_data *abd_data = internal_data;
	struct ab_dram_buffer *buffer;
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	struct dma_buf *dmabuf;

	len = PAGE_ALIGN(len);
	if (!len)
		return ERR_PTR(-EINVAL);

	buffer = ab_dram_buffer_create(abd_data, session, len, contiguous);

	if (!buffer)
		return ERR_PTR(-ENODEV);

	if (IS_ERR(buffer))
		return (void *)buffer;

	exp_info.ops = &dma_buf_ops;
	exp_info.size = buffer->size;
	exp_info.flags = O_RDWR;
	exp_info.priv = buffer;

	dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR(dmabuf)) {
		ab_dram_buffer_destroy(buffer);
		return dmabuf;
	}

	return dmabuf;
}

struct dma_buf *ab_dram_alloc_dma_buf_kernel(size_t len)
{
	if (!initialized)
		return ERR_PTR(-ENOENT);

	/* Session field set to NULL since kernel has no session */
	return ab_dram_alloc_dma_buf(NULL, len, true /* contiguous */);
}
EXPORT_SYMBOL(ab_dram_alloc_dma_buf_kernel);

void ab_dram_free_dma_buf_kernel(struct dma_buf *dmabuf)
{
	if (!dmabuf)
		return;

	dma_buf_put(dmabuf);
}
EXPORT_SYMBOL(ab_dram_free_dma_buf_kernel);

dma_addr_t ab_dram_get_dma_buf_paddr(struct dma_buf *dmabuf)
{
	struct ab_dram_buffer *buffer;

	if (!dmabuf)
		return 0;

	if (!dmabuf->priv)
		return 0;

	buffer = dmabuf->priv;
	return buffer->ab_paddr;
}
EXPORT_SYMBOL(ab_dram_get_dma_buf_paddr);

bool is_ab_dram_dma_buf(struct dma_buf *dmabuf)
{
	if (!dmabuf)
		return false;

	return (strcmp(KBUILD_MODNAME, dmabuf->exp_name) == 0);
}
EXPORT_SYMBOL(is_ab_dram_dma_buf);

static int ab_dram_allocate_memory_fd_internal(struct ab_dram_session *session,
		size_t len, bool contiguous)
{
	int fd;
	struct dma_buf *dmabuf;

	dmabuf = ab_dram_alloc_dma_buf(session, len, contiguous);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	fd = dma_buf_fd(dmabuf, O_CLOEXEC);
	if (fd < 0)
		dma_buf_put(dmabuf);

	return fd;
}

static int ab_dram_allocate_memory_fd(struct ab_dram_session *session,
		unsigned long arg)
{
	struct ab_dram_alloc_request __user *user_req;
	struct ab_dram_alloc_request req;

	user_req = (struct ab_dram_alloc_request __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	return ab_dram_allocate_memory_fd_internal(session, req.size,
			req.flag == ABD_ALLOC_CONTIGUOUS ? true : false);
}

static int ab_dram_open(struct inode *ip, struct file *fp)
{
	struct ab_dram_session *session;

	session = kzalloc(sizeof(struct ab_dram_session), GFP_KERNEL);
	if (!session)
		return -ENOMEM;

	mutex_lock(&internal_data->lock);
	if (internal_data->session_count >= MAX_ABD_SESSION) {
		mutex_unlock(&internal_data->lock);
		kfree(session);
		return -EMFILE;
	}
	internal_data->session_count++;
	mutex_unlock(&internal_data->lock);

	mutex_init(&session->lock);

	/* session allocation count equals to 1 refcount from the
	 * opened driver session plus the number of allocation on
	 * the said session. Therefore initialize alloc_count to 1
	 * for the opened driver session to start with.
	 */
	session->alloc_count = 1;

	fp->private_data = session;
	return 0;
}

static int ab_dram_release(struct inode *ip, struct file *fp)
{
	struct ab_dram_session *session = fp->private_data;

	mutex_lock(&internal_data->lock);
	if (WARN_ON(--internal_data->session_count < 0))
		internal_data->session_count = 0;
	mutex_unlock(&internal_data->lock);

	ab_dram_session_put(session);
	fp->private_data = NULL;
	return 0;
}

static long ab_dram_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	struct ab_dram_session *session = filp->private_data;
	int ret = 0;

	switch (cmd) {
	case AB_DRAM_ALLOCATE_MEMORY_LEGACY:
		ret = ab_dram_allocate_memory_fd_internal(session, (size_t)arg,
				true);
		break;
	case AB_DRAM_ALLOCATE_MEMORY:
		ret = ab_dram_allocate_memory_fd(session, arg);
		break;
	default:
		pr_err("Invalid ioctl command.\n");
		return -EINVAL;
	}

	return ret;
}

static const struct file_operations ab_dram_fops = {
	.owner          = THIS_MODULE,
	.open		= ab_dram_open,
	.release	= ab_dram_release,
	.unlocked_ioctl = ab_dram_ioctl,
};

static int ab_dram_device_create(void)
{
	struct ab_dram_data *dev_data;
	int ret;

	dev_data = kzalloc(sizeof(*dev_data), GFP_KERNEL);
	if (!dev_data)
		return -ENOMEM;

	dev_data->dev.minor = MISC_DYNAMIC_MINOR;
	dev_data->dev.name = "ab-dram";
	dev_data->dev.fops = &ab_dram_fops;
	dev_data->dev.parent = NULL;

	ret = misc_register(&dev_data->dev);
	if (ret) {
		pr_err("failed to register misc device.\n");
		goto err_init;
	}

	mutex_init(&dev_data->lock);
	dev_data->pool = ll_pool_create(AIRBRUSH_DRAM_START_PADDR,
			AIRBRUSH_DRAM_SIZE);
	if (IS_ERR(dev_data->pool)) {
		pr_err("%s: err creating remote buddy pool\n", __func__);
		ret = PTR_ERR(dev_data->pool);
		goto unregister_misc;
	}

	internal_data = dev_data;
	initialized = true;
	return 0;

unregister_misc:
	misc_deregister(&dev_data->dev);
err_init:
	kfree(dev_data);
	return ret;
}

static void __exit ab_dram_exit(void)
{
	misc_deregister(&internal_data->dev);
	initialized = false;
	ll_pool_destroy(internal_data->pool);
	mutex_destroy(&internal_data->lock);
	kfree(internal_data);
}

subsys_initcall(ab_dram_device_create);
module_exit(ab_dram_exit);

MODULE_AUTHOR("Google, Inc.");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Paintbox Airbrush DRAM Manager");
