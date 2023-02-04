/*
 * Debug support for the Paintbox programmable IPU
 *
 * Copyright (C) 2016 Google, Inc.
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

#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/ipu-core.h>
#include <linux/kernel.h>
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "ipu-aon-debug.h"
#include "ipu-apb-debug.h"
#include "ipu-bif-debug.h"
#include "ipu-client.h"
#include "ipu-debug.h"
#include "ipu-dma-debug.h"
#include "ipu-lbp-debug.h"
#include "ipu-regs.h"
#include "ipu-stp-debug.h"

int ipu_debug_vprintf(struct paintbox_data *pb, char *buf, int *written,
		size_t len, const char *format, va_list args)
{
	int ret = 0;

	if (buf && written) {
		ret = vsnprintf(buf + *written, len - *written, format, args);

		if (ret < 0) {
			dev_err(pb->dev, "%s: dump printf error, err = %d",
					__func__, ret);
			return ret;
		}

		*written += ret;
	} else {
		struct va_format vaf;

		vaf.fmt = format;
		vaf.va = &args;

		/* TODO(b/115430771):  Add support for levels */

		pr_info("%pV", &vaf);
	}

	return ret;
}

int ipu_debug_printf(struct paintbox_data *pb, char *buf, int *written,
		size_t len, const char *format, ...)
{
	va_list args;
	int ret;

	va_start(args, format);

	ret = ipu_debug_vprintf(pb, buf, written, len, format, args);

	va_end(args);

	if (ret < 0) {
		dev_err(pb->dev, "%s: dump printf error, err = %d",
				__func__, ret);
		return ret;
	}

	return ret;
}

int ipu_debug_dump_register(struct paintbox_data *pb, uint32_t group_base,
		uint32_t reg_offset, const char *reg_name, char *buf,
		int *written, size_t len)
{
	return ipu_debug_printf(pb, buf, written, len,
			"0x%04lx: %-*s0x%016llx\n", group_base + reg_offset,
			REG_VALUE_COLUMN_NUMBER - REG_NAME_COLUMN_NUMBER,
			reg_name ? reg_name : REG_UNUSED,
			ipu_readq(pb->dev, group_base + reg_offset));
}

int ipu_debug_dump_register_with_value(struct paintbox_data *pb,
		uint32_t group_base, uint32_t reg_offset, uint64_t reg_value,
		const char *reg_name, char *buf, int *written, size_t len)
{
	return ipu_debug_printf(pb, buf, written, len,
			"0x%04lx: %-*s0x%016llx\n", group_base + reg_offset,
			REG_VALUE_COLUMN_NUMBER - REG_NAME_COLUMN_NUMBER,
			reg_name ? reg_name : REG_UNUSED, reg_value);
}

static int ipu_debug_stats_show(struct seq_file *s, void *unused)
{
	struct paintbox_debug *debug = s->private;
	struct paintbox_data *pb = debug->pb;
	char *buf;
	size_t len;
	int ret = 0, written;

	len = seq_get_buf(s, &buf);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&pb->lock);
	if (ipu_reset_is_requested(pb)) {
		mutex_unlock(&pb->lock);
		return -ECONNRESET;
	}

	if (!debug->stats_dump) {
		mutex_unlock(&pb->lock);
		return 0;
	}

	if (debug->resource_id >= 0)
		written = scnprintf(buf, len, "%s%u: ", debug->name,
				debug->resource_id);
	else
		written = scnprintf(buf, len, "%s: ", debug->name);
	if (written < 0) {
		mutex_unlock(&pb->lock);
		dev_err(pb->dev, "%s: error dumping registers %d", __func__,
				ret);
		return written;
	}

	ret = debug->stats_dump(debug, buf + written, len - written);

	mutex_unlock(&pb->lock);

	if (ret < 0)
		return ret;

	written += ret;

	seq_commit(s, written);

	return 0;
}

static int ipu_debug_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, ipu_debug_stats_show, inode->i_private);
}

static const struct file_operations ipu_debug_stats_fops = {
	.open = ipu_debug_stats_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ipu_debug_regs_show(struct seq_file *s, void *unused)
{
	struct paintbox_debug *debug = s->private;
	struct paintbox_data *pb = debug->pb;
	char *buf;
	size_t len;
	int ret, written;

	len = seq_get_buf(s, &buf);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&pb->lock);
	if (ipu_reset_is_requested(pb)) {
		mutex_unlock(&pb->lock);
		return -ECONNRESET;
	}

	if (debug->resource_id >= 0)
		ret = scnprintf(buf, len, "%s%u:\n", debug->name,
				debug->resource_id);
	else
		ret = scnprintf(buf, len, "%s:\n", debug->name);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		dev_err(pb->dev, "%s: error dumping registers %d", __func__,
				ret);
		return ret;
	}

	written = ret;

	ret = ipu_jqs_get(pb);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	ret = debug->register_dump(debug, buf + written, len - written);

	(void)ipu_jqs_put(pb);

	mutex_unlock(&pb->lock);

	if (ret < 0)
		return ret;

	written += ret;

	seq_commit(s, written);

	return 0;
}

static int ipu_debug_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, ipu_debug_regs_show, inode->i_private);
}

static const struct file_operations ipu_debug_regs_fops = {
	.open = ipu_debug_regs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void ipu_debug_free_entry(struct paintbox_debug *debug)
{
	debugfs_remove(debug->stats_dump_dentry);
	debugfs_remove(debug->reg_dump_dentry);
	debugfs_remove(debug->debug_dir);
}

void ipu_debug_create_entry(struct paintbox_data *pb,
		struct paintbox_debug *debug, struct dentry *debug_root,
		const char *name, unsigned int resource_id,
		register_dump_t register_dump, stats_dump_t stats_dump,
		void *arg)
{
	char resource_name[RESOURCE_NAME_LEN];
	int ret;

	debug->pb = pb;
	debug->name = name;
	debug->resource_id = resource_id;
	debug->register_dump = register_dump;
	debug->stats_dump = stats_dump;

	if (debug->resource_id >= 0)
		ret = scnprintf(resource_name, RESOURCE_NAME_LEN, "%s%u", name,
				resource_id);
	else
		ret = scnprintf(resource_name, RESOURCE_NAME_LEN, "%s", name);
	if (ret < 0) {
		dev_err(pb->dev, "%s: error creating debugs entry %d\b",
				__func__, ret);
		return;
	}

	debug->debug_dir = debugfs_create_dir(resource_name, debug_root);
	if (IS_ERR(debug->debug_dir)) {
		dev_err(pb->dev, "%s: err = %ld", __func__,
				PTR_ERR(debug->debug_dir));
		return;
	}

	debug->reg_dump_dentry = debugfs_create_file("regs", 0640,
			debug->debug_dir, debug, &ipu_debug_regs_fops);
	if (IS_ERR(debug->reg_dump_dentry)) {
		dev_err(pb->dev, "%s: err = %ld", __func__,
				PTR_ERR(debug->reg_dump_dentry));
		return;
	}

	debug->stats_dump_dentry = debugfs_create_file("stats", 0640,
			debug->debug_dir, debug, &ipu_debug_stats_fops);
	if (IS_ERR(debug->stats_dump_dentry)) {
		dev_err(pb->dev, "%s: err = %ld", __func__,
				PTR_ERR(debug->stats_dump_dentry));
		return;
	}
}

static int ipu_debug_reg_entry_set(void *data, u64 val)
{
	struct paintbox_debug_reg_entry *reg_entry = data;
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_data *pb = debug->pb;
	int ret;

	mutex_lock(&pb->lock);
	if (ipu_reset_is_requested(pb)) {
		mutex_unlock(&pb->lock);
		return -ECONNRESET;
	}

	ret = ipu_jqs_get(pb);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	reg_entry->write(reg_entry, val);

	ret = ipu_jqs_put(pb);

	mutex_unlock(&pb->lock);

	return ret;
}

static int ipu_debug_reg_entry_get(void *data, u64 *val)
{
	struct paintbox_debug_reg_entry *reg_entry = data;
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_data *pb = debug->pb;
	int ret;

	mutex_lock(&pb->lock);
	if (ipu_reset_is_requested(pb)) {
		mutex_unlock(&pb->lock);
		return -ECONNRESET;
	}

	ret = ipu_jqs_get(pb);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	*val = reg_entry->read(reg_entry);

	ret = ipu_jqs_put(pb);

	mutex_unlock(&pb->lock);

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(ipu_debug_reg_entry_fops, ipu_debug_reg_entry_get,
			ipu_debug_reg_entry_set, "%llx\n");

int ipu_debug_alloc_reg_entries(struct paintbox_data *pb,
		struct paintbox_debug *debug, size_t reg_count)
{
	debug->reg_entries = kcalloc(reg_count,
			sizeof(struct paintbox_debug_reg_entry), GFP_KERNEL);
	if (!debug->reg_entries) {
		debug->num_reg_entries = 0;
		return -ENOMEM;
	}

	debug->num_reg_entries = reg_count;
	debug->pb = pb;

	return 0;
}

void ipu_debug_free_reg_entries(struct paintbox_debug *debug)
{
	unsigned int i;

	for (i = 0; i < debug->num_reg_entries; i++)
		debugfs_remove(debug->reg_entries[i].debug_dentry);

	kfree(debug->reg_entries);

	debug->reg_entries = NULL;
	debug->num_reg_entries = 0;
}

int ipu_debug_create_reg_entry(struct paintbox_data *pb,
		struct paintbox_debug *debug, unsigned int index,
		const char *reg_name, uint32_t reg_offset,
		register_write_t reg_write, register_read_t reg_read)
{
	struct paintbox_debug_reg_entry *reg_entry = &debug->reg_entries[index];

	reg_entry->debug_dentry = debugfs_create_file(reg_name, 0640,
			debug->debug_dir, reg_entry, &ipu_debug_reg_entry_fops);
	if (IS_ERR(reg_entry->debug_dentry)) {
		dev_err(pb->dev, "%s: err = %ld", __func__,
				PTR_ERR(reg_entry->debug_dentry));
		return PTR_ERR(reg_entry->debug_dentry);
	}

	reg_entry->reg_offset = reg_offset;
	reg_entry->write = reg_write;
	reg_entry->read = reg_read;
	reg_entry->debug = debug;

	return 0;
}

void ipu_debug_create_reg_entries(struct paintbox_data *pb,
		struct paintbox_debug *debug, const char **reg_names,
		size_t reg_count, register_write_t reg_write,
		register_read_t reg_read)
{
	int i, ret;

	ret = ipu_debug_alloc_reg_entries(pb, debug, reg_count);
	if (ret < 0)
		return;

	for (i = 0; i < reg_count; i++) {
		if (!reg_names[i])
			continue;

		ret = ipu_debug_create_reg_entry(pb, debug, i,
				reg_names[i], i * IPU_REG_WIDTH, reg_write,
				reg_read);
		if (ret < 0) {
			ipu_debug_free_reg_entries(debug);
			return;
		}
	}
}

static int ipu_debug_reg_dump_show(struct seq_file *s, void *unused)
{
	struct paintbox_data *pb = s->private;
	unsigned int i, j;
	char *buf;
	size_t len;
	int ret, written = 0;

	len = seq_get_buf(s, &buf);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&pb->lock);
	if (ipu_reset_is_requested(pb)) {
		mutex_unlock(&pb->lock);
		return -ECONNRESET;
	}

	ret = ipu_jqs_get(pb);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	for (i = 0; i < pb->lbp.num_lbps; i++) {
		ret = ipu_lbp_dump_registers(&pb->lbp.lbps[i].debug,
				buf + written, len - written);
		if (ret < 0)
			goto err_exit;

		written += ret;

		for (j = 0; j < pb->lbp.max_lbs; j++) {
			ret = ipu_lb_dump_registers(
					&pb->lbp.lbps[i].lbs[j].debug,
					buf + written, len - written);
			if (ret < 0)
				goto err_exit;

			written += ret;
		}
	}

	ret = ipu_jqs_put(pb);

	mutex_unlock(&pb->lock);

	seq_commit(s, written);

	return ret;

err_exit:
	(void)ipu_jqs_put(pb);

	mutex_unlock(&pb->lock);
	dev_err(pb->dev, "%s: register dump error, err = %d", __func__, ret);
	return ret;
}

static int ipu_debug_reg_dump_open(struct inode *inode, struct file *file)
{
	struct paintbox_data *pb = inode->i_private;
	size_t len;

	len = IO_APB_NUM_REGS * REG_DEBUG_BUFFER_SIZE;
	len += IO_AXI_NUM_REGS * REG_DEBUG_BUFFER_SIZE;
	len += pb->dma.num_channels * DMA_DEBUG_BUFFER_SIZE;
	len += pb->lbp.num_lbps * LBP_NUM_REGS * REG_DEBUG_BUFFER_SIZE;
	len += pb->lbp.num_lbps * pb->lbp.max_lbs * LB_NUM_REGS *
			REG_DEBUG_BUFFER_SIZE;

	return single_open_size(file, ipu_debug_reg_dump_show, inode->i_private,
			len);
}

static const struct file_operations ipu_debug_reg_dump_fops = {
	.open = ipu_debug_reg_dump_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void ipu_debug_init(struct paintbox_data *pb)
{
	pb->debug_root = ipu_get_debug_root(pb->dev);

	pb->regs_dentry = debugfs_create_file("regs", 0640, pb->debug_root, pb,
			&ipu_debug_reg_dump_fops);
	if (IS_ERR(pb->regs_dentry)) {
		dev_err(pb->dev, "%s: err = %ld", __func__,
				PTR_ERR(pb->regs_dentry));
		return;
	}
}

void ipu_debug_remove(struct paintbox_data *pb)
{
	debugfs_remove(pb->regs_dentry);
}
