/*
 * IO support for the Paintbox programmable IPU
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
#include <linux/kernel.h>
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>
#include <linux/types.h>

#include "ipu-apb-debug.h"
#include "ipu-client.h"
#include "ipu-debug.h"
#include "ipu-regs.h"

static const char *ipu_apb_reg_names[IO_APB_NUM_REGS] = {
	REG_NAME_ENTRY64(IPU_ISR),
	REG_NAME_ENTRY64(IPU_ITR),
	REG_NAME_ENTRY64(IPU_IER),
	REG_NAME_ENTRY64(DMA_CHAN_ISR),
	REG_NAME_ENTRY64(DMA_CHAN_ITR),
	REG_NAME_ENTRY64(DMA_CHAN_IER),
	REG_NAME_ENTRY64(DMA_CHAN_IMR),
	REG_NAME_ENTRY64(DMA_ERR_ISR),
	REG_NAME_ENTRY64(DMA_ERR_IMR),
	REG_NAME_ENTRY64(DMA_ERR_IER),
	REG_NAME_ENTRY64(DMA_ERR_ITR),
	REG_NAME_ENTRY64(IPU_STP_ISR),
	REG_NAME_ENTRY64(IPU_STP_ITR),
	REG_NAME_ENTRY64(IPU_STP_IER),
	REG_NAME_ENTRY64(IPU_STP_IMR),
	REG_NAME_ENTRY64(STP_ERR_ISR),
	REG_NAME_ENTRY64(STP_ERR_IMR),
	REG_NAME_ENTRY64(STP_ERR_IER),
	REG_NAME_ENTRY64(STP_ERR_ITR),
	REG_NAME_ENTRY64(IPU_STP_GRP_SEL),
};

static int ipu_apb_debug_read_registers(struct seq_file *s, void *data)
{
	struct paintbox_data *pb = dev_get_drvdata(s->private);
	unsigned int i;
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

	for (i = 0; i < IO_APB_NUM_REGS; i++) {
		unsigned int offset;

		if (ipu_apb_reg_names[i] == NULL)
			continue;

		offset = i * IPU_REG_WIDTH_BYTES + IPU_CSR_APB_OFFSET;

		seq_printf(s, "0x%04lx: %-*s0x%016llx\n", offset,
				REG_VALUE_COL_WIDTH, ipu_apb_reg_names[i],
				ipu_readq(pb->dev, offset));
	}

	ret = ipu_jqs_put(pb);

	mutex_unlock(&pb->lock);

	return ret;
}

static int ipu_apb_debug_register_set(void *data, u64 val)
{
	struct ipu_debug_register *reg = (struct ipu_debug_register *)data;
	struct paintbox_data *pb = reg->pb;
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

	ipu_writeq(pb->dev, val, reg->offset);

	ret = ipu_jqs_put(pb);

	mutex_unlock(&pb->lock);

	return ret;
}

static int ipu_apb_debug_register_get(void *data, u64 *val)
{
	struct ipu_debug_register *reg = (struct ipu_debug_register *)data;
	struct paintbox_data *pb = reg->pb;
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

	*val = ipu_readq(pb->dev, reg->offset);

	ret = ipu_jqs_put(pb);

	mutex_unlock(&pb->lock);

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(ipu_apb_debug_register_fops, ipu_apb_debug_register_get,
		ipu_apb_debug_register_set, "%llx\n");

static void ipu_apb_debug_create_register_file(struct paintbox_data *pb,
		struct ipu_debug_register *reg, const char *name,
		unsigned int offset)
{
	reg->offset = IPU_CSR_APB_OFFSET + offset;
	reg->pb = pb;

	reg->dentry = debugfs_create_file(name, 0640, pb->apb_debug_dir, reg,
			&ipu_apb_debug_register_fops);
	if (WARN_ON(IS_ERR(reg->dentry)))
		reg->dentry = NULL;
}

void ipu_apb_debug_init(struct paintbox_data *pb)
{
	unsigned int i;

	pb->apb_debug_dir = debugfs_create_dir("apb", pb->debug_root);
	if (WARN_ON(IS_ERR_OR_NULL(pb->apb_debug_dir)))
		return;

	pb->apb_reg_dump = debugfs_create_devm_seqfile(pb->dev, "regs",
			pb->apb_debug_dir, ipu_apb_debug_read_registers);
	if (WARN_ON(IS_ERR_OR_NULL(pb->apb_reg_dump)))
		return;

	for (i = 0; i < IO_APB_NUM_REGS; i++) {
		if (!ipu_apb_reg_names[i])
			continue;

		ipu_apb_debug_create_register_file(pb,
				&pb->apb_debug_registers[i],
				ipu_apb_reg_names[i], i * IPU_REG_WIDTH_BYTES);
	}
}

void ipu_apb_debug_remove(struct paintbox_data *pb)
{
	debugfs_remove_recursive(pb->apb_debug_dir);
}
