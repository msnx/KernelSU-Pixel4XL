/*
 * BIF support for the Paintbox programmable IPU
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

#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>
#include <linux/types.h>

#include "ipu-bif-debug.h"
#include "ipu-client.h"
#include "ipu-debug.h"
#include "ipu-regs.h"

static const char *ipu_bif_reg_names[IO_AXI_NUM_REGS] = {
	REG_NAME_ENTRY64(MMU_CTRL),
	REG_NAME_ENTRY64(MMU_ENABLE_CTRL),
	REG_NAME_ENTRY64(MMU_PREFETCH_CTRL),
	REG_NAME_ENTRY64(MMU_TABLE_BASE),
	REG_NAME_ENTRY64(MMU_ERR_BASE),
	REG_NAME_ENTRY64(MMU_SYNC),
	REG_NAME_ENTRY64(MMU_FLUSH_CHANNEL),
	REG_NAME_ENTRY64(MMU_FLUSH_ADDRESS),
	REG_NAME_ENTRY64(MMU_FLUSH_FIFO_STATUS),
	REG_NAME_ENTRY64(MMU_ISR),
	REG_NAME_ENTRY64(MMU_ITR),
	REG_NAME_ENTRY64(MMU_IER),
	REG_NAME_ENTRY64(MMU_IMR),
	REG_NAME_ENTRY64(MMU_ISR_OVF),
	REG_NAME_ENTRY64(MMU_ERR_LOG),
	REG_NAME_ENTRY64(BIF_AXI_CTRL_DMA0),
	REG_NAME_ENTRY64(BIF_AXI_CTRL_DMA1),
	REG_NAME_ENTRY64(BIF_AXI_CTRL_DMA2),
	REG_NAME_ENTRY64(BIF_AXI_CTRL_DMA3),
	REG_NAME_ENTRY64(BIF_AXI_CTRL_DMA4),
	REG_NAME_ENTRY64(BIF_AXI_CTRL_DMA5),
	REG_NAME_ENTRY64(BIF_AXI_CTRL_DMA6),
	REG_NAME_ENTRY64(BIF_AXI_CTRL_DMA7),
	REG_NAME_ENTRY64(BIF_AXI_CTRL_MMU),
	REG_NAME_ENTRY64(BIF_ISR),
	REG_NAME_ENTRY64(BIF_ITR),
	REG_NAME_ENTRY64(BIF_IER),
	REG_NAME_ENTRY64(BIF_IMR),
	REG_NAME_ENTRY64(BIF_ISR_OVF),
	REG_NAME_ENTRY64(BIF_TO_ERR_CFG),
	REG_NAME_ENTRY64(BIF_ERR_LOG),
	REG_NAME_ENTRY64(BIF_ERR_LOG_BUS_ADDR),
	REG_NAME_ENTRY64(BIF_PMON_CFG),
	REG_NAME_ENTRY64(BIF_PMON_CNT_0_CFG),
	REG_NAME_ENTRY64(BIF_PMON_CNT_0),
	REG_NAME_ENTRY64(BIF_PMON_CNT_0_STS_ACC),
	REG_NAME_ENTRY64(BIF_PMON_CNT_0_STS),
	REG_NAME_ENTRY64(BIF_PMON_CNT_1_CFG),
	REG_NAME_ENTRY64(BIF_PMON_CNT_1),
	REG_NAME_ENTRY64(BIF_PMON_CNT_1_STS_ACC),
	REG_NAME_ENTRY64(BIF_PMON_CNT_1_STS),
	REG_NAME_ENTRY64(MMU_PMON_CFG),
	REG_NAME_ENTRY64(MMU_PMON_CNT_0_CFG),
	REG_NAME_ENTRY64(MMU_PMON_CNT_0),
	REG_NAME_ENTRY64(MMU_PMON_CNT_0_STS_ACC),
	REG_NAME_ENTRY64(MMU_PMON_CNT_0_STS),
	REG_NAME_ENTRY64(MMU_PMON_CNT_1_CFG),
	REG_NAME_ENTRY64(MMU_PMON_CNT_1),
	REG_NAME_ENTRY64(MMU_PMON_CNT_1_STS_ACC),
	REG_NAME_ENTRY64(MMU_PMON_CNT_1_STS),
	REG_NAME_ENTRY64(AXI_SPARE)
};

static int ipu_bif_debug_read_registers(struct seq_file *s, void *data)
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

	for (i = 0; i < IO_AXI_NUM_REGS; i++) {
		unsigned int offset;

		if (ipu_bif_reg_names[i] == NULL)
			continue;

		offset = i * IPU_REG_WIDTH_BYTES + IPU_CSR_AXI_OFFSET;

		seq_printf(s, "0x%04lx: %-*s0x%016llx\n", offset,
				REG_VALUE_COL_WIDTH, ipu_bif_reg_names[i],
				ipu_readq(pb->dev, offset));
	}

	ret = ipu_jqs_put(pb);

	mutex_unlock(&pb->lock);

	return ret;
}

static int ipu_bif_debug_register_set(void *data, u64 val)
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

static int ipu_bif_debug_register_get(void *data, u64 *val)
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


DEFINE_SIMPLE_ATTRIBUTE(ipu_bif_debug_register_fops, ipu_bif_debug_register_get,
		ipu_bif_debug_register_set, "%llx\n");

static void ipu_bif_debug_create_register_file(struct paintbox_data *pb,
		struct ipu_debug_register *reg, const char *name,
		unsigned int offset)
{
	reg->offset = IPU_CSR_AXI_OFFSET + offset;
	reg->pb = pb;

	reg->dentry = debugfs_create_file(name, 0640, pb->bif_debug_dir, reg,
			&ipu_bif_debug_register_fops);
	if (WARN_ON(IS_ERR(reg->dentry)))
		reg->dentry = NULL;
}

void ipu_bif_debug_init(struct paintbox_data *pb)
{
	unsigned int i;

	pb->bif_debug_dir = debugfs_create_dir("bif", pb->debug_root);
	if (WARN_ON(IS_ERR_OR_NULL(pb->bif_debug_dir)))
		return;

	pb->bif_reg_dump = debugfs_create_devm_seqfile(pb->dev, "regs",
			pb->bif_debug_dir, ipu_bif_debug_read_registers);
	if (WARN_ON(IS_ERR_OR_NULL(pb->bif_reg_dump)))
		return;

	for (i = 0; i < IO_AXI_NUM_REGS; i++) {
		if (!ipu_bif_reg_names[i])
			continue;

		ipu_bif_debug_create_register_file(pb,
				&pb->bif_debug_registers[i],
				ipu_bif_reg_names[i], i * IPU_REG_WIDTH_BYTES);
	}
}

void ipu_bif_debug_remove(struct paintbox_data *pb)
{
	debugfs_remove_recursive(pb->bif_debug_dir);
}
