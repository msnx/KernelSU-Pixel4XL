/*
 * AON debug support for the Paintbox programmable IPU
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
#include <linux/kernel.h>
#include <linux/seq_file.h>
#include <linux/types.h>

#include "ipu-aon-debug.h"
#include "ipu-client.h"
#include "ipu-debug.h"
#include "ipu-power.h"
#include "ipu-regs.h"

static const char *ipu_aon_reg_names[IO_AON_NUM_REGS] = {
	REG_NAME_ENTRY64(IPU_VERSION),
	REG_NAME_ENTRY64(IPU_CHECKSUM),
	REG_NAME_ENTRY64(IPU_CAP),
	REG_NAME_ENTRY64(CLK_GATE_CONTROL_STP_IDLE_GATE_DIS),
	REG_NAME_ENTRY64(CLK_GATE_CONTROL_LBP_IDLE_GATE_DIS),
	REG_NAME_ENTRY64(CLK_GATE_CONTROL),
	REG_NAME_ENTRY64(IDLE_CLK_COUNT),
	REG_NAME_ENTRY64(IPU_CORE_PAIRS_EN),
	REG_NAME_ENTRY64(CORE_POWER_ON_N),
	REG_NAME_ENTRY64(CORE_ISO_ON),
	REG_NAME_ENTRY64(CORE_RAM_ON_N),
	REG_NAME_ENTRY64(IPU_DMA_CHAN_EN),
	REG_NAME_ENTRY64(IO_POWER_ON_N),
	REG_NAME_ENTRY64(IO_ISO_ON),
	REG_NAME_ENTRY64(IO_RAM_ON_N),
	REG_NAME_ENTRY64(SOFT_RESET),
	REG_NAME_ENTRY64(IPU_IO_SWITCHED_CLK_EN),
	REG_NAME_ENTRY64(JQS_BOOT_ADDR),
	REG_NAME_ENTRY64(JQS_CONTROL),
	REG_NAME_ENTRY64(JQS_WATCHDOG_CMP_INIT),
	REG_NAME_ENTRY64(JQS_CACHE_ENABLE),
	REG_NAME_ENTRY64(JQS_CACHE_END_ADDR_MSB),
	REG_NAME_ENTRY64(JQS_I_CACHE_CTRL),
	REG_NAME_ENTRY64(JQS_D_CACHE_CTRL),
	REG_NAME_ENTRY64(IPU_STATUS),
	REG_NAME_ENTRY64(AON_SPARE)
};

static int ipu_aon_debug_read_registers(struct seq_file *s, void *data)
{
	struct paintbox_data *pb = dev_get_drvdata(s->private);
	unsigned int i;

	mutex_lock(&pb->lock);
	if (ipu_reset_is_requested(pb)) {
		mutex_unlock(&pb->lock);
		return -ECONNRESET;
	}

	for (i = 0; i < IO_AON_NUM_REGS; i++) {
		unsigned int offset;

		if (ipu_aon_reg_names[i] == NULL)
			continue;

		offset = i * IPU_REG_WIDTH_BYTES + IPU_CSR_AON_OFFSET;

		seq_printf(s, "0x%04lx: %-*s0x%016llx\n", offset,
				REG_VALUE_COL_WIDTH, ipu_aon_reg_names[i],
				ipu_readq(pb->dev, offset));
	}

	mutex_unlock(&pb->lock);

	return 0;
}

static int ipu_aon_debug_register_set(void *data, u64 val)
{
	struct ipu_debug_register *reg = (struct ipu_debug_register *)data;
	struct paintbox_data *pb = reg->pb;

	mutex_lock(&pb->lock);
	if (ipu_reset_is_requested(pb)) {
		mutex_unlock(&pb->lock);
		return -ECONNRESET;
	}

	ipu_writeq(pb->dev, val, reg->offset);

	mutex_unlock(&pb->lock);

	return 0;
}

static int ipu_aon_debug_register_get(void *data, u64 *val)
{
	struct ipu_debug_register *reg = (struct ipu_debug_register *)data;
	struct paintbox_data *pb = reg->pb;

	mutex_lock(&pb->lock);
	if (ipu_reset_is_requested(pb)) {
		mutex_unlock(&pb->lock);
		return -ECONNRESET;
	}

	*val = ipu_readq(pb->dev, reg->offset);

	mutex_unlock(&pb->lock);

	return 0;
}

static int ipu_aon_debug_min_core_enable_set(void *data, u64 val)
{
	struct paintbox_data *pb = data;

	mutex_lock(&pb->lock);
	if (ipu_reset_is_requested(pb)) {
		mutex_unlock(&pb->lock);
		return -ECONNRESET;
	}

	/* The maximum number of cores is equal to the number of STPs. */
	if (val > pb->stp.num_stps)
		val = pb->stp.num_stps;

	pb->power.min_active_core_count = val;

	if (val > pb->power.active_core_count)
		ipu_power_enable_cores(pb, val);
	else if (val < pb->power.active_core_count)
		ipu_power_disable_cores(pb, val);

	mutex_unlock(&pb->lock);

	return 0;
}

static int ipu_aon_debug_min_core_enable_get(void *data, u64 *val)
{
	struct paintbox_data *pb = data;

	mutex_lock(&pb->lock);
	if (ipu_reset_is_requested(pb)) {
		mutex_unlock(&pb->lock);
		return -ECONNRESET;
	}

	*val = pb->power.min_active_core_count;

	mutex_unlock(&pb->lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(ipu_aon_debug_register_fops, ipu_aon_debug_register_get,
		ipu_aon_debug_register_set, "%llx\n");

DEFINE_SIMPLE_ATTRIBUTE(ipu_aon_debug_min_core_enable_fops,
		ipu_aon_debug_min_core_enable_get,
		ipu_aon_debug_min_core_enable_set, "%llu\n");

static void ipu_aon_debug_create_register_file(struct paintbox_data *pb,
		struct ipu_debug_register *reg, const char *name,
		unsigned int offset)
{
	reg->offset = IPU_CSR_AON_OFFSET + offset;
	reg->pb = pb;

	reg->dentry = debugfs_create_file(name, 0640, pb->aon_debug_dir, reg,
			&ipu_aon_debug_register_fops);
	if (WARN_ON(IS_ERR(reg->dentry)))
		reg->dentry = NULL;
}

void ipu_aon_debug_init(struct paintbox_data *pb)
{
	unsigned int i;

	pb->aon_debug_dir = debugfs_create_dir("aon", pb->debug_root);
	if (WARN_ON(IS_ERR_OR_NULL(pb->aon_debug_dir)))
		return;

	pb->aon_reg_dump = debugfs_create_devm_seqfile(pb->dev, "regs",
			pb->aon_debug_dir, ipu_aon_debug_read_registers);
	if (WARN_ON(IS_ERR_OR_NULL(pb->aon_reg_dump)))
		return;

	for (i = 0; i < IO_AON_NUM_REGS; i++) {
		if (!ipu_aon_reg_names[i])
			continue;

		ipu_aon_debug_create_register_file(pb,
				&pb->aon_debug_registers[i],
				ipu_aon_reg_names[i], i * IPU_REG_WIDTH_BYTES);
	}

	pb->power.min_core_enable_dentry =
			debugfs_create_file("min_core_enable", 0640,
			pb->aon_debug_dir, pb,
			&ipu_aon_debug_min_core_enable_fops);
	WARN_ON(IS_ERR_OR_NULL(pb->power.min_core_enable_dentry));
}

void ipu_aon_debug_remove(struct paintbox_data *pb)
{
	debugfs_remove_recursive(pb->aon_debug_dir);
}
