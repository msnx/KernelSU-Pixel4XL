/*
 * STP debug support for the Paintbox programmable IPU
 *
 * Copyright (C) 2017 Google, Inc.
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
#include <linux/list.h>
#include <linux/seq_file.h>
#include <linux/types.h>
#include <linux/pm_runtime.h>

#include "ipu-client.h"
#include "ipu-debug.h"
#include "ipu-regs.h"
#include "ipu-stp-debug.h"

static inline void ipu_stp_select(struct paintbox_data *pb,
		unsigned int stp_id)
{
	ipu_writel(pb->dev, stp_id, IPU_CSR_STP_OFFSET + STP_SEL);
}

static const char *ipu_stp_reg_names[STP_NUM_REGS] = {
	REG_NAME_ENTRY64(STP_SEL),
	REG_NAME_ENTRY64(STP_CTRL),
	REG_NAME_ENTRY64(STP_START),
	REG_NAME_ENTRY64(STP_MASK),
	REG_NAME_ENTRY64(STP_STAT),
	REG_NAME_ENTRY64(STP_CAP),
	REG_NAME_ENTRY64(STP_ISR),
	REG_NAME_ENTRY64(STP_ITR),
	REG_NAME_ENTRY64(STP_IER),
	REG_NAME_ENTRY64(STP_IMR),
	REG_NAME_ENTRY64(STP_ISR_OVF),
	REG_NAME_ENTRY64(STP_IRQ_LOG),
	REG_NAME_ENTRY64(STP_ERR_LOG),
	REG_NAME_ENTRY64(STP_RAM_CTRL),
	REG_NAME_ENTRY64(STP_RAM_DATA0),
	REG_NAME_ENTRY64(STP_RAM_DATA1),
	REG_NAME_ENTRY64(STP_PMON_CFG),
	REG_NAME_ENTRY64(STP_PMON_CNT_0_CFG),
	REG_NAME_ENTRY64(STP_PMON_CNT_0),
	REG_NAME_ENTRY64(STP_PMON_CNT_0_STS),
	REG_NAME_ENTRY64(STP_PMON_CNT_1_CFG),
	REG_NAME_ENTRY64(STP_PMON_CNT_1),
	REG_NAME_ENTRY64(STP_PMON_CNT_1_STS),
};

static inline void ipu_stp_dump_reg(struct seq_file *s,
		unsigned int reg_offset, uint64_t reg_value)
{
	const char *reg_name = ipu_stp_reg_names[REG_INDEX64(reg_offset)];

	seq_printf(s, "0x%04llx: %-*s0x%016llx\n",
			IPU_CSR_STP_OFFSET + reg_offset,
			REG_VALUE_COL_WIDTH, reg_name, reg_value);
}


static inline void ipu_stp_dump_stp_sel(struct seq_file *s,
		uint64_t reg_value)
{
	ipu_stp_dump_reg(s, STP_SEL, reg_value);
	seq_printf(s, "\tSTP_SEL 0x%02x\n", reg_value & STP_SEL_STP_SEL_MASK);
}

static inline void ipu_stp_dump_stp_ctrl(struct seq_file *s,
		uint64_t reg_value)
{
	ipu_stp_dump_reg(s, STP_CTRL, reg_value);
	seq_printf(s, "\tTS_FR %d PC_FR %d RESUME %d RESET %d ENA %d\n",
			!!(reg_value & STP_CTRL_TS_FR_MASK),
			!!(reg_value & STP_CTRL_PC_FR_MASK),
			!!(reg_value & STP_CTRL_RESUME_MASK),
			!!(reg_value & STP_CTRL_RESET_MASK),
			!!(reg_value & STP_CTRL_ENA_MASK));
}

static inline void ipu_stp_dump_stp_start(struct seq_file *s,
		uint64_t reg_value)
{
	ipu_stp_dump_reg(s, STP_START, reg_value);
}

static inline void ipu_stp_dump_stp_mask(struct seq_file *s,
		uint64_t reg_value)
{
	ipu_stp_dump_reg(s, STP_MASK, reg_value);
}

static inline void ipu_stp_dump_stp_stat(struct seq_file *s,
		uint64_t reg_value)
{
	ipu_stp_dump_reg(s, STP_STAT, reg_value);
	seq_printf(s, "\tSTALLED %d PC 0x%04x\n",
			(reg_value >> STP_STAT_STALLED_SHIFT) &
					STP_STAT_STALLED_M,
			reg_value & STP_STAT_PC_MASK);
}

static inline void ipu_stp_dump_stp_cap(struct seq_file *s,
		uint64_t reg_value)
{
	ipu_stp_dump_reg(s, STP_CAP, reg_value);
	seq_printf(s, "\tHALO_MEM %u words VECTOR_MEM %u words CONST MEM %u words SCALAR_MEM %u words INST_MEM %u instructions\n",
			(reg_value & STP_CAP_HALO_MEM_MASK) >>
					STP_CAP_HALO_MEM_SHIFT,
			(reg_value & STP_CAP_VECTOR_MEM_MASK) >>
					STP_CAP_VECTOR_MEM_SHIFT,
			(reg_value & STP_CAP_CONST_MEM_MASK) >>
					STP_CAP_CONST_MEM_SHIFT,
			(reg_value & STP_CAP_SCALAR_MEM_MASK) >>
					STP_CAP_SCALAR_MEM_SHIFT,
			reg_value & STP_CAP_INST_MEM_MASK);
}

static inline void ipu_stp_dump_stp_isr(struct seq_file *s,
		uint64_t reg_value)
{
	ipu_stp_dump_reg(s, STP_ISR, reg_value);
	seq_printf(s, "\tERR %d INT %d\n",
			!!(reg_value & STP_ISR_ERR_MASK),
			!!(reg_value & STP_ISR_INT_MASK));
}

static inline void ipu_stp_dump_stp_itr(struct seq_file *s,
		uint64_t reg_value)
{
	ipu_stp_dump_reg(s, STP_ITR, reg_value);
	seq_printf(s, "\tERR %d INT %d\n",
			!!(reg_value & STP_ITR_ERR_MASK),
			!!(reg_value & STP_ITR_INT_MASK));
}

static inline void ipu_stp_dump_stp_ier(struct seq_file *s,
		uint64_t reg_value)
{
	ipu_stp_dump_reg(s, STP_IER, reg_value);
	seq_printf(s, "\tERR %d INT %d\n",
			!!(reg_value & STP_IER_ERR_MASK),
			!!(reg_value & STP_IER_INT_MASK));
}

static inline void ipu_stp_dump_stp_imr(struct seq_file *s,
		uint64_t reg_value)
{
	ipu_stp_dump_reg(s, STP_IMR, reg_value);
	seq_printf(s, "\tERR %d INT %d\n",
			!!(reg_value & STP_IMR_ERR_MASK),
			!!(reg_value & STP_IMR_INT_MASK));
}

static inline void ipu_stp_dump_stp_isr_ovf(struct seq_file *s,
		uint64_t reg_value)
{
	ipu_stp_dump_reg(s, STP_ISR_OVF, reg_value);
	seq_printf(s, "\tERR %d INT %d\n",
			!!(reg_value & STP_ISR_OVF_ERR_MASK),
			!!(reg_value & STP_ISR_OVF_INT_MASK));
}

static inline void ipu_stp_dump_stp_irq_log(struct seq_file *s,
		uint64_t reg_value)
{
	ipu_stp_dump_reg(s, STP_IRQ_LOG, reg_value);
}

static inline void ipu_stp_dump_stp_err_log(struct seq_file *s,
		uint64_t reg_value)
{
	ipu_stp_dump_reg(s, STP_ERR_LOG, reg_value);
}

static inline void ipu_stp_dump_stp_ram_ctrl(struct seq_file *s,
		uint64_t reg_value)
{
	ipu_stp_dump_reg(s, STP_RAM_CTRL, reg_value);
	seq_printf(s, "\tRAM_ADDR %u RAM_TARG %u\n",
			(reg_value & STP_RAM_CTRL_RAM_ADDR_MASK) >>
					STP_RAM_CTRL_RAM_ADDR_SHIFT,
			(reg_value & STP_RAM_CTRL_RAM_TARG_MASK) >>
					STP_RAM_CTRL_RAM_TARG_SHIFT);
	seq_printf(s, "\tPRI %d WRITE %d RUN %d\n",
			!!(reg_value & STP_RAM_CTRL_PRI_MASK),
			!!(reg_value & STP_RAM_CTRL_WRITE_MASK),
			!!(reg_value & STP_RAM_CTRL_RUN_MASK));
}

static inline void ipu_stp_dump_stp_ram_data0(struct seq_file *s,
		uint64_t reg_value)
{
	ipu_stp_dump_reg(s, STP_RAM_DATA0, reg_value);
}

static inline void ipu_stp_dump_stp_ram_data1(struct seq_file *s,
		uint64_t reg_value)
{
	ipu_stp_dump_reg(s, STP_RAM_DATA1, reg_value);
}

static inline void ipu_stp_dump_stp_pmon_cfg(struct seq_file *s,
		uint64_t reg_value)
{
	ipu_stp_dump_reg(s, STP_PMON_CFG, reg_value);
	seq_printf(s, "\tENABLE %d\n",
			!!(reg_value & STP_PMON_CFG_ENABLE_MASK));
}

static inline void ipu_stp_dump_stp_pmon_cnt_0_cfg(struct seq_file *s,
		uint64_t reg_value)
{
	ipu_stp_dump_reg(s, STP_PMON_CNT_0_CFG, reg_value);
	seq_printf(s, "\tDEC_INV %d DEC_MATCH %u DEC_MASK %u DEC_SEL %u\n",
			!!(reg_value & STP_PMON_CNT_0_CFG_DEC_INV_MASK),
			(reg_value & STP_PMON_CNT_0_CFG_DEC_MATCH_MASK) >>
					STP_PMON_CNT_0_CFG_DEC_MATCH_SHIFT,
			(reg_value & STP_PMON_CNT_0_CFG_DEC_MASK_MASK) >>
					STP_PMON_CNT_0_CFG_DEC_MASK_SHIFT,
			(reg_value & STP_PMON_CNT_0_CFG_DEC_SEL_MASK) >>
					STP_PMON_CNT_0_CFG_DEC_SEL_SHIFT);
	seq_printf(s, "\tINC_INV %d INC_MATCH %u INC_MASK %u INC_SEL %u\n",
			!!(reg_value & STP_PMON_CNT_0_CFG_INC_INV_MASK),
			(reg_value & STP_PMON_CNT_0_CFG_INC_MATCH_MASK) >>
					STP_PMON_CNT_0_CFG_INC_MATCH_SHIFT,
			(reg_value & STP_PMON_CNT_0_CFG_INC_MASK_MASK) >>
					STP_PMON_CNT_0_CFG_INC_MASK_SHIFT,
			(reg_value & STP_PMON_CNT_0_CFG_INC_SEL_MASK) >>
					STP_PMON_CNT_0_CFG_INC_SEL_SHIFT);
	seq_printf(s, "\tTHRESHOLD %u MODE %u\n",
			(reg_value & STP_PMON_CNT_0_CFG_THRESHOLD_MASK) >>
					STP_PMON_CNT_0_CFG_THRESHOLD_SHIFT,
			(reg_value & STP_PMON_CNT_0_CFG_MODE_MASK) >>
					STP_PMON_CNT_0_CFG_MODE_SHIFT);
}

static inline void ipu_stp_dump_stp_pmon_cnt_0(struct seq_file *s,
		uint64_t reg_value)
{
	ipu_stp_dump_reg(s, STP_PMON_CNT_0, reg_value);
}

static inline void ipu_stp_dump_stp_pmon_cnt_0_sts(struct seq_file *s,
		uint64_t reg_value)
{
	ipu_stp_dump_reg(s, STP_PMON_CNT_0_STS, reg_value);
	seq_printf(s, "\tCNT_OF %d ACC_UF %d ACC_OF %d\n",
			!!(reg_value & STP_PMON_CNT_0_STS_CNT_OF_MASK),
			!!(reg_value & STP_PMON_CNT_0_STS_ACC_UF_MASK),
			!!(reg_value & STP_PMON_CNT_0_STS_ACC_OF_MASK));
}

static inline void ipu_stp_dump_stp_pmon_cnt_1_cfg(struct seq_file *s,
		uint64_t reg_value)
{
	ipu_stp_dump_reg(s, STP_PMON_CNT_1_CFG, reg_value);
	seq_printf(s, "\tDEC_INV %d DEC_MATCH %u DEC_MASK %u DEC_SEL %u\n",
			!!(reg_value & STP_PMON_CNT_1_CFG_DEC_INV_MASK),
			(reg_value & STP_PMON_CNT_1_CFG_DEC_MATCH_MASK) >>
					STP_PMON_CNT_1_CFG_DEC_MATCH_SHIFT,
			(reg_value & STP_PMON_CNT_1_CFG_DEC_MASK_MASK) >>
					STP_PMON_CNT_1_CFG_DEC_MASK_SHIFT,
			(reg_value & STP_PMON_CNT_1_CFG_DEC_SEL_MASK) >>
					STP_PMON_CNT_1_CFG_DEC_SEL_SHIFT);
	seq_printf(s, "\tINC_INV %d INC_MATCH %u INC_MASK %u INC_SEL %u\n",
			!!(reg_value & STP_PMON_CNT_1_CFG_INC_INV_MASK),
			(reg_value & STP_PMON_CNT_1_CFG_INC_MATCH_MASK) >>
					STP_PMON_CNT_1_CFG_INC_MATCH_SHIFT,
			(reg_value & STP_PMON_CNT_1_CFG_INC_MASK_MASK) >>
					STP_PMON_CNT_1_CFG_INC_MASK_SHIFT,
			(reg_value & STP_PMON_CNT_1_CFG_INC_SEL_MASK) >>
					STP_PMON_CNT_1_CFG_INC_SEL_SHIFT);
	seq_printf(s, "\tTHRESHOLD %u MODE %u\n",
			(reg_value & STP_PMON_CNT_1_CFG_THRESHOLD_MASK) >>
					STP_PMON_CNT_1_CFG_THRESHOLD_SHIFT,
			(reg_value & STP_PMON_CNT_1_CFG_MODE_MASK) >>
					STP_PMON_CNT_1_CFG_MODE_SHIFT);
}

static inline void ipu_stp_dump_stp_pmon_cnt_1(struct seq_file *s,
		uint64_t reg_value)
{
	ipu_stp_dump_reg(s, STP_PMON_CNT_1, reg_value);
}

static inline void ipu_stp_dump_stp_pmon_cnt_1_sts(struct seq_file *s,
		uint64_t reg_value)
{
	ipu_stp_dump_reg(s, STP_PMON_CNT_1_STS, reg_value);
	seq_printf(s, "\tCNT_OF %d ACC_UF %d ACC_OF %d\n",
			!!(reg_value & STP_PMON_CNT_1_STS_CNT_OF_MASK),
			!!(reg_value & STP_PMON_CNT_1_STS_ACC_UF_MASK),
			!!(reg_value & STP_PMON_CNT_1_STS_ACC_OF_MASK));
}


int ipu_stp_debug_read_registers(struct seq_file *s, void *data)
{
	struct ipu_stp_debug_regs *debug_reg_dump = s->private;
	struct paintbox_data *pb = debug_reg_dump->pb;
	struct paintbox_stp *stp = debug_reg_dump->stp;
	uint64_t stp_registers[STP_NUM_REGS];
	unsigned int reg_offset;
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

	ipu_stp_select(pb, stp->stp_id);

	for (reg_offset = 0; reg_offset < STP_BLOCK_LEN; reg_offset +=
			IPU_REG_WIDTH_BYTES) {
		if (!ipu_stp_reg_names[REG_INDEX64(reg_offset)])
			continue;

		stp_registers[REG_INDEX64(reg_offset)] =
				ipu_readq(pb->dev, IPU_CSR_STP_OFFSET +
				reg_offset);
	}

	ipu_stp_dump_stp_sel(s, stp_registers[REG_INDEX64(STP_SEL)]);
	ipu_stp_dump_stp_ctrl(s, stp_registers[REG_INDEX64(STP_CTRL)]);
	ipu_stp_dump_stp_start(s, stp_registers[REG_INDEX64(STP_START)]);
	ipu_stp_dump_stp_mask(s, stp_registers[REG_INDEX64(STP_MASK)]);
	ipu_stp_dump_stp_stat(s, stp_registers[REG_INDEX64(STP_STAT)]);
	ipu_stp_dump_stp_cap(s, stp_registers[REG_INDEX64(STP_CAP)]);
	ipu_stp_dump_stp_isr(s, stp_registers[REG_INDEX64(STP_ISR)]);
	ipu_stp_dump_stp_itr(s, stp_registers[REG_INDEX64(STP_ITR)]);
	ipu_stp_dump_stp_ier(s, stp_registers[REG_INDEX64(STP_IER)]);
	ipu_stp_dump_stp_imr(s, stp_registers[REG_INDEX64(STP_IMR)]);
	ipu_stp_dump_stp_isr_ovf(s, stp_registers[REG_INDEX64(STP_ISR_OVF)]);
	ipu_stp_dump_stp_irq_log(s, stp_registers[REG_INDEX64(STP_IRQ_LOG)]);
	ipu_stp_dump_stp_err_log(s, stp_registers[REG_INDEX64(STP_ERR_LOG)]);
	ipu_stp_dump_stp_ram_ctrl(s,
			stp_registers[REG_INDEX64(STP_RAM_CTRL)]);
	ipu_stp_dump_stp_ram_data0(s,
			stp_registers[REG_INDEX64(STP_RAM_DATA0)]);
	ipu_stp_dump_stp_ram_data1(s,
			stp_registers[REG_INDEX64(STP_RAM_DATA1)]);
	ipu_stp_dump_stp_pmon_cfg(s,
			stp_registers[REG_INDEX64(STP_PMON_CFG)]);
	ipu_stp_dump_stp_pmon_cnt_0_cfg(s,
			stp_registers[REG_INDEX64(STP_PMON_CNT_0_CFG)]);
	ipu_stp_dump_stp_pmon_cnt_0(s,
			stp_registers[REG_INDEX64(STP_PMON_CNT_0)]);
	ipu_stp_dump_stp_pmon_cnt_0_sts(s,
			stp_registers[REG_INDEX64(STP_PMON_CNT_0_STS)]);
	ipu_stp_dump_stp_pmon_cnt_1_cfg(s,
			stp_registers[REG_INDEX64(STP_PMON_CNT_1_CFG)]);
	ipu_stp_dump_stp_pmon_cnt_1(s,
			stp_registers[REG_INDEX64(STP_PMON_CNT_1)]);
	ipu_stp_dump_stp_pmon_cnt_1_sts(s,
			stp_registers[REG_INDEX64(STP_PMON_CNT_1_STS)]);

	ret = ipu_jqs_put(pb);

	mutex_unlock(&pb->lock);

	return ret;
}

static int ipu_stp_debug_register_set(void *data, u64 val)
{
	struct ipu_stp_debug_register *reg = data;
	struct paintbox_data *pb = reg->debug_register.pb;
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

	/* TODO(b/115386014):  Move to a JQS message based system. */
	ipu_stp_select(pb, reg->stp_id);
	ipu_writeq(pb->dev, val, reg->debug_register.offset);

	ret = ipu_jqs_put(pb);

	mutex_unlock(&pb->lock);

	return ret;
}

static int ipu_stp_debug_register_get(void *data, u64 *val)
{
	struct ipu_stp_debug_register *reg = data;
	struct paintbox_data *pb = reg->debug_register.pb;
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

	/* TODO(b/115386014):  Move to a JQS message based system. */
	ipu_stp_select(pb, reg->stp_id);
	*val = ipu_readq(pb->dev, reg->debug_register.offset);

	ret = ipu_jqs_put(pb);

	mutex_unlock(&pb->lock);

	return ret;
}


DEFINE_SIMPLE_ATTRIBUTE(ipu_stp_debug_register_fops, ipu_stp_debug_register_get,
		ipu_stp_debug_register_set, "%llx\n");

static void ipu_stp_debug_create_register_file(struct paintbox_data *pb,
		struct paintbox_stp *stp,
		struct ipu_stp_debug_register *reg, const char *name,
		unsigned int offset)
{
	reg->stp_id = stp->stp_id;
	reg->debug_register.offset = IPU_CSR_STP_OFFSET + offset;
	reg->debug_register.pb = pb;

	reg->debug_register.dentry = debugfs_create_file(name, 0640,
			stp->debug_dir, reg, &ipu_stp_debug_register_fops);
	if (WARN_ON(IS_ERR(reg->debug_register.dentry)))
		reg->debug_register.dentry = NULL;
}


static int ipu_stp_reg_dump_open(struct inode *inode, struct file *file)
{
	return single_open_size(file, ipu_stp_debug_read_registers,
			inode->i_private,
			STP_NUM_REGS * REG_DEBUG_BUFFER_SIZE);
}

static const struct file_operations ipu_stp_reg_dump_fops = {
	.open = ipu_stp_reg_dump_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void ipu_stp_debug_init(struct paintbox_data *pb, struct paintbox_stp *stp)
{
	char stp_name[RESOURCE_NAME_LEN];
	unsigned int i, ret;

	stp->debug_reg_dump.pb = pb;
	stp->debug_reg_dump.stp = stp;

	ret = scnprintf(stp_name, RESOURCE_NAME_LEN, "%s%u", "stp",
		stp->stp_id);

	stp->debug_dir = debugfs_create_dir(stp_name, pb->debug_root);
	if (WARN_ON(IS_ERR_OR_NULL(stp->debug_dir)))
		return;

	stp->debug_reg_dump.dentry = debugfs_create_file("regs", 0640,
			stp->debug_dir, &stp->debug_reg_dump,
			&ipu_stp_reg_dump_fops);
	if (WARN_ON(IS_ERR_OR_NULL(stp->debug_reg_dump.dentry)))
		return;

	for (i = 0; i < STP_NUM_REGS; i++) {
		if (!ipu_stp_reg_names[i])
			continue;

		ipu_stp_debug_create_register_file(pb, stp,
				&stp->debug_registers[i],
				ipu_stp_reg_names[i], i * IPU_REG_WIDTH_BYTES);
	}
}

void ipu_stp_debug_remove(struct paintbox_data *pb, struct paintbox_stp *stp)
{
	debugfs_remove_recursive(stp->debug_dir);
}
