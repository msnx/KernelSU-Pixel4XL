/*
 * MMU debug support for the Paintbox programmable IPU
 *
 * Copyright (C) 2019 Google, Inc.
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
#include <linux/types.h>

#include "ipu-client.h"
#include "ipu-debug.h"
#include "ipu-regs.h"

static uint64_t ipu_mmu_reg_entry_read(
		struct paintbox_debug_reg_entry *reg_entry)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_data *pb = debug->pb;

	return ipu_readq(pb->dev, IPU_CSR_AXI_OFFSET +
			reg_entry->reg_offset);
}

static void ipu_mmu_reg_entry_write(
		struct paintbox_debug_reg_entry *reg_entry, uint64_t val)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_data *pb = debug->pb;

	ipu_writeq(pb->dev, val, IPU_CSR_AXI_OFFSET +
			reg_entry->reg_offset);
}

static const char *ipu_mmu_reg_names[IO_AXI_NUM_REGS] = {
	REG_NAME_ENTRY(MMU_CTRL),
	REG_NAME_ENTRY(MMU_ENABLE_CTRL),
	REG_NAME_ENTRY(MMU_PREFETCH_CTRL),
	REG_NAME_ENTRY(MMU_TABLE_BASE),
	REG_NAME_ENTRY(MMU_ERR_BASE),
	REG_NAME_ENTRY(MMU_SYNC),
	REG_NAME_ENTRY(MMU_FLUSH_CHANNEL),
	REG_NAME_ENTRY(MMU_FLUSH_ADDRESS),
	REG_NAME_ENTRY(MMU_FLUSH_FIFO_STATUS),
	REG_NAME_ENTRY(MMU_ISR),
	REG_NAME_ENTRY(MMU_IMR),
	REG_NAME_ENTRY(MMU_ISR_OVF),
	REG_NAME_ENTRY(MMU_ERR_LOG),
	REG_NAME_ENTRY(MMU_PMON_CFG),
	REG_NAME_ENTRY(MMU_PMON_CNT_0_CFG),
	REG_NAME_ENTRY(MMU_PMON_CNT_0),
	REG_NAME_ENTRY(MMU_PMON_CNT_0_STS_ACC),
	REG_NAME_ENTRY(MMU_PMON_CNT_0_STS),
	REG_NAME_ENTRY(MMU_PMON_CNT_1_CFG),
	REG_NAME_ENTRY(MMU_PMON_CNT_1),
	REG_NAME_ENTRY(MMU_PMON_CNT_1_STS_ACC),
	REG_NAME_ENTRY(MMU_PMON_CNT_1_STS),
	REG_NAME_ENTRY(MMU_IER),
	REG_NAME_ENTRY(MMU_ITR),
};

static inline int ipu_mmu_dump_reg(struct paintbox_data *pb,
		uint32_t reg_offset, char *buf, int *written, size_t len)
{
	const char *reg_name = ipu_mmu_reg_names[REG_INDEX(reg_offset)];

	return ipu_debug_dump_register(pb, IPU_CSR_AXI_OFFSET,
		reg_offset, reg_name,
		buf, written, len);
}

static int ipu_dump_mmu_registers(struct paintbox_debug *debug, char *buf,
		size_t len)
{
	struct paintbox_data *pb = debug->pb;
	unsigned int i;
	int ret, written = 0;

	for (i = 0; i < IO_AXI_NUM_REGS; i++) {
		if (ipu_mmu_reg_names[i] == NULL)
			continue;

		ret = ipu_mmu_dump_reg(pb, i * IPU_REG_WIDTH, buf,
				&written, len);
		if (ret < 0) {
			dev_err(pb->dev, "%s: register dump error, err = %d",
					__func__, ret);
			return ret;
		}
	}

	return written;
}

int ipu_mmu_debug_init(struct paintbox_data *pb)
{
	ipu_debug_create_entry(pb, &pb->mmu.debug, pb->debug_root, "mmu",
			-1, ipu_dump_mmu_registers, NULL, &pb->mmu);

	ipu_debug_create_reg_entries(pb, &pb->mmu.debug,
			ipu_mmu_reg_names, IO_AXI_NUM_REGS,
			ipu_mmu_reg_entry_write,
			ipu_mmu_reg_entry_read);

	return 0;
}

/* All sessions must be released before remove can be called. */
void ipu_mmu_remove(struct paintbox_data *pb)
{
	ipu_debug_free_reg_entries(&pb->mmu.debug);
	ipu_debug_free_entry(&pb->mmu.debug);
}
