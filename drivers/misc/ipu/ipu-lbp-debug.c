/*
 * Line Buffer Pool Debug Support for Paintbox IPU
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

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/types.h>

#include "ipu-client.h"
#include "ipu-debug.h"
#include "ipu-lbp.h"
#include "ipu-regs.h"

/* The caller to this function must hold pb->lock */
static uint64_t ipu_lbp_reg_entry_read(
		struct paintbox_debug_reg_entry *reg_entry)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_lbp *lbp = container_of(debug, struct paintbox_lbp,
			debug);
	struct paintbox_data *pb = debug->pb;

	/* TODO(b/115386014):  Replace with JQS debug reg op message */
	ipu_lbp_select(pb, lbp->pool_id);
	return ipu_readq(pb->dev, IPU_CSR_LBP_OFFSET +
			reg_entry->reg_offset);
}

/* The caller to this function must hold pb->lock */
static void ipu_lbp_reg_entry_write(struct paintbox_debug_reg_entry *reg_entry,
		uint64_t val)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_lbp *lbp = container_of(debug, struct paintbox_lbp,
			debug);
	struct paintbox_data *pb = debug->pb;

	/* TODO(b/115386014):  Replace with JQS debug reg op message */
	ipu_lbp_select(pb, lbp->pool_id);
	ipu_writeq(pb->dev, val, IPU_CSR_LBP_OFFSET +
			reg_entry->reg_offset);
}

/* The caller to this function must hold pb->lock */
static uint64_t ipu_lb_reg_entry_read(
		struct paintbox_debug_reg_entry *reg_entry)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_lb *lb = container_of(debug, struct paintbox_lb, debug);
	struct paintbox_lbp *lbp = lb->lbp;
	struct paintbox_data *pb = debug->pb;

	/* TODO(b/115386014):  Replace with JQS debug reg op message */
	ipu_lb_select(pb, lbp->pool_id, lb->lb_id);
	return ipu_readq(pb->dev, IPU_CSR_LBP_OFFSET + LB_BLOCK_START +
			reg_entry->reg_offset);
}

/* The caller to this function must hold pb->lock */
static void ipu_lb_reg_entry_write(struct paintbox_debug_reg_entry *reg_entry,
			uint64_t val)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_lb *lb = container_of(debug, struct paintbox_lb, debug);
	struct paintbox_lbp *lbp = lb->lbp;
	struct paintbox_data *pb = debug->pb;

	ipu_lb_select(pb, lbp->pool_id, lb->lb_id);
	ipu_writeq(pb->dev, val, IPU_CSR_LBP_OFFSET + LB_BLOCK_START +
			reg_entry->reg_offset);
}

static const char *ipu_lbp_reg_names[LBP_NUM_REGS] = {
	REG_NAME_ENTRY(LBP_SEL),
	REG_NAME_ENTRY(LBP_CTRL),
	REG_NAME_ENTRY(LBP_STAT),
	REG_NAME_ENTRY(LBP_CAP0),
	REG_NAME_ENTRY(LBP_CAP1),
	REG_NAME_ENTRY(LBP_RAM_CTRL),
	REG_NAME_ENTRY(LBP_RAM_DATA0),
	REG_NAME_ENTRY(LBP_RAM_DATA1),
	REG_NAME_ENTRY(LBP_RAM_DATA2),
	REG_NAME_ENTRY(LBP_RAM_DATA3),
	REG_NAME_ENTRY(LBP_PMON_CFG),
	REG_NAME_ENTRY(LBP_PMON_CNT_0_CFG),
	REG_NAME_ENTRY(LBP_PMON_CNT_0),
	REG_NAME_ENTRY(LBP_PMON_CNT_0_STS),
	REG_NAME_ENTRY(LBP_PMON_CNT_1_CFG),
	REG_NAME_ENTRY(LBP_PMON_CNT_1),
	REG_NAME_ENTRY(LBP_PMON_CNT_1_STS),
	REG_NAME_ENTRY(LB_CTRL0),
	REG_NAME_ENTRY(LB_OFFSET),
	REG_NAME_ENTRY(LB_BDRY),
	REG_NAME_ENTRY(LB_IMG_SIZE),
	REG_NAME_ENTRY(LB_SB_SIZE),
	REG_NAME_ENTRY(LB_BASE),
	REG_NAME_ENTRY(LB_STAT),
	REG_NAME_ENTRY(LB_L_PARAM),
	REG_NAME_ENTRY(LB_SB_DELTA)
};

static inline int ipu_lbp_dump_reg_simple(struct paintbox_data *pb,
		uint32_t reg_offset, char *buf, int *written, size_t len)
{
	const char *reg_name = ipu_lbp_reg_names[REG_INDEX(reg_offset)];

	return ipu_debug_dump_register(pb, IPU_CSR_LBP_OFFSET, reg_offset,
			reg_name, buf, written, len);
}

static int ipu_lbp_dump_reg(struct paintbox_data *pb, uint32_t reg_offset,
		char *buf, int *written, size_t len, const char *format, ...)
{
	va_list args;
	int ret;

	ret = ipu_lbp_dump_reg_simple(pb, reg_offset, buf, written, len);
	if (ret < 0)
		return ret;

	va_start(args, format);

	ret = ipu_debug_vprintf(pb, buf, written, len, format, args);

	va_end(args);

	return ret;
}

static int ipu_lbp_dmp_lbp_sel_register(struct paintbox_data *pb, char *buf,
		int *written, size_t len)
{
	uint64_t val;

	val = ipu_readq(pb->dev, IPU_CSR_LBP_OFFSET + LBP_SEL);
	return ipu_lbp_dump_reg(pb, LBP_SEL, buf, written, len,
			"\tLB_SEL 0x%02x LBP_SEL 0x%02x\n",
			(val & LBP_SEL_LB_SEL_MASK) >> LBP_SEL_LB_SEL_SHIFT,
			val & LBP_SEL_LBP_SEL_MASK);
}

static int ipu_lbp_dump_lbp_ctrl_register(struct paintbox_data *pb,
		char *buf, int *written, size_t len)
{
	uint64_t val;

	val = ipu_readq(pb->dev, IPU_CSR_LBP_OFFSET + LBP_CTRL);
	return ipu_lbp_dump_reg(pb, LBP_CTRL, buf, written, len,
			"\tLB_INIT 0x%02llx LB_RESET 0x%02llx RESET %d LBP_ENA 0x%02llx\n",
			(val & LBP_CTRL_LB_INIT_MASK) >> LBP_CTRL_LB_INIT_SHIFT,
			(val & LBP_CTRL_LB_RESET_MASK) >>
			LBP_CTRL_LB_RESET_SHIFT,
			!!(val & LBP_CTRL_LBP_RESET_MASK),
			val & LBP_CTRL_LB_ENA_MASK);
}

static int ipu_lbp_dump_lbp_stat_register(struct paintbox_data *pb,
		char *buf, int *written, size_t len)
{
	uint64_t val;

	val = ipu_readq(pb->dev, IPU_CSR_LBP_OFFSET + LBP_STAT);
	return ipu_lbp_dump_reg(pb, LBP_STAT, buf, written, len,
			"\tRD_READY %d WDC_READY %d CRB_READY %d\n",
			!!(val & LBP_STAT_RD_READY_MASK),
			!!(val & LBP_STAT_WDC_READY_MASK),
			!!(val & LBP_STAT_CRB_READY_MASK));
}

static int ipu_lbp_dump_lbp_cap0_register(struct paintbox_data *pb,
		char *buf, int *written, size_t len)
{
	uint64_t val;

	val = ipu_readq(pb->dev, IPU_CSR_LBP_OFFSET + LBP_CAP0);
	return ipu_lbp_dump_reg(pb, LBP_CAP0, buf, written, len,
			"\tMAX_RPTR %u MAX_FB_ROWS %u MAX_CHAN %u MAX_LB %u\n",
			(val & LBP_CAP0_MAX_RPTR_MASK) >>
			LBP_CAP0_MAX_RPTR_SHIFT,
			(val & LBP_CAP0_MAX_FB_ROWS_MASK) >>
			LBP_CAP0_MAX_FB_ROWS_SHIFT,
			(val & LBP_CAP0_MAX_CHAN_MASK) >>
			LBP_CAP0_MAX_CHAN_SHIFT,
			val & LBP_CAP0_MAX_LB_MASK);
}

static int ipu_lbp_dump_lbp_cap1_register(struct paintbox_data *pb,
		char *buf, int *written, size_t len)
{
	uint64_t val;

	val = ipu_readq(pb->dev, IPU_CSR_LBP_OFFSET + LBP_CAP1);
	return ipu_lbp_dump_reg(pb, LBP_CAP1, buf, written, len,
			"\tMEM_SIZE %u bytes\n", val);
}

static int ipu_lb_dump_lb_ctrl0_register(struct paintbox_data *pb,
		char *buf, int *written, size_t len)
{
	uint64_t val;

	val = ipu_readq(pb->dev, IPU_CSR_LBP_OFFSET + LB_CTRL0);
	return ipu_lbp_dump_reg(pb, LB_CTRL0, buf, written, len,
			"\tREUSE_ROWS %u FB_ROWS %u CHANs %u RPTRs %u\n",
			(val & LB_CTRL0_REUSE_ROWS_MASK) >>
			LB_CTRL0_REUSE_ROWS_SHIFT,
			(val & LB_CTRL0_FB_ROWS_MASK) >>
			LB_CTRL0_FB_ROWS_SHIFT,
			(val & LB_CTRL0_NUM_CHAN_MASK) >>
			LB_CTRL0_NUM_CHAN_SHIFT,
			val & LB_CTRL0_NUM_RPTR_MASK);
}

static int ipu_lb_dump_lb_offset_register(struct paintbox_data *pb,
		char *buf, int *written, size_t len)
{
	uint64_t val;

	val = ipu_readq(pb->dev, IPU_CSR_LBP_OFFSET + LB_OFFSET);
	return ipu_lbp_dump_reg(pb, LB_OFFSET, buf, written, len,
			"\tFB_OFFSET %d OFFSET_CHAN %u OFFSET_Y %d OFFSET_X %d\n",
			(int8_t)((val & LB_OFFSET_FB_OFFSET_MASK) >>
			LB_OFFSET_FB_OFFSET_SHIFT),
			(val & LB_OFFSET_OFFSET_CHAN_MASK) >>
			LB_OFFSET_OFFSET_CHAN_SHIFT,
			(int16_t)((val & LB_OFFSET_OFFSET_Y_MASK) >>
			LB_OFFSET_OFFSET_Y_SHIFT),
			(int16_t)(val & LB_OFFSET_OFFSET_X_MASK));
}

static int ipu_lb_dump_lb_bdry_register(struct paintbox_data *pb,
		char *buf, int *written, size_t len)
{
	uint64_t val;

	val = ipu_readq(pb->dev, IPU_CSR_LBP_OFFSET + LB_BDRY);
	return ipu_lbp_dump_reg(pb, LB_BDRY, buf, written, len,
			"\tBDRY_VAL 0x%04x BDRY 0x%x\n",
			(val & LB_BDRY_BDRY_VAL_MASK) >> LB_BDRY_BDRY_VAL_SHIFT,
			val & LB_BDRY_BDRY_MASK);
}

static int ipu_lb_dump_lb_img_size_register(struct paintbox_data *pb,
		char *buf, int *written, size_t len)
{
	uint64_t val;

	val = ipu_readq(pb->dev, IPU_CSR_LBP_OFFSET + LB_IMG_SIZE);
	return ipu_lbp_dump_reg(pb, LB_IMG_SIZE, buf, written, len,
			"\tHEIGHT %upx WIDTH %upx\n",
			(val & LB_IMG_SIZE_IMG_HEIGHT_MASK) >>
			LB_IMG_SIZE_IMG_HEIGHT_SHIFT,
			val & LB_IMG_SIZE_IMG_WIDTH_MASK);
}

static int ipu_lb_dump_lb_sb_size_register(struct paintbox_data *pb,
		char *buf, int *written, size_t len)
{
	uint64_t val;

	val = ipu_readq(pb->dev, IPU_CSR_LBP_OFFSET + LB_SB_SIZE);
	return ipu_lbp_dump_reg(pb, LB_SB_SIZE, buf, written, len,
			"\tSB_ROWS %u SB_COLS %u\n",
			(val & LB_SB_SIZE_SB_ROWS_MASK) >>
			LB_SB_SIZE_SB_ROWS_SHIFT,
			val & LB_SB_SIZE_SB_COLS_MASK);
}

static int ipu_lb_dump_lb_base_register(struct paintbox_data *pb,
		char *buf, int *written, size_t len)
{
	uint64_t val;

	val = ipu_readq(pb->dev, IPU_CSR_LBP_OFFSET + LB_BASE);
	return ipu_lbp_dump_reg(pb, LB_BASE, buf, written, len,
			"\tSB_BASE_ADDR 0x%08x FB_BASE_ADDR 0x%08x\n",
			((val & LB_BASE_SB_BASE_ADDR_MASK) >>
			LB_BASE_SB_BASE_ADDR_SHIFT) <<
			LB_BASE_SB_BASE_ALIGN_SHIFT,
			(val & LB_BASE_FB_BASE_ADDR_MASK) <<
			LB_BASE_FB_BASE_ALIGN_SHIFT);
}

static int ipu_lb_dump_lb_stat_register(struct paintbox_data *pb,
		char *buf, int *written, size_t len)
{
	uint64_t val;

	val = ipu_readq(pb->dev, IPU_CSR_LBP_OFFSET + LB_STAT);
	return ipu_lbp_dump_reg(pb, LB_STAT, buf, written, len,
			"\tRPTR2 EMPTY %d RPTR1 EMPTY %d RTPR0 EMPTY %d FULL %d\n",
			!!(val & LB_STAT_EMPTY2), !!(val & LB_STAT_EMPTY1),
			!!(val & LB_STAT_EMPTY0), !!(val & LB_STAT_FULL_MASK));
}

static int ipu_lb_dump_lb_l_param_register(struct paintbox_data *pb,
		char *buf, int *written, size_t len)
{
	uint64_t val;

	val = ipu_readq(pb->dev, IPU_CSR_LBP_OFFSET + LB_L_PARAM);
	return ipu_lbp_dump_reg(pb, LB_L_PARAM, buf, written, len,
			"\tL_WIDTH %u L_INC %u\n",
			(val & LB_L_PARAM_L_WIDTH_MASK) >>
			LB_L_PARAM_L_WIDTH_SHIFT, val & LB_L_PARAM_L_INC_MASK);
}

static int ipu_lb_dump_lb_sb_delta_register(struct paintbox_data *pb,
		char *buf, int *written, size_t len)
{
	uint64_t val;

	val = ipu_readq(pb->dev, IPU_CSR_LBP_OFFSET + LB_SB_DELTA);
	return ipu_lbp_dump_reg(pb, LB_SB_DELTA, buf, written, len,
			"\tSB_DELTA %u\n", val & LB_SB_DELTA_SB_DELTA_MASK);
}

/* The caller to this function must hold pb->lock */
int ipu_lbp_dump_registers(struct paintbox_debug *debug, char *buf, size_t len)
{
	struct paintbox_lbp *lbp = container_of(debug, struct paintbox_lbp,
			debug);
	struct paintbox_data *pb = debug->pb;
	unsigned int reg_offset;
	int ret, written = 0;

	ipu_lbp_select(pb, lbp->pool_id);

	ret = ipu_lbp_dmp_lbp_sel_register(pb, buf, &written, len);
	if (ret < 0)
		return ret;

	ret = ipu_lbp_dump_lbp_ctrl_register(pb, buf, &written, len);
	if (ret < 0)
		return ret;

	ret = ipu_lbp_dump_lbp_stat_register(pb, buf, &written, len);
	if (ret < 0)
		return ret;

	ret = ipu_lbp_dump_lbp_cap0_register(pb, buf, &written, len);
	if (ret < 0)
		return ret;

	ret = ipu_lbp_dump_lbp_cap1_register(pb, buf, &written, len);
	if (ret < 0)
		return ret;

	for (reg_offset = LBP_RAM_CTRL; reg_offset < LBP_POOL_BLOCK_LEN;
			reg_offset += IPU_REG_WIDTH) {
		if (!ipu_lbp_reg_names[REG_INDEX(reg_offset)])
			continue;

		ret = ipu_lbp_dump_reg_simple(pb, reg_offset, buf,
				&written, len);
		if (ret < 0)
			return ret;
	}

	return written;
}

/* The caller to this function must hold pb->lock */
int ipu_lb_dump_registers(struct paintbox_debug *debug, char *buf, size_t len)
{
	struct paintbox_lb *lb = container_of(debug, struct paintbox_lb, debug);
	struct paintbox_lbp *lbp = lb->lbp;
	struct paintbox_data *pb = debug->pb;
	int ret, written = 0;

	ipu_lb_select(pb, lbp->pool_id, lb->lb_id);

	ret = ipu_lb_dump_lb_ctrl0_register(pb, buf, &written, len);
	if (ret < 0)
		return ret;

	ret = ipu_lb_dump_lb_offset_register(pb, buf, &written, len);
	if (ret < 0)
		return ret;

	ret = ipu_lb_dump_lb_bdry_register(pb, buf, &written, len);
	if (ret < 0)
		return ret;

	ret = ipu_lb_dump_lb_img_size_register(pb, buf, &written, len);
	if (ret < 0)
		return ret;

	ret = ipu_lb_dump_lb_sb_size_register(pb, buf, &written, len);
	if (ret < 0)
		return ret;

	ret = ipu_lb_dump_lb_base_register(pb, buf, &written, len);
	if (ret < 0)
		return ret;

	ret = ipu_lb_dump_lb_stat_register(pb, buf, &written, len);
	if (ret < 0)
		return ret;

	ret = ipu_lb_dump_lb_l_param_register(pb, buf, &written, len);
	if (ret < 0)
		return ret;

	ret = ipu_lb_dump_lb_sb_delta_register(pb, buf, &written, len);
	if (ret < 0)
		return ret;

	return written;
}

void ipu_lbp_debug_init(struct paintbox_data *pb, struct paintbox_lbp *lbp)
{
	ipu_debug_create_entry(pb, &lbp->debug, pb->debug_root, "lbp",
			lbp->pool_id, ipu_lbp_dump_registers, NULL, lbp);

	ipu_debug_create_reg_entries(pb, &lbp->debug, ipu_lbp_reg_names,
			LBP_POOL_NUM_REGS, ipu_lbp_reg_entry_write,
			ipu_lbp_reg_entry_read);
}

void ipu_lbp_debug_remove(struct paintbox_data *pb,
		struct paintbox_lbp *lbp)
{
	ipu_debug_free_reg_entries(&lbp->debug);
	ipu_debug_free_entry(&lbp->debug);
}

void ipu_lb_debug_remove(struct paintbox_data *pb, struct paintbox_lb *lb)
{
	ipu_debug_free_reg_entries(&lb->debug);
	ipu_debug_free_entry(&lb->debug);
}

void ipu_lb_debug_init(struct paintbox_data *pb, struct paintbox_lbp *lbp,
		struct paintbox_lb *lb)
{
	ipu_debug_create_entry(pb, &lb->debug, lbp->debug.debug_dir, "lb",
			lb->lb_id, ipu_lb_dump_registers, NULL, lb);

	ipu_debug_create_reg_entries(pb, &lb->debug,
			&ipu_lbp_reg_names[REG_INDEX(LB_BLOCK_START)],
			LB_NUM_REGS, ipu_lb_reg_entry_write,
			ipu_lb_reg_entry_read);
}
