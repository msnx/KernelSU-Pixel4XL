/*
 * DMA debug support for the Paintbox programmable IPU
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
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "ipu-client.h"
#include "ipu-debug.h"
#include "ipu-dma.h"
#include "ipu-dma-debug.h"
#include "ipu-regs.h"

static const char *ipu_dma_top_reg_names[DMA_TOP_NUM_REGS] = {
	REG_NAME_ENTRY64(DMA_CTRL),
	REG_NAME_ENTRY64(DMA_IRQ_ISR),
	REG_NAME_ENTRY64(DMA_IRQ_VA_ERR_ISR),
	REG_NAME_ENTRY64(DMA_IRQ_SSP_ERR_ISR),
	REG_NAME_ENTRY64(DMA_IRQ_ITR),
	REG_NAME_ENTRY64(DMA_IRQ_VA_ERR_ITR),
	REG_NAME_ENTRY64(DMA_IRQ_SSP_ERR_ITR),
	REG_NAME_ENTRY64(DMA_IRQ_IER),
	REG_NAME_ENTRY64(DMA_IRQ_VA_ERR_IER),
	REG_NAME_ENTRY64(DMA_IRQ_SSP_ERR_IER),
	REG_NAME_ENTRY64(DMA_IRQ_IMR),
	REG_NAME_ENTRY64(DMA_IRQ_VA_ERR_IMR),
	REG_NAME_ENTRY64(DMA_IRQ_SSP_ERR_IMR),
	REG_NAME_ENTRY64(DMA_IRQ_ISR_OVF),
	REG_NAME_ENTRY64(DMA_IRQ_VA_ERR_ISR_OVF),
	REG_NAME_ENTRY64(DMA_IRQ_SSP_ERR_ISR_OVF),
	REG_NAME_ENTRY64(DMA_PMON_CFG),
	REG_NAME_ENTRY64(DMA_PMON_CNT_0_CFG),
	REG_NAME_ENTRY64(DMA_PMON_CNT_0),
	REG_NAME_ENTRY64(DMA_PMON_CNT_0_STS_ACC),
	REG_NAME_ENTRY64(DMA_PMON_CNT_0_STS),
	REG_NAME_ENTRY64(DMA_PMON_CNT_1_CFG),
	REG_NAME_ENTRY64(DMA_PMON_CNT_1),
	REG_NAME_ENTRY64(DMA_PMON_CNT_1_STS_ACC),
	REG_NAME_ENTRY64(DMA_PMON_CNT_1_STS),
	REG_NAME_ENTRY64(DMA_PMON_CNT_2_CFG),
	REG_NAME_ENTRY64(DMA_PMON_CNT_2),
	REG_NAME_ENTRY64(DMA_PMON_CNT_2_STS_ACC),
	REG_NAME_ENTRY64(DMA_PMON_CNT_2_STS),
	REG_NAME_ENTRY64(DMA_PMON_CNT_3_CFG),
	REG_NAME_ENTRY64(DMA_PMON_CNT_3),
	REG_NAME_ENTRY64(DMA_PMON_CNT_3_STS_ACC),
	REG_NAME_ENTRY64(DMA_PMON_CNT_3_STS),
	REG_NAME_ENTRY64(DMA_STAT_CTRL),
	REG_NAME_ENTRY64(DMA_STAT_STATE),
	REG_NAME_ENTRY64(DMA_STAT_PTR),
	REG_NAME_ENTRY64(DMA_STAT_ADDR),
	REG_NAME_ENTRY64(SSP_STATUS),
	REG_NAME_ENTRY64(DMA_SPARE)
};

static const char *ipu_dma_grp_reg_names[DMA_GRP_NUM_REGS] = {
	REG_NAME_ENTRY64(DMA_SEL),
	REG_NAME_ENTRY64(DMA_CHAN_CTRL),
	REG_NAME_ENTRY64(DMA_CHAN_MODE),
	REG_NAME_ENTRY64(DMA_CHAN_IMG_FORMAT),
	REG_NAME_ENTRY64(DMA_CHAN_IMG_SIZE),
	REG_NAME_ENTRY64(DMA_CHAN_IMG_POS),
	REG_NAME_ENTRY64(DMA_CHAN_IMG_LAYOUT),
	REG_NAME_ENTRY64(DMA_CHAN_BIF_XFER),
	REG_NAME_ENTRY64(DMA_CHAN_VA),
	REG_NAME_ENTRY64(DMA_CHAN_VA_BDRY),
	REG_NAME_ENTRY64(DMA_CHAN_NOC_XFER),
	REG_NAME_ENTRY64(DMA_CHAN_SSP_CFG),
	REG_NAME_ENTRY64(DMA_CHAN_NODE),
	REG_NAME_ENTRY64(DMA_CHAN_MODE_RO),
	REG_NAME_ENTRY64(DMA_CHAN_IMG_FORMAT_RO),
	REG_NAME_ENTRY64(DMA_CHAN_IMG_SIZE_RO),
	REG_NAME_ENTRY64(DMA_CHAN_IMG_POS_RO),
	REG_NAME_ENTRY64(DMA_CHAN_IMG_LAYOUT_RO),
	REG_NAME_ENTRY64(DMA_CHAN_BIF_XFER_RO),
	REG_NAME_ENTRY64(DMA_CHAN_VA_RO),
	REG_NAME_ENTRY64(DMA_CHAN_VA_BDRY_RO),
	REG_NAME_ENTRY64(DMA_CHAN_NOC_XFER_RO),
	REG_NAME_ENTRY64(DMA_CHAN_SSP_CFG_RO),
	REG_NAME_ENTRY64(DMA_CHAN_NODE_RO),
	REG_NAME_ENTRY64(DMA_GRP_SPARE)
};

/* The caller to this function must hold pb->lock */
static inline void ipu_dma_select_channel(struct paintbox_data *pb,
		unsigned int channel_id)
{
	ipu_writel(pb->dev, channel_id & (DMA_SEL_GRP_SEL_MASK |
			DMA_SEL_CHAN_SEL_MASK),
			IPU_CSR_DMA_GRP_OFFSET + DMA_SEL);
}

static inline void ipu_dma_dump_top_reg(struct seq_file *s,
		unsigned int reg_offset, uint64_t reg_value)
{
	const char *reg_name = ipu_dma_top_reg_names[REG_INDEX64(reg_offset)];

	seq_printf(s, "0x%04llx: %-*s0x%016llx\n",
			IPU_CSR_DMA_TOP_OFFSET + reg_offset,
			REG_VALUE_COL_WIDTH, reg_name, reg_value);
}

static inline void ipu_dma_dump_grp_reg(struct seq_file *s,
		unsigned int reg_offset, uint64_t reg_value)
{
	const char *reg_name = ipu_dma_grp_reg_names[REG_INDEX64(reg_offset)];

	seq_printf(s, "0x%04llx: %-*s0x%016llx\n",
			IPU_CSR_DMA_GRP_OFFSET + reg_offset,
			REG_VALUE_COL_WIDTH, reg_name, reg_value);
}

static inline const char *ipu_dma_swizzle_to_str(uint64_t val)
{
	switch ((val & DMA_CTRL_AXI_SWIZZLE_MASK) >>
			DMA_CTRL_AXI_SWIZZLE_SHIFT) {
	case DMA_CTRL_AXI_SWIZZLE_NONE:
		return "NONE";
	case DMA_CTRL_AXI_SWIZZLE_BIG_ENDIAN:
		return "BIG ENDIAN";
	case DMA_CTRL_AXI_SWIZZLE_NEIGHBOR_BYTES:
		return "NEIGHBOR BYTES";
	default:
		return "UNKNOWN";
	};
}

static const char *ipu_dma_state_src_mode_to_str(uint64_t dma_stat_state_val,
		uint32_t chan_mode_val)
{
	uint32_t src;

	src = (chan_mode_val & DMA_CHAN_MODE_SRC_MASK) >>
			DMA_CHAN_MODE_SRC_SHIFT;

	if (src == DMA_CHAN_MODE_SRC_DRAM) {
		switch (dma_stat_state_val & DMA_STAT_STATE_SRC_MASK) {
		case 0:
			return "Idle";
		case 1:
			return "Get Gather Coord";
		case 3:
			return "Wait Segment Grant";
		case 4:
			return "Add Calc Delay";
		case 5:
			return "Write Ctl";
		case 6:
			return "Request Data From BIF";
		case 7:
			return "Reset State";
		default:
			return "UNKNOWN";
		};
	} else if (src == DMA_CHAN_MODE_SRC_LBP) {
		switch (dma_stat_state_val & DMA_STAT_STATE_SRC_MASK) {
		case 0:
			return "Cmd Idle";
		case 1:
			return "Request Segments";
		case 2:
			return "Wait Segment Grant";
		case 3:
			return "Cmd";
		case 4:
			return "Cmd Wait";
		case 5:
			return "Retry Cmd";
		case 6:
			return "Ack Wait";
		case 7:
			return "Check Avail";
		default:
			return "UNKNOWN";
		};
	} else {
		return "UNKNOWN";
	}
}

static const char *ipu_dma_state_dst_mode_to_str(uint64_t dma_stat_state_val,
		uint64_t chan_mode_val)
{
	uint32_t dst;

	dst = (chan_mode_val & DMA_CHAN_MODE_DST_MASK) >>
			DMA_CHAN_MODE_DST_SHIFT;

	if (dst == DMA_CHAN_MODE_DST_DRAM) {
		switch ((dma_stat_state_val & DMA_STAT_STATE_DST_MASK) >>
				DMA_STAT_STATE_DST_SHIFT) {
		case 0:
			return "Idle";
		case 1:
			return "Wait Segment";
		case 2:
			return "SSP Read Reqs";
		case 3:
			return "Load Address 0";
		case 4:
			return "Load Address 1";
		case 5:
			return "Load Address 2";
		case 6:
			return "Load Ptrs";
		case 7:
			return "DRAM Write Req";
		case 8:
			return "Wait Ptr";
		case 9:
			return "DRAM Pull Wait";
		case 10:
			return "Check If Last";
		case 11:
			return "DRAM Write Ack Wait";
		default:
			return "UNKNOWN";
		};
	} else if (dst == DMA_CHAN_MODE_DST_LBP) {
		switch ((dma_stat_state_val & DMA_STAT_STATE_DST_MASK) >>
				DMA_STAT_STATE_DST_SHIFT) {
		case 0:
			return "Cmd Idle";
		case 1:
			return "Cmd";
		case 2:
			return "Ack Wait";
		case 3:
			return "Retry Cmd";
		case 4:
			return "Load Ptrs";
		case 5:
			return "Cmd EOF";
		case 6:
			return "Check Avail";
		default:
			return "UNKNOWN";
		};
	} else {
		return "UNKNOWN";
	}
}

static inline unsigned int ipu_dma_get_bit_depth(uint64_t val)
{
	switch ((val & DMA_CHAN_IMG_FORMAT_BIT_DEPTH_MASK) >>
			DMA_CHAN_IMG_FORMAT_BIT_DEPTH_SHIFT) {
	case DMA_CHAN_IMG_FORMAT_BIT_DEPTH8:
		return 8;
	case DMA_CHAN_IMG_FORMAT_BIT_DEPTH10:
		return 10;
	case DMA_CHAN_IMG_FORMAT_BIT_DEPTH12:
		return 12;
	case DMA_CHAN_IMG_FORMAT_BIT_DEPTH14:
		return 14;
	case DMA_CHAN_IMG_FORMAT_BIT_DEPTH16:
		return 16;
	default:
		return 0;
	};
}

static inline const char *ipu_dma_src_to_str(uint64_t val)
{
	switch ((val & DMA_CHAN_MODE_SRC_MASK) >> DMA_CHAN_MODE_SRC_SHIFT) {
	case DMA_CHAN_MODE_SRC_DRAM:
		return "DRAM";
	case DMA_CHAN_MODE_SRC_LBP:
		return "LBP";
	case DMA_CHAN_MODE_SRC_STP:
		return "STP";
	case DMA_CHAN_MODE_SRC_MIPI_IN:
		return "MIPI IN";
	default:
		return "UNKNOWN";
	};
}

static inline const char *ipu_dma_dst_to_str(uint64_t val)
{
	switch ((val & DMA_CHAN_MODE_DST_MASK) >> DMA_CHAN_MODE_DST_SHIFT) {
	case DMA_CHAN_MODE_DST_DRAM:
		return "DRAM";
	case DMA_CHAN_MODE_DST_LBP:
		return "LBP";
	case DMA_CHAN_MODE_DST_STP:
		return "STP";
	case DMA_CHAN_MODE_DST_MIPI_OUT:
		return "MIPI OUT";
	default:
		return "UNKNOWN";
	};
}

static void ipu_dma_debug_dump_dma_sel(struct seq_file *s,
		uint64_t reg_value)
{
	ipu_dma_dump_grp_reg(s, DMA_SEL, reg_value);
	seq_printf(s, "\tGRP_SEL %llu CHAN_SEL %llu\n",
			(reg_value & DMA_SEL_GRP_SEL_MASK) >>
			DMA_SEL_GRP_SEL_SHIFT,
			reg_value & DMA_SEL_CHAN_SEL_MASK);
}

static void ipu_dma_debug_dump_dma_chan_ctrl(struct seq_file *s,
		uint64_t reg_value)
{
	ipu_dma_dump_grp_reg(s, DMA_CHAN_CTRL, reg_value);
	seq_printf(s, "\tSTOP %llu CONTINUOUS %llu CHAN_RESET %llu\n",
			(reg_value & DMA_CHAN_CTRL_STOP_MASK) >>
					DMA_CHAN_CTRL_STOP_SHIFT,
			(reg_value & DMA_CHAN_CTRL_CONTINUOUS_MASK) >>
					DMA_CHAN_CTRL_CONTINUOUS_SHIFT,
			reg_value & DMA_CHAN_CTRL_CHAN_RESET_MASK);
}

static void ipu_dma_debug_dump_dma_chan_mode(struct seq_file *s,
		unsigned int reg_offset,
		uint64_t reg_value)
{
	ipu_dma_dump_grp_reg(s, reg_offset, reg_value);
	seq_printf(s, "\tPRI %u SIGN_EXT %u\n",
			!!(reg_value & DMA_CHAN_MODE_PRI_MASK),
			!!(reg_value & DMA_CHAN_MODE_SIGN_EXT_MASK));
	seq_printf(s, "\tGATHER %u ADDR_MODE %s DST %s SRC %s ENA %u\n",
			!!(reg_value & DMA_CHAN_MODE_GATHER_MASK),
			(reg_value & DMA_CHAN_MODE_ADDR_MODE_MASK) ?
					"PHYSICAL" : "ABSTRACT",
			ipu_dma_dst_to_str(reg_value),
			ipu_dma_src_to_str(reg_value),
			!!(reg_value & DMA_CHAN_MODE_CHAN_ENA_MASK));
}

static void ipu_dma_debug_dump_dma_chan_image(struct seq_file *s,
		unsigned int img_format_offset, uint64_t img_format_value,
		unsigned int img_size_offset, uint64_t img_size_value,
		unsigned int img_pos_offset, uint64_t img_pos_value,
		unsigned int img_layout_offset, uint64_t img_layout_value)
{
	uint64_t val;

	val = img_format_value;
	ipu_dma_dump_grp_reg(s, img_format_offset, val);
	seq_printf(s,
			"\tBLOCK_4x4 %u MIPI_RAW_FORMAT %d BIT DEPTH %u PLANES %llu COMPONENTS %llu\n",
			!!(val & DMA_CHAN_IMG_FORMAT_BLOCK_4X4_MASK),
			!!(val & DMA_CHAN_IMG_FORMAT_MIPI_RAW_FORMAT_MASK),
			ipu_dma_get_bit_depth(val),
			((val & DMA_CHAN_IMG_FORMAT_PLANES_MASK) >>
					DMA_CHAN_IMG_FORMAT_PLANES_SHIFT) + 1,
			(val & DMA_CHAN_IMG_FORMAT_COMPONENTS_MASK) + 1);

	val = img_size_value;
	ipu_dma_dump_grp_reg(s, img_size_offset, val);
	seq_printf(s, "\tIMG_HEIGHT %llu IMG_WIDTH %llu\n",
			(val & DMA_CHAN_IMG_SIZE_IMG_HEIGHT_MASK) >>
			DMA_CHAN_IMG_SIZE_IMG_HEIGHT_SHIFT,
			val & DMA_CHAN_IMG_SIZE_IMG_WIDTH_MASK);

	val = img_pos_value;
	ipu_dma_dump_grp_reg(s, img_pos_offset, val);
	seq_printf(s, "\tLB_START_Y %d LB_START_X %d START_Y %d START_X %d\n",
			(int16_t)((val & DMA_CHAN_IMG_POS_LB_START_Y_MASK) >>
					DMA_CHAN_IMG_POS_LB_START_Y_SHIFT),
			(int16_t)((val & DMA_CHAN_IMG_POS_LB_START_X_MASK) >>
					DMA_CHAN_IMG_POS_LB_START_X_SHIFT),
			(int16_t)((val & DMA_CHAN_IMG_POS_START_Y_MASK) >>
					DMA_CHAN_IMG_POS_START_Y_SHIFT),
			(int16_t)(val & DMA_CHAN_IMG_POS_START_X_MASK));

	val = img_layout_value;
	ipu_dma_dump_grp_reg(s, img_layout_offset, val);
	seq_printf(s, "\tPLANE_STRIDE %llu ROW_STRIDE %llu\n",
			(val & DMA_CHAN_IMG_LAYOUT_PLANE_STRIDE_MASK) >>
					DMA_CHAN_IMG_LAYOUT_PLANE_STRIDE_SHIFT,
			(val & DMA_CHAN_IMG_LAYOUT_ROW_STRIDE_MASK) >>
					DMA_CHAN_IMG_LAYOUT_ROW_STRIDE_SHIFT);
}

static void ipu_dma_debug_dump_dma_chan_bif_xfer(struct seq_file *s,
		unsigned int reg_offset,
		uint64_t reg_value)
{
	ipu_dma_dump_grp_reg(s, reg_offset, reg_value);
	seq_printf(s, "\tOUTSTANDING %llu STRIPE_HEIGHT %llu\n",
			(reg_value & DMA_CHAN_BIF_XFER_OUTSTANDING_MASK) >>
					DMA_CHAN_BIF_XFER_OUTSTANDING_SHIFT,
			reg_value & DMA_CHAN_BIF_XFER_STRIPE_HEIGHT_MASK);
}

static void ipu_dma_debug_dump_dma_chan_va(struct seq_file *s,
		unsigned int reg_offset,
		uint64_t reg_value)
{
	ipu_dma_dump_grp_reg(s, reg_offset, reg_value);
	seq_printf(s, "\tVA 0x%016llx\n", reg_value & DMA_CHAN_VA_BASE_MASK);
}

static void ipu_dma_debug_dump_dma_chan_va_bdry(struct seq_file *s,
		unsigned int reg_offset,
		uint64_t reg_value)
{
	ipu_dma_dump_grp_reg(s, reg_offset, reg_value);
	seq_printf(s, "\tVA BDRY %llu\n",
			reg_value & DMA_CHAN_VA_BDRY_LEN_MASK);
}

static void ipu_dma_debug_dump_dma_chan_noc_xfer(struct seq_file *s,
		unsigned int reg_offset,
		uint64_t reg_value)
{
	ipu_dma_dump_grp_reg(s, reg_offset, reg_value);
	seq_printf(s,
			"\tRETRY_INTERVAL %llu OUTSTANDING %llu SHEET_HEIGHT %llu SHEET_WIDTH %llu\n",
			(reg_value & DMA_CHAN_NOC_XFER_RETRY_INTERVAL_MASK) >>
					DMA_CHAN_NOC_XFER_RETRY_INTERVAL_SHIFT,
			(reg_value & DMA_CHAN_NOC_XFER_OUTSTANDING_MASK) >>
					DMA_CHAN_NOC_XFER_OUTSTANDING_SHIFT,
			(reg_value & DMA_CHAN_NOC_XFER_SHEET_HEIGHT_MASK) >>
					DMA_CHAN_NOC_XFER_SHEET_HEIGHT_SHIFT,
			reg_value & DMA_CHAN_NOC_XFER_SHEET_WIDTH_MASK);
}

static void ipu_dma_debug_dump_dma_chan_ssp_cfg(struct seq_file *s,
		unsigned int reg_offset,
		uint64_t reg_value)
{
	ipu_dma_dump_grp_reg(s, reg_offset, reg_value);
	seq_printf(s,
			"\tMULT_SEGS %d MULT_SHEETS %d PTRS_FIFO_DEPTH %llu LOCS_PER_ROW %llu PIX_PER_LOC %llu\n",
			!!(reg_value & DMA_CHAN_SSP_CFG_MULT_SEGS_MASK),
			!!(reg_value & DMA_CHAN_SSP_CFG_MULT_SHEETS_MASK),
			(reg_value & DMA_CHAN_SSP_CFG_PTRS_FIFO_DEPTH_MASK) >>
					DMA_CHAN_SSP_CFG_PTRS_FIFO_DEPTH_SHIFT,
			(reg_value & DMA_CHAN_SSP_CFG_LOCS_PER_ROW_MASK) >>
					DMA_CHAN_SSP_CFG_LOCS_PER_ROW_SHIFT,
			reg_value & DMA_CHAN_SSP_CFG_PIX_PER_LOC_MASK);
}

static void ipu_dma_debug_dump_dma_chan_node(struct seq_file *s,
		unsigned int reg_offset,
		uint64_t reg_value)
{
	ipu_dma_dump_grp_reg(s, reg_offset, reg_value);
	seq_printf(s, "\tSLICE_ID_LSB %llu SLICE_ID_WIDTH %llu NOC_PORT %u\n",
			(reg_value & DMA_CHAN_NODE_SLICE_ID_LSB_MASK) >>
					DMA_CHAN_NODE_SLICE_ID_LSB_SHIFT,
			(reg_value & DMA_CHAN_NODE_SLICE_ID_WIDTH_MASK) >>
					DMA_CHAN_NODE_SLICE_ID_WIDTH_SHIFT,
			!!(reg_value & DMA_CHAN_NODE_NOC_PORT_MASK));
	seq_printf(s, "\tCORE_ID %llu LB_ID %llu RPTR_ID %llu\n",
			reg_value & DMA_CHAN_NODE_CORE_ID_MASK,
			(reg_value & DMA_CHAN_NODE_LB_ID_MASK) >>
					DMA_CHAN_NODE_LB_ID_SHIFT,
			(reg_value & DMA_CHAN_NODE_RPTR_ID_MASK) >>
					DMA_CHAN_NODE_RPTR_ID_SHIFT);
}

static void ipu_dma_debug_dump_dma_ctrl(struct seq_file *s,
		uint64_t reg_value)
{
	ipu_dma_dump_top_reg(s, DMA_CTRL, reg_value);
	seq_printf(s, "\tAXI_SWIZZLE %s RESET %lld\n",
			ipu_dma_swizzle_to_str(reg_value),
			reg_value & DMA_CTRL_DMA_RESET_MASK);
}

static void ipu_dma_debug_dump_dma_stat_regs(struct seq_file *s,
	uint64_t dma_stat_ctrl_val,
	uint64_t dma_stat_state_val,
	uint64_t dma_stat_ptr_val,
	uint64_t dma_stat_addr_val,
	uint64_t dma_grp_chan_mode_r0)
{
	uint64_t mode_val, val;

	val = dma_stat_ctrl_val;
	ipu_dma_dump_top_reg(s, DMA_STAT_CTRL, val);
	seq_printf(s, "\tNOC_MNGR_PORT %d DST_BIF_SEL_SSP %d ADDR_MODE %d ENABLE %d\n",
			!!(val & DMA_STAT_CTRL_NOC_MNGR_PORT_MASK),
			!!(val & DMA_STAT_CTRL_DST_BIF_SEL_SSP_MASK),
			!!(val & DMA_STAT_CTRL_ADDR_MODE_MASK),
			!!(val & DMA_STAT_CTRL_ENABLE_MASK));

	val = dma_stat_state_val;
	mode_val = dma_grp_chan_mode_r0;
	ipu_dma_dump_top_reg(s, DMA_STAT_STATE, val);
	seq_printf(s, "\tWR_MNGR: CHAN %llu MODE %d STATE %lld\n",
			(val & DMA_STAT_STATE_WR_MNGR_CHAN_MASK) >>
			DMA_STAT_STATE_WR_MNGR_CHAN_SHIFT,
			!!(val & DMA_STAT_STATE_WR_MNGR_STATE_MODE_MASK),
			(val & DMA_STAT_STATE_WR_MNGR_STATE_MASK) >>
			DMA_STAT_STATE_WR_MNGR_STATE_SHIFT);
	seq_printf(s, "\tRD_MNGR: CHAN %llu MODE %d\n",
			(val & DMA_STAT_STATE_RD_MNGR_CHAN_MASK) >>
			DMA_STAT_STATE_RD_MNGR_CHAN_SHIFT,
			!!(val & DMA_STAT_STATE_RD_MNGR_CHAN_MODE_MASK));
	seq_printf(s, "\tDST_MODE %d DST %llu (%s) SRC_MODE %d SRC %llu (%s)\n",
			!!(val & DMA_STAT_STATE_DST_MODE_MASK),
			(val & DMA_STAT_STATE_DST_MASK) >>
			DMA_STAT_STATE_DST_SHIFT,
			ipu_dma_state_dst_mode_to_str(val, mode_val),
			!!(val & DMA_STAT_STATE_SRC_MODE_MASK),
			val & DMA_STAT_STATE_SRC_MASK,
			ipu_dma_state_src_mode_to_str(val, mode_val));

	val = dma_stat_ptr_val;
	ipu_dma_dump_top_reg(s, DMA_STAT_PTR, val);
	seq_printf(s, "\tPTR_MODE %d SHEET_HEIGHT %llu SHEET WIDTH %llu Y %llu X %llu\n",
			!!(val & DMA_STAT_PTR_MODE_MASK),
			(val & DMA_STAT_PTR_SHEET_HEIGHT_MASK) >>
			DMA_STAT_PTR_SHEET_HEIGHT_SHIFT,
			(val & DMA_STAT_PTR_SHEET_WIDTH_MASK) >>
			DMA_STAT_PTR_SHEET_WIDTH_SHIFT,
			(val & DMA_STAT_PTR_Y_MASK) >> DMA_STAT_PTR_Y_SHIFT,
			val & DMA_STAT_PTR_X_MASK);
}

static void ipu_dma_debug_dump_ssp_status(struct seq_file *s,
		uint64_t reg_value)
{
	ipu_dma_dump_top_reg(s, SSP_STATUS, reg_value);
	seq_printf(s, "\tSEG_AVAIL %llu\n", reg_value);
}

int ipu_dma_channel_debug_read_registers(struct seq_file *s, void *data)
{
	struct ipu_dma_channel_debug_regs *debug_reg_dump = s->private;
	struct paintbox_data *pb = debug_reg_dump->pb;
	struct paintbox_dma_channel *channel = debug_reg_dump->channel;
	uint64_t dma_grp_registers[DMA_GRP_NUM_REGS];
	uint64_t dma_stat_ctrl_val;
	uint64_t dma_stat_state_val;
	uint64_t dma_stat_ptr_val;
	uint64_t dma_stat_addr_val;
	uint64_t img_format_value;
	uint64_t img_size_value;
	uint64_t img_pos_value;
	uint64_t img_layout_value;
	uint64_t ssp_status_val;
	unsigned int reg_offset;
	uint64_t val;
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

	ipu_dma_select_channel(pb, channel->channel_id);

	for (reg_offset = 0; reg_offset < DMA_GRP_BLOCK_END; reg_offset +=
			IPU_REG_WIDTH_BYTES) {
		if (!ipu_dma_grp_reg_names[REG_INDEX64(reg_offset)])
			continue;

		dma_grp_registers[REG_INDEX64(reg_offset)] = ipu_readq(pb->dev,
				IPU_CSR_DMA_GRP_OFFSET + reg_offset);
	}
	dma_stat_ctrl_val = ipu_readq(pb->dev,
			IPU_CSR_DMA_TOP_OFFSET + DMA_STAT_CTRL);
	dma_stat_state_val = ipu_readq(pb->dev,
			IPU_CSR_DMA_TOP_OFFSET + DMA_STAT_STATE);
	dma_stat_ptr_val = ipu_readq(pb->dev,
			IPU_CSR_DMA_TOP_OFFSET + DMA_STAT_PTR);
	dma_stat_addr_val = ipu_readq(pb->dev,
			IPU_CSR_DMA_TOP_OFFSET + DMA_STAT_ADDR);
	ssp_status_val = ipu_readq(pb->dev,
			IPU_CSR_DMA_TOP_OFFSET + SSP_STATUS);

	ipu_dma_debug_dump_dma_chan_ctrl(s,
			dma_grp_registers[REG_INDEX64(DMA_CHAN_CTRL)]);

	ipu_dma_debug_dump_dma_chan_mode(s, DMA_CHAN_MODE,
			dma_grp_registers[REG_INDEX64(DMA_CHAN_MODE)]);

	img_format_value = dma_grp_registers[REG_INDEX64(DMA_CHAN_IMG_FORMAT)];
	img_size_value = dma_grp_registers[REG_INDEX64(DMA_CHAN_IMG_SIZE)];
	img_pos_value = dma_grp_registers[REG_INDEX64(DMA_CHAN_IMG_POS)];
	img_layout_value = dma_grp_registers[REG_INDEX64(DMA_CHAN_IMG_LAYOUT)];
	ipu_dma_debug_dump_dma_chan_image(s,
		DMA_CHAN_IMG_FORMAT, img_format_value,
		DMA_CHAN_IMG_SIZE, img_size_value,
		DMA_CHAN_IMG_POS, img_pos_value,
		DMA_CHAN_IMG_LAYOUT, img_layout_value);

	ipu_dma_debug_dump_dma_chan_bif_xfer(s, DMA_CHAN_BIF_XFER,
			dma_grp_registers[REG_INDEX64(DMA_CHAN_BIF_XFER)]);

	ipu_dma_debug_dump_dma_chan_va(s, DMA_CHAN_VA,
			dma_grp_registers[REG_INDEX64(DMA_CHAN_VA)]);

	ipu_dma_debug_dump_dma_chan_va_bdry(s, DMA_CHAN_VA_BDRY,
			dma_grp_registers[REG_INDEX64(DMA_CHAN_VA_BDRY)]);

	ipu_dma_debug_dump_dma_chan_noc_xfer(s, DMA_CHAN_NOC_XFER,
		dma_grp_registers[REG_INDEX64(DMA_CHAN_NOC_XFER)]);

	ipu_dma_debug_dump_dma_chan_ssp_cfg(s, DMA_CHAN_SSP_CFG,
			dma_grp_registers[REG_INDEX64(DMA_CHAN_SSP_CFG)]);

	ipu_dma_debug_dump_dma_chan_node(s, DMA_CHAN_NODE,
			dma_grp_registers[REG_INDEX64(DMA_CHAN_NODE)]);

	ipu_dma_debug_dump_dma_chan_mode(s, DMA_CHAN_MODE_RO,
			dma_grp_registers[REG_INDEX64(DMA_CHAN_MODE_RO)]);

	img_format_value =
			dma_grp_registers[REG_INDEX64(DMA_CHAN_IMG_FORMAT_RO)];
	img_size_value = dma_grp_registers[REG_INDEX64(DMA_CHAN_IMG_SIZE_RO)];
	img_pos_value = dma_grp_registers[REG_INDEX64(DMA_CHAN_IMG_POS_RO)];
	img_layout_value =
			dma_grp_registers[REG_INDEX64(DMA_CHAN_IMG_LAYOUT_RO)];
	ipu_dma_debug_dump_dma_chan_image(s,
		DMA_CHAN_IMG_FORMAT_RO, img_format_value,
		DMA_CHAN_IMG_SIZE_RO, img_size_value,
		DMA_CHAN_IMG_POS_RO, img_pos_value,
		DMA_CHAN_IMG_LAYOUT_RO, img_layout_value);

	ipu_dma_debug_dump_dma_chan_bif_xfer(s, DMA_CHAN_BIF_XFER_RO,
			dma_grp_registers[REG_INDEX64(DMA_CHAN_BIF_XFER_RO)]);

	ipu_dma_debug_dump_dma_chan_bif_xfer(s, DMA_CHAN_BIF_XFER_RO,
			dma_grp_registers[REG_INDEX64(DMA_CHAN_BIF_XFER_RO)]);

	ipu_dma_debug_dump_dma_chan_va(s, DMA_CHAN_VA_RO,
			dma_grp_registers[REG_INDEX64(DMA_CHAN_VA_RO)]);

	ipu_dma_debug_dump_dma_chan_va_bdry(s, DMA_CHAN_VA_BDRY_RO,
			dma_grp_registers[REG_INDEX64(DMA_CHAN_VA_BDRY_RO)]);

	ipu_dma_debug_dump_dma_chan_noc_xfer(s, DMA_CHAN_NOC_XFER_RO,
		dma_grp_registers[REG_INDEX64(DMA_CHAN_NOC_XFER_RO)]);

	ipu_dma_debug_dump_dma_chan_ssp_cfg(s, DMA_CHAN_SSP_CFG_RO,
			dma_grp_registers[REG_INDEX64(DMA_CHAN_SSP_CFG_RO)]);

	ipu_dma_debug_dump_dma_chan_node(s, DMA_CHAN_NODE_RO,
			dma_grp_registers[REG_INDEX64(DMA_CHAN_NODE_RO)]);


	if (pb->dma.debug_enabled) {
		val = dma_grp_registers[REG_INDEX64(DMA_CHAN_MODE_RO)];
		ipu_dma_debug_dump_dma_stat_regs(s,
				dma_stat_ctrl_val,
				dma_stat_state_val,
				dma_stat_ptr_val,
				dma_stat_addr_val,
				val);

		ipu_dma_debug_dump_ssp_status(s, ssp_status_val);
	}

	ret = ipu_jqs_put(pb);

	mutex_unlock(&pb->lock);

	return ret;
}

static int ipu_dma_channel_debug_register_set(void *data, u64 val)
{
	struct ipu_dma_channel_debug_register *reg =
			(struct ipu_dma_channel_debug_register *)data;
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

	ipu_dma_select_channel(pb, reg->channel_id);
	ipu_writeq(pb->dev, val, reg->debug_register.offset);

	ret = ipu_jqs_put(pb);

	mutex_unlock(&pb->lock);

	return ret;
}

static int ipu_dma_channel_debug_register_get(void *data, u64 *val)
{
	struct ipu_dma_channel_debug_register *reg =
			(struct ipu_dma_channel_debug_register *)data;
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

	ipu_dma_select_channel(pb, reg->channel_id);
	*val = ipu_readq(pb->dev, reg->debug_register.offset);

	ret = ipu_jqs_put(pb);

	mutex_unlock(&pb->lock);

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(ipu_dma_channel_debug_register_fops,
		ipu_dma_channel_debug_register_get,
		ipu_dma_channel_debug_register_set, "%llx\n");

static void ipu_dma_channel_debug_create_register_file(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct ipu_dma_channel_debug_register *reg, const char *name,
		unsigned int offset)
{
	reg->channel_id = channel->channel_id;
	reg->debug_register.offset = IPU_CSR_DMA_GRP_OFFSET + offset;
	reg->debug_register.pb = pb;

	reg->debug_register.dentry = debugfs_create_file(name, 0640,
			channel->debug_dir, reg,
			&ipu_dma_channel_debug_register_fops);
	if (WARN_ON(IS_ERR(reg->debug_register.dentry)))
		reg->debug_register.dentry = NULL;
}


static int ipu_dma_channel_reg_dump_open(struct inode *inode, struct file *file)
{
	return single_open_size(file, ipu_dma_channel_debug_read_registers,
			inode->i_private,
			DMA_GRP_NUM_REGS * REG_DEBUG_BUFFER_SIZE);
}

static const struct file_operations ipu_dma_channel_reg_dump_fops = {
	.open = ipu_dma_channel_reg_dump_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void ipu_dma_channel_debug_init(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	char channel_name[RESOURCE_NAME_LEN];
	int ret, i;

	channel->debug_reg_dump.pb = pb;
	channel->debug_reg_dump.channel = channel;

	ret = scnprintf(channel_name, RESOURCE_NAME_LEN, "%s%u", "channel",
		channel->channel_id);

	channel->debug_dir = debugfs_create_dir(channel_name,
			pb->dma.debug_dir);
	if (WARN_ON(IS_ERR_OR_NULL(channel->debug_dir)))
		return;

	channel->debug_reg_dump.dentry = debugfs_create_file("regs", 0640,
			channel->debug_dir, &channel->debug_reg_dump,
			&ipu_dma_channel_reg_dump_fops);
	if (WARN_ON(IS_ERR_OR_NULL(channel->debug_reg_dump.dentry)))
		return;


	for (i = 0; i < DMA_GRP_NUM_REGS; i++) {
		if (!ipu_dma_grp_reg_names[i])
			continue;

		ipu_dma_channel_debug_create_register_file(pb, channel,
				&channel->debug_dma_registers[i],
				ipu_dma_grp_reg_names[i],
				i * IPU_REG_WIDTH_BYTES);
	}
}

void ipu_dma_channel_debug_remove(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	debugfs_remove_recursive(channel->debug_dir);
}

static int ipu_dma_debug_enable_set(void *data, u64 val)
{
	struct paintbox_data *pb = data;
	uint32_t reg_val;
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

	reg_val = ipu_readq(pb->dev, IPU_CSR_DMA_TOP_OFFSET +
				DMA_STAT_CTRL);
	if (val != 0)
		reg_val |= DMA_STAT_CTRL_ENABLE_MASK;
	else
		reg_val &= ~DMA_STAT_CTRL_ENABLE_MASK;

	ipu_writeq(pb->dev, reg_val, IPU_CSR_DMA_TOP_OFFSET +
			DMA_STAT_CTRL);

	pb->dma.debug_enabled = !!val;

	ret = ipu_jqs_put(pb);

	mutex_unlock(&pb->lock);

	return ret;
}

static int ipu_dma_debug_enable_get(void *data, u64 *val)
{
	struct paintbox_data *pb = data;
	uint32_t reg_val;
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

	reg_val = ipu_readq(pb->dev, IPU_CSR_DMA_TOP_OFFSET +
					DMA_STAT_CTRL);
	pb->dma.debug_enabled = !!(reg_val & DMA_STAT_CTRL_ENABLE_MASK);
	*val = pb->dma.debug_enabled;

	ret = ipu_jqs_put(pb);

	mutex_unlock(&pb->lock);

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(ipu_dma_debug_enable_fops, ipu_dma_debug_enable_get,
		ipu_dma_debug_enable_set, "%llx\n");

static int ipu_dma_debug_read_registers(struct seq_file *s, void *data)
{
	struct paintbox_data *pb = dev_get_drvdata(s->private);
	uint64_t dma_top_registers[DMA_TOP_NUM_REGS];
	uint64_t dma_grp_chan_mode_r0;
	uint64_t dma_grp_sel_val;
	uint64_t reg_offset;
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

	dma_grp_sel_val = ipu_readl(pb->dev, IPU_CSR_DMA_GRP_OFFSET + DMA_SEL);
	dma_grp_chan_mode_r0 = ipu_readq(pb->dev,
			IPU_CSR_DMA_GRP_OFFSET + DMA_CHAN_MODE_RO);

	for (reg_offset = DMA_CTRL; reg_offset < DMA_TOP_BLOCK_END;
			reg_offset += IPU_REG_WIDTH_BYTES) {
		if (!ipu_dma_top_reg_names[REG_INDEX64(reg_offset)])
			continue;

		dma_top_registers[REG_INDEX64(reg_offset)] =
				ipu_readq(pb->dev, IPU_CSR_DMA_TOP_OFFSET +
				reg_offset);
	}

	ipu_dma_debug_dump_dma_sel(s, dma_grp_sel_val);

	ipu_dma_debug_dump_dma_ctrl(s,
			dma_top_registers[REG_INDEX64(DMA_CTRL)]);

	for (reg_offset = DMA_IRQ_ISR; reg_offset <= DMA_PMON_CNT_3_STS;
			reg_offset += IPU_REG_WIDTH_BYTES) {
		if (!ipu_dma_top_reg_names[REG_INDEX64(reg_offset)])
			continue;

		ipu_dma_dump_top_reg(s, reg_offset,
				dma_top_registers[REG_INDEX64(reg_offset)]);
	}

	ipu_dma_debug_dump_dma_stat_regs(s,
			dma_top_registers[REG_INDEX64(DMA_STAT_CTRL)],
			dma_top_registers[REG_INDEX64(DMA_STAT_STATE)],
			dma_top_registers[REG_INDEX64(DMA_STAT_PTR)],
			dma_top_registers[REG_INDEX64(DMA_STAT_ADDR)],
			dma_grp_chan_mode_r0);

	ipu_dma_debug_dump_ssp_status(s,
			dma_top_registers[REG_INDEX64(SSP_STATUS)]);

	ret = ipu_jqs_put(pb);

	mutex_unlock(&pb->lock);

	return ret;
}

static int ipu_dma_debug_register_set(void *data, u64 val)
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

static int ipu_dma_debug_register_get(void *data, u64 *val)
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

DEFINE_SIMPLE_ATTRIBUTE(ipu_dma_debug_register_fops,
		ipu_dma_debug_register_get,	ipu_dma_debug_register_set,
		"%llx\n");

static void ipu_dma_debug_create_register_file(struct paintbox_data *pb,
		struct ipu_debug_register *reg, const char *name,
		unsigned int offset)
{
	reg->offset = IPU_CSR_DMA_TOP_OFFSET + offset;
	reg->pb = pb;

	reg->dentry = debugfs_create_file(name, 0640, pb->dma.debug_dir, reg,
			&ipu_dma_debug_register_fops);
	if (WARN_ON(IS_ERR(reg->dentry)))
		reg->dentry = NULL;
}

void ipu_dma_debug_init(struct paintbox_data *pb)
{
	unsigned int i;

	pb->dma.debug_dir = debugfs_create_dir("dma", pb->debug_root);
	if (WARN_ON(IS_ERR_OR_NULL(pb->dma.debug_dir)))
		return;

	pb->dma.debug_enable_dentry = debugfs_create_file("debug_en", 0640,
			pb->dma.debug_dir, pb, &ipu_dma_debug_enable_fops);
	if (WARN_ON(IS_ERR_OR_NULL(pb->dma.debug_enable_dentry)))
		return;

	pb->dma.debug_reg_dump = debugfs_create_devm_seqfile(pb->dev, "regs",
			pb->dma.debug_dir, ipu_dma_debug_read_registers);
	if (WARN_ON(IS_ERR_OR_NULL(pb->dma.debug_reg_dump)))
		return;

	for (i = 0; i < DMA_TOP_NUM_REGS; i++) {
		if (!ipu_dma_top_reg_names[i])
			continue;

		ipu_dma_debug_create_register_file(pb,
				&pb->dma.debug_registers[i],
				ipu_dma_top_reg_names[i],
				i * IPU_REG_WIDTH_BYTES);
	}
}

void ipu_dma_debug_remove(struct paintbox_data *pb)
{
	debugfs_remove_recursive(pb->dma.debug_dir);
}
