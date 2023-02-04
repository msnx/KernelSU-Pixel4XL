/*
 * Airbrush PCIe function driver
 *
 * Copyright (C) 2018 Samsung Electronics Co. Ltd.
 *              http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ABC_PCIE_SFR_H
#define __ABC_PCIE_SFR_H

#define SYSREG_FSYS_OFFSET		0x30000

#define ML3_DEV

#ifdef ML3_DEV
#define SYSREG_FSYS_DBI_OVERRIDE	(SYSREG_FSYS_OFFSET + 0x354)
#else
#define SYSREG_FSYS_DBI_OVERRIDE	(SYSREG_FSYS_OFFSET + 0x358)
#endif
#define SYSREG_FSYS_INTERRUPT		(SYSREG_FSYS_OFFSET + 0x3f4)

#define	DBI_OVERRIDE_MASK		0x7
#define DBI_OVERRIDE_DMA		0x7
#define DBI_OVERRIDE_IATU		0x6

#define ABC_PCIE_DBI_BASE		0x800000
#define ABC_PCIE_SUB_CTRL_BASE		0x780000

#define PF0_ATU_CAP_IATU_REGION_CTRL_1_OFF_OUTBOUND	0x0000
#define PF0_ATU_CAP_IATU_REGION_CTRL_2_OFF_OUTBOUND	0x0004
#define PF0_ATU_CAP_IATU_LWR_BASE_ADDR_OFF_OUTBOUND	0x0008
#define PF0_ATU_CAP_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND	0x000c
#define PF0_ATU_CAP_IATU_LIMIT_ADDR_OFF_OUTBOUND	0x0010
#define PF0_ATU_CAP_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND	0x0014
#define PF0_ATU_CAP_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND 0x0018

#define PF0_ATU_CAP_IATU_REGION_CTRL_1_OFF_INBOUND      0x0100
#define PF0_ATU_CAP_IATU_REGION_CTRL_2_OFF_INBOUND      0x0104
#define PF0_ATU_CAP_IATU_LWR_BASE_ADDR_OFF_INBOUND      0x0108
#define PF0_ATU_CAP_IATU_UPPER_BASE_ADDR_OFF_INBOUND    0x010c
#define PF0_ATU_CAP_IATU_LIMIT_ADDR_OFF_INBOUND         0x0110
#define PF0_ATU_CAP_IATU_LWR_TARGET_ADDR_OFF_INBOUND    0x0114
#define PF0_ATU_CAP_IATU_UPPER_TARGET_ADDR_OFF_INBOUND  0x0118

#define IATU_REGION_OFFSET				0x200
#define IATU_ENABLE					(0x1 << 31)
/* Parameters for the waiting for iATU enabled routine */
#define IATU_ENABLE_DISABLE_RETRIES			(15)
#define IATU_WAIT_TIME_IN_MSEC				(3)

/* Inbound IATU lower target address must be aligned to a
 * CX_ATU_MIN_REGION_SIZE kB boundary (in address match mode); and to the
 * Bar size boundary (in BAR match mode) so that these bits are always '0'.
 * If the BAR is smaller than the iATU region size, then the iATU target
 * address must align to the iATU region size; otherwise it must align to the
 * BAR size.
 *
 * A write to this location is ignored by the PCIe controller.
 *   Field size depends on log2(CX_ATU_MIN_REGION_SIZE) in address match mode.
 *   Field size depends on log2(BAR_MASK+1) in BAR match mode.
 *
 * Currently CX_ATU_MIN_REGION_SIZE is 4KB. So lower 12bits are masked and
 * are write ignored.
 *
 * Please refer "1.47.15 IATU_LWR_TARGET_ADDR_OFF_INBOUND_i" section in
 * designware register description manual (v5.00a)
 */
#define IATU_LWR_TARGET_ADDR_INBOUND_MASK		(0xFFFFF000)

#define DMA_READ_DOORBELL				0x0030
#define DMA_READ_ENGINE					0x002C
#define DMA_READ_INTERRUPT_MASK				0x00a8
#define DMA_READ_CHANNEL_CONTROL_1			0x0300
#define DMA_READ_TRANSFER_SIZE				0x0308
#define DMA_READ_SAR_LOW				0x030C
#define DMA_READ_SAR_HIGH				0x0310
#define DMA_READ_DAR_LOW				0x0314
#define DMA_READ_DAR_HIGH				0x0318
#define DMA_LLP_LOW_OFF_RDCH				0x031C
#define DMA_LLP_HIGH_OFF_RDCH				0x0320

#define DMA_WRITE_DOORBELL				0x0010
#define DMA_WRITE_ENGINE				0x000C
#define DMA_WRITE_INTERRUPT_MASK			0x0054
#define DMA_WRITE_CHANNEL_CONTROL_1			0x0200
#define DMA_WRITE_TRANSFER_SIZE				0x0208
#define DMA_WRITE_SAR_LOW				0x020C
#define DMA_WRITE_SAR_HIGH				0x0210
#define DMA_WRITE_DAR_LOW				0x0214
#define DMA_WRITE_DAR_HIGH				0x0218
#define DMA_LLP_LOW_OFF_WRCH				0x021C
#define DMA_LLP_HIGH_OFF_WRCH				0x0220

#define DMA_MASK					0x0000
#define DMA_ENABLE					0x0001
#define DMA_READ_OFFSET					0x0200
#define DMA_WRITE_OFFSET				0x0200
#define DMA_UPPER_ADDRESS_MASK				0xFFFFFFFF
#define DMA_LOWER_ADDRESS_MASK				0xFFFFFFFF
#define DMA_SHIFT					0x1F

#define DMA_WRITE_INT_STATUS_OFF			0x4C
#define	DMA_WRITE_INT_MASK_OFF				0x54
#define DMA_WRITE_INT_CLEAR_OFF				0x58
#define DMA_WRITE_ERR_STATUS_OFF			0x5C
#define DMA_WRITE_DONE_IMWR_LOW_OFF			0x60
#define DMA_WRITE_DONE_IMWR_HIGH_OFF			0x64
#define DMA_WRITE_ABORT_IMWR_LOW_OFF			0x68
#define DMA_WRITE_ABORT_IMWR_HIGH_OFF			0x6C

#define DMA_READ_INT_STATUS_OFF				0xA0
#define	DMA_READ_INT_MASK_OFF				0xA8
#define DMA_READ_INT_CLEAR_OFF				0xAC
#define DMA_READ_ERR_STATUS_LOW_OFF			0xB4
#define DMA_READ_ERR_STATUS_HIGH_OFF			0xB8
#define DMA_READ_LINKED_LIST_ERR_EN_OFF			0xC4


#define RD_DONE_INT_STATUS				0xFF
#define RD_ABORT_INT_STATUS				(0xFF << 16)
#define RD_ABROT_INT_STATUS_SHIFT			16

#define L1SUB_CONTROL1_REG				0x168
#define LINK_CONTROL_LINK_STATUS_REG			0x80
#define PCIE_CAP_DEV_CTRL_STS2_REG			0x98
#define LINK_CONTROL2_LINK_STATUS2_REG			0xA0
#define PORT_LOGIC_GEN2_CTRL_OFF			0x80C
#define ACK_F_ASPM_CTRL_OFF		0x70C
#define ACK_F_ASPM_CTRL_OFF_L1_DELAY_POS		27
#define ACK_F_ASPM_CTRL_OFF_L1_DELAY_MASK		\
	(7 << ACK_F_ASPM_CTRL_OFF_L1_DELAY_POS)
#define ACK_F_ASPM_CTRL_OFF_N_L0S_ENTRY_POS		30
#define ACK_F_ASPM_CTRL_OFF_N_L0S_ENTRY_MASK		\
	(1 << ACK_F_ASPM_CTRL_OFF_N_L0S_ENTRY_POS)
#define PME_EN						0x44
#define CLOCK_REQ_EN					0x80024

#define CLEAR_ASPM					0xFFFFFFFC
#define ENABLE_ASPM					0x3
#define CLEAR_L1_SUBSTATES				0xFFFFFFF3
#define ENABLE_L1_SUBSTATES				0xC
#define LTR_ENABLE					(0x1 << 10)
#define ASPM_L0s_ENABLE					(0x1 << 0)
#define ASPM_L1_ENABLE					(0x1 << 1)
#define ASPM_L1_2_ENABLE				(0x1 << 2)
#define ASPM_L1_1_ENABLE				(0x1 << 3)
#define LINK_SPEED					0xF
#define TARGET_LINK_SPEED				0xF
#define DIRECTED_SPEED_CHANGE				(0x1 << 17)

#define MSI_CAP_OFF_10H_REG				0x60

#define MSI_CAP_MASK_31					(0x1 << 31)

#define ABC_READY_ENTR_L23				0x68
#define ENTR_L23_EN					0x1
#define ENTR_L23_MASK					0x1

#define GEN3_RELATED_OFF_REG				0x890
#define GEN3_ZRXDC_NONCOMPL_EN				0x1
#define GEN3_ZRXDC_NONCOMPL_MASK			0x1

#endif
