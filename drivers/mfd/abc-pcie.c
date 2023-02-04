/*
 * Airbrush PCIe function driver
 *
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *              http://www.samsung.com
 *
 * Author: Sayanta Pattanayak <sayanta.p@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#if IS_ENABLED(CONFIG_ARM64_DMA_USE_IOMMU)
#include <asm/dma-iommu.h>
#endif
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/file.h>
#include <linux/mfd/core.h>
#include <linux/mfd/syscon.h>
#include <linux/of_device.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/resource.h>
#include <linux/signal.h>
#include <linux/types.h>

#include <linux/airbrush-sm-ctrl.h>
#if IS_ENABLED(CONFIG_PCI_MSM)
#include <linux/msm_pcie.h>
#endif
#include <linux/mfd/abc-pcie.h>
#include <linux/mfd/abc-pcie-sfr.h>

#include "abc-pcie-private.h"

/* We must hold the fsys_reg_lock spinlock while accessing DBI registers and
 * SYSREG_FSYS_DBI_OVERRIDE. If SYSREG_FSYS_DBI_OVERRIDE is modified, we must
 * hold the spinlock until SYSREG_FSYS_DBI_OVERRIDE is set back to its original
 * value.
 */

#define UPPER(address) ((unsigned int)((address & 0xFFFFFFFF00000000) >> 32))
#define LOWER(address) ((unsigned int)(address & 0x00000000FFFFFFFF))

/* iATU region 0 is reserved for ABC configuration registers */
#define ABC_CONFIG_IATU_REGION 1

#define ABC_INVALID_DATA	0xFFFFFFFF

static struct abc_device *abc_dev;

enum l1_entry_delay_value {
	delay_1us,
	delay_2us,
	delay_4us,
	delay_8us,
	delay_16us,
	delay_32us,
	delay_64us
};

static void abc_pcie_enable_irqs(struct pci_dev *pdev);
static void abc_pcie_disable_irqs(struct pci_dev *pdev);

static const struct mfd_cell abc_mfd_of_nommu_devs[] = {
	{
		.name = "ab-pmu",
		.of_compatible = "abc,airbrush-pmu",
	},
	/* NOTE: ab-pmu must be defined before ab-clk in list */
	{
		.name = "ab-clk",
		.of_compatible = "abc,airbrush-clk",
	},
	{
		.name = "airbrush-tmu",
		.of_compatible = "abc,airbrush-tmu",
	},
	{
		.name = "airbrush-ddr",
		.of_compatible = "abc,airbrush-ddr",
	},
};

void __iomem *get_tpu_virt(void)
{
	return abc_dev->tpu_config;
}

bool abc_pcie_enumerated(void)
{
	if (!abc_dev || !abc_dev->pcie_config ||
			atomic_read(&abc_dev->link_state) !=
			(ABC_PCIE_LINK_ACTIVE | ABC_PCIE_SMMU_ATTACHED))
		return false;
	return true;
}

int abc_pcie_config_read(u32 offset, u32 len, u32 *data)
{
	void __iomem *base_offset;
	unsigned long flags;
	bool is_dbi_register = ((DBI_START) <= offset) &&
				(offset < (FSYS_NIC_GPV));
#ifdef CONFIG_MULTIPLE_BAR_MAP_FOR_ABC_SFR
	struct inb_region   ir;
	u32 region_offset;
#endif

	if (!abc_dev || !abc_dev->pcie_config || !abc_dev->fsys_config ||
			atomic_read(&abc_dev->link_state) !=
			(ABC_PCIE_LINK_ACTIVE | ABC_PCIE_SMMU_ATTACHED))
		return -ENOTCONN;

	if (((TPU_START) <= offset) && (offset < (IPU_START))) {
		offset -= TPU_START;
		return tpu_config_read(offset, len, data);
	} else if (((IPU_START) <= offset) && (offset < (MIF_START))) {
		offset -= IPU_START;
		return ipu_config_read(offset, len, data);
#ifndef CONFIG_MULTIPLE_BAR_MAP_FOR_ABC_SFR
	} else {
		base_offset = abc_dev->base_config + offset;
#else
	} else if (((FSYS_START) <= offset) && (offset < (DBI_START))) {
		base_offset = abc_dev->fsys_config +
				(offset - FSYS_START);
	} else if (is_dbi_register) {
		base_offset = abc_dev->pcie_config +
				(offset - DBI_START);
	} else if (((AON_AXI2APB) <= offset) && (offset < (AON_NIC_GPV))) {
		base_offset = abc_dev->aon_config +
				(offset - AON_AXI2APB);
	} else {
		region_offset = offset & ~(ABC_MISC_SFR_REGION_MASK);
		ir.target_pcie_address = ABC_SFR_BASE + region_offset;
		ir.u_target_pcie_address = 0x0;
		ir.mode =  MEM_MATCH;
		ir.region = 3;
		ir.memmode = 0;
		ir.bar = 3;
		set_inbound_iatu(ir);
		region_offset = offset & (ABC_MISC_SFR_REGION_MASK);
		base_offset = abc_dev->sfr_misc_config + region_offset;
#endif
	}

	if (is_dbi_register)
		/* Prevent concurrent access to SYSREG_FSYS's DBI_OVERRIDE. */
		spin_lock_irqsave(&abc_dev->fsys_reg_lock, flags);
	*data = readl(base_offset);
	if (is_dbi_register)
		spin_unlock_irqrestore(&abc_dev->fsys_reg_lock, flags);

	return 0;
}

int abc_pcie_config_write(u32 offset, u32 len, u32 data)
{
	void __iomem *base_offset;
	unsigned long flags;
	bool is_dbi_register = ((DBI_START) <= offset) &&
				(offset < (FSYS_NIC_GPV));
#ifdef CONFIG_MULTIPLE_BAR_MAP_FOR_ABC_SFR
	struct inb_region   ir;
	u32 region_offset;
#endif

	if (!abc_dev || !abc_dev->pcie_config || !abc_dev->fsys_config ||
			atomic_read(&abc_dev->link_state) !=
			(ABC_PCIE_LINK_ACTIVE | ABC_PCIE_SMMU_ATTACHED))
		return -ENOTCONN;

	if (((TPU_START) <= offset) && (offset < (IPU_START))) {
		offset -= TPU_START;
		return tpu_config_write(offset, len, data);
	} else if (((IPU_START) <= offset) && (offset < (MIF_START))) {
		offset -= IPU_START;
		return ipu_config_write(offset, len, data);
#ifndef CONFIG_MULTIPLE_BAR_MAP_FOR_ABC_SFR
	} else {
		base_offset = abc_dev->base_config + offset;
#else
	} else if (((FSYS_START) <= offset) && (offset < (DBI_START))) {
		base_offset = abc_dev->fsys_config +
				(offset - FSYS_START);
	} else if (is_dbi_register) {
		base_offset = abc_dev->pcie_config +
				(offset - DBI_START);
	} else if (((AON_AXI2APB) <= offset) && (offset < (AON_NIC_GPV))) {
		base_offset = abc_dev->aon_config +
				(offset - AON_AXI2APB);
	} else {
		region_offset = offset & ~(ABC_MISC_SFR_REGION_MASK);
		ir.target_pcie_address = ABC_SFR_BASE + region_offset;
		ir.u_target_pcie_address = 0x0;
		ir.mode =  MEM_MATCH;
		ir.region = 3;
		ir.memmode = 0;
		ir.bar = 3;
		set_inbound_iatu(ir);
		region_offset = offset & (ABC_MISC_SFR_REGION_MASK);
		base_offset = abc_dev->sfr_misc_config + region_offset;
#endif
	}

	__iowmb();
	if (is_dbi_register)
		/* Prevent concurrent access to SYSREG_FSYS's DBI_OVERRIDE. */
		spin_lock_irqsave(&abc_dev->fsys_reg_lock, flags);
	writel_relaxed(data, base_offset);
	if (is_dbi_register)
		spin_unlock_irqrestore(&abc_dev->fsys_reg_lock, flags);

	return 0;
}

int aon_config_read(u32 offset, u32 len, u32 *data)
{
	void __iomem *base_offset;

	if (!abc_dev || !abc_dev->aon_config ||
			atomic_read(&abc_dev->link_state) !=
			(ABC_PCIE_LINK_ACTIVE | ABC_PCIE_SMMU_ATTACHED))
		return -ENOTCONN;

	base_offset = abc_dev->aon_config + offset;
	*data = readl(base_offset);
	return 0;
}

int aon_config_write(u32 offset, u32 len, u32 data)
{
	void __iomem *base_offset;

	if (!abc_dev || !abc_dev->aon_config ||
			atomic_read(&abc_dev->link_state) !=
			(ABC_PCIE_LINK_ACTIVE | ABC_PCIE_SMMU_ATTACHED))
		return -ENOTCONN;

	base_offset = abc_dev->aon_config + offset;
	__iowmb();
	writel_relaxed(data, base_offset);
	return 0;
}

int ipu_config_read(u32 offset, u32 len, u32 *data)
{
	void __iomem *base_offset;

	if (!abc_dev || !abc_dev->ipu_config ||
			atomic_read(&abc_dev->link_state) !=
			(ABC_PCIE_LINK_ACTIVE | ABC_PCIE_SMMU_ATTACHED))
		return -ENOTCONN;

	base_offset = abc_dev->ipu_config + offset;
	*data = readl(base_offset);
	return 0;
}

int ipu_config_write(u32 offset, u32 len, u32 data)
{
	void __iomem *base_offset;

	if (!abc_dev || !abc_dev->ipu_config ||
			atomic_read(&abc_dev->link_state) !=
			(ABC_PCIE_LINK_ACTIVE | ABC_PCIE_SMMU_ATTACHED))
		return -ENOTCONN;

	base_offset = abc_dev->ipu_config + offset;
	__iowmb();
	writel_relaxed(data, base_offset);
	return 0;
}

int tpu_config_read(u32 offset, u32 len, u32 *data)
{
	void __iomem *base_offset;

	if (!abc_dev || !abc_dev->tpu_config ||
			atomic_read(&abc_dev->link_state) !=
			(ABC_PCIE_LINK_ACTIVE | ABC_PCIE_SMMU_ATTACHED))
		return -ENOTCONN;

	base_offset = abc_dev->tpu_config + offset;
	*data = readl(base_offset);
	return 0;
}

int tpu_config_write(u32 offset, u32 len, u32 data)
{
	void __iomem *base_offset;

	if (!abc_dev || !abc_dev->tpu_config ||
			atomic_read(&abc_dev->link_state) !=
			(ABC_PCIE_LINK_ACTIVE | ABC_PCIE_SMMU_ATTACHED))
		return -ENOTCONN;

	base_offset = abc_dev->tpu_config + offset;
	__iowmb();
	writel_relaxed(data, base_offset);
	return 0;
}

/* DEBUG ONLY: Reconfiguring iATU for each transaction is very costly
 * memory_config_read can be used to read from SRAM and DRAM of ABC.
 */
int memory_config_read(u32 offset, u32 len, u32 *data)
{
#if IS_ENABLED(CONFIG_MULTIPLE_BAR_MAP_FOR_ABC_SFR)
	void __iomem *base_offset;
	struct inb_region   ir;
	u32 region_offset;
	int ret = 0;

	if (!abc_dev || !abc_dev->memory_config ||
			atomic_read(&abc_dev->link_state) !=
			(ABC_PCIE_LINK_ACTIVE | ABC_PCIE_SMMU_ATTACHED))
		return -ENOTCONN;

	region_offset = offset & ~(ABC_MEMORY_REGION_MASK);

	/* Check whether mapping is required or not */
	if (abc_dev->memory_map != region_offset) {
		ir.target_pcie_address = region_offset;
		ir.u_target_pcie_address = 0x0;
		ir.mode =  MEM_MATCH;
		ir.region = 4;
		ir.bar = 3;
		ir.memmode = 0;
		ret = set_inbound_iatu(ir);
		if (ret < 0) {
			pr_err("%s: set_inbound_iatu failed.\n", __func__);
			return ret;
		}
		abc_dev->memory_map = region_offset;
	}

	region_offset = offset & ABC_MEMORY_REGION_MASK;
	base_offset = abc_dev->memory_config + region_offset;
	*data = readl(base_offset);
#else
	int ret;
	struct device *dev = abc_dev->dev;
	struct bar_mapping mapping;
	int bar = BAR_2;

	ret = abc_pcie_map_bar_region(dev, dev, bar, len, (uint64_t)offset,
			&mapping);
	if (ret < 0) {
		pr_err("%s: unable to map for bar region, ret=%d\n", __func__,
				ret);
		return ret;
	}

	memcpy_fromio((void *)data, mapping.bar_vaddr, len);

	ret = abc_pcie_unmap_bar_region(dev, dev, &mapping);
	if (ret < 0) {
		pr_err("%s: unable to unmap for bar region, ret=%d\n",
				__func__, ret);
		return ret;
	}
#endif
	return 0;
}

/* DEBUG ONLY: Reconfiguring iATU for each transaction is very costly
 * memory_config_write can be used to read from SRAM and DRAM of ABC.
 */
int memory_config_write(u32 offset, u32 len, u32 data)
{
#if IS_ENABLED(CONFIG_MULTIPLE_BAR_MAP_FOR_ABC_SFR)
	void __iomem *base_offset;
	struct inb_region   ir;
	u32 region_offset;
	int ret = 0;

	if (!abc_dev || !abc_dev->memory_config ||
			atomic_read(&abc_dev->link_state) !=
			(ABC_PCIE_LINK_ACTIVE | ABC_PCIE_SMMU_ATTACHED))
		return -ENOTCONN;

	region_offset = offset & ~(ABC_MEMORY_REGION_MASK);

	/* Check whether mapping is required or not */
	if (abc_dev->memory_map != region_offset) {
		ir.target_pcie_address = region_offset;
		ir.u_target_pcie_address = 0x0;
		ir.mode =  MEM_MATCH;
		ir.region = 4;
		ir.bar = 3;
		ir.memmode = 0;
		ret = set_inbound_iatu(ir);
		if (ret < 0) {
			pr_err("%s: set_inbound_iatu failed.\n", __func__);
			return ret;
		}
		abc_dev->memory_map = region_offset;
	}

	region_offset = offset & ABC_MEMORY_REGION_MASK;
	base_offset = abc_dev->memory_config + region_offset;
	__iowmb();
	writel_relaxed(data, base_offset);
#else
	int ret;
	struct device *dev = abc_dev->dev;
	struct bar_mapping mapping;
	int bar = BAR_2;

	ret = abc_pcie_map_bar_region(dev, dev /* owner */, bar, len,
			(uint64_t)offset, &mapping);
	if (ret < 0) {
		pr_err("%s: unable to map for bar region, ret=%d\n", __func__,
				ret);
		return ret;
	}

	memcpy_toio(mapping.bar_vaddr, (void *)(&data), len);

	ret = abc_pcie_unmap_bar_region(dev, dev /* owner */, &mapping);
	if (ret < 0) {
		pr_err("%s: unable to unmap for bar region, ret=%d\n",
				__func__, ret);
		return ret;
	}
#endif
	return 0;
}

#if IS_ENABLED(CONFIG_AIRBRUSH_SM)
u32 abc_pcie_get_linkspeed(void)
{
	u32 link_status;

	abc_pcie_config_read(ABC_PCIE_DBI_BASE + LINK_CONTROL_LINK_STATUS_REG,
				0x0, &link_status);
	return (link_status >> 16) & LINK_SPEED;
}

u32 abc_pcie_get_linkstate(void)
{
	u32 link_state;
	u32 l1_substate, val;

	abc_pcie_config_read(ABC_PCIE_DBI_BASE + LINK_CONTROL_LINK_STATUS_REG,
			 0x0, &link_state);
	abc_pcie_config_read(ABC_PCIE_DBI_BASE + L1SUB_CONTROL1_REG,
			 0x0, &l1_substate);
	if (link_state & ASPM_L1_ENABLE) {
		if (!(l1_substate & (ASPM_L1_2_ENABLE | ASPM_L1_1_ENABLE)))
			return ASPM_L10;
		val = readl_relaxed(abc_dev->fsys_config + CLOCK_REQ_EN) & 0x1;
		__iormb(val);
		if (!val)
			return ASPM_UNKNOWN;
		if (l1_substate & ASPM_L1_2_ENABLE)
			return ASPM_L12;
		if (l1_substate & ASPM_L1_1_ENABLE)
			return ASPM_L11;
	}
	if (link_state & ASPM_L0s_ENABLE)
		return ASPM_L0s;

	/* If ASPM is disabled only L0 */
	return NOASPM;
}

void abc_pcie_set_linkspeed(u32 target_linkspeed)
{
	u32 link_status2;
	u32 val;
	u32 current_linkspeed;

	if (target_linkspeed == 0)
		return;

	current_linkspeed = abc_pcie_get_linkspeed();
	if (target_linkspeed == current_linkspeed)
		return;

	/* Changing the target link speed in link_control2_link_status2 */
	abc_pcie_config_read(ABC_PCIE_DBI_BASE + LINK_CONTROL2_LINK_STATUS2_REG,
			 0x0, &link_status2);
	link_status2 &= ~(TARGET_LINK_SPEED);
	link_status2 |= target_linkspeed;
	abc_pcie_config_write(
		ABC_PCIE_DBI_BASE + LINK_CONTROL2_LINK_STATUS2_REG,
		0x0, link_status2);

	/* Asserting the directed speed change */
	abc_pcie_config_read(ABC_PCIE_DBI_BASE + PORT_LOGIC_GEN2_CTRL_OFF,
			 0x0, &val);
	val |= DIRECTED_SPEED_CHANGE;
	abc_pcie_config_write(ABC_PCIE_DBI_BASE + PORT_LOGIC_GEN2_CTRL_OFF,
			  0x0, val);
}

/* Caller must hold abc_dev->fsys_reg_lock */
static void abc_l1ss_timeout_ctrl(bool enable)
{
	u32 val;

	val = readl(abc_dev->pcie_config
			+ GEN3_RELATED_OFF_REG);

	if (enable)
		val |= GEN3_ZRXDC_NONCOMPL_EN;
	else
		val &= ~(GEN3_ZRXDC_NONCOMPL_MASK);

	__iowmb();
	writel_relaxed(val,
			abc_dev->pcie_config + GEN3_RELATED_OFF_REG);

}

static void abc_l1_entry_ctrl(enum l1_entry_delay_value delay,
	bool use_l0s_for_entry)
{
	u32 val;
	unsigned long flags;

	/* Prevent concurrent access to SYSREG_FSYS's DBI_OVERRIDE. */
	spin_lock_irqsave(&abc_dev->fsys_reg_lock, flags);

	val = readl(abc_dev->pcie_config
				+ ACK_F_ASPM_CTRL_OFF);
	val &= ~ACK_F_ASPM_CTRL_OFF_L1_DELAY_MASK;
	val |= ((delay << ACK_F_ASPM_CTRL_OFF_L1_DELAY_POS)
		& ACK_F_ASPM_CTRL_OFF_L1_DELAY_MASK);

	if (use_l0s_for_entry)
		val &= (~ACK_F_ASPM_CTRL_OFF_N_L0S_ENTRY_MASK);
	else
		val |= ACK_F_ASPM_CTRL_OFF_N_L0S_ENTRY_MASK;

	__iowmb();
	writel_relaxed(val,
		abc_dev->pcie_config + ACK_F_ASPM_CTRL_OFF);

	spin_unlock_irqrestore(&abc_dev->fsys_reg_lock, flags);
}

void abc_set_l1_entry_delay(int delay)
{
	delay = min(delay, delay_64us);
	delay = max(delay_1us, delay);
	abc_l1_entry_ctrl(delay, false);
}

static void abc_set_pcie_pm_ctrl(struct abc_pcie_pm_ctrl *pmctrl)
{
	u32 aspm_l11_l12;
	u32 l1_l0s_enable;
	u32 val;
	unsigned long flags;

	/* Prevent concurrent access to SYSREG_FSYS's DBI_OVERRIDE. */
	spin_lock_irqsave(&abc_dev->fsys_reg_lock, flags);

	if (abc_dev->l1sub_reg_cache == ABC_PCIE_CACHE_UNKNOWN)
		abc_dev->l1sub_reg_cache =
			readl_relaxed(abc_dev->pcie_config
			+ L1SUB_CONTROL1_REG);
	if (abc_dev->link_status_reg_cache == ABC_PCIE_CACHE_UNKNOWN)
		abc_dev->link_status_reg_cache =
			readl_relaxed(abc_dev->pcie_config
			+ LINK_CONTROL_LINK_STATUS_REG);

	aspm_l11_l12 = abc_dev->l1sub_reg_cache;
	l1_l0s_enable = abc_dev->link_status_reg_cache;

	aspm_l11_l12 &= ~(0xC);
	aspm_l11_l12 |= (pmctrl->aspm_L11 << 3);
	aspm_l11_l12 |= (pmctrl->aspm_L12 << 2);

	l1_l0s_enable &= ~(0x3);
	l1_l0s_enable |= (pmctrl->l0s_en << 0);
	l1_l0s_enable |= (pmctrl->l1_en << 1);

	__iowmb();

	/* Enabling ASPM L11 & L12, PCI_PM L11 & L12 */
	if (aspm_l11_l12 != abc_dev->l1sub_reg_cache)
		writel_relaxed(aspm_l11_l12,
				abc_dev->pcie_config + L1SUB_CONTROL1_REG);

	/* Enabling L1 or L0s or both */
	if (l1_l0s_enable != abc_dev->link_status_reg_cache)
		writel_relaxed(l1_l0s_enable, abc_dev->pcie_config
				+ LINK_CONTROL_LINK_STATUS_REG);

	if (pmctrl->l1_en)
		abc_l1ss_timeout_ctrl(false);

	spin_unlock_irqrestore(&abc_dev->fsys_reg_lock, flags);

#if IS_ENABLED(CONFIG_PCI_MSM)
	/* set rc l1ss state to match ep's */
	if (aspm_l11_l12 != abc_dev->l1sub_reg_cache)
		msm_pcie_set_l1ss_state(abc_dev->pdev,
				pmctrl->aspm_L12 ? MSM_PCIE_PM_L1SS_L12 :
				pmctrl->aspm_L11 ? MSM_PCIE_PM_L1SS_L11 :
				MSM_PCIE_PM_L1SS_DISABLE);
#endif

	/* Prevent concurrent access to SYSREG_FSYS's DBI_OVERRIDE. */
	spin_lock_irqsave(&abc_dev->fsys_reg_lock, flags);

	abc_dev->l1sub_reg_cache = aspm_l11_l12;
	abc_dev->link_status_reg_cache = l1_l0s_enable;

	/* LTR Enable */
	if (pmctrl->aspm_L12 && !abc_dev->ltr_enable) {
		val = readl(
			abc_dev->pcie_config + PCIE_CAP_DEV_CTRL_STS2_REG);
		val |= LTR_ENABLE;
		__iowmb();
		writel_relaxed(val,
			abc_dev->pcie_config + PCIE_CAP_DEV_CTRL_STS2_REG);
		abc_dev->ltr_enable = true;
	}


	if (pmctrl->l1_en && !abc_dev->clk_req_enable) {
		/* Clock Request Enable*/
		writel_relaxed(0x1, abc_dev->fsys_config + CLOCK_REQ_EN);
		spin_unlock_irqrestore(&abc_dev->fsys_reg_lock, flags);
		/* 32us delay was selected after testing */
		abc_l1_entry_ctrl(delay_32us, pmctrl->l0s_en);
		abc_dev->clk_req_enable = true;
	} else {
		spin_unlock_irqrestore(&abc_dev->fsys_reg_lock, flags);
	}

}

void abc_pcie_set_linkstate(u32 target_linkstate)
{
	struct abc_pcie_pm_ctrl smctrl;
	u32 current_linkstate;
	u32 val;

	smctrl.l0s_en = ABC_PCIE_PM_DISABLE;
	smctrl.l1_en = ABC_PCIE_PM_DISABLE;
	smctrl.aspm_L11 = ABC_PCIE_PM_DISABLE;
	smctrl.aspm_L12 = ABC_PCIE_PM_DISABLE;

	current_linkstate = abc_dev->link_config_cache;
	if (target_linkstate == current_linkstate)
		return;


	switch (target_linkstate) {
	case PM_L2:
		abc_pcie_config_read(ABC_PCIE_SUB_CTRL_BASE +
				ABC_READY_ENTR_L23,
				0x0, &val);
		val |= ENTR_L23_EN;
		abc_pcie_config_write(ABC_PCIE_SUB_CTRL_BASE +
				      ABC_READY_ENTR_L23,
				      0x4, val);
		abc_dev->ltr_enable = false;
		abc_dev->clk_req_enable = false;
		abc_dev->l1sub_reg_cache = ABC_PCIE_CACHE_UNKNOWN;
		abc_dev->link_status_reg_cache = ABC_PCIE_CACHE_UNKNOWN;
		abc_dev->link_config_cache = target_linkstate;
		return;
	case ASPM_L12:
		smctrl.aspm_L12 = ABC_PCIE_PM_ENABLE;
		/* FALLTHROUGH */
	case ASPM_L11:
		smctrl.aspm_L11 = ABC_PCIE_PM_ENABLE;
		/* FALLTHROUGH */
	case ASPM_L10:
		smctrl.l1_en = ABC_PCIE_PM_ENABLE;
		break;
	case ASPM_L0s:
		smctrl.l0s_en = ABC_PCIE_PM_ENABLE;
		break;
	case NOASPM:
		break;
	default:
		dev_err(abc_dev->dev, "Invalid pcie linkstate requested\n");
		WARN_ON(1);
		return;
	}

	abc_set_pcie_pm_ctrl(&smctrl);
	abc_dev->link_config_cache = target_linkstate;
}

u32 string_to_integer(char *string)
{
	if (!strcmp(string, "L0s"))
		return ASPM_L0s;
	else if (!strcmp(string, "L0"))
		return NOASPM;
	else if (!strcmp(string, "L1.1"))
		return ASPM_L11;
	else if (!strcmp(string, "L1.2"))
		return ASPM_L12;
	else if (!strcmp(string, "L1"))
		return ASPM_L10;
	else if (!strcmp(string, "L3") || !strcmp(string, "L2"))
		return PM_L2;
	else
		return NOASPM;
}

int abc_pcie_state_manager(const struct block_property *current_property,
		const struct block_property *target_property,
		enum block_state block_substate_id, void *data)
{
	u32 target_linkstate =
		string_to_integer(target_property->substate_name);

	if (!target_property)
		return -EINVAL;

	/* change to the requested speed and state */
	ab_sm_start_ts(AB_SM_TS_PCIE_SET_LINKSTATE);
	abc_pcie_set_linkstate(target_linkstate);
	ab_sm_record_ts(AB_SM_TS_PCIE_SET_LINKSTATE);

	return 0;
}
#endif

int abc_set_aspm_state(bool state)
{
	u32 aspm_sts;
	u32 aspm_l1_sub_states;
	unsigned long flags;

	/* Prevent concurrent access to SYSREG_FSYS's DBI_OVERRIDE. */
	spin_lock_irqsave(&abc_dev->fsys_reg_lock, flags);

	if (state) {
		/* Enabling ASPM L0s, L1 support [1:0] = 0x3 */
		aspm_sts = readl(abc_dev->pcie_config +
				 LINK_CONTROL_LINK_STATUS_REG);
		aspm_sts &= CLEAR_ASPM;
		aspm_sts |= ENABLE_ASPM;
		__iowmb();
		writel_relaxed(aspm_sts, abc_dev->pcie_config +
		       LINK_CONTROL_LINK_STATUS_REG);

		/* Enabling ASPM L1 substates L1.1 [3:3] = 0x1, [2:2] = 0x1 */
		aspm_l1_sub_states = readl(abc_dev->pcie_config +
					   L1SUB_CONTROL1_REG);
		aspm_l1_sub_states &= CLEAR_L1_SUBSTATES;
		aspm_l1_sub_states |= ENABLE_L1_SUBSTATES;
		__iowmb();
		writel_relaxed(aspm_l1_sub_states, abc_dev->pcie_config +
		       L1SUB_CONTROL1_REG);
	} else {
		/* Disabling ASPM L0s, L1 support [1:0] = 0x0 */
		aspm_sts = readl(abc_dev->pcie_config +
				 LINK_CONTROL_LINK_STATUS_REG);
		aspm_sts &= CLEAR_ASPM;
		__iowmb();
		writel_relaxed(aspm_sts, abc_dev->pcie_config +
		       LINK_CONTROL_LINK_STATUS_REG);

		/* Disabling ASPM L1 substates L1.1 [3:3] = 0x0, [2:2] = 0x0 */
		aspm_l1_sub_states = readl(abc_dev->pcie_config +
					   L1SUB_CONTROL1_REG);
		aspm_l1_sub_states &= CLEAR_L1_SUBSTATES;
		__iowmb();
		writel_relaxed(aspm_l1_sub_states, abc_dev->pcie_config +
		       L1SUB_CONTROL1_REG);
	}

	spin_unlock_irqrestore(&abc_dev->fsys_reg_lock, flags);

	return 0;
}

int set_inbound_iatu(struct inb_region ir)
{
	unsigned long flags;
	u32 val;
	u32 set_val;
	u32 config = ir.memmode;
	int bar = ir.bar;
	u32 iatu_offset = (ir.region * IATU_REGION_OFFSET);
	u32 ctrl_2_set_val;
	u32 rdata;
	int retries, ret = 0;

	if (bar > BAR_4) {
		pr_err("Exceeding BAR number\n");
		return -EIO;
	}

	spin_lock_irqsave(&abc_dev->fsys_reg_lock, flags);

	/* Set SYSREG_FSYS DBI_OVERRIDE for iATU access mode */
	val = readl(
		abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);

	set_val = val & ~(DBI_OVERRIDE_MASK);
	set_val |= DBI_OVERRIDE_IATU;

	__iowmb();
	writel_relaxed(set_val,
		abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);

	/* enable iATU region by setting ctrl to 0x8000000 */
	ctrl_2_set_val = IATU_CTRL_2_REGION_EN_MASK <<
		IATU_CTRL_2_REGION_EN_SHIFT;

	__iowmb();

	if (ir.mode == BAR_MATCH) {
		/* Set MATCH_MODE to BAR_match mode: 1 */
		ctrl_2_set_val |= IATU_CTRL_2_MATCH_MODE_MASK <<
			IATU_CTRL_2_MATCH_MODE_SHIFT;
		/* Set BAR_NUM field */
		ctrl_2_set_val |= (bar & IATU_CTRL_2_BAR_NUM_MASK) <<
			IATU_CTRL_2_BAR_NUM_SHIFT;
	} else if (ir.mode == MEM_MATCH) {
		/* Lower Base address */
		writel_relaxed(ir.base_address,
				(abc_dev->pcie_config + iatu_offset +
				 PF0_ATU_CAP_IATU_LWR_BASE_ADDR_OFF_INBOUND));

		/* Upper Base address */
		writel_relaxed(ir.u_base_address,
				(abc_dev->pcie_config + iatu_offset +
				 PF0_ATU_CAP_IATU_UPPER_BASE_ADDR_OFF_INBOUND));

		/* Limit */
		writel_relaxed(ir.limit_address,
				(abc_dev->pcie_config + iatu_offset +
				 PF0_ATU_CAP_IATU_LIMIT_ADDR_OFF_INBOUND));
	}

	/* Lower Target address */
	writel_relaxed(ir.target_pcie_address,
		(abc_dev->pcie_config + iatu_offset +
		 PF0_ATU_CAP_IATU_LWR_TARGET_ADDR_OFF_INBOUND));

	/* Upper Target address */
	writel_relaxed(ir.u_target_pcie_address,
		(abc_dev->pcie_config + iatu_offset +
		 PF0_ATU_CAP_IATU_UPPER_TARGET_ADDR_OFF_INBOUND));

	/* Configuring Region control */
	writel_relaxed(config,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_REGION_CTRL_1_OFF_INBOUND));

	__iowmb();
	/* Enable region */
	writel_relaxed(ctrl_2_set_val,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_REGION_CTRL_2_OFF_INBOUND));

	/* Make sure ATU enable takes effect before any subsequent
	 * transactions. Currently the exact time is not specified in the
	 * user manual. So, following the synopsys designware driver.
	 */
	for (retries = 0; retries < IATU_ENABLE_DISABLE_RETRIES; retries++) {
		rdata = readl(abc_dev->pcie_config + iatu_offset +
				PF0_ATU_CAP_IATU_REGION_CTRL_2_OFF_INBOUND);

		if (rdata & IATU_ENABLE)
			break;
		mdelay(IATU_WAIT_TIME_IN_MSEC);
	}

	if (!(rdata & IATU_ENABLE)) {
		pr_err("%s: IATU enable timedout!!\n", __func__);
		ret = -ETIMEDOUT;
	}

	__iowmb();
	writel_relaxed(val,
			abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);

	spin_unlock_irqrestore(&abc_dev->fsys_reg_lock, flags);

	return ret;
}

static int disable_inbound_iatu_region(u32 region)
{
	unsigned long flags;
	u32 val;
	u32 set_val;
	u32 iatu_offset = (region * IATU_REGION_OFFSET);
	u32 rdata;
	int retries, ret = 0;

	if (region >= NUM_IATU_REGIONS) {
		pr_err("%s: Invalid iATU region: %d\n", __func__, region);
		return -EINVAL;
	}

	spin_lock_irqsave(&abc_dev->fsys_reg_lock, flags);

	/* Set SYSREG_FSYS DBI_OVERRIDE for iATU access mode */
	val = readl_relaxed(
			abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);

	set_val = val & ~(DBI_OVERRIDE_MASK);
	set_val |= DBI_OVERRIDE_IATU;

	__iowmb();
	writel_relaxed(set_val,
		abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);

	__iowmb();
	/* Enable region */
	writel_relaxed(0x0, abc_dev->pcie_config + iatu_offset +
			PF0_ATU_CAP_IATU_REGION_CTRL_2_OFF_INBOUND);

	/* Make sure ATU disable takes effect before any subsequent
	 * transactions. Currently the exact time is not specified in the
	 * user manual. So, following the synopsys designware driver.
	 */
	for (retries = 0; retries < IATU_ENABLE_DISABLE_RETRIES; retries++) {
		rdata = readl_relaxed(abc_dev->pcie_config + iatu_offset +
				PF0_ATU_CAP_IATU_REGION_CTRL_2_OFF_INBOUND);

		if (!(rdata & IATU_ENABLE))
			break;
		mdelay(IATU_WAIT_TIME_IN_MSEC);
	}

	if (rdata & IATU_ENABLE) {
		pr_err("%s: IATU disable timedout!!\n", __func__);
		ret = -ETIMEDOUT;
	}

	__iowmb();
	writel_relaxed(val,
			abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);

	spin_unlock_irqrestore(&abc_dev->fsys_reg_lock, flags);

	return ret;
}

int set_outbound_iatu(struct outb_region outreg)
{
	unsigned long flags;
	u32 val;
	u32 set_val;

	u32 iatu_offset = (outreg.region * IATU_REGION_OFFSET);

	u32 config = outreg.memmode;

	spin_lock_irqsave(&abc_dev->fsys_reg_lock, flags);

	/* Set SYSREG_FSYS DBI_OVERRIDE for iATU access mode */
	val = readl(
		abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);
	set_val = val & ~(DBI_OVERRIDE_MASK);
	set_val |= DBI_OVERRIDE_IATU;

	__iowmb();
	writel_relaxed(set_val,
		abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);

	/* Writing the lower source address */
	writel_relaxed(outreg.base_address,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_LWR_BASE_ADDR_OFF_OUTBOUND));

	/* Writing the upper source address */
	writel_relaxed(outreg.u_base_address,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND));

	/* Wrting the limit register */
	writel_relaxed(outreg.limit_address,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_LIMIT_ADDR_OFF_OUTBOUND));

	/* Writing the lower target address*/
	writel_relaxed(outreg.target_pcie_address,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND));

	/* Writing the upper target address*/
	writel_relaxed(outreg.u_target_pcie_address,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND));

	/* Configuring Region control */
	writel_relaxed(config,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_REGION_CTRL_1_OFF_OUTBOUND));

	/* Enable region */
	writel_relaxed(0x80000000,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_REGION_CTRL_2_OFF_OUTBOUND));

	writel_relaxed(val, abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);

	spin_unlock_irqrestore(&abc_dev->fsys_reg_lock, flags);

	return 0;
}

#if IS_ENABLED(CONFIG_ARM64_DMA_USE_IOMMU)
static int abc_pcie_smmu_attach(struct device *dev)
{
	struct abc_pcie_devdata *abc = dev_get_drvdata(dev);
	int ret;

	if (WARN_ON(!abc->iommu_mapping))
		return -EINVAL;

	ret = arm_iommu_attach_device(dev, abc->iommu_mapping);
	if (ret) {
		dev_err(dev, "%s: failed to attach device to IOMMU, ret%d\n",
				__func__, ret);
		return ret;
	}

	/*
	 * Sets both dma_mask and coherent_dma_mask to 64 bits.
	 * If coherent_dma_mask is left 32 bits, of_dma_configure() will
	 * override dma_mask to 32 bits during the probe of some mfd children
	 * devices if they have OF bindings.
	 */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret) {
		dev_err(dev, "unable to set 64-bit DMA mask (%d)\n", ret);
		return ret;
	}

	atomic_fetch_or(ABC_PCIE_SMMU_ATTACHED,
			&abc_dev->link_state);
	return 0;
}

static void abc_pcie_smmu_detach(struct device *dev)
{
	atomic_fetch_and(~ABC_PCIE_SMMU_ATTACH_STATE_MASK,
			&abc_dev->link_state);
	arm_iommu_detach_device(dev);
}

static int abc_pcie_smmu_setup(struct device *dev, struct abc_pcie_devdata *abc)
{
	int atomic_ctx = 1;
	int ret;
#if !IS_ENABLED(CONFIG_MFD_ABC_PCIE_SMMU_IOVA)
	int bypass_enable = 1;
#endif

	abc->iommu_mapping = arm_iommu_create_mapping(&platform_bus_type,
			ABC_PCIE_SMMU_BASE, ABC_PCIE_SMMU_SIZE);
	if (IS_ERR_OR_NULL(abc->iommu_mapping)) {
		ret = PTR_ERR(abc->iommu_mapping) ?: -ENODEV;
		abc->iommu_mapping = NULL;
		dev_err(dev, "%s: Failed to create IOMMU mapping (%d)\n",
				__func__, ret);
		return ret;
	}

	ret = iommu_domain_set_attr(abc->iommu_mapping->domain,
			DOMAIN_ATTR_ATOMIC, &atomic_ctx);
	if (ret < 0) {
		dev_err(dev, "%s: Set atomic attribute to SMMU failed (%d)\n",
				__func__, ret);
		goto release_mapping;
	}

#if !IS_ENABLED(CONFIG_MFD_ABC_PCIE_SMMU_IOVA)
	ret = iommu_domain_set_attr(abc->iommu_mapping->domain,
			DOMAIN_ATTR_S1_BYPASS, &bypass_enable);
	if (ret < 0) {
		dev_err(dev, "%s: Set bypass attribute to SMMU failed (%d)\n",
				__func__, ret);
		goto release_mapping;
	}
#endif

	ret = abc_pcie_smmu_attach(dev);
	if (ret)
		goto release_mapping;

	return 0;

release_mapping:
	arm_iommu_release_mapping(abc->iommu_mapping);
	abc->iommu_mapping = NULL;

	return ret;
}

static void abc_pcie_smmu_remove(struct device *dev,
		struct abc_pcie_devdata *abc)
{
	abc_pcie_smmu_detach(dev);
	arm_iommu_release_mapping(abc->iommu_mapping);
}
#else
static inline int abc_pcie_smmu_attach(struct device *dev)
{
	atomic_fetch_or(ABC_PCIE_SMMU_ATTACHED,
			&abc_dev->link_state);
	return 0;
}

static inline void abc_pcie_smmu_detach(struct device *dev)
{
	atomic_fetch_and(~ABC_PCIE_SMMU_ATTACH_STATE_MASK,
			&abc_dev->link_state);
}

static inline int abc_pcie_smmu_setup(struct device *dev)
{
	return 0;
}

static inline void abc_pcie_smmu_remove(struct device *dev,
		struct abc_pcie_devdata *abc) { }
#endif

static int allocate_bar_range(struct device *dev, uint32_t bar,
		size_t size_aligned, uint64_t *bar_offset)
{
	struct abc_pcie_devdata *abc = dev_get_drvdata(dev);
	unsigned long bar_addr;

	if (bar != BAR_2) {
		dev_err(dev, "%s: unable to allocate on BAR %d\n", __func__,
				bar);
		return -EINVAL;
	}

	bar_addr = gen_pool_alloc(abc->iatu_mappings.bar2_pool,
			size_aligned);
	if (!bar_addr) {
		dev_err(dev, "%s: allocation failed\n", __func__);
		return -ENOMEM;
	}

	*bar_offset = bar_addr - abc->abc_dev->bar_base[BAR_2].start;
	return 0;
}

/* Function placeholder: to be implemented when deallocate is supported. */
static int free_bar_range(struct device *dev, uint32_t bar,
		size_t size_aligned, uint64_t bar_offset)
{
	struct abc_pcie_devdata *abc = dev_get_drvdata(dev);

	if (bar != BAR_2) {
		dev_err(dev, "%s: selecting BAR %d is invalid\n", __func__,
				bar);
		return -EINVAL;
	}

	gen_pool_free(abc->iatu_mappings.bar2_pool,
			bar_offset + abc->abc_dev->bar_base[BAR_2].start,
			size_aligned);

	return 0;
}

int abc_pcie_get_inbound_iatu(struct device *dev, struct device *owner)
{
	struct abc_pcie_devdata *abc = dev_get_drvdata(dev);
	struct iatu_status *iatu;
	int iatu_id = -1;
	int i;

	mutex_lock(&abc->mutex);
	/* Find first free iATU region. Skipping iATU1 for it is reserved for
	 * BAR0 mapping
	 */
	for (i = 0; i < NUM_IATU_REGIONS; ++i) {
		if (abc->iatu_mappings.iatus[i].is_used == false) {
			iatu_id = i;
			break;
		}
	}

	/* If no iatu is avaialble return EBUSY error */
	if (iatu_id == -1) {
		mutex_unlock(&abc->mutex);
		dev_err(dev, "All iATU are currently in use.\n");
		return -EBUSY;
	}

	iatu = &abc->iatu_mappings.iatus[iatu_id];

	iatu->is_used = true;
	iatu->owner = owner;

	mutex_unlock(&abc->mutex);
	return iatu_id;
}

int abc_pcie_put_inbound_iatu(struct device *dev, struct device *owner,
		int iatu_id)
{
	struct abc_pcie_devdata *abc = dev_get_drvdata(dev);
	struct iatu_status *iatu = &abc->iatu_mappings.iatus[iatu_id];

	/* Check if valid iatu is selected */
	if (iatu_id < 0 || iatu_id >= NUM_IATU_REGIONS ||
			iatu_id == ABC_CONFIG_IATU_REGION) {
		dev_err(dev, "Invalid iatu region id:%d\n", iatu_id);
		return -EINVAL;
	}

	mutex_lock(&abc->mutex);

	if (owner != iatu->owner) {
		mutex_unlock(&abc->mutex);
		dev_err(dev, "Error: attempt to put iATU owned by different component\n");
		return -EACCES;
	}

	if (iatu->size != 0) {
		mutex_unlock(&abc->mutex);
		dev_err(dev, "Error: Attemp to put iATU without freeing mapping\n");
		return -EBUSY;
	}

	iatu->is_used = false;
	iatu->owner = NULL;
	iatu->bar = 0;
	iatu->bar_offset = 0;
	iatu->ab_paddr = 0;
	iatu->size = 0;

	mutex_unlock(&abc->mutex);
	return 0;
}

int abc_pcie_map_iatu(struct device *dev, struct device *owner, uint32_t bar,
		size_t size, uint64_t ab_paddr, struct bar_mapping *mapping)
{
	struct abc_pcie_devdata *abc = dev_get_drvdata(dev);
	int iatu_id = mapping->iatu;
	struct iatu_status *iatu = &abc->iatu_mappings.iatus[iatu_id];
	struct inb_region ir;
	uint64_t bar_range_offset;
	uint64_t host_addr;
	uint32_t ab_paddr_aligned;
	size_t size_aligned;
	size_t requested_size = size;
	int ret;

	if (size == 0) {
		dev_err(dev, "Invalid argument: size is 0\n");
		return -EINVAL;
	}

	/* Check if valid iatu is selected */
	if (iatu_id < 0 || iatu_id >= NUM_IATU_REGIONS ||
			iatu_id == ABC_CONFIG_IATU_REGION) {
		dev_err(dev, "Invalid iatu region id:%d\n", iatu_id);
		return -EINVAL;
	}

	ab_paddr_aligned = ab_paddr & IATU_LWR_TARGET_ADDR_INBOUND_MASK;
	size = size + (ab_paddr & ~IATU_LWR_TARGET_ADDR_INBOUND_MASK);

	/* set size to be 4kB aligned */
	size_aligned = ALIGN(size, IATU_REGION_ALIGNMENT);

	mutex_lock(&abc->mutex);

	if (iatu->owner != owner) {
		mutex_unlock(&abc->mutex);
		dev_err(dev, "Error: attempt to modify iATU used by another component\n");
		return -EACCES;
	}

	/* allocate a range in the target bar */
	ret = allocate_bar_range(dev, bar, size_aligned, &bar_range_offset);
	if (ret < 0) {
		mutex_unlock(&abc->mutex);
		dev_err(dev, "BAR range allocation failed.\n");
		return ret;
	}

	host_addr = abc->abc_dev->bar_base[bar].start +
			bar_range_offset;

	if (host_addr >> 32) {
		free_bar_range(dev, bar, size_aligned, bar_range_offset);
		mutex_unlock(&abc->mutex);
		dev_err(dev, "Invalid host_addr: address higher than 4GB\n");
		return -EINVAL;
	}

	ir.region = iatu_id;
	ir.mode = MEM_MATCH;
	ir.memmode = 0;
	ir.bar = bar;
	ir.base_address = (uint32_t)(host_addr & 0xFFFFFFFF);
	ir.u_base_address = (uint32_t)(host_addr >> 32);
	ir.limit_address = (uint32_t)((host_addr + size_aligned - 1) &
			0xFFFFFFFF);
	ir.target_pcie_address = (uint32_t)ab_paddr_aligned;
	ir.u_target_pcie_address = (uint32_t)(ab_paddr >> 32);

	ret = set_inbound_iatu(ir);
	if (ret < 0) {
		free_bar_range(dev, bar, size_aligned, bar_range_offset);
		mutex_unlock(&abc->mutex);
		dev_err(dev, "set_inbound_iatu failed.\n");
		return ret;
	}

	iatu->bar = bar;
	iatu->bar_offset = bar_range_offset;
	iatu->ab_paddr = ab_paddr_aligned;
	iatu->size = size_aligned;

	mapping->bar = bar;
	mapping->mapping_size = requested_size;
	mapping->bar_vaddr = abc->bar[bar] + bar_range_offset +
				(ab_paddr & ~IATU_LWR_TARGET_ADDR_INBOUND_MASK);

	mutex_unlock(&abc->mutex);
	return 0;
}

int abc_pcie_unmap_iatu(struct device *dev, struct device *owner,
		struct bar_mapping *mapping)
{
	struct abc_pcie_devdata *abc = dev_get_drvdata(dev);
	int iatu_id = mapping->iatu;
	struct iatu_status *iatu = &abc->iatu_mappings.iatus[iatu_id];
	int ret;

	if (iatu_id < 0 || iatu_id >= NUM_IATU_REGIONS ||
			iatu_id == ABC_CONFIG_IATU_REGION) {
		dev_err(dev, "%s: incorrect state: invalid iatu id: %d",
				__func__, iatu_id);
		return -EINVAL;
	}

	mutex_lock(&abc->mutex);

	if (owner != iatu->owner) {
		mutex_unlock(&abc->mutex);
		dev_err(dev, "Error: attempt to put iATU owned by different component\n");
		return -EACCES;
	}

	if (!iatu->is_used) {
		mutex_unlock(&abc->mutex);
		dev_err(dev, "%s: inconsistent state: select iatu is not used",
				__func__);
		return -EINVAL;
	}

	if (atomic_read(&abc_dev->link_state) == (ABC_PCIE_LINK_ACTIVE |
				ABC_PCIE_SMMU_ATTACHED))
		disable_inbound_iatu_region(iatu_id);

	ret = free_bar_range(dev, iatu->bar, iatu->size, iatu->bar_offset);
	if (ret < 0) {
		mutex_unlock(&abc->mutex);
		dev_err(dev, "%s: freeing bar allocation failed\n", __func__);
		return ret;
	}

	iatu->bar = 0;
	iatu->bar_offset = 0;
	iatu->ab_paddr = 0;
	iatu->size = 0;

	mapping->bar = 0;
	mapping->mapping_size = 0;
	mapping->bar_vaddr = NULL;

	mutex_unlock(&abc->mutex);

	return 0;
}

int abc_pcie_map_bar_region(struct device *dev, struct device *owner,
		uint32_t bar, size_t size, uint64_t ab_paddr,
		struct bar_mapping *mapping)
{
	int iatu_id = abc_pcie_get_inbound_iatu(dev, owner);
	int ret;

	if (iatu_id < 0)
		return iatu_id;

	mapping->iatu = iatu_id;

	ret = abc_pcie_map_iatu(dev, owner, bar, size, ab_paddr, mapping);
	if (ret < 0) {
		(void)abc_pcie_put_inbound_iatu(dev, owner, iatu_id);
		return ret;
	}

	return 0;
}

int abc_pcie_unmap_bar_region(struct device *dev, struct device *owner,
		struct bar_mapping *mapping)
{
	int ret;

	ret = abc_pcie_unmap_iatu(dev, owner, mapping);
	if (ret < 0) {
		dev_err(dev, "Error: unmap iatu failed\n");
		return ret;
	}

	ret = abc_pcie_put_inbound_iatu(dev, owner, mapping->iatu);
	if (ret < 0) {
		dev_err(dev, "Error: fail to release iatu region:%d\n",
				mapping->iatu);
		return ret;
	}

	return 0;
}

dma_addr_t abc_dma_map_single(void *ptr,  size_t size,
		enum dma_data_direction dir)
{
	return dma_map_single(&abc_dev->pdev->dev, ptr, size, dir);
}
dma_addr_t abc_dma_map_page(struct page *page, size_t offset, size_t size,
		enum dma_data_direction dir)
{
	return dma_map_page(&abc_dev->pdev->dev, page, offset,  size, dir);
}

void abc_dma_unmap_page(dma_addr_t addr, size_t size,
		enum dma_data_direction dir)
{
	dma_unmap_page(&abc_dev->pdev->dev, addr, size, dir);
}
void abc_dma_unmap_single(dma_addr_t addr, size_t size,
		enum dma_data_direction dir)
{
	dma_unmap_single(&abc_dev->pdev->dev, addr, size, dir);
}
void *abc_alloc_coherent(size_t size, dma_addr_t *dma_addr)
{
	return dma_alloc_coherent(&abc_dev->pdev->dev, size, dma_addr,
			GFP_KERNEL |  GFP_DMA | __GFP_ZERO
			);
}

void abc_free_coherent(size_t size, void *cpu_addr, dma_addr_t dma_addr)
{
	dma_free_coherent(&abc_dev->pdev->dev, size, cpu_addr, dma_addr);
}

/**
 *  dma_mblk_start
 *    Perform a link-list multi-block DMA Access.
 *	  Consecutive channels will be used.
 *	IMPORTANT: DMA driver assumes that this function will always return
 *               zero.
 *               If this is no longer the case abc-pcie-dma.c::dma_callback()
 *               must be modified.
 */
int dma_mblk_start(uint8_t chan, enum dma_data_direction dir,
			    phys_addr_t *start_addr, int num_channels)
{
	u32 dma_offset;
	u32 val;
	u32 set_val;
	unsigned long flags;
	int i;

	if (!(dir == DMA_FROM_DEVICE || dir == DMA_TO_DEVICE) ||
			(num_channels < 0) ||
			(num_channels + chan >= ABC_DMA_MAX_CHAN))
		return -EINVAL;

	spin_lock_irqsave(&abc_dev->fsys_reg_lock, flags);

	/* Set SYSREG_FSYS DBI_OVERRIDE for DMA access mode */
	val = readl_relaxed(abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);
	__iormb(val);

	set_val = val & ~(DBI_OVERRIDE_MASK);
	set_val |= DBI_OVERRIDE_DMA;
	__iowmb();

	writel_relaxed(set_val,
		abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);
	__iowmb();

	if (dir == DMA_FROM_DEVICE) {
		writel_relaxed(DMA_ENABLE,
			abc_dev->pcie_config + DMA_WRITE_ENGINE);
		__iowmb();

		writel_relaxed(DMA_MASK, abc_dev->pcie_config +
			DMA_WRITE_INTERRUPT_MASK);

		for (i = 0; i < num_channels; ++i) {
			u32 list_addr_l = LOWER((uint64_t)start_addr[i]);
			u32 list_addr_u = UPPER((uint64_t)start_addr[i]);

			pr_debug("DMA MBLK WRITE[EP2AP]: CH%d\n",
				i + chan);
			dma_offset = (i + chan) * DMA_WRITE_OFFSET;

			writel_relaxed(0x04000308, abc_dev->pcie_config +
				dma_offset + DMA_WRITE_CHANNEL_CONTROL_1);
			writel_relaxed(list_addr_l, abc_dev->pcie_config +
				dma_offset + DMA_LLP_LOW_OFF_WRCH);
			writel_relaxed(list_addr_u, abc_dev->pcie_config +
				dma_offset + DMA_LLP_HIGH_OFF_WRCH);
		}

		__iowmb();

		for (i = 0; i < num_channels; ++i)
			writel_relaxed(i + chan,
				abc_dev->pcie_config + DMA_WRITE_DOORBELL);

	} else {
		writel_relaxed(DMA_ENABLE,
			abc_dev->pcie_config + DMA_READ_ENGINE);
		__iowmb();

		writel_relaxed(DMA_MASK, abc_dev->pcie_config +
		DMA_READ_INTERRUPT_MASK);

		for (i = 0; i < num_channels; ++i) {
			u32 list_addr_l = LOWER((uint64_t)start_addr[i]);
			u32 list_addr_u = UPPER((uint64_t)start_addr[i]);

			pr_debug("DMA MBLK READ[AP2EP]: CH%d\n",
				i + chan);
			dma_offset = (i + chan) * DMA_READ_OFFSET;

			writel_relaxed(0x04000308, abc_dev->pcie_config +
				dma_offset + DMA_READ_CHANNEL_CONTROL_1);
			writel_relaxed(list_addr_l, abc_dev->pcie_config +
				dma_offset + DMA_LLP_LOW_OFF_RDCH);
			writel_relaxed(list_addr_u, abc_dev->pcie_config +
				dma_offset + DMA_LLP_HIGH_OFF_RDCH);
		}

		__iowmb();

		for (i = 0; i < num_channels; ++i)
			writel_relaxed(i + chan,
				abc_dev->pcie_config + DMA_READ_DOORBELL);
	}

	__iowmb();
	writel_relaxed(val, abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);

	spin_unlock_irqrestore(&abc_dev->fsys_reg_lock, flags);

	return 0;
}

/**
 *  dma_sblk_start
 *    Perform a single block DMA Access.
 *    IMPORTANT: DMA driver assumes that this function will always return
 *               zero (the first check below is not expected to fail).
 *               If this is no longer the case abc-pcie-dma.c::dma_callback()
 *               must be modified.
 */
int dma_sblk_start(uint8_t chan, enum dma_data_direction dir,
					struct dma_element_t *blk)
{
	u32 dma_offset;
	u32 val;
	u32 set_val;
	unsigned long flags;

	if (!(dir == DMA_FROM_DEVICE || dir == DMA_TO_DEVICE) ||
			(chan >= ABC_DMA_MAX_CHAN))
		return -EINVAL;

	spin_lock_irqsave(&abc_dev->fsys_reg_lock, flags);

	/* Set SYSREG_FSYS DBI_OVERRIDE for DMA access mode */
	val = readl(abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);
	set_val = val & ~(DBI_OVERRIDE_MASK);
	set_val |= DBI_OVERRIDE_DMA;
	__iowmb();
	writel_relaxed(set_val,
			abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);

	if (dir ==  DMA_TO_DEVICE) {
		dma_offset = chan * DMA_READ_OFFSET;

		__iowmb();

		writel_relaxed(DMA_ENABLE, abc_dev->pcie_config +
				DMA_READ_ENGINE);
		writel_relaxed(DMA_MASK, abc_dev->pcie_config +
				DMA_READ_INTERRUPT_MASK);
		writel_relaxed(0x04000008, abc_dev->pcie_config + dma_offset +
				DMA_READ_CHANNEL_CONTROL_1);
		writel_relaxed(blk->len, abc_dev->pcie_config + dma_offset +
				DMA_READ_TRANSFER_SIZE);
		writel_relaxed(blk->src_addr, abc_dev->pcie_config +
				dma_offset + DMA_READ_SAR_LOW);
		writel_relaxed(blk->src_u_addr, abc_dev->pcie_config +
				dma_offset + DMA_READ_SAR_HIGH);
		writel_relaxed(blk->dst_addr, abc_dev->pcie_config +
				dma_offset + DMA_READ_DAR_LOW);
		writel_relaxed(blk->dst_u_addr, abc_dev->pcie_config +
				dma_offset + DMA_READ_DAR_HIGH);
		writel_relaxed(chan, abc_dev->pcie_config + DMA_READ_DOORBELL);
	} else {
		dma_offset = chan * DMA_WRITE_OFFSET;

		__iowmb();

		writel_relaxed(DMA_ENABLE, abc_dev->pcie_config +
				DMA_WRITE_ENGINE);
		writel_relaxed(DMA_MASK, abc_dev->pcie_config +
				DMA_WRITE_INTERRUPT_MASK);
		writel_relaxed(0x04000008, abc_dev->pcie_config + dma_offset +
				DMA_WRITE_CHANNEL_CONTROL_1);
		writel_relaxed(blk->len, abc_dev->pcie_config + dma_offset +
				DMA_WRITE_TRANSFER_SIZE);
		writel_relaxed(blk->src_addr, abc_dev->pcie_config +
				dma_offset + DMA_WRITE_SAR_LOW);
		writel_relaxed(blk->src_u_addr, abc_dev->pcie_config +
				dma_offset + DMA_WRITE_SAR_HIGH);
		writel_relaxed(blk->dst_addr, abc_dev->pcie_config +
				dma_offset + DMA_WRITE_DAR_LOW);
		writel_relaxed(blk->dst_u_addr, abc_dev->pcie_config +
				dma_offset + DMA_WRITE_DAR_HIGH);
		writel_relaxed(chan, abc_dev->pcie_config + DMA_WRITE_DOORBELL);
	}

	__iowmb();
	writel_relaxed(val, abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);

	spin_unlock_irqrestore(&abc_dev->fsys_reg_lock, flags);

	return 0;
}

int abc_reg_notifier_callback(struct notifier_block *nb)
{
	int ret;

	if (!abc_dev || !abc_dev->pcie_config)
		return -EAGAIN;

	ret = atomic_notifier_chain_register(&abc_dev->intnc_notifier, nb);
	if (ret < 0)
		pr_err("%s: Could not register notifier, ret\n", __func__, ret);

	return ret;
}
EXPORT_SYMBOL(abc_reg_notifier_callback);

int abc_reg_irq_callback(irq_cb_t sys_cb, int irq_no, void *data)
{
	unsigned long flags;

	if (irq_no >= ABC_MSI_0_TMU_AON && irq_no <= ABC_MSI_14_FLUSH_DONE) {
		spin_lock_irqsave(&abc_dev->lock, flags);
		abc_dev->sys_cb[irq_no] = sys_cb;
		abc_dev->sys_cb_data[irq_no] = data;
		spin_unlock_irqrestore(&abc_dev->lock, flags);
		return 0;
	}
	return -EINVAL;
}
EXPORT_SYMBOL(abc_reg_irq_callback);

int abc_reg_dma_irq_callback(irq_dma_cb_t dma_cb, int dma_chan)
{
	unsigned long flags;

	spin_lock_irqsave(&abc_dev->dma_callback_lock, flags);

	if (dma_chan >= 0 && dma_chan <= 7)
		abc_dev->dma_cb[dma_chan] = dma_cb;

	spin_unlock_irqrestore(&abc_dev->dma_callback_lock, flags);

	return 0;
}

int abc_register_pcie_link_blocking_event(struct notifier_block *nb)
{
	if (!abc_dev)
		return -EAGAIN;

	return blocking_notifier_chain_register(
				&abc_dev->pcie_link_subscribers, nb);
}
EXPORT_SYMBOL(abc_register_pcie_link_blocking_event);

int abc_unregister_pcie_link_blocking_event(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(
				&abc_dev->pcie_link_subscribers, nb);
}
EXPORT_SYMBOL(abc_unregister_pcie_link_blocking_event);

/**
 * abc_pcie_link_notify_blocking - call PCIe link blocking notifier chain
 * @event: PCIe link notifier type (see include/linux/mfd/abc-pcie-notifier.h)
 *
 * Intended to be called by abc-pcie mfd driver only.
 * Returns NOTIFY_DONE from the last driver called if all went well,
 * or NOTIFY_STOP or NOTIFY_BAD immediately if a driver returns that,
 * or -EAGAIN if abc-pcie mfd driver has not initialized.
 */
int abc_pcie_link_notify_blocking(unsigned long event)
{
	if (!abc_dev)
		return -EAGAIN;

	return blocking_notifier_call_chain(&abc_dev->pcie_link_subscribers,
					    event,
					    NULL);
}

static irqreturn_t abc_pcie_dma_irq_handler(int irq, void *ptr)
{
	u32 override_val;
	u32 override_set_val;
	u32 dma_chan;
	u32 dma_rd_stat;
	u32 dma_wr_stat;
	struct abc_pcie_dma_irq_data *dma_irq_data = ptr;
	irq_dma_cb_t dma_cb;

	spin_lock(&abc_dev->fsys_reg_lock);

	/* Set SYSREG_FSYS DBI_OVERRIDE for DMA access mode */
	override_val = readl(abc_dev->fsys_config +
					SYSREG_FSYS_DBI_OVERRIDE);
	override_set_val = override_val & ~(DBI_OVERRIDE_MASK);
	override_set_val = DBI_OVERRIDE_DMA;
	__iowmb();
	writel_relaxed(override_set_val, abc_dev->fsys_config +
					SYSREG_FSYS_DBI_OVERRIDE);

	if (dma_irq_data->dma_type == ABC_PCIE_DMA_READ) {

		/* DMA Read Callback Implementation */
		dma_rd_stat = readl(
			abc_dev->pcie_config + DMA_READ_INT_STATUS_OFF);

		dma_chan = dma_irq_data->dma_channel;
		pr_debug("---dma_rd_stat = 0x%x--, channel: 0x%x\n",
			 dma_rd_stat, dma_chan);

		__iowmb();
		writel_relaxed((0x1 << dma_chan) | (0x1 << (dma_chan + 16)),
				abc_dev->pcie_config + DMA_READ_INT_CLEAR_OFF);
	} else if (dma_irq_data->dma_type == ABC_PCIE_DMA_WRITE) {

		/* DMA Write Callback Implementation */
		dma_wr_stat = readl(
			abc_dev->pcie_config + DMA_WRITE_INT_STATUS_OFF);

		dma_chan = dma_irq_data->dma_channel;
		pr_debug("---dma_wr_stat = 0x%x--, channel: 0x%x\n",
			 dma_wr_stat, dma_chan);

		__iowmb();
		writel_relaxed((0x1 << dma_chan) | (0x1 << (dma_chan + 16)),
				abc_dev->pcie_config + DMA_WRITE_INT_CLEAR_OFF);
	}

	__iowmb();
	writel_relaxed(override_val,
		abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);

	spin_unlock(&abc_dev->fsys_reg_lock);

	spin_lock(&abc_dev->dma_callback_lock);
	dma_chan = dma_irq_data->dma_channel;
	dma_cb = abc_dev->dma_cb[dma_chan];
	if (!dma_cb)
		goto unlock;

	if (dma_irq_data->dma_type == ABC_PCIE_DMA_READ) {
		if (dma_rd_stat & (1 << dma_chan))
			dma_cb(dma_chan, DMA_TO_DEVICE, DMA_DONE);

		if (dma_rd_stat & (1 << (dma_chan + 16)))
			dma_cb(dma_chan, DMA_TO_DEVICE, DMA_ABORT);
	} else if (dma_irq_data->dma_type == ABC_PCIE_DMA_WRITE) {
		if (dma_wr_stat & (1 << dma_chan))
			dma_cb(dma_chan, DMA_FROM_DEVICE, DMA_DONE);

		if (dma_wr_stat & (1 << (dma_chan + 16)))
			dma_cb(dma_chan, DMA_FROM_DEVICE, DMA_ABORT);
	}

unlock:
	spin_unlock(&abc_dev->dma_callback_lock);

	return IRQ_HANDLED;
}

static irqreturn_t abc_pcie_irq_handler(int irq, void *ptr)
{
	struct pci_dev *pdev = ptr;
	u32 intnc_val;
	u32 msi_cap_val;

	irq = irq - pdev->irq;
	pr_info_ratelimited("MSI Irq number is : %d\n", irq);
	spin_lock(&abc_dev->lock);

	if (irq >= ABC_MSI_0_TMU_AON && irq <= ABC_MSI_14_FLUSH_DONE) {
		if (abc_dev->sys_cb[irq] != NULL) {
			/* Callback respective handler */
			abc_dev->sys_cb[irq](irq, abc_dev->sys_cb_data[irq]);
		}
	} else if (irq == ABC_MSI_AON_INTNC) {
		/*
		 * SYSREG_FSYS's DBI_OVERRIDE register would change the access
		 * mode to override the address of MSI_CAP_OFF_10H_REG. So this
		 * lock is required to prevent the access for DBI_OVERRIDE and
		 * access MSI_CAP_OFF_10H_REG safely.
		 */
		spin_lock(&abc_dev->fsys_reg_lock);
		/* Mask 31st MSI during Interrupt handling period */
		msi_cap_val = readl(
				abc_dev->pcie_config + MSI_CAP_OFF_10H_REG);
		__iowmb();
		writel_relaxed((msi_cap_val | MSI_CAP_MASK_31),
			abc_dev->pcie_config + MSI_CAP_OFF_10H_REG);
		spin_unlock(&abc_dev->fsys_reg_lock);

		/* Check the sysreg status register and do the callback */
		intnc_val = readl(
			abc_dev->fsys_config + SYSREG_FSYS_INTERRUPT);
		atomic_notifier_call_chain(&abc_dev->intnc_notifier,
					   irq, (void *)(u64)intnc_val);

		/* Prevent the access for SYSREG_FSYS's DBI_OVERRIDE. */
		spin_lock(&abc_dev->fsys_reg_lock);
		__iowmb();
		writel_relaxed(msi_cap_val,
			abc_dev->pcie_config + MSI_CAP_OFF_10H_REG);
		spin_unlock(&abc_dev->fsys_reg_lock);
	}
	spin_unlock(&abc_dev->lock);
	return IRQ_HANDLED;
}

#define ABC_BASE_OTP_WRAPPER		0x10BB0000
#define OTP_CHIP_ID_ADDR		(ABC_BASE_OTP_WRAPPER + 0x10)
#define OTP_CHIP_ID_SHIFT		20
#define OTP_CHIP_ID_MASK		(0xF << OTP_CHIP_ID_SHIFT)

static int abc_pcie_get_chip_id_handler(void *ctx, enum ab_chip_id *val)
{
	uint32_t data;
	struct device *dev = (struct device *)ctx;
	int ret = abc_pcie_config_read(OTP_CHIP_ID_ADDR & 0xffffff, 0x0, &data);

	if (ret) {
		dev_err(dev, "Unable to read ab chip id (err %d)\n", ret);
		return ret;
	}

	data = (data & OTP_CHIP_ID_MASK) >> OTP_CHIP_ID_SHIFT;
	*val = (enum ab_chip_id)data;
	return 0;
}

static int abc_pcie_enter_el2_handler(void *ctx)
{
	struct device *dev = (struct device *)ctx;

	dev_info(dev, "%s: enter\n", __func__);

	if (!abc_dev)
		return -ENOTCONN;

	/*
	 * If PCIe link is not enabled, this handler should not have been
	 * called.
	 */
	if (WARN_ON(!(atomic_read(&abc_dev->link_state) &
					ABC_PCIE_LINK_STATE_MASK)))
		return -EINVAL;

	dev_info(dev, "Broadcast Enter EL2 notification\n");

	/* Broadcast this event to subscribers */
	abc_pcie_link_notify_blocking(ABC_PCIE_LINK_PRE_DISABLE |
					ABC_PCIE_LINK_ENTER_EL2);

	dev_info(dev, "%s: disabling dma\n", __func__);
	/* Call PCIe DMA ops after notifying other clients */
	if (abc_dev->dma_device_ops)
		abc_dev->dma_device_ops->pre_disable();

	dev_info(dev, "%s: disabling irq\n", __func__);
	/* Disable PCIe interrupts during EL2 */
	abc_pcie_disable_irqs(abc_dev->pdev);

	dev_info(dev, "%s: detaching SMMU\n", __func__);

	/* Detach the PCIe EP device to the ARM sMMU */
	abc_pcie_smmu_detach((struct device *)ctx);

	dev_info(dev, "%s: done\n", __func__);

	return 0;
}

static int abc_pcie_exit_el2_handler(void *ctx)
{
	struct device *dev = (struct device *)ctx;
	uint32_t test_read_data = 0;
	int ret;

	dev_info(dev, "%s: enter\n", __func__);

	if (!abc_dev)
		return -ENOTCONN;

	/*
	 * If PCIe link is not enabled, this handler should not have been
	 * called.
	 */
	if (WARN_ON(!(atomic_read(&abc_dev->link_state) &
					ABC_PCIE_LINK_STATE_MASK)))
		return -ENOTCONN;


	dev_info(dev, "%s: attaching SMMU\n", __func__);

	/* Re-attach the PCIe EP device to the ARM sMMU */
	ret = abc_pcie_smmu_attach((struct device *)ctx);
	if (ret) {
		dev_err(dev, "%s: failed to attach SMMU: %d\n",
			__func__, ret);
		return ret;
	}

	dev_info(dev, "%s: enabling irq\n", __func__);

	/* Enable PCIe interrupts on EL2 exit */
	abc_pcie_enable_irqs(abc_dev->pdev);

	dev_info(dev, "%s: re-enabling dma\n", __func__);
	/* Call PCIe DMA ops before notifying other clients */
	if (abc_dev->dma_device_ops)
		abc_dev->dma_device_ops->post_enable();

	dev_info(dev, "%s: testing pcie read\n", __func__);
	ret = abc_pcie_config_read(ABC_BASE_OTP_WRAPPER & 0xffffff,
					   0x0, &test_read_data);

	if (ret || test_read_data == ABC_INVALID_DATA) {
		/*
		 * Note: if it were an smmu issue, this error log
		 * may not have a chance to print.
		 */
		dev_err(dev, "%s: failed to read 0x%08x, val=0x%08X, ret=%d\n",
			__func__, ABC_BASE_OTP_WRAPPER, test_read_data, ret);

		return (test_read_data == ABC_INVALID_DATA) ? -ENOTCONN : ret;
	}

	dev_info(dev, "%s: test read [0x%08x] = 0x%08x\n",
			__func__, ABC_BASE_OTP_WRAPPER, test_read_data);

	dev_info(dev, "Broadcast Exit EL2 notification\n");

	/* Broadcast this event to subscribers */
	abc_pcie_link_notify_blocking(ABC_PCIE_LINK_POST_ENABLE |
					ABC_PCIE_LINK_EXIT_EL2);

	dev_info(dev, "%s: done\n", __func__);

	return 0;
}

/* ab_ready also implies that PCIe link is enable */
static int abc_pcie_ab_ready_handler(void *ctx)
{
	struct device *dev = (struct device *)ctx;

	dev_dbg(dev,
		"%s: ab_ready is high; PCIe link is enabled by host\n",
		__func__);

	if (!abc_dev)
		return -ENOTCONN;

	/*
	 * Set link_state to active.
	 * If PCIe link is already enabled, there is no need to broadcast a
	 * POST_ENABLE event.  This situation can be valid because it may
	 * happen during PCIe enmueration.
	 */
	if (atomic_fetch_or(ABC_PCIE_LINK_STATE_MASK, &abc_dev->link_state) &
			ABC_PCIE_LINK_STATE_MASK)
		return 0;

	abc_pcie_enable_irqs(abc_dev->pdev);

	/* Call PCIe DMA ops before notifying other clients */
	if (abc_dev->dma_device_ops)
		abc_dev->dma_device_ops->post_enable();

	/* Broadcast this event to subscribers */
	abc_pcie_link_notify_blocking(ABC_PCIE_LINK_POST_ENABLE);
	return 0;
}

static int abc_pcie_pre_disable_handler(void *ctx)
{
	struct device *dev = (struct device *)ctx;

	dev_dbg(dev,
		"%s: PCIe link will be disabled by host\n",
		__func__);

	if (!abc_dev)
		return -ENOTCONN;

	/*
	 * If PCIe link is already disabled, there is no need to broadcast a
	 * PRE_DISABLE event.  This situation is not supposed to happen.
	 */
	if (WARN_ON(!(atomic_read(&abc_dev->link_state) &
					ABC_PCIE_LINK_STATE_MASK)))
		return 0;

	/* Broadcast this event to subscribers */
	abc_pcie_link_notify_blocking(ABC_PCIE_LINK_PRE_DISABLE);

	/* Call PCIe DMA ops after notifying other clients */
	if (abc_dev->dma_device_ops)
		abc_dev->dma_device_ops->pre_disable();

	abc_pcie_disable_irqs(abc_dev->pdev);

	/*
	 * Set link_state to inactive.
	 * If link_state is already inactive, it's likely a programming bug.
	 */
	WARN_ON(!(atomic_fetch_and(~ABC_PCIE_LINK_STATE_MASK,
					&abc_dev->link_state) &
				ABC_PCIE_LINK_STATE_MASK));

	return 0;
}

static int abc_pcie_linkdown_handler(void *ctx)
{
	struct device *dev = (struct device *)ctx;
	int ret = 0;

	dev_dbg(dev,
		"%s: PCIe link unexpectedly went down\n",
		__func__);

	if (!abc_dev)
		return -ENOTCONN;

	/*
	 * Unlike abc_pcie_pre_disable_handler, here link_state is set to
	 * inactive as soon as possible before notifying subscribers.
	 *
	 * If PCIe link is already disabled, there is no need to broadcast a
	 * LINK_ERROR event.  This situation is not supposed to happen, though.
	 */
	if (WARN_ON(!(atomic_fetch_and(~ABC_PCIE_LINK_STATE_MASK,
						&abc_dev->link_state) &
					ABC_PCIE_LINK_STATE_MASK)))
		return 0;

	dev_warn(dev, "Broadcast link error notification\n");
	abc_pcie_link_notify_blocking(ABC_PCIE_LINK_ERROR);

	/*
	 * If linkdown happens during EL2 mode, make sure to re-attach SMMU
	 * and skip disabling irqs.
	 * If linkdown happens during EL1 mode, disable irqs.
	 */
	if (!(atomic_read(&abc_dev->link_state) &
			  ABC_PCIE_SMMU_ATTACH_STATE_MASK)) {
		dev_info(dev, "linkdown during EL2 mode; re-attach smmu\n");
		ret = abc_pcie_smmu_attach(dev);
		if (ret)
			dev_err(dev, "failed to attach SMMU: %d\n", ret);

		dev_info(dev, "handle linkdown event in dma driver\n");
		if (abc_dev->dma_device_ops)
			abc_dev->dma_device_ops->link_error();
	} else {
		if (abc_dev->dma_device_ops)
			abc_dev->dma_device_ops->link_error();

		dev_info(dev, "linkdown while EL1 has access; disable irq\n");
		abc_pcie_disable_irqs(abc_dev->pdev);
	}

	return ret;
}

static struct ab_sm_mfd_ops mfd_ops = {
	.enter_el2 = &abc_pcie_enter_el2_handler,
	.exit_el2 = &abc_pcie_exit_el2_handler,
	.get_chip_id = &abc_pcie_get_chip_id_handler,
	.ab_ready = &abc_pcie_ab_ready_handler,
	.pcie_pre_disable = &abc_pcie_pre_disable_handler,
	.pcie_linkdown = &abc_pcie_linkdown_handler,
};

static void abc_pcie_enable_irqs(struct pci_dev *pdev)
{
	int i;

	dev_dbg(&pdev->dev, "%s: enter\n", __func__);

	for (i = ABC_MSI_0_TMU_AON; i < ABC_MSI_RD_DMA_0; i++) {
		/*
		 * ABC_MSI_2_IPU_IRQ0 and ABC_MSI_3_IPU_IRQ1 are registered by
		 * the paintbox IPU driver.
		 */
		if (i == ABC_MSI_2_IPU_IRQ0 || i == ABC_MSI_3_IPU_IRQ1)
			continue;

		if (i == ABC_MSI_4_TPU_IRQ0 || i == ABC_MSI_5_TPU_IRQ1) {
			/*
			 * TODO(b/120049047): move enable_irq decisions
			 * to child drivers.
			 */
			enable_irq(pdev->irq + i);
			continue;
		}
		enable_irq(pdev->irq + i);
	}

	for (i = ABC_MSI_RD_DMA_0; i <= ABC_MSI_WR_DMA_7; i++)
		enable_irq(pdev->irq + i);

	enable_irq(pdev->irq + ABC_MSI_AON_INTNC);
}

static void abc_pcie_disable_irqs(struct pci_dev *pdev)
{
	int i;

	dev_dbg(&pdev->dev, "%s: enter\n", __func__);

	disable_irq(pdev->irq + ABC_MSI_AON_INTNC);

	for (i = ABC_MSI_RD_DMA_0; i <= ABC_MSI_WR_DMA_7; i++)
		disable_irq(pdev->irq + i);

	for (i = ABC_MSI_0_TMU_AON; i < ABC_MSI_RD_DMA_0; i++) {
		/*
		 * ABC_MSI_2_IPU_IRQ0 and ABC_MSI_3_IPU_IRQ1 are registered by
		 * the paintbox IPU driver.
		 */
		if (i == ABC_MSI_2_IPU_IRQ0 || i == ABC_MSI_3_IPU_IRQ1)
			continue;

		if (i == ABC_MSI_4_TPU_IRQ0 || i == ABC_MSI_5_TPU_IRQ1) {
			/*
			 * TODO(b/120049047): move disable_irq decisions
			 * to child drivers.
			 */
			disable_irq(pdev->irq + i);
			continue;
		}
		disable_irq(pdev->irq + i);
	}
}

uint32_t abc_pcie_irq_init(struct pci_dev *pdev)
{
	int err, vector, i;
	struct abc_pcie_devdata *abc = dev_get_drvdata(&pdev->dev);

	abc->msi = pci_msi_vec_count(pdev);

	vector = pci_alloc_irq_vectors(pdev, 1, abc->msi, PCI_IRQ_MSI);
	if (vector < 0) {
		dev_err(&pdev->dev, "failed to get MSI interrupts\n");
		return vector;
	}

	/* MSI IRQs Request for system IP's */
	for (i = ABC_MSI_0_TMU_AON; i < ABC_MSI_RD_DMA_0; i++) {
		/*
		 * ABC_MSI_2_IPU_IRQ0 and ABC_MSI_3_IPU_IRQ1 are registered by
		 * the paintbox IPU driver.  Similar for the 2 high prio
		 * TPU interrupts.
		 */
		if (i == ABC_MSI_2_IPU_IRQ0 || i == ABC_MSI_3_IPU_IRQ1 ||
		    i == ABC_MSI_4_TPU_IRQ0 || i == ABC_MSI_5_TPU_IRQ1)
			continue;
		err = request_irq(pdev->irq + i,
				abc_pcie_irq_handler, 0,
				DRV_NAME_ABC_PCIE, pdev);
		if (err) {
			dev_err(&pdev->dev, "failed to req MSI:%d, err:%d\n",
					pdev->irq + i, err);
			goto free_irq;
		}
		dev_dbg(&pdev->dev, "request irq:%d\n", pdev->irq+i);
	}

	/* MSI IRQs request for dma read & write channels */
	for (i = 0; i < NUM_EP_DMA_CHANNELS; i++) {
		abc->dma_irq_data_rd[i].pdev = pdev;
		abc->dma_irq_data_rd[i].dma_channel = i;
		abc->dma_irq_data_rd[i].dma_type = ABC_PCIE_DMA_READ;

		abc->dma_irq_data_wr[i].pdev = pdev;
		abc->dma_irq_data_wr[i].dma_channel = i;
		abc->dma_irq_data_wr[i].dma_type = ABC_PCIE_DMA_WRITE;

		err = request_irq(pdev->irq + ABC_MSI_RD_DMA_0 + i,
				abc_pcie_dma_irq_handler, 0,
				DRV_NAME_ABC_PCIE,
				&abc->dma_irq_data_rd[i]);
		if (err) {
			dev_err(&pdev->dev, "failed to req MSI:%d, err:%d\n",
				pdev->irq + ABC_MSI_RD_DMA_0 + i, err);
			goto free_irq;
		}

		err = request_irq(pdev->irq + ABC_MSI_WR_DMA_0 + i,
				abc_pcie_dma_irq_handler, 0,
				DRV_NAME_ABC_PCIE,
				&abc->dma_irq_data_wr[i]);
		if (err) {
			dev_err(&pdev->dev, "failed to req MSI:%d, err:%d\n",
				pdev->irq + ABC_MSI_WR_DMA_0 + i, err);
			goto free_irq;
		}

		dev_dbg(&pdev->dev, "request irq:%d\n", pdev->irq+i);
	}

	err = request_irq(pdev->irq + ABC_MSI_AON_INTNC,
			abc_pcie_irq_handler, 0,
			DRV_NAME_ABC_PCIE, pdev);
	if (err) {
		dev_err(&pdev->dev, "failed to req MSI:%d, err:%d\n",
				pdev->irq + ABC_MSI_AON_INTNC, err);
		goto free_irq;
	}
	dev_dbg(&pdev->dev, "request irq:%d\n", pdev->irq+ABC_MSI_AON_INTNC);
	return 0;

free_irq:
	for (--i; i >= ABC_MSI_0_TMU_AON; i--) {
		if (i == ABC_MSI_2_IPU_IRQ0 || i == ABC_MSI_3_IPU_IRQ1 ||
		    i == ABC_MSI_4_TPU_IRQ0 || i == ABC_MSI_5_TPU_IRQ1)
			continue;
		free_irq(pdev->irq + i, pdev);
	}

	pci_free_irq_vectors(pdev);
	return err;

}

static void abc_pcie_irq_free(struct pci_dev *pdev)
{
	unsigned int i;
	struct abc_pcie_devdata *abc = dev_get_drvdata(&pdev->dev);

	for (i = ABC_MSI_0_TMU_AON; i <= ABC_MSI_14_FLUSH_DONE; i++) {
		if (i == ABC_MSI_2_IPU_IRQ0 || i == ABC_MSI_3_IPU_IRQ1 ||
		    i == ABC_MSI_4_TPU_IRQ0 || i == ABC_MSI_5_TPU_IRQ1)
			continue;
		free_irq(pdev->irq + i, pdev);
	}

	/* MSI IRQs request for dma read & write channels */
	for (i = 0; i < NUM_EP_DMA_CHANNELS; i++) {
		free_irq(pdev->irq + ABC_MSI_RD_DMA_0 + i,
			 &abc->dma_irq_data_rd[i]);

		free_irq(pdev->irq + ABC_MSI_WR_DMA_0 + i,
			 &abc->dma_irq_data_wr[i]);
	}

	free_irq(pdev->irq + ABC_MSI_AON_INTNC, pdev);

	pci_free_irq_vectors(pdev);
}

#define IPU_INTNC_NOTIFIER	0 /* index of intnc-notifier-chain ipu prop */
static struct property_entry ipu_properties[] = {
	/* filled in with notifier chain for mux'ed low-priority interrupts */
	PROPERTY_ENTRY_U64("intnc-notifier-chain", 0),
	{ }
};

static const struct resource ipu_resources[] = {
	{
		.name = DRV_NAME_ABC_PCIE_IPU,
		.start = ABC_MSI_2_IPU_IRQ0,
		.end = ABC_MSI_2_IPU_IRQ0,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = DRV_NAME_ABC_PCIE_IPU,
		.start = ABC_MSI_3_IPU_IRQ1,
		.end = ABC_MSI_3_IPU_IRQ1,
		.flags = IORESOURCE_IRQ,
	}
};

#define TPU_MEM_MAPPING		0 /* index of tpu-mem-mapping property */
#define TPU_INTNC_NOTIFIER	1 /* index of intnc-notifier-chain tpu prop */
static struct property_entry tpu_properties[] = {
	/* filled in with VA of ioremap of tpu-mem */
	PROPERTY_ENTRY_U64("tpu-mem-mapping", 0),
	/* filled in with notifier chain for mux'ed low-priority interrupts */
	PROPERTY_ENTRY_U64("intnc-notifier-chain", 0),
	{ }
};

static struct resource tpu_resources[] = {
	{
		.name = "tpu-mem",
		.start = 0,
		.end = (2 * 1024 * 1024) - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "tpu-scalar-core-0-irq",
		.start = ABC_MSI_4_TPU_IRQ0,
		.end = ABC_MSI_4_TPU_IRQ0,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "tpu-instr-queue-irq",
		.start = ABC_MSI_5_TPU_IRQ1,
		.end = ABC_MSI_5_TPU_IRQ1,
		.flags = IORESOURCE_IRQ,
	},
};

#define DEV(_name, _r) { .name = _name }
#define DEVPROP(_name, _r, _p)						   \
	{ .name = _name, .num_resources = ARRAY_SIZE(_r), .resources = _r,   \
	      .properties = _p }

static struct mfd_cell abc_pcie_bar0[] = {
	DEVPROP(DRV_NAME_ABC_PCIE_TPU, tpu_resources, tpu_properties),
	DEVPROP(DRV_NAME_ABC_PCIE_IPU, ipu_resources, ipu_properties),
#ifndef CONFIG_MULTIPLE_BAR_MAP_FOR_ABC_SFR
	DEV(DRV_NAME_ABC_PCIE_DMA, pcie_dma_resources),
#endif
};

#ifdef CONFIG_MULTIPLE_BAR_MAP_FOR_ABC_SFR
static struct mfd_cell abc_pcie_bar2[] = {
	DEV(DRV_NAME_ABC_PCIE_DMA, pcie_dma_resources)
};
#endif

static const struct pci_device_id abc_pcie_ids[] = {
	/* ABC Vendor and DeviceID */
	{ PCI_DEVICE(0x1ae0, 0xabcd) },
	{ 0, }
};

static int abc_pcie_init_child_devices(struct pci_dev *pdev)
{
	int err;

	/* fill in address of notifier block for the INTNC mux'ed IRQ */
	ipu_properties[IPU_INTNC_NOTIFIER].value.u64_data =
		(u64)&abc_dev->intnc_notifier;

	/* fill in tpu-mem-mapping with VA of our mapping for tpu-mem */
	tpu_properties[TPU_MEM_MAPPING].value.u64_data =
	    (u64)abc_dev->tpu_config;
	/* fill in address of notifier block for the INTNC mux'ed IRQ */
	tpu_properties[TPU_INTNC_NOTIFIER].value.u64_data =
	    (u64)&abc_dev->intnc_notifier;

	err = mfd_add_devices(&pdev->dev, PLATFORM_DEVID_NONE, abc_pcie_bar0,
			ARRAY_SIZE(abc_pcie_bar0), &pdev->resource[0],
			pdev->irq, NULL);
	if (err < 0) {
		dev_err(&pdev->dev, "mfd_add_devices[0] failed: %d\n", err);
		return err;
	}

#ifdef CONFIG_MULTIPLE_BAR_MAP_FOR_ABC_SFR
	err = mfd_add_devices(&pdev->dev, PLATFORM_DEVID_NONE, abc_pcie_bar2,
			ARRAY_SIZE(abc_pcie_bar2), &pdev->resource[2], 0, NULL);
	if (err < 0) {
		mfd_remove_devices(&pdev->dev);
		dev_err(&pdev->dev, "mfd_add_devices[2] failed: %d\n", err);
		return err;
	}
#endif

	return 0;
}

void ab_pcie_register_dma_device_ops(struct abc_pcie_dma_ops *dma_device_ops)
{
	abc_dev->dma_device_ops = dma_device_ops;
}

void ab_pcie_unregister_dma_device_ops(void)
{
	abc_dev->dma_device_ops = NULL;
}

static int abc_pcie_probe(struct pci_dev *pdev,
				   const struct pci_device_id *ent)
{
	int i;
	int err;
	int bar;
	void __iomem *base;
	struct device *dev = &pdev->dev;
	struct abc_pcie_devdata *abc;
	struct resource *res;
	struct pci_bus_region region;
	unsigned long flags;

	if (pci_is_bridge(pdev))
		return -ENODEV;

	abc = kzalloc(sizeof(struct abc_pcie_devdata), GFP_KERNEL);
	if (abc == NULL)
		return -ENOMEM;

	abc_dev = kzalloc(sizeof(struct abc_device), GFP_KERNEL);
	if (abc_dev == NULL) {
		err = -ENOMEM;
		goto err_alloc_abc_dev;
	}

	abc_dev->pdev = pdev;
	abc_dev->dev  = dev;
	abc->abc_dev = abc_dev;
	atomic_set(&abc_dev->link_state, ABC_PCIE_LINK_NOT_ACTIVE);
	abc_dev->link_config_cache = ABC_PCIE_CACHE_UNKNOWN;
	abc_dev->l1sub_reg_cache = ABC_PCIE_CACHE_UNKNOWN;
	abc_dev->link_status_reg_cache = ABC_PCIE_CACHE_UNKNOWN;
	abc_dev->ltr_enable = false;
	abc_dev->clk_req_enable = false;
	/* Assigning abc_pcie_devdata as driver data to abc_pcie driver */
	dev_set_drvdata(&pdev->dev, abc);

	err = abc_pcie_smmu_setup(dev, abc);
	if (err) {
		dev_err(dev, "Failed to set up SMMU (%d)\n", err);
		goto err_smmu_setup;
	}

	err = pci_enable_device(pdev);
	if (err) {
		dev_err(dev, "Cannot enable PCI device\n");
		goto err_pci_enable_dev;
	}

	err = pci_request_regions(pdev, DRV_NAME_ABC_PCIE);
	if (err) {
		dev_err(dev, "Cannot obtain PCI resources\n");
		goto err_pci_request_regions;
	}

	pci_set_master(pdev);

	mutex_init(&abc->mutex);
	spin_lock_init(&abc_dev->lock);
	spin_lock_init(&abc_dev->fsys_reg_lock);
	spin_lock_init(&abc_dev->dma_callback_lock);
	ATOMIC_INIT_NOTIFIER_HEAD(&abc_dev->intnc_notifier);

	bar = 0;
	/* Restricting till BAR4 mapping */
	while (bar < 5) {
		base = pci_ioremap_bar(pdev, bar);
		if (!base) {
			dev_warn(dev, "failed to read BAR%d\n", bar);
			if (bar >= 5)
				goto exit_loop;
			WARN_ON(bar == BAR_0);
		}
		abc->bar[bar] = base;
		res = &pdev->resource[bar];

#ifdef CONFIG_MULTIPLE_BAR_MAP_FOR_ABC_SFR
		if (bar == 0) {
			/* In current setup BAR0 Mapped for IPU & TPU */
			abc_dev->tpu_config = base;
			/* IPU is located at 2MB from TPU block */
			abc_dev->ipu_config = base + (2 * 1024 * 1024);
		}
		if (bar == 2) {
			/* In current setup BAR2 Mapped for FSYS & DBI */
			abc_dev->fsys_config = base;
			/* PCIe-DBI base address is at 1MB from FSYS base */
			abc_dev->pcie_config = base + (1024 * 1024);
			for (i = 0; i < ARRAY_SIZE(abc_pcie_bar2); i++) {
				/* Assigning abc_device as platform data to
				 * abc_pcie mfd cell drivers
				 */
				abc_pcie_bar2[i].platform_data = abc_dev;
				abc_pcie_bar2[i].pdata_size = sizeof(*abc_dev);
			}
		}
		if (bar == 4) {
			/* In current setup BAR4 Mapped for AON */
			abc_dev->aon_config = base;
		}
		if (bar == 3)
			abc_dev->memory_config = base;
#else
		if (bar == 0) {
			/* Mapping complete SFR region into single BAR */
			abc_dev->base_config = base;
			abc_dev->tpu_config = base;
			abc_dev->ipu_config = base + IPU_START;
			abc_dev->fsys_config = base + FSYS_START;
			abc_dev->pcie_config = base + DBI_START;
			abc_dev->aon_config = base + AON_AXI2APB;
			for (i = 0; i < ARRAY_SIZE(abc_pcie_bar0); i++) {
				/* Assigning abc_device as platform data to
				 * abc_pcie mfd cell drivers
				 */
				abc_pcie_bar0[i].platform_data = abc_dev;
				abc_pcie_bar0[i].pdata_size = sizeof(*abc_dev);
			}
		}
		if (bar == 2)
			abc_dev->bar2_base = base;
		if (bar == 4)
			abc_dev->bar4_base = base;
#endif
		pcibios_resource_to_bus(pdev->bus, &region, res);
		abc_dev->bar_base[bar].start = region.start;
		abc_dev->bar_base[bar].end   = region.end;
#ifdef CONFIG_MULTIPLE_BAR_MAP_FOR_ABC_SFR
		bar++;
#else
		bar += 2;
#endif
	}

	pci_enable_pcie_error_reporting(pdev);
	pci_save_state(pdev);
exit_loop:
	/* iatu 1 is used by firmware for BAR_0 mapping, it is in use by
	 * default
	 */
	abc->iatu_mappings.iatus[1].is_used = true;

	abc->iatu_mappings.bar2_pool =
		devm_gen_pool_create(dev, fls(IATU_REGION_ALIGNMENT) - 1, -1,
				NULL);
	if (IS_ERR(abc->iatu_mappings.bar2_pool)) {
		err = PTR_ERR(abc->iatu_mappings.bar2_pool);
		goto err_pcie_init;
	}

	err = gen_pool_add(abc->iatu_mappings.bar2_pool,
			abc_dev->bar_base[BAR_2].start,
			abc_dev->bar_base[BAR_2].end -
			abc_dev->bar_base[BAR_2].start + 1, -1);
	if (err < 0)
		goto err_pcie_init;

	/* IRQ handling */
#if CONFIG_PCI_MSI
	err = abc_pcie_irq_init(pdev);
	if (err) {
		dev_err(&pdev->dev, "abc_irq_init failed\n");
		goto err_pcie_init;
	}
#endif
#if IS_ENABLED(CONFIG_AIRBRUSH_SM)
	/* Registering the callback to the ASM */
	ab_sm_register_blk_callback(BLK_FSYS,
			abc_pcie_state_manager, (void *)pdev);
#endif
	pci_set_drvdata(pdev, abc);

	atomic_set(&abc_dev->link_state, ABC_PCIE_LINK_ACTIVE |
			ABC_PCIE_SMMU_ATTACHED);

	BLOCKING_INIT_NOTIFIER_HEAD(&abc_dev->pcie_link_subscribers);

	err = mfd_add_devices(dev, PLATFORM_DEVID_NONE, abc_mfd_of_nommu_devs,
			ARRAY_SIZE(abc_mfd_of_nommu_devs), NULL, 0, NULL);
	if (err < 0)
		goto err_add_mfd_child;

	err = abc_pcie_init_child_devices(pdev);
	if (err < 0)
		goto err_ipu_tpu_init;

	/*
	 * Double check that dma_mask does not happen to be overridden by mfd
	 * children devices.
	 */
	WARN_ON(dma_get_mask(dev) != DMA_BIT_MASK(64));

	/* Register state manager operations */
	mfd_ops.ctx = dev;
	ab_sm_register_mfd_ops(&mfd_ops);

	/* Prevent concurrent access to SYSREG_FSYS's DBI_OVERRIDE. */
	spin_lock_irqsave(&abc_dev->fsys_reg_lock, flags);
	abc_l1ss_timeout_ctrl(false);
	spin_unlock_irqrestore(&abc_dev->fsys_reg_lock, flags);

	return 0;

err_ipu_tpu_init:
	mfd_remove_devices(dev);
err_add_mfd_child:
	abc_pcie_irq_free(pdev);
err_pcie_init:
	pci_release_regions(pdev);
err_pci_request_regions:
	pci_disable_device(pdev);
err_pci_enable_dev:
	abc_pcie_smmu_remove(dev, abc);
err_smmu_setup:
	kfree(abc_dev);
err_alloc_abc_dev:
	kfree(abc);

	return err;
}

static void abc_pcie_remove(struct pci_dev *pdev)
{
	enum pci_barno bar;
	struct abc_pcie_devdata *abc = pci_get_drvdata(pdev);

	mfd_remove_devices(&pdev->dev);
	abc_pcie_smmu_remove(&pdev->dev, abc);

	for (bar = BAR_0; bar <= BAR_4; bar += 2) {
		if (abc->bar[bar])
			iounmap(abc->bar[bar]);
	}

	abc_pcie_irq_free(pdev);
	pci_set_drvdata(pdev, NULL);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	ab_sm_unregister_mfd_ops();
	kfree(abc_dev);
	abc_dev = NULL;
	kfree(abc);
}

static void abc_dev_disable(struct pci_dev *pdev)
{
	if (pci_is_enabled(pdev)) {
		pci_disable_pcie_error_reporting(pdev);
		pci_disable_device(pdev);
	}
}

/* TODO(b/117430457):  PCIe errors need to be communicated to the MFD children.
 */

static pci_ers_result_t abc_error_detected(struct pci_dev *pdev,
						pci_channel_state_t state)
{
	struct device *dev = &pdev->dev;

	atomic_set(&abc_dev->link_state, ABC_PCIE_LINK_NOT_ACTIVE);

	switch (state) {
	case pci_channel_io_normal:
		return PCI_ERS_RESULT_CAN_RECOVER;
	case pci_channel_io_frozen:
		dev_warn(dev,
			"frozen state error detected, reset controller\n");
		abc_dev_disable(pdev);
		return PCI_ERS_RESULT_NEED_RESET;
	case pci_channel_io_perm_failure:
		dev_warn(dev,
			"failure state error detected, request disconnect\n");
		return PCI_ERS_RESULT_DISCONNECT;
	}
	return PCI_ERS_RESULT_NEED_RESET;
}

static pci_ers_result_t abc_slot_reset(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	int result;

	dev_dbg(dev, "restart after slot reset\n");
	pci_restore_state(pdev);
	result = pci_enable_device(pdev);
	if (result) {
		dev_err(dev, "%s:PCIe device enable failed result=%d\n",
				__func__, result);
		return result;
	}
	pci_set_master(pdev);
	result = pci_enable_pcie_error_reporting(pdev);
	if (result)
		dev_warn(dev, "%s:PCIe error reporting enable failed res=%d\n",
				__func__, result);
	result = pci_save_state(pdev);
	if (result)
		dev_warn(dev, "%s:PCIe state saving failed result=%d\n",
				__func__, result);

	return PCI_ERS_RESULT_RECOVERED;
}

static void abc_error_resume(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	int result;

	dev_dbg(dev, "Resume after recovery\n");

	result = pci_cleanup_aer_uncorrect_error_status(pdev);
	if (result)
		dev_err(dev, "%s:AERUncorrectable status clean fail,res=%d\n",
			__func__, result);
}

static const struct pci_error_handlers abc_err_handler = {
	.error_detected	= abc_error_detected,
	.slot_reset	= abc_slot_reset,
	.resume		= abc_error_resume,
};

static struct pci_driver abc_pcie_driver = {
	.name = DRV_NAME_ABC_PCIE,
	.id_table = abc_pcie_ids,
	.probe = abc_pcie_probe,
	.remove = abc_pcie_remove,
	.shutdown = abc_pcie_remove,
	.err_handler = &abc_err_handler,
};

module_pci_driver(abc_pcie_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sayanta Pattanayak <sayanta.p@samsung.com>");
MODULE_DESCRIPTION("Airbrush PCI-e function Driver");
