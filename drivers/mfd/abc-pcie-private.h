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

#ifndef __ABC_PCIE_PRIVATE_H
#define __ABC_PCIE_PRIVATE_H

#include <linux/genalloc.h>
#include <linux/mfd/abc-pcie.h>
#include <linux/cdev.h>

#define IATU_CTRL_2_REGION_EN_MASK 0x1
#define IATU_CTRL_2_REGION_EN_SHIFT 31

#define IATU_CTRL_2_MATCH_MODE_MASK 0x1
#define IATU_CTRL_2_MATCH_MODE_SHIFT 30

#define IATU_CTRL_2_BAR_NUM_MASK 0x7
#define IATU_CTRL_2_BAR_NUM_SHIFT 8

#define NUM_IATU_REGIONS 16
#define IATU_REGION_ALIGNMENT (4 * 1024) /* 4kB */

struct iatu_status {
	bool is_used;
	struct device *owner;
	uint32_t bar;
	uint32_t bar_offset;
	uint32_t ab_paddr;
	size_t size;
};

struct iatu_bar_mapping {
	struct iatu_status iatus[NUM_IATU_REGIONS];
	struct gen_pool *bar2_pool;
};

#define ABC_PCIE_DMA_READ	(0x0)
#define ABC_PCIE_DMA_WRITE	(0x1)

struct abc_pcie_dma_irq_data {
	struct pci_dev	*pdev;
	unsigned int	dma_channel;
	unsigned int	dma_type;
};

struct abc_pcie_devdata {
	struct abc_device *abc_dev;
#if IS_ENABLED(CONFIG_ARM64_DMA_USE_IOMMU)
	struct dma_iommu_mapping *iommu_mapping;
#endif
	int irq;
	struct cdev c_dev;
	void __iomem *bar[6];
	uint32_t msi;
	struct mutex mutex;
	struct iatu_bar_mapping iatu_mappings;

	struct abc_pcie_dma_irq_data dma_irq_data_rd[NUM_EP_DMA_CHANNELS];
	struct abc_pcie_dma_irq_data dma_irq_data_wr[NUM_EP_DMA_CHANNELS];
};

/* DISABLE for 0, ENABLE for 1 */
enum {
	ABC_PCIE_PM_DISABLE,
	ABC_PCIE_PM_ENABLE
};

#define ABC_PCIE_LINK_STATE_SHIFT 0
#define ABC_PCIE_LINK_STATE_MASK (1 << ABC_PCIE_LINK_STATE_SHIFT)
#define ABC_PCIE_SMMU_ATTACH_STATE_SHIFT 1
#define ABC_PCIE_SMMU_ATTACH_STATE_MASK (1 << ABC_PCIE_SMMU_ATTACH_STATE_SHIFT)
/* enum for link_state */
enum {
	ABC_PCIE_LINK_NOT_ACTIVE = 0,
	ABC_PCIE_LINK_ACTIVE = ABC_PCIE_LINK_STATE_MASK,
};

/* enum for smmu attachment state */
enum {
	ABC_PCIE_SMMU_NOT_ATTACHED = 0,
	ABC_PCIE_SMMU_ATTACHED = ABC_PCIE_SMMU_ATTACH_STATE_MASK,
};
#endif
