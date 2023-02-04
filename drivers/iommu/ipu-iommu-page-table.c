/*
 * IOMMU page table for the Paintbox programmable IPU library
 *
 * Copyright 2019 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Derived from CPU-agnostic ARM page table allocator.
 *
 * Copyright (C) 2014 ARM Limited
 *
 * Author: Will Deacon <will.deacon@arm.com>
 */
#include <asm/barrier.h>
#include <linux/ab-dram.h>
#include <linux/mfd/abc-pcie-dma.h>
#include <linux/atomic.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <linux/mfd/abc-pcie.h>
#include <linux/types.h>

#include "io-pgtable.h"
#include "ipu-iommu-page-table.h"

typedef u64 arm_lpae_iopte;

/* Struct accessors */
#define io_pgtable_to_data(x)						\
	container_of((x), struct arm_lpae_io_pgtable, iop)

#define io_pgtable_ops_to_data(x)					\
	io_pgtable_to_data(io_pgtable_ops_to_pgtable(x))

struct arm_lpae_io_pgtable {
	struct io_pgtable	iop;

	int			levels;
	unsigned int		pgd_bits;
	size_t			pgd_size;
	unsigned long		pg_shift;
	unsigned long		bits_per_level;

	void			*pgd;
	void			*pgd_ttbr1;
};

#if IS_ENABLED(CONFIG_IPU_IOMMU_PAGE_TABLE_ON_AB)

struct ipu_iommu_pt_stat_data {
	uint64_t total;
	uint32_t count;
};

enum {
	IPU_IOMMU_PT_CNT_ABDRAM_ALLOC,
	IPU_IOMMU_PT_CNT_LOCAL_ALLOC,
	IPU_IOMMU_PT_CNT_LVL1_PAGES,
	IPU_IOMMU_PT_CNT_ACTIVE_MAPPING,
	IPU_IOMMU_PT_CNT_COUNT
};

static const char * const ipu_iommu_pt_stat_names[] = {
	[IPU_IOMMU_PT_CNT_ABDRAM_ALLOC] = "allocated on abdram",
	[IPU_IOMMU_PT_CNT_LOCAL_ALLOC] = "allocated on host",
	[IPU_IOMMU_PT_CNT_LVL1_PAGES] = "allocated level 1 entries",
	[IPU_IOMMU_PT_CNT_ACTIVE_MAPPING] = "active iommu mappings"
};

static inline void ipu_iommu_add_sample(struct ipu_iommu_pt_stat_data *stat,
						int64_t sample)
{
	stat->total += sample;
	stat->count++;
}

struct ipu_iommu_pt_stat_data ipu_iommu_pt_stat[IPU_IOMMU_PT_CNT_COUNT];

#define COLLECT_SAMPLE(stat_type, val) \
	ipu_iommu_add_sample(&ipu_iommu_pt_stat[stat_type], val)

ssize_t ipu_iommu_pgtable_report_status(size_t max_size, char *buf)
{
	ssize_t pos;
	int i;

	pos = scnprintf(buf, max_size, "iommu status:\n");

	for (i = 0; i < IPU_IOMMU_PT_CNT_COUNT; i++) {
		pos += scnprintf(buf + pos, max_size - pos, "%s: %#lx (%d)\n",
			ipu_iommu_pt_stat_names[i], ipu_iommu_pt_stat[i].total,
			ipu_iommu_pt_stat[i].count);
	}

	return pos;
}

struct ipu_iommu_page_table_shadow_entry {
	/* link to the arm page table */
	arm_lpae_iopte *page_table_entry;
	/* link to the next level entry */
	struct  ipu_iommu_page_table_shadow_entry *next_lvl;
	struct dma_buf *ab_dram_dma_buf;
};

struct ipu_iommu_page_table {
	struct ipu_iommu_page_table_shadow_entry shadow_table;
	struct io_pgtable_ops *pgtbl_ops;
	struct io_pgtable_ops ops;
	dma_addr_t arm_page_table_dma_address;
	int iatu;
	void *dma_dev;
	struct bar_mapping mapping;
	bool ab_dram_up;
	bool ab_dram_suspended;

	/* implements delay write for map/map_sg:
	 * don't write as long as the memory write are adjacent.
	 * when the first write is received set the ab_backup_needed flag
	 * for every write after that
	 * if start address (both physical and virtual) are adjacent
	 * just increase the counter otherwise write prev buffer and
	 * start a new one.
	 * when the operation finishes write the remaining buffer.
	 */
	bool ab_backup_needed;
	dma_addr_t ab_backup_dest_addr;
	void *ab_backup_source_addr;
	size_t ab_backup_size;

};

#define ARM_LPAE_MAX_ADDR_BITS		48
#define ARM_LPAE_S2_MAX_CONCAT_PAGES	16
#define ARM_LPAE_MAX_LEVELS		4


#define io_pgtable_ops_to_ipu_page_table(x)			\
	container_of((x), struct ipu_iommu_page_table, ops)


/*
 * For consistency with the architecture, we always consider
 * ARM_LPAE_MAX_LEVELS levels, with the walk starting at level n >=0
 */
#define ARM_LPAE_START_LVL(d)		(ARM_LPAE_MAX_LEVELS - (d)->levels)

/*
 * Calculate the right shift amount to get to the portion describing level l
 * in a virtual address mapped by the pagetable in d.
 */
#define ARM_LPAE_LVL_SHIFT(l, d)	\
	((((d)->levels -				\
	((l) - ARM_LPAE_START_LVL(d) + 1)) \
	* (d)->bits_per_level) + (d)->pg_shift)

#define ARM_LPAE_GRANULE(d)		(1UL << (d)->pg_shift)

#define ARM_LPAE_PAGES_PER_PGD(d)					\
	DIV_ROUND_UP((d)->pgd_size, ARM_LPAE_GRANULE(d))

/*
 * Calculate the index at level l used to map virtual address a using the
 * pagetable in d.
 */
#define ARM_LPAE_PGD_IDX(l, d)						\
	((l) == ARM_LPAE_START_LVL(d) ? ilog2(ARM_LPAE_PAGES_PER_PGD(d)) : 0)

#define ARM_LPAE_LVL_MASK(l, d)						\
	((l) == ARM_LPAE_START_LVL(d) ?	(1 << (d)->pgd_bits) - 1 :	\
					(1 << (d)->bits_per_level) - 1)
#define ARM_LPAE_LVL_IDX(a, l, d)	\
	(((u64)(a) >> ARM_LPAE_LVL_SHIFT(l, d)) &	\
	((1 << ((d)->bits_per_level + \
	ARM_LPAE_PGD_IDX(l, d))) - 1))

/* Calculate the block/page mapping size at level l for pagetable in d. */
#define ARM_LPAE_BLOCK_SIZE(l, d)					\
	(1ULL << (ilog2(sizeof(arm_lpae_iopte)) + \
		((ARM_LPAE_MAX_LEVELS - (l)) * (d)->bits_per_level)))


#define PTE_ARM_TO_IPU DIV_ROUND_UP( \
	sizeof(struct ipu_iommu_page_table_shadow_entry), \
	sizeof(arm_lpae_iopte))

/* Page table bits */
#define ARM_LPAE_PTE_TYPE_SHIFT		0
#define ARM_LPAE_PTE_TYPE_MASK		0x3

#define ARM_LPAE_PTE_TYPE_BLOCK		1
#define ARM_LPAE_PTE_TYPE_TABLE		3
#define ARM_LPAE_PTE_TYPE_PAGE		3

#define ARM_LPAE_PTE_SH_MASK		(((arm_lpae_iopte)0x3) << 8)
#define ARM_LPAE_PTE_NSTABLE		(((arm_lpae_iopte)1) << 63)
#define ARM_LPAE_PTE_XN			(((arm_lpae_iopte)3) << 53)
#define ARM_LPAE_PTE_AF			(((arm_lpae_iopte)1) << 10)
#define ARM_LPAE_PTE_SH_NS		(((arm_lpae_iopte)0) << 8)
#define ARM_LPAE_PTE_SH_OS		(((arm_lpae_iopte)2) << 8)
#define ARM_LPAE_PTE_SH_IS		(((arm_lpae_iopte)3) << 8)
#define ARM_LPAE_PTE_NS			(((arm_lpae_iopte)1) << 5)
#define ARM_LPAE_PTE_VALID		(((arm_lpae_iopte)1) << 0)

#define ARM_LPAE_PTE_ATTR_LO_MASK	(((arm_lpae_iopte)0x3ff) << 2)
/* Ignore the contiguous bit for block splitting */
#define ARM_LPAE_PTE_ATTR_HI_MASK	(((arm_lpae_iopte)6) << 52)
#define ARM_LPAE_PTE_ATTR_MASK		(ARM_LPAE_PTE_ATTR_LO_MASK |	\
					 ARM_LPAE_PTE_ATTR_HI_MASK)
/* Software bit for solving coherency races */
#define ARM_LPAE_PTE_SW_SYNC		(((arm_lpae_iopte)1) << 55)

/* Stage-1 PTE */
#define ARM_LPAE_PTE_AP_PRIV_RW		(((arm_lpae_iopte)0) << 6)
#define ARM_LPAE_PTE_AP_UNPRIV		(((arm_lpae_iopte)1) << 6)
#define ARM_LPAE_PTE_AP_PRIV_RO		(((arm_lpae_iopte)2) << 6)
#define ARM_LPAE_PTE_AP_RO		(((arm_lpae_iopte)3) << 6)
#define ARM_LPAE_PTE_ATTRINDX_MASK	0x7
#define ARM_LPAE_PTE_ATTRINDX_SHIFT	2
#define ARM_LPAE_PTE_nG			(((arm_lpae_iopte)1) << 11)

/* Stage-2 PTE */
#define ARM_LPAE_PTE_HAP_FAULT		(((arm_lpae_iopte)0) << 6)
#define ARM_LPAE_PTE_HAP_READ		(((arm_lpae_iopte)1) << 6)
#define ARM_LPAE_PTE_HAP_WRITE		(((arm_lpae_iopte)2) << 6)
#define ARM_LPAE_PTE_MEMATTR_OIWB	(((arm_lpae_iopte)0xf) << 2)
#define ARM_LPAE_PTE_MEMATTR_NC		(((arm_lpae_iopte)0x5) << 2)
#define ARM_LPAE_PTE_MEMATTR_DEV	(((arm_lpae_iopte)0x1) << 2)

/* Register bits */
#define ARM_32_LPAE_TCR_EAE		(1 << 31)
#define ARM_64_LPAE_S2_TCR_RES1		(1 << 31)

#define ARM_LPAE_TCR_EPD1		(1 << 23)

#define ARM_LPAE_TCR_TG0_4K		(0 << 14)
#define ARM_LPAE_TCR_TG0_64K		(1 << 14)
#define ARM_LPAE_TCR_TG0_16K		(2 << 14)

#define ARM_LPAE_TCR_SH0_SHIFT		12
#define ARM_LPAE_TCR_SH0_MASK		0x3
#define ARM_LPAE_TCR_SH_NS		0
#define ARM_LPAE_TCR_SH_OS		2
#define ARM_LPAE_TCR_SH_IS		3

#define ARM_LPAE_TCR_ORGN0_SHIFT	10
#define ARM_LPAE_TCR_IRGN0_SHIFT	8
#define ARM_LPAE_TCR_RGN_MASK		0x3
#define ARM_LPAE_TCR_RGN_NC		0
#define ARM_LPAE_TCR_RGN_WBWA		1
#define ARM_LPAE_TCR_RGN_WT		2
#define ARM_LPAE_TCR_RGN_WB		3

#define ARM_LPAE_TCR_SL0_SHIFT		6
#define ARM_LPAE_TCR_SL0_MASK		0x3

#define ARM_LPAE_TCR_T0SZ_SHIFT		0
#define ARM_LPAE_TCR_SZ_MASK		0xf

#define ARM_LPAE_TCR_PS_SHIFT		16
#define ARM_LPAE_TCR_PS_MASK		0x7

#define ARM_LPAE_TCR_IPS_SHIFT		32
#define ARM_LPAE_TCR_IPS_MASK		0x7

#define ARM_LPAE_TCR_PS_32_BIT		0x0ULL
#define ARM_LPAE_TCR_PS_36_BIT		0x1ULL
#define ARM_LPAE_TCR_PS_40_BIT		0x2ULL
#define ARM_LPAE_TCR_PS_42_BIT		0x3ULL
#define ARM_LPAE_TCR_PS_44_BIT		0x4ULL
#define ARM_LPAE_TCR_PS_48_BIT		0x5ULL

#define ARM_LPAE_MAIR_ATTR_SHIFT(n)	((n) << 3)
#define ARM_LPAE_MAIR1_ATTR_SHIFT(n)	((n-4) << 3)
#define ARM_LPAE_MAIR_ATTR_MASK		0xff
#define ARM_LPAE_MAIR_ATTR_DEVICE	0x04
#define ARM_LPAE_MAIR_ATTR_NC		0x44
#define ARM_LPAE_MAIR_ATTR_WBRWA	0xff
#define ARM_LPAE_MAIR_ATTR_UPSTREAM	0xf4
#define ARM_LPAE_MAIR_ATTR_LLC_NWA	0xe4
#define ARM_LPAE_MAIR_ATTR_IDX_NC	0
#define ARM_LPAE_MAIR_ATTR_IDX_CACHE	1
#define ARM_LPAE_MAIR_ATTR_IDX_DEV	2
#define ARM_LPAE_MAIR_ATTR_IDX_UPSTREAM	3
#define ARM_LPAE_MAIR_ATTR_IDX_LLC_NWA	0x4ULL

/* IOPTE accessors */
#define iopte_deref(pte, d)						\
	(__va(iopte_val(pte) & ((1ULL << ARM_LPAE_MAX_ADDR_BITS) - 1)	\
	& ~(ARM_LPAE_GRANULE(d) - 1ULL)))

#define iopte_type(pte, l)					\
	(((pte) >> ARM_LPAE_PTE_TYPE_SHIFT) & ARM_LPAE_PTE_TYPE_MASK)

#define iopte_prot(pte)	((pte) & ARM_LPAE_PTE_ATTR_MASK)

#define iopte_leaf(pte, l)					\
	(l == (ARM_LPAE_MAX_LEVELS - 1) ?			\
		(iopte_type(pte, l) == ARM_LPAE_PTE_TYPE_PAGE) :	\
		(iopte_type(pte, l) == ARM_LPAE_PTE_TYPE_BLOCK))

#define iopte_to_pfn(pte, d)					\
	(((pte) & ((1ULL << ARM_LPAE_MAX_ADDR_BITS) - 1)) >> (d)->pg_shift)

#define pfn_to_iopte(pfn, d)					\
	(((pfn) << (d)->pg_shift) & ((1ULL << ARM_LPAE_MAX_ADDR_BITS) - 1))

static inline struct arm_lpae_io_pgtable *
	ipu_iommu_pgtable_to_arm_data(struct ipu_iommu_page_table *pg_table)
{
	return io_pgtable_ops_to_data(pg_table->pgtbl_ops);
}

/*
 * We'll use some ignored bits in table entries to keep track of the number
 * of page mappings beneath the table.  The maximum number of entries
 * beneath any table mapping in armv8 is 8192 (which is possible at the
 * 2nd- and 3rd-level when using a 64K granule size).  The bits at our
 * disposal are:
 *
 *     4k granule: [54..52], [11..2]
 *    64k granule: [54..52], [15..2]
 *
 * [54..52], [11..2] is enough bits for tracking table mappings at any
 * level for any granule, so we'll use those.
 */
#define BOTTOM_IGNORED_MASK 0x3ff
#define BOTTOM_IGNORED_SHIFT 2
#define BOTTOM_IGNORED_NUM_BITS 10
#define TOP_IGNORED_MASK 0x7ULL
#define TOP_IGNORED_SHIFT 52
#define IOPTE_RESERVED_MASK ((BOTTOM_IGNORED_MASK << BOTTOM_IGNORED_SHIFT) | \
			     (TOP_IGNORED_MASK << TOP_IGNORED_SHIFT))

static arm_lpae_iopte iopte_val(arm_lpae_iopte table_pte)
{
	return table_pte & ~IOPTE_RESERVED_MASK;
}

static arm_lpae_iopte _iopte_bottom_ignored_val(
	arm_lpae_iopte table_pte)
{
	return (table_pte & (BOTTOM_IGNORED_MASK << BOTTOM_IGNORED_SHIFT))
		>> BOTTOM_IGNORED_SHIFT;
}

static arm_lpae_iopte _iopte_top_ignored_val(
	arm_lpae_iopte table_pte)
{
	return (table_pte & (TOP_IGNORED_MASK << TOP_IGNORED_SHIFT))
		>> TOP_IGNORED_SHIFT;
}

static int iopte_tblcnt(arm_lpae_iopte table_pte)
{
	return (_iopte_bottom_ignored_val(table_pte) |
		(_iopte_top_ignored_val(table_pte) << BOTTOM_IGNORED_NUM_BITS));
}

static void iopte_tblcnt_set(arm_lpae_iopte *table_pte, int val)
{
	arm_lpae_iopte pte = iopte_val(*table_pte);

	pte |= ((val & BOTTOM_IGNORED_MASK) << BOTTOM_IGNORED_SHIFT) |
		 (((val & (TOP_IGNORED_MASK << BOTTOM_IGNORED_NUM_BITS))
		   >> BOTTOM_IGNORED_NUM_BITS) << TOP_IGNORED_SHIFT);
	*table_pte = pte;
}

static void iopte_tblcnt_sub(arm_lpae_iopte *table_ptep, int cnt)
{
	arm_lpae_iopte current_cnt = iopte_tblcnt(*table_ptep);

	current_cnt -= cnt;
	iopte_tblcnt_set(table_ptep, current_cnt);
}

static void iopte_tblcnt_add(arm_lpae_iopte *table_ptep, int cnt)
{
	arm_lpae_iopte current_cnt = iopte_tblcnt(*table_ptep);

	current_cnt += cnt;
	iopte_tblcnt_set(table_ptep, current_cnt);
}

static int ipu_iommu_page_table_dma_transfer(
		struct ipu_iommu_page_table *pg_table,
		void *local_buffer, dma_addr_t remote_addr,
		size_t size)
{
	int err;
	struct abc_pcie_kernel_dma_desc desc;

	memset((void *)&desc, 0, sizeof(desc));
	if (is_vmalloc_addr(local_buffer)) {
		/* mapping is handled by DMA driver for VMALLOC buffers */
		desc.local_buf_kind = DMA_BUFFER_KIND_VMALLOC;
		desc.local_buf = local_buffer;
	} else {
		desc.local_buf = (void *)dma_map_single(pg_table->dma_dev,
			local_buffer, size,
			DMA_TO_DEVICE);
		if (desc.local_buf == 0)
			return -ENOMEM;
		desc.local_buf_kind = DMA_BUFFER_KIND_CMA;
	}

	desc.remote_buf = (uint64_t)remote_addr;
	desc.remote_buf_kind = DMA_BUFFER_KIND_USER;
	desc.size = size;
	desc.dir = DMA_TO_DEVICE;

	err = abc_pcie_issue_sessionless_dma_xfer_sync(&desc);
	if (err)
		dev_err(pg_table->dma_dev,
			"Error (%d) transferring page table with PCIe DMA\n",
			err);
	if (!is_vmalloc_addr(local_buffer))
		dma_unmap_single(pg_table->dma_dev, (dma_addr_t)desc.local_buf,
			size, DMA_TO_DEVICE);

	return err;
}

/* update memory on airbrush */
static int ipu_iommu_page_table_update_ab(
	struct ipu_iommu_page_table *pg_table,
	struct io_pgtable_cfg *cfg,
	dma_addr_t addr,
	arm_lpae_iopte *pte,
	size_t sz)
{
	int err;

	if (!pg_table->ab_dram_up)
		return 0;

	/* for larger sizes using DMA transfer saves CPU workload and
	 * reaches a faster rate
	 */
	if (sz >= SZ_32K)
		return ipu_iommu_page_table_dma_transfer(pg_table,
			pte, addr, sz);

	err =  abc_pcie_map_iatu(pg_table->dma_dev,
			pg_table->dma_dev /* owner */, BAR_2,
			sz, addr,
			&pg_table->mapping);
	if (err) {
		dev_err(cfg->iommu_dev, "%s Error: (%d) unable to map airbrush memory",
			__func__, err);
		return err;
	}
	memcpy_toio(pg_table->mapping.bar_vaddr, pte, sz);
	err = abc_pcie_unmap_iatu(pg_table->dma_dev,
			pg_table->dma_dev /* owner */, &pg_table->mapping);
	if (err) {
		dev_err(cfg->iommu_dev,
			"%s: Error: (%d) unable to unmap airbrush memory\n",
			__func__, err);
		return err;
	}
	return err;
}

static inline bool ipu_iommu_next_is_backup(
	struct ipu_iommu_page_table *pg_table,
	dma_addr_t addr,
	arm_lpae_iopte *ptep)
{
	return pg_table->ab_backup_source_addr +
		pg_table->ab_backup_size == ptep &&
		pg_table->ab_backup_dest_addr +
		pg_table->ab_backup_size == addr;
}

static int __ipu_iommu_page_table_send_updates_to_mem(
	struct ipu_iommu_page_table *pg_table,
	struct io_pgtable_cfg *cfg)
{
	if (!pg_table->ab_backup_needed)
		return 0;
	pg_table->ab_backup_needed = false;
	return ipu_iommu_page_table_update_ab(
		pg_table, cfg,
		pg_table->ab_backup_dest_addr,
		pg_table->ab_backup_source_addr,
		pg_table->ab_backup_size);

	return 0;
}

/* update a single page on airbrush
 * function logic implements delay writes and will
 * try to update all a adjacent memory segemnts
 * push_update - don't try to delay writes and push
 *    write to memory right away.
 */
static int ipu_iommu_page_table_update_ab_page(
	struct ipu_iommu_page_table *pg_table,
	struct io_pgtable_cfg *cfg,
	dma_addr_t addr,
	arm_lpae_iopte *ptep,
	bool push_update)
{
	int err = 0;

	if (push_update)
		return ipu_iommu_page_table_update_ab(
				pg_table, cfg,
				addr,
				ptep,
				sizeof(*ptep));

	/* check if we already started a delay block
	 * if we did check if the current block is adjacent
	 * if so just update the size
	 * otherwise write the prev block
	 */
	if (pg_table->ab_backup_needed) {
		if (ipu_iommu_next_is_backup(pg_table,
				addr, ptep)) {
			pg_table->ab_backup_size += sizeof(*ptep);
		} else {
			err = ipu_iommu_page_table_update_ab(
				pg_table, cfg,
				pg_table->ab_backup_dest_addr,
				pg_table->ab_backup_source_addr,
				pg_table->ab_backup_size);
			pg_table->ab_backup_needed = false;
		}
	}
	/* start a new delay block */
	if (!pg_table->ab_backup_needed) {
		pg_table->ab_backup_needed = true;
		pg_table->ab_backup_size = sizeof(*ptep);
		pg_table->ab_backup_source_addr = ptep;
		pg_table->ab_backup_dest_addr = addr;
	}
	return err;
}

/* set airbrush memory to 0 */
static int ipu_iommu_page_table_reset_ab_mem(
	struct ipu_iommu_page_table *pg_table,
	struct io_pgtable_cfg *cfg,
	struct dma_buf *ab_dram_dma_buf,
	uint64_t delta_addr,
	size_t size)
{
	dma_addr_t addr = ab_dram_get_dma_buf_paddr(ab_dram_dma_buf) +
		delta_addr * sizeof(arm_lpae_iopte);
	int err;

	if (!pg_table->ab_dram_up)
		return 0;

	err =  abc_pcie_map_iatu(pg_table->dma_dev,
			pg_table->dma_dev /* owner */, BAR_2,
			size, addr,
			&pg_table->mapping);
	if (err) {
		dev_err(cfg->iommu_dev,
			"%s Error: (%d) unable to map airbrush memory\n",
			__func__, err);
		return err;
	}
	memset_io(pg_table->mapping.bar_vaddr, 0, size);

	err = abc_pcie_unmap_iatu(pg_table->dma_dev,
			pg_table->dma_dev /* owner */, &pg_table->mapping);
	if (err) {
		dev_err(cfg->iommu_dev,
			"%s: Error: (%d) unable to unmap airbrush memory\n",
			__func__, err);
		return err;
	}
	return err;
}

static struct ipu_iommu_page_table_shadow_entry
	__ipu_iommu_pgtable_alloc_pages(
		struct ipu_iommu_page_table *pg_table,
		size_t size, gfp_t gfp,
		struct io_pgtable_cfg *cfg, void *cookie, bool alloc_shadow,
		bool reset_ab_mem)
{
	struct device *dev = cfg->iommu_dev;
	struct ipu_iommu_page_table_shadow_entry res;

	memset(&res, 0, sizeof(res));
	res.page_table_entry = kvzalloc(size, gfp);

	if (!res.page_table_entry) {
		dev_err(dev,
			"%s Error allocating local page table\n", __func__);
		return res;
	}

	if (alloc_shadow) {
		res.next_lvl = kvzalloc(PTE_ARM_TO_IPU * size, gfp);
		if (!res.next_lvl)
			goto out_free_local;
	} else {
		res.next_lvl = NULL;
	}

	res.ab_dram_dma_buf = ab_dram_alloc_dma_buf_kernel(size);
	if (IS_ERR(res.ab_dram_dma_buf)) {
		dev_err(dev, "%s Error (%d) allocating ab dram page table\n",
			__func__, PTR_ERR(res.ab_dram_dma_buf));
		res.ab_dram_dma_buf = NULL;
		goto free_shadow;
	}
	/* reset shadow memory */
	if (reset_ab_mem && ipu_iommu_page_table_reset_ab_mem(pg_table,
		cfg, res.ab_dram_dma_buf, 0, size)){
		dev_err(dev, "%s Error resetting airbrush memory\n", __func__);
		goto free_ab_dram;
	}

	COLLECT_SAMPLE(IPU_IOMMU_PT_CNT_ABDRAM_ALLOC, size);
	COLLECT_SAMPLE(IPU_IOMMU_PT_CNT_LOCAL_ALLOC,
		size * (1 + (alloc_shadow ? PTE_ARM_TO_IPU : 0)));

	return res;

free_ab_dram:
	ab_dram_free_dma_buf_kernel(res.ab_dram_dma_buf);
free_shadow:
	if (alloc_shadow)
		kvfree(res.next_lvl);
	res.next_lvl = NULL;
out_free_local:
	kvfree(res.page_table_entry);
	res.page_table_entry = NULL;
	return res;
}

static void __ipu_iommu_pgtable_free_pages(
	struct ipu_iommu_page_table_shadow_entry *entry,
	size_t size,
	struct io_pgtable_cfg *cfg,
	void *cookie)
{
	COLLECT_SAMPLE(IPU_IOMMU_PT_CNT_LOCAL_ALLOC, -size *
		(1 + (entry->next_lvl ? PTE_ARM_TO_IPU : 0)));
	kvfree(entry->page_table_entry);
	entry->page_table_entry = NULL;
	kvfree(entry->next_lvl);
	entry->next_lvl = NULL;
	if (entry->ab_dram_dma_buf)
		COLLECT_SAMPLE(IPU_IOMMU_PT_CNT_ABDRAM_ALLOC, -size);
	ab_dram_free_dma_buf_kernel(entry->ab_dram_dma_buf);
	entry->ab_dram_dma_buf = NULL;
}

static void __ipu_iommu_pgtbl_set_pte(struct ipu_iommu_page_table *pg_table,
	struct io_pgtable_cfg *cfg,
	dma_addr_t addr,
	arm_lpae_iopte *ptep,
	arm_lpae_iopte pte,
	bool push_update)
{
	*ptep = pte;
	ipu_iommu_page_table_update_ab_page(pg_table,
		cfg, addr, ptep, push_update);
}

static void __ipu_iommu_pgtbl_init_pte(struct ipu_iommu_page_table *pg_table,
	struct arm_lpae_io_pgtable *data,
	phys_addr_t paddr, arm_lpae_iopte prot,
	int lvl,
	arm_lpae_iopte *ptep,
	dma_addr_t addr)
{
	arm_lpae_iopte pte = prot;

	if (data->iop.cfg.quirks & IO_PGTABLE_QUIRK_ARM_NS)
		pte |= ARM_LPAE_PTE_NS;

	if (lvl == ARM_LPAE_MAX_LEVELS - 1)
		pte |= ARM_LPAE_PTE_TYPE_PAGE;
	else
		pte |= ARM_LPAE_PTE_TYPE_BLOCK;

	pte |= ARM_LPAE_PTE_AF | ARM_LPAE_PTE_SH_OS;
	pte |= pfn_to_iopte(paddr >> data->pg_shift, data);

	__ipu_iommu_pgtbl_set_pte(pg_table,
		&data->iop.cfg, addr, ptep, pte,
		false /* push_update */);
}

static int ipu_iommu_pgtbl_init_pte(struct ipu_iommu_page_table *pg_table,
	struct arm_lpae_io_pgtable *data,
	phys_addr_t paddr,
	arm_lpae_iopte prot, int lvl,
	arm_lpae_iopte *ptep, arm_lpae_iopte *prev_ptep,
	dma_addr_t addr)
{
	arm_lpae_iopte pte = *ptep;

	/* We require an unmap first */
	if (pte & ARM_LPAE_PTE_VALID) {
		WARN_ON(true);
		return -EEXIST;
	}

	__ipu_iommu_pgtbl_init_pte(pg_table,
		data, paddr, prot,
		lvl, ptep,	addr);

	if (prev_ptep)
		iopte_tblcnt_add(prev_ptep, 1);
	return 0;
}

static arm_lpae_iopte ipu_iommu_pgtbl_install_table(
	struct ipu_iommu_page_table *pg_table,
	dma_addr_t table,
	arm_lpae_iopte *ptep,
	arm_lpae_iopte curr,
	struct io_pgtable_cfg *cfg,
	int ref_count,
	dma_addr_t addr)
{
	arm_lpae_iopte old, new;

	new = (uint64_t)(table) | ARM_LPAE_PTE_TYPE_TABLE;
	if (cfg->quirks & IO_PGTABLE_QUIRK_ARM_NS)
		new |= ARM_LPAE_PTE_NSTABLE;
	iopte_tblcnt_set(&new, ref_count);

	/*
	 * Ensure the table itself is visible before its PTE can be.
	 * Whilst we could get away with cmpxchg64_release below, this
	 * doesn't have any ordering semantics when !CONFIG_SMP.
	 */
	dma_wmb();

	old = cmpxchg64_relaxed(ptep, curr, new);

	/* update airbrush memeory */
	ipu_iommu_page_table_update_ab_page(pg_table,
		cfg, addr, ptep, false /* push */);
	return old;
}

struct map_state {
	unsigned long iova_end;
	unsigned int pgsize;
	arm_lpae_iopte *pgtable;
	arm_lpae_iopte *prev_pgtable;
	arm_lpae_iopte *pte_start;
	unsigned int num_pte;
};
/* map state optimization works at level 3 (the 2nd-to-last level) */
#define MAP_STATE_LVL 3

static int __ipu_iommu_pgtable_map(struct ipu_iommu_page_table *pg_table,
	unsigned long iova,
	phys_addr_t paddr, size_t size, arm_lpae_iopte prot,
	int lvl, struct ipu_iommu_page_table_shadow_entry *ptep,
	arm_lpae_iopte *prev_ptep)
{
	arm_lpae_iopte pte;
	struct ipu_iommu_page_table_shadow_entry *nptep;
	arm_lpae_iopte *arm_ptep;
	struct arm_lpae_io_pgtable *data =
		ipu_iommu_pgtable_to_arm_data(pg_table);
	size_t block_size = ARM_LPAE_BLOCK_SIZE(lvl, data);
	size_t tblsz = ARM_LPAE_GRANULE(data);
	struct io_pgtable_cfg *cfg = &data->iop.cfg;
	void *cookie = data->iop.cookie;
	int diff = ARM_LPAE_LVL_IDX(iova, lvl, data);

	/* Find our entry at the current level */
	arm_ptep = ptep->page_table_entry + diff;

	/* If we can install a leaf entry at this level, then do so */
	if (size == block_size && (size & cfg->pgsize_bitmap)) {
		/* map to virtual address -
		 * nothing spacial to do here for ipu
		 */
		return ipu_iommu_pgtbl_init_pte(
			pg_table, data, paddr, prot, lvl,
			arm_ptep, prev_ptep,
			ab_dram_get_dma_buf_paddr(ptep->ab_dram_dma_buf) +
				diff * sizeof(arm_lpae_iopte));
	}
	/* We can't allocate tables at the final level */
	if (WARN_ON(lvl >= ARM_LPAE_MAX_LEVELS - 1))
		return -EINVAL;

	if (WARN_ON(ptep->next_lvl == NULL)) {
		dev_err(cfg->iommu_dev,
			"%s iommu error: current level %d is missing",
			__func__, lvl);
		return -EINVAL;
	}

	nptep = ptep->next_lvl + diff;

	/* Grab a pointer to the next level */
	pte = READ_ONCE(*arm_ptep);
	if (!pte) {
		struct ipu_iommu_page_table_shadow_entry nshadow =
			__ipu_iommu_pgtable_alloc_pages(
			pg_table,
			tblsz,
			GFP_KERNEL,
			cfg, cookie,
			lvl < ARM_LPAE_MAX_LEVELS - 2,
			true /* reset ab mem */);

		if (!nshadow.page_table_entry)
			return -ENOMEM;

		if (lvl < ARM_LPAE_MAX_LEVELS - 2)
			COLLECT_SAMPLE(IPU_IOMMU_PT_CNT_LVL1_PAGES, tblsz);

		/* installing the dma side pointer */
		pte = ipu_iommu_pgtbl_install_table(pg_table,
			ab_dram_get_dma_buf_paddr(nshadow.ab_dram_dma_buf),
				arm_ptep, 0, cfg, 0,
			ab_dram_get_dma_buf_paddr(ptep->ab_dram_dma_buf) +
				diff * sizeof(arm_lpae_iopte));
		if (pte) {
			dev_err(cfg->iommu_dev,
				"%s Error: old page table entry is not empty (0x%llx)",
				pte);
			__ipu_iommu_pgtable_free_pages(&nshadow,
				tblsz, cfg, cookie);
		}	else {
			if (WARN_ON(nptep->next_lvl != NULL ||
					nptep->page_table_entry != NULL)) {
				dev_err(cfg->iommu_dev,
					"%s iommu error: trying to allocate over an existing entry"
					, __func__);
				__ipu_iommu_pgtable_free_pages(&nshadow,
					tblsz, cfg, cookie);
				return -EEXIST;
			}
			*nptep = nshadow;
		}
	}

	if (pte) {
		if (iopte_leaf(pte, lvl)) {
			/* We require an unmap first */
			WARN_ON(true);
			return -EEXIST;
		}
		if (lvl < ARM_LPAE_MAX_LEVELS - 2 &&
			WARN_ON(nptep->next_lvl == NULL)) {
			dev_err(cfg->iommu_dev,
				"%s iommu error: next level is missing",
				__func__);
			return -EEXIST;
		}
	}

	/* Rinse, repeat */
	return __ipu_iommu_pgtable_map(pg_table, iova,
		paddr, size, prot, lvl + 1, nptep,
		arm_ptep);
}

static arm_lpae_iopte ipu_iommu_pgtbl_prot_to_pte(
	struct arm_lpae_io_pgtable *data,
	int prot)
{
	arm_lpae_iopte pte;

	if (data->iop.fmt == ARM_64_LPAE_S1 ||
	    data->iop.fmt == ARM_32_LPAE_S1) {
		pte = ARM_LPAE_PTE_nG;

		if (prot & IOMMU_WRITE)
			pte |= (prot & IOMMU_PRIV) ? ARM_LPAE_PTE_AP_PRIV_RW
					: ARM_LPAE_PTE_AP_UNPRIV;
		else
			pte |= (prot & IOMMU_PRIV) ? ARM_LPAE_PTE_AP_PRIV_RO
					: ARM_LPAE_PTE_AP_RO;

		if (!(prot & IOMMU_PRIV))
			pte |= ARM_LPAE_PTE_AP_UNPRIV;

		if (prot & IOMMU_MMIO)
			pte |= (ARM_LPAE_MAIR_ATTR_IDX_DEV
				<< ARM_LPAE_PTE_ATTRINDX_SHIFT);
		else if (prot & IOMMU_CACHE)
			pte |= (ARM_LPAE_MAIR_ATTR_IDX_CACHE
				<< ARM_LPAE_PTE_ATTRINDX_SHIFT);
		else if (prot & IOMMU_USE_UPSTREAM_HINT)
			pte |= (ARM_LPAE_MAIR_ATTR_IDX_UPSTREAM
				<< ARM_LPAE_PTE_ATTRINDX_SHIFT);
		else if (prot & IOMMU_USE_LLC_NWA)
			pte |= (ARM_LPAE_MAIR_ATTR_IDX_LLC_NWA
				<< ARM_LPAE_PTE_ATTRINDX_SHIFT);
	} else {
		pte = ARM_LPAE_PTE_HAP_FAULT;
		if (prot & IOMMU_READ)
			pte |= ARM_LPAE_PTE_HAP_READ;
		if (prot & IOMMU_WRITE)
			pte |= ARM_LPAE_PTE_HAP_WRITE;
		if (prot & IOMMU_MMIO)
			pte |= ARM_LPAE_PTE_MEMATTR_DEV;
		else if (prot & IOMMU_CACHE)
			pte |= ARM_LPAE_PTE_MEMATTR_OIWB;
		else
			pte |= ARM_LPAE_PTE_MEMATTR_NC;
	}

	if (prot & IOMMU_NOEXEC)
		pte |= ARM_LPAE_PTE_XN;

	return pte;
}

int ipu_iommu_pgtable_map(struct io_pgtable_ops *ops, unsigned long iova,
	phys_addr_t paddr, size_t size, int iommu_prot)
{
	struct arm_lpae_io_pgtable *data;
	struct ipu_iommu_page_table *pg_table;
	int ret, lvl;
	arm_lpae_iopte prot;

	if (!ops)
		return -EINVAL;

	pg_table = io_pgtable_ops_to_ipu_page_table(ops);
	data = ipu_iommu_pgtable_to_arm_data(pg_table);
	lvl = ARM_LPAE_START_LVL(data);
	pg_table->ab_dram_suspended = false;

	/* If no access, then nothing to do */
	if (!(iommu_prot & (IOMMU_READ | IOMMU_WRITE)))
		return 0;

	if (WARN_ON(iova >= (1ULL << data->iop.cfg.ias) ||
		    paddr >= (1ULL << data->iop.cfg.oas)))
		return -ERANGE;

	prot = ipu_iommu_pgtbl_prot_to_pte(data, iommu_prot);
	ret = __ipu_iommu_pgtable_map(pg_table, iova, paddr,
		size, prot, lvl, &pg_table->shadow_table, NULL);

	__ipu_iommu_page_table_send_updates_to_mem(pg_table,
		&data->iop.cfg);
	/*
	 * Synchronise all PTE updates for the new mapping before there's
	 * a chance for anything to kick off a table walk for the new iova.
	 */
	wmb();
	return ret;
}

/* return the lvl1 pointer for the given address
 * updated the lvl counter by the given count
 * if next level doesn't exist - allocate
 */
static int ipu_iommu_return_lvl1(struct ipu_iommu_page_table *pg_table,
	struct arm_lpae_io_pgtable *data, unsigned long iova, size_t tblsz,
	void *cookie, struct io_pgtable_cfg *cfg, int count,
	struct ipu_iommu_page_table_shadow_entry **res)
{
	struct ipu_iommu_page_table_shadow_entry *shadow_entry =
		&pg_table->shadow_table;
	int lvl = ARM_LPAE_START_LVL(data);
	int diff = ARM_LPAE_LVL_IDX(iova, lvl, data);
	arm_lpae_iopte *ptep =
		shadow_entry->page_table_entry + diff;

	if (!(*ptep)) {
		struct ipu_iommu_page_table_shadow_entry nshadow;

		nshadow = __ipu_iommu_pgtable_alloc_pages(pg_table,
			tblsz, GFP_KERNEL, cfg, cookie,
			true /*need shadow entry */,
			true /* reset ab mem */);
		if (!nshadow.page_table_entry)
			return -ENOMEM;
		ipu_iommu_pgtbl_install_table(pg_table,
			ab_dram_get_dma_buf_paddr(nshadow.ab_dram_dma_buf),
			ptep, 0, cfg, 0,
			ab_dram_get_dma_buf_paddr(shadow_entry->ab_dram_dma_buf)
			+ diff * sizeof(arm_lpae_iopte));

		*(shadow_entry->next_lvl + diff) = nshadow;
	}
	iopte_tblcnt_add(ptep, count);
	*res = shadow_entry->next_lvl + diff;
	return 0;
}

static int ipu_iommu_pgtable_bw_map(
	struct ipu_iommu_page_table *pg_table,
	struct arm_lpae_io_pgtable *data,
	arm_lpae_iopte prot,
	unsigned long iova, phys_addr_t paddr,
	size_t size, size_t *ret_size)
{
	struct io_pgtable_cfg *cfg = &data->iop.cfg;
	void *cookie = data->iop.cookie;
	size_t tblsz = ARM_LPAE_GRANULE(data);
	int blocks_needed;
	struct ipu_iommu_page_table_shadow_entry nshadow;
	u64 blk;
	int err = 0;
	struct ipu_iommu_page_table_shadow_entry *shadow_entry;
	struct ipu_iommu_page_table_shadow_entry *nshadow_entry;
	int lvl = ARM_LPAE_START_LVL(data) + 1;
	int diff;
	arm_lpae_iopte *ptep;
	dma_addr_t ab_addr, ab_src_addr;


	dev_dbg(cfg->iommu_dev,
		"%s called for sz 0x%llx, phy addr 0x%llx, iova 0x%llx",
		__func__, size, paddr, iova);
	*ret_size = 0;
	size = min_t(unsigned long, size + iova,
		(iova + SZ_1G) & ~(SZ_1G - 1)) - iova;
	blocks_needed = size / SZ_2M;
	size = blocks_needed * SZ_2M;

	err = ipu_iommu_return_lvl1(pg_table, data, iova, tblsz,
		cookie, cfg, 1,	&shadow_entry);
	if (err) {
		dev_err(cfg->iommu_dev,
			"%s error (%d) locating level 1 entry", __func__,
			err);
		return err;
	}

	nshadow = __ipu_iommu_pgtable_alloc_pages(pg_table,
		tblsz * blocks_needed, GFP_KERNEL, cfg, cookie,
		false /*last lvl does need shadow entry */,
		false /* reset ab mem */);
	if (!nshadow.page_table_entry)
		return -ENOMEM;

	ab_addr = ab_dram_get_dma_buf_paddr(nshadow.ab_dram_dma_buf);
	diff = ARM_LPAE_LVL_IDX(iova, lvl, data);
	ptep = shadow_entry->page_table_entry + diff;
	ab_src_addr = ab_dram_get_dma_buf_paddr(
		shadow_entry->ab_dram_dma_buf) +
		diff * sizeof(arm_lpae_iopte);
	nshadow_entry = (shadow_entry->next_lvl + diff);

	/* install lvl page entries */
	for (blk = 0; blk < blocks_needed; ++blk) {
		ipu_iommu_pgtbl_install_table(pg_table,
			ab_addr, ptep, 0, cfg,
			SZ_4K / sizeof(arm_lpae_iopte) /* ref count */,
			ab_src_addr);

		/* update pointer to the new lvl 2 pointer */
		nshadow_entry->next_lvl = NULL;
		nshadow_entry->page_table_entry =
			(void *)nshadow.page_table_entry
			+ tblsz * blk;
		nshadow_entry->ab_dram_dma_buf = (blk == 0 ?
			nshadow.ab_dram_dma_buf : NULL);

		ab_addr += tblsz;
		ab_src_addr += sizeof(arm_lpae_iopte);
		++ptep;
		++nshadow_entry;
	}

	lvl++;
	ab_addr = ab_dram_get_dma_buf_paddr(nshadow.ab_dram_dma_buf);
	ptep = nshadow.page_table_entry;
	while (*ret_size < size) {
		ipu_iommu_pgtbl_init_pte(
			pg_table, data, paddr, prot, lvl,
			ptep, NULL,
			ab_addr);

		paddr += SZ_4K;
		(*ret_size) += SZ_4K;
		ab_addr += sizeof(arm_lpae_iopte);
		++ptep;
	}

	return err;
}

static int ipu_iommu_pgtable_map_sg(
	struct io_pgtable_ops *ops, unsigned long iova,
	struct scatterlist *sg, unsigned int nents,
	int iommu_prot, size_t *size)
{
	struct arm_lpae_io_pgtable *data;
	struct ipu_iommu_page_table *pg_table;
	int lvl;
	size_t mapped = 0;
	struct io_pgtable_cfg *cfg;
	arm_lpae_iopte prot;
	struct scatterlist *s;
	int i, ret;
	unsigned int min_pagesz;

	if (!ops)
		return -EINVAL;

	/* If no access, then nothing to do */
	if (!(iommu_prot & (IOMMU_READ | IOMMU_WRITE)))
		goto out_err;
	pg_table = io_pgtable_ops_to_ipu_page_table(ops);
	data = ipu_iommu_pgtable_to_arm_data(pg_table);
	lvl = ARM_LPAE_START_LVL(data);
	cfg = &data->iop.cfg;
	prot = ipu_iommu_pgtbl_prot_to_pte(data, iommu_prot);
	min_pagesz = 1 << __ffs(cfg->pgsize_bitmap);
	pg_table->ab_dram_suspended = false;

	for_each_sg(sg, s, nents, i) {
		phys_addr_t phys = page_to_phys(sg_page(s)) + s->offset;
		size_t size = s->length;

		/*
		 * We are mapping on IOMMU page boundaries, so offset within
		 * the page must be 0. However, the IOMMU may support pages
		 * smaller than PAGE_SIZE, so s->offset may still represent
		 * an offset of that boundary within the CPU page.
		 */
		if (!IS_ALIGNED(s->offset, min_pagesz))
			goto out_err;

		while (size) {
			size_t pgsize = iommu_pgsize(
				cfg->pgsize_bitmap, iova | phys, size);
			/* see if we can allocate full 2 mb blocks */
			if (pgsize == SZ_4K && size >= SZ_2M &&
					!(iova & (SZ_2M - 1)))
				ret = ipu_iommu_pgtable_bw_map(pg_table,
						data, prot, iova, phys, size,
						&pgsize);
			else
				ret = __ipu_iommu_pgtable_map(
					pg_table, iova, phys,
					pgsize, prot, lvl,
					&pg_table->shadow_table,
					NULL);
			if (ret)
				goto out_err;

			iova += pgsize;
			mapped += pgsize;
			phys += pgsize;
			size -= pgsize;
		}
	}

	__ipu_iommu_page_table_send_updates_to_mem(pg_table,
		cfg);

	COLLECT_SAMPLE(IPU_IOMMU_PT_CNT_ACTIVE_MAPPING, 1);
	return mapped;

out_err:
	/* Return the size of the partial mapping so that they can be undone */
	*size = mapped;
	return 0;
}

static void __ipu_iommu_free_pgtable(struct ipu_iommu_page_table *pg_table,
	int lvl, struct ipu_iommu_page_table_shadow_entry *shadow_ptep,
	int count)
{
	struct ipu_iommu_page_table_shadow_entry *curr, *end;
	struct arm_lpae_io_pgtable *data =
		ipu_iommu_pgtable_to_arm_data(pg_table);
	unsigned long table_size;
	void *cookie = data->iop.cookie;
	int cnt = 1;

	if (lvl == ARM_LPAE_START_LVL(data))
		table_size = data->pgd_size;
	else
		table_size = ARM_LPAE_GRANULE(data);


	if (lvl == ARM_LPAE_MAX_LEVELS - 1) {
		if (WARN_ON(shadow_ptep->next_lvl != NULL)) {
			dev_err(pg_table->dma_dev,
				"%s Error: last lavel of shadow table is not empty",
				__func__);
		}
		__ipu_iommu_pgtable_free_pages(shadow_ptep,
			table_size * count,
			&data->iop.cfg,
			cookie);
		return;
	}

	curr = shadow_ptep->next_lvl - 1;
	end = (void *)curr + PTE_ARM_TO_IPU * table_size;

	while (curr != end) {
		struct ipu_iommu_page_table_shadow_entry *shadow_centry = end;

		end--;
		if (shadow_centry->next_lvl == NULL)
			continue;

		if (shadow_centry->ab_dram_dma_buf == NULL) {
			++cnt;
			shadow_centry->page_table_entry = NULL;
			shadow_centry->next_lvl = NULL;
			continue;
		}

		__ipu_iommu_free_pgtable(pg_table,
			lvl + 1, shadow_centry, cnt);
		cnt = 1;
	}

	__ipu_iommu_pgtable_free_pages(shadow_ptep,
			table_size,
			&data->iop.cfg,
			cookie);
}

static int __ipu_iommu_load_pgtable(struct ipu_iommu_page_table *pg_table,
	int lvl, struct ipu_iommu_page_table_shadow_entry *shadow_ptep,
	int count)
{
	struct ipu_iommu_page_table_shadow_entry *curr, *end;
	int err;
	struct arm_lpae_io_pgtable *data =
		ipu_iommu_pgtable_to_arm_data(pg_table);
	struct io_pgtable_cfg *cfg = &data->iop.cfg;
	unsigned long table_size;
	dma_addr_t ab_addr = ab_dram_get_dma_buf_paddr(
				shadow_ptep->ab_dram_dma_buf);
	int cnt = 1;

	if (lvl == ARM_LPAE_START_LVL(data))
		table_size = data->pgd_size;
	else
		table_size = ARM_LPAE_GRANULE(data);

	err = ipu_iommu_page_table_update_ab(pg_table,
		cfg, ab_addr, shadow_ptep->page_table_entry,
		table_size * count);
	if (err)
		return err;

	if (lvl == ARM_LPAE_MAX_LEVELS - 1)
		return 0;

	curr = shadow_ptep->next_lvl - 1;
	end = (void *)curr + PTE_ARM_TO_IPU * table_size;

	while (curr != end) {
		struct ipu_iommu_page_table_shadow_entry *shadow_centry = end;

		end--;
		if (shadow_centry->next_lvl == NULL)
			continue;

		if (shadow_centry->ab_dram_dma_buf == NULL) {
			++cnt;
			continue;
		}

		err = __ipu_iommu_load_pgtable(pg_table,
			lvl + 1, shadow_centry, cnt);
		cnt = 1;
		if (err)
			return err;
	}
	return 0;
}

static void __ipu_iommu_free_pgtable_top(
	struct ipu_iommu_page_table *pg_table,
	struct arm_lpae_io_pgtable *data)
{
	__ipu_iommu_free_pgtable(pg_table,
		ARM_LPAE_START_LVL(data), &pg_table->shadow_table, 1);
	kfree(data);
}

void ipu_iommu_free_pgtable(struct io_pgtable_ops *ops)
{
	struct io_pgtable *iop;
	struct arm_lpae_io_pgtable *data;
	struct ipu_iommu_page_table *pg_table;
	struct io_pgtable_ops *arm_ops;
	int err;

	if (!ops)
		return;
	pg_table = io_pgtable_ops_to_ipu_page_table(ops);
	arm_ops = pg_table->pgtbl_ops;
	data = ipu_iommu_pgtable_to_arm_data(pg_table);
	iop = container_of(arm_ops, struct io_pgtable, ops);
	__ipu_iommu_free_pgtable_top(pg_table, data);
	io_pgtable_tlb_flush_all(iop);

	/* free iatu */
	err = abc_pcie_put_inbound_iatu(pg_table->dma_dev,
		pg_table->dma_dev /* owner */, pg_table->iatu);
	if (err)
		dev_err(iop->cfg.iommu_dev,
			"%s error %d freeing iatu %d\n",
			__func__, err, pg_table->iatu);

	kfree(pg_table);
}

static int __ipu_iommu_pgtable_unmap(struct ipu_iommu_page_table *pg_table,
	unsigned long iova, size_t size, int lvl,
	struct ipu_iommu_page_table_shadow_entry *shadow_ptep)
{
	struct arm_lpae_io_pgtable *data =
		ipu_iommu_pgtable_to_arm_data(pg_table);
	arm_lpae_iopte *ptep;
	arm_lpae_iopte pte;
	struct io_pgtable *iop = &data->iop;
	int diff;
	struct ipu_iommu_page_table_shadow_entry *shadow_nptep;

	/* Something went horribly wrong and we ran out of page table */
	if (WARN_ON(lvl == ARM_LPAE_MAX_LEVELS))
		return 0;

	diff = ARM_LPAE_LVL_IDX(iova, lvl, data);
	ptep = shadow_ptep->page_table_entry + diff;
	shadow_nptep = shadow_ptep->next_lvl + diff;

	pte = READ_ONCE(*ptep);
	if (WARN_ON(!pte)) {
		dev_err(iop->cfg.iommu_dev,
			"%s page table was not found", __func__);
		return 0;
	}

	/* If the size matches this level, we're in the right place */
	if (size == ARM_LPAE_BLOCK_SIZE(lvl, data)) {
		__ipu_iommu_pgtbl_set_pte(pg_table,
			&iop->cfg,
			ab_dram_get_dma_buf_paddr(
				shadow_ptep->ab_dram_dma_buf) +
			(sizeof(arm_lpae_iopte) * diff),
			ptep,
			0, false /* push_update */);

		if (!iopte_leaf(pte, lvl))
			__ipu_iommu_free_pgtable(
				pg_table, lvl + 1, shadow_nptep, 1);

		if (shadow_ptep->next_lvl)
			memset(shadow_ptep->next_lvl + diff, 0,
				sizeof(*shadow_ptep));

		return size;
	} else if ((lvl == ARM_LPAE_MAX_LEVELS - 2) &&
		size % SZ_2M == 0) {
		/* blk free */
		struct ipu_iommu_page_table_shadow_entry *shadow_tmp;
		int i, entries, max_entries = size / SZ_2M;
		dma_addr_t ab_addr;

		if (!iopte_leaf(pte, lvl)) {
			/* count */
			for (shadow_tmp = shadow_nptep + 1, entries = 1;
					entries < max_entries;
					++entries, ++shadow_tmp)
				if (shadow_tmp->ab_dram_dma_buf != NULL ||
					shadow_tmp->page_table_entry ==
					NULL)
					break;

			__ipu_iommu_free_pgtable(
				pg_table, lvl + 1, shadow_nptep, entries);
		} else {
			for (entries = 1; entries < max_entries; ++entries) {
				pte = READ_ONCE(
					*(shadow_ptep->page_table_entry +
					diff + entries));
				if (!iopte_leaf(pte, lvl))
					break;
			}
		}

		ab_addr = ab_dram_get_dma_buf_paddr(
					shadow_ptep->ab_dram_dma_buf);
		for (i = 0; i < entries; ++i) {
			__ipu_iommu_pgtbl_set_pte(pg_table,
				&iop->cfg,
				ab_addr +
				(sizeof(arm_lpae_iopte) * (i + diff)),
				ptep + i,
				0, false /* push_update */);
		}

		memset(shadow_ptep->next_lvl + diff, 0,
				sizeof(*shadow_ptep) * entries);

		return entries * SZ_2M;
	} else if ((lvl == ARM_LPAE_MAX_LEVELS - 2) && !iopte_leaf(pte, lvl)) {
		/*
		 * page we want to free is in the next level
		 *  this should be the most common case
		 *  clearing level -1 entries: - only arm table
		 */
		arm_lpae_iopte *table = shadow_nptep->page_table_entry;
		int tl_offset = ARM_LPAE_LVL_IDX(iova, lvl + 1, data);
		int entry_size = ARM_LPAE_GRANULE(data);
		int max_entries = ARM_LPAE_BLOCK_SIZE(lvl, data) >>
				data->pg_shift;
		int entries = min_t(int, size / entry_size,
			max_entries - tl_offset);
		int table_len = entries * sizeof(*table);
		/*
		 * This isn't a block mapping so it must be a table mapping
		 * and since it's the 2nd-to-last level the next level has
		 * to be all page mappings.  Zero them all out in one fell
		 * swoop.
		 */

		table += tl_offset;

		iopte_tblcnt_sub(ptep, entries);
		if (!iopte_tblcnt(*ptep)) {
			/* no valid mappings left under this table. free it. */
			__ipu_iommu_free_pgtable(
				pg_table, lvl + 1, shadow_nptep, 1);
			__ipu_iommu_pgtbl_set_pte(pg_table,
				&iop->cfg,
				ab_dram_get_dma_buf_paddr(
					shadow_ptep->ab_dram_dma_buf) +
				(sizeof(arm_lpae_iopte) * diff),
				ptep,
				0, false /* push_update */);
			memset(shadow_ptep->next_lvl + diff, 0,
				sizeof(*shadow_ptep));

		} else {
			memset(table, 0, table_len);
			/* reset the airbrush memory */
			if (ipu_iommu_page_table_reset_ab_mem(
					pg_table,
					&data->iop.cfg,
					shadow_nptep->ab_dram_dma_buf,
					tl_offset,
					table_len)) {
				dev_err(iop->cfg.iommu_dev,
					"%s error resetting ab memory\n",
					__func__);
				return -EEXIST;
			}
		}

		return entries * entry_size;
	} else if (iopte_leaf(pte, lvl)) {
		/*
		 * Insert a table at the next level to map the old region,
		 * minus the part we want to unmap
		 * this is not supported - should never happen
		 */
		dev_err(iop->cfg.iommu_dev,
			"%s unsupported implementation!", __func__);
		return -EEXIST;
	}
	/* Keep on walkin' */
	return __ipu_iommu_pgtable_unmap(pg_table,
		iova, size, lvl + 1, shadow_nptep);
}

static size_t ipu_iommu_pgtable_unmap(struct io_pgtable_ops *ops,
	unsigned long iova,	size_t size)
{
	struct ipu_iommu_page_table *pg_table;
	struct arm_lpae_io_pgtable *data;
	size_t unmapped;
	int lvl;
	struct ipu_iommu_page_table_shadow_entry *shadow_table;

	if (!ops)
		return 0;
	pg_table = io_pgtable_ops_to_ipu_page_table(ops);
	data = ipu_iommu_pgtable_to_arm_data(pg_table);
	unmapped = 0;
	lvl = ARM_LPAE_START_LVL(data);
	shadow_table = &pg_table->shadow_table;
	pg_table->ab_dram_suspended = false;

	if (WARN_ON(iova >= (1ULL << data->iop.cfg.ias)))
		return 0;

	while (unmapped < size) {
		size_t ret, size_to_unmap, remaining;

		remaining = (size - unmapped);
		size_to_unmap = iommu_pgsize(data->iop.cfg.pgsize_bitmap, iova,
						remaining);

		if (size_to_unmap >= SZ_2M) {
			size_to_unmap = min_t(unsigned long, remaining,
					(ALIGN(iova + 1, SZ_1G) - iova));
			size_to_unmap &= ~(SZ_2M - 1);
		} else
			size_to_unmap = size_to_unmap >= SZ_2M ?
					size_to_unmap :
					min_t(unsigned long, remaining,
					(ALIGN(iova + 1, SZ_2M) - iova));
		ret = __ipu_iommu_pgtable_unmap(pg_table, iova, size_to_unmap,
			lvl, shadow_table);
		if (ret == 0)
			break;
		unmapped += ret;
		iova += ret;
	}
	if (unmapped) {
		__ipu_iommu_page_table_send_updates_to_mem(pg_table,
			&data->iop.cfg);
		io_pgtable_tlb_flush_all(&data->iop);
	}

	COLLECT_SAMPLE(IPU_IOMMU_PT_CNT_ACTIVE_MAPPING, -1);
	return unmapped;
}

static int ipu_iommu_pgtable_iova_to_pte(struct ipu_iommu_page_table *pg_table,
				unsigned long iova, int *plvl_ret,
				arm_lpae_iopte *ptep_ret)
{
	struct arm_lpae_io_pgtable *data;
	struct ipu_iommu_page_table_shadow_entry *shadow_entry;
	arm_lpae_iopte pte, *ptep;

	data = ipu_iommu_pgtable_to_arm_data(pg_table);
	shadow_entry = &pg_table->shadow_table;
	ptep = shadow_entry->page_table_entry;
	*plvl_ret = ARM_LPAE_START_LVL(data);
	*ptep_ret = 0;

	do {
		int diff;
		/* Valid IOPTE pointer? */
		if (!ptep)
			return -EINVAL;

		diff = ARM_LPAE_LVL_IDX(iova, *plvl_ret, data);
		/* Grab the IOPTE we're interested in */
		pte = *(ptep + diff);

		/* Valid entry? */
		if (!pte)
			return -EINVAL;

		/* Leaf entry? */
		if (iopte_leaf(pte, *plvl_ret))
			goto found_translation;

		shadow_entry = shadow_entry->next_lvl + diff;
		ptep = shadow_entry->page_table_entry;
		/* Take it to the next level */
	} while (++(*plvl_ret) < ARM_LPAE_MAX_LEVELS);

	/* Ran out of page tables to walk */
	return -EINVAL;

found_translation:
	*ptep_ret = pte;
	return 0;
}

static uint64_t ipu_iommu_pgtable_iova_get_pte(struct io_pgtable_ops *ops,
	unsigned long iova)
{
	arm_lpae_iopte pte;
	int lvl;

	if (!ipu_iommu_pgtable_iova_to_pte(
		io_pgtable_ops_to_ipu_page_table(ops),
		iova, &lvl, &pte))
		return pte;
	return 0;
}

phys_addr_t ipu_iommu_pgtable_iova_to_phys(struct io_pgtable_ops *ops,
	unsigned long iova)
{
	struct ipu_iommu_page_table *pg_table;
	struct arm_lpae_io_pgtable *data;
	arm_lpae_iopte pte;
	int lvl;
	phys_addr_t phys = 0;

	if (!ops)
		return 0;
	pg_table = io_pgtable_ops_to_ipu_page_table(ops);
	data = ipu_iommu_pgtable_to_arm_data(pg_table);

	if (!ipu_iommu_pgtable_iova_to_pte(pg_table, iova, &lvl, &pte)) {
		iova &= ((1 << ARM_LPAE_LVL_SHIFT(lvl, data)) - 1);
		phys = ((phys_addr_t)iopte_to_pfn(pte, data) <<
			data->pg_shift) | iova;
	}

	return phys;
}

static bool ipu_iommu_pgtbl_is_iova_coherent(struct io_pgtable_ops *ops,
					 unsigned long iova)
{
	return -EEXIST;
}

static void ipu_iommu_pgtbl_restrict_pgsizes(struct io_pgtable_cfg *cfg)
{
	unsigned long granule;

	/*
	 * We need to restrict the supported page sizes to match the
	 * translation regime for a particular granule. Aim to match
	 * the CPU page size if possible, otherwise prefer smaller sizes.
	 * While we're at it, restrict the block sizes to match the
	 * chosen granule.
	 */
	if (cfg->pgsize_bitmap & PAGE_SIZE)
		granule = PAGE_SIZE;
	else if (cfg->pgsize_bitmap & ~PAGE_MASK)
		granule = 1UL << __fls(cfg->pgsize_bitmap & ~PAGE_MASK);
	else if (cfg->pgsize_bitmap & PAGE_MASK)
		granule = 1UL << __ffs(cfg->pgsize_bitmap & PAGE_MASK);
	else
		granule = 0;

	switch (granule) {
	case SZ_4K:
		cfg->pgsize_bitmap &= (SZ_4K | SZ_2M | SZ_1G);
		break;
	case SZ_16K:
		cfg->pgsize_bitmap &= (SZ_16K | SZ_32M);
		break;
	case SZ_64K:
		cfg->pgsize_bitmap &= (SZ_64K | SZ_512M);
		break;
	default:
		cfg->pgsize_bitmap = 0;
	}
}

static struct arm_lpae_io_pgtable *
ipu_iommu_pgtbl_alloc_pgtable(struct io_pgtable_cfg *cfg)
{
	unsigned long va_bits, pgd_bits;
	struct arm_lpae_io_pgtable *data;

	ipu_iommu_pgtbl_restrict_pgsizes(cfg);

	if (!(cfg->pgsize_bitmap & (SZ_4K | SZ_16K | SZ_64K)))
		return NULL;

	if (cfg->ias > ARM_LPAE_MAX_ADDR_BITS)
		return NULL;

	if (cfg->oas > ARM_LPAE_MAX_ADDR_BITS)
		return NULL;

	if (cfg->iommu_dev->dma_pfn_offset) {
		dev_err(cfg->iommu_dev,
			"Cannot accommodate DMA offset for IOMMU page tables\n");
		return NULL;
	}

	data = kmalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return NULL;

	data->pg_shift = __ffs(cfg->pgsize_bitmap);
	data->bits_per_level = data->pg_shift - ilog2(
		sizeof(arm_lpae_iopte));

	va_bits = cfg->ias - data->pg_shift;
	data->levels = DIV_ROUND_UP(va_bits, data->bits_per_level);

	/* Calculate the actual size of our pgd (without concatenation) */
	pgd_bits = va_bits - (data->bits_per_level * (data->levels - 1));
	data->pgd_bits = pgd_bits;
	data->pgd_size = 1UL << (pgd_bits + ilog2(
		sizeof(arm_lpae_iopte)));

	data->iop.ops = (struct io_pgtable_ops) {
		.is_iova_coherent = ipu_iommu_pgtbl_is_iova_coherent,
	};

	return data;
}

static struct io_pgtable *
ipu_iommu_page_table_alloc_pgtable(struct io_pgtable_cfg *cfg, void *cookie,
	struct ipu_iommu_page_table *pg_table)
{
	u64 reg, sl;
	struct arm_lpae_io_pgtable *data;

	/* The NS quirk doesn't apply at stage 2 */
	if (!(cfg->quirks & IO_PGTABLE_QUIRK_ARM_NS))
		return NULL;

	data = ipu_iommu_pgtbl_alloc_pgtable(cfg);
	if (!data)
		return NULL;

	/*
	 * Concatenate PGDs at level 1 if possible in order to reduce
	 * the depth of the stage-2 walk.
	 */
	if (data->levels == ARM_LPAE_MAX_LEVELS) {
		unsigned long pgd_pages;

		pgd_pages = data->pgd_size >> ilog2(
			sizeof(arm_lpae_iopte));
		if (pgd_pages <= ARM_LPAE_S2_MAX_CONCAT_PAGES) {
			data->pgd_size = pgd_pages << data->pg_shift;
			data->levels--;
		}
	}

	/* VTCR */
	reg = ARM_64_LPAE_S2_TCR_RES1 |
	     (ARM_LPAE_TCR_SH_IS << ARM_LPAE_TCR_SH0_SHIFT) |
	     (ARM_LPAE_TCR_RGN_WBWA << ARM_LPAE_TCR_IRGN0_SHIFT) |
	     (ARM_LPAE_TCR_RGN_WBWA << ARM_LPAE_TCR_ORGN0_SHIFT);

	sl = ARM_LPAE_START_LVL(data);

	switch (ARM_LPAE_GRANULE(data)) {
	case SZ_4K:
		reg |= ARM_LPAE_TCR_TG0_4K;
		sl++; /* SL0 format is different for 4K granule size */
		break;
	case SZ_16K:
		reg |= ARM_LPAE_TCR_TG0_16K;
		break;
	case SZ_64K:
		reg |= ARM_LPAE_TCR_TG0_64K;
		break;
	}

	switch (cfg->oas) {
	case 32:
		reg |= (ARM_LPAE_TCR_PS_32_BIT << ARM_LPAE_TCR_PS_SHIFT);
		break;
	case 36:
		reg |= (ARM_LPAE_TCR_PS_36_BIT << ARM_LPAE_TCR_PS_SHIFT);
		break;
	case 40:
		reg |= (ARM_LPAE_TCR_PS_40_BIT << ARM_LPAE_TCR_PS_SHIFT);
		break;
	case 42:
		reg |= (ARM_LPAE_TCR_PS_42_BIT << ARM_LPAE_TCR_PS_SHIFT);
		break;
	case 44:
		reg |= (ARM_LPAE_TCR_PS_44_BIT << ARM_LPAE_TCR_PS_SHIFT);
		break;
	case 48:
		reg |= (ARM_LPAE_TCR_PS_48_BIT << ARM_LPAE_TCR_PS_SHIFT);
		break;
	default:
		goto out_free_data;
	}

	reg |= (64ULL - cfg->ias) << ARM_LPAE_TCR_T0SZ_SHIFT;
	reg |= (~sl & ARM_LPAE_TCR_SL0_MASK) << ARM_LPAE_TCR_SL0_SHIFT;
	cfg->arm_lpae_s2_cfg.vtcr = reg;

	/* Allocate pgd pages */
	pg_table->shadow_table = __ipu_iommu_pgtable_alloc_pages(
		pg_table,
		data->pgd_size,
		GFP_KERNEL, cfg, cookie, true /*alloc_shadow*/,
		true /* reset mem */);

	COLLECT_SAMPLE(IPU_IOMMU_PT_CNT_LVL1_PAGES, data->pgd_size);

	if (!pg_table->shadow_table.page_table_entry)
		goto out_free_data;

	pg_table->arm_page_table_dma_address =
		ab_dram_get_dma_buf_paddr(
			pg_table->shadow_table.ab_dram_dma_buf);
	data->pgd = (void *)pg_table->arm_page_table_dma_address;

	if (!data->pgd)
		goto out_free_data;

	/* Ensure the empty pgd is visible before any actual TTBR write */
	wmb();

	/* VTTBR */
	cfg->arm_lpae_s2_cfg.vttbr = (u64)pg_table->arm_page_table_dma_address;
	return &data->iop;

out_free_data:
	kfree(data);
	return NULL;
}

struct io_pgtable_ops *ipu_iommu_page_table_alloc_ops(
	struct io_pgtable_cfg *cfg,
	void *cookie)
{
	arm_lpae_iopte *ptep;
	struct arm_lpae_io_pgtable *data;
	static struct io_pgtable *iop;

	struct ipu_iommu_page_table *pg_table =
		kzalloc(sizeof(*pg_table), GFP_KERNEL);
	if (pg_table == NULL) {
		dev_err(cfg->iommu_dev,
			"%s failed to allocate page table\n", __func__);
		return NULL;
	}

	/* iatu */
	pg_table->dma_dev = cfg->iommu_dev->parent->parent;
	pg_table->iatu = abc_pcie_get_inbound_iatu(pg_table->dma_dev,
		pg_table->dma_dev /* owner */);
	if (pg_table->iatu < 0) {
		dev_err(cfg->iommu_dev,
			"%s failed to acquire iatu\n", __func__);
		goto free_pg_table;
	}
	memset(&pg_table->mapping, 0, sizeof(struct bar_mapping));
	pg_table->mapping.iatu = pg_table->iatu;

	/* create arm l1 page table */
	iop = ipu_iommu_page_table_alloc_pgtable(cfg, cookie, pg_table);
	if (!iop) {
		dev_err(cfg->iommu_dev,
			"%s pgtable_ops - alloc failed\n", __func__);
		goto release_iatu;
	}
	iop->fmt	= ARM_64_LPAE_S2;
	iop->cookie	= cookie;
	iop->cfg	= *cfg;

	pg_table->pgtbl_ops = &iop->ops;
	data = io_pgtable_ops_to_data(pg_table->pgtbl_ops);
	ptep = data->pgd;
	if (cfg->arm_lpae_s2_cfg.vttbr == 0) {
		dev_err(cfg->iommu_dev,
			"%s pgtable_ops - dma mapping failed\n",
			__func__);
		goto free_page_table;
	}

	pg_table->ops = (struct io_pgtable_ops) {
		.map            = ipu_iommu_pgtable_map,
		.map_sg			= ipu_iommu_pgtable_map_sg,
		.unmap          = ipu_iommu_pgtable_unmap,
		.iova_to_phys   = ipu_iommu_pgtable_iova_to_phys,
		.iova_to_pte    = ipu_iommu_pgtable_iova_get_pte
	};

	dev_dbg(cfg->iommu_dev,
		"%s iommu page table was created in device memory!",
		__func__);
	return &pg_table->ops;

free_page_table:
	__ipu_iommu_free_pgtable_top(pg_table, data);

release_iatu:
	abc_pcie_put_inbound_iatu(pg_table->dma_dev,
			pg_table->dma_dev /* owner */,
			pg_table->mapping.iatu);

free_pg_table:
	kfree(pg_table);

	return NULL;
}

dma_addr_t ipu_iommu_pg_table_get_dma_address(struct io_pgtable_ops *ops)
{
	struct ipu_iommu_page_table *pg_table;

	pg_table = io_pgtable_ops_to_ipu_page_table(ops);
	return pg_table->arm_page_table_dma_address;
}

void ipu_iommu_pgtable_mem_up(struct io_pgtable_ops *ops)
{
	struct ipu_iommu_page_table *pg_table;
	struct arm_lpae_io_pgtable *data;

	pg_table = io_pgtable_ops_to_ipu_page_table(ops);
	if (pg_table->ab_dram_up)
		return;

	data = ipu_iommu_pgtable_to_arm_data(pg_table);
	pg_table->ab_dram_up = true;

	if (!pg_table->ab_dram_suspended && __ipu_iommu_load_pgtable(pg_table,
			ARM_LPAE_START_LVL(data), &pg_table->shadow_table, 1)) {
		dev_err(pg_table->dma_dev, "%s error loading page table\n",
			__func__);
		pg_table->ab_dram_up = false;
	}
	pg_table->ab_dram_suspended = false;
}
void ipu_iommu_pgtable_mem_down(struct io_pgtable_ops *ops, bool suspend)
{
	struct ipu_iommu_page_table *pg_table;

	pg_table = io_pgtable_ops_to_ipu_page_table(ops);
	if (pg_table->ab_dram_up || pg_table->ab_dram_suspended)
		pg_table->ab_dram_suspended = suspend;
	pg_table->ab_dram_up = false;
}

void ipu_iommu_pgtable_update_device(struct io_pgtable_ops *ops,
		struct device *dev, void *cookie)
{
	struct ipu_iommu_page_table *pg_table;
	struct arm_lpae_io_pgtable *data;
	struct io_pgtable_cfg *cfg;

	pg_table = io_pgtable_ops_to_ipu_page_table(ops);
	data = ipu_iommu_pgtable_to_arm_data(pg_table);
	cfg = &data->iop.cfg;
	cfg->iommu_dev = dev;
	data->iop.cookie = cookie;
	pg_table->dma_dev = cfg->iommu_dev->parent->parent;
}

#else /* IS_ENABLED(CONFIG_IPU_IOMMU_PAGE_TABLE_ON_AB) */

struct io_pgtable_ops *ipu_iommu_page_table_alloc_ops(
	struct io_pgtable_cfg *cfg,
	void *cookie)
{
	struct io_pgtable_ops *ops;
	arm_lpae_iopte *ptep;
	struct arm_lpae_io_pgtable *data;

	/* create arm l1 page table */
	ops = alloc_io_pgtable_ops(ARM_64_LPAE_S2, cfg,
			cookie);

	data = io_pgtable_ops_to_data(ops);
	ptep = data->pgd;

	/* dma map l1 page table */
	cfg->arm_lpae_s2_cfg.vttbr = abc_dma_map_single(
		ptep, data->pgd_size,
		DMA_TO_DEVICE);
	if (cfg->arm_lpae_s2_cfg.vttbr == 0) {
		dev_err(cfg->iommu_dev, "%s pgtable_ops - dma mapping failed\n",
			__func__);
		goto free_ops_data;
	}

	dev_dbg(cfg->iommu_dev,
		"%s iommu page table was created in host memory!",
		__func__);
	return ops;

free_ops_data:
	kfree(data);

	return NULL;
}

dma_addr_t ipu_iommu_pg_table_get_dma_address(struct io_pgtable_ops *ops)
{
	struct arm_lpae_io_pgtable *data = io_pgtable_ops_to_data(ops);
	struct io_pgtable_cfg *cfg = &data->iop.cfg;

	return cfg->arm_lpae_s2_cfg.vttbr;
}

void ipu_iommu_free_pgtable(struct io_pgtable_ops *ops)
{
	struct arm_lpae_io_pgtable *data;
	struct io_pgtable_cfg *cfg;

	data = io_pgtable_ops_to_data(ops);
	cfg = &data->iop.cfg;
	/* dma unmap */
	abc_dma_unmap_single(cfg->arm_lpae_s2_cfg.vttbr, data->pgd_size,
			DMA_TO_DEVICE);

	free_io_pgtable_ops(ops);
}

void ipu_iommu_pgtable_update_device(struct io_pgtable_ops *ops,
		struct device *dev, void *cookie)
{
	struct arm_lpae_io_pgtable *data;
	struct io_pgtable_cfg *cfg;

	data = io_pgtable_ops_to_data(ops);
	cfg = &data->iop.cfg;
	cfg->iommu_dev = dev;
	data->iop.cookie = cookie;
}
ssize_t ipu_iommu_pgtable_report_status(size_t max_size, char *buf)
{
	return scnprintf(buf, max_size, "iommu status not supported\n");
}
/* no need to take any action */
void ipu_iommu_pgtable_mem_up(struct io_pgtable_ops *ops) {}
void ipu_iommu_pgtable_mem_down(struct io_pgtable_ops *ops, bool suspend) {}

#endif /* IS_ENABLED(CONFIG_IPU_IOMMU_PAGE_TABLE_ON_AB) */
