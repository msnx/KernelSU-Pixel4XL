/*
 * IOMMU page table for the Paintbox programmable IPU library
 *
 * Copyright 2019 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __IPU_IOMMU_PAGE_TABLE__
#define __IPU_IOMMU_PAGE_TABLE__

/* create the page table */
struct io_pgtable_ops *ipu_iommu_page_table_alloc_ops(
	struct io_pgtable_cfg *cfg,
	void *cookie);

/* free that page table */
void ipu_iommu_free_pgtable(struct io_pgtable_ops *ops);

/* get the dma address of the tree root */
dma_addr_t ipu_iommu_pg_table_get_dma_address(struct io_pgtable_ops *ops);

/* update the page table when the holding device changes */
void ipu_iommu_pgtable_update_device(struct io_pgtable_ops *ops,
		struct device *dev, void *cookie);

/* prints the current page table status to the input buf
 * return: size used
 */
ssize_t ipu_iommu_pgtable_report_status(size_t max_size, char *buf);

/* notify the page table if the memory is active or not */
void ipu_iommu_pgtable_mem_up(struct io_pgtable_ops *ops);
void ipu_iommu_pgtable_mem_down(struct io_pgtable_ops *ops, bool suspend);

#endif /* __IPU_IOMMU_PAGE_TABLE__ */
