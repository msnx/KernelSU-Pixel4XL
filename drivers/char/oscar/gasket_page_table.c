// SPDX-License-Identifier: GPL-2.0
/*
 * Implementation of Gasket page table support.
 *
 * Copyright (C) 2018 Google, Inc.
 */

/*
 * Implementation of Gasket page table support.
 *
 * This file assumes 4kB pages throughout; can be factored out when necessary.
 *
 * There is a configurable number of page table entries, as well as a
 * configurable bit index for the extended address flag. Both of these are
 * specified in gasket_page_table_init through the page_table_config parameter.
 *
 * The following example assumes:
 *   page_table_config->total_entries = 8192
 *   page_table_config->extended_bit = 63
 *
 * Address format:
 * Simple addresses - those whose containing pages are directly placed in the
 * device's address translation registers - are laid out as:
 * [ 63 - 25: 0 | 24 - 12: page index | 11 - 0: page offset ]
 * page index:  The index of the containing page in the device's address
 *              translation registers.
 * page offset: The index of the address into the containing page.
 *
 * Extended address - those whose containing pages are contained in a second-
 * level page table whose address is present in the device's address translation
 * registers - are laid out as:
 * [ 63: flag | 62 - 34: 0 | 33 - 21: dev/level 0 index |
 *   20 - 12: host/level 1 index | 11 - 0: page offset ]
 * flag:        Marker indicating that this is an extended address. Always 1.
 * dev index:   The index of the first-level page in the device's extended
 *              address translation registers.
 * host index:  The index of the containing page in the [host-resident] second-
 *              level page table.
 * page offset: The index of the address into the containing [second-level]
 *              page.
 */
#include "gasket_page_table.h"

#include <linux/device.h>
#include <linux/file.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pagemap.h>
#include <linux/seq_file.h>
#include <linux/vmalloc.h>

#include "gasket_constants.h"
#include "gasket_core.h"

/* Constants & utility macros */
/* The number of pages that can be mapped into each second-level page table. */
#define GASKET_PAGES_PER_SUBTABLE 512

/* The starting position of the page index in a simple virtual address. */
#define GASKET_SIMPLE_PAGE_SHIFT 12

/* Flag indicating that a [device] slot is valid for use. */
#define GASKET_VALID_SLOT_FLAG 1

/*
 * The starting position of the level 0 page index (i.e., the entry in the
 * device's extended address registers) in an extended address.
 * Also can be thought of as (log2(PAGE_SIZE) + log2(PAGES_PER_SUBTABLE)),
 * or (12 + 9).
 */
#define GASKET_EXTENDED_LVL0_SHIFT 21

/*
 * Number of first level pages that Gasket chips support. Equivalent to
 * log2(NUM_LVL0_PAGE_TABLES)
 *
 * At a maximum, allowing for a 34 bits address space (or 16GB)
 *   = GASKET_EXTENDED_LVL0_WIDTH + (log2(PAGE_SIZE) + log2(PAGES_PER_SUBTABLE)
 * or, = 13 + 9 + 12
 */
#define GASKET_EXTENDED_LVL0_WIDTH 13

/*
 * The starting position of the level 1 page index (i.e., the entry in the
 * host second-level/sub- table) in an extended address.
 */
#define GASKET_EXTENDED_LVL1_SHIFT 12

/*
 * Utilities for accessing flags bitfields.
 */
#define MASK(field)            (((1u << field##_WIDTH) - 1) << field##_SHIFT)
#define GET(field, flags)      (((flags) & MASK(field)) >> field##_SHIFT)
#define SET(field, flags, val) (((flags) & ~MASK(field)) | ((val) << field##_SHIFT))

#define FLAGS_STATUS_SHIFT 0
#define FLAGS_STATUS_WIDTH 1

#define FLAGS_DMA_DIRECTION_SHIFT 1
#define FLAGS_DMA_DIRECTION_WIDTH 2

/* If set, extended level-1 subtable is device-managed */
#define FLAGS_DEV_SUBTABLE_SHIFT 31
#define FLAGS_DEV_SUBTABLE_WIDTH 1

/* If set, device-managed subtable is already initialized */
#define FLAGS_DEV_SUBTABLE_INITED_SHIFT 30
#define FLAGS_DEV_SUBTABLE_INITED_WIDTH 1

/* Type declarations */
/* Valid states for a struct gasket_page_table_entry. */
enum pte_status {
	PTE_FREE,
	PTE_INUSE,
};

/*
 * Mapping metadata for a single page.
 *
 * In this file, host-side page table entries are referred to as that (or PTEs).
 * Where device vs. host entries are differentiated, device-side or -visible
 * entries are called "slots". A slot may be either an entry in the device's
 * address translation table registers or an entry in a second-level page
 * table ("subtable").
 *
 * The full data in this structure is visible on the host [of course]. Only
 * the address contained in dma_addr is communicated to the device; that points
 * to the actual page mapped and described by this structure.
 */
struct gasket_page_table_entry {
	/*
	 * Internal structure matches gasket_page_table_ioctl_flags.flags.
	 * NOTE: All fields should have a default value of 0. This ensures that
	 * the kernel will be backwards compatible with old drivers.
	 */
	u32 flags;

	/*
	 * Index for alignment into host vaddrs.
	 * When a user specifies a host address for a mapping, that address may
	 * not be page-aligned. Offset is the index into the containing page of
	 * the host address (i.e., host_vaddr & (PAGE_SIZE - 1)).
	 * This is necessary for translating between user-specified addresses
	 * and page-aligned addresses.
	 */
	int offset;

	/* Address of the page in DMA space. */
	dma_addr_t dma_addr;

	/* Linux page descriptor for the page described by this structure. */
	struct page *page;

	/*
	 * If this is an extended and first-level entry, sublevel points
	 * to the second-level entries underneath this entry.
	 */
	struct gasket_page_table_entry *sublevel;

	/* Info specific to device-managed subtables */
	struct gasket_dev_subtable dev_subtable;
};

/*
 * Maintains virtual to physical address mapping for a coherent page that is
 * allocated by this module for a given device.
 * Note that coherent pages mappings virt mapping cannot be tracked by the
 * Linux kernel, and coherent pages don't have a struct page associated,
 * hence Linux kernel cannot perform a get_user_page_xx() on a phys address
 * that was allocated coherent.
 * This structure trivially implements this mechanism.
 */
struct gasket_coherent_page_entry {
	/* Phys address, dma'able by the owner device */
	dma_addr_t paddr;

	/* Kernel virtual address */
	u64 user_virt;

	/* User virtual address that was mapped by the mmap kernel subsystem */
	u64 kernel_virt;

	/*
	 * Whether this page has been mapped into a user land process virtual
	 * space
	 */
	u32 in_use;
};

/*
 * [Host-side] page table descriptor.
 *
 * This structure tracks the metadata necessary to manage both simple and
 * extended page tables.
 */
struct gasket_page_table {
	/* The config used to create this page table. */
	struct gasket_page_table_config config;

	/* The number of simple (single-level) entries in the page table. */
	uint num_simple_entries;

	/* The number of extended (two-level) entries in the page table. */
	uint num_extended_entries;

	/* Array of [host-side] page table entries. */
	struct gasket_page_table_entry *entries;

	/* Number of actively mapped kernel pages in this table. */
	uint num_active_pages;

	/* Device register: base of/first slot in the page table. */
	u64 __iomem *base_slot;

	/* Device register: holds the offset indicating the start of the
	 * extended address region of the device's address translation table.
	 */
	u64 __iomem *extended_offset_reg;

	struct gasket_dev *gasket_dev;

	/* Location of the extended address bit for this Gasket device. */
	u64 extended_flag;

	/* Mutex to protect page table internals. */
	struct mutex mutex;

	/* Number of coherent pages accessible thru by this page table */
	int num_coherent_pages;

	/*
	 * List of coherent memory (physical) allocated for a device.
	 *
	 * This structure also remembers the user virtual mapping, this is
	 * hacky, but we need to do this because the kernel doesn't keep track
	 * of the user coherent pages (pfn pages), and virt to coherent page
	 * mapping.
	 * TODO: use find_vma() APIs to convert host address to vm_area, to
	 * dma_addr_t instead of storing user virtu address in
	 * gasket_coherent_page_entry
	 *
	 * Note that the user virtual mapping is created by the driver, in
	 * gasket_mmap function, so user_virt belongs in the driver anyhow.
	 */
	struct gasket_coherent_page_entry *coherent_pages;
};

/* See gasket_page_table.h for description. */
int gasket_page_table_init(struct gasket_page_table **ppg_tbl,
			   const struct gasket_bar_data *bar_data,
			   const struct gasket_page_table_config *page_table_config,
			   struct gasket_dev *gasket_dev)
{
	ulong bytes;
	struct gasket_page_table *pg_tbl;
	ulong total_entries = page_table_config->total_entries;

	/*
	 * TODO: Verify config->total_entries against value read from the
	 * hardware register that contains the page table size.
	 */
	if (total_entries == ULONG_MAX) {
		dev_dbg(gasket_dev->dev, "Error reading page table size. "
			"Initializing page table with size 0\n");
		total_entries = 0;
	}

	dev_dbg(gasket_dev->dev,
		"Attempting to initialize page table of size 0x%lx\n",
		total_entries);

	dev_dbg(gasket_dev->dev,
		"Table has base reg 0x%x, extended offset reg 0x%x\n",
		page_table_config->base_reg,
		page_table_config->extended_reg);

	*ppg_tbl = kzalloc(sizeof(**ppg_tbl), GFP_KERNEL);
	if (!*ppg_tbl) {
		dev_dbg(gasket_dev->dev, "No memory for page table\n");
		return -ENOMEM;
	}

	pg_tbl = *ppg_tbl;
	bytes = total_entries * sizeof(struct gasket_page_table_entry);
	if (bytes != 0) {
		pg_tbl->entries = vzalloc(bytes);
		if (!pg_tbl->entries) {
			dev_dbg(gasket_dev->dev,
				"No memory for address translation metadata\n");
			kfree(pg_tbl);
			*ppg_tbl = NULL;
			return -ENOMEM;
		}
	}

	mutex_init(&pg_tbl->mutex);
	memcpy(&pg_tbl->config, page_table_config, sizeof(*page_table_config));
	if (pg_tbl->config.mode == GASKET_PAGE_TABLE_MODE_NORMAL ||
	    pg_tbl->config.mode == GASKET_PAGE_TABLE_MODE_SIMPLE) {
		pg_tbl->num_simple_entries = total_entries;
		pg_tbl->num_extended_entries = 0;
		pg_tbl->extended_flag = 1ull << page_table_config->extended_bit;
	} else {
		pg_tbl->num_simple_entries = 0;
		pg_tbl->num_extended_entries = total_entries;
		pg_tbl->extended_flag = 0;
	}
	pg_tbl->num_active_pages = 0;
	pg_tbl->base_slot =
		(u64 __iomem *)&bar_data->virt_base[page_table_config->base_reg];
	pg_tbl->extended_offset_reg =
		(u64 __iomem *)&bar_data->virt_base[page_table_config->extended_reg];
	pg_tbl->gasket_dev = gasket_dev;
	dev_dbg(gasket_dev->dev, "Page table initialized successfully\n");
	return 0;
}

/*
 * Check if a range of PTEs is free.
 * The page table mutex must be held by the caller.
 */
static bool gasket_is_pte_range_free(struct gasket_page_table_entry *ptes,
				     uint num_entries)
{
	int i;

	for (i = 0; i < num_entries; i++) {
		if (GET(FLAGS_STATUS, ptes[i].flags) != PTE_FREE)
			return false;
	}

	return true;
}

/*
 * Free a second level page [sub]table.
 * The page table mutex must be held before this call.
 */
static void gasket_free_extended_subtable(struct gasket_page_table *pg_tbl,
					  struct gasket_page_table_entry *pte)
{
	/* Release the page table from the driver */
	pte->flags = SET(FLAGS_STATUS, pte->flags, PTE_FREE);

	vfree(pte->sublevel);

	if (GET(FLAGS_DEV_SUBTABLE, pte->flags)) {
		gasket_subtable_manage_cb_t subtbl_manage_cb =
			gasket_get_subtable_manage_cb(pg_tbl->gasket_dev);

		if (subtbl_manage_cb)
			subtbl_manage_cb(pg_tbl->gasket_dev,
					 GASKET_DEV_SUBTABLE_DEALLOC,
					 &pte->dev_subtable);
	} else {
		if (pte->dma_addr)
			dma_unmap_page(pg_tbl->gasket_dev->dma_dev,
				       pte->dma_addr, PAGE_SIZE, DMA_TO_DEVICE);

		if (pte->page)
			free_page((ulong)page_address(pte->page));
	}

	memset(pte, 0, sizeof(struct gasket_page_table_entry));
}

/*
 * Actually perform collection.
 * The page table mutex must be held by the caller.
 */
static void
gasket_page_table_garbage_collect_nolock(struct gasket_page_table *pg_tbl)
{
	struct gasket_page_table_entry *pte;
	u64 __iomem *slot;

	gasket_update_hw_status(pg_tbl->gasket_dev);

	for (pte = pg_tbl->entries + pg_tbl->num_simple_entries,
	     slot = pg_tbl->base_slot + pg_tbl->num_simple_entries;
	     pte < pg_tbl->entries + pg_tbl->config.total_entries;
	     pte++, slot++) {
		if (GET(FLAGS_STATUS, pte->flags) == PTE_INUSE &&
		    gasket_is_pte_range_free(pte->sublevel,
					     GASKET_PAGES_PER_SUBTABLE)) {

			if (pg_tbl->gasket_dev->status != GASKET_STATUS_DEAD)
				/* Release the page table from the device */
				writeq(0, slot);

			gasket_free_extended_subtable(pg_tbl, pte);
		}
	}
}

/* See gasket_page_table.h for description. */
void gasket_page_table_garbage_collect(struct gasket_page_table *pg_tbl)
{
	mutex_lock(&pg_tbl->mutex);
	gasket_page_table_garbage_collect_nolock(pg_tbl);
	mutex_unlock(&pg_tbl->mutex);
}

/* See gasket_page_table.h for description. */
void gasket_page_table_cleanup(struct gasket_page_table *pg_tbl)
{
	/* Deallocate free second-level tables. */
	gasket_page_table_garbage_collect(pg_tbl);

	vfree(pg_tbl->entries);
	pg_tbl->entries = NULL;
	kfree(pg_tbl);
}

/* See gasket_page_table.h for description. */
int gasket_page_table_partition(struct gasket_page_table *pg_tbl,
				uint num_simple_entries)
{
	int i, start;

	mutex_lock(&pg_tbl->mutex);
	if (num_simple_entries > pg_tbl->config.total_entries) {
		mutex_unlock(&pg_tbl->mutex);
		return -EINVAL;
	}

	gasket_page_table_garbage_collect_nolock(pg_tbl);

	start = min(pg_tbl->num_simple_entries, num_simple_entries);

	for (i = start; i < pg_tbl->config.total_entries; i++) {
		if (GET(FLAGS_STATUS, pg_tbl->entries[i].flags) != PTE_FREE) {
			dev_err(pg_tbl->gasket_dev->dev,
				"entry %d is not free\n", i);
			mutex_unlock(&pg_tbl->mutex);
			return -EBUSY;
		}
	}

	pg_tbl->num_simple_entries = num_simple_entries;
	pg_tbl->num_extended_entries =
		pg_tbl->config.total_entries - num_simple_entries;
	writeq(num_simple_entries, pg_tbl->extended_offset_reg);

	mutex_unlock(&pg_tbl->mutex);
	return 0;
}
EXPORT_SYMBOL(gasket_page_table_partition);

/*
 * Return whether a host buffer was mapped as coherent memory.
 *
 * A Gasket page_table currently support one contiguous dma range, mapped to one
 * contiguous virtual memory range. Check if the host_addr is within that range.
 */
static int is_coherent(struct gasket_page_table *pg_tbl, ulong host_addr)
{
	u64 min, max;

	/* whether the host address is within user virt range */
	if (!pg_tbl->coherent_pages)
		return 0;

	min = (u64)pg_tbl->coherent_pages[0].user_virt;
	max = min + PAGE_SIZE * pg_tbl->num_coherent_pages;

	return min <= host_addr && host_addr < max;
}

/* Does dma address fall within the coherent memory range? */
static int is_coherent_dma(struct gasket_page_table *pg_tbl, ulong dma_addr)
{
	u64 min, max;

	if (!pg_tbl->coherent_pages)
		return 0;

	min = (u64)pg_tbl->coherent_pages[0].paddr;
	max = min + PAGE_SIZE * pg_tbl->num_coherent_pages;

	return min <= dma_addr && dma_addr < max;
}

/*
 * Get and map last level page table buffers.
 *
 * slots is the location(s) to write device-mapped page address. If this is a
 * simple mapping, these will be address translation registers. If this is
 * an extended mapping, these will be within a second-level page table
 * allocated by the host and so must have their __iomem attribute casted away.
 */
static int gasket_perform_mapping(struct gasket_page_table *pg_tbl,
				  struct gasket_page_table_entry *ptes,
				  u64 __iomem *slots, ulong host_addr,
				  uint num_pages, u32 flags,
				  int is_simple_mapping, bool is_user_addr,
				  bool dev_managed_subtable)
{
	int ret;
	ulong offset;
	struct page **pages = NULL;
	dma_addr_t dma_addr;
	uintptr_t page_addr;
	int i;
	int user_pages_mapped = 0;

	if (is_user_addr) {
		pages = kcalloc(num_pages, sizeof(struct page *), GFP_KERNEL);
		if (!pages)
			return -ENOMEM;
		ret = get_user_pages_fast(host_addr & PAGE_MASK, num_pages, 1,
					  pages);
		if (ret <= 0) {
			dev_dbg(pg_tbl->gasket_dev->dev,
				"get user pages failed addr=%pK "
				"npages=%u [ret=%d]\n",
				(void *)host_addr, num_pages, ret);
			kfree(pages);
			return ret ? ret : -ENOMEM;
		}
		if (ret < num_pages) {
			dev_dbg(pg_tbl->gasket_dev->dev,
				"get user pages partial failed addr=%pK "
				"npages=%u pinned=%d\n",
				(void *)host_addr, num_pages, ret);
			num_pages = ret;
			ret = -ENOMEM;
			goto unpin;
		}
	}

	for (i = 0; i < num_pages; i++) {
		page_addr = host_addr + i * PAGE_SIZE;
		offset = page_addr & (PAGE_SIZE - 1);

		if (!is_user_addr) {
			ptes[i].page = NULL;
			ptes[i].offset = offset;
			ptes[i].dma_addr = page_addr - offset;

		} else {
			ptes[i].page = pages[i];
			ptes[i].offset = offset;

			/* Map the page into DMA space. */
			ptes[i].dma_addr =
				dma_map_page(pg_tbl->gasket_dev->dma_dev,
					     ptes[i].page, 0, PAGE_SIZE,
					     GET(FLAGS_DMA_DIRECTION, flags));
			if (dma_mapping_error(pg_tbl->gasket_dev->dma_dev,
					      ptes[i].dma_addr)) {
				dev_dbg(pg_tbl->gasket_dev->dma_dev,
					"dma mapping error i=%d page=%pK vaddr=%pK\n",
					i, ptes[i].page, (void *)page_addr);
				memset(&ptes[i], 0,
				       sizeof(struct gasket_page_table_entry));
				ret = -EINVAL;
				goto unpin;
			}

			++pg_tbl->num_active_pages;
			user_pages_mapped++;
		}

		/* Make the DMA-space address available to the device. */
		dma_addr = (ptes[i].dma_addr + offset) | GASKET_VALID_SLOT_FLAG;

		if (is_simple_mapping)
			writeq(dma_addr, &slots[i]);
		else
			((u64 __force *)slots)[i] = dma_addr;

		/* Set PTE flags equal to flags param with STATUS=PTE_INUSE. */
		ptes[i].flags = SET(FLAGS_STATUS, flags, PTE_INUSE);
	}

	kfree(pages);
	return 0;

 unpin:
	if (is_user_addr) {
		/*
		 * Release pages pinned here but not mapped; unmap will do the
		 * rest.
		 */
		for (i = user_pages_mapped; i < num_pages; i++)
			put_page(pages[i]);
		kfree(pages);
	}
	return ret;
}

/*
 * Return the index of the page for the address in the simple table.
 * Does not perform validity checking.
 */
static int gasket_simple_page_idx(struct gasket_page_table *pg_tbl,
				  ulong dev_addr)
{
	return (dev_addr >> GASKET_SIMPLE_PAGE_SHIFT) &
		(pg_tbl->config.total_entries - 1);
}

/*
 * Return the level 0 page index for the given address.
 * Does not perform validity checking.
 */
static ulong gasket_extended_lvl0_page_idx(struct gasket_page_table *pg_tbl,
					   ulong dev_addr)
{
	return (dev_addr >> GASKET_EXTENDED_LVL0_SHIFT) &
		(pg_tbl->config.total_entries - 1);
}

/*
 * Return the level 1 page index for the given address.
 * Does not perform validity checking.
 */
static ulong gasket_extended_lvl1_page_idx(struct gasket_page_table *pg_tbl,
					   ulong dev_addr)
{
	return (dev_addr >> GASKET_EXTENDED_LVL1_SHIFT) &
	       (GASKET_PAGES_PER_SUBTABLE - 1);
}

/*
 * Allocate page table entries in a simple table.
 * The page table mutex must be held by the caller.
 */
static int gasket_alloc_simple_entries(struct gasket_page_table *pg_tbl,
				       ulong dev_addr, uint num_pages)
{
	if (!gasket_is_pte_range_free(pg_tbl->entries +
				      gasket_simple_page_idx(pg_tbl, dev_addr),
				      num_pages))
		return -EBUSY;

	return 0;
}

/*
 * Release host state associated with a page table entry. Unmapping the
 * associated entry from the device happens separately, if the device is
 * reachable.
 *
 * The page table mutex must be held by the caller.
 */
static void gasket_release_entry(struct gasket_page_table *pg_tbl,
				 struct gasket_page_table_entry *pte)
{
	if (GET(FLAGS_STATUS, pte->flags) == PTE_FREE)
		return;

	if (pte->page) {
		dma_unmap_page(pg_tbl->gasket_dev->dma_dev, pte->dma_addr,
			       PAGE_SIZE,
			       GET(FLAGS_DMA_DIRECTION, pte->flags));

		if (!PageReserved(pte->page))
			SetPageDirty(pte->page);

		put_page(pte->page);
		--pg_tbl->num_active_pages;
	}

	/* and clear the PTE. */
	memset(pte, 0, sizeof(*pte));
}

/*
 * Release host state for a range of subtable entries.  This is actually only
 * called to release every entry in the subtable, but is called from functions
 * that operate on subranges, which potentially could be specified in the
 * future.  We do not currently do that.
 * The page table mutex must be held by the caller.
 */
static void gasket_release_subtable(struct gasket_page_table *pg_tbl,
				    struct gasket_page_table_entry *subtable,
				    uint slot_idx, uint len)
{
	int i;

	for (i = slot_idx; i < slot_idx + len; i++)
		gasket_release_entry(pg_tbl, &subtable[i]);
}

/*
 * Unmap and release mapped pages.
 * The page table mutex must be held by the caller.
 */
static void gasket_perform_unmapping(struct gasket_page_table *pg_tbl,
				     struct gasket_page_table_entry *ptes,
				     u64 __iomem *slots, uint num_pages,
				     int is_simple_mapping)
{
	int i;
	/*
	 * For each page table entry and corresponding entry in the device's
	 * address translation table:
	 */
	for (i = 0; i < num_pages; i++) {
		/* release the address from the device, */
		if (is_simple_mapping)
			writeq(0, &slots[i]);
		else
			((u64 __force *)slots)[i] = 0;

		gasket_release_entry(pg_tbl, &ptes[i]);
	}
}

/*
 * Unmap and release pages mapped to simple addresses.
 * The page table mutex must be held by the caller.
 */
static void gasket_unmap_simple_pages(struct gasket_page_table *pg_tbl,
				      ulong dev_addr, uint num_pages)
{
	uint slot = gasket_simple_page_idx(pg_tbl, dev_addr);

	gasket_perform_unmapping(pg_tbl, pg_tbl->entries + slot,
				 pg_tbl->base_slot + slot, num_pages, 1);
}

/* Get pointer to extended level-1 page table, mapped to CPU if needed. */
static u64 __iomem *get_subtable(struct gasket_page_table *pg_tbl,
				 struct gasket_page_table_entry *pte)
{
	u64 __iomem *slot_base = NULL;

	if (GET(FLAGS_DEV_SUBTABLE, pte->flags)) {
		gasket_subtable_manage_cb_t subtbl_manage_cb =
			gasket_get_subtable_manage_cb(pg_tbl->gasket_dev);

		if (subtbl_manage_cb)
			slot_base =
				subtbl_manage_cb(pg_tbl->gasket_dev,
						 GASKET_DEV_SUBTABLE_MAP_TO_CPU,
						 &pte->dev_subtable);
	} else {
		slot_base = (u64 __iomem *)(page_address(pte->page)
					    + pte->offset);
	}

	return slot_base;
}

/* Unmap/sync extended Level-1 page table. */
static void put_subtable(struct gasket_page_table *pg_tbl,
			 struct gasket_page_table_entry *pte,
			 uint lvl1_start_idx, uint len)
{
	gasket_subtable_manage_cb_t subtbl_manage_cb;

	if (!GET(FLAGS_DEV_SUBTABLE, pte->flags)) {
		/* Sync CPU updates to subtable in DRAM */
		dma_sync_single_for_device(pg_tbl->gasket_dev->dma_dev,
					   pte->dma_addr +
					   lvl1_start_idx * sizeof(u64),
					   len * sizeof(u64), DMA_TO_DEVICE);

		return;
	}

	subtbl_manage_cb = gasket_get_subtable_manage_cb(pg_tbl->gasket_dev);
	subtbl_manage_cb(pg_tbl->gasket_dev, GASKET_DEV_SUBTABLE_UNMAP_FROM_CPU,
			 &pte->dev_subtable);
}

/*
 * Finish updates to extended level-1 page table.  If first init, mark unmapped
 * entries invalid and update the level-0 MMU slot to point to this page table
 * (now that it's fully initialized).
 */
static void finish_subtable(struct gasket_page_table *pg_tbl,
			    struct gasket_page_table_entry *pte,
			    u64 __iomem *host_addr, uint lvl0_idx,
			    uint lvl1_start_idx, uint len)
{
	u64 __iomem *slot;
	ulong lvl1_next_idx = lvl1_start_idx + len;

	if (!GET(FLAGS_DEV_SUBTABLE, pte->flags) ||
	    GET(FLAGS_DEV_SUBTABLE_INITED, pte->flags)) {
		put_subtable(pg_tbl, pte, lvl1_start_idx, len);
		return;
	}

	/* Set uninitialized entries invalid */
	if (lvl1_start_idx)
		memset_io(host_addr, 0, lvl1_start_idx * 8);

	if (lvl1_next_idx < GASKET_PAGES_PER_SUBTABLE) {
		slot = host_addr + lvl1_next_idx;
		memset_io(slot, 0,
			  (GASKET_PAGES_PER_SUBTABLE - lvl1_next_idx) * 8);
	}

	pte->flags = SET(FLAGS_DEV_SUBTABLE_INITED, pte->flags, 1);
	put_subtable(pg_tbl, pte, 0, GASKET_PAGES_PER_SUBTABLE);
	slot = pg_tbl->base_slot + pg_tbl->num_simple_entries + lvl0_idx;
	/* write sub-level PTE address to the device page table slot */
	writeq(pte->dma_addr | GASKET_VALID_SLOT_FLAG, slot);
}

/* Update extended subtable on unmap. */
static void gasket_unmap_update_subtable(struct gasket_page_table *pg_tbl,
					 struct gasket_page_table_entry *pte,
					 uint slot_idx, uint len)
{
	u64 __iomem *slot_base;

	slot_base = get_subtable(pg_tbl, pte);
	if (!IS_ERR_OR_NULL(slot_base)) {
		gasket_perform_unmapping(pg_tbl,
					 pte->sublevel + slot_idx,
					 slot_base + slot_idx, len, 0);
		put_subtable(pg_tbl, pte, slot_idx, len);
	}
}

/*
 * Unmap and release buffers to extended addresses.
 * The page table mutex must be held by the caller.
 */
static void gasket_unmap_extended_pages(struct gasket_page_table *pg_tbl,
					ulong dev_addr, uint num_pages,
					bool update_subtables)
{
	uint slot_idx, remain, len;
	struct gasket_page_table_entry *pte;

	remain = num_pages;
	slot_idx = gasket_extended_lvl1_page_idx(pg_tbl, dev_addr);
	pte = pg_tbl->entries + pg_tbl->num_simple_entries +
	      gasket_extended_lvl0_page_idx(pg_tbl, dev_addr);

	while (remain > 0) {
		len = min(remain, GASKET_PAGES_PER_SUBTABLE - slot_idx);

		if (GET(FLAGS_STATUS, pte->flags) == PTE_INUSE) {
			if (update_subtables)
				gasket_unmap_update_subtable(pg_tbl, pte,
							     slot_idx, len);
			else
				gasket_release_subtable(pg_tbl, pte->sublevel,
							slot_idx, len);
		}

		remain -= len;
		slot_idx = 0;
		pte++;
	}
}

/* Evaluates to nonzero if the specified virtual address is simple. */
static inline bool gasket_addr_is_simple(struct gasket_page_table *pg_tbl,
					 ulong addr)
{
	return !((addr) & (pg_tbl)->extended_flag);
}

/*
 * Convert (simple, page, offset) into a device address.
 * Examples:
 * Simple page 0, offset 32:
 *  Input (1, 0, 32), Output 0x20
 * Simple page 1000, offset 511:
 *  Input (1, 1000, 511), Output 0x3E81FF
 * Extended page 0, offset 32:
 *  Input (0, 0, 32), Output 0x8000000020
 * Extended page 1000, offset 511:
 *  Input (0, 1000, 511), Output 0x8003E81FF
 */
static ulong gasket_components_to_dev_address(struct gasket_page_table *pg_tbl,
					      int is_simple, uint page_index,
					      uint offset)
{
	ulong dev_addr = (page_index << GASKET_SIMPLE_PAGE_SHIFT) | offset;

	return is_simple ? dev_addr : (pg_tbl->extended_flag | dev_addr);
}

/*
 * Validity checking for simple addresses.
 *
 * Verify that address translation commutes (from address to/from page + offset)
 * and that the requested page range starts and ends within the set of
 * currently-partitioned simple pages.
 */
static bool gasket_is_simple_dev_addr_bad(struct gasket_page_table *pg_tbl,
					  ulong dev_addr, uint num_pages)
{
	ulong page_offset = dev_addr & (PAGE_SIZE - 1);
	ulong page_index =
		(dev_addr / PAGE_SIZE) & (pg_tbl->config.total_entries - 1);

	if (gasket_components_to_dev_address(pg_tbl, 1, page_index,
					     page_offset) != dev_addr) {
		dev_err(pg_tbl->gasket_dev->dev, "address is invalid, 0x%lX\n",
			dev_addr);
		return true;
	}

	if (page_index >= pg_tbl->num_simple_entries) {
		dev_err(pg_tbl->gasket_dev->dev,
			"starting slot at %lu is too large, max is < %u\n",
			page_index, pg_tbl->num_simple_entries);
		return true;
	}

	if (page_index + num_pages > pg_tbl->num_simple_entries) {
		dev_err(pg_tbl->gasket_dev->dev,
			"ending slot at %lu is too large, max is <= %u\n",
			page_index + num_pages, pg_tbl->num_simple_entries);
		return true;
	}

	return false;
}

/*
 * Validity checking for extended addresses.
 *
 * Verify that address translation commutes (from address to/from page +
 * offset) and that the requested page range starts and ends within the set of
 * currently-partitioned extended pages.
 */
static bool gasket_is_extended_dev_addr_bad(struct gasket_page_table *pg_tbl,
					    ulong dev_addr, uint num_pages)
{
	/* Starting byte index of dev_addr into the first mapped page */
	ulong page_offset = dev_addr & (PAGE_SIZE - 1);
	ulong page_global_idx, page_lvl0_idx;
	ulong num_lvl0_pages;
	ulong addr;

	/* check if the device address is out of bound */
	addr = dev_addr & ~((pg_tbl)->extended_flag);
	if (addr >> (GASKET_EXTENDED_LVL0_WIDTH + GASKET_EXTENDED_LVL0_SHIFT)) {
		dev_err(pg_tbl->gasket_dev->dev,
			"device address out of bounds: 0x%lx\n", dev_addr);
		return true;
	}

	/* Find the starting sub-page index in the space of all sub-pages. */
	page_global_idx = (dev_addr / PAGE_SIZE) &
		(pg_tbl->config.total_entries * GASKET_PAGES_PER_SUBTABLE - 1);

	/* Find the starting level 0 index. */
	page_lvl0_idx = gasket_extended_lvl0_page_idx(pg_tbl, dev_addr);

	/* Get the count of affected level 0 pages. */
	num_lvl0_pages = (num_pages + GASKET_PAGES_PER_SUBTABLE - 1) /
		GASKET_PAGES_PER_SUBTABLE;

	if (gasket_components_to_dev_address(pg_tbl, 0, page_global_idx,
					     page_offset) != dev_addr) {
		dev_err(pg_tbl->gasket_dev->dev, "address is invalid: 0x%lx\n",
			dev_addr);
		return true;
	}

	if (page_lvl0_idx >= pg_tbl->num_extended_entries) {
		dev_err(pg_tbl->gasket_dev->dev,
			"starting level 0 slot at %lu is too large, max is < "
			"%u\n", page_lvl0_idx, pg_tbl->num_extended_entries);
		return true;
	}

	if (page_lvl0_idx + num_lvl0_pages > pg_tbl->num_extended_entries) {
		dev_err(pg_tbl->gasket_dev->dev,
			"ending level 0 slot at %lu is too large, max is <= %u\n",
			page_lvl0_idx + num_lvl0_pages,
			pg_tbl->num_extended_entries);
		return true;
	}

	return false;
}

/*
 * Non-locking entry to unmapping routines.
 * The page table mutex must be held by the caller.
 */
static void gasket_page_table_unmap_nolock(struct gasket_page_table *pg_tbl,
					   ulong dev_addr, uint num_pages)
{
	if (!num_pages)
		return;

	if (gasket_addr_is_simple(pg_tbl, dev_addr))
		gasket_unmap_simple_pages(pg_tbl, dev_addr, num_pages);
	else
		gasket_unmap_extended_pages(pg_tbl, dev_addr, num_pages, true);
}

/*
 * Allocate and map pages to simple addresses.
 * If there is an error, no pages are mapped.
 */
static int gasket_map_simple_pages(struct gasket_page_table *pg_tbl,
				   ulong host_addr, ulong dev_addr,
				   uint num_pages, u32 flags, bool is_user_addr)
{
	int ret;
	uint slot_idx = gasket_simple_page_idx(pg_tbl, dev_addr);

	ret = gasket_alloc_simple_entries(pg_tbl, dev_addr, num_pages);
	if (ret) {
		dev_err(pg_tbl->gasket_dev->dev,
			"page table slots %u (@ 0x%lx) to %u are not available\n",
			slot_idx, dev_addr, slot_idx + num_pages - 1);
		return ret;
	}

	ret = gasket_perform_mapping(pg_tbl, pg_tbl->entries + slot_idx,
				     pg_tbl->base_slot + slot_idx, host_addr,
				     num_pages, flags, 1, is_user_addr, false);

	if (ret) {
		gasket_page_table_unmap_nolock(pg_tbl, dev_addr, num_pages);
		dev_err(pg_tbl->gasket_dev->dev, "gasket_perform_mapping %d\n",
			ret);
	}
	return ret;
}

/*
 * Allocate and map an extended second level subtable in host memory.
 * The page table mutex must be held by the caller.
 */
static int gasket_alloc_subtable_host(
	 struct gasket_page_table *pg_tbl, struct gasket_page_table_entry *pte)
{
	ulong page_addr;

	/*
	 * GFP_DMA flag must be passed to architectures for which
	 * part of the memory range is not considered DMA'able.
	 */
	page_addr = get_zeroed_page(GFP_KERNEL | GFP_DMA);
	if (!page_addr)
		return -ENOMEM;
	pte->page = virt_to_page((void *)page_addr);

	/* Map the page into DMA space. */
	pte->dma_addr = dma_map_page(pg_tbl->gasket_dev->dma_dev, pte->page, 0,
				     PAGE_SIZE, DMA_TO_DEVICE);
	if (dma_mapping_error(pg_tbl->gasket_dev->dma_dev, pte->dma_addr)) {
		phys_addr_t page_physaddr;

		page_physaddr = page_to_phys(pte->page);
		dev_dbg(pg_tbl->gasket_dev->dma_dev,
			"%s: fail to map page [pfn %pK phys %pap]\n",
			__func__, (void *)page_to_pfn(pte->page),
			&page_physaddr);
		free_page(page_addr);
		pte->page = NULL;
		return -ENOMEM;
	}
	return 0;
}

/*
 * Allocate a second level page table.
 * The page table mutex must be held by the caller.
 */
static int gasket_alloc_extended_subtable(struct gasket_page_table *pg_tbl,
					  struct gasket_page_table_entry *pte,
					  u64 __iomem *slot,
					  bool dev_alloc_subtable)
{
	gasket_subtable_manage_cb_t subtbl_manage_cb = NULL;
	int ret;

	pte->sublevel = vzalloc(sizeof(struct gasket_page_table_entry) *
				GASKET_PAGES_PER_SUBTABLE);
	if (!pte->sublevel)
		return -ENOMEM;

	if (dev_alloc_subtable)
		subtbl_manage_cb =
			gasket_get_subtable_manage_cb(pg_tbl->gasket_dev);

	if (subtbl_manage_cb) {
		void *dma_addr;

		dma_addr = subtbl_manage_cb(pg_tbl->gasket_dev,
				       GASKET_DEV_SUBTABLE_ALLOC,
				       &pte->dev_subtable);
		if (!IS_ERR_OR_NULL(dma_addr)) {
			pte->flags = SET(FLAGS_DEV_SUBTABLE, pte->flags, 1);
			pte->dma_addr = (dma_addr_t)dma_addr;
		}
	}

	if (!GET(FLAGS_DEV_SUBTABLE, pte->flags)) {
		ret = gasket_alloc_subtable_host(pg_tbl, pte);
		if (ret)
			goto free_sublevel;

		/*
		 * Write sub-level PTE address to the device page table slot.
		 * For on-device PTEs, this will happen at finish_subtable()
		 * once the lazy init of all entries is complete and the
		 * entire subtable is valid.
		 */
		writeq(pte->dma_addr | GASKET_VALID_SLOT_FLAG, slot);
	}

	pte->flags = SET(FLAGS_STATUS, pte->flags, PTE_INUSE);
	return 0;

free_sublevel:
	vfree(pte->sublevel);
	memset(pte, 0, sizeof(struct gasket_page_table_entry));
	return -ENOMEM;
}

/*
 * Allocate slots in an extended page table.  Check to see if a range of page
 * table slots are available. If necessary, memory is allocated for second level
 * page tables.
 *
 * Note that memory for second level page tables is allocated as needed, but
 * that memory is only freed on the final close	of the device file, when the
 * page tables are repartitioned, or the the device is removed.  If there is an
 * error or if the full range of slots is not available, any memory
 * allocated for second level page tables remains allocated until final close,
 * repartition, or device removal.
 *
 * The page table mutex must be held by the caller.
 */
static int gasket_alloc_extended_entries(struct gasket_page_table *pg_tbl,
					 ulong dev_addr, uint num_entries,
					 bool dev_alloc_ptes)
{
	int ret = 0;
	uint remain, subtable_slot_idx, len;
	struct gasket_page_table_entry *pte;
	u64 __iomem *slot;

	remain = num_entries;
	subtable_slot_idx = gasket_extended_lvl1_page_idx(pg_tbl, dev_addr);
	pte = pg_tbl->entries + pg_tbl->num_simple_entries +
	      gasket_extended_lvl0_page_idx(pg_tbl, dev_addr);
	slot = pg_tbl->base_slot + pg_tbl->num_simple_entries +
	       gasket_extended_lvl0_page_idx(pg_tbl, dev_addr);

	while (remain > 0) {
		len = min(remain,
			  GASKET_PAGES_PER_SUBTABLE - subtable_slot_idx);

		if (GET(FLAGS_STATUS, pte->flags) == PTE_FREE) {
			ret = gasket_alloc_extended_subtable(pg_tbl, pte, slot,
							     dev_alloc_ptes);
			if (ret) {
				dev_err(pg_tbl->gasket_dev->dev,
					"no memory for extended addr subtable\n");
				return ret;
			}
		} else {
			if (!gasket_is_pte_range_free(pte->sublevel +
						      subtable_slot_idx, len))
				return -EBUSY;
		}

		remain -= len;
		subtable_slot_idx = 0;
		pte++;
		slot++;
	}

	return 0;
}

/*
 * gasket_map_extended_pages - Get and map buffers to extended addresses.
 * If there is an error, no pages are mapped.
 */
static int gasket_map_extended_pages(struct gasket_page_table *pg_tbl,
				     ulong host_addr, ulong dev_addr,
				     uint num_pages, u32 flags,
				     bool is_user_addr, bool dev_alloc_ptes)
{
	int ret;
	ulong dev_addr_end;
	uint remain, len;
	uint lvl0_idx = gasket_extended_lvl0_page_idx(pg_tbl, dev_addr);
	uint lvl1_idx = gasket_extended_lvl1_page_idx(pg_tbl, dev_addr);
	struct gasket_page_table_entry *pte;
	u64 __iomem *slot_base;

	ret = gasket_alloc_extended_entries(pg_tbl, dev_addr, num_pages,
					    dev_alloc_ptes);
	if (ret) {
		dev_addr_end = dev_addr + (num_pages / PAGE_SIZE) - 1;
		dev_err(pg_tbl->gasket_dev->dev,
			"page table slots (%lu,%lu) (@ 0x%lx) to (%lu,%lu) are "
			"not available\n",
			lvl0_idx, lvl1_idx, dev_addr,
			gasket_extended_lvl0_page_idx(pg_tbl, dev_addr_end),
			gasket_extended_lvl1_page_idx(pg_tbl, dev_addr_end));
		return ret;
	}

	remain = num_pages;
	pte = pg_tbl->entries + pg_tbl->num_simple_entries + lvl0_idx;

	while (remain > 0) {
		len = min(remain, GASKET_PAGES_PER_SUBTABLE - lvl1_idx);
		slot_base = get_subtable(pg_tbl, pte);
		if (!IS_ERR_OR_NULL(slot_base)) {
			ret = gasket_perform_mapping(
				     pg_tbl, pte->sublevel + lvl1_idx,
				     slot_base + lvl1_idx, host_addr, len,
				     flags, 0, is_user_addr,
				     GET(FLAGS_DEV_SUBTABLE, pte->flags));
			finish_subtable(pg_tbl, pte, slot_base, lvl0_idx,
					lvl1_idx, len);
		} else {
			ret = -ENOMEM;
		}

		if (ret) {
			gasket_page_table_unmap_nolock(pg_tbl, dev_addr,
						       num_pages);
			return ret;
		}

		remain -= len;
		lvl0_idx++;
		lvl1_idx = 0;
		pte++;
		host_addr += len * PAGE_SIZE;
	}

	return 0;
}

/*
 * See gasket_page_table.h for general description.
 *
 * gasket_page_table_map calls either gasket_map_simple_pages() or
 * gasket_map_extended_pages() to actually perform the mapping.
 *
 * The page table mutex is held for the entire operation.
 */
int gasket_page_table_map(struct gasket_page_table *pg_tbl, ulong host_addr,
			  ulong dev_addr, uint num_pages, u32 flags,
			  bool is_user_addr, bool dev_alloc_ptes)
{
	ulong host_map_addr = host_addr;
	int ret;

	if (!num_pages)
		return 0;

	if (GET(FLAGS_DMA_DIRECTION, flags) == DMA_NONE) {
		dev_err(pg_tbl->gasket_dev->dev,
			"invalid DMA direction flags=0x%lx\n",
			(unsigned long)flags);
		return -EINVAL;
	}

	/* Mask off flags other than DMA direction. */
	flags &= MASK(FLAGS_DMA_DIRECTION);

	/*
	 * If the userspace address falls within the range devoted to
	 * coherent memory then substitute the DMA address of the kernel-
	 * alloc'ed coherent range and switch to non-userspace mapping.
	 */
	if (is_user_addr && is_coherent(pg_tbl, host_addr)) {
		u64 off =
			(u64)host_addr -
			(u64)pg_tbl->coherent_pages[0].user_virt;
		host_map_addr = pg_tbl->coherent_pages[0].paddr + off;
		is_user_addr = false;
	}

	mutex_lock(&pg_tbl->mutex);

	if (gasket_addr_is_simple(pg_tbl, dev_addr)) {
		ret = gasket_map_simple_pages(pg_tbl, host_map_addr, dev_addr,
					      num_pages, flags, is_user_addr);
	} else {
		ret = gasket_map_extended_pages(pg_tbl, host_map_addr, dev_addr,
						num_pages, flags, is_user_addr,
						dev_alloc_ptes);
	}

	mutex_unlock(&pg_tbl->mutex);

	dev_dbg(pg_tbl->gasket_dev->dev,
		"%s done: ha %llx daddr %llx num %d, flags %x ret %d\n",
		__func__, (unsigned long long)host_addr,
		(unsigned long long)dev_addr, num_pages, flags, ret);
	return ret;
}
EXPORT_SYMBOL(gasket_page_table_map);

/*
 * See gasket_page_table.h for general description.
 *
 * gasket_page_table_unmap takes the page table lock and calls either
 * gasket_unmap_simple_pages() or gasket_unmap_extended_pages() to
 * actually unmap the pages from device space.
 *
 * The page table mutex is held for the entire operation.
 */
void gasket_page_table_unmap(struct gasket_page_table *pg_tbl, ulong dev_addr,
			     uint num_pages)
{
	if (!num_pages)
		return;

	mutex_lock(&pg_tbl->mutex);
	gasket_page_table_unmap_nolock(pg_tbl, dev_addr, num_pages);
	mutex_unlock(&pg_tbl->mutex);
}
EXPORT_SYMBOL(gasket_page_table_unmap);

static void
gasket_page_table_unmap_all_nolock(struct gasket_page_table *pg_tbl,
				   bool update_subtables)
{
	gasket_unmap_simple_pages(pg_tbl,
				  gasket_components_to_dev_address(pg_tbl, 1, 0,
								   0),
				  pg_tbl->num_simple_entries);
	gasket_unmap_extended_pages(pg_tbl,
				    gasket_components_to_dev_address(pg_tbl, 0,
								     0, 0),
				    pg_tbl->num_extended_entries *
				    GASKET_PAGES_PER_SUBTABLE,
				    update_subtables);
}

/*
 * Device unreachable, cleanup local state without touching chip.
 * Caller must hold pg_tbl->mutex.
 */
static void gasket_page_table_reset_local(struct gasket_page_table *pg_tbl)
{
	struct gasket_page_table_entry *ptes = pg_tbl->entries;
	uint i;

	/* Release the simple entries. */
	for (i = 0; i < pg_tbl->num_simple_entries; i++)
		gasket_release_entry(pg_tbl, &ptes[i]);

	/* Release the extended entries. */
	for (i = pg_tbl->num_simple_entries; i < pg_tbl->config.total_entries;
	     i++) {
		u64 __iomem *slot_base;

		if (GET(FLAGS_STATUS, ptes[i].flags) == PTE_FREE)
			continue;

		gasket_release_subtable(pg_tbl, ptes[i].sublevel, 0,
					GASKET_PAGES_PER_SUBTABLE);

		if (GET(FLAGS_DEV_SUBTABLE, ptes[i].flags))
			continue;

		/* Invalidate all entries of host-resident subtable. */
		slot_base = (u64 __iomem *)(page_address(ptes[i].page) +
					    ptes[i].offset);
		memset(slot_base, 0,
		       GASKET_PAGES_PER_SUBTABLE * sizeof(u64));
	}
}

/* See gasket_page_table.h for description. */
void gasket_page_table_reset(struct gasket_page_table *pg_tbl,
			     bool update_subtables)
{
	mutex_lock(&pg_tbl->mutex);
	gasket_update_hw_status(pg_tbl->gasket_dev);

	if (pg_tbl->gasket_dev->status == GASKET_STATUS_DEAD) {
		gasket_page_table_reset_local(pg_tbl);
	} else {
		gasket_page_table_unmap_all_nolock(pg_tbl, update_subtables);
		writeq(pg_tbl->config.total_entries,
		       pg_tbl->extended_offset_reg);
	}

	mutex_unlock(&pg_tbl->mutex);
}

/* See gasket_page_table.h for description. */
bool gasket_page_table_are_addrs_bad(
	struct gasket_page_table *pg_tbl, ulong host_addr, ulong dev_addr,
	ulong bytes)
{
	if (host_addr & (PAGE_SIZE - 1)) {
		dev_err(pg_tbl->gasket_dev->dev,
			"host mapping address %pK must be page aligned\n",
			(void *)host_addr);
		return true;
	}

	return gasket_page_table_is_dev_addr_bad(pg_tbl, dev_addr, bytes);
}
EXPORT_SYMBOL(gasket_page_table_are_addrs_bad);

/* See gasket_page_table.h for description. */
bool gasket_page_table_is_dev_addr_bad(
	struct gasket_page_table *pg_tbl, ulong dev_addr, ulong bytes)
{
	uint num_pages = bytes / PAGE_SIZE;

	if (bytes & (PAGE_SIZE - 1)) {
		dev_err(pg_tbl->gasket_dev->dev,
			"mapping size 0x%lX must be page aligned\n", bytes);
		return true;
	}

	if (num_pages == 0) {
		dev_err(pg_tbl->gasket_dev->dev,
			"requested mapping is less than one page: %lu / %lu\n",
			bytes, PAGE_SIZE);
		return true;
	}

	if (gasket_addr_is_simple(pg_tbl, dev_addr))
		return gasket_is_simple_dev_addr_bad(pg_tbl, dev_addr,
						     num_pages);
	return gasket_is_extended_dev_addr_bad(pg_tbl, dev_addr, num_pages);
}
EXPORT_SYMBOL(gasket_page_table_is_dev_addr_bad);

/* See gasket_page_table.h for description. */
uint gasket_page_table_max_size(struct gasket_page_table *page_table)
{
	if (!page_table)
		return 0;
	return page_table->config.total_entries;
}
EXPORT_SYMBOL(gasket_page_table_max_size);

/* See gasket_page_table.h for description. */
uint gasket_page_table_num_entries(struct gasket_page_table *pg_tbl)
{
	if (!pg_tbl)
		return 0;
	return pg_tbl->num_simple_entries + pg_tbl->num_extended_entries;
}
EXPORT_SYMBOL(gasket_page_table_num_entries);

/* See gasket_page_table.h for description. */
uint gasket_page_table_num_simple_entries(struct gasket_page_table *pg_tbl)
{
	if (!pg_tbl)
		return 0;
	return pg_tbl->num_simple_entries;
}
EXPORT_SYMBOL(gasket_page_table_num_simple_entries);

/* See gasket_page_table.h for description. */
uint gasket_page_table_num_active_pages(struct gasket_page_table *pg_tbl)
{
	if (!pg_tbl)
		return 0;
	return pg_tbl->num_active_pages;
}
EXPORT_SYMBOL(gasket_page_table_num_active_pages);

/* See gasket_page_table.h */
int gasket_page_table_system_status(struct gasket_page_table *page_table)
{
	if (!page_table)
		return GASKET_STATUS_LAMED;

	if (gasket_page_table_num_entries(page_table) == 0) {
		dev_dbg(page_table->gasket_dev->dev, "Page table size is 0\n");
		return GASKET_STATUS_LAMED;
	}

	return GASKET_STATUS_ALIVE;
}

/* Record the host_addr to coherent dma memory mapping. */
int gasket_set_user_virt(
	struct gasket_dev *gasket_dev, u64 size, dma_addr_t dma_address,
	ulong vma)
{
	int j;
	struct gasket_page_table *pg_tbl;

	unsigned int num_pages = size / PAGE_SIZE;

	/*
	 * TODO: for future chipset, better handling of the case where multiple
	 * page tables are supported on a given device
	 */
	pg_tbl = gasket_dev->page_table[0];
	if (!pg_tbl) {
		dev_dbg(gasket_dev->dev, "%s: invalid page table index\n",
			__func__);
		return 0;
	}
	for (j = 0; j < num_pages; j++) {
		pg_tbl->coherent_pages[j].user_virt =
			(u64)vma + j * PAGE_SIZE;
	}
	return 0;
}

/* Allocate a block of coherent memory. */
int gasket_alloc_coherent_memory(struct gasket_dev *gasket_dev, u64 size,
				 dma_addr_t *dma_address, u64 index)
{
	dma_addr_t handle;
	void *mem;
	int j;
	unsigned int num_pages = (size + PAGE_SIZE - 1) / PAGE_SIZE;
	const struct gasket_driver_desc *driver_desc =
		gasket_get_driver_desc(gasket_dev);

	if (!gasket_dev->page_table[index])
		return -EFAULT;

	if (num_pages == 0)
		return -EINVAL;

	mutex_lock(&gasket_dev->page_table[index]->mutex);
	mem = dma_alloc_coherent(gasket_dev->dma_dev, num_pages * PAGE_SIZE,
				 &handle, GFP_KERNEL);
	if (!mem)
		goto nomem;

	gasket_dev->page_table[index]->num_coherent_pages = num_pages;

	/* allocate the physical memory block */
	gasket_dev->page_table[index]->coherent_pages =
		kcalloc(num_pages, sizeof(struct gasket_coherent_page_entry),
			GFP_KERNEL);
	if (!gasket_dev->page_table[index]->coherent_pages)
		goto nomem;

	gasket_dev->coherent_buffer.length_bytes =
		PAGE_SIZE * (num_pages);
	gasket_dev->coherent_buffer.phys_base = handle;
	gasket_dev->coherent_buffer.virt_base = mem;

	*dma_address = driver_desc->coherent_buffer_description.base;
	for (j = 0; j < num_pages; j++) {
		gasket_dev->page_table[index]->coherent_pages[j].paddr =
			handle + j * PAGE_SIZE;
		gasket_dev->page_table[index]->coherent_pages[j].kernel_virt =
			(u64)mem + j * PAGE_SIZE;
	}

	mutex_unlock(&gasket_dev->page_table[index]->mutex);
	return 0;

nomem:
	if (mem) {
		dma_free_coherent(gasket_dev->dma_dev, num_pages * PAGE_SIZE,
				  mem, handle);
		gasket_dev->coherent_buffer.length_bytes = 0;
		gasket_dev->coherent_buffer.virt_base = NULL;
		gasket_dev->coherent_buffer.phys_base = 0;
	}

	kfree(gasket_dev->page_table[index]->coherent_pages);
	gasket_dev->page_table[index]->coherent_pages = NULL;
	gasket_dev->page_table[index]->num_coherent_pages = 0;
	mutex_unlock(&gasket_dev->page_table[index]->mutex);
	return -ENOMEM;
}

/* Free a block of coherent memory. */
int gasket_free_coherent_memory(struct gasket_dev *gasket_dev, u64 size,
				dma_addr_t dma_address, u64 index)
{
	const struct gasket_driver_desc *driver_desc;

	if (!gasket_dev->page_table[index])
		return -EFAULT;

	driver_desc = gasket_get_driver_desc(gasket_dev);

	if (driver_desc->coherent_buffer_description.base != dma_address)
		return -EADDRNOTAVAIL;

	gasket_free_coherent_memory_all(gasket_dev, index);

	return 0;
}

/* Release all coherent memory. */
void gasket_free_coherent_memory_all(
	struct gasket_dev *gasket_dev, u64 index)
{
	if (!gasket_dev->page_table[index])
		return;

	mutex_lock(&gasket_dev->page_table[index]->mutex);
	if (gasket_dev->coherent_buffer.length_bytes) {
		dma_free_coherent(gasket_dev->dma_dev,
				  gasket_dev->coherent_buffer.length_bytes,
				  gasket_dev->coherent_buffer.virt_base,
				  gasket_dev->coherent_buffer.phys_base);
		gasket_dev->coherent_buffer.length_bytes = 0;
		gasket_dev->coherent_buffer.virt_base = NULL;
		gasket_dev->coherent_buffer.phys_base = 0;
	}

	kfree(gasket_dev->page_table[index]->coherent_pages);
	gasket_dev->page_table[index]->coherent_pages = NULL;
	gasket_dev->page_table[index]->num_coherent_pages = 0;
	mutex_unlock(&gasket_dev->page_table[index]->mutex);
}

/*
 * Keep track of current contiguous mapping range as page tables traversed.
 * If page_count is zero then no range has been started yet.
 */
struct mmu_dump_range {
	/* starting TPU device address of range */
	ulong dev_addr_start;
	/* starting DMA address of range */
	dma_addr_t dma_addr_start;
	/* starting host physical address of range if user mapping, else 0 */
	phys_addr_t phys_addr_start;
	/* number of PAGE_SIZE pages in range so far, zero for none */
	uint page_count;
	/* is this a coherent memory mapping? */
	bool coherent;
};

/* Stats kept by the MMU traverse code and dumped at the end */
struct mmu_dump_stats {
	uint total_pages;		/* # total pages mapped */
	uint simple_pages;		/* # simple pages mapped */
	uint ext_pages;			/* # extended pages mapped */
	uint user_pages;		/* # userspace pages mapped */
	uint nonuser_pages;		/* # non-user (dev+coherent) pages */
	uint dev_subtables;		/* # on-device subtables in use */
	uint coherent_pages;		/* # coherent memory pages mapped */
	uint devpte_userpages;		/* # user pages in on-device ptes */
	uint hostpte_nonuserpages;	/* # non-user pages in on-host ptes */
};

/* Dump the current accumulated range of mappings. */
static void dump_range(struct seq_file *s, struct mmu_dump_range *range)
{
	seq_printf(s, "0x%016llx: 0x%016llx", range->dev_addr_start,
		   range->dma_addr_start);
	if (range->phys_addr_start)
		seq_printf(s, " 0x%016llx", range->phys_addr_start);
	seq_printf(s, " %u %c\n", range->page_count,
		   range->coherent ? 'c' : range->phys_addr_start ? 'u' : 'd');
}

/*
 * Add next PTE to current range, or dump previous and start a new one.
 * If pte is NULL then there's no new range, just dump any accumulated range.
 */
static void dump_next_pte(struct seq_file *s, struct gasket_page_table *pg_tbl,
			  ulong dev_addr, struct gasket_page_table_entry *pte,
			  struct mmu_dump_range *range,
			  struct mmu_dump_stats *stats)
{
	/* non-zero for userspace mapping physical address */
	phys_addr_t this_physaddr = 0;

	if (pte && pte->page)
		this_physaddr = page_to_phys(pte->page);

	/* Dump current range and/or start a new range? */
	if (!range->page_count || !pte ||
	    range->dev_addr_start + range->page_count * PAGE_SIZE != dev_addr ||
	    range->dma_addr_start + range->page_count * PAGE_SIZE !=
	    pte->dma_addr ||
	    ((range->phys_addr_start || this_physaddr) &&
	     range->phys_addr_start + range->page_count * PAGE_SIZE !=
	     this_physaddr)) {
		/* Dump current range, if any. */
		if (range->page_count)
			dump_range(s, range);

		/* Start new range. */
		range->dev_addr_start = dev_addr;
		range->phys_addr_start = this_physaddr;
		range->dma_addr_start = pte ? pte->dma_addr : 0;
		range->page_count = pte ? 1 : 0;
		range->coherent = pte ? is_coherent_dma(pg_tbl, pte->dma_addr)
			: false;
	} else {
		/* Extend current range by one more page. */
		range->page_count++;
	}

	if (pte) {
		stats->total_pages++;

		if (gasket_addr_is_simple(pg_tbl, dev_addr))
			stats->simple_pages++;
		else
			stats->ext_pages++;

		if (pte->page)
			stats->user_pages++;
		else
			stats->nonuser_pages++;

		if (is_coherent_dma(pg_tbl, pte->dma_addr))
			stats->coherent_pages++;
	}
}

static void dump_page_table(struct gasket_page_table *pg_tbl,
			    struct seq_file *s)
{
	struct gasket_page_table_entry *ptes;
	struct mmu_dump_range range;
	struct mmu_dump_stats stats;
	uint i, j;

	ptes = pg_tbl ? pg_tbl->entries : NULL;
	if (!ptes) {
		seq_puts(s, "uninitialized\n");
		return;
	}

	memset(&stats, 0, sizeof(stats));
	range.page_count = 0;

	for (i = 0; i < pg_tbl->num_simple_entries; i++) {
		if (GET(FLAGS_STATUS, ptes[i].flags) != PTE_FREE)
			dump_next_pte(s, pg_tbl, i * PAGE_SIZE, &ptes[i],
				      &range, &stats);
	}

	for (i = pg_tbl->num_simple_entries; i < pg_tbl->config.total_entries;
	     i++) {
		if (GET(FLAGS_STATUS, ptes[i].flags) == PTE_FREE)
			continue;

		if (GET(FLAGS_DEV_SUBTABLE, ptes[i].flags))
			stats.dev_subtables++;

		for (j = 0; j < GASKET_PAGES_PER_SUBTABLE; j++) {
			ulong dev_addr;

			if (GET(FLAGS_STATUS, ptes[i].sublevel[j].flags) ==
			    PTE_FREE)
				continue;

			dev_addr = (i - pg_tbl->num_simple_entries) *
				GASKET_PAGES_PER_SUBTABLE * PAGE_SIZE +
				j * PAGE_SIZE;
			dev_addr |= pg_tbl->extended_flag;

			/*
			 * Count non-user pages (will be device, not coherent,
			 * pages) mapped using on-host subtables.  Also count
			 * user pages mapped using on-device subtables.  We
			 * report these as potential performance problems.
			 */
			if (!GET(FLAGS_DEV_SUBTABLE, ptes[i].flags)) {
				if (!ptes[i].sublevel[j].page)
					stats.hostpte_nonuserpages++;
			} else if (ptes[i].sublevel[j].page) {
				stats.devpte_userpages++;
			}

			dump_next_pte(s, pg_tbl, dev_addr, &ptes[i].sublevel[j],
				      &range, &stats);
		}
	}

	dump_next_pte(s, pg_tbl, 0, NULL, &range, &stats);
	seq_printf(s, "pages: %u simple: %u ext: %u user: %u dev: %u"
		   " coherent: %d\n",
		   stats.total_pages, stats.simple_pages, stats.ext_pages,
		   stats.user_pages, stats.nonuser_pages - stats.coherent_pages,
		   stats.coherent_pages);
	seq_printf(s, "dev subtbl: %u devst-usrpg: %u hostst-nusrpg: %u\n",
		   stats.dev_subtables, stats.devpte_userpages,
		   stats.hostpte_nonuserpages);
}

void gasket_page_table_dump(struct gasket_dev *gasket_dev,
			    struct seq_file *s)
{
	uint i;

	for (i = 0; i < gasket_dev->num_page_tables; i++) {
		seq_printf(s, "page table %u\n", i);
		dump_page_table(gasket_dev->page_table[i], s);
	}
}
