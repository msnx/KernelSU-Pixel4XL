// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the Oscar chip.
 *
 * Copyright (C) 2017 Google, Inc.
 */

#include <asm/current.h>
#include <linux/airbrush-sm-notifier.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mfd/abc-pcie-notifier.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/refcount.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include <linux/mfd/abc-pcie.h>
#include "oscar.h"

#include "gasket_core.h"
#include "gasket_interrupt.h"
#include "gasket_page_table.h"
#include "gasket_sysfs.h"

/* for abc ioctls, move later */
#include <linux/dma-buf.h>
#include <linux/ab-dram.h>
#include <linux/mfd/abc-pcie-dma.h>
#include <uapi/linux/abc-pcie-dma.h>

#define DRIVER_NAME "abc-pcie-tpu"
#define DRIVER_VERSION "0.3"

#define OSCAR_BAR_SIZE 0x100000

/* Number of bytes allocated for coherent memory. */
#define OSCAR_CH_MEM_BYTES (PAGE_SIZE * MAX_NUM_COHERENT_PAGES)

/* Access PCI memory via BAR 0 for the Gasket framework */
#define OSCAR_BAR_INDEX 0

/* The number of user-mappable memory ranges in Oscar BAR. */
#define NUM_BAR_RANGES 4

#define OSCAR_BAR_OFFSET 0
#define OSCAR_CM_OFFSET 0x1000000

/* The number of nodes in an Oscar chip. */
#define NUM_NODES 1
/* The number of TPU tiles in an Oscar chip */
#define NTILES	15

/* TPU logical interrupts handled by this driver */
#define OSCAR_SCALAR_CORE_0_INT	0 /* signalled via an MSI IRQ */
#define OSCAR_INSTR_QUEUE_INT	1 /* signalled via an MSI IRQ */
#define OSCAR_LOWPRIO_INT	2 /* AKA wireinterrupt_2, notified by parent */

#define OSCAR_N_IRQS		2 /* logical interrupts 0 and 1 are IRQs */
#define OSCAR_N_INTS		3 /* 3 logical interrupts including mux'ed */

#define SC_0_IRQ_NAME		"tpu-scalar-core-0-irq"
#define INST_QUEUE_IRQ_NAME	"tpu-instr-queue-irq"

/*
 * The total number of entries in the page table. Should match the value read
 * from the register OSCAR_BAR_REG_HIB_PAGE_TABLE_SIZE.
 */
#define OSCAR_PAGE_TABLE_TOTAL_ENTRIES 2048

#define OSCAR_EXTENDED_SHIFT 63 /* Extended address bit position. */

#define OSCAR_RESET_RETRY 120	/* check reset 120 times */
#define OSCAR_RESET_DELAY 100	/* wait 100 ms between checks */
				/* total 12 sec wait maximum */
#define OSCAR_ACK_DELAY	  1     /* wait 1 ms between checking reg writes */

/* enum sysfs_attribute_type: enumeration of the supported sysfs entries. */
enum sysfs_attribute_type {
	ATTR_KERNEL_HIB_PAGE_TABLE_SIZE,
	ATTR_KERNEL_HIB_SIMPLE_PAGE_TABLE_SIZE,
	ATTR_KERNEL_HIB_NUM_ACTIVE_PAGES,
};

/*
 * Register offsets into BAR memory.
 * Only values necessary for driver implementation are defined.
 */
enum oscar_bar_regs {
	OSCAR_BAR_REG_TILECONF1_DEEPSLEEP = 0x2020,
	OSCAR_BAR_REG_TILECONF1_ERROR_TILE = 0x2370,
	OSCAR_BAR_REG_TILECONF1_ERROR_MASK_TILE = 0x2378,
	OSCAR_BAR_REG_TILECONF1_ERROR_INFO_TILE = 0x2390,
	OSCAR_BAR_REG_SC_RUNCTRL = 0x4018,
	OSCAR_BAR_REG_SC_RUNSTATUS = 0x4258,
	OSCAR_BAR_REG_SC_ERROR_MASK = 0x4268,
	OSCAR_BAR_REG_SC_ERROR_INFO = 0x4280,
	OSCAR_BAR_REG_HIB_PAGE_TABLE_SIZE = 0x6000,
	OSCAR_BAR_REG_KERNEL_HIB_EXTENDED_TABLE = 0x6008,
	OSCAR_BAR_REG_HIB_PAGE_TABLE_INIT = 0x6078,
	OSCAR_BAR_REG_USER_HIB_DMA_PAUSE = 0x86D8,
	OSCAR_BAR_REG_USER_HIB_DMA_PAUSED = 0x86E0,
	OSCAR_BAR_REG_USER_HIB_ERROR_STATUS = 0x86F0,
	OSCAR_BAR_REG_USER_HIB_FIRST_ERROR_STATUS = 0x8700,
	OSCAR_BAR_REG_USER_HIB_PAGE_FAULT_ADDR = 0x8738,
	OSCAR_BAR_REG_USER_HIB_TILECONFIG1 = 0x8790,
	OSCAR_BAR_REG_HIB_PAGE_TABLE = 0x10000,

	/* AON Always On (Top Level) Registers. */
	OSCAR_BAR_REG_AON_RESET = 0x20000,
	OSCAR_BAR_REG_AON_CLOCK_ENABLE = 0x20008,
	OSCAR_BAR_REG_AON_LOGIC_SHUTDOWN_PRE = 0x00020010,
	OSCAR_BAR_REG_AON_LOGIC_SHUTDOWN_ALL = 0x00020018,
	OSCAR_BAR_REG_AON_TILEMEM_RETN = 0x00020020,
	OSCAR_BAR_REG_AON_MEM_SHUTDOWN = 0x00020028,
	OSCAR_BAR_REG_AON_MEM_SHUTDOWN_ACK = 0x00020030,
	OSCAR_BAR_REG_AON_CLAMP_ENABLE = 0x00020038,
	OSCAR_BAR_REG_AON_FORCE_QUIESCE = 0x00020040,
	OSCAR_BAR_REG_AON_AXITIEOFFS = 0x00020048,
	OSCAR_BAR_REG_AON_AXIBRIDGEDEBUG1 = 0x00020070,
};

/* AON_LOGIC_SHUTDOWN_* registers mask for logic_shutdown_*_ack field */
#define MASK_AON_LOGIC_SHUTDOWN_ACK (0x1fffffull << 17)
/* AON_MEM_SHUTDOWN_ACK register mask for memory_shutdown_ack field */
#define MASK_AON_MEM_SHUTDOWN_ACK (0x1fffff)
/* SC_RUNCTRL register mask for trigger field */
#define MASK_SC_RUNCTRL_TRIGGER (0x3)

/* Central PMU register defines used here. */
#define CENTRAL_PMU_BASE	(AON_AXI2APB + 0xA0000) /* 0xBA0000 */
#define TPU_POWER_CTL_OVERRIDE_0	(CENTRAL_PMU_BASE + 0x2500)
#define TPU_POWER_CTL_OVERRIDE_ENABLE	(1 << 0)
#define TILE_ISOLATION_EN		(0x1ffff << 2)

/* Configuration for page table. */
static struct gasket_page_table_config oscar_page_table_configs[NUM_NODES] = {
	{
		.id = 0,
		.mode = GASKET_PAGE_TABLE_MODE_NORMAL,
		.total_entries = OSCAR_PAGE_TABLE_TOTAL_ENTRIES,
		.base_reg = OSCAR_BAR_REG_HIB_PAGE_TABLE,
		.extended_reg = OSCAR_BAR_REG_KERNEL_HIB_EXTENDED_TABLE,
		.extended_bit = OSCAR_EXTENDED_SHIFT,
	},
};

/* The regions in the BAR0 space that can be mapped into user space. */
static const struct gasket_mappable_region
oscar_mappable_regions[NUM_BAR_RANGES] = {
	{ 0x0000, 0x1000 },
	{ 0x2000, 0x1000 },
	{ 0x4000, 0x1000 },
	{ 0x8000, 0x1000 },
};

static const struct gasket_mappable_region cm_mappable_regions[1] = {
	{ 0x00000, OSCAR_CH_MEM_BYTES }
};

/* Gasket interrupt data, not really used since we manage our own IRQs */
static struct gasket_interrupt_desc oscar_interrupts[OSCAR_N_INTS];

/* Describes an Airbrush DRAM buffer mapped to TPU */
struct abc_buffer {
	refcount_t refcount;
	struct list_head abc_buffers_list; /* protected by abc_buffers_lock */
	struct oscar_dev *oscar_dev;
	struct dma_buf *dma_buf;
	struct mutex mapping_lock; /* protects sg_table and below */
	struct sg_table *sg_table;
	struct dma_buf_attachment *dma_buf_attachment;
};

/* Airbrush DRAM extended level-1 subtable metadata */
struct abc_subtable {
	struct dma_buf *dma_buf;
	struct sg_table *sg_table;
	struct dma_buf_attachment *dma_buf_attachment;
	struct bar_mapping bar_mapping;
};

/* One of these per oscar device instance */
struct oscar_dev {
	struct gasket_dev *gasket_dev;
	bool parent_ioremap;
	int irqs[OSCAR_N_IRQS]; /* maps MSI IRQs to TPU logical ints */
	struct notifier_block lowprio_irq_nb;
	struct atomic_notifier_head *lowprio_irq_nh;
	/* List of all active ABC DRAM buffers, protected by abc_buffers_lock */
	struct mutex abc_buffers_lock;
	struct list_head abc_buffers;
	struct mutex dev_avail_lock; /* protects clk_lockout, pcie_lockout */
	bool clk_lockout; /* clock change in progress */
	bool pcie_lockout; /* PCIe link blocked */
	struct notifier_block clk_change_nb;
	struct notifier_block pcie_link_change_nb;
	struct dentry *d_entry; /* debugfs dir for this device */
};


static bool check_dev_avail(struct oscar_dev *oscar_dev)
{
	bool ret;

	mutex_lock(&oscar_dev->dev_avail_lock);
	ret = !oscar_dev->clk_lockout && !oscar_dev->pcie_lockout;
	mutex_unlock(&oscar_dev->dev_avail_lock);
	return ret;
}

static int oscar_device_open_cb(struct gasket_dev *gasket_dev)
{
	struct oscar_dev *oscar_dev =
		platform_get_drvdata(gasket_dev->platform_dev);

	if (!check_dev_avail(oscar_dev))
		return -EIO;
	return gasket_reset_nolock(gasket_dev);
}

static int oscar_get_status(struct gasket_dev *gasket_dev)
{
	struct oscar_dev *oscar_dev =
		platform_get_drvdata(gasket_dev->platform_dev);

	return check_dev_avail(oscar_dev) ? GASKET_STATUS_ALIVE :
		GASKET_STATUS_DEAD;
}

/* Caller must hold abc_buffers_lock */
static struct abc_buffer *abc_get_buffer_locked(struct oscar_dev *oscar_dev,
						int fd)
{
	struct abc_buffer *abc_buffer;
	struct dma_buf *dma_buf = dma_buf_get(fd);

	if (IS_ERR(dma_buf))
		return NULL;

	dma_buf_put(dma_buf);

	list_for_each_entry(abc_buffer, &oscar_dev->abc_buffers,
			    abc_buffers_list) {
		if (abc_buffer->dma_buf == dma_buf) {
			refcount_inc(&abc_buffer->refcount);
			return abc_buffer;
		}
	}
	return NULL;
}

static struct abc_buffer *abc_get_buffer(struct oscar_dev *oscar_dev, int fd)
{
	struct abc_buffer *buffer;

	mutex_lock(&oscar_dev->abc_buffers_lock);
	buffer = abc_get_buffer_locked(oscar_dev, fd);
	mutex_unlock(&oscar_dev->abc_buffers_lock);
	return buffer;
}

/*
 * Caller must hold abc_buffers_lock and abc_buffer->mapping_lock.
 * abc_buffer not valid on return, and abc_buffers list entry is deleted, use
 * safe iterators.
 */
static void oscar_abc_release_buffer(struct abc_buffer *abc_buffer)
{
	list_del(&abc_buffer->abc_buffers_list);
	if (abc_buffer->sg_table)
		dma_buf_unmap_attachment(abc_buffer->dma_buf_attachment,
					 abc_buffer->sg_table, DMA_TO_DEVICE);
	dma_buf_detach(abc_buffer->dma_buf, abc_buffer->dma_buf_attachment);
	dma_buf_put(abc_buffer->dma_buf); /* release our ref on dma_buf */
	kfree(abc_buffer);
}

/*
 * Caller must hold abc_buffers_lock (and not hold abc_buffer->mapping_lock).
 * abc_buffer may not be valid on return, and abc_buffers list entry may be
 * deleted, use safe iterators.
 */
static void abc_put_buffer_locked(struct abc_buffer *abc_buffer)
{
	mutex_lock(&abc_buffer->mapping_lock);
	if (refcount_dec_and_test(&abc_buffer->refcount))
		oscar_abc_release_buffer(abc_buffer);
	else
		mutex_unlock(&abc_buffer->mapping_lock);

}

/* abc_buffer not valid on return */
static void abc_put_buffer(struct abc_buffer *abc_buffer)
{
	struct oscar_dev *oscar_dev = abc_buffer->oscar_dev;

	mutex_lock(&oscar_dev->abc_buffers_lock);
	abc_put_buffer_locked(abc_buffer);
	mutex_unlock(&oscar_dev->abc_buffers_lock);
}

/*
 * Add a dma_buf fd to our list of Airbrush "buffers".
 * Caller must hold abc_buffers_lock.
 */
static struct abc_buffer *abc_add_fd_locked(struct oscar_dev *oscar_dev, int fd)
{
	struct dma_buf *abc_dma_buf;
	struct dma_buf_attachment *dma_buf_attachment;
	struct abc_buffer *abc_buffer;
	struct abc_buffer *temp_abc_buf;
	int ret;

	/* Grab a ref to the dma_buf for so long as it's in our list. */
	abc_dma_buf = dma_buf_get(fd);
	if (IS_ERR(abc_dma_buf))
		return (struct abc_buffer *)abc_dma_buf;

	if (!is_ab_dram_dma_buf(abc_dma_buf)) {
		ret = -EINVAL;
		goto put_fd;
	}

	dma_buf_attachment =
		dma_buf_attach(abc_dma_buf, oscar_dev->gasket_dev->dev);
	if (IS_ERR(dma_buf_attachment)) {
		ret = PTR_ERR(dma_buf_attachment);
		goto put_fd;
	}

	abc_buffer = kzalloc(sizeof(struct abc_buffer), GFP_KERNEL);
	if (!abc_buffer) {
		ret = -ENOMEM;
		goto detach_buf;
	}

	abc_buffer->dma_buf = abc_dma_buf;
	abc_buffer->dma_buf_attachment = dma_buf_attachment;
	abc_buffer->oscar_dev = oscar_dev;
	INIT_LIST_HEAD(&abc_buffer->abc_buffers_list);
	refcount_set(&abc_buffer->refcount, 1);
	mutex_init(&abc_buffer->mapping_lock);

	temp_abc_buf = abc_get_buffer_locked(oscar_dev, fd);
	if (WARN_ON(temp_abc_buf)) {
		abc_put_buffer_locked(temp_abc_buf);
		kfree(abc_buffer);
		ret = -EBUSY;
		goto detach_buf;
	}

	list_add_tail(&abc_buffer->abc_buffers_list, &oscar_dev->abc_buffers);
	return abc_buffer;

detach_buf:
	dma_buf_detach(abc_dma_buf, dma_buf_attachment);
put_fd:
	dma_buf_put(abc_dma_buf);
	return ERR_PTR(ret);
}

static int oscar_abc_map_buffer(struct oscar_dev *oscar_dev,
				struct oscar_abdram_map_ioctl __user *argp)
{
	struct gasket_dev *gasket_dev = oscar_dev->gasket_dev;
	struct oscar_abdram_map_ioctl ibuf;
	struct abc_buffer *abc_buffer;
	struct scatterlist *sg;
	u64 dev_addr;
	dma_addr_t dma_addr;
	int fd;
	uint len;
	int i, j;
	int ret;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	if (ibuf.page_table_index >= gasket_dev->num_page_tables)
		return -EFAULT;

	fd = ibuf.fd;
	mutex_lock(&oscar_dev->abc_buffers_lock);
	abc_buffer = abc_get_buffer_locked(oscar_dev, fd);

	/*
	 * If this fd isn't already in our list of tracked ab-dram buffer fds,
	 * add it now.
	 */
	if (!abc_buffer)
		abc_buffer = abc_add_fd_locked(oscar_dev, fd);

	mutex_unlock(&oscar_dev->abc_buffers_lock);

	if (IS_ERR_OR_NULL(abc_buffer))
		return -EINVAL;

	mutex_lock(&abc_buffer->mapping_lock);
	if (!abc_buffer->sg_table) {
		abc_buffer->sg_table =
			dma_buf_map_attachment(abc_buffer->dma_buf_attachment,
					       DMA_TO_DEVICE);
		if (IS_ERR(abc_buffer->sg_table)) {
			ret = PTR_ERR(abc_buffer->sg_table);
			abc_buffer->sg_table = NULL;
			goto unlock_put_buffer;
		}
	}

	dev_addr = ibuf.device_address;

	for (i = 0, sg = abc_buffer->sg_table->sgl;
	     i < abc_buffer->sg_table->nents; i++, sg = sg_next(sg)) {
		dma_addr = sg_dma_address(sg);
		len = sg_dma_len(sg);

		if (gasket_page_table_are_addrs_bad(
			    gasket_dev->page_table[ibuf.page_table_index],
			    dma_addr, dev_addr, len)) {
			ret = -EINVAL;
			goto unmap_sg;
		}

		ret = gasket_page_table_map(
			gasket_dev->page_table[ibuf.page_table_index],
			dma_addr, dev_addr, len / PAGE_SIZE,
			DMA_TO_DEVICE << 1, false, true);
		if (ret)
			goto unmap_sg;

		dev_addr += len;
	}

	refcount_inc(&abc_buffer->refcount);

unmap_sg:
	if (ret) {
		dev_addr = ibuf.device_address;

		for (j = 0, sg = abc_buffer->sg_table->sgl; j < i;
		     j++, sg = sg_next(sg)) {
			len = sg_dma_len(sg);

			gasket_page_table_unmap(
				gasket_dev->page_table[ibuf.page_table_index],
				dev_addr, len / PAGE_SIZE);

			dev_addr += len;
		}
		dma_buf_unmap_attachment(abc_buffer->dma_buf_attachment,
					 abc_buffer->sg_table, DMA_TO_DEVICE);
		abc_buffer->sg_table = NULL;
	}
unlock_put_buffer:
	mutex_unlock(&abc_buffer->mapping_lock);
	abc_put_buffer(abc_buffer);
	return ret;
}

static int
oscar_abc_unmap_abdram(struct oscar_dev *oscar_dev,
		       struct oscar_abdram_map_ioctl __user *argp)
{
	struct gasket_dev *gasket_dev = oscar_dev->gasket_dev;
	struct abc_buffer *abc_buffer;
	struct oscar_abdram_map_ioctl ibuf;
	struct scatterlist *sg;
	size_t len = 0;
	int i;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	if (ibuf.page_table_index >= oscar_dev->gasket_dev->num_page_tables)
		return -EINVAL;

	abc_buffer = abc_get_buffer(oscar_dev, ibuf.fd);
	if (!abc_buffer)
		return -EINVAL;

	mutex_lock(&abc_buffer->mapping_lock);
	if (!WARN_ON(!abc_buffer->sg_table)) {
		for_each_sg(abc_buffer->sg_table->sgl, sg,
			    abc_buffer->sg_table->nents, i)
			len += sg_dma_len(sg);
		gasket_page_table_unmap(
			gasket_dev->page_table[ibuf.page_table_index],
			ibuf.device_address, len / PAGE_SIZE);
		/*
		 * Drop reference for the mapping, warn and recover if
		 * unbalanced.
		 */
		if (WARN_ON(refcount_dec_and_test(&abc_buffer->refcount)))
			refcount_set(&abc_buffer->refcount, 1);
	}
	mutex_unlock(&abc_buffer->mapping_lock);
	abc_put_buffer(abc_buffer);
	return 0;
}

static void *oscar_abc_subtable_alloc(struct gasket_dev *gasket_dev,
				      struct gasket_dev_subtable *devsubtable)
{
	struct dma_buf *abc_dma_buf;
	struct dma_buf_attachment *dma_buf_attachment;
	struct abc_subtable *abc_subtable;
	void *ret;

	abc_dma_buf = ab_dram_alloc_dma_buf_kernel(PAGE_SIZE);
	if (IS_ERR(abc_dma_buf))
		return abc_dma_buf;

	abc_subtable = kzalloc(sizeof(*abc_subtable), GFP_KERNEL);
	if (!abc_subtable) {
		ret = ERR_PTR(-ENOMEM);
		goto free_dma_buf;
	}

	dma_buf_attachment =
		dma_buf_attach(abc_dma_buf, gasket_dev->dev);
	if (IS_ERR(dma_buf_attachment)) {
		ret = dma_buf_attachment;
		goto free_metadata;
	}

	abc_subtable->dma_buf = abc_dma_buf;
	abc_subtable->dma_buf_attachment = dma_buf_attachment;
	abc_subtable->sg_table =
		dma_buf_map_attachment(dma_buf_attachment, DMA_TO_DEVICE);
	if (IS_ERR(abc_subtable->sg_table)) {
		ret = abc_subtable->sg_table;
		goto detach_dma_buf;
	}

	devsubtable->driver_data = abc_subtable;
	return (void *)sg_dma_address(abc_subtable->sg_table->sgl);

detach_dma_buf:
	dma_buf_detach(abc_dma_buf, dma_buf_attachment);
free_metadata:
	kfree(abc_subtable);
free_dma_buf:
	ab_dram_free_dma_buf_kernel(abc_dma_buf);
	return ret;
}

static void oscar_abc_subtable_dealloc(struct gasket_dev *gasket_dev,
				       struct gasket_dev_subtable *devsubtable)
{
	struct abc_subtable *abc_subtable = devsubtable->driver_data;

	dma_buf_unmap_attachment(abc_subtable->dma_buf_attachment,
				 abc_subtable->sg_table, DMA_TO_DEVICE);
	dma_buf_detach(abc_subtable->dma_buf,
		       abc_subtable->dma_buf_attachment);
	ab_dram_free_dma_buf_kernel(abc_subtable->dma_buf);
	kfree(abc_subtable);
}

static void *oscar_abc_subtable_map(struct gasket_dev *gasket_dev,
				    struct gasket_dev_subtable *devsubtable)
{
	struct abc_subtable *abc_subtable = devsubtable->driver_data;
	uint64_t ab_addr = sg_dma_address(abc_subtable->sg_table->sgl);
	int ret;

	ret = abc_pcie_map_bar_region(gasket_dev->dma_dev, gasket_dev->dev,
				      BAR_2, PAGE_SIZE, ab_addr,
				      &abc_subtable->bar_mapping);
	if (ret)
		return ERR_PTR(ret);

	return abc_subtable->bar_mapping.bar_vaddr;
}

static void oscar_abc_subtable_unmap(struct gasket_dev *gasket_dev,
				     struct gasket_dev_subtable *devsubtable)
{
	struct abc_subtable *abc_subtable = devsubtable->driver_data;

	abc_pcie_unmap_bar_region(gasket_dev->dma_dev, gasket_dev->dev,
				  &abc_subtable->bar_mapping);
}

static void *oscar_subtable_manage_cb(struct gasket_dev *gasket_dev,
				      enum gasket_dev_subtable_action action,
				      struct gasket_dev_subtable *devsubtable)
{
	void *ret = NULL;

	switch (action) {
	case GASKET_DEV_SUBTABLE_ALLOC:
		ret = oscar_abc_subtable_alloc(gasket_dev, devsubtable);
		break;
	case GASKET_DEV_SUBTABLE_DEALLOC:
		oscar_abc_subtable_dealloc(gasket_dev, devsubtable);
		break;
	case GASKET_DEV_SUBTABLE_MAP_TO_CPU:
		ret = oscar_abc_subtable_map(gasket_dev, devsubtable);
		break;
	case GASKET_DEV_SUBTABLE_UNMAP_FROM_CPU:
		oscar_abc_subtable_unmap(gasket_dev, devsubtable);
		break;
	default:
		ret = ERR_PTR(-ENOTTY);
		break;
	}

	return ret;
}

/* Enters GCB reset state. */
static int oscar_enter_reset(struct gasket_dev *gasket_dev)
{
	struct oscar_dev *oscar_dev =
		platform_get_drvdata(gasket_dev->platform_dev);
	struct abc_buffer *abc_buffer;
	struct abc_buffer *buftemp;

	mutex_lock(&oscar_dev->abc_buffers_lock);
	list_for_each_entry_safe(abc_buffer, buftemp, &oscar_dev->abc_buffers,
				 abc_buffers_list) {
		mutex_lock(&abc_buffer->mapping_lock);
		oscar_abc_release_buffer(abc_buffer);
	}
	mutex_unlock(&oscar_dev->abc_buffers_lock);

	if (!check_dev_avail(oscar_dev))
		return -EIO;

	/* 1. Check whether we are already in reset to guard HIB access. */
	if (gasket_dev_read_64(gasket_dev, OSCAR_BAR_INDEX,
			       OSCAR_BAR_REG_AON_RESET) == 0) {
		/* 1a. Enable DMA Pause. */
		gasket_dev_write_64(gasket_dev, 1, OSCAR_BAR_INDEX,
				    OSCAR_BAR_REG_USER_HIB_DMA_PAUSE);
		/* 1b. Wait for DMA Pause to complete. */
		if (gasket_wait_with_reschedule(gasket_dev, OSCAR_BAR_INDEX,
						OSCAR_BAR_REG_USER_HIB_DMA_PAUSED,
						1, 1, OSCAR_RESET_RETRY,
						OSCAR_RESET_DELAY)) {
			dev_err(gasket_dev->dev,
				"DMA pause failed after timeout (%d ms)\n",
				OSCAR_RESET_RETRY * OSCAR_RESET_DELAY);
			return -ETIMEDOUT;
		}
	}

	/* 2. Enable Quiesce. */
	gasket_dev_write_64(gasket_dev, 1, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_FORCE_QUIESCE);

	/*
	 *  3. Disable Clock Enable.
	 *  - clock_enable = 0.
	 *  - cb_idle_override = 1.
	 */
	gasket_dev_write_64(gasket_dev, 2, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_CLOCK_ENABLE);

	/* 4. Enable memory shutdown, read reg to force write to commit. */
	gasket_dev_write_64(gasket_dev, 0x1ffff, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_MEM_SHUTDOWN);
	gasket_dev_read_64(gasket_dev, OSCAR_BAR_INDEX,
			   OSCAR_BAR_REG_AON_MEM_SHUTDOWN);

	/*
	 * 5. Enable clock, to allow reset to propagate, read to force commit.
	 *  - clock_enable = 1.
	 *  - cb_idle_override = 1.
	 */
	gasket_dev_write_64(gasket_dev, 3, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_CLOCK_ENABLE);
	gasket_dev_read_64(gasket_dev, OSCAR_BAR_INDEX,
			   OSCAR_BAR_REG_AON_CLOCK_ENABLE);

	/* 6. Enable Reset. */
	gasket_dev_write_64(gasket_dev, 1, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_RESET);

	/*
	 *  7. Disable clock, read to force commit.
	 *  - clock_enable = 0.
	 *  - cb_idle_override = 1.
	 */
	gasket_dev_write_64(gasket_dev, 2, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_CLOCK_ENABLE);
	gasket_dev_read_64(gasket_dev, OSCAR_BAR_INDEX,
			   OSCAR_BAR_REG_AON_CLOCK_ENABLE);

	/* 8. Enable Clamp. */
	gasket_dev_write_64(gasket_dev, 0x1ffff, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_CLAMP_ENABLE);

	/* 9. Disable tile control for deepsleep, read to force commit. */
	gasket_dev_write_64(gasket_dev, 0xffff, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_TILEMEM_RETN);
	gasket_dev_read_64(gasket_dev, OSCAR_BAR_INDEX,
			   OSCAR_BAR_REG_AON_TILEMEM_RETN);

	/* 10. Enable second pass logic shutdown, read to force commit. */
	gasket_dev_write_64(gasket_dev, 0x1ffff, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_LOGIC_SHUTDOWN_ALL);
	gasket_dev_read_64(gasket_dev, OSCAR_BAR_INDEX,
			   OSCAR_BAR_REG_AON_LOGIC_SHUTDOWN_ALL);

	/* 11. Enable first pass logic shutdown, read to force commit. */

	gasket_dev_write_64(gasket_dev, 0x1ffff, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_LOGIC_SHUTDOWN_PRE);
	gasket_dev_read_64(gasket_dev, OSCAR_BAR_INDEX,
			   OSCAR_BAR_REG_AON_LOGIC_SHUTDOWN_PRE);
	return 0;
}

/* Called on final close via device_close_cb. */
static int oscar_device_cleanup(struct gasket_dev *gasket_dev)
{
	return oscar_enter_reset(gasket_dev);
}

/*
 * Avoid TPU memory corruption problems at power up by temporarily ungating
 * TPU FRC_CLK and then re-gating.
 */
static void tpu_frc_clk_ungate_gate(void)
{
	uint32_t reg;

	abc_pcie_config_read(TPU_POWER_CTL_OVERRIDE_0, 4, &reg);
	/*
	 * Assert iolation for each tile (TILE_ISOLATION_EN), leave
	 * FRC_CLKGATE_EN set.
	 */
	reg &= ~(TILE_ISOLATION_EN);
	abc_pcie_config_write(TPU_POWER_CTL_OVERRIDE_0, 4, reg);
	/*
	 * Set OVERRIDE_ENABLE, which overrides the internal clock gate enable
	 * signal and drives it to FRC_CLKGATE_EN (which is enabled).  This
	 * allows some registers that come up in an unknown state to be flushed
	 * and prevents them from corrupting memory retention registers.
	 */
	abc_pcie_config_read(TPU_POWER_CTL_OVERRIDE_0, 4, &reg);
	reg |= TPU_POWER_CTL_OVERRIDE_ENABLE;
	abc_pcie_config_write(TPU_POWER_CTL_OVERRIDE_0, 4, reg);
	udelay(1);
	/*
	 * Clear OVERRIDE_ENABLE, which stops overriding the clock gate enable,
	 * thereby disabling the clock again.
	 */
	abc_pcie_config_read(TPU_POWER_CTL_OVERRIDE_0, 4, &reg);
	reg &= ~(TPU_POWER_CTL_OVERRIDE_ENABLE);
	abc_pcie_config_write(TPU_POWER_CTL_OVERRIDE_0, 4, reg);
}

/* Quits GCB reset state. */
static int oscar_quit_reset(struct gasket_dev *gasket_dev)
{
	uint64_t reg;

	/* 1. Enable logic shutdown. */
	/* 1a. Disable first pass switches, wait until ack matches. */
	gasket_dev_write_64(gasket_dev, 0, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_LOGIC_SHUTDOWN_PRE);
	if (gasket_wait_with_reschedule(gasket_dev, OSCAR_BAR_INDEX,
					OSCAR_BAR_REG_AON_LOGIC_SHUTDOWN_PRE,
					MASK_AON_LOGIC_SHUTDOWN_ACK, 0,
					OSCAR_RESET_RETRY, OSCAR_ACK_DELAY)) {
		dev_err(gasket_dev->dev,
			"Logic shutdown pre failed after timeout (%d ms)\n",
			OSCAR_RESET_RETRY * OSCAR_ACK_DELAY);
		return -ETIMEDOUT;
	}

	/* 1b. Disable second pass switches, wait until ack matches. */
	gasket_dev_write_64(gasket_dev, 0, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_LOGIC_SHUTDOWN_ALL);
	if (gasket_wait_with_reschedule(gasket_dev, OSCAR_BAR_INDEX,
					OSCAR_BAR_REG_AON_LOGIC_SHUTDOWN_ALL,
					MASK_AON_LOGIC_SHUTDOWN_ACK, 0,
					OSCAR_RESET_RETRY, OSCAR_ACK_DELAY)) {
		dev_err(gasket_dev->dev,
			"Logic shutdown all failed after timeout (%d ms)\n",
			OSCAR_RESET_RETRY * OSCAR_ACK_DELAY);
		return -ETIMEDOUT;
	}

	/*
	 * 2. Enable Clock Enable, and set idle_override to force the clock on.
	 *  - clock_enable = 1.
	 *  - cb_idle_override = 1.
	 */
	gasket_dev_write_64(gasket_dev, 3, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_CLOCK_ENABLE);

	/* 3. Read clock enable to force previous write to commit. */
	gasket_dev_read_64(gasket_dev, OSCAR_BAR_INDEX,
			   OSCAR_BAR_REG_AON_CLOCK_ENABLE);

	/*
	 * 4. Disable Clock Enable.
	 *  - clock_enable = 0.
	 *  - cb_idle_override = 1.
	 */
	gasket_dev_write_64(gasket_dev, 2, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_CLOCK_ENABLE);

	/* 5. Read clock enable to force previous write to commit. */
	gasket_dev_read_64(gasket_dev, OSCAR_BAR_INDEX,
			   OSCAR_BAR_REG_AON_CLOCK_ENABLE);

	/* 6. Disable memory shutdown, wait for ack. */
	gasket_dev_write_64(gasket_dev, 0, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_MEM_SHUTDOWN);
	if (gasket_wait_with_reschedule(gasket_dev, OSCAR_BAR_INDEX,
					OSCAR_BAR_REG_AON_MEM_SHUTDOWN_ACK,
					MASK_AON_MEM_SHUTDOWN_ACK, 0,
					OSCAR_RESET_RETRY, OSCAR_ACK_DELAY)) {
		dev_err(gasket_dev->dev,
			"Memory shutdown failed after timeout (%d ms)\n",
			OSCAR_RESET_RETRY * OSCAR_ACK_DELAY);
		return -ETIMEDOUT;
	}

	/* 7. Enable tile retention when memory powered down. */
	gasket_dev_write_64(gasket_dev, 0, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_TILEMEM_RETN);

	/*
	 * 8. Enable clock, with dynamic activity based clock gating.
	 *  - clock_enable = 1.
	 *  - cb_idle_override = 0.
	 */
	gasket_dev_write_64(gasket_dev, 1, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_CLOCK_ENABLE);

	/*
	 * 8a. Temporarily ungate FRC_CLK to force consistent memory retention
	 * registers state.
	 */
	tpu_frc_clk_ungate_gate();

	/* 9. Disable Clamp. */
	gasket_dev_write_64(gasket_dev, 0, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_CLAMP_ENABLE);

	/* 10. Disable Reset. */
	gasket_dev_write_64(gasket_dev, 0, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_RESET);

	/* 11. Disable Quiesce. */
	gasket_dev_write_64(gasket_dev, 0, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_FORCE_QUIESCE);

	/*
	 * 12. Confirm moved out of reset by reading any CSR with a known
	 * initial value.  Scalar core run control must be zero.
	 */
	if (gasket_wait_with_reschedule(gasket_dev, OSCAR_BAR_INDEX,
					OSCAR_BAR_REG_SC_RUNCTRL,
					MASK_SC_RUNCTRL_TRIGGER, 0,
					OSCAR_RESET_RETRY, OSCAR_ACK_DELAY)) {
		dev_err(gasket_dev->dev,
			"Wait for reset confirm failed after timeout (%d ms)\n",
			OSCAR_RESET_RETRY * OSCAR_ACK_DELAY);
		return -ETIMEDOUT;
	}

	/*
	 * Set tpu_aon register axiTieOffsReg fields arcache and awcache
	 * bit 1, allowing DRAM controller to combine our 16B transactions into
	 * 32B transactions to get full bandwidth.
	 * TODO: May not be necessary on later Airbrush cards.
	 */
	reg = gasket_dev_read_64(gasket_dev, OSCAR_BAR_INDEX,
				 OSCAR_BAR_REG_AON_AXITIEOFFS);
	reg |= 0x2 << 18 | 0x2 << 2;
	gasket_dev_write_64(gasket_dev, reg, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_AXITIEOFFS);
	return 0;
}

static int oscar_reset(struct gasket_dev *gasket_dev)
{
	int ret = 0;

	ret = oscar_enter_reset(gasket_dev);
	if (ret < 0)
		return ret;
	return oscar_quit_reset(gasket_dev);
}

static uint oscar_ioctl_check_permissions(struct file *filp, uint cmd)
{
	return !!(filp->f_mode & FMODE_WRITE);
}

static long oscar_ioctl(struct file *filp, uint cmd, void __user *argp)
{
	struct gasket_dev *gasket_dev = filp->private_data;
	struct oscar_dev *oscar_dev =
		platform_get_drvdata(gasket_dev->platform_dev);
	struct oscar_abdram_map_ioctl __user *abdram_map;

	if (!oscar_ioctl_check_permissions(filp, cmd))
		return -EPERM;

	switch (cmd) {
	case OSCAR_IOCTL_ABC_MAP_ABDRAM:
		abdram_map = (struct oscar_abdram_map_ioctl *)argp;
		return oscar_abc_map_buffer(oscar_dev, abdram_map);
	case OSCAR_IOCTL_ABC_UNMAP_ABDRAM:
		abdram_map = (struct oscar_abdram_map_ioctl *)argp;
		return oscar_abc_unmap_abdram(oscar_dev, abdram_map);
	default:
		return -ENOTTY; /* unknown command */
	}
}

static ssize_t sysfs_show(struct device *device, struct device_attribute *attr,
			  char *buf)
{
	int ret;
	struct gasket_dev *gasket_dev;
	struct gasket_sysfs_attribute *gasket_attr;
	enum sysfs_attribute_type type;

	gasket_dev = gasket_sysfs_get_device_data(device);
	if (!gasket_dev) {
		dev_err(device, "No Gasket device sysfs mapping found\n");
		return -ENODEV;
	}

	gasket_attr = gasket_sysfs_get_attr(device, attr);
	if (!gasket_attr) {
		dev_err(device, "No Gasket device sysfs attr data found\n");
		gasket_sysfs_put_device_data(device, gasket_dev);
		return -ENODEV;
	}

	type = (enum sysfs_attribute_type)gasket_attr->data.attr_type;
	switch (type) {
	case ATTR_KERNEL_HIB_PAGE_TABLE_SIZE:
		ret = scnprintf(buf, PAGE_SIZE, "%u\n",
				gasket_page_table_num_entries(
					gasket_dev->page_table[0]));
		break;
	case ATTR_KERNEL_HIB_SIMPLE_PAGE_TABLE_SIZE:
		ret = scnprintf(buf, PAGE_SIZE, "%u\n",
				gasket_page_table_num_entries(
					gasket_dev->page_table[0]));
		break;
	case ATTR_KERNEL_HIB_NUM_ACTIVE_PAGES:
		ret = scnprintf(buf, PAGE_SIZE, "%u\n",
				gasket_page_table_num_active_pages(
					gasket_dev->page_table[0]));
		break;
	default:
		dev_dbg(gasket_dev->dev, "Unknown attribute: %s\n",
			attr->attr.name);
		ret = 0;
		break;
	}

	gasket_sysfs_put_attr(device, gasket_attr);
	gasket_sysfs_put_device_data(device, gasket_dev);
	return ret;
}

static struct gasket_sysfs_attribute oscar_sysfs_attrs[] = {
	GASKET_SYSFS_RO(node_0_page_table_entries, sysfs_show,
		ATTR_KERNEL_HIB_PAGE_TABLE_SIZE),
	GASKET_SYSFS_RO(node_0_simple_page_table_entries, sysfs_show,
		ATTR_KERNEL_HIB_SIMPLE_PAGE_TABLE_SIZE),
	GASKET_SYSFS_RO(node_0_num_mapped_pages, sysfs_show,
		ATTR_KERNEL_HIB_NUM_ACTIVE_PAGES),
	GASKET_END_OF_ATTR_ARRAY
};

static struct gasket_driver_desc oscar_gasket_desc = {
	.name = DRIVER_NAME,
	.driver_version = DRIVER_VERSION,
	.major = 120,
	.minor = 0,
	.module = THIS_MODULE,

	.num_page_tables = NUM_NODES,
	.page_table_bar_index = OSCAR_BAR_INDEX,
	.page_table_configs = oscar_page_table_configs,
	.page_table_extended_bit = OSCAR_EXTENDED_SHIFT,

	.bar_descriptions = {
		{
			OSCAR_BAR_SIZE,
			(VM_WRITE | VM_READ),
			OSCAR_BAR_OFFSET,
			NUM_BAR_RANGES,
			oscar_mappable_regions,
			PCI_BAR
		},
		GASKET_UNUSED_BAR,
		GASKET_UNUSED_BAR,
		GASKET_UNUSED_BAR,
		GASKET_UNUSED_BAR,
		GASKET_UNUSED_BAR,
	},
	.coherent_buffer_description = {
		OSCAR_CH_MEM_BYTES,
		(VM_WRITE | VM_READ),
		OSCAR_CM_OFFSET,
	},
	.interrupt_type = DEVICE_MANAGED,
	.num_interrupts = OSCAR_N_INTS,
	.interrupts = oscar_interrupts,

	.device_open_cb = oscar_device_open_cb,
	.device_close_cb = oscar_device_cleanup,
	.ioctl_handler_cb = oscar_ioctl,
	.device_status_cb = oscar_get_status,
	.hardware_revision_cb = NULL,
	.device_reset_cb = oscar_reset,
	.subtable_manage_cb = oscar_subtable_manage_cb,
};

static irqreturn_t oscar_interrupt_handler(int irq, void *arg)
{
	struct oscar_dev *oscar_dev = (struct oscar_dev *)arg;
	struct gasket_interrupt_data *interrupt_data =
		oscar_dev->gasket_dev->interrupt_data;
	int i;

	/* Map this IRQ to a TPU logical interrupt, send to gasket */
	for (i = 0; i < OSCAR_N_IRQS; i++) {
		if (irq == oscar_dev->irqs[i]) {
			gasket_handle_interrupt(interrupt_data, i);
			return IRQ_HANDLED;
		}
	}

	return IRQ_NONE;
}

static int oscar_lowprio_irq_notify(struct notifier_block *nb,
				    unsigned long irq, void *data)
{
	struct oscar_dev *oscar_dev =
	    container_of(nb, struct oscar_dev, lowprio_irq_nb);
	struct gasket_interrupt_data *interrupt_data =
		oscar_dev->gasket_dev->interrupt_data;
	u32 intnc_val = (u64)data;

	if (irq == ABC_MSI_AON_INTNC &&
	    (intnc_val & 1 << INTNC_TPU_WIREINTERRUPT2))
		gasket_handle_interrupt(interrupt_data, OSCAR_LOWPRIO_INT);
	return NOTIFY_OK;
}

static int oscar_ab_sm_clk_listener(struct notifier_block *nb,
				    unsigned long action, void *data)
{
	struct oscar_dev *oscar_dev =
		container_of(nb, struct oscar_dev, clk_change_nb);

	switch (action) {
	case AB_TPU_PRE_RATE_CHANGE:
		mutex_lock(&oscar_dev->dev_avail_lock);
		oscar_dev->clk_lockout = true; /* dev unavail while changing */
		mutex_unlock(&oscar_dev->dev_avail_lock);
		break;
	case AB_TPU_POST_RATE_CHANGE:
	case AB_TPU_ABORT_RATE_CHANGE:
		mutex_lock(&oscar_dev->dev_avail_lock);
		oscar_dev->clk_lockout = false;
		mutex_unlock(&oscar_dev->dev_avail_lock);
		break;
	default:
		return NOTIFY_DONE;
	}
	return NOTIFY_OK;
}

static int oscar_abc_pcie_link_change_listener(
	struct notifier_block *nb, unsigned long action, void *data)
{
	struct oscar_dev *oscar_dev =
		container_of(nb, struct oscar_dev, pcie_link_change_nb);

	if (action & (ABC_PCIE_LINK_PRE_DISABLE | ABC_PCIE_LINK_ERROR)) {
		mutex_lock(&oscar_dev->dev_avail_lock);
		oscar_dev->pcie_lockout = true; /* PCIe link unavailable */
		mutex_unlock(&oscar_dev->dev_avail_lock);
		return NOTIFY_OK;
	}

	if (action & ABC_PCIE_LINK_POST_ENABLE) {
		mutex_lock(&oscar_dev->dev_avail_lock);
		oscar_dev->pcie_lockout = false;
		mutex_unlock(&oscar_dev->dev_avail_lock);
		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}

static void oscar_interrupt_cleanup(struct oscar_dev *oscar_dev)
{
	int ret;

	ret = atomic_notifier_chain_unregister(oscar_dev->lowprio_irq_nh,
					       &oscar_dev->lowprio_irq_nb);
	if (ret)
		dev_warn(oscar_dev->gasket_dev->dev,
			 "Unregister lowprio irq notifier failed: %d\n", ret);
}

static struct dumpregs_range {
	enum oscar_bar_regs firstreg;
	enum oscar_bar_regs lastreg;
} global_statusregs_ranges[] = {
	{
		.firstreg = OSCAR_BAR_REG_AON_RESET,
		.lastreg = OSCAR_BAR_REG_AON_AXIBRIDGEDEBUG1,
	},
	{
		.firstreg = OSCAR_BAR_REG_USER_HIB_DMA_PAUSED,
		.lastreg = OSCAR_BAR_REG_USER_HIB_DMA_PAUSED,
	},
	{
		.firstreg = OSCAR_BAR_REG_USER_HIB_ERROR_STATUS,
		.lastreg = OSCAR_BAR_REG_USER_HIB_FIRST_ERROR_STATUS,
	},
	{
		.firstreg = OSCAR_BAR_REG_USER_HIB_PAGE_FAULT_ADDR,
		.lastreg = OSCAR_BAR_REG_USER_HIB_PAGE_FAULT_ADDR,
	},
	{
		.firstreg = OSCAR_BAR_REG_SC_RUNSTATUS,
		.lastreg = OSCAR_BAR_REG_SC_ERROR_MASK,
	},
	{
		.firstreg = OSCAR_BAR_REG_SC_ERROR_INFO,
		.lastreg = OSCAR_BAR_REG_SC_ERROR_INFO,
	},
};

static struct dumpregs_range tile_statusregs_ranges[] = {
	{
		.firstreg = OSCAR_BAR_REG_TILECONF1_DEEPSLEEP,
		.lastreg = OSCAR_BAR_REG_TILECONF1_DEEPSLEEP,
	},
	{
		.firstreg = OSCAR_BAR_REG_TILECONF1_ERROR_TILE,
		.lastreg = OSCAR_BAR_REG_TILECONF1_ERROR_MASK_TILE,
	},
	{
		.firstreg = OSCAR_BAR_REG_TILECONF1_ERROR_INFO_TILE,
		.lastreg = OSCAR_BAR_REG_TILECONF1_ERROR_INFO_TILE,
	},
};

static void dump_statusregs_ranges(struct seq_file *s,
				   struct oscar_dev *oscar_dev,
				   struct dumpregs_range *ranges,
				   int nranges)
{
	int i;
	enum oscar_bar_regs reg;
	uint64_t val;

	for (i = 0; i < nranges; i++) {
		for (reg = ranges[i].firstreg; reg <= ranges[i].lastreg;
		     reg += sizeof(val)) {
			val = gasket_dev_read_64(oscar_dev->gasket_dev,
						 OSCAR_BAR_INDEX, reg);
			seq_printf(s, "0x%08lx: 0x%016llx\n", reg, val);
		}
	}
}

static int statusregs_show(struct seq_file *s, void *data)
{
	struct oscar_dev *oscar_dev = s->private;
	int tileid;

	if (!check_dev_avail(oscar_dev))
		return -EIO;

	dump_statusregs_ranges(s, oscar_dev, global_statusregs_ranges,
			       ARRAY_SIZE(global_statusregs_ranges));
	for (tileid = 0; tileid <= NTILES; tileid++) {
		gasket_dev_write_64(oscar_dev->gasket_dev, tileid,
				    OSCAR_BAR_INDEX,
				    OSCAR_BAR_REG_USER_HIB_TILECONFIG1);
		seq_printf(s, "tile %d:\n", tileid);
		dump_statusregs_ranges(s, oscar_dev, tile_statusregs_ranges,
				       ARRAY_SIZE(tile_statusregs_ranges));
	}
	return 0;
}

static int statusregs_open(struct inode *inode, struct file *file)
{
	return single_open(file, statusregs_show, inode->i_private);
}

static const struct file_operations statusregs_ops = {
	.open = statusregs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.owner = THIS_MODULE,
	.release = single_release,
};

static int mmustatus_show(struct seq_file *s, void *data)
{
	struct oscar_dev *oscar_dev = s->private;

	if (!check_dev_avail(oscar_dev))
		return -EIO;

	gasket_page_table_dump(oscar_dev->gasket_dev, s);
	return 0;
}

static int mmustatus_open(struct inode *inode, struct file *file)
{
	return single_open(file, mmustatus_show, inode->i_private);
}

static const struct file_operations mmustatus_ops = {
	.open = mmustatus_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.owner = THIS_MODULE,
	.release = single_release,
};

static void oscar_setup_debugfs(struct oscar_dev *oscar_dev,
				struct device *dev)
{
	oscar_dev->d_entry =
		debugfs_create_dir(dev_name(dev), NULL);
	if (!oscar_dev->d_entry)
		return;

	debugfs_create_file("statusregs", 0660, oscar_dev->d_entry, oscar_dev,
			    &statusregs_ops);
	debugfs_create_file("mmustatus", 0660, oscar_dev->d_entry, oscar_dev,
			    &mmustatus_ops);
}

static int oscar_setup_device(struct platform_device *pdev,
			      struct oscar_dev *oscar_dev,
			      struct gasket_dev *gasket_dev)
{
	struct resource *r;
	int irq;
	ulong page_table_ready;
	int retries = 0;
	struct device *dev = &pdev->dev;
	phys_addr_t mem_phys;
	resource_size_t mem_size;
	void *mem_virt = NULL;
	u64 u64prop;
	int ret;

	/* Use the parent abc device for DMA, which has the IOMMU config */
	gasket_set_dma_device(gasket_dev, pdev->dev.parent);
	/*
	 * The memory resource passes the physical address range of our
	 * memory region in the associated BAR of the parent mfd device.
	 */
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(dev, "cannot get mem resource\n");
		return -ENODEV;
	}
	mem_phys = r->start;
	mem_size = resource_size(r);

	/*
	 * If the parent device has already mapped our region (to handle
	 * multi-function device-specific init actions) then it passes the
	 * virtual address of the I/O remapping to us in property
	 * "tpu-mem-mapping".
	 */

	ret = device_property_read_u64(dev, "tpu-mem-mapping", &u64prop);
	if (!ret)
		mem_virt = (void *)u64prop;

	if (mem_virt) {
		oscar_dev->parent_ioremap = true;
	} else {
		dev_info(dev, "no tpu-mem-mapping from parent, remapping\n");
		mem_virt = ioremap_nocache(mem_phys, mem_size);
		if (!mem_virt) {
			dev_err(dev, "failed to map our memory region\n");
			return -ENODEV;
		}
	}

	gasket_dev->bar_data[OSCAR_BAR_INDEX].phys_base = mem_phys;
	gasket_dev->bar_data[OSCAR_BAR_INDEX].length_bytes = mem_size;
	gasket_dev->bar_data[OSCAR_BAR_INDEX].virt_base = mem_virt;

	irq = platform_get_irq_byname(pdev, SC_0_IRQ_NAME);
	if (irq < 0) {
		dev_err(dev, "cannot get %s\n", SC_0_IRQ_NAME);
		return -ENODEV;
	}
	oscar_dev->irqs[OSCAR_SCALAR_CORE_0_INT] = irq;
	ret = devm_request_irq(dev, irq, oscar_interrupt_handler, IRQF_ONESHOT,
			       SC_0_IRQ_NAME, oscar_dev);
	if (ret) {
		dev_err(dev, "failed to request irq %d\n", irq);
		return ret;
	}

	irq = platform_get_irq_byname(pdev, INST_QUEUE_IRQ_NAME);
	if (irq < 0) {
		dev_err(dev, "cannot get %s\n", INST_QUEUE_IRQ_NAME);
		return -ENODEV;
	}
	oscar_dev->irqs[OSCAR_INSTR_QUEUE_INT] = irq;
	ret = devm_request_irq(dev, irq, oscar_interrupt_handler, IRQF_ONESHOT,
			       INST_QUEUE_IRQ_NAME, oscar_dev);
	if (ret) {
		dev_err(dev, "failed to request irq %d\n", irq);
		return -ENODEV;
	}

	ret = device_property_read_u64(dev, "intnc-notifier-chain", &u64prop);
	if (!ret)
		oscar_dev->lowprio_irq_nh =
		    (struct atomic_notifier_head *)u64prop;
	if (oscar_dev->lowprio_irq_nh) {
		oscar_dev->lowprio_irq_nb.notifier_call =
		    oscar_lowprio_irq_notify;
		ret = atomic_notifier_chain_register(oscar_dev->lowprio_irq_nh,
						    &oscar_dev->lowprio_irq_nb);
		if (ret)
			dev_warn(dev,
				 "Cannot register notifier for lowprio irq\n");
	} else {
		dev_warn(dev,
			 "no intnc non-critical irq notifier supplied\n");
	}

	oscar_dev->clk_change_nb.notifier_call = oscar_ab_sm_clk_listener;
	ret = ab_sm_register_clk_event(&oscar_dev->clk_change_nb);
	if (ret)
		dev_warn(dev, "failed to subscribe to clk event (%d)\n", ret);

	oscar_dev->pcie_link_change_nb.notifier_call =
		oscar_abc_pcie_link_change_listener;
	ret = abc_register_pcie_link_blocking_event(
			&oscar_dev->pcie_link_change_nb);
	if (ret)
		dev_warn(dev,
			 "failed to subscribe to pcie link block event (%d)\n",
			 ret);

	oscar_reset(gasket_dev);

	while (retries < OSCAR_RESET_RETRY) {
		page_table_ready =
		    gasket_dev_read_64(gasket_dev, OSCAR_BAR_INDEX,
				       OSCAR_BAR_REG_HIB_PAGE_TABLE_INIT);
		if (page_table_ready)
			break;
		schedule_timeout(msecs_to_jiffies(OSCAR_RESET_DELAY));
		retries++;
	}

	if (retries == OSCAR_RESET_RETRY) {
		dev_err(gasket_dev->dev, "Page table init timed out\n");
		return -ETIMEDOUT;
	}

	oscar_setup_debugfs(oscar_dev, &pdev->dev);
	return 0;
}

static int oscar_probe(struct platform_device *pdev)
{
	int ret;
	struct oscar_dev *oscar_dev;
	struct gasket_dev *gasket_dev;
	struct device *dev = &pdev->dev;

	ret = gasket_platform_add_device(pdev, &gasket_dev);
	if (ret) {
		dev_err(dev, "error adding gasket device\n");
		return ret;
	}

	oscar_dev = kmalloc(sizeof(struct oscar_dev), GFP_KERNEL);
	if (!oscar_dev) {
		ret = -ENOMEM;
		goto remove_device;
	}

	platform_set_drvdata(pdev, oscar_dev);
	oscar_dev->gasket_dev = gasket_dev;
	INIT_LIST_HEAD(&oscar_dev->abc_buffers);
	mutex_init(&oscar_dev->abc_buffers_lock);
	mutex_init(&oscar_dev->dev_avail_lock);
	oscar_dev->clk_lockout = false;
	oscar_dev->pcie_lockout = false;

	ret = oscar_setup_device(pdev, oscar_dev, gasket_dev);
	if (ret) {
		dev_err(dev, "Setup device failed\n");
		goto remove_device;
	}

	ret = gasket_sysfs_create_entries(gasket_dev->dev_info.device,
					  oscar_sysfs_attrs);
	if (ret)
		dev_err(dev, "error creating device sysfs entries\n");

	ret = gasket_enable_device(gasket_dev);
	if (ret) {
		dev_err(dev, "error enabling gasket device\n");
		goto remove_device;
	}

	/* Place device in low power mode until opened */
	oscar_enter_reset(gasket_dev);
	return 0;

remove_device:
	gasket_platform_remove_device(pdev);
	kfree(oscar_dev);
	return ret;
}

static int oscar_remove(struct platform_device *pdev)
{
	struct oscar_dev *oscar_dev = platform_get_drvdata(pdev);
	struct gasket_dev *gasket_dev = oscar_dev->gasket_dev;

	ab_sm_unregister_clk_event(&oscar_dev->clk_change_nb);
	abc_unregister_pcie_link_blocking_event(
		&oscar_dev->pcie_link_change_nb);
	debugfs_remove_recursive(oscar_dev->d_entry);
	gasket_disable_device(gasket_dev);
	oscar_interrupt_cleanup(oscar_dev);
	if (!oscar_dev->parent_ioremap)
		iounmap(gasket_dev->bar_data[OSCAR_BAR_INDEX].virt_base);
	gasket_platform_remove_device(pdev);
	kfree(oscar_dev);
	return 0;
}

static struct platform_driver oscar_platform_driver = {
		.probe = oscar_probe,
		.remove = oscar_remove,
		.driver = {
			.name = DRIVER_NAME,
		}
};

static int __init oscar_init(void)
{
	int ret;

	ret = platform_driver_register(&oscar_platform_driver);
	if (!ret)
		ret = gasket_register_device(&oscar_gasket_desc);
	if (ret)
		platform_driver_unregister(&oscar_platform_driver);
	return ret;
}

static void __exit oscar_exit(void)
{
	gasket_unregister_device(&oscar_gasket_desc);
	platform_driver_unregister(&oscar_platform_driver);
}

MODULE_DESCRIPTION("Google Oscar driver");
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("John Joseph <jnjoseph@google.com>");
module_init(oscar_init);
module_exit(oscar_exit);
