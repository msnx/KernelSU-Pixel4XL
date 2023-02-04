/*
 * Airbrush MFD adapter driver for the Paintbox programmable IPU
 *
 * Copyright (C) 2018 Google, Inc.
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

#include <linux/ab-dram.h>
#include <linux/mfd/abc-pcie.h>
#include <linux/mfd/abc-pcie-dma.h>
#include <linux/airbrush-sm-notifier.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/mfd/abc-pcie-notifier.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include "ipu-adapter.h"
#include "ipu-core-jqs-msg-transport.h"
#include "ipu-core-jqs.h"
#include "ipu-regs.h"

#define IPU_ADAPTER_STATE_PCIE_READY (1 << 0)
#define IPU_ADAPTER_STATE_DRAM_READY (1 << 1)

struct ipu_adapter_shared_buffer {
	struct ipu_shared_buffer base;
	struct dma_buf *ab_dram_dma_buf;
	bool mapped_to_bar;
	/* data in this struct only valid if mapped_to_bar is true */
	struct bar_mapping mapping;
	struct list_head sbuf_entry;
};

struct ipu_adapter_jqs_buffer {
	struct ipu_jqs_buffer base;
	struct dma_buf *ab_dram_dma_buf;
};

#define IPU_ADAPTER_AB_MFD_MAX_INTERRUPTS 2

struct ipu_adapter_ab_mfd_data {
	struct paintbox_bus *bus;
	struct device *dev;
	atomic_t state;
	uint64_t ipu_clock_rate_hz;

	/* Depending on the IOMMU configuration the board the IPU may need
	 * to use MFD parent's device for mapping DMA buffers.  Otherwise, the
	 * dma device can be set to our own device and use SWIOTLB to map
	 * buffers.
	 */
	struct list_head sbuf_list;
	struct device *dma_dev;
	struct platform_device *pdev;
	struct paintbox_pdata pdata;
	struct paintbox_bus_ops ops;
	struct completion dma_completion;
	struct mutex sync_lock;
	size_t pio_threshold;

	struct notifier_block low_priority_irq_nb;
	struct atomic_notifier_head *low_priority_irq_nh;

	struct notifier_block clk_change_nb;
	struct notifier_block pcie_link_blocking_nb;

	int irqs[IPU_ADAPTER_AB_MFD_MAX_INTERRUPTS];
	bool interrupts_enabled;
};

/* Paintbox IO virtual address space bounds */
#define PAINTBOX_IOVA_START		0x200000000
#define PAINTBOX_IOVA_SIZE		0x400000000

/* TODO(b/115433779):  Figure out if there is a way to get this information from
 * the system.
 */
#define IPU_INPUT_ADDR_SIZE	43 /* bits */

/* TODO(b/115433779):  Determine appropriate values for airbrush. */
#define IPU_OUTPUT_ADDR_SIZE	32 /* bits */

/* Android APs usually use 4K pages.  This may change in future versions. */
#define IPU_PAGE_SIZE_BITMAP	(SZ_4K | SZ_2M | SZ_1G)

static inline bool ipu_clock_rate_is_active(uint64_t rate)
{
	return rate > 0;
}

static inline bool ipu_adapter_link_is_ready(
		struct ipu_adapter_ab_mfd_data *dev_data)
{
	return !!(atomic_read(&dev_data->state) & IPU_ADAPTER_STATE_PCIE_READY);
}

static inline bool ipu_adapter_dram_is_ready(
		struct ipu_adapter_ab_mfd_data *dev_data)
{
	return !!(atomic_read(&dev_data->state) & IPU_ADAPTER_STATE_DRAM_READY);
}

static void ipu_adapter_ab_mfd_writel(struct device *dev, uint32_t val,
		unsigned int offset)
{
	int ret;

	ret = ipu_config_write(offset, sizeof(uint32_t), val);

	/* TODO(b/114760293):  An error writing an IPU register should be
	 * considered a catastrophic failure.  The IPU stack should cancel any
	 * running jobs and reset the IPU.
	 */
	if (ret < 0)
		dev_err(dev, "%s: write error offset %u val 0x%08x, ret %d\n",
				__func__, offset, val, ret);

}

static void ipu_adapter_ab_mfd_writeq(struct device *dev, uint64_t val,
		unsigned int offset)
{
	ipu_adapter_ab_mfd_writel(dev, (uint32_t)(val & 0xFFFFFFFF), offset);
	ipu_adapter_ab_mfd_writel(dev, (uint32_t)(val >> 32), offset +
			sizeof(uint32_t));
}

static uint32_t ipu_adapter_ab_mfd_readl(struct device *dev,
		unsigned int offset)
{
	uint32_t val;
	int ret;

	ret = ipu_config_read(offset, sizeof(uint32_t), &val);
	if (ret < 0) {
		/* TODO(b/114760293):  An error writing an IPU register should
		 * be considered a catastrophic failure.  The IPU stack should
		 * cancel any running jobs and reset the IPU.
		 */
		dev_err(dev, "%s: read error offset %u val 0x%08x, ret %d\n",
				__func__, offset, val, ret);
		return 0;
	}

	return val;
}

static uint64_t ipu_adapter_ab_mfd_readq(struct device *dev,
		unsigned int offset)
{
	uint32_t low, high;

	low = ipu_adapter_ab_mfd_readl(dev, offset);
	high = ipu_adapter_ab_mfd_readl(dev, offset + sizeof(uint32_t));

	return (((uint64_t)high) << 32) | low;
}

#define CENTRAL_PMU_BASE		(AON_AXI2APB + 0xA0000) /* 0xBA0000 */
#define PMU_IPU_POWER_CTL_OVERRIDE0	(CENTRAL_PMU_BASE + 0x2400)
#define PMU_IPU_CORE_ISO_EN		(0x7FFF << 2)
#define PMU_IPU_OVERRIDE_EN		(0x1 << 0)

static void ipu_adapter_frc_clock_ungate(struct device *dev)
{
	uint32_t val;

	abc_pcie_config_read(PMU_IPU_POWER_CTL_OVERRIDE0, sizeof(uint32_t),
			&val);

	val &= ~(PMU_IPU_CORE_ISO_EN);
	abc_pcie_config_write(PMU_IPU_POWER_CTL_OVERRIDE0, sizeof(uint32_t),
			val);

	abc_pcie_config_read(PMU_IPU_POWER_CTL_OVERRIDE0, sizeof(uint32_t),
			&val);
	val |= PMU_IPU_OVERRIDE_EN;
	abc_pcie_config_write(PMU_IPU_POWER_CTL_OVERRIDE0, sizeof(uint32_t),
			val);

	udelay(1);

	abc_pcie_config_read(PMU_IPU_POWER_CTL_OVERRIDE0, sizeof(uint32_t),
			&val);
	val &= ~(PMU_IPU_OVERRIDE_EN);
	abc_pcie_config_write(PMU_IPU_POWER_CTL_OVERRIDE0, sizeof(uint32_t),
			val);
}

static int ipu_adapter_ab_mfd_map_to_bar(struct device *dev,
		struct ipu_adapter_shared_buffer *shared_buffer)
{
	struct ipu_shared_buffer *shared_buffer_base = &shared_buffer->base;
	int ret;

	if (shared_buffer->mapped_to_bar) {
		dev_warn(dev, "%s: shared buffer already mapped to bar\n",
				__func__);
		return -EINVAL;
	}

	ret = abc_pcie_map_bar_region(dev->parent, dev, BAR_2,
			shared_buffer_base->size,
			shared_buffer_base->jqs_paddr,
			&shared_buffer->mapping);
	if (ret < 0) {
		dev_err(dev, "bar mapping failed\n");
		return ret;
	}

	shared_buffer->mapped_to_bar = true;
	return 0;
}

static int ipu_adapter_ab_mfd_unmap_from_bar(struct device *dev,
		struct ipu_adapter_shared_buffer *shared_buffer)
{
	int ret;

	if (!shared_buffer->mapped_to_bar) {
		dev_warn(dev, "%s: shared buffer not mapped to bar\n",
				__func__);
		return -EINVAL;
	}

	ret = abc_pcie_unmap_bar_region(dev->parent, dev,
			&shared_buffer->mapping);
	if (ret < 0) {
		dev_err(dev, "bar unmapping failed\n");
		return ret;
	}

	shared_buffer->mapped_to_bar = false;
	memset(&shared_buffer->mapping, 0, sizeof(shared_buffer->mapping));
	return 0;
}

static struct ipu_shared_buffer *ipu_adapter_ab_mfd_alloc_shared_memory(
		struct device *dev, size_t size)
{
	struct ipu_adapter_ab_mfd_data *dev_data = dev_get_drvdata(dev);
	struct ipu_adapter_shared_buffer *sbuf;
	struct dma_buf *ab_dram_dmabuf;
	int ret;

	sbuf = kzalloc(sizeof(*sbuf), GFP_KERNEL);
	if (sbuf == NULL)
		return ERR_PTR(-ENOMEM);

	sbuf->base.host_vaddr = dma_alloc_coherent(dev_data->dma_dev, size,
			&sbuf->base.host_dma_addr, GFP_KERNEL);
	if (!sbuf->base.host_vaddr) {
		ret = -ENOMEM;
		goto err_exit_free_shared_buffer;
	}

	ab_dram_dmabuf = ab_dram_alloc_dma_buf_kernel(size);
	if (IS_ERR(ab_dram_dmabuf)) {
		ret =  PTR_ERR(ab_dram_dmabuf);
		goto err_exit_free_dma_coherent;
	}

	sbuf->ab_dram_dma_buf = ab_dram_dmabuf;
	sbuf->base.size = size;
	sbuf->base.jqs_paddr = ab_dram_get_dma_buf_paddr(ab_dram_dmabuf);

	ret = ipu_adapter_ab_mfd_map_to_bar(dev, sbuf);
	if (ret < 0)
		goto err_exit_free_dma_buffer;

	mutex_lock(&dev_data->sync_lock);
	list_add_tail(&sbuf->sbuf_entry, &dev_data->sbuf_list);
	mutex_unlock(&dev_data->sync_lock);

	return &sbuf->base;

err_exit_free_dma_buffer:
	ab_dram_free_dma_buf_kernel(ab_dram_dmabuf);
err_exit_free_dma_coherent:
	dma_free_coherent(dev_data->dma_dev, size, sbuf->base.host_vaddr,
				sbuf->base.host_dma_addr);
err_exit_free_shared_buffer:
	kfree(sbuf);

	return ERR_PTR(ret);
}

void ipu_adapter_ab_mfd_free_shared_memory(struct device *dev,
			struct ipu_shared_buffer *shared_buffer_base)
{
	struct ipu_adapter_ab_mfd_data *dev_data = dev_get_drvdata(dev);
	struct ipu_adapter_shared_buffer *sbuf;

	if (WARN_ON(!shared_buffer_base))
		return;

	sbuf = container_of(shared_buffer_base,
		struct ipu_adapter_shared_buffer, base);

	if (sbuf->mapped_to_bar)
		ipu_adapter_ab_mfd_unmap_from_bar(dev, sbuf);

	if (sbuf->ab_dram_dma_buf)
		ab_dram_free_dma_buf_kernel(sbuf->ab_dram_dma_buf);

	if (shared_buffer_base->host_vaddr)
		dma_free_coherent(dev_data->dma_dev, shared_buffer_base->size,
				shared_buffer_base->host_vaddr,
				shared_buffer_base->host_dma_addr);

	mutex_lock(&dev_data->sync_lock);
	list_del(&sbuf->sbuf_entry);
	mutex_unlock(&dev_data->sync_lock);
}

void ipu_adapter_ab_mfd_suspend_shared_memory(
		struct ipu_adapter_ab_mfd_data *dev_data)
{
	struct ipu_adapter_shared_buffer *sbuf, *sbuf_next;

	mutex_lock(&dev_data->sync_lock);

	list_for_each_entry_safe(sbuf, sbuf_next, &dev_data->sbuf_list,
			sbuf_entry) {
		if (sbuf->mapped_to_bar)
			ipu_adapter_ab_mfd_unmap_from_bar(dev_data->dev, sbuf);
	}

	mutex_unlock(&dev_data->sync_lock);
}

void ipu_adapter_ab_mfd_resume_shared_memory(
		struct ipu_adapter_ab_mfd_data *dev_data)
{
	struct ipu_adapter_shared_buffer *sbuf, *sbuf_next;

	mutex_lock(&dev_data->sync_lock);

	list_for_each_entry_safe(sbuf, sbuf_next, &dev_data->sbuf_list,
			sbuf_entry)
		ipu_adapter_ab_mfd_map_to_bar(dev_data->dev, sbuf);

	mutex_unlock(&dev_data->sync_lock);
}

void ipu_adapter_ab_mfd_free_all_shared_memory(
		struct ipu_adapter_ab_mfd_data *dev_data)
{
	struct ipu_adapter_shared_buffer *sbuf, *sbuf_next;

	list_for_each_entry_safe(sbuf, sbuf_next, &dev_data->sbuf_list,
			sbuf_entry)
		ipu_adapter_ab_mfd_free_shared_memory(dev_data->dev,
				&sbuf->base);
}

static void ipu_adapter_ab_mfd_sync_dma(struct device *dev,
		struct ipu_adapter_shared_buffer *shared_buffer,
		uint32_t offset, size_t size,
		enum dma_data_direction direction)
{
	int err = 0;
	struct abc_pcie_kernel_dma_desc desc;

	memset((void *)&desc, 0, sizeof(desc));
	desc.local_buf = (void *)shared_buffer->base.host_dma_addr + offset;
	desc.local_buf_kind = DMA_BUFFER_KIND_CMA;
	desc.remote_buf = (uint64_t)shared_buffer->base.jqs_paddr+offset;
	desc.remote_buf_kind = DMA_BUFFER_KIND_USER;
	desc.size = size;
	desc.dir = direction;
	err = abc_pcie_issue_sessionless_dma_xfer_sync(&desc);
	if (err)
		dev_warn(dev, "%s: DMA transfer failed\n",
			__func__);
}

static void ipu_adapter_ab_mfd_sync_pio(struct device *dev,
		struct ipu_adapter_shared_buffer *shared_buffer,
		uint32_t offset, size_t size,
		enum dma_data_direction direction)
{
	void __iomem *io_vaddr = shared_buffer->mapping.bar_vaddr + offset;
	void *buffer_vaddr = shared_buffer->base.host_vaddr + offset;

	if (direction == DMA_TO_DEVICE)
		memcpy_toio(io_vaddr, buffer_vaddr, size);
	else
		memcpy_fromio(buffer_vaddr, io_vaddr, size);
}

static ssize_t pio_threshold_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct ipu_adapter_ab_mfd_data *dev_data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", dev_data->pio_threshold);
}

static ssize_t pio_threshold_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int err;
	unsigned long size;
	struct ipu_adapter_ab_mfd_data *dev_data = dev_get_drvdata(dev);

	err = kstrtoul(buf, 0, &size);
	if (err)
		return err;

	if (size == 0)
		return -EINVAL;

	mutex_lock(&dev_data->sync_lock);
	dev_data->pio_threshold = size;
	mutex_unlock(&dev_data->sync_lock);

	return count;
}

static DEVICE_ATTR(pio_threshold, 0664, pio_threshold_show,
			pio_threshold_store);

void ipu_adapter_ab_mfd_sync_shared_memory(struct device *dev,
		struct ipu_shared_buffer *shared_buffer_base, uint32_t offset,
		size_t size, enum dma_data_direction direction)
{
	struct ipu_adapter_ab_mfd_data *dev_data;
	struct ipu_adapter_shared_buffer *shared_buffer;

	if (WARN_ON(!shared_buffer_base))
		return;

	dev_data = dev_get_drvdata(dev);
	shared_buffer = container_of(shared_buffer_base,
		struct ipu_adapter_shared_buffer, base);

	mutex_lock(&dev_data->sync_lock);

	if (!ipu_adapter_dram_is_ready(dev_data)) {
		dev_err(dev, "Sync failed because DRAM is not ready.\n");
		mutex_unlock(&dev_data->sync_lock);
		return;
	}


	if (size <= dev_data->pio_threshold &&
			shared_buffer->mapped_to_bar)
		ipu_adapter_ab_mfd_sync_pio(dev, shared_buffer, offset,
				size, direction);
	else
		ipu_adapter_ab_mfd_sync_dma(dev, shared_buffer, offset,
				size, direction);

	mutex_unlock(&dev_data->sync_lock);
}

struct ipu_jqs_buffer *ipu_adapter_ab_mfd_alloc_jqs_memory(struct device *dev,
			size_t size)
{
	struct ipu_adapter_jqs_buffer *jqs_buf;
	struct dma_buf *dma_buf;

	jqs_buf = kzalloc(sizeof(*jqs_buf), GFP_KERNEL);
	if (jqs_buf == NULL)
		return ERR_PTR(-ENOMEM);

	dma_buf = ab_dram_alloc_dma_buf_kernel(size);
	if (IS_ERR(dma_buf)) {
		kfree(jqs_buf);
		return (struct ipu_jqs_buffer *)dma_buf;
	}

	jqs_buf->base.jqs_paddr = ab_dram_get_dma_buf_paddr(dma_buf);
	jqs_buf->base.size = size;
	jqs_buf->ab_dram_dma_buf = dma_buf;

	return &jqs_buf->base;
}

void ipu_adapter_ab_mfd_free_jqs_memory(struct device *dev,
		struct ipu_jqs_buffer *buf)
{
	struct ipu_adapter_jqs_buffer *jqs_buf;

	if (!buf)
		return;

	jqs_buf = container_of(buf, struct ipu_adapter_jqs_buffer, base);

	ab_dram_free_dma_buf_kernel(jqs_buf->ab_dram_dma_buf);
	kfree(jqs_buf);
}

static struct device *ipu_adapter_ab_mfd_get_dma_device(struct device *dev)
{
	struct ipu_adapter_ab_mfd_data *dev_data = dev_get_drvdata(dev);

	return dev_data->dma_dev;
}

static irqreturn_t ipu_adapter_ab_mfd_interrupt_handler(int irq, void *arg)
{
	struct ipu_adapter_ab_mfd_data *dev_data =
		(struct ipu_adapter_ab_mfd_data *)arg;

	return ipu_core_jqs_msg_transport_interrupt_handler(dev_data->bus);
}

static irqreturn_t ipu_adapter_ab_mfd_interrupt_thread(int irq, void *arg)
{
	struct ipu_adapter_ab_mfd_data *dev_data =
		(struct ipu_adapter_ab_mfd_data *)arg;

	return ipu_core_jqs_msg_transport_interrupt_thread(dev_data->bus);
}

static int ipu_adapter_ab_mfd_low_priority_irq_notify(struct notifier_block *nb,
		unsigned long irq, void *data)
{
	struct ipu_adapter_ab_mfd_data *dev_data =
		container_of(nb, struct ipu_adapter_ab_mfd_data,
				low_priority_irq_nb);
	uint32_t intnc_val = (enum abc_intnc_ints)data;

	if (irq != ABC_MSI_AON_INTNC)
		return NOTIFY_DONE;

	if (intnc_val & 1 << INTNC_IPU_HPM_APBIF)
		dev_err(dev_data->dev,
				"%s: IPU Hardware Performance Monitor Interrupt Received",
				__func__);

	if (intnc_val & 1 << INTNC_IPU_ERR) {
		dev_err(dev_data->dev, "%s: JQS watchdog tripped", __func__);
		ipu_bus_notify_fatal_error(dev_data->bus);
	}

	if (intnc_val & 1 << INTNC_PPMU_IPU)
		dev_err(dev_data->dev,
				"%s: IPU Performance Monitor Interrupt Received",
				__func__);

	return NOTIFY_OK;
}

static int ipu_adapter_ab_mfd_atomic_sync32_shared_memory(struct device *dev,
			struct ipu_shared_buffer *shared_buffer_base,
			uint32_t offset, enum dma_data_direction direction)
{
	struct ipu_adapter_shared_buffer *sbuf;
	void __iomem *io_vaddr;
	uint32_t *buffer_vaddr;
	struct ipu_adapter_ab_mfd_data *dev_data;
	int wr_val;

	if (WARN_ON(!shared_buffer_base))
		return -EINVAL;

	sbuf = container_of(shared_buffer_base,
		struct ipu_adapter_shared_buffer, base);
	io_vaddr = sbuf->mapping.bar_vaddr + offset;
	buffer_vaddr = sbuf->base.host_vaddr + offset;
	dev_data = dev_get_drvdata(dev);

	mutex_lock(&dev_data->sync_lock);

	if (!ipu_adapter_dram_is_ready(dev_data)) {
		mutex_unlock(&dev_data->sync_lock);
		return -ENOLINK;
	}

	if ((unsigned long)buffer_vaddr % sizeof(uint32_t) != 0) {
		dev_err(dev, "%s: error: unaligned access\n", __func__);
		mutex_unlock(&dev_data->sync_lock);
		return -EINVAL;
	}


	if (sbuf->mapped_to_bar) {
		if (direction == DMA_TO_DEVICE) {
			wr_val = *buffer_vaddr;
			__iowmb();
			writel_relaxed(wr_val, io_vaddr);
		} else {
			*buffer_vaddr = readl(io_vaddr);
		}
	} else {
		ipu_adapter_ab_mfd_sync_dma(dev, sbuf, offset,
				sizeof(uint32_t), direction);
	}

	mutex_unlock(&dev_data->sync_lock);
	return 0;
}

bool ipu_adapter_is_ready(struct ipu_adapter_ab_mfd_data *dev_data)
{
	return (ipu_adapter_link_is_ready(dev_data) &&
			ipu_adapter_dram_is_ready(dev_data));
}

bool ipu_adapter_ab_mfd_is_ready(struct device *dev)
{
	struct ipu_adapter_ab_mfd_data *dev_data = dev_get_drvdata(dev);

	return ipu_adapter_is_ready(dev_data);
}

void ipu_adapter_ipu_pre_rate_change(
		struct ipu_adapter_ab_mfd_data *dev_data,
		struct ab_clk_notifier_data *clk_data)
{
	struct paintbox_bus *bus = dev_data->bus;

	dev_dbg(dev_data->dev,
			"%s: IPU rate will change from %lu Hz to %lu Hz",
			__func__, clk_data->old_rate, clk_data->new_rate);

	/* If the new rate is active the post rate change will handle
	 * any adapter changes or bus notifications
	 */
	if (ipu_clock_rate_is_active(clk_data->new_rate))
		return;

	ipu_bus_notify_suspend_jqs(bus);
	dev_data->ipu_clock_rate_hz = clk_data->new_rate;
}

void ipu_adapter_ipu_post_rate_change(
		struct ipu_adapter_ab_mfd_data *dev_data,
		struct ab_clk_notifier_data *clk_data)
{
	struct paintbox_bus *bus = dev_data->bus;

	dev_dbg(dev_data->dev,
			"%s: IPU rate has changed from %lu Hz to %lu Hz",
			__func__, clk_data->old_rate, clk_data->new_rate);

	/* If the new rate is inactive the pre rate change already
	 * handled any adapter changes or bus notifications
	 */
	if (!ipu_clock_rate_is_active(clk_data->new_rate))
		return;

	dev_data->ipu_clock_rate_hz = clk_data->new_rate;
	ipu_bus_notify_ready(bus, dev_data->ipu_clock_rate_hz);
}

void ipu_adapter_ipu_abort_rate_change(
		struct ipu_adapter_ab_mfd_data *dev_data,
		struct ab_clk_notifier_data *clk_data)
{
	struct paintbox_bus *bus = dev_data->bus;

	dev_err(dev_data->dev,
			"%s: IPU rate has aborted change %lu Hz to %lu Hz",
			__func__, clk_data->old_rate, clk_data->new_rate);

	/* Treat this as a clock going down
	 */
	dev_data->ipu_clock_rate_hz = clk_data->old_rate;
	ipu_bus_notify_suspend_jqs(bus);
}

void ipu_adapter_dram_pre_rate_change(
		struct ipu_adapter_ab_mfd_data *dev_data,
		struct ab_clk_notifier_data *clk_data,
		bool pre_data_loss)
{
	struct paintbox_bus *bus = dev_data->bus;

	dev_dbg(dev_data->dev,
			"%s: DRAM rate will change from %lu Hz to %lu Hz",
			__func__, clk_data->old_rate, clk_data->new_rate);

	if (pre_data_loss) {
		dev_dbg(dev_data->dev, "%s: DRAM down, no retention", __func__);
		ipu_bus_notify_shutdown(bus);
	} else if (clk_data->new_rate != clk_data->old_rate) {
		if (!clk_data->new_rate) {
			dev_dbg(dev_data->dev, "%s: DRAM down, self refresh",
					__func__);
			ipu_bus_notify_suspend_jqs(bus);
		}

		ipu_bus_notify_suspend_dram(bus);
	}

	mutex_lock(&dev_data->sync_lock);
	atomic_andnot(IPU_ADAPTER_STATE_DRAM_READY, &dev_data->state);
	mutex_unlock(&dev_data->sync_lock);
}

void ipu_adapter_dram_post_rate_change(
		struct ipu_adapter_ab_mfd_data *dev_data,
		struct ab_clk_notifier_data *clk_data)
{
	struct paintbox_bus *bus = dev_data->bus;

	dev_dbg(dev_data->dev,
			"%s: DRAM rate has changed from %lu Hz to %lu Hz",
			__func__, clk_data->old_rate, clk_data->new_rate);

	/* If the new rate is inactive the pre rate change already
	 * handled any adapter changes or bus notifications
	 */
	if (!clk_data->new_rate)
		return;

	atomic_or(IPU_ADAPTER_STATE_DRAM_READY, &dev_data->state);
	ipu_bus_notify_ready(bus, dev_data->ipu_clock_rate_hz);
}

static int ipu_adapter_ab_sm_clk_listener(struct notifier_block *nb,
					  unsigned long action,
					  void *data)
{
	/* TODO(b/120037131): implement listener as needed */
	struct ab_clk_notifier_data *clk_data =
			(struct ab_clk_notifier_data *)data;
	struct ipu_adapter_ab_mfd_data *dev_data =
			container_of(nb,
				     struct ipu_adapter_ab_mfd_data,
				     clk_change_nb);

	if (action & AB_DRAM_PRE_RATE_CHANGE) {

		if (action & AB_DRAM_DATA_PRE_OFF) {
			dev_warn(dev_data->dev,
				 "DRAM data will be lost; please free ringbuffer\n");
			/* TODO(b/128524484) additional work on client side */
		}
		ipu_adapter_dram_pre_rate_change(dev_data, clk_data,
				!!(action & AB_DRAM_DATA_PRE_OFF));
		return NOTIFY_OK;
	}

	if (action & AB_DRAM_POST_RATE_CHANGE) {

		if (action & AB_DRAM_DATA_POST_OFF) {
			dev_warn(dev_data->dev,
				 "DRAM data is lost; suggest double check whether ringbuffer has been freed\n");
			/* TODO(b/128524484) only if necessary */
		}

		ipu_adapter_dram_post_rate_change(dev_data, clk_data);
		return NOTIFY_OK;
	}

	switch (action) {
	case AB_IPU_PRE_RATE_CHANGE:
		ipu_adapter_ipu_pre_rate_change(dev_data, clk_data);
		break;
	case AB_IPU_POST_RATE_CHANGE:
		ipu_adapter_ipu_post_rate_change(dev_data, clk_data);
		break;
	case AB_IPU_ABORT_RATE_CHANGE:
		ipu_adapter_ipu_abort_rate_change(dev_data, clk_data);
		break;
	default:
		return NOTIFY_DONE;  /* Don't care */
	}

	return NOTIFY_OK;
}

static void ipu_adapter_ab_mfd_enable_interrupts(
		struct ipu_adapter_ab_mfd_data *dev_data)
{
	struct platform_device *pdev = dev_data->pdev;
	int irq_index, ret;

	if (dev_data->interrupts_enabled)
		return;

	for (irq_index = 0; irq_index < platform_irq_count(pdev);
			irq_index++)
		enable_irq(dev_data->irqs[irq_index]);

	ret = atomic_notifier_chain_register(dev_data->low_priority_irq_nh,
			&dev_data->low_priority_irq_nb);
	if (ret < 0)
		dev_err(dev_data->dev,
				"Cannot register notifier for low priority irq, ret=%d\n",
				ret);

	dev_data->interrupts_enabled = true;
}

static void ipu_adapter_ab_mfd_disable_interrupts(
		struct ipu_adapter_ab_mfd_data *dev_data)
{
	struct platform_device *pdev = dev_data->pdev;
	int irq_index, ret;

	if (!dev_data->interrupts_enabled)
		return;

	for (irq_index = 0; irq_index < platform_irq_count(pdev);
			irq_index++)
		disable_irq(dev_data->irqs[irq_index]);

	ret = atomic_notifier_chain_unregister(dev_data->low_priority_irq_nh,
			&dev_data->low_priority_irq_nb);
	if (ret < 0)
		dev_err(dev_data->dev,
				"Cannot unregister notifier for low priority irq, ret=%d\n",
				ret);

	dev_data->interrupts_enabled = false;
}

static int ipu_adapter_pcie_blocking_listener(struct notifier_block *nb,
					      unsigned long action,
					      void *data)
{
	struct ipu_adapter_ab_mfd_data *dev_data =
			container_of(nb,
				     struct ipu_adapter_ab_mfd_data,
				     pcie_link_blocking_nb);
	struct paintbox_bus *bus = dev_data->bus;
	uint32_t val;

	if ((action & ABC_PCIE_LINK_ERROR) &&
			ipu_adapter_link_is_ready(dev_data)) {
		dev_err(dev_data->dev, "%s: PCIe link error\n", __func__);
		atomic_andnot(IPU_ADAPTER_STATE_PCIE_READY, &dev_data->state);
		ipu_adapter_ab_mfd_disable_interrupts(dev_data);
		ipu_adapter_ab_mfd_suspend_shared_memory(dev_data);
		ipu_bus_notify_shutdown(bus);
		return NOTIFY_OK;
	}

	if ((action & ABC_PCIE_LINK_POST_ENABLE) &&
			!ipu_adapter_link_is_ready(dev_data)) {
		dev_dbg(dev_data->dev, "%s: PCIe link available\n", __func__);
		/* make sure that there are no hanging interrupts */
		ipu_config_read(IPU_CSR_JQS_OFFSET + JQS_SYS_DBL,
			sizeof(uint32_t), &val);
		ipu_adapter_ab_mfd_enable_interrupts(dev_data);
		ipu_adapter_ab_mfd_resume_shared_memory(dev_data);
		atomic_or(IPU_ADAPTER_STATE_PCIE_READY, &dev_data->state);
		ipu_bus_notify_ready(bus, dev_data->ipu_clock_rate_hz);
		return NOTIFY_OK;
	}

	if ((action & ABC_PCIE_LINK_PRE_DISABLE) &&
			ipu_adapter_link_is_ready(dev_data)) {
		dev_dbg(dev_data->dev, "%s: PCIe link going down\n", __func__);
		ipu_bus_notify_suspend_jqs(bus);
		ipu_bus_notify_suspend_dram(bus);
		atomic_andnot(IPU_ADAPTER_STATE_PCIE_READY, &dev_data->state);
		ipu_adapter_ab_mfd_disable_interrupts(dev_data);
		/* flush any active recovery work */
		if (test_bit(IPU_RECOVERY_BIT, &bus->recovery_active))
			flush_work(&bus->recovery_work);
		ipu_adapter_ab_mfd_suspend_shared_memory(dev_data);
		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}

static void ipu_adapter_ab_mfd_set_platform_data(struct platform_device *pdev,
		struct paintbox_pdata *pdata)
{
	pdata->page_size_bitmap = IPU_PAGE_SIZE_BITMAP;
	pdata->input_address_size = IPU_INPUT_ADDR_SIZE;
	pdata->output_address_size = IPU_OUTPUT_ADDR_SIZE;
	pdata->dma_base = PAINTBOX_IOVA_START;
	pdata->dma_size = PAINTBOX_IOVA_SIZE;

	pdata->capabilities.version_major =
			(IPU_VERSION_DEF & IPU_VERSION_MAJOR_MASK) >>
					IPU_VERSION_MAJOR_SHIFT;
	pdata->capabilities.version_minor =
			(IPU_VERSION_DEF & IPU_VERSION_MINOR_MASK) >>
					IPU_VERSION_MAJOR_SHIFT;
	pdata->capabilities.version_build =
			IPU_VERSION_DEF & IPU_VERSION_INCR_MASK;
	pdata->capabilities.is_fpga =
			!!(IPU_VERSION_DEF & IPU_VERSION_FPGA_BUILD_MASK);

	/* TODO(b/115434021):  Figure out how to get the hardware ID from the
	 * MFD if it is possible.  Otherwise we will need a lookup table and
	 * use the the IPU_VERSION register to reconcile it.
	 */
	pdata->capabilities.hardware_id = 11;

	pdata->capabilities.is_simulator = false;
	dev_dbg(&pdev->dev, "Paintbox IPU Version %u.%u.%u %s Hardware ID %u\n",
			pdata->capabilities.version_major,
			pdata->capabilities.version_minor,
			pdata->capabilities.version_build,
			pdata->capabilities.is_fpga ? "FPGA" : "",
			pdata->capabilities.hardware_id);

	pdata->capabilities.iommu_enabled = iommu_present(&ipu_bus_type);

	pdata->capabilities.num_stps = IPU_CAP_DEF & IPU_CAP_NUM_STP_MASK;
	pdata->capabilities.num_lbps = (IPU_CAP_DEF & IPU_CAP_NUM_LBP_MASK) >>
			IPU_CAP_NUM_LBP_SHIFT;
	pdata->capabilities.num_dma_channels =
			(IPU_CAP_DEF & IPU_CAP_NUM_DMA_CHAN_MASK) >>
					IPU_CAP_NUM_DMA_CHAN_SHIFT;
	pdata->capabilities.num_interrupts =
			pdata->capabilities.num_dma_channels +
			pdata->capabilities.num_stps + NUM_BIF_INTERRUPTS +
			NUM_MMU_INTERRUPTS;

	dev_dbg(&pdev->dev,
			"STPs %u LBPs %u DMA Channels %u\n",
			pdata->capabilities.num_stps,
			pdata->capabilities.num_lbps,
			pdata->capabilities.num_dma_channels);
}

static void ipu_adapter_ab_mfd_set_bus_ops(struct paintbox_bus_ops *ops)
{
	ops->write32 = &ipu_adapter_ab_mfd_writel;
	ops->write64 = &ipu_adapter_ab_mfd_writeq;
	ops->read32 = &ipu_adapter_ab_mfd_readl;
	ops->read64 = &ipu_adapter_ab_mfd_readq;
	ops->alloc_shared_memory = &ipu_adapter_ab_mfd_alloc_shared_memory;
	ops->free_shared_memory = &ipu_adapter_ab_mfd_free_shared_memory;
	ops->sync_shared_memory = &ipu_adapter_ab_mfd_sync_shared_memory;
	ops->atomic_sync32_shared_memory =
			&ipu_adapter_ab_mfd_atomic_sync32_shared_memory;
	ops->get_dma_device = &ipu_adapter_ab_mfd_get_dma_device;
	ops->alloc_jqs_memory = &ipu_adapter_ab_mfd_alloc_jqs_memory;
	ops->free_jqs_memory = &ipu_adapter_ab_mfd_free_jqs_memory;
	ops->is_ready = &ipu_adapter_ab_mfd_is_ready;
	ops->frc_clock_ungate = &ipu_adapter_frc_clock_ungate;
}

struct paintbox_bus *ipu_bus_from_device(struct device *parent_dev)
{
	struct ipu_adapter_ab_mfd_data *dev_data = dev_get_drvdata(parent_dev);

	return dev_data->bus;
}

static int ipu_adapter_ab_mfd_register_low_priority_irq(
		struct ipu_adapter_ab_mfd_data *dev_data)
{
	int ret;
	uint64_t prop;

	ret = device_property_read_u64(dev_data->dev, "intnc-notifier-chain",
			&prop);
	if (ret < 0) {
		dev_err(dev_data->dev,
				"intnc-notifier-chain property not supplied, ret=%d\n",
				ret);
		return ret;
	}

	dev_data->low_priority_irq_nh =
		(struct atomic_notifier_head *)prop;

	if (!dev_data->low_priority_irq_nh) {
		dev_err(dev_data->dev,
				"no intnc non-critical irq notifier supplied\n");
		return -ENOENT;
	}

	dev_data->low_priority_irq_nb.notifier_call =
			ipu_adapter_ab_mfd_low_priority_irq_notify;
	ret = atomic_notifier_chain_register(dev_data->low_priority_irq_nh,
			&dev_data->low_priority_irq_nb);
	if (ret) {
		dev_err(dev_data->dev,
				"Cannot register notifier for low priority irq, ret=%d\n",
				ret);
		return ret;
	}

	return 0;
}

const struct attribute *ipu_adapter_ab_mfd_attrs[] = {
	&dev_attr_pio_threshold.attr,
	NULL,
};

static int ipu_adapter_ab_mfd_probe(struct platform_device *pdev)
{
	struct ipu_adapter_ab_mfd_data *dev_data;
	int ret, irq_index;

	dev_data = devm_kzalloc(&pdev->dev, sizeof(*dev_data), GFP_KERNEL);
	if (dev_data == NULL)
		return -ENOMEM;

	platform_set_drvdata(pdev, dev_data);

	dev_data->dev = &pdev->dev;
	dev_data->pdev = pdev;
	dev_data->pio_threshold = SZ_32K;

	ipu_adapter_ab_mfd_set_platform_data(pdev, &dev_data->pdata);
	ipu_adapter_ab_mfd_set_bus_ops(&dev_data->ops);

	mutex_init(&dev_data->sync_lock);

	INIT_LIST_HEAD(&dev_data->sbuf_list);

	init_completion(&dev_data->dma_completion);

	if (WARN_ON(platform_irq_count(pdev) >
			IPU_ADAPTER_AB_MFD_MAX_INTERRUPTS))
		return -EINVAL;

	for (irq_index = 0; irq_index < platform_irq_count(pdev); irq_index++) {
		int irq = platform_get_irq(pdev, irq_index);

		if (irq < 0) {
			dev_err(&pdev->dev,
					"%s: platform_get_irq failed, err %d\n",
					__func__, irq);
			return -ENODEV;
		}

		ret = devm_request_threaded_irq(&pdev->dev, irq,
				ipu_adapter_ab_mfd_interrupt_handler,
				ipu_adapter_ab_mfd_interrupt_thread,
				IRQF_ONESHOT, dev_name(&pdev->dev), dev_data);
		if (ret < 0) {
			dev_err(&pdev->dev,
					"%s: failed to request irq, err %d\n",
					__func__, ret);
			return ret;
		}

		dev_data->irqs[irq_index] = irq;
	}

	ret = ipu_adapter_ab_mfd_register_low_priority_irq(dev_data);
	if (ret < 0) {
		dev_err(&pdev->dev,
				"%s: failed to register low priority irq, err %d\n",
				__func__, ret);
		return ret;
	}

	dev_data->interrupts_enabled = true;

	if (iommu_present(pdev->dev.parent->bus)) {
		dev_data->dma_dev = pdev->dev.parent;
	} else {
		arch_setup_dma_ops(&pdev->dev, 0, 0, NULL,
				false /* coherent */);
		dev_data->dma_dev = &pdev->dev;
	}

	atomic_or(IPU_ADAPTER_STATE_PCIE_READY, &dev_data->state);

	ret = ipu_bus_initialize(&pdev->dev, &dev_data->ops,
			&dev_data->pdata, &dev_data->bus);
	if (ret < 0)
		return ret;

#if IS_ENABLED(CONFIG_IPU_IOMMU)
	ret = ipu_bus_device_register(dev_data->bus, "ipu-iommu",
			PAINTBOX_DEVICE_TYPE_IOMMU);
	if (ret < 0) {
		dev_err(&pdev->dev, "IOMMU device register failed, ret %d\n",
				ret);
		goto err_deinitialize_bus;
	}
#endif

	ret = ipu_bus_device_register(dev_data->bus, "paintbox-ipu",
			PAINTBOX_DEVICE_TYPE_IPU);
	if (ret < 0) {
		dev_err(&pdev->dev,
				"IPU device register failed, ret %d\n", ret);
		goto err_deinitialize_bus;
	}

	dev_data->clk_change_nb.notifier_call = ipu_adapter_ab_sm_clk_listener;
	ret = ab_sm_register_clk_event(&dev_data->clk_change_nb);
	if (ret) {
		dev_err(&pdev->dev,
			"failed to subscribe to clk event, ret %d\n", ret);
		goto err_deinitialize_bus;
	}

	dev_data->pcie_link_blocking_nb.notifier_call =
				ipu_adapter_pcie_blocking_listener;
	ret = abc_register_pcie_link_blocking_event(
				&dev_data->pcie_link_blocking_nb);
	if (ret) {
		dev_err(&pdev->dev,
			"failed to subscribe to PCIe blocking link event, ret %d\n",
			ret);
		goto err_unregister_sm_notifer;
	}

	ret = sysfs_create_files(&pdev->dev.kobj, ipu_adapter_ab_mfd_attrs);
	if (ret) {
		dev_err(&pdev->dev, "failed to create sysfs entries, ret %d\n",
			ret);
		goto err_unregister_pcie_notifer;
	}

	return 0;

err_unregister_pcie_notifer:
	abc_unregister_pcie_link_blocking_event(
			&dev_data->pcie_link_blocking_nb);
err_unregister_sm_notifer:
	ab_sm_unregister_clk_event(&dev_data->clk_change_nb);
err_deinitialize_bus:
	ipu_bus_deinitialize(dev_data->bus);
	atomic_notifier_chain_unregister(dev_data->low_priority_irq_nh,
				&dev_data->low_priority_irq_nb);

	return ret;
}

static int ipu_adapter_ab_mfd_remove(struct platform_device *pdev)
{
	struct ipu_adapter_ab_mfd_data *dev_data =
			platform_get_drvdata(pdev);

	sysfs_remove_files(&pdev->dev.kobj, ipu_adapter_ab_mfd_attrs);
	abc_unregister_pcie_link_blocking_event(
				&dev_data->pcie_link_blocking_nb);
	ab_sm_unregister_clk_event(&dev_data->clk_change_nb);
	ipu_bus_deinitialize(dev_data->bus);
	ipu_adapter_ab_mfd_free_all_shared_memory(dev_data);
	dev_data->bus = NULL;
	atomic_notifier_chain_unregister(dev_data->low_priority_irq_nh,
			&dev_data->low_priority_irq_nb);

	return 0;
}

static struct platform_driver ipu_adapter_ab_mfd_driver = {
	.driver = {
		.name	= DRV_NAME_ABC_PCIE_IPU,
		.probe_type = PROBE_FORCE_SYNCHRONOUS,
	},
	.probe		= ipu_adapter_ab_mfd_probe,
	.remove		= ipu_adapter_ab_mfd_remove,
};

module_platform_driver(ipu_adapter_ab_mfd_driver);

MODULE_AUTHOR("Google, Inc.");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("IPU Airbrush MFD Adapter Driver");
