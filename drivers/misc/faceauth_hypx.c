/*
 * Google FaceAuth driver interface to hypx
 *
 * Copyright (C) 2018-2019 Google, Inc.
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

#include <asm/cacheflush.h>
#include <asm/compiler.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <misc/faceauth_hypx.h>
#include <soc/qcom/scm.h>
#include <soc/qcom/secure_buffer.h>

#define CREATE_TRACE_POINTS
#include <trace/events/faceauth.h>

#define HYPX_SMC_ID(func) (0x43DEAD00 | func)
#define HYPX_SMC_FUNC_CHECK_PIL_COMPLETION HYPX_SMC_ID(0x1)
#define HYPX_SMC_FUNC_INIT HYPX_SMC_ID(0x2)
#define HYPX_SMC_FUNC_PROCESS HYPX_SMC_ID(0x3)
#define HYPX_SMC_FUNC_CHECK_PROCESS_RESULT HYPX_SMC_ID(0x4)
#define HYPX_SMC_FUNC_CLEANUP HYPX_SMC_ID(0x5)
#define HYPX_SMC_FUNC_GET_DEBUG_RESULT HYPX_SMC_ID(0x6)
#define HYPX_SMC_FUNC_GET_DEBUG_DATA HYPX_SMC_ID(0x7)
#define HYPX_SMC_FUNC_GET_DEBUG_BUFFER HYPX_SMC_ID(0x8)
#define HYPX_SMC_FUNC_DRIVER_PROBE HYPX_SMC_ID(0x9)
#define HYPX_SMC_FUNC_DRIVER_REMOVE HYPX_SMC_ID(0xa)
#define HYPX_SMC_FUNC_CHECK_FW_STATUS HYPX_SMC_ID(0xb)

#define PIL_DMA_TIMEOUT 3000
#define INPUT_IMAGE_WIDTH 480
#define INPUT_IMAGE_HEIGHT 640
#define CALIBRATION_SIZE 1024
#define DEBUG_DATA_BIN_SIZE (2 * 1024 * 1024)
#define CONTEXT_SWITCH_TIMEOUT_MS 40
/* different from EL1 since it'll take longer in each assess */
#define CONTEXT_SWITCH_TO_FACEAUTH_US 600

#define ERR_FW_READY 3

/* Error code for EL2 error */
#define ERR_SUCCESS 0
#define ERR_PIL_COMPLETE 1
#define ERR_PIL_INCOMPLETE 2
#define ERR_FW_READY 3
#define ERR_SECURE_CAM 5
#define ERR_DMA 6
#define ERR_LOCK 7
#define ERR_NOBUFFER 8
#define ERR_NON_SECURE_MODE 9
#define ERR_M0_BOOT 10
#define ERR_BUSY_CLEANUP 11
#define ERR_SIZE_MISMATCH 12
#define ERR_BLOCKED_REQ 13
#define ERR_UNSUPPORTED_REQ 14
#define ERR_LINK_UNSTABLE 15
#define ERR_OUT_OF_BOUND 16

struct hypx_mem_segment {
	/* address of the segment begin */
	uint32_t addr;
	/* number of pages in the segment */
	uint32_t pages;
} __packed;

#define HYPX_MEMSEGS_NUM (PAGE_SIZE / sizeof(struct hypx_mem_segment))

/* Note that this struct is passed to EL2, thus it needs to be
 * allocated from physically contiguous memory region accessible
 * by EL2.
 */
struct hypx_blob {
	struct hypx_mem_segment segments[HYPX_MEMSEGS_NUM];
} __packed;

struct faceauth_data {
	struct hypx_blob *hypx_blob;

	enum dma_data_direction direction;

	__user void *buffer;

	struct dma_buf *dma_buf;
	struct dma_buf_attachment *attach;
	struct sg_table *sg_table;

	/* This flag specifies if blob been originally allocated by HLOS
	 * and assigned to EXT_DSP+SECCAM.
	 * If the flag is true it means the blob need to be reassigned
	 * back to HLOS at the end of faceauth transaction.
	 */
	bool need_reassign_to_hlos;
};

/* Keep this struct in sync with HypX firmware code at
 * https://team.git.corp.google.com/soc-gerrit-admin/faceauth-hypx/
 * Put fields in order to avoid unaligned access that EL2 does not like
 */
struct hypx_fa_init {
	uint64_t verbosity_level;
	/* phy address for 4KiB temporary buffer used by EL2 DMA engine */
	uint64_t bounce_buff;
	uint64_t features;
} __packed;

struct hypx_fa_process {
	uint64_t image_dot_left; /* PHY addr */
	uint64_t image_dot_right; /* PHY addr */
	uint64_t image_flood_left; /* PHY addr */
	uint64_t image_flood_right; /* PHY addr */
	uint64_t calibration; /* PHY addr */
	uint64_t citadel_token; /* PHY addr */
	int16_t cache_flush_indexes[FACEAUTH_MAX_CACHE_FLUSH_SIZE];

	uint32_t operation;
	uint32_t profile_id;
	uint32_t input_time_ms;

	uint32_t image_dot_left_size;
	uint32_t image_dot_right_size;
	uint32_t image_flood_left_size;
	uint32_t image_flood_right_size;
	uint32_t calibration_size;

	uint32_t is_secure_camera;
	uint32_t citadel_input;
	uint32_t citadel_token_size;
	uint32_t citadel_input2;
} __packed;

struct hypx_fa_process_results {
	uint32_t result;
	uint32_t angles;
	uint32_t fw_version;
	int32_t error_code;
	uint64_t debug_buffer; /* PHY addr*/
	uint32_t debug_buffer_size;
	uint32_t citadel_lockout_event;
	uint32_t citadel_output1;
	uint32_t citadel_output2;
	uint32_t exception_number;
	uint32_t citadel_token_size;
	uint64_t citadel_token; /* PHY addr */
	uint64_t deferred_autocal; /* PHY addr */
	uint32_t deferred_autocal_size;
	uint32_t aux_data[FACEAUTH_AUX_DATA_SIZE];
} __packed;

struct hypx_fa_debug_data {
	uint64_t image_left;
	uint64_t image_right;
	uint64_t image_flood_left;
	uint64_t image_flood_right;
	uint64_t ab_state; /* PHY addr */
	uint64_t output_buffers;
	uint32_t offset_int_state;
	uint32_t offset_ab_state;
	uint32_t image_left_size;
	uint32_t image_right_size;
	uint32_t image_flood_left_size;
	uint32_t image_flood_right_size;
	uint32_t internal_state_struct_size;
	uint32_t buffer_list_size;
	uint32_t buffer_base;
	uint32_t exception_number;
	uint32_t fault_address;
	uint32_t ab_link_reg;
	uint64_t calibration_buffer;
	uint32_t calibration_size;
} __packed;

static int parse_el2_return(int code)
{
	if (code == ERR_SECURE_CAM)
		pr_err("faceauth: EL2: Insecure path detected");
	else if (code == ERR_DMA)
		pr_err("faceauth: EL2: DMA transfter failed");
	else if (code == ERR_LOCK)
		pr_err("faceauth: EL2: Region lock failed");
	else if (code == ERR_NOBUFFER)
		pr_err("faceauth: EL2: No buffer for alloc/dealloc");
	else if (code == ERR_NON_SECURE_MODE)
		pr_err("faceauth: EL2: Not in secure mode");
	else if (code == ERR_M0_BOOT)
		pr_err("faceauth: EL2: Failed to boot M0");
	else if (code == ERR_BUSY_CLEANUP)
		pr_err("faceauth: EL2: M0 cleanup in progress");
	else if (code == ERR_SIZE_MISMATCH)
		pr_err("faceauth: EL2: Input data size mismatch");
	else if (code == ERR_BLOCKED_REQ)
		pr_err("faceauth: EL2: Blocked EL2 request");
	else if (code == ERR_UNSUPPORTED_REQ)
		pr_err("faceauth: EL2: Unsupported EL2 request");
	else if (code == ERR_LINK_UNSTABLE)
		pr_err("faceauth: EL2: PCIe link down");
	else if (code == ERR_OUT_OF_BOUND)
		pr_err("faceauth: EL2: DMA buffer out of boundary");
	else
		pr_err("faceauth: EL2: Undefined return code: %d", code);
	return -EINVAL;
}

static void hypx_free_blob_userbuf(struct faceauth_data *data)
{
	int source_vm[] = { VMID_CP_DSP_EXT, VMID_HLOS_FREE };
	int dest_vm[] = { VMID_HLOS };
	int dest_perm[] = { PERM_READ | PERM_WRITE | PERM_EXEC };
	struct hypx_blob *blob = data->hypx_blob;
	int i;

	WARN_ON(data->dma_buf);

	for (i = 0; i < HYPX_MEMSEGS_NUM; i++) {
		uint64_t phy_addr;
		void *virt_addr;
		int ret = 0;
		int order = 0;

		if (!blob->segments[i].addr)
			break;
		phy_addr = (uint64_t)blob->segments[i].addr * PAGE_SIZE;
		virt_addr = phys_to_virt(phy_addr);
		order = order_base_2(blob->segments[i].pages);

		if (data->need_reassign_to_hlos) {
			ret = hyp_assign_phys(
				phy_addr, blob->segments[i].pages * PAGE_SIZE,
				source_vm, ARRAY_SIZE(source_vm), dest_vm,
				dest_perm, ARRAY_SIZE(dest_vm));
			if (ret)
				pr_err("hyp_assign_phys returned an error %d\n",
				       ret);
		}

		free_pages((unsigned long)virt_addr, order);
	}

	free_page((unsigned long)blob);
	data->hypx_blob = NULL;
}

/* Reassign segments from EXT_DSP to HLOS and copy its data to
 * buffer if buffer is not NULL
 */
static int hypx_copy_from_blob_userbuf(struct device *dev,
				       struct faceauth_data *data,
				       void __user *buffer, size_t size,
				       bool copy_user)
{
	int source_vm[] = { VMID_CP_DSP_EXT, VMID_HLOS_FREE };
	int dest_vm[] = { VMID_HLOS };
	int dest_perm[] = { PERM_READ | PERM_WRITE | PERM_EXEC };
	struct hypx_blob *blob = data->hypx_blob;
	void __user *buffer_iter = buffer;
	uint64_t buffer_iter_remaining = size;
	uint64_t tocopy;
	int i;
	int lret, ret = 0;

	/* We expect that the buffer data comes from Airbrush and thus it
	 * assigned to EXT_DSP
	 */

	WARN_ON(!data->need_reassign_to_hlos);

	for (i = 0; i < HYPX_MEMSEGS_NUM; i++) {
		uint64_t phy_addr;
		void *virt_addr;

		if (!blob->segments[i].addr)
			break;

		tocopy = min(buffer_iter_remaining,
			     (uint64_t)blob->segments[i].pages * PAGE_SIZE);
		phy_addr = (uint64_t)blob->segments[i].addr * PAGE_SIZE;
		virt_addr = phys_to_virt(phy_addr);

		lret = hyp_assign_phys(phy_addr,
				       blob->segments[i].pages * PAGE_SIZE,
				       source_vm, ARRAY_SIZE(source_vm),
				       dest_vm, dest_perm, ARRAY_SIZE(dest_vm));

		dma_sync_single_for_cpu(dev, phy_addr,
					blob->segments[i].pages * PAGE_SIZE,
					DMA_FROM_DEVICE);

		if (lret) {
			ret = lret;
			pr_err("hyp_assign_phys returned an error %d\n", ret);
		}
		if (buffer) {
			if (copy_user) {
				ret = copy_to_user(buffer_iter, virt_addr,
						   tocopy);
				if (ret) {
					pr_err("copy from blob cp failed: %d",
					       ret);
					return -EFAULT;
				}
			} else {
				memcpy(buffer_iter, virt_addr, tocopy);
			}
			buffer_iter += tocopy;
		}

		buffer_iter_remaining -= tocopy;
	}

	/* We assigned the blob to HLOS already, no need for additional
	 * reassignment
	 */
	data->need_reassign_to_hlos = false;

	return ret;
}

/* Create a faceauth blob of specified size.
 * If buffer is not NULL then copy the data to the created blob.
 */
static void hypx_create_blob_userbuf(struct device *dev,
				     struct faceauth_data *data,
				     void __user *buffer, size_t size)
{
	uint64_t buffer_iter_remaining;
	const void __user *buffer_iter;
	struct hypx_blob *blob;
	struct hypx_mem_segment *segments_iter;
	int i;

	data->buffer = buffer;

	/* note that allocated page is not reclaimable */
	blob = (void *)get_zeroed_page(GFP_KERNEL);
	if (!blob) {
		pr_err("Cannot allocate memory for hypx segments blob\n");
		goto exit1;
	}
	WARN_ON(virt_to_phys(blob) % PAGE_SIZE);
	data->hypx_blob = blob;

	buffer_iter = buffer;
	buffer_iter_remaining = size;
	segments_iter = blob->segments;

	data->need_reassign_to_hlos = true;

	for (i = 0; i < HYPX_MEMSEGS_NUM; i++) {
		int page_order =
			min(MAX_ORDER - 1,
			    order_base_2(buffer_iter_remaining / PAGE_SIZE));
		uint64_t size = (1ULL << page_order) * PAGE_SIZE;
		uint64_t tocopy = min(buffer_iter_remaining, size);

		int ret = 0;
		void *out_buffer = NULL;
		int source_vm[] = { VMID_HLOS };
		int dest_vm[] = { VMID_CP_DSP_EXT, VMID_HLOS_FREE };
		int dest_perm[] = { PERM_READ | PERM_WRITE,
				    PERM_READ | PERM_WRITE };

		if (!tocopy)
			break;

		while (!out_buffer) {
			if (page_order < 0) {
				pr_err("Cannot allocate memory for copying data for hypx");
				goto exit2;
			}

			out_buffer = (void *)__get_free_pages(
				GFP_KERNEL | __GFP_NOWARN | __GFP_COMP,
				page_order);
			size = (1ULL << page_order) * PAGE_SIZE;
			tocopy = min(buffer_iter_remaining, size);
			if (!out_buffer)
				page_order--;
		}

		if (buffer)
			copy_from_user(out_buffer, buffer_iter, tocopy);

		dma_sync_single_for_device(dev, virt_to_phys(out_buffer), size,
					   DMA_TO_DEVICE);

		/*
		 * In the future this hyp_assign call will be invoked for
		 * Camera buffers by camera stack itself.
		 * We will need to assign only calibration/debug buffers.
		 */
		ret = hyp_assign_phys(virt_to_phys(out_buffer), size, source_vm,
				      ARRAY_SIZE(source_vm), dest_vm, dest_perm,
				      ARRAY_SIZE(dest_vm));
		if (ret)
			pr_err("hyp_assign_phys returned an error %d\n", ret);

		segments_iter->addr = virt_to_phys(out_buffer) / PAGE_SIZE;
		segments_iter->pages = 1ULL << page_order;
		segments_iter++;

		buffer_iter_remaining -= tocopy;
		if (buffer)
			buffer_iter += tocopy;
	}

	if (buffer_iter_remaining) {
		pr_err("Memory allocator is fragmented so we were not able to fit %d into segments header\n",
		       size);
		goto exit2;
	}

	dma_sync_single_for_device(dev, virt_to_phys(blob), PAGE_SIZE,
				   DMA_TO_DEVICE);

	return;

exit2:
	hypx_free_blob_userbuf(data);
exit1:
	return;
}

/* Returns PHY address for the allocated buffer */
static void hypx_create_blob_dmabuf(struct device *dev,
				    struct faceauth_data *data, int dmabuf_fd,
				    enum dma_data_direction dir,
				    bool is_secure_camera)
{
	struct scatterlist *sg;
	int i, ret = 0;

	int source_vm[] = { VMID_HLOS };
	int dest_vm[] = { VMID_CP_DSP_EXT, VMID_HLOS_FREE };
	int dest_perm[] = { PERM_READ | PERM_WRITE, PERM_READ | PERM_WRITE };

	/* If we deal with secure camera buffer then no assignment to DSP
	 * is needed
	 */
	bool perform_assignment_to_dsp = !is_secure_camera;

	data->direction = dir;
	data->need_reassign_to_hlos = perform_assignment_to_dsp;

	data->dma_buf = dma_buf_get(dmabuf_fd);
	if (IS_ERR(data->dma_buf)) {
		pr_err("dma_buf_get: %d\n", PTR_ERR(data->dma_buf));
		goto err1;
	}

	/* prepare dma_buf for DMA */
	data->attach = dma_buf_attach(data->dma_buf, dev);
	if (IS_ERR(data->attach)) {
		pr_err("dma_buf_attach: %d\n", PTR_ERR(data->attach));
		goto err2;
	}

	if (is_secure_camera)
		data->attach->dma_map_attrs |= DMA_ATTR_SKIP_CPU_SYNC;

	/* map to get the sg_table */
	data->sg_table = dma_buf_map_attachment(data->attach, dir);
	if (IS_ERR(data->sg_table)) {
		pr_err("dma_buf_map_attachment: %d\n", PTR_ERR(data->sg_table));
		goto err3;
	}

	if (!is_secure_camera)
		dma_sync_sg_for_device(dev, data->sg_table->sgl,
				       data->sg_table->nents, DMA_TO_DEVICE);

	/* struct hypx_blob struct have to be page aligned as we remap
	 * it to EL2 memory
	 */
	data->hypx_blob = (void *)get_zeroed_page(GFP_KERNEL);
	if (!data->hypx_blob) {
		pr_err("Cannot allocate memory for hypx data\n");
		goto err4;
	}
	if (WARN_ON(virt_to_phys(data->hypx_blob) % PAGE_SIZE)) {
		pr_err("data->hypx_blob is not PAGE aligned");
		goto err5;
	}

	if (data->sg_table->nents >= HYPX_MEMSEGS_NUM) {
		pr_err("the index of segments is greater than HYPX_MEMSEGS_NUM");
		goto err5;
	}
	for_each_sg(data->sg_table->sgl, sg, data->sg_table->nents, i) {
		WARN_ON(page_to_phys(sg_page(sg)) % PAGE_SIZE);
		data->hypx_blob->segments[i].addr =
			page_to_phys(sg_page(sg)) / PAGE_SIZE;
		WARN_ON(sg->length % PAGE_SIZE);
		data->hypx_blob->segments[i].pages = sg->length / PAGE_SIZE;
		if (!is_secure_camera)
			dma_sync_single_for_device(
				dev,
				(uint64_t)data->hypx_blob->segments[i].addr *
					PAGE_SIZE,
				data->hypx_blob->segments[i].pages * PAGE_SIZE,
				DMA_TO_DEVICE);
	}

	if (perform_assignment_to_dsp) {
		/*
		 * We need to re-assign only buffers that come from HLOS.
		 * Secure camera buffers are already assigned to EXT_DSP(RO)
		 */
		ret = hyp_assign_table(data->sg_table, source_vm,
				       ARRAY_SIZE(source_vm), dest_vm,
				       dest_perm, ARRAY_SIZE(dest_vm));
		if (ret) {
			pr_err("hyp_assign_table error: %d\n", ret);
			goto err5;
		}
	}

	dma_sync_single_for_device(dev, virt_to_phys(data->hypx_blob),
				   PAGE_SIZE, DMA_TO_DEVICE);
	return;

err5:
	free_page((unsigned long)data->hypx_blob);
	data->hypx_blob = NULL;
err4:
	dma_buf_unmap_attachment(data->attach, data->sg_table, data->direction);
	data->sg_table = NULL;
err3:
	dma_buf_detach(data->dma_buf, data->attach);
err2:
	dma_buf_put(data->dma_buf);
err1:
	return;
}

static void hypx_free_blob_dmabuf(struct device *dev,
				  struct faceauth_data *data)
{
	int source_vm[] = { VMID_CP_DSP_EXT, VMID_HLOS_FREE };
	int dest_vm[] = { VMID_HLOS };
	int dest_perm[] = { PERM_READ | PERM_WRITE | PERM_EXEC };
	int ret = 0;

	if (data->need_reassign_to_hlos) {
		ret = hyp_assign_table(data->sg_table, source_vm,
				       ARRAY_SIZE(source_vm), dest_vm,
				       dest_perm, ARRAY_SIZE(dest_vm));
		if (ret)
			pr_err("hyp_assign_table error: %d\n", ret);
	}

	dma_buf_unmap_attachment(data->attach, data->sg_table, data->direction);
	dma_buf_detach(data->dma_buf, data->attach);
	dma_buf_put(data->dma_buf);
	free_page((unsigned long)data->hypx_blob);

	data->hypx_blob = NULL;
}

static void hypx_free_blob(struct device *dev, struct faceauth_data *data)
{
	if (data->dma_buf)
		hypx_free_blob_dmabuf(dev, data);
	else
		hypx_free_blob_userbuf(data);
}

static int allocate_bounce_buffer(struct device *dev, void **vaddr,
				  dma_addr_t *paddr)
{
	int ret = 0;
	int source_vm[] = { VMID_HLOS };
	int dest_vm[] = { VMID_CP_DSP_EXT, VMID_HLOS_FREE };
	int dest_perm[] = { PERM_READ | PERM_WRITE, PERM_READ | PERM_WRITE };
	void *buffer_addr;
	dma_addr_t buffer_paddr;

	*vaddr = NULL;
	*paddr = 0;

	buffer_addr =
		dma_alloc_coherent(dev, PAGE_SIZE, &buffer_paddr, GFP_KERNEL);
	if (!buffer_addr) {
		ret = -ENOMEM;
		goto exit2;
	}

	ret = hyp_assign_phys(buffer_paddr, PAGE_SIZE, source_vm,
			      ARRAY_SIZE(source_vm), dest_vm, dest_perm,
			      ARRAY_SIZE(dest_vm));
	if (ret) {
		pr_err("hyp_assign_phys returned an error %d\n", ret);
		goto exit1;
	}

	*vaddr = buffer_addr;
	*paddr = buffer_paddr;
	return ret;

exit1:
	dma_free_coherent(dev, PAGE_SIZE, buffer_addr, buffer_paddr);
exit2:
	return ret;
}

static int deallocate_bounce_buffer(struct device *dev, void **vaddr,
				    dma_addr_t *paddr)
{
	int ret = 0;
	int source_vm[] = { VMID_CP_DSP_EXT, VMID_HLOS_FREE };
	int dest_vm[] = { VMID_HLOS };
	int dest_perm[] = { PERM_READ | PERM_WRITE | PERM_EXEC };

	if (*vaddr == NULL || *paddr == 0)
		return -EINVAL;

	ret = hyp_assign_phys(*paddr, PAGE_SIZE, source_vm,
			      ARRAY_SIZE(source_vm), dest_vm, dest_perm,
			      ARRAY_SIZE(dest_vm));
	if (ret) {
		pr_err("hyp_assign_phys returned an error %d\n", ret);
		return ret;
	}

	dma_free_coherent(dev, PAGE_SIZE, *vaddr, *paddr);
	*vaddr = NULL;
	*paddr = 0;

	return ret;
}

static void *bounce_buff;
static dma_addr_t bounce_buff_bus_addr;

int el2_faceauth_probe(struct device *dev)
{
	int ret = 0;
	struct scm_desc desc = { 0 };
	void *buffer_vaddr;
	dma_addr_t buffer_paddr;

	ret = allocate_bounce_buffer(dev, &buffer_vaddr, &buffer_paddr);
	if (ret) {
		pr_err("allocate_bounce_buffer returned an error %d\n", ret);
		goto exit2;
	}

	desc.arginfo = SCM_ARGS(2);
	desc.args[0] = (phys_addr_t)buffer_paddr;
	desc.args[1] = PAGE_SIZE;
	ret = scm_call2(HYPX_SMC_FUNC_DRIVER_PROBE, &desc);
	if (ret) {
		pr_err("HypX driver probe failed. scm_call %d\n", ret);
		goto exit1;
	}

	bounce_buff = buffer_vaddr;
	bounce_buff_bus_addr = buffer_paddr;
	return 0;

exit1:
	ret = deallocate_bounce_buffer(dev, &buffer_vaddr, &buffer_paddr);
	if (ret)
		pr_err("deallocate_bounce_buffer returned an error %d\n", ret);
exit2:
	return ret;
}

int el2_faceauth_remove(struct device *dev)
{
	int ret = 0;
	struct scm_desc desc = { 0 };

	desc.arginfo = SCM_ARGS(0);
	ret = scm_call2(HYPX_SMC_FUNC_DRIVER_REMOVE, &desc);
	if (ret)
		pr_err("HypX driver remove failed. scm_call %d\n", ret);

	ret = deallocate_bounce_buffer(dev, &bounce_buff,
				       &bounce_buff_bus_addr);
	if (ret)
		pr_err("deallocate_bounce_buffer returned an error %d\n", ret);

	return ret;
}

int el2_faceauth_wait_pil_dma_over(void)
{
	struct scm_desc check_dma_desc = { 0 };
	int ret = 0;
	unsigned long stop;

	stop = jiffies + msecs_to_jiffies(PIL_DMA_TIMEOUT);

	do {
		check_dma_desc.arginfo = SCM_ARGS(0);

		ret = scm_call2(HYPX_SMC_FUNC_CHECK_PIL_COMPLETION,
				&check_dma_desc);
		if (ret)
			return ret;

		ret = check_dma_desc.ret[0];
		if (ret == ERR_PIL_COMPLETE) {
			/* DMA completed successfully */
			return 0;
		} else if (ret == ERR_PIL_INCOMPLETE) {
			/* DMA is still running */
			usleep_range(1000, 2000);
		} else {
			/* EL2 call failed */
			return -EFAULT;
		}
		if (time_before(stop, jiffies)) {
			pr_err("PIL DMA timeout!\n");
			return -ETIME;
		}

	} while (true);
}

int el2_faceauth_init(struct device *dev, struct faceauth_init_data *data,
		      uint64_t verbosity_level)
{
	int ret = 0;
	int status_ret = 0;
	struct scm_desc desc = { 0 };
	struct hypx_fa_init *hypx_data;
	unsigned long save_trace;
	unsigned long stop;

	hypx_data = (void *)get_zeroed_page(GFP_KERNEL);
	if (!hypx_data) {
		ret = -ENOMEM;
		goto exit2;
	}

	hypx_data->verbosity_level = verbosity_level;
	hypx_data->features = data->features;

	dma_sync_single_for_device(dev, virt_to_phys(hypx_data), PAGE_SIZE,
				   DMA_TO_DEVICE);

	desc.args[0] = virt_to_phys(hypx_data);
	desc.arginfo = SCM_ARGS(1);
	save_trace = jiffies;
	ret = scm_call2(HYPX_SMC_FUNC_INIT, &desc);
	trace_faceauth_el2_duration(HYPX_SMC_FUNC_INIT & 0xFF,
				    jiffies_to_usecs(jiffies - save_trace));
	if (ret) {
		pr_err("Failed scm_call %d\n", ret);
		goto exit1;
	}
	ret = desc.ret[0];
	if (ret) {
		ret = parse_el2_return(ret);
		goto exit1;
	}

	stop = jiffies + msecs_to_jiffies(CONTEXT_SWITCH_TIMEOUT_MS);
	usleep_range(CONTEXT_SWITCH_TO_FACEAUTH_US,
		     CONTEXT_SWITCH_TO_FACEAUTH_US + 1);

	for (;;) {
		desc.arginfo = SCM_ARGS(0);
		ret = scm_call2(HYPX_SMC_FUNC_CHECK_FW_STATUS, &desc);
		if (ret) {
			pr_err("Failed scm_call %d\n", ret);
			goto exit1;
		}

		status_ret = desc.ret[0];
		if (status_ret == ERR_FW_READY)
			break;

		if (time_before(stop, jiffies)) {
			pr_err("el2: Faceauth FW context switch timeout!\n");
			ret = -ETIME;
			goto exit1;
		}
		usleep_range(100, 1000);
	}

exit1:
	free_page((unsigned long)hypx_data);
exit2:
	return ret;
}

int el2_faceauth_cleanup(struct device *dev)
{
	int ret = 0;
	struct scm_desc desc = { 0 };
	unsigned long save_trace;

	desc.arginfo = SCM_ARGS(0);
	save_trace = jiffies;
	ret = scm_call2(HYPX_SMC_FUNC_CLEANUP, &desc);
	if (ret)
		pr_err("Failed scm_call %d\n", ret);
	trace_faceauth_el2_duration(HYPX_SMC_FUNC_CLEANUP & 0xFF,
				    jiffies_to_usecs(jiffies - save_trace));
	ret = desc.ret[0];
	if (ret)
		ret = parse_el2_return(ret);
	/*
	 * Sleep for 20ms for AB to cleanup to reduce scm_call block.
	 * 20ms is a measured time that most of the time AB
	 * will finish wiping in this time. In this case we don't need
	 * extra waiting in EL2 under most of the cases.
	 */
	usleep_range(20000, 21000);

	return ret;
}

int el2_faceauth_process(struct device *dev, struct faceauth_start_data *data,
			 bool is_secure_camera)
{
	int ret = 0;
	struct scm_desc desc = { 0 };
	bool pass_images_to_el2;
	unsigned long save_trace = 0;
	struct hypx_fa_process *hypx_data;
	struct faceauth_data image_dot_left = { 0 }, image_dot_right = { 0 },
			     image_flood_left = { 0 },
			     image_flood_right = { 0 }, calibration = { 0 },
			     citadel_token = { 0 };

	pass_images_to_el2 = data->operation == COMMAND_ENROLL ||
			     data->operation == COMMAND_VALIDATE;

	hypx_data = (void *)get_zeroed_page(GFP_KERNEL);
	if (!hypx_data) {
		ret = -ENOMEM;
		pr_err("Cannot allocate memory for hypx_data\n");
		goto err1;
	}

	hypx_data->is_secure_camera = is_secure_camera;
	hypx_data->operation = data->operation;
	hypx_data->profile_id = data->profile_id;
	hypx_data->input_time_ms = data->input_time_ms;
	hypx_data->citadel_input = data->citadel_input;
	hypx_data->citadel_input2 = data->citadel_input2;
	if (pass_images_to_el2) {
		hypx_create_blob_dmabuf(dev, &image_dot_left,
					data->image_dot_left_fd, DMA_TO_DEVICE,
					is_secure_camera);
		if (!image_dot_left.hypx_blob)
			goto err2;
		hypx_data->image_dot_left =
			virt_to_phys(image_dot_left.hypx_blob);
		hypx_data->image_dot_left_size = data->image_dot_left_size;

		hypx_create_blob_dmabuf(dev, &image_dot_right,
					data->image_dot_right_fd, DMA_TO_DEVICE,
					is_secure_camera);
		if (!image_dot_right.hypx_blob)
			goto err2;
		hypx_data->image_dot_right =
			virt_to_phys(image_dot_right.hypx_blob);
		hypx_data->image_dot_right_size = data->image_dot_right_size;

		hypx_create_blob_dmabuf(dev, &image_flood_left,
					data->image_flood_fd, DMA_TO_DEVICE,
					is_secure_camera);
		if (!image_flood_left.hypx_blob)
			goto err2;
		hypx_data->image_flood_left =
			virt_to_phys(image_flood_left.hypx_blob);
		hypx_data->image_flood_left_size = data->image_flood_size;

		/* TODO: remove check once it's a required parameter */
		if (data->image_flood_right_fd) {
			hypx_create_blob_dmabuf(dev, &image_flood_right,
						data->image_flood_right_fd,
						DMA_TO_DEVICE,
						is_secure_camera);
			if (!image_flood_right.hypx_blob) {
				goto err2;
			} else {
				hypx_data->image_flood_right = virt_to_phys(
					image_flood_right.hypx_blob);
				hypx_data->image_flood_right_size =
					data->image_flood_right_size;
			}
		}

		hypx_create_blob_dmabuf(dev, &calibration, data->calibration_fd,
					DMA_TO_DEVICE, false);
		if (!calibration.hypx_blob)
			goto err2;
		hypx_data->calibration = virt_to_phys(calibration.hypx_blob);
		hypx_data->calibration_size = data->calibration_size;
	}

	hypx_data->citadel_token_size = 0;
	if (data->citadel_token_size &&
	    (data->operation == COMMAND_ENROLL_COMPLETE ||
	     data->operation == COMMAND_VALIDATE ||
	     data->operation == COMMAND_SET_FEATURE ||
	     data->operation == COMMAND_CLR_FEATURE ||
	     data->operation == COMMAND_RESET_LOCKOUT ||
	     data->operation == COMMAND_VERIFY_HAT)) {
		hypx_create_blob_userbuf(dev, &citadel_token,
					 data->citadel_token,
					 data->citadel_token_size);
		if (!citadel_token.hypx_blob)
			goto err2;
		hypx_data->citadel_token =
			virt_to_phys(citadel_token.hypx_blob);
		hypx_data->citadel_token_size = data->citadel_token_size;
	}

	if (data->operation == COMMAND_ENROLL_COMPLETE) {
		memcpy(hypx_data->cache_flush_indexes,
		       data->cache_flush_indexes,
		       sizeof(data->cache_flush_indexes));
	}

	dma_sync_single_for_device(dev, virt_to_phys(hypx_data), PAGE_SIZE,
				   DMA_TO_DEVICE);

	desc.args[0] = virt_to_phys(hypx_data);
	desc.arginfo = SCM_ARGS(1);
	save_trace = jiffies;
	ret = scm_call2(HYPX_SMC_FUNC_PROCESS, &desc);
	if (ret)
		pr_err("Failed scm_call %d\n", ret);

	ret = desc.ret[0];
	if (ret)
		ret = parse_el2_return(ret);

	trace_faceauth_el2_duration(HYPX_SMC_FUNC_PROCESS & 0xFF,
				    jiffies_to_usecs(jiffies - save_trace));

err2:
	if (hypx_data->citadel_token)
		hypx_free_blob_userbuf(&citadel_token);
	if (hypx_data->calibration)
		hypx_free_blob_dmabuf(dev, &calibration);
	if (hypx_data->image_flood_right)
		hypx_free_blob_dmabuf(dev, &image_flood_right);
	if (hypx_data->image_flood_left)
		hypx_free_blob_dmabuf(dev, &image_flood_left);
	if (hypx_data->image_dot_right)
		hypx_free_blob_dmabuf(dev, &image_dot_right);
	if (hypx_data->image_dot_left)
		hypx_free_blob_dmabuf(dev, &image_dot_left);

	free_page((unsigned long)hypx_data);

err1:
	return ret;
}

int el2_faceauth_get_process_result(struct device *dev,
				    struct faceauth_start_data *data)
{
	int ret = 0;
	unsigned long save_trace;
	struct scm_desc desc = { 0 };
	struct hypx_fa_process_results *hypx_data;
	struct faceauth_data citadel_token = { 0 }, deferred_autocal = { 0 };

	hypx_data = (void *)get_zeroed_page(GFP_KERNEL);
	if (!hypx_data) {
		ret = -ENOMEM;
		pr_err("Cannot allocate memory for hypx_data\n");
		goto exit1;
	}

	hypx_data->citadel_token_size = 0;
	if (data->citadel_token_size && data->operation == COMMAND_VALIDATE) {
		hypx_create_blob_userbuf(dev, &citadel_token,
					 data->citadel_token,
					 data->citadel_token_size);
		if (!citadel_token.hypx_blob)
			goto exit2;

		hypx_data->citadel_token =
			virt_to_phys(citadel_token.hypx_blob);
		hypx_data->citadel_token_size = data->citadel_token_size;
	}

	if (data->deferred_autocal_size) {
		/* The autocal buffer is for a small amount of data and
		 * we restrict its size to 4K
		 */
		if (data->deferred_autocal_size > PAGE_SIZE) {
			pr_err("deferred_autocal buffer is too big: %d\n",
			       data->deferred_autocal_size);
			ret = -EMSGSIZE;
			goto exit1;
		}
		hypx_create_blob_dmabuf(dev, &deferred_autocal,
					data->deferred_autocal_fd,
					DMA_FROM_DEVICE, true);
		if (!deferred_autocal.hypx_blob)
			goto exit1;
		hypx_data->deferred_autocal =
			virt_to_phys(deferred_autocal.hypx_blob);
		hypx_data->deferred_autocal_size = data->deferred_autocal_size;
	}

	dma_sync_single_for_device(dev, virt_to_phys(hypx_data), PAGE_SIZE,
				   DMA_TO_DEVICE);

	desc.arginfo = SCM_ARGS(1);
	desc.args[0] = virt_to_phys(hypx_data);
	save_trace = jiffies;
	ret = scm_call2(HYPX_SMC_FUNC_CHECK_PROCESS_RESULT, &desc);
	trace_faceauth_el2_duration(HYPX_SMC_FUNC_CHECK_PROCESS_RESULT & 0xFF,
				    jiffies_to_usecs(jiffies - save_trace));
	if (ret) {
		pr_err("Failed scm_call %d\n", ret);
		goto exit2;
	}

	ret = desc.ret[0];
	if (ret) {
		ret = parse_el2_return(ret);
		goto exit2;
	}

	dma_sync_single_for_cpu(dev, virt_to_phys(hypx_data), PAGE_SIZE,
				DMA_FROM_DEVICE);

	data->result = hypx_data->result;
	data->angles = hypx_data->angles;
	data->lockout_event = hypx_data->citadel_lockout_event;
	data->citadel_output1 = hypx_data->citadel_output1;
	data->citadel_output2 = hypx_data->citadel_output2;
	data->fw_version = hypx_data->fw_version;
	data->error_code = hypx_data->error_code;
	data->ab_exception_number = hypx_data->exception_number;
	memcpy(data->aux_data, hypx_data->aux_data,
	       sizeof(hypx_data->aux_data));


	if (hypx_data->citadel_token) {
		ret = hypx_copy_from_blob_userbuf(dev, &citadel_token,
						  data->citadel_token,
						  data->citadel_token_size,
						  true);
		if (ret) {
			pr_err("Failed hypx_copy_from_blob_userbuf %d\n", ret);
			goto exit2;
		}
	}

exit2:
	if (hypx_data->deferred_autocal)
		hypx_free_blob(dev, &deferred_autocal);
	if (hypx_data->citadel_token)
		hypx_free_blob_userbuf(&citadel_token);
	free_page((unsigned long)hypx_data);

exit1:
	return ret;
}

int el2_faceauth_gather_debug_log(struct device *dev,
				  struct faceauth_debug_data *data)
{
	int ret = 0;
	struct scm_desc desc = { 0 };
	struct hypx_fa_process_results *hypx_data;
	struct faceauth_data debug_buf = { 0 };

	hypx_data = (void *)get_zeroed_page(GFP_KERNEL);
	if (!hypx_data) {
		ret = -ENOMEM;
		pr_err("Cannot allocate memory for hypx_data\n");
		goto exit3;
	}

	if (data->buffer_fd)
		hypx_create_blob_dmabuf(dev, &debug_buf, data->buffer_fd,
					DMA_TO_DEVICE, false);
	else
		hypx_create_blob_userbuf(dev, &debug_buf, NULL,
					 data->debug_buffer_size);

	if (!debug_buf.hypx_blob) {
		pr_err("Cannot allocate memory for debug_buffer");
		goto exit2;
	}

	hypx_data->debug_buffer = virt_to_phys(debug_buf.hypx_blob);
	hypx_data->debug_buffer_size = data->debug_buffer_size;

	dma_sync_single_for_device(dev, virt_to_phys(hypx_data), PAGE_SIZE,
				   DMA_TO_DEVICE);

	desc.arginfo = SCM_ARGS(1);
	desc.args[0] = virt_to_phys(hypx_data);

	ret = scm_call2(HYPX_SMC_FUNC_GET_DEBUG_RESULT, &desc);
	if (ret) {
		pr_err("Failed scm_call %d\n", ret);
		goto exit1;
	}

	ret = desc.ret[0];
	if (ret) {
		ret = parse_el2_return(ret);
		goto exit1;
	}

	dma_sync_single_for_cpu(dev, virt_to_phys(hypx_data), PAGE_SIZE,
				DMA_FROM_DEVICE);

	ret = hypx_copy_from_blob_userbuf(dev, &debug_buf, data->debug_buffer,
					  data->debug_buffer_size, true);
	if (ret) {
		pr_err("Failed hypx_copy_from_blob_userbuf %d\n", ret);
		goto exit1;
	}

exit1:
	hypx_free_blob(dev, &debug_buf);
exit2:
	free_page((unsigned long)hypx_data);
exit3:
	return ret;
}

/*
 * NOT sure the exact size, using a more than necessary size
 */
#define AB_STATE_SIZE 262144 /* 2^18 */
#define IMAGE_SIZE (INPUT_IMAGE_WIDTH * INPUT_IMAGE_HEIGHT)

int el2_gather_debug_data(struct device *dev, void *destination_buffer,
			  uint32_t buffer_size)
{
	int err = 0;

	struct faceauth_debug_entry *debug_entry = destination_buffer;
	uint32_t current_offset;
	struct faceauth_buffer_list *output_buffers;
	int buffer_idx;
	int buffer_list_size;
	uint64_t ret;
	struct hypx_fa_debug_data *hypx_data;
	struct scm_desc desc = { 0 };
	struct faceauth_data image_dot_left = { 0 }, image_dot_right = { 0 },
			     image_flood_left = { 0 },
			     image_flood_right = { 0 }, calibration = { 0 },
			     ab_state = { 0 }, output_blob = { 0 };

	hypx_data = (void *)get_zeroed_page(GFP_KERNEL);
	if (!hypx_data) {
		err = -ENOMEM;
		pr_err("Cannot allocate memory for hypx_data\n");
		goto exit_hypx_data;
	}

	hypx_data->offset_int_state =
		offsetof(struct faceauth_airbrush_state, internal_state_size);
	hypx_data->offset_ab_state =
		offsetof(struct faceauth_debug_entry, ab_state);

	/*
	 * We are going to copy things into these blobs,
	 * so we don't care their content. Re-using debug
	 * entry to reduce alloc mem. This works base on
	 * the fact that debug_entry is larger than image
	 * size.
	 */
	hypx_create_blob_userbuf(dev, &image_dot_left, NULL, IMAGE_SIZE);
	if (!image_dot_left.hypx_blob)
		goto exit;
	hypx_data->image_left = virt_to_phys(image_dot_left.hypx_blob);
	hypx_data->image_left_size = IMAGE_SIZE;

	hypx_create_blob_userbuf(dev, &image_dot_right, NULL, IMAGE_SIZE);
	if (!image_dot_right.hypx_blob)
		goto exit;
	hypx_data->image_right = virt_to_phys(image_dot_right.hypx_blob);
	hypx_data->image_right_size = IMAGE_SIZE;

	hypx_create_blob_userbuf(dev, &image_flood_left, NULL, IMAGE_SIZE);
	if (!image_flood_left.hypx_blob)
		goto exit;
	hypx_data->image_flood_left = virt_to_phys(image_flood_left.hypx_blob);
	hypx_data->image_flood_left_size = IMAGE_SIZE;

	hypx_create_blob_userbuf(dev, &image_flood_right, NULL,
				 IMAGE_SIZE);
	if (!image_flood_right.hypx_blob)
		goto exit;
	hypx_data->image_flood_right =
	    virt_to_phys(image_flood_right.hypx_blob);
	hypx_data->image_flood_right_size = IMAGE_SIZE;

	hypx_create_blob_userbuf(dev, &calibration, NULL, CALIBRATION_SIZE);
	if (!calibration.hypx_blob)
		goto exit;
	hypx_data->calibration_buffer = virt_to_phys(calibration.hypx_blob);
	hypx_data->calibration_size = CALIBRATION_SIZE;

	hypx_create_blob_userbuf(dev, &ab_state, NULL, AB_STATE_SIZE);
	if (!ab_state.hypx_blob)
		goto exit;
	hypx_data->ab_state = virt_to_phys(ab_state.hypx_blob);

	dma_sync_single_for_device(dev, virt_to_phys(hypx_data), PAGE_SIZE,
				   DMA_TO_DEVICE);

	desc.args[0] = virt_to_phys(hypx_data);
	desc.arginfo = SCM_ARGS(1);

	err = scm_call2(HYPX_SMC_FUNC_GET_DEBUG_DATA, &desc);
	if (err) {
		pr_err("Failed scm_call %d\n", err);
		goto exit;
	}

	ret = desc.ret[0];
	if (ret) {
		err = parse_el2_return(ret);
		goto exit;
	}

	dma_sync_single_for_cpu(dev, virt_to_phys(hypx_data), PAGE_SIZE,
				DMA_FROM_DEVICE);

	err = hypx_copy_from_blob_userbuf(dev, &ab_state,
					  &debug_entry->ab_state,
					  hypx_data->internal_state_struct_size,
					  false);
	if (err) {
		pr_err("Failed hypx_copy_from_blob_userbuf internal_state %d\n",
		       err);
		goto exit;
	}

	debug_entry->ab_exception_number = hypx_data->exception_number;
	debug_entry->fault_address = hypx_data->fault_address;
	debug_entry->ab_link_reg = hypx_data->ab_link_reg;

	current_offset = offsetof(struct faceauth_debug_entry, ab_state) +
			 hypx_data->internal_state_struct_size;

	if (debug_entry->ab_state.command == COMMAND_ENROLL ||
	    debug_entry->ab_state.command == COMMAND_VALIDATE) {
		err = hypx_copy_from_blob_userbuf(
			dev, &image_dot_left,
			(uint8_t *)debug_entry + current_offset,
			hypx_data->image_left_size, false);

		debug_entry->left_dot.offset_to_image = current_offset;
		debug_entry->left_dot.image_size = IMAGE_SIZE;
		current_offset += IMAGE_SIZE;

		if (err) {
			pr_err("Error saving left dot image\n");
			goto exit;
		}

		err = hypx_copy_from_blob_userbuf(
			dev, &image_dot_right,
			(uint8_t *)debug_entry + current_offset,
			hypx_data->image_right_size, false);

		debug_entry->right_dot.offset_to_image = current_offset;
		debug_entry->right_dot.image_size = IMAGE_SIZE;
		current_offset += IMAGE_SIZE;
		if (err) {
			pr_err("Error saving right dot image\n");
			goto exit;
		}

		err = hypx_copy_from_blob_userbuf(
			dev, &image_flood_left,
			(uint8_t *)debug_entry + current_offset,
			hypx_data->image_flood_left_size, false);

		debug_entry->left_flood.offset_to_image = current_offset;
		debug_entry->left_flood.image_size = IMAGE_SIZE;
		current_offset += IMAGE_SIZE;
		if (err) {
			pr_err("Error saving left flood image\n");
			goto exit;
		}

		err = hypx_copy_from_blob_userbuf(
			dev, &image_flood_right,
			(uint8_t *)debug_entry + current_offset,
			hypx_data->image_flood_right_size, false);

		debug_entry->right_flood.offset_to_image = current_offset;
		debug_entry->right_flood.image_size = IMAGE_SIZE;
		current_offset += IMAGE_SIZE;
		if (err) {
			pr_err("Error saving right flood image\n");
			goto exit;
		}

		err = hypx_copy_from_blob_userbuf(
			dev, &calibration,
			(uint8_t *)debug_entry + current_offset,
			hypx_data->calibration_size, false);
		debug_entry->calibration.offset_to_image = current_offset;
		debug_entry->calibration.image_size = CALIBRATION_SIZE;
		current_offset += CALIBRATION_SIZE;
		if (err) {
			pr_err("Error saving calibration buffer\n");
			goto exit;
		}
	} else {
		debug_entry->left_dot.offset_to_image = 0;
		debug_entry->left_dot.image_size = 0;
		debug_entry->right_dot.offset_to_image = 0;
		debug_entry->right_dot.image_size = 0;
		debug_entry->left_flood.offset_to_image = 0;
		debug_entry->left_flood.image_size = 0;
		debug_entry->right_flood.offset_to_image = 0;
		debug_entry->right_flood.image_size = 0;
		debug_entry->calibration.offset_to_image = 0;
		debug_entry->calibration.image_size = 0;
	}

	output_buffers = &debug_entry->ab_state.output_buffers;
	if (!output_buffers) {
		err = -EMSGSIZE;
		pr_err("No output buffer\n");
		goto exit;
	}
	buffer_idx = output_buffers->buffer_count - 1;
	if (buffer_idx < 0) {
		pr_err("No available buffer");
		goto exit;
	}
	buffer_list_size =
		output_buffers->buffers[buffer_idx].offset_to_buffer +
		output_buffers->buffers[buffer_idx].size;

	if (buffer_list_size + current_offset > DEBUG_DATA_BIN_SIZE) {
		err = -EMSGSIZE;
		pr_err("Wrong output buffer size\n");
		goto exit;
	}

	if (output_buffers->buffer_base != 0 && buffer_list_size > 0) {
		hypx_create_blob_userbuf(dev, &output_blob, NULL,
					 buffer_list_size);
		if (!output_blob.hypx_blob)
			goto exit;
		hypx_data->output_buffers = virt_to_phys(output_blob.hypx_blob);
		hypx_data->buffer_list_size = buffer_list_size;
		hypx_data->buffer_base = output_buffers->buffer_base;

		dma_sync_single_for_device(dev, virt_to_phys(hypx_data),
					   PAGE_SIZE, DMA_TO_DEVICE);

		err = scm_call2(HYPX_SMC_FUNC_GET_DEBUG_BUFFER, &desc);
		if (err)
			pr_err("Failed scm_call %d\n", err);

		ret = desc.ret[0];
		if (ret) {
			err = parse_el2_return(ret);
			goto exit;
		}

		dma_sync_single_for_cpu(dev, virt_to_phys(hypx_data), PAGE_SIZE,
					DMA_FROM_DEVICE);

		err = hypx_copy_from_blob_userbuf(dev, &output_blob,
						  (uint8_t *)debug_entry +
							  current_offset,
						  buffer_list_size, false);

		output_buffers->buffer_base = current_offset;
		current_offset += buffer_list_size;
	}

exit:
	if (hypx_data->output_buffers)
		hypx_free_blob_userbuf(&output_blob);
	if (hypx_data->ab_state)
		hypx_free_blob_userbuf(&ab_state);
	if (hypx_data->calibration_buffer)
		hypx_free_blob_userbuf(&calibration);
	if (hypx_data->image_flood_right)
		hypx_free_blob_userbuf(&image_flood_right);
	if (hypx_data->image_flood_left)
		hypx_free_blob_userbuf(&image_flood_left);
	if (hypx_data->image_right)
		hypx_free_blob_userbuf(&image_dot_right);
	if (hypx_data->image_left)
		hypx_free_blob_userbuf(&image_dot_left);

	free_page((unsigned long)hypx_data);

exit_hypx_data:
	return err;
}
