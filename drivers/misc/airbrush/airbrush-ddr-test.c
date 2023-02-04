/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors: Shaik Ameer Basha(shaik.ameer@samsung.com)
 *
 * Airbrush DDR test code.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#include <linux/airbrush-sm-ctrl.h>
#include <linux/delay.h>
#include <linux/pci.h>

#include "airbrush-ddr.h"
#include "airbrush-ddr-internal.h"
#include "airbrush-pmic-ctrl.h"
#include "airbrush-regs.h"

#define DMA_TEST_USE_PATTERN_FOR_PCIE_DMA
#define DMA_SZ_MB	(8)
#define DMA_SZ		(DMA_SZ_MB * 1024 * 1024)
#define DMA_CHANNEL	(0)
#define AB_DDR_BASE	(0x20000000)
#define DMA_COMPLETION_TIMEOUT_MSEC (DMA_SZ_MB * 50)
#define DMA_TEST_MAX_MISMATCH_PRINTS (500)

static DECLARE_COMPLETION(dma_completion);

static int dma_callback(uint8_t chan, enum dma_data_direction dir,
			enum abc_dma_trans_status status)
{
	complete(&dma_completion);
	return 0;
}

static void __get_pcie_dma_size_info(unsigned int test_data,
			int *dma_size, int *num_transfers)
{
	int num_1mb_chunks;

	/* get the number of 1MB chunks to be used for pcie dma test.
	 * minimum size should be 1 chunk (i.e., 1MB) and maximum size should
	 * be 512 chunks (i.e., 512MB)
	 */
	num_1mb_chunks = DDR_TEST_PCIE_DMA_SIZE_MB(test_data);
	if (!num_1mb_chunks)
		num_1mb_chunks = 1;
	if (num_1mb_chunks > 512)
		num_1mb_chunks = 512;

	if (num_1mb_chunks >= DMA_SZ_MB) {
		*dma_size = DMA_SZ;
		/* not to complicate, rounding off to the nearest
		 * DMA_SZ_MB boundary
		 */
		*num_transfers = (num_1mb_chunks * 1024 * 1024) / DMA_SZ;
	} else {

		/* this scenario is mostly used in read/write eye measurement */
		*dma_size = num_1mb_chunks * 1024 * 1024;
		*num_transfers = 1;
	}
}

static int ddr_rw_test_pcie_dma_write(unsigned int test_data, char *host_vaddr,
			dma_addr_t host_paddr, int test_iter)
{
	int i, dma_size;
	struct dma_element_t desc;
	dma_addr_t ab_paddr = AB_DDR_BASE;
	static char dma_patterns[] = { 0xff, 0xa5, 0x5a, 0x00, 0xaa };
	uint32_t num_patterns = ARRAY_SIZE(dma_patterns);
	int num_dma_transfers;
	unsigned long timeout;
	static int num_timeouts;

	if (num_timeouts)
		pr_err("pcie dma write: Error!! write num_timeouts[%d]\n",
			num_timeouts);

	__get_pcie_dma_size_info(test_data, &dma_size, &num_dma_transfers);

#ifdef DMA_TEST_USE_PATTERN_FOR_PCIE_DMA
	for (i = 0; i < dma_size; i++)
		host_vaddr[i] = dma_patterns[(i + test_iter) % num_patterns];
#else
	for (i = 0; i < dma_size; i++)
		host_vaddr[i] = (i + test_iter) & 0xff;
#endif
	desc.len = dma_size;
	desc.chan = DMA_CHANNEL;
	desc.src_addr = (uint32_t)(host_paddr & 0xFFFFFFFF);
	desc.src_u_addr = (uint32_t)(host_paddr >> 32);

	for (i = 0; i < num_dma_transfers; i++) {
		desc.dst_addr = (uint32_t)(ab_paddr & 0xFFFFFFFF);
		desc.dst_u_addr = (uint32_t)(ab_paddr >> 32);

		reinit_completion(&dma_completion);

		(void)dma_sblk_start(DMA_CHANNEL, DMA_TO_DEVICE, &desc);

		timeout = msecs_to_jiffies(DMA_COMPLETION_TIMEOUT_MSEC);
		if (!wait_for_completion_timeout(&dma_completion, timeout)) {
			pr_err("%s: error!!! pcie dma interrupt timedout\n",
				__func__);
			WARN_ON(1);
			num_timeouts++;
			return DDR_FAIL;
		}

		ab_paddr = ab_paddr + dma_size;
	}

	return DDR_SUCCESS;
}

static int ddr_rw_test_pcie_dma_read_compare(unsigned int test_data,
			char *host_vaddr, dma_addr_t host_paddr, int test_iter)
{
	int i, j, fail_cnt = 0, dma_size;
	struct dma_element_t desc;
	dma_addr_t ab_paddr = AB_DDR_BASE;
	static char dma_patterns[] = { 0xff, 0xa5, 0x5a, 0x00, 0xaa };
	uint32_t num_patterns = ARRAY_SIZE(dma_patterns);
	int num_dma_transfers;
	unsigned long timeout;
	static int num_failures, num_timeouts;

	if (num_failures || num_timeouts)
		pr_err("[ddr rw test] Error!! read fails[%d] timeouts[%d]\n",
			num_failures, num_timeouts);

	__get_pcie_dma_size_info(test_data, &dma_size, &num_dma_transfers);

	desc.len = dma_size;
	desc.chan = DMA_CHANNEL;
	desc.dst_addr = (uint32_t)(host_paddr & 0xFFFFFFFF);
	desc.dst_u_addr = (uint32_t)(host_paddr >> 32);

	for (j = 0; j < num_dma_transfers; j++) {
		desc.src_addr = (uint32_t)(ab_paddr & 0xFFFFFFFF);
		desc.src_u_addr = (uint32_t)(ab_paddr >> 32);

		reinit_completion(&dma_completion);

		(void)dma_sblk_start(DMA_CHANNEL, DMA_FROM_DEVICE, &desc);

		timeout = msecs_to_jiffies(DMA_COMPLETION_TIMEOUT_MSEC);
		if (!wait_for_completion_timeout(&dma_completion, timeout)) {
			pr_err("%s: error!!! pcie dma interrupt timedout\n",
				__func__);
			WARN_ON(1);
			num_timeouts++;
			return DDR_FAIL;
		}

		for (i = 0; i < dma_size; i++) {

#ifdef DMA_TEST_USE_PATTERN_FOR_PCIE_DMA

			if (host_vaddr[i] !=
				dma_patterns[(i + test_iter) % num_patterns]) {
				fail_cnt++;

				if (DDR_TEST_NOPRINT & test_data)
					return DDR_FAIL;

				pr_err("mismatch for address 0x%x ",
					AB_DDR_BASE + (j * dma_size) + i);
				pr_err("cur: 0x%x, exp: 0x%x\n",
					host_vaddr[i],
					dma_patterns[(i + test_iter) %
						num_patterns]);

				/* Limit the error prints */
				if (fail_cnt > DMA_TEST_MAX_MISMATCH_PRINTS)
					break;
			}
#else
			if (host_vaddr[i] != ((i + test_iter) & 0xff)) {
				fail_cnt++;

				if (DDR_TEST_NOPRINT & test_data)
					return DDR_FAIL;

				pr_err("mismatch for address 0x%x ",
					AB_DDR_BASE + (j * dma_size) + i);
				pr_err("cur: 0x%x, exp: 0x%x\n",
						host_vaddr[i],
						((i + test_iter) & 0xff));
			}
#endif
		}

		ab_paddr = ab_paddr + dma_size;
	}

	if (DDR_TEST_NOPRINT & test_data)
		return DDR_SUCCESS;

	if (fail_cnt) {
		pr_err("ddr pcie dma r/w test fail. mismatch count :0x%x\n",
				fail_cnt);
		num_failures++;
		return DDR_FAIL;
	}

	pr_info("ddr pcie dma read/write test is success\n");
	return DDR_SUCCESS;
}

static int ddr_rw_test_pcie_dma(struct ab_ddr_context *ddr_ctx,
				unsigned int test_data)
{
	char *host_vaddr;
	dma_addr_t host_paddr;
	int ret = DDR_SUCCESS;
	static int test_iter;

	host_vaddr = abc_alloc_coherent(DMA_SZ, &host_paddr);
	if (!host_vaddr) {
		pr_err("%s: Error!!! unable to allocate memory for dma test\n",
			__func__);
		return DDR_FAIL;
	}

	(void)abc_reg_dma_irq_callback(&dma_callback, DMA_CHANNEL);

	if (DDR_BOOT_TEST_WRITE & test_data) {
		/* Change the DMA write pattern every time */
		test_iter++;

		/* for time tracking get the write start time info */
		ddr_ctx->st_write = ktime_get_boottime();

		ret = ddr_rw_test_pcie_dma_write(test_data,
				host_vaddr, host_paddr, test_iter);

		/* for time tracking get the write end time info */
		ddr_ctx->et_write = ktime_get_boottime();
	}

	if (DDR_BOOT_TEST_READ & test_data) {
		/* for time tracking get the read start time info */
		ddr_ctx->st_read = ktime_get_boottime();

		ret |= ddr_rw_test_pcie_dma_read_compare(test_data,
				host_vaddr, host_paddr, test_iter);

		/* for time tracking get the read end time info */
		ddr_ctx->et_read = ktime_get_boottime();
	}

	(void)abc_reg_dma_irq_callback(NULL, DMA_CHANNEL);
	abc_free_coherent(DMA_SZ, host_vaddr, host_paddr);

	return ret;
}

static int ddr_rw_test_memtester(struct ab_ddr_context *ddr_ctx,
				 unsigned int test_data)
{
	int i, data, fail_cnt = 0;
	uint32_t ddr_addr[] = {
		0x20000000, 0x20000004, 0x20008004, 0x20008008,
		0x20800008, 0x2080000C, 0x28000000, 0x28000004,
		0x28008004, 0x28008008, 0x28800008, 0x2880000C,
		0x30000000, 0x30000004, 0x30008004, 0x30008008,
		0x30800008, 0x3080000C, 0x38000000, 0x38000004,
		0x38008004, 0x38008008, 0x38800008, 0x3880000C,
		0x3ffffff8, 0x3ffffffc };

	uint32_t ddr_data[] = {
		0x12345678, 0xabcdef89, 0xa5a5a5a5, 0x12345678,
		0xabcdef89, 0xa5a5a5a5, 0x12345678, 0xabcdef89,
		0xa5a5a5a5, 0x12345678, 0xabcdef89, 0xa5a5a5a5,
		0xa5a5a5a5, 0x12345678, 0xabcdef89, 0xa5a5a5a5,
		0xabcdef89, 0xa5a5a5a5, 0x12345678, 0xabcdef89,
		0xa5a5a5a5, 0x12345678, 0xabcdef89, 0xa5a5a5a5,
		0xa5a5a5a5, 0x12345678 };

	uint32_t num_patterns = ARRAY_SIZE(ddr_data);

	if (DDR_BOOT_TEST_WRITE & test_data) {
		/* for time tracking get the write start time info */
		ddr_ctx->st_write = ktime_get_boottime();

		for (i = 0; i < num_patterns; i++)
			ddr_mem_wr(ddr_addr[i], ddr_data[i]);

		/* for time tracking get the write end time info */
		ddr_ctx->et_write = ktime_get_boottime();
	}

	if (DDR_BOOT_TEST_READ & test_data) {
		/* for time tracking get the read start time info */
		ddr_ctx->st_read = ktime_get_boottime();

		for (i = 0; i < num_patterns; i++) {
			data = ddr_mem_rd(ddr_addr[i]);
			if (ddr_data[i] == data) {
				pr_debug("0x%x: 0x%x\n", ddr_addr[i], data);
				continue;
			}

			fail_cnt++;
			if (DDR_TEST_NOPRINT & test_data) {
				/* get the read end time info */
				ddr_ctx->et_read = ktime_get_boottime();
				return DDR_FAIL;
			}

			pr_err("Mismatch!! 0x%x: CUR:0x%x, EXP:0x%x\n",
					ddr_addr[i], data, ddr_data[i]);
		}

		/* for time tracking get the read end time info */
		ddr_ctx->et_read = ktime_get_boottime();
	} else {
		return DDR_SUCCESS;
	}

	if (DDR_TEST_NOPRINT & test_data)
		return DDR_SUCCESS;

	if (fail_cnt) {
		pr_err("error!! ddr read/write test is fail\n");
		return DDR_FAIL;
	}

	pr_info("ddr read/write test is success\n");
	return DDR_SUCCESS;
}

/* Caller must hold ddr_ctx->ddr_lock */
int __ab_ddr_read_write_test(void *ctx, unsigned int test_data)
{
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;
	int ret = DDR_FAIL;

	/* ddr_mem_rd/wr based test to check the ddr data integrity */
	if (test_data & DDR_TEST_MEMTESTER)
		ret = ddr_rw_test_memtester(ddr_ctx, test_data);
	else if (test_data & DDR_TEST_PCIE_DMA) /* PCIe DMA read/write */
		ret = ddr_rw_test_pcie_dma(ddr_ctx, test_data);
	else
		pr_err("error!! undefined ddr r/w test\n");

	return ret;
}

int ab_ddr_read_write_test(void *ctx, unsigned int test_data)
{
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;
	int ret = DDR_FAIL;

	if (!ddr_ctx->is_setup_done) {
		pr_err("ddr rw test: error!! ddr setup is not called\n");
		return -EAGAIN;
	}

	if (ddr_ctx->ddr_state != DDR_ON) {
		pr_err("ddr_read_write_test: Invalid ddr state: %d\n",
			ddr_ctx->ddr_state);
		return DDR_FAIL;
	}

	/* Make sure to write the pattern before reading it back when the
	 * previous ddr state is DDR_OFF
	 */
	if (ddr_ctx->prev_ddr_state == DDR_OFF)
		test_data |= DDR_BOOT_TEST_WRITE;

	mutex_lock(&ddr_ctx->ddr_lock);
	ret = __ab_ddr_read_write_test(ctx, test_data);
	mutex_unlock(&ddr_ctx->ddr_lock);

	return ret;
}
